//! Loss-only UDP transport tests: the sink never acknowledges or executes commands.

use std::future::{poll_fn, Future};
use std::io::ErrorKind;
use std::pin::Pin;
use std::task::Poll;

use par6_proto::command as cmd;

use super::*;

async fn silent_sink() -> (Client, UdpSocket) {
    let sink = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let client = Client::connect(ClientConfig {
        host: "127.0.0.1".into(),
        port: sink.local_addr().unwrap().port(),
        timeout: Duration::from_secs(60),
        retries: 1,
        status: StatusTransport::Unicast {
            host: Ipv4Addr::LOCALHOST,
        },
        status_port: 0,
        mtu: 1400,
    })
    .await
    .unwrap();
    (client, sink)
}

fn move_command(client: &Client) -> Command {
    Command::MoveJ(cmd::MoveJ {
        key: client.fresh_key(),
        angles: [0.0; 6],
        duration: None,
        speed: Some(0.2),
        accel: Some(1.0),
        blend_radius: Some(0.0),
        rel: false,
    })
}

async fn receive_request<F>(mut request: Pin<&mut F>, sink: &UdpSocket) -> Vec<u8>
where
    F: Future<Output = Result<Option<u64>, ClientError>>,
{
    let mut bytes = [0; 2048];
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Poll::Ready(result) = poll_once(request.as_mut()).await {
            panic!("Unacknowledged request ended before send: {result:?}");
        }
        match sink.try_recv(&mut bytes) {
            Ok(n) => {
                let (_, command) = par6_proto::decode_command(&bytes[..n]).unwrap();
                assert!(matches!(command, Command::MoveJ(_)));
                return bytes[..n].to_vec();
            }
            Err(error) if error.kind() == ErrorKind::WouldBlock => {
                assert!(Instant::now() < deadline, "No command datagram arrived");
                // Keep the virtual clock fixed while the real I/O driver
                // handles readiness; only the test advances retry timers.
                tokio::task::yield_now().await;
            }
            Err(error) => panic!("UDP receive failed: {error}"),
        }
    }
}

async fn poll_once<F: Future>(mut future: Pin<&mut F>) -> Poll<F::Output> {
    poll_fn(|cx| Poll::Ready(future.as_mut().poll(cx))).await
}

async fn advance_past_timer(duration: Duration) {
    // Tokio rounds deadlines up to milliseconds; advance() alone need not
    // process expired timers before the next poll. This registered barrier
    // proves the driver processed the deadline while all time stays virtual.
    let barrier = tokio::time::sleep(duration);
    tokio::pin!(barrier);
    assert!(poll_once(barrier.as_mut()).await.is_pending());
    tokio::time::advance(duration + Duration::from_millis(2)).await;
    barrier.await;
}

fn assert_no_datagram(sink: &UdpSocket) {
    let mut bytes = [0; 2048];
    match sink.try_recv(&mut bytes) {
        Err(error) if error.kind() == ErrorKind::WouldBlock => {}
        result => panic!("Closed or dropped request sent another datagram: {result:?}"),
    }
}

#[tokio::test]
async fn close_during_backoff_joins_the_request_without_retrying() {
    let (client, sink) = silent_sink().await;
    let mut request = Box::pin(client.queued(move_command(&client)));
    receive_request(request.as_mut(), &sink).await;
    // Only the transport timers are virtual. The command above crossed a real
    // socket, and no synthetic response advances the request state machine.
    tokio::time::pause();
    advance_past_timer(client.config().timeout).await;
    assert!(poll_once(request.as_mut()).await.is_pending());
    assert_eq!(client.inner.pending.lock().unwrap().len(), 1);

    let late = client.queued(move_command(&client));
    let before_close = tokio::time::Instant::now();
    let ((), result) = tokio::join!(client.close_joined(), request);
    assert!(matches!(result, Err(ClientError::Closed)));
    assert_eq!(tokio::time::Instant::now(), before_close);
    assert!(client.inner.pending.lock().unwrap().is_empty());
    assert_no_datagram(&sink);

    // A future created before close but first polled afterward is closed too.
    client.close_joined().await;
    assert!(matches!(late.await, Err(ClientError::Closed)));
    assert_no_datagram(&sink);
}

#[tokio::test]
async fn dropping_a_request_removes_pending_registration_in_every_wait() {
    let (client, sink) = silent_sink().await;
    for in_backoff in [false, true] {
        let mut request = Box::pin(client.queued(move_command(&client)));
        receive_request(request.as_mut(), &sink).await;
        tokio::time::pause();
        if in_backoff {
            advance_past_timer(client.config().timeout).await;
            assert!(poll_once(request.as_mut()).await.is_pending());
        }
        assert_eq!(client.inner.pending.lock().unwrap().len(), 1);
        drop(request);
        assert!(
            client.inner.pending.lock().unwrap().is_empty(),
            "Dropping an unacknowledged request retained its pending entry"
        );
        tokio::time::advance(client.config().timeout * 3).await;
        assert_no_datagram(&sink);
        tokio::time::resume();
    }
    client.close_joined().await;
}

#[tokio::test]
async fn an_open_request_still_retries_the_identical_idempotent_datagram() {
    let (client, sink) = silent_sink().await;
    let mut request = Box::pin(client.queued(move_command(&client)));
    let first = receive_request(request.as_mut(), &sink).await;
    tokio::time::pause();
    advance_past_timer(client.config().timeout).await;
    assert!(poll_once(request.as_mut()).await.is_pending());
    // First retry delay is 50 ms plus at most 49 ms key-derived jitter.
    advance_past_timer(Duration::from_millis(100)).await;
    let retry = receive_request(request.as_mut(), &sink).await;
    assert_eq!(retry, first);
    advance_past_timer(client.config().timeout).await;
    assert!(matches!(request.await, Ok(None)));
    assert!(client.inner.pending.lock().unwrap().is_empty());
    assert_no_datagram(&sink);
    client.close_joined().await;
}
