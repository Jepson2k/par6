//! A disk writer has an unbounded I/O deadline and must not share the RT thread.
use par6_rt::{
    diagnostics::{CaptureReader, CaptureSample},
    ArmState,
};
use std::{
    fs::OpenOptions,
    io::{self, BufWriter, Write},
    num::NonZeroU64,
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::JoinHandle,
    time::Duration,
};

pub(crate) const DEFAULT_MAX_SAMPLES: NonZeroU64 = NonZeroU64::new(900_000).unwrap();

pub(crate) fn spawn(
    path: &Path,
    mut reader: CaptureReader,
    fingerprint: &str,
    dt: f64,
    shutdown: Arc<AtomicBool>,
    max_samples: NonZeroU64,
) -> io::Result<(JoinHandle<()>, par6_proto::CaptureIdentity)> {
    assert_eq!(fingerprint.len(), 64);
    let mut options = OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let mut out = BufWriter::new(options.open(path)?);
    out.write_all(b"PAR6CAP2")?;
    out.write_all(&dt.to_le_bytes())?;
    out.write_all(fingerprint.as_bytes())?;
    let pid = u64::from(std::process::id());
    out.write_all(&pid.to_le_bytes())?;
    let started = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(io::Error::other)?
        .as_secs_f64();
    out.write_all(&started.to_le_bytes())?;
    out.flush()?;
    let identity = par6_proto::CaptureIdentity {
        pid,
        started,
        dt,
        fingerprint: fingerprint.to_owned(),
    };
    let thread = std::thread::Builder::new()
        .name("par6d-capture".into())
        .spawn(move || {
            // Capture duration can exceed an hour without making disk use unbounded.
            let result = (|| -> io::Result<()> {
                let mut count = 0u64;
                loop {
                    if let Some(s) = reader.pop() {
                        write_sample(&mut out, &s)?;
                        count += 1;
                        if count.is_multiple_of(25) {
                            out.flush()?;
                        }
                        if count >= max_samples.get() {
                            log::info!("diagnostic capture reached its {count}-sample budget");
                            break;
                        }
                    } else if shutdown.load(Ordering::Acquire) {
                        break;
                    } else {
                        std::thread::sleep(Duration::from_millis(1));
                    }
                }
                out.flush()
            })();
            if let Err(e) = result {
                log::error!("diagnostic capture failed: {e}");
            }
        })?;
    Ok((thread, identity))
}

fn write_sample(w: &mut impl Write, frame: &CaptureSample) -> io::Result<()> {
    let s = &frame.state;
    w.write_all(&s.tick.to_le_bytes())?;
    w.write_all(&frame.elapsed_ns.to_le_bytes())?;
    let stale = s.node_freshness[..6]
        .iter()
        .any(|v| *v != par6_bus::Freshness::Fresh);
    let flags = u64::from(s.error_active)
        | (u64::from(s.homed) << 1)
        | (u64::from(s.state == ArmState::Enabled) << 2)
        | (u64::from(stale) << 3)
        | (u64::from(s.gravity_comp) << 4)
        | ((s.mode as u64) << 8);
    w.write_all(&flags.to_le_bytes())?;
    for row in [
        &s.q,
        &s.qd,
        &s.tau,
        &s.q_commanded,
        &s.qd_commanded,
        &s.tau_commanded,
        &s.q_target,
        &s.gravity_torque_nm,
    ] {
        for x in row {
            w.write_all(&x.to_le_bytes())?;
        }
    }
    // StateSnapshot already maps physical CAN node IDs into joint order.
    for node in &s.nodes[..6] {
        w.write_all(&node.current_ma.map_or(f64::NAN, f64::from).to_le_bytes())?;
    }
    for node in &s.nodes[..6] {
        w.write_all(&node.kt_nm_a.map_or(f64::NAN, f64::from).to_le_bytes())?;
    }
    for x in &s.homing.effective_current_limit_ma[..6] {
        w.write_all(&f64::from(*x).to_le_bytes())?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use par6_rt::{diagnostics::capture_channel, StateSnapshot};
    use std::io::{Read, Seek, SeekFrom};

    #[test]
    fn recording_can_continue_beyond_one_hour_at_250_hz() {
        let limit = 900_001_u64;
        let path = std::env::temp_dir().join(format!(
            "par6-capture-budget-{}-{}.bin",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let (mut writer, reader) = capture_channel(2048);
        let shutdown = Arc::new(AtomicBool::new(false));
        let (thread, _) = spawn(
            &path,
            reader,
            &"0".repeat(64),
            0.004,
            shutdown.clone(),
            NonZeroU64::new(limit).unwrap(),
        )
        .unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(60);
        let mut sent = 0;
        'feed: for tick in 1..=limit {
            let sample = StateSnapshot {
                tick,
                ..StateSnapshot::default()
            };
            while !writer.push(&sample) {
                if thread.is_finished() || std::time::Instant::now() >= deadline {
                    break 'feed;
                }
                std::thread::yield_now();
            }
            sent = tick;
        }
        shutdown.store(true, Ordering::Release);
        thread.join().unwrap();
        let mut file = std::fs::File::open(&path).unwrap();
        let size = file.metadata().unwrap().len();
        file.seek(SeekFrom::End(-552)).unwrap();
        let mut last_tick = [0; 8];
        file.read_exact(&mut last_tick).unwrap();
        std::fs::remove_file(path).unwrap();
        assert_eq!(
            sent, limit,
            "writer ended before accepting the requested capture"
        );
        assert_eq!(size, 96 + limit * 552, "capture was silently truncated");
        assert_eq!(u64::from_le_bytes(last_tick), limit);
    }
}
