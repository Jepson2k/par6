//! Blocking wrapper: a private tokio runtime driving the async
//! [`Client`], mirroring the reference sync client's `_run(inner…)`
//! delegation. Usage: `sync.block_on(sync.client().angles())`.

use tokio::runtime::Runtime;

use crate::core::{Client, ClientConfig};
use crate::error::ClientError;

/// A blocking par6 client for synchronous programs.
pub struct SyncClient {
    rt: Runtime,
    client: Client,
}

impl SyncClient {
    /// Connect with the given configuration.
    pub fn connect(cfg: ClientConfig) -> Result<Self, ClientError> {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()?;
        let client = rt.block_on(Client::connect(cfg))?;
        Ok(Self { rt, client })
    }

    /// Connect with [`ClientConfig::default`] (environment-driven).
    pub fn connect_default() -> Result<Self, ClientError> {
        Self::connect(ClientConfig::default())
    }

    /// The async client every call delegates to.
    pub fn client(&self) -> &Client {
        &self.client
    }

    /// Run one async client call to completion on the private runtime.
    pub fn block_on<F: std::future::Future>(&self, fut: F) -> F::Output {
        self.rt.block_on(fut)
    }

    /// Stop the listeners and wake every waiter.
    pub fn close(&self) {
        self.client.close();
    }
}

impl Drop for SyncClient {
    fn drop(&mut self) {
        self.client.close();
    }
}
