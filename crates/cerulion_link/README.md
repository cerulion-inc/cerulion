# cerulion_link

A thin wrapper over [iroh](https://docs.rs/iroh) that gives
[Cerulion](https://github.com/cerulion-inc/cerulion) dial-by-key QUIC
connections: an endpoint's identity is its device key, so peers connect by key
alone. Cerulion uses zenoh on the local network and this crate across the
internet, where it carries raw Cerulion wire frames and the operations protocol.
It never touches zenoh and does not depend on `cerulion_core`.

```rust,no_run
use cerulion_link::{accept_one, alpn, build_endpoint, EndpointConfig, LinkError, RelayConfig};

async fn serve(device_secret: [u8; 32]) -> Result<(), LinkError> {
    let endpoint = build_endpoint(
        EndpointConfig::new(device_secret).with_relay(RelayConfig::N0Default),
    )
    .await?;

    while let Some(accepted) = accept_one(&endpoint).await? {
        if accepted.alpn == alpn::WIRE {
            // read and write frames on accepted.connection
        }
    }
    Ok(())
}
```

The crate also provides `dial`, length-prefixed frame streams
(`open_frame_stream`, `read_frame`, `write_frame`), one-way streams for
per-topic data, a synchronous `Read + Write` adapter over a QUIC stream
(`QuicOpsStream`), and relay configuration (`RelayConfig`).

## Who uses it

The `cerulion connect` worker, the `cerulion-netd` gateway daemon and the
robot-side remote-access daemon all build on it.

See [remote access](https://github.com/cerulion-inc/cerulion/blob/main/docs/remote_plane.md).
Design notes for contributors live in
[docs/internals/remote-access.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/remote-access.md).

## License

Licensed under either of the
[MIT license](https://github.com/cerulion-inc/cerulion/blob/main/docs/legal/LICENSE-MIT)
or the
[Apache License, Version 2.0](https://github.com/cerulion-inc/cerulion/blob/main/docs/legal/LICENSE-APACHE),
at your option. Most of Cerulion is AGPL-3.0-only; this crate is permissive on
purpose, so that closed-source clients and vendor firmware can link it.
