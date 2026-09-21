# cerulion_pairing

Offline-verifiable pairing identity for
[Cerulion](https://github.com/cerulion-inc/cerulion) robots.

This crate holds formats, cryptography and state machines only. It performs no
network I/O and does not depend on iroh: the already authenticated peer key is
passed in, so the verifier embeds in robot firmware and the client library
embeds in a desktop app.

| Module | Role |
|---|---|
| `format` | Certificate, grant and epoch types with canonical signing bytes |
| `verify` | Robot-side chain verifier, access list and tamper-evident trust store |
| `pake` | The CPace fallback ceremony (bounded attempts, short lifetime, single session) |
| `pop` | Proof of possession: sign and verify a device-key challenge |
| `client` | Device key handling, certificate carry and the pairing driver |

A robot trusts a set of root keys, verifies a certificate chain without calling
any server, and keeps a local access list, so an established pairing keeps
working offline.

## Who uses it

The `cerulion pair` and `cerulion connect` verbs, the `cerulion-netd`
gateway daemon and the robot-side remote-access daemon all build on it.

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
