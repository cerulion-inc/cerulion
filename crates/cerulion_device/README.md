# cerulion_device

GPU residence for [Cerulion](https://github.com/cerulion-inc/cerulion)
message payloads. A payload is resident when a GPU can address it where it
already lies, so publishing to a GPU consumer copies nothing.

One `DeviceBackend` trait and its first backend:

| Backend | Library (opened at run time) | Status |
|---|---|---|
| `HostShm` | none | always available; runs the conformance suite on every machine |

No vendor library is linked, so a build needs no vendor SDK. A machine
without a backend's library gets an error value from that backend's `probe`.

`SegmentRegistry` registers each shared-memory mapping with the device once
and then translates slot addresses to device addresses without allocating.

## Tests

`tests/conformance_test.rs` runs one suite against every backend that probes
on the machine. HostShm always runs. To prove a backend ran rather than
skipped, name it:

```bash
CERULION_DEVICE_REQUIRE=host_shm cargo test -p cerulion_device --test conformance_test
```

## License

AGPL-3.0-only.
