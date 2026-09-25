# cerulion_py

Raw, zero-copy Python client for the Cerulion transport (PyO3 wheel,
standalone cargo workspace - NOT a member of the root one).

```bash
pip install maturin
cd cerulion_py && maturin develop --release
```

```python
import cerulion

session = cerulion.connect()
pub = session.publisher("/topic", schema_hash=1, max_payload_len=1 << 20)
sub = session.subscriber("/topic", depth=4)

pub.publish(b"hello", timestamp_ns=42)   # single-copy publish
frame = sub.receive(1000)                # zero-copy: frame views shared memory
print(frame.to_bytes())                  # materialised (copies)
frame.release()
```

Docs: `../../docs/python.md`. Tests: `python -m pytest tests -q` from a
non-source cwd with `CERULION_PY_FIXTURE=target/release/cerulion_py_fixture`.
