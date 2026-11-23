# Provenance verify

`cerulion rcp bench --verify` hashes every file of the measured code and checks it against the provenance
ledger. It exits 0 only when both manifests have their snapshot ids and every file matches its ledger row.
It installs nothing and needs only `python3`.

## Sub-features

- `verify-ok` prints both snapshot ids and `verify: ok` on an untouched folder.
- `verify-fail` exits 1 and names each file that differs from the ledger.

## How to get to it (user POV)

- `cerulion rcp bench --verify`.
- A checkout runs the same program with `bash harness/rcp.sh verify`.

## Driving it with hv.py

Preconditions:

- None. No install and no server.

- **Untouched.** Run `bash harness/rcp.sh verify`. Exit 0, the lines
  `plain: snapshot sha256:6f040ca7...` and `rcp: snapshot sha256:4f8bbe79...`, then
  `verify: ok, this folder holds the code the benchmark measured`.
- **Changed copy.** Copy the folder without its install and change one measured file in the copy:
  `C=$(mktemp -d)`, `rsync -a --exclude bx --exclude venv --exclude out harness/ "$C/"`, then
  `echo "# changed" >> "$C/measured/rcp/rcpx/perception.py"` and `bash "$C/rcp.sh" verify`. Exit 1 with
  `PROBLEM: plain: rcpx/perception.py differs from the ledger` and `verify: 2 problem(s)`. Remove the copy.
- **Fence.** `$HV doctor` prints `PASS  measured/ and provenance/ untouched` in the checkout itself.

## Gotchas

- Never run the changed-file step in the checkout. It edits the measured code.
- One file counts twice when both trees ship it (the plain and the harness setups share most files).
