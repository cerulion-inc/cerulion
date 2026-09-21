# License History

This file records the licensing history of Cerulion.
## Cutover: MIT OR Apache-2.0 → AGPL-3.0-only

- **Date:** 2026-06-28
- **Change:** The entire Cerulion workspace is relicensed from
  `MIT OR Apache-2.0` to **`AGPL-3.0-only`** (GNU Affero General Public License,
  version 3.0 only; no "or any later version" grant).
- **Base commit (last commit distributed under `MIT OR Apache-2.0`):**
  `52a2e8bab9fbde2d4f9d85866446adfe7451fdad`
- **New license text:** `LICENSE` (verbatim AGPL-3.0 from
  <https://www.gnu.org/licenses/agpl-3.0.txt>).
- **Retained for provenance (NOT removed):** `LICENSE-MIT` and `LICENSE-APACHE`
  remain in the tree under `docs/legal/` to document the prior terms. They no longer describe the
  license of new releases.
- **Commercial license:** AGPL-3.0-only is paired with commercial
  licensing (available on request; `docs/legal/COMMERCIAL.md` describes how to obtain it)
  and a **Contributor License Agreement (CLA)** (`.github/CLA`).

## What this cutover does and does not do

- **Going forward**, releases from the relicensing commit onward are governed by
  AGPL-3.0-only.
- **It cannot retroactively revoke already-granted permissive rights.** Every
  commit up to and including the base commit above was made available under
  `MIT OR Apache-2.0`. Anyone who obtained that code under those terms keeps the
  MIT/Apache-2.0 rights they were granted *for that code*: a permissive license
  already granted on distributed code is irrevocable. Downstreams may continue to
  fork the pre-cutover tree under MIT/Apache-2.0.

## Mechanics of this change

- Root `Cargo.toml` `[workspace.package].license` → `AGPL-3.0-only`
  (inherited by every crate via `license.workspace = true`).
- `crates/rmw_cerulion/Cargo.toml` explicit `license` → `AGPL-3.0-only`.
- `// SPDX-License-Identifier: AGPL-3.0-only` prepended as the first line of
  every first-party Rust source file (a one-time sweep). **Exclusion:** the
  `tests/ui/` trybuild compile-fail fixtures
  are intentionally skipped: their `.stderr` snapshots pin exact line:col, so a
  prepended header would shift every diagnostic and break the oracle. They are
  test inputs, not distributed library source; revisit if counsel wants headers
  there too (it requires re-blessing every `.stderr`).
  **Exclusion:** `crates/test_fixtures/test_node_raw_ffi_template_cdylib/src/lib.rs` is
  also skipped: it is a byte-for-byte frozen oracle of `generate_lib_rs()`
  output (pinned by `templates.rs::test_generate_lib_rs_matches_oracle_fixture`).
  The generator emits no SPDX header, so a prepended line breaks the byte-equality
  oracle and fails the test suite. Same rationale as `tests/ui/`: test input, not
  distributed library source; revisit if counsel wants the header (it requires
  regenerating the fixture via `cargo run -p cerulion_cli_engine --example
  dump_raw_ffi_emit`).
- **Scope of per-file SPDX headers.** "First-party Rust source" means the
  source of the workspace-member crates (`cerulion_core`, `cerulion_cli`,
  `cerulion_cli_engine`, `cerulion_cli_tui`, `cerulion_macros`, `rmw_cerulion`,
  `native_ros2_messages`, and the `crates/test_fixtures/*` cdylibs). Non-member
  directories are **not** individually headered, notably `benches/` (standalone
  benchmark harnesses and generated node crates, each with its own `Cargo.toml`,
  none published to crates.io). This is a per-file-annotation scope decision, not
  a licensing carve-out: **every file in this repository, headered or not, is
  governed by the top-level `LICENSE` (AGPL-3.0-only), except the three
  permissive crates `cerulion_link`, `cerulion_pairing` and `cerulion_wire`,
  which carry their own `MIT OR Apache-2.0` terms.** The SPDX headers are a
  convenience for downstreams reading individual files, not the sole grant.
- `deny.toml` `[licenses].allow` now includes `AGPL-3.0-only`.
- `NOTICE` and `AUTHORS` added.
