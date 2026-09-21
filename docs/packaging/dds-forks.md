# Publishing the DDS Forks

The release workflow publishes the Cerulion workspace, but it does not publish
the two external DDS forks. Publish these crates manually before a release that
updates or consumes them.

## Why two forks exist

`cerulion-rustdds` is retained only for
`DomainParticipantBuilder::participant_lease_duration`. RustDDS 0.14.2 already
provides endpoint `USER_DATA` parsing and the
`discovered_readers()`/`discovered_writers()` snapshot accessors.

The second fork is still required because published `ros2-client` owns the
`rustdds ^0.14` dependency edge. A root `[patch.crates-io]` override does not
reach published consumers, so the renamed `cerulion-ros2-client` dependency
selects the renamed RustDDS package while preserving the Rust library name
`rustdds`.

## Registry installation

Every build of this workspace, a source checkout included, resolves
`cerulion-rustdds` and `cerulion-ros2-client` from crates.io. The workspace no
longer carries a `[patch.crates-io]` override for the DDS stack, because a
patch does not reach published consumers: packaged `cerulion_dds` depends on
`cerulion-ros2-client`, which in turn selects the published `cerulion-rustdds`
while preserving the Rust library names `ros2_client` and `rustdds`.

So a new upstream release must be published as both renamed fork crates
before anything can build against it, including this workspace. To build
against unpublished fork work, add these two entries to the root
manifest's existing `[patch.crates-io]` table (it already declares that table
for `re_grpc_server`, and a second table header of the same name is a manifest
error), then remove them before committing:

```toml
cerulion-rustdds = { path = "../RustDDS" }
cerulion-ros2-client = { path = "../cerulion-ros2-client" }
```

## Current provenance

| Package | Upstream release | Upstream commit | Fork repository | Fork branch |
|---|---|---|---|---|
| `cerulion-rustdds` | `Atostek/RustDDS` 0.14.2 | `0a255c6c60e4e0ce644265fa362b0aa01b4034be` | `https://github.com/cerulion-inc/RustDDS` | `chore/publish-as-cerulion-rustdds-0142` |
| `cerulion-ros2-client` | `Atostek/ros2-client` 0.10.1 | `c60958d9c252e46779aaaf86bd2e986e68a52646` | `https://github.com/cerulion-inc/cerulion-ros2-client` | `chore/publish-as-cerulion-ros2-client-0101` |

For both releases, the upstream release `src/` tree was verified byte-identical
to the corresponding crates.io `.crate` tarball. The RustDDS fork then adds
only the two lease commits; the ros2-client fork leaves `src/` unchanged and
changes only package metadata plus the renamed RustDDS dependency.

## Manual publication procedure

For a new upstream release:

1. Fetch the canonical upstream repository and release tag. Use
   `Atostek/RustDDS` and `Atostek/ros2-client`; the former is the canonical
   repository behind the historical `jhelovuo/RustDDS` redirect.
2. Download the matching crates.io `.crate` tarball and compare its `src/`
   tree with the selected release commit before making fork changes.
3. In the RustDDS checkout, create a fresh branch from the release commit.
   For the current 0.14.2 fork, cherry-pick the two lease commits from
   the FORK, `cerulion-inc/RustDDS` (they exist in no `Atostek/RustDDS`
   branch or tag, so add that fork as a second remote and fetch it
   first): `e8e80c27` (`participant_lease_duration`), carried
   on the publish branch as `ac89cb74`, and `4f18a034` (the SPDP derivation
   and period-test pins), carried as `6d9bf5c9`. For a future upstream
   release, cherry-pick the same two fork commits. If upstream has changed
   the surrounding code, port only that lease work by hand.
4. Re-apply the RustDDS publication changes:
   `name = "cerulion-rustdds"`, the matching upstream version,
   `repository = "https://github.com/cerulion-inc/RustDDS"`, and
   `[lib] name = "rustdds"`. Update the description, README paragraph, and
   `NOTICE` to identify the upstream release and the lease-only delta.
5. Run the RustDDS package checks and publish it first:

   ```text
   RUSTUP_TOOLCHAIN=1.93.0 cargo publish --dry-run
   RUSTUP_TOOLCHAIN=1.93.0 cargo publish
   ```

   Wait until the published version is visible in the crates.io index.
6. In the ros2-client checkout, create a fresh branch from the matching
   upstream release commit. Re-apply the package rename,
   `[lib] name = "ros2_client"`, repository metadata, README, and `NOTICE`.
   Change only the dependency to:

   ```toml
   rustdds = { package = "cerulion-rustdds", version = "<matching-version>" }
   ```

   Preserve all source imports, feature names, and feature/default-feature
   behavior. Run its dry-run, then publish it:

   ```text
   RUSTUP_TOOLCHAIN=1.93.0 cargo publish --dry-run
   RUSTUP_TOOLCHAIN=1.93.0 cargo publish
   ```

7. Bump `cerulion_dds` to the new `cerulion-ros2-client` version, update the
   provenance comments, remove any temporary path overrides, and regenerate
   the root `Cargo.lock` against crates.io. Confirm the lock has registry
   sources and checksums for both renamed packages.
8. Run the normal workspace release gates. Do not claim that a workspace
   `cargo publish --workspace` run published either external fork.

Cerulion depends on the published fork crates listed above. Each carries a
`repository` URL that resolves to its fork repository; crate metadata cannot be
edited after publish, so the fork repository must exist before its crate is
published.

The release workflow's `cargo publish --workspace` publishes only the
Cerulion workspace. It does not publish either external fork, and the
release-artifact smoke waits only for the workspace crates used by the
scaffolded node (`cerulion_macros`, `cerulion_core`, and
`native_ros2_messages`).

## Exit condition

When upstream RustDDS ships a participant lease duration knob, remove both
renamed forks, depend on plain `rustdds`/`ros2-client`, regenerate the lock, and
revalidate the live discovery path against a live peer.
