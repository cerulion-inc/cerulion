# Cerulion Debian and APT distribution

The Debian package is named `cerulion` and installs the CLI, the two daemon
binaries, the ROS 2 Jazzy rmw and the heap hook into `/usr/bin`:

* `cerulion`
* `cerulion-netd`
* `cerulion-connectd`
* `librmw_cerulion.so`
* `libcerulion_heaphook.so`

Keeping the files together is required because the CLI resolves its siblings
by its own directory: `cerulion connect` finds `cerulion-connectd` there, and
`cerulion ros2 run` and `cerulion ros2 launch` find `librmw_cerulion.so` there
and stage a minimal ament prefix that links it, so ROS 2's rmw discovery never
sees `/usr/bin` in a path. The same verbs prepend `libcerulion_heaphook.so` to
the ROS 2 child's `LD_PRELOAD` because it sits there (`CERULION_ROS2_PRELOAD=off`
disables that); the hook is a malloc interposer that lets the rmw fill an
unbounded message field straight into a loan slot, and it is transparent to
a node that never arms a borrow window. The package is assembled from the
release archive, so the bytes in the package are the release bytes. The rmw
library is built inside the `ros:jazzy` image against the real Jazzy headers
(`tools/scripts/build_rmw_jazzy.sh`); the hook is plain Rust over libc and is
built on the release runner like the binaries. The release workflow installs
each package in a clean Jazzy container and runs a talker and a listener
through it, checks that stock rclpy loads the shipped rmw, then checks that
the hook is preloaded into a node (the hook's own load breadcrumb and the
node's memory map, against a control with the file absent) before
publishing (`tools/scripts/verify_rmw_deb.sh`). It does not run `ros2
topic echo` against the talker: the ros2 CLI graph tools do not see
Cerulion topics (see the compatibility page), and `cerulion topic
list` and `cerulion topic echo` are the tools that read the shared-memory
plane.

The package depends on `libc6 (>= 2.35)`, `libstdc++6` and `libgcc-s1`, so
it installs only on Ubuntu 22.04+, Debian 12+, or another distribution with
glibc 2.35 or newer; `apt` refuses the install on an older release. The hook
needs glibc 2.34 or newer by design, which that floor already covers. ROS 2
itself is a `Suggests` (`ros-jazzy-ros-base`), not a dependency: the library
is loaded by a ROS 2 process and links no ROS library of its own, and a
`Depends` would refuse the CLI on every host without the ROS apt repository.
The rmw works where ROS 2 Jazzy works, Ubuntu 24.04 with `ros-jazzy-ros-base`
installed; on a host without ROS 2 the CLI installs and the `ros2` verbs
report that `ros2` is not on `PATH`. macOS packages carry neither the rmw
nor the hook because ROS 2 Jazzy has no macOS binaries and the hook
interposes glibc's allocator.

## License files

Every release archive and the Debian package built from it carry four license
files, because the binaries statically link their whole dependency graph and
most of those licenses require the text and the copyright notice to accompany
a binary distribution. `LICENSE` is the AGPL-3.0-only text the combined work
is under. `NOTICE` is the hand-written attribution for the components a reader
is most likely to care about, including the vendored ROS 2 message packages.
`LICENSE-BSD-3-CLAUSE` is the BSD text that seven of those message packages
require. `THIRD-PARTY-LICENSES.md` is the exhaustive machine-generated
inventory: every crate linked into the three binaries, grouped by license,
with each license text reproduced once. It is produced by `cargo about` from
the locked dependency graph under the policy in `tools/release/about.toml`, whose accepted license list mirrors the `[licenses]` allow list in
`deny.toml` so the release cannot ship a license CI would refuse. In the
archive the four sit at the top level beside the binaries; in the package they
install into `/usr/share/doc/cerulion/` alongside the `copyright` file, which
points at each of them by path. `build_deb.sh` refuses an archive that is
missing any of the four rather than producing a package without it.

## APT installation

The production APT repository is served from
`d2tdat71jcoj6e.cloudfront.net`.

Install the signing key and repository:

```bash
(
set -euo pipefail
sudo install -d -m 0755 /usr/share/keyrings
keyring_tmp=$(mktemp)
trap 'rm -f "$keyring_tmp"' EXIT
if ! curl -fsSL https://d2tdat71jcoj6e.cloudfront.net/cerulion-archive-keyring.gpg \
  -o "$keyring_tmp"; then
  printf '%s\n' 'error: could not download the Cerulion APT keyring' >&2
  exit 1
fi
if ! keyring_colons=$(gpg --batch --show-keys --with-colons "$keyring_tmp"); then
  printf '%s\n' 'error: could not inspect the downloaded Cerulion APT keyring' >&2
  exit 1
fi
keyring_fingerprints=$(printf '%s\n' "$keyring_colons" |
  awk -F: '$1 == "pub" { primary=1; next }
    primary && $1 == "fpr" { print toupper($10); primary=0; next }
    $1 == "sub" { primary=0 }' | sort -u)
expected_fingerprints=$(printf '%s\n' \
  953C69FD279D9256FA4B71C5DA5FDE5D0DC6D91B |
  sort -u)
if [ "$keyring_fingerprints" != "$expected_fingerprints" ]; then
  printf 'error: downloaded Cerulion APT keyring primary fingerprint set does not match the expected set (expected: %s; got: %s)\n' \
    "$expected_fingerprints" "$keyring_fingerprints" >&2
  exit 1
fi
sudo install -m 0644 "$keyring_tmp" \
  /usr/share/keyrings/cerulion-archive-keyring.gpg
rm -f "$keyring_tmp"
trap - EXIT
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/cerulion-archive-keyring.gpg] https://d2tdat71jcoj6e.cloudfront.net stable main" \
  | sudo tee /etc/apt/sources.list.d/cerulion.list >/dev/null
sudo apt-get update && sudo apt-get install cerulion
)
```

When a key rotation starts, add the new primary fingerprint as another line
in the `expected_fingerprints` `printf` list above before publishing the
overlap keyring. Keep both fingerprints in the list throughout the overlap;
remove the outgoing fingerprint again after the final new-key-only keyring is
published.

This is an apt-native update path: `apt upgrade` brings in new Cerulion
versions. A machine that installed a bare `.deb` does not receive updates from
APT; it must download a newer release asset or add this repository.
Because the keyring is a `Recommends`, clients with
`APT::Install-Recommends "false"` must install or upgrade
`cerulion-archive-keyring` explicitly. Otherwise a key rotation can leave the
client without the new verification key and `apt-get update` will fail with
`NO_PUBKEY`.
These no-Recommends clients are the population at risk during key rotation;
the standalone `.deb` remains installable without the keyring by design.

## Publisher inputs

`APT_GPG_KEY_ID` optionally lists the archive-signing fingerprints, separated
by commas and in signing order. When empty, publishing uses the recorded
organization fingerprint above.

`APT_KEYRING_KEYS` optionally lists the public-key fingerprints to include in
the standalone bootstrap keyring, separated by commas. It defaults to
`APT_GPG_KEY_ID`; during the two-publish key-removal transition it deliberately
remains a superset of the signing list.

`APT_GPG_PUBLIC_KEYS` is an optional armored public-key bundle imported by the
publish job after `APT_GPG_PRIVATE_KEY`. Set it during the key-removal
transition when `APT_KEYRING_KEYS` retains a key whose private material has
already been removed from `APT_GPG_PRIVATE_KEY`. The publish preflight checks
that every key in `APT_KEYRING_KEYS` is available after both bundles are
imported.

`APT_S3_PREFIX` controls the repository path in the bucket. The optional
`APT_CLOUDFRONT_PATH_PREFIX` only controls the path prefix used for CloudFront
invalidation and does not change where publication uploads objects in S3.
Configure the CloudFront origin mapping separately so that this viewer path
serves the repository from `APT_S3_PREFIX`, and use the same viewer path in
the client source URL. Leave the variable empty when the distribution serves
the repository at its root.

`APT_KEYRING_REVISION` is an optional non-negative integer that increments when
the archive keyring rotates, so clients receive the replacement keyring package.
The current distribution (its id is the repository variable
`APT_CLOUDFRONT_DISTRIBUTION_ID`) serves the repository at its
root, and therefore leaves `APT_CLOUDFRONT_PATH_PREFIX` empty.

The `cerulion-archive-keyring` package carries two files: the archive keyring
at `/usr/share/keyrings/cerulion-archive-keyring.gpg` and the repository entry
at `/etc/apt/sources.list.d/cerulion.list`. The entry is a conffile, so an
operator who repoints the machine at a mirror keeps that edit across upgrades.
Publication copies the package to `cerulion-archive-keyring.deb` at the
repository root, beside `cerulion-archive-keyring.gpg`, so that a client has
one stable address to bootstrap from: the copy under `pool/` is version
stamped and moves every release. That copy is made only after metadata
publication succeeds, so the bootstrap package can lag the live keyring but
never lead it, in either direction of a key rotation.

`APT_PUBLIC_URL` is an optional repository variable naming the address clients
fetch from, which is what the `sources.list` entry inside the keyring package
says. It defaults to the published repository named above; set it when
publishing somewhere else, and set it to the local server when building a
repository for a test, so that the package under test points at the repository
under test.

The publish role's ARN is the repository secret `APT_AWS_ROLE_ARN`. The S3 bucket
(the repository variable `APT_S3_BUCKET`) is
versioned and the role intentionally has no `s3:DeleteObject` permission:
publication must never delete pool files or repository objects.

Publication uploads immutable `by-hash/` objects first, then the single
client-visible signed metadata object, `InRelease`, and canonical index paths
last. A failure before the canonical-index overwrite therefore leaves the
previous generation fully intact; additive by-hash objects from the failed
generation are harmless. If a canonical-index overwrite fails, the publisher
retries restoration of the captured indexes before restoring `InRelease`. If
an index remains unavailable, it leaves signed metadata as-is, retains the
backup directory, reports which indexes were restored or failed, and prints
copyable commands for completing index restoration before restoring signed
metadata. This residual window affects only clients that ignore
`Acquire-By-Hash`; by-hash clients have already received content-addressed
objects before signed metadata names them. Restoration does not require delete
permission.

An atomic `Release`/`Release.gpg` pair is impossible on fixed S3 object keys:
copying either object first exposes a mismatch window. Every APT since 2013
prefers `InRelease`, and the Ubuntu 22.04 and 24.04 clients supported by this
project always use it. Pre-`InRelease` clients fail closed on a missing
signature rather than reporting `BADSIG`. The bucket has never
published a detached signature, so no stale detached object can be served. The publish role
has no `s3:DeleteObject` permission, which is why publication must not start
maintaining an object that may later need retirement. Before publication, the
guard refuses to proceed if `Release` or `Release.gpg` already exists: an
operator with delete permission must retire those keys manually before
switching a repository to InRelease-only publication.

The repository builder writes a new `dists.<epoch>` generation, retains the
previous generations, and atomically swaps the local `dists` symlink with a
temporary symlink and `mv -T`. Before publishing, the workflow downloads and
inspects the currently published keyring. It fails closed if that keyring
cannot be read, rather than guessing which cache ordering is safe.

The APT workflow selects `release-artifacts.yml` push runs whose head SHA
matches the tag commit and whose head branch is either the tag or empty (some
tag events omit the branch). It considers the candidates newest first and
decides on each candidate's `release` job (`Publish GitHub Release assets`),
never on the run's overall conclusion: a queued or in-progress job is waited
for, a completed job that did not succeed counts as unsuccessful, and a
successful one has the debs downloaded. The run's install-smoke and Homebrew
jobs fail for reasons unrelated to the assets, so their failure must not block
publication of debs that already exist. A failed run for another tag at the
same commit cannot by itself abort publication; the workflow fails only when
every matching candidate's release job concludes unsuccessfully.

## Re-publishing an existing tag

A tag whose release assets exist but whose APT publish was skipped or lost
(a gate refusal, an expired wait, a fixed publisher script) is re-published
by dispatching the workflow by hand from the branch whose scripts should do
the publishing, normally `main`:

```bash
gh workflow run apt-repo.yml --ref main -f tag=v1.0.0
```

The gate is the same as on a tag push: there must be a `release-artifacts.yml`
run at the tag's commit whose `release` job succeeded, so the two debs are on
the GitHub Release. A tag that does not exist, a prerelease tag, or missing
publisher configuration fails the dispatch loudly instead of skipping it.
Publication is additive: re-running a tag already in the pool re-signs and
re-uploads the repository metadata and never deletes a pool file or a
previous generation (the publish role has no delete permission).

When the incoming key set is a superset of the published set (a widening
publish), the workflow uploads the bootstrap keyring, invalidates its path,
and waits for that invalidation to complete before uploading `InRelease`. A
second invalidation covers the dists and pool paths after that upload. This
ordering ensures new metadata never reaches an edge before its verifying
keyring. Every non-virgin publication must also keep its incoming primary
signer in the currently published keyring. Direct A-to-B replacement is
refused; widen from A to A+B first.

When the incoming key set removes a published fingerprint (a narrowing
publish), the workflow follows the mirror image of widening: it publishes
metadata signed by the surviving key, invalidates `dists/*`, and waits for
that invalidation to complete before uploading the narrowed keyring. This
prevents a client from seeing the new keyring while CloudFront still serves
cached metadata signed only by the removed key. It then invalidates the pool
after the narrowed keyring is published; the old key remains available until
the signed metadata has propagated through the edge.

If the `dists/*` invalidation after metadata publication fails, the publisher
restores the captured previous metadata generation before reporting failure.
CloudFront can still serve the new metadata from cache after that rollback, so
the failure diagnostics include the exact `aws cloudfront create-invalidation`
command for `/dists/*`; run it after restoring the metadata before relying on
the repository again.

During narrowing, the publisher invalidates the additive package pool after
publishing the new metadata and narrowed keyring. It retries a failed pool
invalidation three times and then reports failure without rolling back: the
metadata and keyring have already propagated, and the pool contains additive
objects with nothing to undo. CloudFront may still serve pool objects whose
bytes do not match the published index hashes, so `apt-get install` can fail
until the pool cache is invalidated. Run:

```text
aws cloudfront create-invalidation --distribution-id <distribution-id> --paths "/<prefix>/pool/*"
```

The widening publication invalidates `dists/*` and `pool/*` together. If that
combined invalidation fails, publication still reports the stale-pool
consequence and prints the same `/pool/*` recovery command alongside the
metadata recovery guidance.

If canonical-index rollback exhausts its retries, the new `InRelease` remains
published while the old indexes have been restored, or only partially
restored, and `/dists/*` is still invalidated. This leaves signed metadata
from the new generation alongside indexes that may not match it. The retained
backup and diagnostic recovery commands are authoritative when this
incomplete state is reported; complete the repair manually before relying on
APT clients again.

There is one uncatchable interruption window: a `SIGKILL` immediately after
the new `InRelease` is uploaded but before the canonical indexes are
published. `SIGKILL` cannot be trapped by the publisher, so no in-process
ordering can eliminate that window. The `by-hash/` objects referenced by the
new `InRelease` are uploaded first and are content-addressed, so supported
APT clients using by-hash remain coherent. Only clients with by-hash disabled
can observe new signed metadata alongside old canonical indexes. Rerun the
publication to complete the canonical-index update and repair that bounded
window.

Prerelease tags are GitHub-release-only downloads and are never published to
the stable APT suite. This avoids Debian ordering a prerelease revision above
the later final release and stranding clients that installed the prerelease.
SemVer prerelease identifiers containing a hyphen, such as `rc-1`, are
rejected. The emitted upstream version has no Debian revision, so a hyphen
cannot be preserved safely; mapping it to `~` could silently invert precedence
against `rc.1`. Build metadata without hyphens (`+build`) remains supported.
Build metadata identifiers containing a hyphen, such as `build-a`, are also
rejected because mapping the separator would make distinct SemVer values share
one Debian package filename and APT pool path.

## Key custody and rotation

The signing key is held by the Cerulion organization release role, not in this
repository. The publish job imports the concatenated armored
`APT_GPG_PRIVATE_KEY` keys, prints their fingerprints, and derives
`cerulion-archive-keyring.gpg` from `APT_KEYRING_KEYS` (or the signing list
when that override is empty). When `APT_GPG_PUBLIC_KEYS` is configured, its
armored public keys are imported into the same protected GPG home before the
keyring list is validated. `InRelease` carries the primary key's cleartext
signature, where the primary is the first fingerprint in the signing list.
The builder also writes a local `Release.gpg` (one detached signature per
signing key, concatenated); it is neither verified nor uploaded to the
repository. The keyring carries every
listed public key. Before promotion, the builder rejects any generation whose
keyring does not contain every fingerprint that signed its metadata. No public
key is committed.

The current archive signing key is the Cerulion organization packaging role
key (`packaging@cerulion.com`):

* Fingerprint: `953C69FD279D9256FA4B71C5DA5FDE5D0DC6D91B`
* Long key ID: `DA5FDE5D0DC6D91B` (short key ID: `0DC6D91B`)

Clients can confirm that the served keyring matches this fingerprint with:

```bash
gpg --show-keys --with-colons /usr/share/keyrings/cerulion-archive-keyring.gpg
```

The `fpr` record in that output must contain the fingerprint above. During a
rotation, it contains both the old and new fingerprints during the overlap.

To rotate the key, first prepare a new key in a private directory:

```bash
set -euo pipefail
umask 077
if ! gnupg_home="$(mktemp -d)"; then
  printf '%s\n' 'error: could not create a temporary GPG home' >&2
  exit 1
fi
export GNUPGHOME="$gnupg_home"
chmod 700 "$GNUPGHOME"
trap 'rm -rf "$gnupg_home"' EXIT
if ! gpg --batch --pinentry-mode loopback --passphrase '' --quick-generate-key \
    'Cerulion APT <packaging@cerulion.com>' ed25519 sign 2y; then
  printf '%s\n' 'error: could not generate the APT signing key' >&2
  exit 1
fi
if ! fingerprint="$(gpg --batch --with-colons --list-secret-keys |
    awk -F: '$1 == "fpr" { print $10; exit }')"; then
  printf '%s\n' 'error: could not inspect the generated APT signing key' >&2
  exit 1
fi
if [ -z "$fingerprint" ]; then
  printf '%s\n' 'error: generated APT signing key has no fingerprint' >&2
  exit 1
fi
if ! gpg --batch --armor --export-secret-keys "$fingerprint" > APT_GPG_PRIVATE_KEY.asc; then
  rm -f APT_GPG_PRIVATE_KEY.asc
  printf '%s\n' 'error: could not export the APT signing key' >&2
  exit 1
fi
chmod 0600 APT_GPG_PRIVATE_KEY.asc
if ! gpg --batch --export "$fingerprint" > cerulion-archive-keyring.gpg; then
  rm -f cerulion-archive-keyring.gpg
  printf '%s\n' 'error: could not export the APT keyring' >&2
  exit 1
fi
chmod 0644 cerulion-archive-keyring.gpg
rm -rf "$gnupg_home"
trap - EXIT
unset GNUPGHOME
printf 'new APT signing fingerprint: %s\n' "$fingerprint"
```

Keep `APT_GPG_PRIVATE_KEY.asc` mode `0600`, import it only into the protected
release secret, and shred it after import:

```bash
shred -u APT_GPG_PRIVATE_KEY.asc
```

Publish the transition in four stages:

1. **Widening: keyring first.** Set `APT_GPG_PRIVATE_KEY` to the concatenated armored old and new private
   keys, set `APT_GPG_KEY_ID` to both fingerprints in that order, and bump
   `APT_KEYRING_REVISION`. The old fingerprint must be first: `InRelease` is
   clearsigned only by the first fingerprint. The local detached-signature
   test also verifies both keys, and the keyring package contains both. The
   builder rejects a generation unless its published keyring contains every
   fingerprint used to sign that generation; during this sequence the live
   overlap keyring therefore remains a superset of the next generation's
   signing keys. The publisher refuses any non-virgin publication when its
   primary `InRelease` signer is not already in the currently published
   keyring. Direct A-to-B replacement is refused, so widen from A to A+B first.
   Keep the old fingerprint first in `APT_KEYRING_KEYS` during this stage.
   Before publishing the overlap, add the new fingerprint as a second line
   in the `expected_fingerprints` list in this document's bootstrap snippet.
   Keep both expected fingerprints through the overlap.
2. **Inventory only: no publication.** Before removing the old key, perform a mandatory inventory check: verify
   that every client, including clients with `APT::Install-Recommends "false"`,
   has installed the overlap `cerulion-archive-keyring` containing both
   fingerprints. Give clients time to run `apt-get update && apt-get upgrade`;
   no-Recommends clients must be explicitly installed/upgraded with the
   keyring package. They can then verify `InRelease` with the still-trusted old
   key and receive the two-key keyring through APT. Keep the old fingerprint
   first in `APT_KEYRING_KEYS` throughout this inventory stage as well.
3. **Widening/equal: keyring first.** Remove the old key from `APT_GPG_KEY_ID` but keep it in the keyring
   fingerprint list, set `APT_GPG_PUBLIC_KEYS` to an armored export of the old
   public key, and remove the old private key from `APT_GPG_PRIVATE_KEY`. Bump
   `APT_KEYRING_REVISION`. The publish preflight must confirm that the public
   bundle supplies the retained old fingerprint. This publish signs new
   metadata only with the new key while continuing to serve a keyring
   containing both keys.
4. **Narrowing: metadata first.** After clients have received that publish, remove the old key from the
   keyring fingerprint list, remove it from `APT_GPG_PUBLIC_KEYS`, and bump
   `APT_KEYRING_REVISION` again. The next publish can then package the
   new-key-only keyring and make the new key primary. The workflow first waits
   for `dists/*` invalidation after publishing new-key-signed metadata, then
   publishes and invalidates the narrowed keyring. Remove the outgoing
   fingerprint from the `expected_fingerprints` list in this document's
   bootstrap snippet after that final publish.

Dropping the old key from signing before clients have installed the two-key
keyring breaks those clients: they cannot use APT to obtain the replacement
keyring. Dropping it from the keyring in the same publish is also unsafe for
clients still verifying the outgoing metadata. The two removal publishes keep
the bootstrap keyring a superset of both generations at every transition.
When rotating, update the recorded fingerprint and key IDs above in the same
commit as the signing-secret and repository-variable changes.

After the final removal publish, shred `APT_GPG_PRIVATE_KEY.asc` and remove
the generated public keyring from the working directory. The next tagged
publish serves the new derived keyring and signs the new repository metadata
with the corresponding private key.
