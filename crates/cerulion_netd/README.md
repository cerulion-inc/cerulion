# cerulion_netd

The per-computer network gateway daemon for
[Cerulion](https://github.com/cerulion-inc/cerulion), `cerulion-netd`.

One `cerulion-netd` runs per computer. It owns that machine's single zenoh
session and serves remote topics to every local tool: `cerulion topic echo`,
`cerulion viz` and Cerulion Studio all ask it for a remote robot's topic, the
frames cross the network once, and netd mirrors them into local shared memory
where every reader subscribes to the same copy. The recorder captures local
producer traffic, so it records a remote topic once netd has mirrored it; see
the recording guide.

## Install

```bash
cargo install --locked cerulion_cli cerulion_netd
```

The release installer and the Homebrew and Debian packages already place
`cerulion-netd` beside the `cerulion` binary; see the
[installation guide](https://docs.cerulion.com/cerulion/installation).

## Running it

You normally never start it by hand. The first command that needs a remote topic
spawns it detached, and a demand-started daemon exits once its clients have been
idle for the grace period (30 seconds by default). A daemon started with a
listen address stays available for remote clients.

On a robot that should be discoverable, run it as a standing service with a
listen address. That also raises the `_cerulion._tcp` mDNS beacon that
`cerulion topic list` browses:

```bash
CERULION_NETD_LISTEN=tcp/0.0.0.0:7683 cerulion-netd
```

`cerulion-netd --help` lists every environment variable, and the
[networking guide](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots)
covers the whole picture.

## Features

- `wan` (default): the iroh plane for robots reached across the internet.
  `--no-default-features` builds the LAN-only (zenoh) daemon.

## Library

The crate also exposes `NetdClient`, the client the CLI and the visualization
daemon use to connect to (or spawn) the daemon and hold a topic demand. Design
notes for contributors live in
[docs/internals/network-daemons.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/network-daemons.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
