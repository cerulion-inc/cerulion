# cerulion_hygiene

The Unix-socket lifecycle shared by the
[Cerulion](https://github.com/cerulion-inc/cerulion) desk daemons
(`cerulion-netd`, `cerulion-vizd`, `cerulion-wsd`) and the clients that
connect to them.

A daemon is described by one `DaemonSocket` value (the constants `NETD`,
`VIZD` and `WSD`), and everything a daemon and its clients must agree on comes
from it:

- **The socket-path ladder**: the daemon's environment variable if set, else
  `$XDG_RUNTIME_DIR/cerulion/<name>.sock`, else `$HOME/.cerulion/<name>.sock`,
  else `/tmp/cerulion-<euid>/<name>.sock`.
- **The single-daemon lock**: an exclusive `flock` on a pidfile beside the
  socket, taken before the socket is touched, so two starts can never both win
  and a crashed daemon's leftover socket is recovered without a PID probe.
- **The socket-directory check**: the directory is checked, never trusted,
  because anyone who can write into it could replace the socket.

## Who uses it

This is an internal building block: it is published because `cerulion_netd` and
the `cerulion` CLI depend on it. As a user you meet it only through the socket
environment variables (`CERULION_NETD_SOCK`, `CERULION_VIZD_SOCK`,
`CERULION_WSD_SOCKET`) documented in the
[API reference](https://github.com/cerulion-inc/cerulion/blob/main/docs/user-api.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
