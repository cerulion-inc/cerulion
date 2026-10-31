# cerulion_telemetry

Consent and content-free usage events for the desk surfaces of
[Cerulion](https://github.com/cerulion-inc/cerulion): the `cerulion` CLI and
`cerulion-vizd`. Robot and runtime crates never depend on it.

- **Consent** is resolved from `DO_NOT_TRACK`, then `CERULION_TELEMETRY`, then
  the shared `telemetry.json` consent file. An unreadable consent file counts
  as opted out.
- **The property guard** drops any value that looks like a URL, an email
  address or a path, and any string longer than 128 characters, so an event
  can only carry names, flags, counts and buckets.
- **The `posthog` feature** adds a bounded batch client with a hard shutdown
  budget. Without the feature, or without a project key, every call is a
  no-op and the crate has no network or serialization dependency.

What is sent and how to turn it off is documented in
[docs/telemetry.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/telemetry.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
