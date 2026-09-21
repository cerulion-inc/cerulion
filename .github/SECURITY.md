# Security Policy

## Supported Versions

Cerulion is pre-1.0. We support only the latest tagged release: there are
no LTS branches or backports. Upgrade to the latest release before
reporting a vulnerability found on an older one.

## Reporting a Vulnerability

**Do not open a public GitHub issue for security vulnerabilities.**

If you discover a security vulnerability in Cerulion, please report it responsibly:

1. **Email**: Send a detailed report to security@cerulion.com
2. **Include**:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if any)

## Response Timeline

- **Acknowledgment**: Within 48 hours of report
- **Initial assessment**: Within 5 business days
- **Fix timeline**: Depends on severity, but we aim for:
  - Critical: mitigation guidance immediately; a patch as fast as possible, typically within a week
  - High: 2 weeks
  - Medium: next release cycle
  - Low: next release cycle

## Scope

The following are in scope for security reports:

- **Shared memory safety**: Unauthorized access to shared memory segments
- **Wire format parsing**: Buffer overflows, out-of-bounds reads in header/payload parsing
- **Network transport**: Data injection or spoofing on the LAN plane; any pairing or authentication bypass on the remote plane
- **Account sign-in**: The device-code login, token handling, and the credential files under `~/.cerulion`
- **Code generation**: Injection via crafted schema files
- **Dynamic library loading**: Unsafe cdylib loading paths

## Out of Scope

- Denial of service via resource exhaustion (shared memory limits are OS-level)
- Issues in upstream dependencies (report to iceoryx2, zenoh, etc. directly)
- Issues requiring physical access to the host machine

## Disclosure Policy

We follow coordinated disclosure. We will:

1. Confirm the vulnerability and determine its impact
2. Develop and test a fix
3. Release a patched version
4. Publicly disclose the vulnerability with credit to the reporter (unless anonymity is requested)

We appreciate your help in keeping Cerulion and its users safe.
