#!/usr/bin/env python3
"""leak_scan.py: the in-tree leak guard.

Keeps machine names, addresses, home paths, logins, people and location
metadata out of everything this repository publishes: file contents, file and
branch names, commit messages and identities, pull request text, and media
containers.

Two tiers:
  GENERIC classes ship here. They are SHAPES only (a home path, an address in
  a shared range, a login at a host). Nothing in this file names a real
  machine, address, login or person.
  PRIVATE patterns never ship. They load from the environment or from a file
  outside the tree. A private hit prints the file, the line number and the
  pattern INDEX, never the matched text, so a CI log cannot publish the name.
  A generic hit whose text is itself an address, host, login or path segment
  (IDENTITY_CLASSES) prints its value only with the private tier loaded and
  text output; with no tier, or in --format github, it prints a masked shape.

Usage:
  leak_scan.py tree      [--ref REV | --staged] [--files-from FILE|-] [--untracked] [common]
  leak_scan.py diff      (--range A..B | --staged) [common]
  leak_scan.py messages  [--range A..B] [--message-file F] [--pr-title-env VAR]
                         [--pr-body-env VAR] [--pr-title-file F] [--pr-body-file F]
                         [--allow-email-file F] [--ident-from-git] [--require-commits] [common]
  leak_scan.py names     [--ref REV | --staged | --range A..B]
                         [--branch NAME | --branch-env VAR] [--untracked] [common]
  leak_scan.py media     [--ref REV | --staged | --files-from FILE|-] [common]
  leak_scan.py hook      (pre-commit | commit-msg FILE)
  leak_scan.py --self-test
  leak_scan.py --list-classes
  leak_scan.py --list-lan-values
  common: --require-private  --no-private  --hard CLASS (repeatable)
          --format text|github  --allow FILE  --no-allow  --allow-empty  --quiet

Private pattern sources, first one that yields a pattern wins (never merged):
  1. env LEAK_PATTERNS (newline separated)
  2. the file named by env LEAK_PATTERNS_FILE
  3. the default file under the user's config directory
Entry forms, one per line, '#' comments:
  word:<literal>   case-insensitive, alphanumeric boundaries, tolerant of a
                   separator at every letter/digit transition (fast path)
  text:<literal>   case-insensitive substring, no boundaries (fast path)
  <regex>          any other line is a Python regex (slow path)
  An entry may end in a category tag from a fixed public vocabulary
  (@host @device @person @login @lan @nickname @slug @hygiene). A tagged
  hit reports as private#<index>@<tag>. A machine spoken about by a
  NICKNAME rather than by its host name is @nickname: no generic class can
  see that shape, so the list is the only place it is refused.

Exit codes:
  0  ran, controls passed, zero HARD hits
  1  ran, at least one HARD hit (or a stale allowlist entry in full-tree mode)
  2  malformed invocation
  3  COULD NOT RUN (bad ref, git failure, zero units, a private pattern that
     does not compile, a dead built-in control, or --require-private with no
     private pattern loaded)

Stdlib only. Python 3.8 or newer.
"""
import argparse
import base64
import bisect
import binascii
import collections
import json
import os
import re
import stat
import struct
import subprocess
import sys
import tempfile
import time
import unicodedata
import zlib

EXIT_OK, EXIT_HIT, EXIT_USAGE, EXIT_NORUN = 0, 1, 2, 3


class NoRun(Exception):
    """The scan could not run. Maps to exit 3."""


class Usage(Exception):
    """Malformed invocation. Maps to exit 2."""


# ---------------------------------------------------------------------------
# Fragments. Every literal that a class would match is ASSEMBLED here, so this
# file scans clean against its own class table with no allowlist entry. The
# self-test pins that both ways.
# ---------------------------------------------------------------------------
SL = '/'
BSL = chr(92)
ESC = chr(27)
TILDE = chr(126)
P_MAC = SL + 'Users' + SL
P_LINUX = SL + 'home' + SL
P_TEMP = SL + 'var' + SL + 'folders' + SL
OVERLAY_WORDS = ('tail' + 'scale', 'tail' + 'net', 'magic' + 'dns', 'head' + 'scale')
DNS_LABEL = 'ts'
DNS_TLD = 'net'
DASH_EN = chr(0x2013)
DASH_EM = chr(0x2014)
DASH_ESCAPE = BSL + 'u201'
PROJECT_DOMAIN = 'cerulion' + '.com'
FORGE_NOREPLY = 'users.noreply.' + 'github.com'
FORGE_WEBFLOW = 'noreply@' + 'github.com'
PRAGMA_WORD = 'leak-scan' + ':'
TEXT_CAP = 64 * 1024 * 1024
CHUNK = 1024 * 1024
OVERLAP = 4096
NESTED_CAP = 4 * 1024 * 1024
INFLATE_CAP = 1024 * 1024
MAX_REPORT_LINES = 20
WITHHELD = '(text withheld: a private pattern touches this line)'
# `nickname` is the class for a machine spoken about by a nickname rather than
# by its host name ("the Air", "the big box"): the shape carries no host name,
# no address and no login, so no generic class can see it and only an entry in
# this list can.
TAGS = ('host', 'device', 'person', 'login', 'lan', 'nickname', 'slug', 'hygiene')
# The generic classes whose matched text IS the address, host, login or path
# segment. Their value prints only where the private tier is loaded AND the
# output is a terminal (text format): with no tier the scanner cannot know the
# value is not a name the tier would have withheld, and a CI log (github
# format) outlives the force-push that scrubs a branch, so both get a masked
# shape. style-dash and overlay-word are not here: their text names nothing.
IDENTITY_CLASSES = frozenset((
    'lan-addr', 'cgnat-addr', 'home-mac', 'home-linux', 'home-win', 'home-tilde', 'temp-root',
    'login-at-host', 'mdns-local', 'host-field', 'overlay-dns', 'email-personal', 'acl-tag'))
MASK_RX = re.compile(r'[^\W_]')

# Published placeholder vocabularies. Explicit sets, never a length rule.
PH_USER = frozenset((
    'dev', 'someone', 'you', 'me', 'user', 'username', 'name', 'example', 'runner', 'shared',
    'alice', 'bob', 'ubuntu', 'root', 'ros', 'robot', 'builder', 'ci', 'yourname', 'your_user',
    'your-user', 'foo', 'bar', 'test', 'tester', 'nobody', 'x', 'u', 'h', 'op', 'unitree',
    'vscode', 'docker', 'app', 'work', 'build', 'home', 'jenkins', 'git'))
PH_HOST = frozenset((
    'box-x86', 'box-arm', 'box-jetson', 'box-ci', 'mac-m4', 'localhost', 'host', 'hostname',
    'example', 'redacted', 'unknown', 'runner', 'robot', 'desk', 'none', 'null', 'my-robot',
    'myrobot', 'robot1', 'robot2', 'go2', 'test-host', 'testhost'))
ROLE_LOCAL = frozenset((
    'licensing', 'packaging', 'bench', 'conduct', 'security', 'dev', 'build-test', 'legal',
    'support', 'hello', 'press', 'noreply'))
ROLE_PREFIXES = ('apt-',)
CONSUMER_MAIL = ('gmail', 'googlemail', 'outlook', 'hotmail', 'live', 'yahoo', 'icloud', 'me',
                 'mac', 'proton', 'protonmail', 'pm', 'fastmail', 'hey', 'aol', 'gmx', 'yandex',
                 'qq', '163')

# The published example set for private-range addresses: the distinct values
# present in the tree when the guard landed (regenerate and review with
# --list-lan-values). New text reuses one of these or a documentation range.
# A subnet-shaped allowance is NOT used: real addresses live in the same
# conventional subnets that examples use, so only exact values are excused.
LAN_EXAMPLES = frozenset((
    '10.0.0.1', '10.0.0.2', '10.0.0.3', '10.0.0.4', '10.0.0.42', '10.0.0.5', '10.0.0.6',
    '10.0.0.7', '10.0.0.8', '10.0.0.9', '10.1.2.3', '10.1.2.5', '10.9.9.9', '192.168.0.1',
    '192.168.1.0', '192.168.1.1', '192.168.1.10', '192.168.1.11', '192.168.1.126',
    '192.168.1.127', '192.168.1.128', '192.168.1.20', '192.168.1.254', '192.168.1.255',
    '192.168.1.4', '192.168.1.42', '192.168.1.49', '192.168.1.5', '192.168.1.50',
    '192.168.1.51', '192.168.1.63', '192.168.1.64', '192.168.1.65', '192.168.1.66',
    '192.168.1.7', '192.168.1.8', '192.168.1.9', '192.168.123.100', '192.168.123.161',
    '192.168.123.18', '192.168.123.5', '192.168.123.99'))

GUARD_FILES = frozenset((
    'tools/scripts/leak_scan.py', 'tools/scripts/leak_scan_allow.txt',
    'tools/scripts/install_hooks.sh', '.github/workflows/leak-guard.yml',
    'docs/internals/leak-guard.md'))
GUARD_PREFIXES = ('tools/hooks/',)

OCT = r'(?:25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)'
NB_L = r'(?<![0-9.])'
NB_R = r'(?![0-9]|\.[0-9])'
SEG = r'([^/\s"\'`<>|:,;)\]}]+)'
IPV4_RX = re.compile(OCT + r'(?:\.' + OCT + r'){3}')
ANSI_RX = re.compile(
    '\x1b\\[[0-9;?]*[ -/]*[@-~]|\x1b\\][^\x07\x1b\n]*(?:\x07|\x1b' + BSL + BSL + ')|\x1b[@-Z'
    + BSL + BSL + '-_]')
DEBRACKET_RX = re.compile(r'\[([A-Za-z0-9])\]')
UESC_RX = re.compile(BSL + BSL + r'u([0-9a-fA-F]{4})')
DATA_URI_RX = re.compile(r'base64,([A-Za-z0-9+/=\s]{16,})')
PRAGMA_RX = re.compile(re.escape(PRAGMA_WORD) + r'\s*allow\s+(\S+)\s+(.*)$')
# The control sequence forms only (CSI and OSC): used on a DECODED view, where a
# bare escape that is not one of these is blanked rather than allowed to eat the
# letter after it.
CSI_OSC_RX = re.compile(
    '\x1b\\[[0-9;?]*[ -/]*[@-~]|\x1b\\][^\x07\x1b\n]*(?:\x07|\x1b' + BSL + BSL + ')')
# Backslash escapes (C, Rust, Python, shell and regex spellings) and runs of
# percent escapes, in four passes so that every pattern starts with a literal
# and the engine can skip to the next candidate instead of trying each offset:
# numeric and ESC-producing escapes (a callback decodes them), percent runs, then
# a backslash before a single letter (an escape in every one of those languages,
# so the letter goes with it), then a backslash before anything else (dropped).
ESC_NUMERIC_RX = re.compile(
    BSL + BSL + r'(?:x([0-9a-fA-F]{2})|u\{([0-9a-fA-F]{1,6})\}|u([0-9a-fA-F]{4})'
    r'|U([0-9a-fA-F]{8})|([0-7]{1,3})|([eE]))')
PCT_RUN_RX = re.compile(r'(?:%[0-9a-fA-F]{2})+')
ESC_LETTER_RX = re.compile(BSL + BSL + r'[A-Za-z]')
ESC_OTHER_RX = re.compile(BSL + BSL + r'(.)')
INVISIBLE_EXTRA = ((0x034F, 0x034F), (0xFE00, 0xFE0F), (0xE0100, 0xE01EF))
_CF_RX = [None]
_CONTROL_RX = re.compile('[\x00-\x08\x0a-\x1f\x7f-\x9f\u2028\u2029]')


def _cf_rx():
    """A class of every Unicode format character (Cf) plus the invisible joiners
    and selectors, built once from the running interpreter's Unicode tables."""
    if _CF_RX[0] is None:
        ranges = []
        start = prev = None
        for code in range(0x80, sys.maxunicode + 1):
            if unicodedata.category(chr(code)) != 'Cf':
                continue
            if start is None:
                start = prev = code
            elif code == prev + 1:
                prev = code
            else:
                ranges.append((start, prev))
                start = prev = code
        if start is not None:
            ranges.append((start, prev))
        ranges.extend(INVISIBLE_EXTRA)
        body = ''.join('%s-%s' % (re.escape(chr(a)), re.escape(chr(b))) if a != b
                       else re.escape(chr(a)) for a, b in ranges)
        _CF_RX[0] = re.compile('[' + body + ']')
    return _CF_RX[0]


def drop_cf(text):
    """The text without format characters: a zero width space, a soft hyphen or a
    word joiner splits a name on the screen of nothing and must not split it here."""
    if text.isascii():
        return text
    return _cf_rx().sub('', text)


def _decoded_char(code):
    if code == 0x1B:
        return ESC
    if code < 0x20 or 0x7F <= code <= 0x9F or 0xD800 <= code <= 0xDFFF or code in (
            0x2028, 0x2029) or code > 0x10FFFF:
        return ' '
    return chr(code)


def _decode_numeric(m):
    for i, base in ((1, 16), (2, 16), (3, 16), (4, 16), (5, 8)):
        if m.group(i):
            return _decoded_char(int(m.group(i), base))
    return ESC


def _decode_percent(m):
    raw = bytes(int(h, 16) for h in m.group(0)[1:].split('%'))
    return ''.join(_decoded_char(ord(ch)) for ch in raw.decode('utf-8', 'replace'))


def decode_escapes(text):
    """One pass over backslash and percent escapes, then the control sequences they
    spell are stripped and every other decoded control character is a space, so a
    name written right after a literal escape has the boundary it has on screen."""
    out = text
    if BSL in out:
        out = ESC_NUMERIC_RX.sub(_decode_numeric, out)
    if '%' in out:
        out = PCT_RUN_RX.sub(_decode_percent, out)
    if ESC in out:
        out = CSI_OSC_RX.sub('', out).replace(ESC, ' ')
    if BSL in out:
        out = ESC_LETTER_RX.sub(' ', out)
        out = ESC_OTHER_RX.sub(r'\1', out)
    return out


def _blank_unchanged(view, parent):
    """The view with every line identical to the parent's blanked: the parent was
    scanned already, and hits are keyed per line. Line counts are equal by
    construction (no transform adds or removes a newline); if not, keep the view."""
    if '\n' not in view:
        return view
    a, b = view.split('\n'), parent.split('\n')
    if len(a) != len(b):
        return view
    return '\n'.join('' if x == y else x for x, y in zip(a, b))


def text_views(text):
    """The raw text and every normalised view of it that could hide a name: escapes
    stripped, format characters dropped, brackets removed, JSON escapes and then
    backslash and percent escapes decoded (twice, for once-nested strings). Each
    derived view carries only the lines its parent did not already show."""
    out = [text]
    a = text
    if not a.isascii():
        a = drop_cf(a)
    if ESC in a:
        a = ANSI_RX.sub('', a)
    if a != text:
        out.append(_blank_unchanged(a, text))
    if '[' in a:
        d = DEBRACKET_RX.sub(r'\1', a)
        if d != a:
            out.append(_blank_unchanged(d, a))
    if (BSL + '/') in a or (BSL + 'u') in a:
        j = a.replace(BSL + '/', '/')
        j = UESC_RX.sub(lambda m: _safe_chr(int(m.group(1), 16)), j)
        if j != a:
            out.append(_blank_unchanged(j, a))
    e = a
    for _ in range(2):
        if BSL not in e and '%' not in e:
            break
        n = decode_escapes(e)
        if n == e:
            break
        nb = _blank_unchanged(n, e)
        if not nb.isascii():
            nb = drop_cf(nb)
        if '[' in nb:
            nb = DEBRACKET_RX.sub(r'\1', nb)
        out.append(nb)
        e = n
    return out


def clean(text):
    """A fragment about to be PRINTED: no escape sequence, no format character, no
    control character, no line break, and no old-style workflow command marker."""
    if ESC in text:
        text = ANSI_RX.sub('', text)
    text = drop_cf(text)
    text = _CONTROL_RX.sub('?', text)
    return text.replace('##[', '# #[')


def shape_of(text):
    """`text` with every letter and digit replaced: the length and the punctuation
    stay, the value does not."""
    return MASK_RX.sub('x', clean(text))[:80]


def mask_shape(text):
    """The matched text as a masked shape, so the author can tell which token on
    the line it was, and the value does not leave the scanner."""
    return '(value masked, %d chars: %s)' % (len(clean(text)), shape_of(text))


def safe_line(line):
    """Every printed line passes here: a line break inside it would let untrusted
    text (a path, a match) start a new line at column zero, which is where a
    workflow command lives; a control or format character has no business in a
    log either."""
    line = _CONTROL_RX.sub('?', line)
    if not line.isascii():
        line = drop_cf(line)
    return line.replace('##[', '# #[')


def user_ok(seg):
    """True when a path or login segment is a published placeholder."""
    s = seg.lower()
    return (s in PH_USER or s.startswith(('<', '$', '{', '%', '*', '.', TILDE))
            or 'example' in s or s.startswith('your') or s == 'boxhome')


def host_ok(host):
    """True when a host label is a published placeholder or an address (other classes judge)."""
    s = host.lower().rstrip('.')
    if s in PH_HOST or s.startswith(('example', 'your', 'my', 'robot-', 'box-', 'mac-')):
        return True
    if s.endswith(('.example', '.invalid', '.test', '.example.com', '.example.org',
                   '.example.net')):
        return True
    return IPV4_RX.fullmatch(s) is not None


def in_results_tree(path):
    parts = path.lower().split('/')
    return 'results' in parts or 'evidence' in parts


def is_guard_file(path):
    return path in GUARD_FILES or path.startswith(GUARD_PREFIXES)


def _network_definition(line, end, addr):
    """True when the address at line[:end] is followed by /N and has no host bits set."""
    m = re.match(r'/(\d{1,2})(?!\d)', line[end:end + 4])
    if not m:
        return False
    bits = int(m.group(1))
    if bits > 30:
        return False
    a, b, c, d = (int(x) for x in addr.split('.'))
    value = (a << 24) | (b << 16) | (c << 8) | d
    hostmask = (1 << (32 - bits)) - 1
    return value & hostmask == 0


class Cls(object):
    """One generic class: a shape, its anchors, its filter and its severity per mode."""

    def __init__(self, cid, rx, anchors, flt, sev, sample, flags=re.I, what=''):
        self.id = cid
        self.rx = re.compile(rx, flags)
        self.anchors = [a.lower() for a in anchors]
        self.flt = flt
        self._sev = sev
        self.sample = sample
        self.what = what

    def sev(self, mode, path):
        s = self._sev.get(mode)
        if callable(s):
            return s(path)
        return s


ALL_HARD = {'tree': 'HARD', 'diff': 'HARD', 'messages': 'HARD', 'names': 'HARD'}
CONTENT_HARD = {'tree': 'HARD', 'diff': 'HARD', 'messages': 'HARD', 'names': None}


def _sev_tilde(path):
    return 'HARD' if in_results_tree(path) else 'REPORT'


def _sev_overlay(path):
    return 'HARD' if path.startswith(('.github/', 'tools/')) else 'REPORT'


def _sev_dash(path):
    return 'HARD' if is_guard_file(path) else 'REPORT'


STANDIN_USER = 'qz' + 'rkv'
STANDIN_HOST = 'labrig' + '-42'
STANDIN_WORD = 'labrig' + '42'


def _addr(*octets):
    return '.'.join(str(o) for o in octets)


def build_classes(neuter=None):
    """The class table. `neuter` is a self-test seam: that class gets a regex that cannot match."""
    out = []

    def add(cid, rx, anchors, flt, sev, sample, flags=re.I, what=''):
        if cid == neuter:
            rx = r'(?!x)x'
        out.append(Cls(cid, rx, anchors, flt, sev, sample, flags, what))

    add('home-mac', re.escape(P_MAC) + SEG + '/', [P_MAC],
        lambda m, ln: not user_ok(m.group(1)), ALL_HARD,
        P_MAC + STANDIN_USER + '/src', what='a macOS home path')
    add('home-linux', r'(?<![A-Za-z0-9_.-])' + re.escape(P_LINUX) + SEG + '/?', [P_LINUX],
        lambda m, ln: not user_ok(m.group(1)), ALL_HARD,
        P_LINUX + STANDIN_USER + '/src', what='a Linux home path')
    add('home-win', r'[A-Za-z]:' + BSL * 2 + r'{1,2}Users' + BSL * 2 + r'{1,2}([^'
        + BSL * 2 + r'\s"\'<>|:]+)', [BSL + 'users'],
        lambda m, ln: not user_ok(m.group(1)), ALL_HARD,
        'C:' + BSL + 'Users' + BSL + STANDIN_USER + BSL + 'src', what='a Windows home path')
    add('home-tilde', r'(?<![A-Za-z0-9_/.' + TILDE + r'-])' + TILDE + r'/([A-Za-z0-9_]'
        + SEG[1:-2] + '*)', [TILDE + '/'],
        lambda m, ln: not user_ok(m.group(1)),
        {'tree': _sev_tilde, 'diff': _sev_tilde, 'messages': 'HARD', 'names': None},
        'cd ' + TILDE + '/' + STANDIN_USER + '-notes/run', what='a path under the home shorthand')
    add('temp-root', re.escape(P_TEMP) + r'[A-Za-z0-9_+]{2}/[A-Za-z0-9_+]{6,}/', [P_TEMP],
        None, ALL_HARD, P_TEMP + 'ab/cdefgh123456/T/', what='a per-user temp root')

    lo, hi = _addr(100, 64, 0, 0), _addr(100, 127, 255, 255)

    def cgnat_hit(m, ln):
        a = m.group(0)
        if a in (lo, hi):
            return False
        return not _network_definition(ln, m.end(), a)

    add('cgnat-addr', NB_L + r'100\.(?:6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.' + OCT + r'\.' + OCT
        + NB_R, ['100.'], cgnat_hit, ALL_HARD, 'peer ' + _addr(100, 64, 0, 1) + ' up',
        what='an address in the RFC 6598 shared range')
    suffix = DNS_LABEL + r'\.' + DNS_TLD
    add('overlay-dns', r'(?:(?<![A-Za-z0-9-])(?:[a-z0-9-]+\.)+|\.)' + suffix + r'(?![A-Za-z0-9-])',
        ['.' + DNS_LABEL + '.' + DNS_TLD], None, ALL_HARD,
        'ssh host.' + STANDIN_USER + '.' + DNS_LABEL + '.' + DNS_TLD,
        what="the overlay vendor's DNS suffix")

    def lan_hit(m, ln):
        a = m.group(0)
        if a in LAN_EXAMPLES:
            return False
        return not _network_definition(ln, m.end(), a)

    add('lan-addr', NB_L + r'(?:10\.' + OCT + r'|192\.168|172\.(?:1[6-9]|2\d|3[01]))\.' + OCT
        + r'\.' + OCT + NB_R, ['10.', '192.168.', '172.'], lan_hit, ALL_HARD,
        'robot at ' + _addr(10, 77, 13, 9), what='an RFC 1918 address outside the example set')

    def login_hit(m, ln):
        user = m.group(1) or m.group(3)
        host = m.group(2) or m.group(4)
        if user.lower() == 'git':
            return False
        return not (user_ok(user) and host_ok(host))

    add('login-at-host',
        r'(?:\b(?:ssh|scp|rsync|sftp|mosh|ssh-copy-id)\b[^\n|;&]*?\s)([a-z_][a-z0-9_.-]*)@'
        r'([a-z0-9][a-z0-9_.-]*)'
        r'|(?<![A-Za-z0-9_.+-])([a-z_][a-z0-9_-]*)@([a-z0-9][a-z0-9.-]*)(?=:[' + TILDE + r'/])',
        ['@'], login_hit, ALL_HARD, 'ssh ' + STANDIN_USER + '@' + STANDIN_HOST,
        what='a login at a host')

    def mdns_hit(m, ln):
        lab = m.group(1).lower()
        if lab in PH_HOST or lab.startswith(('example', 'your', 'my', 'robot-', 'box-', 'mac-')):
            return False
        ctx = ln[max(0, m.start() - 12):m.start()].lower()
        hostish = '-' in lab or any(ch.isdigit() for ch in lab)
        locator = ctx.endswith(('tcp/', 'udp/', '://', '@', 'ssh ', 'ping ', 'robot ',
                                'connect '))
        return hostish or locator

    add('mdns-local', r'(?<![A-Za-z0-9_<{$.-])([a-z0-9][a-z0-9-]*)\.local(?![A-Za-z0-9_(.-])',
        ['.local'], mdns_hit, ALL_HARD, 'tcp/' + STANDIN_HOST + '.local:7447',
        what='a multicast DNS host name')

    def email_hit(m, ln):
        local, domain = m.group(1).lower(), m.group(2).lower()
        if domain == PROJECT_DOMAIN:
            return not (local in ROLE_LOCAL or local.startswith(ROLE_PREFIXES))
        return True

    add('email-personal',
        r'([A-Za-z0-9._%+-]+)@((?:' + '|'.join(CONSUMER_MAIL)
        + r')\.(?:com|me|net|org|co\.[a-z]{2}|ch|de)|' + re.escape(PROJECT_DOMAIN) + r')\b',
        ['@'], email_hit, CONTENT_HARD, STANDIN_USER + '@' + 'gmail' + '.com',
        what='a personal mail address, or a non-role address at the project domain')

    def hostfield_hit(m, ln):
        v = m.group(3).lower()
        if v in ('linux', 'darwin', 'freebsd', 'windows'):
            return False
        return v not in PH_HOST and not v.startswith(('box-', 'mac-', 'example', 'robot-'))

    add('host-field',
        r'(?:"(uname|nodename|hostname|host_name|machine_name|computer_name)"\s*:\s*"'
        r'|(?-i:\b(uname|nodename)\b)\s*[:=]\s*"?)(?:(?:Linux|Darwin|FreeBSD)\s+)?'
        r'([A-Za-z0-9][A-Za-z0-9._-]*)',
        ['uname', 'nodename', 'hostname', 'host_name', 'machine_name', 'computer_name'],
        hostfield_hit, CONTENT_HARD, '"nodename": "' + STANDIN_HOST + '"',
        what='a host identity field with a value')
    add('acl-tag', r'(?<![A-Za-z0-9_.:/-])tag:[a-z][a-z0-9]*(?:-[a-z0-9]+)+(?![A-Za-z0-9_])',
        ['tag:'], None, CONTENT_HARD, 'owner tag' + ':' + 'lab-runner', flags=0,
        what='an overlay access control tag')
    add('overlay-word', '|'.join(OVERLAY_WORDS), list(OVERLAY_WORDS), None,
        {'tree': _sev_overlay, 'diff': _sev_overlay, 'messages': 'REPORT', 'names': 'REPORT'},
        'join the ' + OVERLAY_WORDS[1] + ' first', what='an overlay network product word')
    add('style-dash', '[' + DASH_EN + DASH_EM + ']|' + re.escape(DASH_ESCAPE) + '[34]',
        [DASH_EN, DASH_EM, DASH_ESCAPE], None,
        {'tree': _sev_dash, 'diff': _sev_dash, 'messages': 'HARD', 'names': 'REPORT'},
        'range 3' + DASH_EN + '5', what='an en dash or an em dash')
    return out


# Classes with no regex: they are raised by structure, not by a line shape.
STRUCT_CLASSES = (
    ('identity-email', 'HARD', 'an author or committer address outside the identity policy'),
    ('media-meta', 'HARD', 'identifying metadata inside a media container'),
    ('media-time', 'REPORT', 'a timestamp inside a media container'),
    ('media-unparsed', 'HARD', 'a media, document or archive container no walker can read'),
)
CLASS_FLOOR = 15


# ---------------------------------------------------------------------------
# Private patterns.
# ---------------------------------------------------------------------------
class PrivatePattern(object):
    __slots__ = ('index', 'tag', 'rx', 'anchor')

    def __init__(self, index, tag, rx, anchor):
        self.index, self.tag, self.rx, self.anchor = index, tag, rx, anchor

    def label(self):
        return 'private#%d%s' % (self.index, ('@' + self.tag) if self.tag else '')

    def allow_class(self):
        return ('private@' + self.tag) if self.tag else None


def _expand_word(lit):
    """word:<literal> to (regex source, anchor). Separator tolerant at letter/digit seams."""
    if re.fullmatch(r'[0-9.]+', lit):
        return NB_L + re.escape(lit) + NB_R, lit.lower()
    tokens = re.findall(r'[A-Za-z]+|[0-9]+|[-_. ]+|.', lit, re.S)
    parts, runs, prev = [], [], None
    for tok in tokens:
        if re.fullmatch(r'[-_. ]+', tok):
            kind = 'sep'
            if prev != 'sep':
                parts.append('[-_. ]?')
        elif tok.isalpha() and tok.isascii():
            kind = 'alpha'
            if prev == 'digit':
                parts.append('[-_. ]?')
            parts.append(re.escape(tok))
            runs.append(tok)
        elif tok.isdigit():
            kind = 'digit'
            if prev == 'alpha':
                parts.append('[-_. ]?')
            parts.append(re.escape(tok))
            runs.append(tok)
        else:
            kind = 'other'
            parts.append(re.escape(tok))
        prev = kind
    if not runs:
        return None, None
    anchor = max(runs, key=len).lower()
    return r'(?<![A-Za-z0-9])' + ''.join(parts) + r'(?![A-Za-z0-9])', anchor


INLINE_COMMENT_RX = re.compile(r'\s+#(?:\s|$).*')
TRAILING_TAG_RX = re.compile(r'\s+@(\S+)$')
PREFIX_RX = re.compile(r'(word|text):', re.I)
GLUED_TAG_RX = re.compile(r'@(?:' + '|'.join(TAGS) + r')$', re.I)
ACCEPTED_FORMS = ('accepted forms: a blank line; a line starting with #; word:<literal>, '
                  'text:<literal> or <regex>, then optionally whitespace and @<tag> (%s), then '
                  'optionally whitespace, # and a comment; LF line endings'
                  % ' '.join('@' + t for t in TAGS))


def _entry_fault(body):
    """Why `body` (the comment and the trailing tag already cut) is not an entry,
    or None. Every shape here used to LOAD as a pattern that could never match:
    the word: expander made the stray `@`, `#` or word a required literal, so the
    tier counted as LOADED(n) while the entry was dead. The reasons name no text."""
    if any(ord(ch) < 0x20 or ch == '\x7f' for ch in body):
        return 'a control character (a tab, say) inside the entry'
    if re.search(r'\s//', body):
        return 'a comment starts with #, not //'
    if re.search(r'\s#', body):
        return 'a comment needs a space after the #'
    if re.search(r'\s@', body):
        return 'the tag must be the last thing on the line, and there is one tag'
    if GLUED_TAG_RX.search(body):
        return 'the tag needs whitespace before it'
    tokens = body.split()
    if len(tokens) > 1 and tokens[-1].lower() in TAGS:
        return 'a tag starts with @'
    m = PREFIX_RX.match(body)
    if m and m.group(1) not in ('word', 'text'):
        return 'the prefix is lowercase, word: or text:'
    if m:
        rest = body[5:]
        if not rest:
            return 'nothing after the prefix'
        if rest[0].isspace():
            return 'no whitespace after the prefix'
        if m.group(1) == 'word' and ('@' in rest or '#' in rest):
            return 'a word: literal cannot contain @ or #'
    return None


def parse_private(text):
    """Parse private entries. Raises NoRun naming the INDEX, the LIST LINE and the
    accepted forms, never the entry.

    The grammar is strict: a line is blank, a comment (`#` first), or exactly one
    entry, `word:<literal>`, `text:<literal>` or `<regex>`, then optionally
    whitespace and `@<tag>` from the vocabulary, then optionally whitespace, `#`
    and a comment. A byte order mark is dropped. Anything else refuses the load:
    a tolerant parser turned every ordinary misspelling into an entry that
    loaded, counted as LOADED and matched nothing."""
    out = []
    index = 0
    for lineno, raw in enumerate(text.lstrip('\ufeff').split('\n'), 1):
        line = raw.strip().lstrip('\ufeff')
        if not line or line.startswith('#'):
            continue
        index += 1
        if '\r' in raw:
            raise NoRun('private pattern #%d is malformed (a carriage return; Windows line '
                        'endings); list line %d; %s' % (index, lineno, ACCEPTED_FORMS))
        line = INLINE_COMMENT_RX.sub('', line).rstrip()
        tag = None
        m = TRAILING_TAG_RX.search(line)
        if m:
            if m.group(1).lower() not in TAGS:
                raise NoRun('private pattern #%d ends in a tag outside the vocabulary (%s); '
                            'list line %d' % (index, ' '.join('@' + t for t in TAGS), lineno))
            tag = m.group(1).lower()
            line = line[:m.start()].rstrip()
        fault = _entry_fault(line)
        if fault:
            raise NoRun('private pattern #%d is malformed (%s); list line %d; %s'
                        % (index, fault, lineno, ACCEPTED_FORMS))
        try:
            if line.startswith('word:'):
                src, anchor = _expand_word(line[5:])
                if src is None:
                    raise re.error('empty word')
                rx = re.compile(src, re.I)
            elif line.startswith('text:'):
                lit = line[5:]
                rx, anchor = re.compile(re.escape(lit), re.I), lit.lower()
            else:
                if '[[:' in line:
                    raise re.error('POSIX class')
                rx, anchor = re.compile(line, re.I | re.M), None
                if rx.search('') is not None:
                    raise re.error('matches empty')
        except (re.error, OverflowError, RecursionError):
            raise NoRun('private pattern #%d does not compile; list line %d' % (index, lineno))
        out.append(PrivatePattern(index, tag, rx, anchor))
    return out


def load_private(env, home):
    """Returns (patterns, kind, warnings). First source that yields a pattern wins."""
    warnings = []
    text = env.get('LEAK_PATTERNS', '')
    if text.strip():
        pats = parse_private(text)
        if pats:
            return pats, 'env', warnings
    candidates = []
    if env.get('LEAK_PATTERNS_FILE'):
        candidates.append((env['LEAK_PATTERNS_FILE'], 'file-env'))
    if home:
        candidates.append((os.path.join(home, '.config', 'cerulion', 'leak-patterns.txt'),
                           'default-file'))
    for path, kind in candidates:
        try:
            st = os.stat(path)
            with open(path, 'r', encoding='utf-8') as fh:
                body = fh.read()
        except OSError:
            if kind == 'file-env':
                raise NoRun('the private pattern file named by the environment is unreadable')
            continue
        except UnicodeDecodeError:
            raise NoRun('the private pattern file (%s) is not UTF-8' % kind)
        if stat.S_IMODE(st.st_mode) & 0o077:
            warnings.append('private pattern file (%s) is readable by others; chmod 600 it'
                            % kind)
        pats = parse_private(body)
        if pats:
            return pats, kind, warnings
    return [], None, warnings


class PrivateSet(object):
    def __init__(self, patterns):
        self.patterns = list(patterns)
        self.fast = [p for p in self.patterns if p.anchor]
        self.slow = [p for p in self.patterns if not p.anchor]
        self.combined = None
        if self.slow:
            try:
                self.combined = re.compile(
                    '|'.join('(?:%s)' % p.rx.pattern for p in self.slow), re.I | re.M)
            except re.error:
                self.combined = None

    def __len__(self):
        return len(self.patterns)

    def touches(self, text):
        """True when any private pattern matches `text` in ANY view of it."""
        for v in text_views(text):
            if any(p.rx.search(v) for p in self.patterns):
                return True
        return False

    def _view_touch(self, text):
        """The first pattern that matches a NORMALISED view of `text` (not the raw
        text): such a match has no span in the raw text, so the whole of `text`
        is withheld rather than a guessed part of it."""
        for v in text_views(text)[1:]:
            for p in self.patterns:
                if p.rx.search(v):
                    return p
        return None

    def mask(self, text):
        """`text` with every private match replaced by the pattern's label. Spans
        come from EVERY pattern on the raw text and are merged before anything is
        replaced, so the order of the entries cannot leave the tail of a longer
        entry in clear; a match found only in a normalised view withholds the
        whole string."""
        label = self.view_label(text)
        if label is not None:
            return label
        return mask_spans(text, self._spans(text))

    def view_label(self, text):
        """The label that stands for the WHOLE of `text` when a pattern matches only
        a normalised view of it (such a match has no span in the raw text), else
        None."""
        p = self._view_touch(text)
        return None if p is None else '<%s>' % p.label()

    def path_spans(self, path):
        """Every private match in `path` as (start, end, label): the spans on the raw
        text, plus each whole component a normalised view of which matches, so a
        match found only in a view withholds that component and keeps the rest of
        the location."""
        spans = self._spans(path)
        pos = 0
        for comp in path.split('/'):
            if comp:
                label = self.view_label(comp)
                if label is not None:
                    spans.append((pos, pos + len(comp), label))
            pos += len(comp) + 1
        return spans

    def mask_path(self, path):
        """A path masked per component; a match found only in a view of the whole
        path (and of no one component) withholds the whole location."""
        spans = self.path_spans(path)
        if not spans:
            return self.view_label(path) or path
        return mask_spans(path, spans)

    def _spans(self, text):
        spans = []
        for p in self.patterns:
            for m in p.rx.finditer(text):
                if m.end() > m.start():
                    spans.append((m.start(), m.end(), '<%s>' % p.label()))
        return spans


def merge_spans(spans):
    """(start, end, tag) spans sorted and merged where they touch or overlap. The
    tag of the leftmost survives; where two start together the longer wins, then
    the one added first (a private label ahead of a generic one)."""
    out = []
    for start, end, tag in sorted(spans, key=lambda t: (t[0], -t[1])):
        if out and start <= out[-1][1]:
            out[-1][1] = max(out[-1][1], end)
        else:
            out.append([start, end, tag])
    return out


def mask_spans(text, spans):
    """`text` with every (start, end, label) span replaced by its label. Spans are
    merged before anything is replaced, so the order of the entries cannot leave
    the tail of a longer one in clear."""
    if not spans:
        return text
    out = []
    pos = 0
    for start, end, label in merge_spans(spans):
        out.append(text[pos:start])
        out.append(label)
        pos = end
    out.append(text[pos:])
    return ''.join(out)


# The generic twin of the private path masker: wherever an identity-bearing
# VALUE prints as a masked shape, a value inside a file or directory NAME prints
# as <class:shape> in its place. The location beside a masked value must not
# carry that value, and a CI log keeps the location after the branch is scrubbed.
def identity_label(cid, text):
    return '<%s:%s>' % (cid, shape_of(text))


def identity_spans(classes, text):
    """Every identity-bearing generic match in `text` as (start, end, class id): the
    classes, regexes and filters the name scan runs, over the same text. Two
    adjacent matches can share their boundary character (two home paths share
    a slash), so the search resumes ON the last character of a match, not after
    it; a sub-match found that way sits inside the match before it and merges."""
    spans = []
    low = text.lower()
    for c in classes:
        if c.id not in IDENTITY_CLASSES or not any(a in low for a in c.anchors):
            continue
        pos = 0
        while pos <= len(text):
            m = c.rx.search(text, pos)
            if m is None:
                break
            if m.end() > m.start() and (c.flt is None or c.flt(m, text)):
                spans.append((m.start(), m.end(), c.id))
            pos = max(m.end() - 1, m.start() + 1)
    return spans


def identity_path_spans(classes, path):
    """Returns (spans, withheld). `spans` are the (start, end, label) triples to mask
    in `path`: every match on the raw path, plus each whole component a normalised
    view of which matches (such a match has no span in the raw text), as
    `PrivateSet.path_spans` does for the tier. `withheld` is a label for the WHOLE
    path when a normalised view of the path holds more matches of some class than
    the raw spans and the components account for (a home path whose separators
    are encoded, so no one component holds it), else None."""
    raw = identity_spans(classes, path)
    spans = [(s, e, identity_label(cid, path[s:e])) for s, e, cid in merge_spans(raw)]
    seen = collections.Counter(cid for _, _, cid in raw)
    pos = 0
    for comp in path.split('/'):
        if comp:
            for v in text_views(comp)[1:]:
                found = identity_spans(classes, v)
                if found:
                    spans.append((pos, pos + len(comp), identity_label(found[0][2], comp)))
                    seen.update(cid for _, _, cid in found)
                    break
        pos += len(comp) + 1
    for v in text_views(path)[1:]:
        extra = collections.Counter(cid for _, _, cid in identity_spans(classes, v)) - seen
        if extra:
            return spans, identity_label(sorted(extra)[0], path)
    return spans, None


# ---------------------------------------------------------------------------
# Allowlist and pragma.
# ---------------------------------------------------------------------------
def glob_to_rx(glob):
    out, i = [], 0
    while i < len(glob):
        if glob.startswith('**/', i):
            out.append('(?:.*/)?')
            i += 3
        elif glob.startswith('**', i):
            out.append('.*')
            i += 2
        elif glob[i] == '*':
            out.append('[^/]*')
            i += 1
        elif glob[i] == '?':
            out.append('[^/]')
            i += 1
        else:
            out.append(re.escape(glob[i]))
            i += 1
    return re.compile(''.join(out) + r'\Z')


class AllowEntry(object):
    __slots__ = ('glob', 'rx', 'cls', 'reason', 'lineno', 'matched_files', 'suppressed')

    def __init__(self, glob, cls, reason, lineno):
        self.glob, self.rx, self.cls, self.reason, self.lineno = (
            glob, glob_to_rx(glob), cls, reason, lineno)
        self.matched_files = 0
        self.suppressed = 0


def parse_allow(text, class_ids, hard_ids):
    """Parse `glob | class | reason` lines. Raises Usage on a malformed entry."""
    entries = []
    for lineno, raw in enumerate(text.split('\n'), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        fields = [f.strip() for f in line.split('|')]
        if len(fields) != 3:
            raise Usage('allowlist line %d: want exactly `glob | class | reason`' % lineno)
        glob, cls, reason = fields
        if cls.startswith('private'):
            if not (cls.startswith('private@') and cls[8:] in TAGS):
                raise Usage('allowlist line %d: a private waiver must name a category '
                            '(private@<tag>); a bare private waiver is refused' % lineno)
        elif cls not in class_ids:
            raise Usage('allowlist line %d: unknown class %s' % (lineno, cls))
        if len(reason) < 12:
            raise Usage('allowlist line %d: the reason must be at least 12 characters' % lineno)
        if not glob:
            raise Usage('allowlist line %d: empty glob' % lineno)
        if glob.strip('*/') == '' and (cls in hard_ids or cls.startswith('private')):
            raise Usage('allowlist line %d: a match-everything glob is refused' % lineno)
        entries.append(AllowEntry(glob, cls, reason, lineno))
    return entries


# ---------------------------------------------------------------------------
# Report and output contract.
# ---------------------------------------------------------------------------
class Hit(object):
    __slots__ = ('cls', 'sev', 'path', 'line', 'text', 'where', 'private', 'ctx')

    def __init__(self, cls, sev, path, line, text, where, private, ctx=None):
        self.cls, self.sev, self.path, self.line = cls, sev, path, line
        self.text, self.where, self.private, self.ctx = text, where, private, ctx


class Stats(object):
    def __init__(self):
        self.files = self.text = self.binary = self.symlink = 0
        self.units = self.json_leaves = self.json_unparsed = 0
        self.oversize = self.skipped = self.pragmas = self.pragmas_refused = 0
        self.commits = self.media = 0


def gh_escape(s, prop=False):
    s = s.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')
    if prop:
        s = s.replace(':', '%3A').replace(',', '%2C')
    return s


class Scanner(object):
    """One scan. The sweep and the self-test both go through this class."""

    def __init__(self, mode, classes, private, allow, out, fmt='text', hard=(), quiet=False):
        self.mode = mode
        self.classes = classes
        self.private = private
        self.allow = allow
        self.out = out
        self.fmt = fmt
        self.hard = set(hard)
        self.quiet = quiet
        self.hits = []
        self.seen = set()
        self.stats = Stats()
        self.suppressed = 0
        self.all_paths = []
        self.media_kinds = None
        self.sevmode = 'tree' if mode == 'media' else mode
        self.listed = False
        # Every (path, line) a private pattern touched, recorded BEFORE the
        # allowlist and the duplicate check: a generic hit on such a line prints
        # no matched text whatever else happened to the private hit.
        self.private_lines = set()

    # -- recording ---------------------------------------------------------
    def _allowed(self, path, allow_cls):
        if not allow_cls:
            return False
        ok = False
        for e in self.allow:
            if e.cls == allow_cls and e.rx.match(path):
                e.suppressed += 1
                ok = True
        return ok

    def _pragma(self, raw_line, cid):
        m = PRAGMA_RX.search(raw_line)
        if not m or self.mode == 'messages':
            return False
        if m.group(1) != cid:
            return False
        if cid.startswith('private') or len(m.group(2).strip()) < 12:
            self.stats.pragmas_refused += 1
            return False
        self.stats.pragmas += 1
        return True

    def add(self, cid, sev, path, line, text, where='', private=None, raw_line='',
            ctx=None):
        if cid in self.hard:
            sev = 'HARD'
        if private is not None:
            self.private_lines.add((path, line))
        key = (cid, path, line, where)
        if key in self.seen:
            return
        self.seen.add(key)
        if self._allowed(path, private.allow_class() if private is not None else cid):
            self.suppressed += 1
            return
        if raw_line and self._pragma(raw_line, cid):
            return
        self.hits.append(Hit(cid, sev, path, line, text, where, private, ctx))

    # -- the one line scanner -------------------------------------------------
    def scan_unit(self, path, text, line_base=1, where='', sevpath=None, deep=True,
                  skip_private_tags=()):
        """Scan one multi-line unit. Line numbers are line_base + index."""
        sevpath = sevpath if sevpath is not None else path
        raw_lines = None
        self.stats.units += text.count('\n') + 1
        views = text_views(text) if deep else [text]
        for v in views:
            low = v.lower()
            starts = None
            lines = None
            for c in self.classes:
                sev = c.sev(self.sevmode, sevpath)
                if sev is None:
                    continue
                cand = None
                for a in c.anchors:
                    i = low.find(a)
                    while i != -1:
                        if starts is None:
                            starts = _line_starts(low)
                        li = bisect.bisect_right(starts, i) - 1
                        if cand is None:
                            cand = set()
                        cand.add(li)
                        if li + 1 >= len(starts):
                            break
                        i = low.find(a, starts[li + 1])
                if not cand:
                    continue
                if lines is None:
                    lines = v.split('\n')
                for li in sorted(cand):
                    line = lines[li]
                    for m in c.rx.finditer(line):
                        if c.flt is not None and not c.flt(m, line):
                            continue
                        if raw_lines is None:
                            raw_lines = text.split('\n')
                        raw_line = raw_lines[li] if li < len(raw_lines) else ''
                        self.add(c.id, sev, path, line_base + li, m.group(0), where,
                                 raw_line=raw_line, ctx=(raw_line, line))
                        break
            if self.private is None:
                continue
            for p in self.private.fast:
                if p.tag in skip_private_tags:
                    continue
                i = low.find(p.anchor)
                while i != -1:
                    if starts is None:
                        starts = _line_starts(low)
                    if lines is None:
                        lines = v.split('\n')
                    li = bisect.bisect_right(starts, i) - 1
                    if p.rx.search(lines[li]):
                        if raw_lines is None:
                            raw_lines = text.split('\n')
                        self.add(p.label(), 'HARD', path, line_base + li, '', where, private=p,
                                 raw_line=raw_lines[li] if li < len(raw_lines) else '')
                    if li + 1 >= len(starts):
                        break
                    i = low.find(p.anchor, starts[li + 1])
            if self.private.slow:
                self._scan_slow(path, v, text, line_base, where, skip_private_tags)

    def _scan_slow(self, path, v, text, line_base, where, skip_private_tags=()):
        ps = self.private
        starts = None
        lines = None
        raw_lines = None

        def record(p, li):
            nonlocal raw_lines
            if raw_lines is None:
                raw_lines = text.split('\n')
            self.add(p.label(), 'HARD', path, line_base + li, '', where, private=p,
                     raw_line=raw_lines[li] if li < len(raw_lines) else '')

        if ps.combined is not None and not skip_private_tags:
            done = set()
            for m in ps.combined.finditer(v):
                if starts is None:
                    starts = _line_starts(v)
                    lines = v.split('\n')
                li = bisect.bisect_right(starts, m.start()) - 1
                if li in done:
                    continue
                done.add(li)
                for p in ps.slow:
                    if p.rx.search(lines[li]):
                        record(p, li)
            return
        for p in ps.slow:
            if p.tag in skip_private_tags:
                continue
            for m in p.rx.finditer(v):
                if starts is None:
                    starts = _line_starts(v)
                li = bisect.bisect_right(starts, m.start()) - 1
                record(p, li)

    # -- output ----------------------------------------------------------------
    def redact(self, text):
        return self.private.mask(text) if self.private is not None else text

    def redact_path(self, path):
        """A location as it may be PRINTED. A private match is replaced by its label,
        and wherever a generic identity-bearing VALUE prints as a masked shape
        (`masks_values`) such a value inside the path is masked the same way, so
        the location beside a masked value cannot carry it through a file or
        directory name. Both span sets come from the RAW path and are merged
        before anything is replaced: replacing the private match first can break
        the generic shape around it (a login before a private host) and leave the
        rest in clear. A match found only in a normalised view of one component
        withholds that component; one found only in a view of the whole path
        withholds the whole location."""
        spans = []
        if self.private is not None:
            spans += self.private.path_spans(path)
        if self.masks_values():
            generic, withheld = identity_path_spans(self.classes, path)
            if withheld is not None:
                return withheld
            spans += generic
        if spans:
            return mask_spans(path, spans)
        if self.private is not None:
            return self.private.view_label(path) or path
        return path

    def masks_values(self):
        """True when an identity-bearing generic hit prints a masked shape instead
        of its value: no private tier is loaded (the main workflow's lint step,
        a fork pull request), or the output is a CI log (github format)."""
        return self.private is None or self.fmt == 'github'

    def _private_touches(self, h):
        """True when a private pattern touched the line a generic hit sits on, in
        any view of it. Decided on the LINE, never on the extracted match: a
        private name can sit half inside the match, be split by an escape the
        generic class ignored, or be shadowed by a shorter entry."""
        if self.private is None:
            return False
        if (h.path, h.line) in self.private_lines:
            return True
        if h.ctx is None:
            return False
        raw_line, view_line = h.ctx
        return self.private.touches(raw_line) or self.private.touches(view_line)

    def emit_hits(self):
        hard = [h for h in self.hits if h.sev == 'HARD']
        report = [h for h in self.hits if h.sev != 'HARD']
        for h in hard:
            self.out(self._format(h))
        if not self.quiet:
            for h in report[:MAX_REPORT_LINES]:
                self.out(self._format(h))
            if len(report) > MAX_REPORT_LINES:
                self.out('REPORT ... %d more report-only line(s) not shown'
                         % (len(report) - MAX_REPORT_LINES))
        return len(hard), len(report)

    def _format(self, h):
        path = self.redact_path(h.path)
        path_masked = path != h.path
        loc = '%s:%d' % (clean(path), h.line)
        word = 'HIT' if h.sev == 'HARD' else 'REPORT'
        gh = self.fmt == 'github' and h.sev == 'HARD'
        if h.private is not None:
            tail = (' [%s]' % h.where) if h.where in ('name', 'binary', 'symlink', 'metadata',
                                                      'json', 'nested') else ''
            if gh:
                return self._gh_error(h, path_masked, loc, 'private pattern #%d'
                                      % h.private.index, tail if path_masked else '')
            return '%s %s %s%s' % (word, h.cls, loc, tail)
        masked = h.cls in IDENTITY_CLASSES and self.masks_values()
        if self._private_touches(h):
            text = WITHHELD
        elif masked:
            text = mask_shape(h.text)
        else:
            text = clean(self.redact(h.text))[:80]
        where = h.where
        if self.masks_values() and where.startswith('json '):
            # An unlocated JSON leaf's key path can itself be a name, so the
            # context decides whether it prints, not the class of the hit that
            # happened to land on the leaf.
            where = 'json'
        tail = (' [%s]' % clean(self.redact(where))) if where else ''
        if gh:
            return self._gh_error(h, path_masked, loc, h.cls, ': %s%s' % (text, tail))
        return '%s %s %s: %s%s' % (word, h.cls, loc, text, tail)

    def _gh_error(self, h, path_masked, loc, head, rest):
        """One workflow error command. The forge anchors the file property to the
        file, so the property carries the path VERBATIM (escaped), and only when
        nothing in it had to be masked: a masked location names no file and rides
        the message instead, so the log still says where without saying what."""
        if path_masked:
            return '::error ::' + gh_escape('%s %s%s' % (head, loc, rest))
        return '::error file=%s,line=%d::%s' % (gh_escape(h.path, True), max(h.line, 1),
                                                 gh_escape(head + rest))


def _safe_chr(code):
    return ' ' if code < 0x20 or 0xD800 <= code <= 0xDFFF else chr(code)


def _line_starts(text):
    starts = [0]
    i = text.find('\n')
    while i != -1:
        starts.append(i + 1)
        i = text.find('\n', i + 1)
    return starts


# ---------------------------------------------------------------------------
# Git access. Every return code is read; a failure is NoRun, never a zero.
# ---------------------------------------------------------------------------
class Git(object):
    def __init__(self, root, env):
        self.root = root
        self.env = env

    def run(self, args, input_bytes=None, ok_codes=(0,)):
        try:
            p = subprocess.run(['git'] + list(args), cwd=self.root, env=self.env,
                               input=input_bytes, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
        except OSError:
            raise NoRun('git is not available')
        if p.returncode not in ok_codes:
            raise NoRun('git %s failed (rc %d)' % (args[0], p.returncode))
        return p.stdout

    def text(self, args):
        return self.run(args).decode('utf-8', 'replace')

    def commit_exists(self, rev):
        p = subprocess.run(['git', 'cat-file', '-e', rev + '^{commit}'], cwd=self.root,
                           env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return p.returncode == 0

    def check_range(self, rng):
        if '..' not in rng:
            raise Usage('a range must be spelled A..B')
        a, b = rng.split('..', 1)
        a, b = a.strip('.'), b.strip('.')
        if not a or not b:
            raise Usage('a range must be spelled A..B')
        for r in (a, b):
            if not self.commit_exists(r):
                raise NoRun('range end %s is not a commit that exists here' % r)
        return a, b

    def merge_base(self, a, b):
        p = subprocess.run(['git', 'merge-base', a, b], cwd=self.root, env=self.env,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p.returncode != 0:
            return None
        return p.stdout.decode().strip()

    def blobs(self, shas):
        """Read many blobs through one cat-file process. Returns {sha: bytes}."""
        want = [s for s in dict.fromkeys(shas)]
        if not want:
            return {}
        data = self.run(['cat-file', '--batch'], input_bytes=('\n'.join(want) + '\n').encode())
        out = {}
        pos = 0
        for sha in want:
            nl = data.find(b'\n', pos)
            if nl == -1:
                raise NoRun('git cat-file output ended early')
            header = data[pos:nl].decode('utf-8', 'replace').split()
            pos = nl + 1
            if len(header) == 2 and header[1] == 'missing':
                raise NoRun('git object missing')
            size = int(header[2])
            out[sha] = data[pos:pos + size]
            pos += size + 1
        return out


def split_list(data):
    """A NUL or newline separated list of paths."""
    if isinstance(data, bytes):
        data = data.decode('utf-8', 'replace')
    if '\0' in data:
        items = data.split('\0')
    else:
        items = data.split('\n')
    return [i for i in items if i.strip()]


def unquote_git_path(p):
    if len(p) >= 2 and p[0] == '"' and p[-1] == '"':
        body = p[1:-1]
        try:
            return body.encode('latin-1').decode('unicode_escape').encode('latin-1').decode(
                'utf-8', 'replace')
        except (UnicodeDecodeError, UnicodeEncodeError):
            return body
    return p


class Entry(object):
    """One thing to scan: a path plus a way to get its bytes."""
    __slots__ = ('path', 'kind', 'sha', 'target')

    def __init__(self, path, kind, sha=None, target=None):
        self.path, self.kind, self.sha, self.target = path, kind, sha, target


def enumerate_worktree(git, untracked, files_from=None):
    if files_from is not None:
        paths = files_from
    else:
        paths = split_list(git.run(['ls-files', '-z']))
        if untracked:
            paths += split_list(git.run(['ls-files', '-z', '--others', '--exclude-standard']))
    out = []
    for p in paths:
        full = os.path.join(git.root, p)
        if os.path.islink(full):
            out.append(Entry(p, 'symlink', target=os.readlink(full)))
        elif os.path.isfile(full):
            out.append(Entry(p, 'file'))
        elif os.path.isdir(full):
            continue
        else:
            out.append(Entry(p, 'missing'))
    return out


def enumerate_ref(git, rev):
    if not git.commit_exists(rev):
        raise NoRun('ref %s is not a commit that exists here' % rev)
    data = git.run(['ls-tree', '-r', '-z', rev])
    out = []
    for rec in data.split(b'\0'):
        if not rec:
            continue
        meta, path = rec.split(b'\t', 1)
        mode, typ, sha = meta.decode().split()
        p = path.decode('utf-8', 'replace')
        if typ == 'commit':
            continue
        out.append(Entry(p, 'symlink' if mode == '120000' else 'blob', sha=sha))
    return out


def enumerate_staged(git, files_from=None):
    data = git.run(['ls-files', '-s', '-z'])
    want = set(files_from) if files_from is not None else None
    out = []
    for rec in data.split(b'\0'):
        if not rec:
            continue
        meta, path = rec.split(b'\t', 1)
        mode, sha, _stage = meta.decode().split()
        p = path.decode('utf-8', 'replace')
        if want is not None and p not in want:
            continue
        if mode == '160000':
            continue
        out.append(Entry(p, 'symlink' if mode == '120000' else 'blob', sha=sha))
    return out


def load_entries(git, entries):
    """Yield (entry, bytes_or_None, symlink_target_or_None)."""
    shas = [e.sha for e in entries if e.sha]
    blobs = git.blobs(shas) if shas else {}
    for e in entries:
        if e.kind == 'missing':
            yield e, None, None
        elif e.kind == 'symlink':
            target = e.target if e.target is not None else blobs[e.sha].decode('utf-8',
                                                                                 'replace')
            yield e, None, target
        elif e.kind == 'blob':
            yield e, blobs[e.sha], None
        else:
            full = os.path.join(git.root, e.path)
            try:
                size = os.path.getsize(full)
                if size > TEXT_CAP:
                    yield e, full, None
                    continue
                with open(full, 'rb') as fh:
                    yield e, fh.read(), None
            except OSError:
                raise NoRun('unreadable file in the scan set')


# ---------------------------------------------------------------------------
# Surfaces for one blob: text with views, JSON leaves, data URIs, binary
# strings, container text.
# ---------------------------------------------------------------------------
ASCII_RUN = re.compile(rb'[\x20-\x7e]{6,}')
UTF16_RUN = re.compile(rb'(?:[\x20-\x7e]\x00){6,}')
JSON_EXT = ('.json', '.jsonl', '.ndjson')


def is_binary(data):
    return b'\0' in data[:8192]


def binary_strings(data):
    out = []
    for m in ASCII_RUN.finditer(data):
        out.append(m.group(0).decode('ascii'))
    for m in UTF16_RUN.finditer(data):
        out.append(m.group(0).decode('utf-16-le', 'replace'))
    return out


def inflate_capped(payload, cap=INFLATE_CAP):
    d = zlib.decompressobj()
    try:
        return d.decompress(payload, cap)
    except zlib.error:
        return b''


def png_chunk_text(typ, body):
    """The text of one PNG text chunk, inflated under a cap when compressed."""
    if typ == b'tEXt':
        return body.replace(b'\0', b': ').decode('latin-1')
    if typ == b'zTXt':
        key, _, rest = body.partition(b'\0')
        return key.decode('latin-1') + ': ' + inflate_capped(rest[1:]).decode('latin-1',
                                                                             'replace')
    if typ == b'iTXt':
        key, _, rest = body.partition(b'\0')
        flag = rest[:1]
        rest = rest[2:]
        _lang, _, rest = rest.partition(b'\0')
        _tkey, _, txt = rest.partition(b'\0')
        if flag == b'\1':
            txt = inflate_capped(txt)
        return key.decode('latin-1') + ': ' + txt.decode('utf-8', 'replace')
    return None


def png_texts(data):
    """Text chunks of a PNG, compressed ones inflated under a cap."""
    out = []
    pos = 8
    while pos + 8 <= len(data):
        ln, typ = struct.unpack('>I4s', data[pos:pos + 8])
        text = png_chunk_text(typ, data[pos + 8:pos + 8 + ln])
        if text is not None:
            out.append(text)
        pos += 12 + ln
        if typ == b'IEND':
            break
    return out


def scan_blob(sc, path, data, where=''):
    """Scan one blob's content on every surface it has."""
    if isinstance(data, str):
        # An oversize file: streamed in chunks, raw and ANSI views only.
        sc.stats.oversize += 1
        _scan_stream(sc, path, data)
        return
    if is_binary(data):
        if not where:
            sc.stats.binary += 1
        texts = binary_strings(data)
        if data[:8] == b'\x89PNG\r\n\x1a\n':
            texts += png_texts(data)
        if texts:
            sc.scan_unit(path, '\n'.join(texts), 0, where or 'binary')
        return
    if not where:
        sc.stats.text += 1
    text = data.decode('utf-8', 'replace')
    sc.scan_unit(path, text, 1, where)
    lower = path.lower()
    if lower.endswith(JSON_EXT):
        _scan_json(sc, path, text, lower)
    if 'base64,' in text:
        for m in DATA_URI_RX.finditer(text):
            b64 = re.sub(r'\s+', '', m.group(1))
            try:
                nested = base64.b64decode(b64[:NESTED_CAP * 4 // 3 + 4], validate=False)
            except (binascii.Error, ValueError):
                continue
            if nested:
                scan_blob(sc, path, nested, 'nested')


def _scan_stream(sc, path, full):
    line = 1
    with open(full, 'rb') as fh:
        carry = b''
        while True:
            chunk = fh.read(CHUNK)
            if not chunk:
                break
            buf = carry + chunk
            text = buf.decode('utf-8', 'replace')
            sc.scan_unit(path, text, max(1, line - carry.count(b'\n')), '')
            line += chunk.count(b'\n')
            carry = buf[-OVERLAP:]


def _json_walk(node, path, out):
    if isinstance(node, dict):
        for k, v in node.items():
            out.append((path + '.' + str(k), str(k), None))
            _json_walk(v, path + '.' + str(k), out)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            _json_walk(v, '%s[%d]' % (path, i), out)
    elif isinstance(node, str):
        key = path.rsplit('.', 1)[-1].split('[', 1)[0]
        out.append((path, node, key))


def _scan_json(sc, path, text, lower):
    docs = []
    try:
        if lower.endswith('.json'):
            docs.append(json.loads(text))
        else:
            for ln in text.split('\n'):
                if ln.strip():
                    docs.append(json.loads(ln))
    except (ValueError, RecursionError):
        sc.stats.json_unparsed += 1
        return
    leaves = []
    for d in docs:
        _json_walk(d, '$', leaves)
    sc.stats.json_leaves += len(leaves)
    dropped = False
    for jpath, value, key in leaves:
        unit = ('"%s": "%s"' % (key, value)) if key is not None else value
        before = len(sc.hits)
        sc.scan_unit(path, unit, 0, 'json ' + jpath)
        for h in sc.hits[before:]:
            # Locate the leaf in the raw text so the line number is real when it can be.
            line = _find_line(text, value)
            if line:
                key2 = (h.cls, path, line, '')
                if key2 in sc.seen:
                    h.sev = None
                    dropped = True
                    continue
                sc.seen.add(key2)
                h.line = line
                h.where = ''
            elif h.private is not None:
                h.where = 'json'
    if dropped:
        sc.hits = [h for h in sc.hits if h.sev is not None]


def _find_line(text, value):
    for cand in (json.dumps(value)[1:-1], json.dumps(value, ensure_ascii=False)[1:-1],
                 json.dumps(value)[1:-1].replace('/', BSL + '/')):
        i = text.find(cand)
        if i != -1:
            return text.count('\n', 0, i) + 1
    return 0


def scan_entries(sc, git, entries, sevmode_names=False):
    """Tree surfaces over a set of entries. Path names are line 0 of their own unit."""
    for e, data, target in load_entries(git, entries):
        sc.stats.files += 1
        if e.kind == 'missing':
            sc.stats.skipped += 1
            continue
        sc.all_paths.append(e.path)
        sc.scan_unit(e.path, e.path, 0, 'name')
        if target is not None:
            sc.stats.symlink += 1
            sc.scan_unit(e.path, target, 0, 'symlink')
            continue
        if sevmode_names:
            continue
        scan_blob(sc, e.path, data)


# ---------------------------------------------------------------------------
# Modes.
# ---------------------------------------------------------------------------
def run_tree(sc, git, args):
    files_from = _files_from(args)
    sc.listed = files_from is not None
    if args.ref:
        entries = enumerate_ref(git, args.ref)
        if files_from is not None:
            want = set(files_from)
            entries = [e for e in entries if e.path in want]
    elif args.staged:
        entries = enumerate_staged(git, files_from)
    else:
        entries = enumerate_worktree(git, args.untracked, files_from)
    if files_from is not None and (args.ref or args.staged):
        _note_unlisted(sc, files_from, entries)
    scan_entries(sc, git, entries)
    return files_from is None


def run_names(sc, git, args, env):
    if args.range:
        a, b = git.check_range(args.range)
        base = git.merge_base(a, b) or a
        data = git.run(['diff', '--name-status', '-z', '-M', '--diff-filter=AR', base, b])
        recs = [r for r in data.split(b'\0') if r]
        paths = []
        i = 0
        while i < len(recs):
            status = recs[i].decode('utf-8', 'replace')
            if status.startswith('R'):
                paths.append(recs[i + 2].decode('utf-8', 'replace'))
                i += 3
            else:
                paths.append(recs[i + 1].decode('utf-8', 'replace'))
                i += 2
        entries = [Entry(p, 'missing') for p in paths]
        for e in entries:
            sc.stats.files += 1
            sc.scan_unit(e.path, e.path, 0, 'name')
    else:
        if args.ref:
            entries = enumerate_ref(git, args.ref)
        elif args.staged:
            entries = enumerate_staged(git, getattr(args, 'staged_set', None))
        else:
            entries = enumerate_worktree(git, args.untracked)
        scan_entries(sc, git, entries, sevmode_names=True)
    branch = None
    if args.branch:
        branch = args.branch
    elif args.branch_env:
        branch = env.get(args.branch_env, '')
    if branch:
        sc.scan_unit('branch', branch, 0, 'name')
    return not args.range


def run_diff(sc, git, args):
    if args.staged:
        cmd = ['-c', 'core.quotepath=off', 'diff', '--cached', '-U0', '--no-color',
               '--no-ext-diff']
        newside = None
    else:
        a, b = git.check_range(args.range)
        base = git.merge_base(a, b) or a
        cmd = ['-c', 'core.quotepath=off', 'diff', '-U0', '--no-color', '--no-ext-diff', base,
               b]
        newside = b
    text = git.text(cmd)
    path = None
    line = 0
    added = {}
    binaries = []
    for raw in text.split('\n'):
        if raw.startswith('+++ '):
            p = raw[4:]
            if p == '/dev/null':
                path = None
            else:
                path = unquote_git_path(p[2:] if p.startswith('b/') else p)
                sc.stats.files += 1
                sc.scan_unit(path, path, 0, 'name')
            continue
        if raw.startswith('Binary files ') and raw.endswith(' differ'):
            m = re.match(r'Binary files (?:a/)?(.*?) and (?:b/)?(.*?) differ$', raw)
            if m and m.group(2) != '/dev/null':
                binaries.append(unquote_git_path(m.group(2)))
            continue
        if raw.startswith('@@'):
            m = re.match(r'@@ -\d+(?:,\d+)? \+(\d+)', raw)
            line = int(m.group(1)) if m else 1
            continue
        if path is None:
            continue
        if raw.startswith('+') and not raw.startswith('+++'):
            added.setdefault(path, []).append((line, raw[1:]))
            line += 1
    for p, items in added.items():
        # Consecutive added lines are joined so a multi-line view (ANSI, JSON) sees them together.
        run = []
        for ln, txt in items:
            if run and ln != run[-1][0] + 1:
                _flush_added(sc, p, run)
                run = []
            run.append((ln, txt))
        _flush_added(sc, p, run)
    for p in binaries:
        sc.stats.files += 1
        try:
            if newside is None:
                data = git.run(['show', ':' + p])
            else:
                data = git.run(['show', newside + ':' + p])
        except NoRun:
            continue
        scan_blob(sc, p, data)
    return False


def _flush_added(sc, path, run):
    if not run:
        return
    sc.scan_unit(path, '\n'.join(t for _, t in run), run[0][0], '')


def identity_ok(email, allow_emails):
    e = email.strip().lower()
    if e in allow_emails:
        return True
    if e.endswith('@' + FORGE_NOREPLY) or e == FORGE_WEBFLOW:
        return True
    if e.endswith('@' + PROJECT_DOMAIN):
        local = e.split('@', 1)[0]
        return local in ROLE_LOCAL or local.startswith(ROLE_PREFIXES)
    return False


SCISSORS_RX = re.compile(r'^\S{1,8} -{24} >8 -{24}$')


def strip_message_file(text, editor_used=False, cleanup='default'):
    """What git will KEEP of a message file, with the dropped lines blanked so
    line numbers stay real.

    Comment lines are dropped only when git will drop them: an editor was used
    (git tells the hook otherwise with GIT_EDITOR=:) and the cleanup mode is the
    default or `strip`. With `-m`, `-F`, or a `whitespace`, `verbatim` or
    `scissors` cleanup, a line starting with `#` is part of the commit and is
    scanned. Text below a scissors line is dropped only when git truncates there
    (an editor was used, or the cleanup mode is `scissors`) AND the block is
    git's own: the marker, then git's comment lines, then the diff that `-v`
    appends. A pasted scissors line followed by anything else is ordinary
    content, and so is the whole file of a `-m` or `-F` commit."""
    lines = text.split('\n')
    keep_comments = not editor_used or cleanup in ('whitespace', 'verbatim', 'scissors')
    cut = None
    for i, ln in enumerate(lines):
        if not (editor_used or cleanup == 'scissors') or not SCISSORS_RX.match(ln):
            continue
        mark = ln[:1]
        j = i + 1
        while j < len(lines) and lines[j].startswith(mark):
            j += 1
        if j > i + 1 and (j >= len(lines) or lines[j].startswith('diff --git ')):
            cut = i
            break
    out = []
    for i, ln in enumerate(lines):
        if cut is not None and i >= cut:
            out.append('')
        elif not keep_comments and ln.startswith('#'):
            out.append('')
        else:
            out.append(ln)
    return '\n'.join(out)


def run_messages(sc, git, args, env):
    allow_emails = set()
    if args.allow_email_file:
        try:
            with open(args.allow_email_file, 'r', encoding='utf-8') as fh:
                allow_emails = set(l.strip().lower() for l in fh if l.strip()
                                   and not l.startswith('#'))
        except OSError:
            raise NoRun('the allow-email file is unreadable')
    sources = 0

    def ident(label, name, email):
        # A commit's author and committer NAME is the person's forge identity and is
        # public by nature, so a private pattern tagged @person does not judge it; every
        # other pattern (a machine name, a login) and every generic class still does.
        sc.scan_unit(label + ' name', name, 1, '', skip_private_tags=('person',))
        sc.scan_unit(label + ' email', email, 1, '')
        if not identity_ok(email, allow_emails):
            sc.add('identity-email', 'HARD', label + ' email', 1,
                   'not a forge noreply, web-flow, project role or allow-listed address')

    if args.range:
        a, b = git.check_range(args.range)
        fmt = '%H%x00%an%x00%ae%x00%cn%x00%ce%x00%B%x01'
        data = git.text(['log', '--format=' + fmt, a + '..' + b])
        recs = [r for r in data.split('\x01') if r.strip()]
        if not recs and args.require_commits:
            raise NoRun('zero commits in the range on a pull request event')
        for r in recs:
            sha, an, ae, cn, ce, body = r.lstrip('\n').split('\x00', 5)
            sc.stats.commits += 1
            sources += 1
            label = 'commit:' + sha[:12]
            sc.scan_unit(label, body.rstrip('\n'), 1, '')
            ident(label + ' author', an, ae)
            ident(label + ' committer', cn, ce)
    if args.message_file:
        try:
            with open(args.message_file, 'r', encoding='utf-8', errors='replace') as fh:
                body = strip_message_file(fh.read(), getattr(args, 'editor_used', False),
                                          getattr(args, 'cleanup', 'default'))
        except OSError:
            raise NoRun('the message file is unreadable')
        sources += 1
        sc.scan_unit('message', body, 1, '')
    if args.ident_from_git:
        for label, var in (('author', 'GIT_AUTHOR_IDENT'), ('committer', 'GIT_COMMITTER_IDENT')):
            raw = git.text(['var', var]).strip()
            m = re.match(r'(.*?)\s*<([^>]*)>', raw)
            if m:
                sources += 1
                ident(label, m.group(1), m.group(2))
    for what, envname, fname in (('pr-title', args.pr_title_env, args.pr_title_file),
                                 ('pr-body', args.pr_body_env, args.pr_body_file)):
        body = None
        if envname:
            body = env.get(envname, '')
        elif fname:
            try:
                with open(fname, 'r', encoding='utf-8', errors='replace') as fh:
                    body = fh.read()
            except OSError:
                raise NoRun('the %s file is unreadable' % what)
        if body is not None:
            sources += 1
            sc.scan_unit(what, body, 1, '')
    if sources == 0:
        raise NoRun('messages mode was given nothing to scan')
    return False


def _files_from(args):
    """The listed paths. A NUL separated list (git -z) is never quoted, so it is
    taken verbatim; only a newline separated list may carry git's quoting."""
    if not getattr(args, 'files_from', None):
        return None
    if args.files_from == '-':
        data = sys.stdin.buffer.read()
    else:
        try:
            with open(args.files_from, 'rb') as fh:
                data = fh.read()
        except OSError:
            raise NoRun('the files-from list is unreadable')
    paths = split_list(data)
    if b'\0' in data:
        return paths
    return [unquote_git_path(p) for p in paths]


def _note_unlisted(sc, want, entries):
    """Every listed path that the scan source does not hold counts as skipped."""
    found = set(e.path for e in entries)
    sc.stats.skipped += sum(1 for p in dict.fromkeys(want) if p not in found)


# ---------------------------------------------------------------------------
# Media containers. Walkers PARSE the container; nothing here greps bytes.
# A finding is (kind, detail, severity, text_or_None).
# ---------------------------------------------------------------------------
GIF_LOOP_APPS = (b'NETSCAPE2.0', b'ANIMEXTS1.0')
EXIF_TAGS = {0x010F: 'make', 0x0110: 'model', 0x0131: 'software', 0x013B: 'artist',
             0x8298: 'copyright', 0x8825: 'gps-ifd', 0x9C9B: 'xp-title', 0x9C9D: 'xp-author',
             0xA430: 'owner-name', 0xA431: 'body-serial'}
BMFF_CONTAINERS = {b'moov', b'trak', b'mdia', b'minf', b'stbl', b'udta', b'ilst', b'edts',
                   b'meta'}
BMFF_FLAG = {b'\xa9xyz': 'location', b'loci': 'location', b'\xa9mak': 'device-make',
             b'\xa9mod': 'device-model', b'\xa9swr': 'software', b'\xa9nam': 'title',
             b'\xa9cmt': 'comment', b'\xa9aut': 'author', b'Exif': 'exif', b'XMP_': 'xmp',
             b'uuid': 'uuid-box'}
BMFF_KEYWORDS = (b'location', b'.make', b'.model', b'GPS', b'com.android.', b'creationdate')
MEDIA_EXT = {'.png': 'png', '.jpg': 'jpeg', '.jpeg': 'jpeg', '.gif': 'gif', '.webp': 'webp',
             '.mp4': 'bmff', '.mov': 'bmff', '.m4v': 'bmff', '.heic': 'bmff', '.heif': 'bmff',
             '.avif': 'bmff', '.3gp': 'bmff'}
UNPARSED_EXT = ('.pdf', '.tif', '.tiff', '.webm', '.mkv', '.avi', '.zip', '.gz', '.tgz',
                '.tar', '.bz2', '.xz', '.7z', '.rar', '.wav', '.mp3', '.flac', '.ogg', '.bmp',
                '.ico', '.psd', '.doc', '.docx', '.xls', '.xlsx', '.ppt', '.pptx', '.odt')
UNPARSED_MAGIC = ((b'%PDF', 'pdf'), (b'PK\x03\x04', 'zip'), (b'\x1f\x8b', 'gzip'),
                  (b'II*\x00', 'tiff'), (b'MM\x00*', 'tiff'), (b'\x1a\x45\xdf\xa3', 'ebml'),
                  (b'OggS', 'ogg'), (b'fLaC', 'flac'), (b'ID3', 'mp3'), (b'7z\xbc\xaf', '7z'),
                  (b'Rar!', 'rar'), (b'\xfd7zXZ', 'xz'), (b'BZh', 'bzip2'))


def walk_png(d):
    out, pos, chunks = [], 8, 0
    while pos + 8 <= len(d):
        ln, typ = struct.unpack('>I4s', d[pos:pos + 8])
        body = d[pos + 8:pos + 8 + ln]
        chunks += 1
        if typ in (b'tEXt', b'zTXt', b'iTXt', b'eXIf'):
            out.append(('png-' + typ.decode('latin-1'), 'len=%d' % ln, 'HARD',
                        png_chunk_text(typ, body)))
        elif typ == b'tIME':
            out.append(('png-tIME', 'len=%d' % ln, 'REPORT', None))
        pos += 12 + ln
        if typ == b'IEND':
            break
    if pos < len(d):
        out.append(('png-trailing', 'bytes=%d' % (len(d) - pos), 'HARD', None))
    out.append(('png-info', 'chunks=%d' % chunks, 'INFO', None))
    return out


def walk_gif(d):
    out = []
    try:
        pos = 13
        flags = d[10]
        if flags & 0x80:
            pos += 3 * (2 ** ((flags & 7) + 1))
        frames = apps = 0
        ended = False
        while pos < len(d):
            b = d[pos]
            if b == 0x3B:
                pos += 1
                ended = True
                break
            if b == 0x2C:
                frames += 1
                lf = d[pos + 9]
                pos += 10
                if lf & 0x80:
                    pos += 3 * (2 ** ((lf & 7) + 1))
                pos += 1
                while d[pos]:
                    pos += d[pos] + 1
                pos += 1
            elif b == 0x21:
                label = d[pos + 1]
                pos += 2
                first = None
                data = b''
                while d[pos]:
                    blk = d[pos + 1:pos + 1 + d[pos]]
                    if first is None:
                        first = blk
                    else:
                        data += blk
                    pos += d[pos] + 1
                pos += 1
                if label == 0xFE:
                    txt = ((first or b'') + data).decode('latin-1')
                    out.append(('gif-comment', 'len=%d' % len(txt), 'HARD', txt))
                elif label == 0x01:
                    out.append(('gif-plaintext', 'len=%d' % len(data), 'HARD',
                                data.decode('latin-1')))
                elif label == 0xFF:
                    apps += 1
                    if first not in GIF_LOOP_APPS:
                        out.append(('gif-application', 'len=%d' % len(data), 'HARD',
                                    (first or b'').decode('latin-1')))
            else:
                out.append(('gif-parse-error', 'byte 0x%02x at %d' % (b, pos), 'HARD', None))
                return out
        if not ended:
            out.append(('gif-parse-error', 'no trailer', 'HARD', None))
        elif pos < len(d):
            out.append(('gif-trailing', 'bytes=%d' % (len(d) - pos), 'HARD', None))
        out.append(('gif-info', 'frames=%d app_blocks=%d' % (frames, apps), 'INFO', None))
    except IndexError:
        out.append(('gif-parse-error', 'truncated', 'HARD', None))
    return out


def walk_tiff(t):
    out = []
    if t[:2] == b'II':
        e = '<'
    elif t[:2] == b'MM':
        e = '>'
    else:
        return [('exif-parse-error', 'bad byte order', 'HARD', None)]
    if len(t) < 8:
        return [('exif-parse-error', 'truncated', 'HARD', None)]
    off = struct.unpack(e + 'I', t[4:8])[0]
    todo, seen = [off], set()
    while todo:
        off = todo.pop()
        if off in seen or off + 2 > len(t):
            continue
        seen.add(off)
        n = struct.unpack(e + 'H', t[off:off + 2])[0]
        for i in range(n):
            ent = t[off + 2 + 12 * i:off + 14 + 12 * i]
            if len(ent) < 12:
                break
            tag, typ, cnt, val = struct.unpack(e + 'HHII', ent)
            if tag in EXIF_TAGS:
                text = None
                if typ == 2 and cnt > 4 and val + cnt <= len(t):
                    text = t[val:val + cnt].rstrip(b'\0').decode('latin-1', 'replace')
                elif typ == 2:
                    text = ent[8:8 + cnt].rstrip(b'\0').decode('latin-1', 'replace')
                out.append(('exif-' + EXIF_TAGS[tag], 'tag=0x%04x' % tag, 'HARD', text))
            if tag == 0x8769:
                todo.append(val)
    return out


def walk_jpeg(d):
    out, pos, segs = [], 2, 0
    while pos + 4 <= len(d) and d[pos] == 0xFF:
        m = d[pos + 1]
        if m == 0xDA:
            break
        ln = struct.unpack('>H', d[pos + 2:pos + 4])[0]
        body = d[pos + 4:pos + 2 + ln]
        segs += 1
        if m == 0xE1 and body[:6] == b'Exif\0\0':
            out += walk_tiff(body[6:])
        elif m == 0xE1 and body.startswith(b'http://ns.adobe.com/xap'):
            out.append(('jpeg-xmp', 'len=%d' % ln, 'HARD', body.decode('utf-8', 'replace')))
        elif m == 0xFE:
            out.append(('jpeg-comment', 'len=%d' % ln, 'HARD', body.decode('latin-1')))
        elif m == 0xED:
            out.append(('jpeg-iptc', 'len=%d' % ln, 'HARD', None))
        pos += 2 + ln
    end = d.rfind(b'\xff\xd9')
    if end == -1:
        out.append(('jpeg-parse-error', 'no end marker', 'HARD', None))
    elif end + 2 < len(d):
        out.append(('jpeg-trailing', 'bytes=%d' % (len(d) - end - 2), 'HARD', None))
    out.append(('jpeg-info', 'segments=%d' % segs, 'INFO', None))
    return out


def walk_bmff(d, lo=0, hi=None, depth=0, out=None):
    top = out is None
    out = [] if out is None else out
    hi = len(d) if hi is None else hi
    pos = lo
    boxes = 0
    while pos + 8 <= hi and depth < 12:
        sz, typ = struct.unpack('>I4s', d[pos:pos + 8])
        boxes += 1
        hdr = 8
        if sz == 1:
            if pos + 16 > hi:
                break
            sz = struct.unpack('>Q', d[pos + 8:pos + 16])[0]
            hdr = 16
        elif sz == 0:
            sz = hi - pos
        if sz < hdr or pos + sz > hi:
            out.append(('bmff-parse-error', 'box at %d' % pos, 'HARD', None))
            break
        if typ in BMFF_FLAG:
            body = d[pos + hdr:pos + sz]
            text = None
            if typ[:1] == b'\xa9' or typ == b'loci':
                text = body.decode('latin-1', 'replace')
            out.append(('bmff-' + BMFF_FLAG[typ], 'len=%d' % sz, 'HARD', text))
        if typ == b'keys':
            body = d[pos + hdr:pos + sz]
            for kw in BMFF_KEYWORDS:
                if kw in body:
                    out.append(('bmff-keys', 'keyword ' + kw.decode('latin-1'), 'HARD', None))
        if typ in BMFF_CONTAINERS:
            inner = pos + hdr
            if typ == b'meta' and d[inner + 4:inner + 8] != b'hdlr':
                inner += 4
            walk_bmff(d, inner, pos + sz, depth + 1, out)
        pos += sz
    if top:
        out.append(('bmff-info', 'top-level boxes=%d' % boxes, 'INFO', None))
    return out


def walk_webp(d):
    out, pos, chunks = [], 12, 0
    while pos + 8 <= len(d):
        typ, ln = struct.unpack('<4sI', d[pos:pos + 8])
        chunks += 1
        if typ in (b'EXIF', b'XMP '):
            body = d[pos + 8:pos + 8 + ln]
            text = body.decode('utf-8', 'replace') if typ == b'XMP ' else None
            out.append(('webp-' + typ.decode().strip().lower(), 'len=%d' % ln, 'HARD', text))
            if typ == b'EXIF':
                out += walk_tiff(body[6:] if body[:6] == b'Exif\0\0' else body)
        pos += 8 + ln + (ln & 1)
    out.append(('webp-info', 'chunks=%d' % chunks, 'INFO', None))
    return out


def sniff_media(data, path):
    """(kind, walker) for a container, ('unparsed', label) for one with no walker, else None."""
    if data[:8] == b'\x89PNG\r\n\x1a\n':
        return 'png', walk_png
    if data[:6] in (b'GIF87a', b'GIF89a'):
        return 'gif', walk_gif
    if data[:2] == b'\xff\xd8':
        return 'jpeg', walk_jpeg
    if data[:4] == b'RIFF' and data[8:12] == b'WEBP':
        return 'webp', walk_webp
    if data[4:8] in (b'ftyp', b'moov', b'mdat', b'wide', b'free'):
        return 'bmff', walk_bmff
    for magic, label in UNPARSED_MAGIC:
        if data.startswith(magic):
            return 'unparsed', label
    ext = os.path.splitext(path.lower())[1]
    if ext in MEDIA_EXT:
        return 'unparsed', 'declared ' + MEDIA_EXT[ext] + ' with foreign bytes'
    if ext in UNPARSED_EXT:
        return 'unparsed', 'extension ' + ext
    return None


def run_media(sc, git, args):
    files_from = _files_from(args)
    sc.listed = files_from is not None
    if args.ref:
        entries = enumerate_ref(git, args.ref)
        if files_from is not None:
            want = set(files_from)
            entries = [e for e in entries if e.path in want]
    elif args.staged:
        entries = enumerate_staged(git, files_from if files_from is not None
                                   else getattr(args, 'staged_set', None))
    else:
        entries = enumerate_worktree(git, False, files_from)
    if files_from is not None and (args.ref or args.staged):
        _note_unlisted(sc, files_from, entries)
    kinds = {}
    for e, data, target in load_entries(git, entries):
        sc.stats.files += 1
        if e.kind == 'missing':
            sc.stats.skipped += 1
            continue
        sc.all_paths.append(e.path)
        if target is not None or isinstance(data, str):
            continue
        sn = sniff_media(data, e.path)
        if sn is None:
            continue
        kind, walker = sn
        sc.stats.media += 1
        sc.stats.units += 1
        kinds[kind] = kinds.get(kind, 0) + 1
        if kind == 'unparsed':
            sc.add('media-unparsed', 'HARD', e.path, 0, walker, 'container')
            continue
        findings = walker(data)
        detail = []
        info = []
        for fkind, fdetail, fsev, ftext in findings:
            if fsev == 'INFO':
                info.append(fdetail)
                continue
            cid = 'media-time' if fsev == 'REPORT' else 'media-meta'
            sc.add(cid, fsev, e.path, 0, fkind + ' ' + fdetail, 'container')
            detail.append(fkind)
            if ftext:
                sc.scan_unit(e.path, ftext, 0, 'metadata')
        if not sc.quiet:
            sc.out('media %s %s: %s (%s)' % (kind, clean(sc.redact_path(e.path)),
                                             ', '.join(detail) if detail else 'clean',
                                             '; '.join(info) if info else 'no parse stats'))
    sc.media_kinds = kinds
    return files_from is None and not args.staged


# ---------------------------------------------------------------------------
# Controls, driver, CLI.
# ---------------------------------------------------------------------------
def run_controls(classes, private_patterns, out, neuter=None):
    """Every generic class must hit its own sample; the private tier must find a canary."""
    dead = []
    sc = Scanner('tree', classes, None, [], lambda s: None)
    for c in classes:
        before = len(sc.hits)
        sc.scan_unit('control', c.sample, 1, '', sevpath='results/control')
        if not any(h.cls == c.id for h in sc.hits[before:]):
            dead.append(c.id)
    canary = 'zq' + binascii.hexlify(os.urandom(6)).decode()
    idx = len(private_patterns) + 1
    canary_pat = parse_private('word:' + canary)[0]
    canary_pat.index = idx
    if neuter == 'private-canary':
        canary_pat.rx = re.compile(r'(?!x)x')
    ps = PrivateSet(list(private_patterns) + [canary_pat])
    sc2 = Scanner('tree', classes, ps, [], lambda s: None)
    sc2.scan_unit('control', 'seen on ' + canary + ' today', 1, '')
    if not any(h.private is not None and h.private.index == idx for h in sc2.hits):
        dead.append('private-canary')
    if dead:
        out('CONTROL FAILED: ' + ', '.join(dead))
        raise NoRun('a built-in control is dead: ' + ', '.join(dead))


def default_allow_path(root):
    return os.path.join(root, 'tools', 'scripts', 'leak_scan_allow.txt')


def load_allow(args, root, classes):
    if getattr(args, 'no_allow', False):
        return []
    path = getattr(args, 'allow', None) or default_allow_path(root)
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            text = fh.read()
    except OSError:
        if getattr(args, 'allow', None):
            raise NoRun('the allowlist file is unreadable')
        return []
    ids = set(c.id for c in classes) | set(s[0] for s in STRUCT_CLASSES)
    hard = set(c.id for c in classes if any(c.sev(m, 'x') == 'HARD' for m in
                                           ('tree', 'diff', 'messages', 'names')))
    hard |= set(s[0] for s in STRUCT_CLASSES if s[1] == 'HARD')
    return parse_allow(text, ids, hard)


def check_allow_usage(sc, entries_paths, private_loaded):
    """Full-tree rule: an entry that matched nothing, or excused nothing, fails the
    run. Returns (problems, notes). A private@<tag> waiver whose tag no loaded
    private entry carries cannot be judged on use, and what the secret holds must
    not decide whether a push to main is red, so it is NOTED, not failed; its path
    check still runs, because that depends on the tree alone. A waiver for a
    generic class, and a private waiver whose tag is loaded, keep both checks."""
    problems, notes = [], []
    mode_classes = {'tree': None, 'media': ('media-meta', 'media-time', 'media-unparsed')}
    if sc.mode not in mode_classes:
        return problems, notes
    loaded_tags = set()
    if sc.private is not None:
        loaded_tags = set(p.tag for p in sc.private.patterns if p.tag)
    for e in sc.allow:
        if e.cls.startswith('private'):
            if not private_loaded or sc.mode == 'media':
                continue
        elif sc.mode == 'media' and e.cls not in mode_classes['media']:
            continue
        elif sc.mode == 'tree' and e.cls in mode_classes['media']:
            continue
        e.matched_files = sum(1 for p in entries_paths if e.rx.match(p))
        if e.matched_files == 0:
            problems.append('allowlist line %d matches no file in the tree (stale)' % e.lineno)
        elif e.cls.startswith('private') and e.cls[8:] not in loaded_tags:
            notes.append('allowlist line %d unused: no %s entry loaded' % (e.lineno, e.cls[8:]))
        elif e.suppressed == 0:
            problems.append('allowlist line %d excused nothing (unused)' % e.lineno)
    return problems, notes


def find_root(cwd, env):
    p = subprocess.run(['git', 'rev-parse', '--show-toplevel'], cwd=cwd, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if p.returncode != 0:
        raise NoRun('not inside a git worktree')
    return p.stdout.decode('utf-8', 'replace').strip()


def build_parser():
    ap = _Parser(prog='leak_scan.py', add_help=True, allow_abbrev=False,
                 description='the in-tree leak guard')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--list-classes', action='store_true')
    ap.add_argument('--list-lan-values', action='store_true')
    sub = ap.add_subparsers(dest='mode', parser_class=_Parser)

    def common(p):
        p.add_argument('--require-private', action='store_true')
        p.add_argument('--no-private', action='store_true')
        p.add_argument('--hard', action='append', default=[])
        p.add_argument('--format', choices=('text', 'github'), default='text')
        p.add_argument('--allow')
        p.add_argument('--no-allow', action='store_true')
        p.add_argument('--allow-empty', action='store_true')
        p.add_argument('--quiet', action='store_true')

    t = sub.add_parser('tree', allow_abbrev=False)
    t.add_argument('--ref')
    t.add_argument('--staged', action='store_true')
    t.add_argument('--files-from')
    t.add_argument('--untracked', action='store_true')
    common(t)
    d = sub.add_parser('diff', allow_abbrev=False)
    d.add_argument('--range')
    d.add_argument('--staged', action='store_true')
    common(d)
    m = sub.add_parser('messages', allow_abbrev=False)
    m.add_argument('--range')
    m.add_argument('--message-file')
    m.add_argument('--pr-title-env')
    m.add_argument('--pr-body-env')
    m.add_argument('--pr-title-file')
    m.add_argument('--pr-body-file')
    m.add_argument('--allow-email-file')
    m.add_argument('--ident-from-git', action='store_true')
    m.add_argument('--require-commits', action='store_true')
    common(m)
    n = sub.add_parser('names', allow_abbrev=False)
    n.add_argument('--ref')
    n.add_argument('--staged', action='store_true')
    n.add_argument('--range')
    n.add_argument('--branch')
    n.add_argument('--branch-env')
    n.add_argument('--untracked', action='store_true')
    common(n)
    me = sub.add_parser('media', allow_abbrev=False)
    me.add_argument('--ref')
    me.add_argument('--staged', action='store_true')
    me.add_argument('--files-from')
    common(me)
    h = sub.add_parser('hook', allow_abbrev=False)
    h.add_argument('which', choices=('pre-commit', 'commit-msg'))
    h.add_argument('message_file', nargs='?')
    common(h)
    return ap


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise Usage(message)


def parse_args(argv):
    ap = build_parser()
    if argv and argv[0] in ('--self-test', '--list-classes', '--list-lan-values'):
        if len(argv) > 1:
            raise Usage('nothing may follow ' + argv[0])
    args = ap.parse_args(argv)
    if not (args.self_test or args.list_classes or args.list_lan_values) and not args.mode:
        raise Usage('a mode is required (tree, diff, messages, names, media, hook)')
    if args.mode == 'diff' and not (args.range or args.staged):
        raise Usage('diff needs --range A..B or --staged')
    if args.mode in ('tree', 'names', 'media') and getattr(args, 'ref', None) and getattr(
            args, 'staged', False):
        raise Usage('--ref and --staged are exclusive')
    if args.mode == 'hook' and args.which == 'commit-msg' and not args.message_file:
        raise Usage('hook commit-msg needs the message file')
    return args


def run_mode(args, root, env, out, neuter=None, home=None):
    """Runs one mode. Returns the exit code. `out` receives every printed line."""
    classes = build_classes(neuter)
    if args.no_private and args.require_private:
        raise Usage('--no-private and --require-private are exclusive')
    if args.no_private:
        private_patterns, kind, warnings = [], None, []
    else:
        private_patterns, kind, warnings = load_private(env, home)
    for w in warnings:
        out('WARNING: ' + w)
    if args.require_private and not private_patterns:
        out('PRIVATE PATTERNS NOT LOADED and --require-private was given')
        return EXIT_NORUN
    run_controls(classes, private_patterns, out, neuter)
    private = PrivateSet(private_patterns) if private_patterns else None
    if args.mode == 'hook':
        return run_hook(args, root, env, out, classes, private, kind)
    allow = load_allow(args, root, classes)
    git = Git(root, env)
    sc = Scanner(args.mode, classes, private, allow, out, args.format, args.hard, args.quiet)
    t0 = time.time()
    if args.mode == 'messages':
        full_mode = run_messages(sc, git, args, env)
    elif args.mode == 'names':
        full_mode = run_names(sc, git, args, env)
    else:
        full_mode = {'tree': run_tree, 'diff': run_diff, 'media': run_media}[args.mode](
            sc, git, args)
    if sc.listed and sc.stats.skipped:
        sc.emit_hits()
        out('leak_scan %s: NO RUN, %d listed path(s) are not in the scan source (renamed or '
            'deleted since the list was made, or a stale list)' % (args.mode,
                                                                   sc.stats.skipped))
        return EXIT_NORUN
    if sc.stats.units == 0 and not args.allow_empty:
        out('leak_scan %s: NO RUN, zero units scanned' % args.mode)
        return EXIT_NORUN
    hard, report = sc.emit_hits()
    problems = []
    if full_mode:
        problems, notes = check_allow_usage(sc, sc.all_paths, private is not None)
        for n in notes:
            out('ALLOWLIST NOTE: ' + n)
        for p in problems:
            out('ALLOWLIST: ' + p)
    if private is None and not args.quiet:
        out('PRIVATE PATTERNS NOT LOADED: names, devices, people and real LAN addresses were '
            'NOT checked.')
    by_class = {}
    for h in sc.hits:
        by_class[h.cls] = by_class.get(h.cls, 0) + 1
    status = 'OK' if hard == 0 and not problems else 'FAIL'
    if not (args.quiet and status == 'OK'):
        if by_class:
            out('by-class: ' + ' '.join('%s=%d' % (k, by_class[k]) for k in sorted(by_class)))
        st = sc.stats
        out('leak_scan %s: %s hard=%d report=%d files=%d (text=%d binary=%d symlink=%d) '
            'units=%d json_leaves=%d classes=%d private=%s allowlist=%d pragmas=%d '
            'controls=ok wall=%.1fs%s' % (
                args.mode, status, hard, report, st.files, st.text, st.binary, st.symlink,
                st.units, st.json_leaves, len(classes),
                ('LOADED(%d,%s)' % (len(private), kind)) if private else
                'NOT-LOADED(generic classes only)',
                len(allow), st.pragmas, time.time() - t0,
                _extras(sc)))
    return EXIT_OK if status == 'OK' else EXIT_HIT


def _extras(sc):
    st = sc.stats
    bits = []
    if st.pragmas_refused:
        bits.append('pragmas_refused=%d' % st.pragmas_refused)
    if st.json_unparsed:
        bits.append('json_unparsed=%d' % st.json_unparsed)
    if st.oversize:
        bits.append('oversize=%d' % st.oversize)
    if st.skipped:
        bits.append('skipped=%d' % st.skipped)
    if st.commits:
        bits.append('commits=%d' % st.commits)
    if sc.suppressed:
        bits.append('suppressed=%d' % sc.suppressed)
    if sc.media_kinds is not None:
        bits.append('media=%d' % st.media)
        for k in sorted(sc.media_kinds):
            bits.append('%s=%d' % (k, sc.media_kinds[k]))
    return (' ' + ' '.join(bits)) if bits else ''


def run_hook(args, root, env, out, classes, private, kind):
    """The pre-commit and commit-msg hooks, in one process: three scans, one note."""
    git = Git(root, env)
    allow = load_allow(args, root, classes)
    rc = EXIT_OK
    if private is None:
        out('leak-guard: private patterns NOT loaded, generic classes only')
    if args.which == 'pre-commit':
        branch = git.text(['rev-parse', '--abbrev-ref', 'HEAD']).strip()
        changed = split_list(git.run(['diff', '--cached', '--name-only', '-z',
                                      '--diff-filter=ACMRT']))
        plan = (('diff', {'staged': True, 'range': None}),
                ('names', {'staged': True, 'range': None, 'ref': None, 'branch': branch,
                           'branch_env': None, 'untracked': False, 'staged_set': changed}),
                ('media', {'staged': True, 'ref': None, 'files_from': None,
                           'staged_set': changed}))
    else:
        # git sets GIT_EDITOR=: for the hook when no editor is launched (-m, -F,
        # a script): the message is then kept as written, comment lines included.
        cleanup = git.run(['config', '--get', 'commit.cleanup'], ok_codes=(0, 1)).decode(
            'utf-8', 'replace').strip().lower() or 'default'
        plan = (('messages', {'range': None, 'message_file': args.message_file,
                              'pr_title_env': None, 'pr_body_env': None, 'pr_title_file': None,
                              'pr_body_file': None, 'allow_email_file': None,
                              'ident_from_git': True, 'require_commits': False,
                              'editor_used': env.get('GIT_EDITOR') != ':',
                              'cleanup': cleanup}),)
    for mode, extra in plan:
        sc = Scanner(mode, classes, private, allow, out, args.format, args.hard, quiet=True)
        ns = argparse.Namespace(**extra)
        if mode == 'messages':
            run_messages(sc, git, ns, env)
        elif mode == 'diff':
            run_diff(sc, git, ns)
        elif mode == 'names':
            run_names(sc, git, ns, env)
        else:
            run_media(sc, git, ns)
        hard, _ = sc.emit_hits()
        if hard:
            out('leak-guard %s: %d hard hit(s) in %s mode; fix them, or add a reviewed '
                'allowlist entry, or bypass once with --no-verify' % (args.which, hard, mode))
            rc = EXIT_HIT
    if rc == EXIT_OK:
        out('leak-guard %s: OK (private=%s)' % (
            args.which, ('loaded,' + kind) if private else 'not loaded'))
    return rc


def list_classes(out):
    for c in build_classes():
        sevs = ' '.join('%s=%s' % (m, _sev_name(c, m)) for m in
                        ('tree', 'diff', 'messages', 'names'))
        out('%-16s %s  [%s]' % (c.id, sevs, c.what))
    for cid, sev, what in STRUCT_CLASSES:
        out('%-16s %s  [%s]' % (cid, sev, what))
    out('placeholder users: ' + ' '.join(sorted(PH_USER)))
    out('placeholder hosts: ' + ' '.join(sorted(PH_HOST)))
    out('role local parts: ' + ' '.join(sorted(ROLE_LOCAL)) + ' ' + ' '.join(
        p + '*' for p in ROLE_PREFIXES))
    out('lan example values: %d' % len(LAN_EXAMPLES))
    out('private tags: ' + ' '.join(TAGS))


def _sev_name(c, mode):
    s = c._sev.get(mode)
    if callable(s):
        return 'scoped'
    return str(s)


def list_lan_values(root, env, out):
    """Maintenance: the distinct RFC 1918 values in the tree, for reviewing LAN_EXAMPLES."""
    git = Git(root, env)
    rx = re.compile(NB_L + r'(?:10\.' + OCT + r'|192\.168|172\.(?:1[6-9]|2\d|3[01]))\.' + OCT
                    + r'\.' + OCT + NB_R)
    values = {}
    for e, data, target in load_entries(git, enumerate_worktree(git, False)):
        if not isinstance(data, bytes) or is_binary(data):
            continue
        text = data.decode('utf-8', 'replace')
        for m in rx.finditer(text):
            if text[m.end():m.end() + 1] == '/':
                continue
            values[m.group(0)] = values.get(m.group(0), 0) + 1
    for v in sorted(values, key=lambda a: tuple(int(x) for x in a.split('.'))):
        out('%-18s %d%s' % (v, values[v], '' if v in LAN_EXAMPLES else '  NOT in LAN_EXAMPLES'))
    out('%d distinct value(s)' % len(values))


# ---------------------------------------------------------------------------
# Self-test. Every fixture string is ASSEMBLED from fragments and from the
# class definitions at run time, so this file holds no literal leak. The same
# Scanner serves the sweep and these arms.
# ---------------------------------------------------------------------------
PW = STANDIN_WORD            # the private-tier stand-in, spelled joined
SU = STANDIN_USER            # a login and home-segment stand-in
SH = STANDIN_HOST            # a host stand-in
PERSON = 'Orvald' + ' ' + 'Pentwistle'
PLAIN_USER = 'thorn' + 'wick'   # a login NO tier in the self-test knows
RED1 = '<private#1@host>'
EXPECTED_ARMS = 174


def _png(chunks):
    def chunk(typ, body):
        return struct.pack('>I', len(body)) + typ + body + struct.pack(
            '>I', zlib.crc32(typ + body) & 0xffffffff)
    ihdr = chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 0, 0, 0, 0))
    idat = chunk(b'IDAT', zlib.compress(b'\0\0'))
    return (b'\x89PNG\r\n\x1a\n' + ihdr + b''.join(chunk(t, b) for t, b in chunks) + idat
            + chunk(b'IEND', b''))


def _gif(extra=b''):
    head = b'GIF89a' + struct.pack('<HHBBB', 1, 1, 0x80, 0, 0) + b'\0\0\0\xff\xff\xff'
    img = b'\x2c' + struct.pack('<HHHHB', 0, 0, 1, 1, 0) + b'\x02\x02\x44\x01\x00'
    loop = b'\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00'
    return head + loop + extra + img + b'\x3b'


def _gif_comment(text):
    return b'\x21\xfe' + bytes([len(text)]) + text + b'\0'


def _jpeg(tiff):
    app1 = b'Exif\0\0' + tiff
    return (b'\xff\xd8\xff\xe1' + struct.pack('>H', len(app1) + 2) + app1
            + b'\xff\xda\x00\x02\xff\xd9')


def _tiff_gps(make):
    body = make.encode() + b'\0'
    n = 2
    ifd = struct.pack('<H', n)
    ifd += struct.pack('<HHII', 0x010F, 2, len(body), 8 + 2 + 12 * n + 4)
    ifd += struct.pack('<HHII', 0x8825, 4, 1, 38)
    ifd += struct.pack('<I', 0)
    return b'II*\0' + struct.pack('<I', 8) + ifd + body


def _tiff_plain():
    return (b'II*\0' + struct.pack('<I', 8) + struct.pack('<H', 1)
            + struct.pack('<HHII', 0x0112, 3, 1, 1) + struct.pack('<I', 0))


def _box(t, body):
    return struct.pack('>I', len(body) + 8) + t + body


def _mp4(dirty, text=b'+00.0000+000.0000/'):
    ftyp = _box(b'ftyp', b'qt  \0\0\0\0qt  ')
    if dirty:
        return ftyp + _box(b'moov', _box(b'mvhd', b'\0' * 100) + _box(
            b'udta', _box(b'\xa9xyz', b'\0\x12\x15\xc7' + text)))
    return ftyp + _box(b'moov', _box(b'mvhd', b'\0' * 100)) + _box(
        b'mdat', b'\xa9xyz GPS location bytes that live inside media data')


def _fixture_tree():
    """Returns (files, expected) for the tree surfaces. Paths are repo relative."""
    lan_real = _addr(10, 77, 13, 9)
    cg = _addr(100, 64, 0, 1)
    ov = OVERLAY_WORDS[0]
    dns = SU + '.' + DNS_LABEL + '.' + DNS_TLD
    files = {}
    exp = []

    def put(path, text, *classes):
        files[path] = text.encode('utf-8') if isinstance(text, str) else text
        for c in classes:
            exp.append((c, path))

    put('src/lib.rs', '//! crate docs, measured on ' + PW + '\n'
        '/// doc: see ' + P_MAC + SU + '/src for the setup\n'
        '// note: nothing here\n'
        'fn main() { let peer = "' + cg + '"; }\n',
        'home-mac', 'cgnat-addr', 'private#1@host')
    put('tools/gen.py', '# tuned on ' + P_LINUX + SU + '/x by ' + PW + '\n'
        "ap.add_argument('--peer', help='peer at " + lan_real + "')\n"
        '# join the ' + ov + ' first\n',
        'home-linux', 'lan-addr', 'overlay-word', 'private#1@host')
    put('src/x.cpp', '// C:' + BSL + 'Users' + BSL + SU + BSL + 'x\n', 'home-win')
    put('scripts/run.sh', '# temp ' + P_TEMP + 'ab/cdefgh123456/T/ on ' + PW + '\n',
        'temp-root', 'private#1@host')
    put('.github/workflows/x.yml', '# resolve host.' + dns + '\n# owner tag' + ':'
        + 'lab-runner\n# ' + OVERLAY_WORDS[1] + ' up\n',
        'overlay-dns', 'acl-tag', 'overlay-word')
    put('README.md', 'ssh ' + SU + '@' + SH + '\nmail ' + SU + '@gmail' + '.com\n'
        'contact ' + PERSON + ' for access\n',
        'login-at-host', 'email-personal', 'private#3@person')
    put('cfg/x.xml', '<!-- tcp/' + SH + '.local:7447 -->\n', 'mdns-local')
    put('tools/rt4/keys.json', '{"' + lan_real + '": {"cmd": "join the ' + ov + ' '
        + BSL + 'u0026' + BSL + 'u0026 now"}}\n')
    put('results/run.json', '{"invocations": [{"argv": ["' + BSL + '/home' + BSL + '/' + SU
        + BSL + '/bin"], "uname": "Linux ' + SH + ' 5.15"}], "skips": [{"reason": "' + PW
        + ' was busy' + BSL + 'nssh ' + SU + '@' + SH + '"}]}\n',
        'home-linux', 'host-field', 'login-at-host', 'private#1@host')
    put('results/run.log', 'cd ' + TILDE + '/' + SU + '-notes/run\nrobot=' + ESC + '[1m' + PW
        + ESC + '[0m done\n', 'home-tilde', 'private#1@host')
    put('notes/deb.md', 'seen [' + PW[0] + ']' + PW[1:] + ' twice\n', 'private#1@host')
    put('logs/x.log', 'ts=1 body={"path": "' + BSL + '/home' + BSL + '/' + SU + BSL + '/x"}\n',
        'home-linux')
    b64 = base64.b64encode((PW + ' inside the image').encode()).decode()
    put('img/d.svg', '<svg><desc>drawn on ' + PW + '</desc><text>x</text>'
        '<image href="data:image/png;base64,' + b64 + '"/></svg>\n', 'private#1@host')
    put('blob/a.bin', b'\0\0' + PW.encode() + b' here\0' + (P_LINUX + SU + '/x').encode()
        + b'\0', 'private#1@host', 'home-linux')
    put('blob/b.bin', b'\0' + PW.encode('utf-16-le') + b'\0\0', 'private#1@host')
    put('media/t.png', _png([(b'tEXt', b'Comment\0rendered on ' + PW.encode()),
                             (b'zTXt', b'Note\0\0' + zlib.compress(
                                 (P_LINUX + SU + '/run').encode()))]),
        'private#1@host', 'home-linux')
    put('media/c.gif', _gif(_gif_comment(b'made on ' + PW.encode())), 'private#1@host')
    put('media/app.gif', _gif(b'\x21\xff\x0bXMP DataXMP\x03\x01\x00\x00\x00'))
    put('media/g.jpg', _jpeg(_tiff_gps(PW)))
    put('media/l.mp4', _mp4(True))
    put('docs/' + PW + '-notes.md', 'clean body\nrange 3' + DASH_EN + '5\n')
    put(PW + '/x.txt', 'clean\n')
    exp.append(('private#1@host', 'docs/' + RED1 + '-notes.md'))
    exp.append(('private#1@host', RED1 + '/x.txt'))
    put('forms.txt', PW + '\n' + PW[:6] + '-' + PW[6:] + '\n' + PW[:6] + '_' + PW[6:] + '\n'
        + PW.upper() + '\n', 'private#1@host')
    put('tools/hooks/x', 'a guard file with a dash ' + DASH_EM + '\n', 'style-dash')
    put('style.md', 'a report line ' + DASH_EN + '\n')
    put('clean/ok.md', ' '.join((
        _addr(100, 64, 0, 0) + '/10', _addr(100, 64, 0, 0), _addr(100, 127, 255, 255),
        _addr(192, 0, 2, 17), _addr(198, 51, 100, 7), _addr(203, 0, 113, 9),
        _addr(10, 0, 0, 1), _addr(192, 168, 123, 18), _addr(192, 168, 1, 0) + '/24',
        '<name>.local', 'robot-a.local', 'settings.local.json', TILDE + '/service',
        P_MAC + 'dev/x', P_LINUX + 'ubuntu/x', TILDE + '/.config', 'licensing@'
        + PROJECT_DOMAIN, 'x@example.invalid', 'git@' + 'github.com:org/repo',
        'ssh ubuntu@box-x86', '"uname": "Linux box-arm 5.15"', 'tag' + ':' + 'x')) + '\n')
    # identity values in a FILE or DIRECTORY name, none of them known to the tier:
    # the location printed beside a masked value must not carry them, on a name
    # hit, on a content hit inside such a file, and on a media container's line
    put('n/' + lan_real + '.md', 'clean body\n', 'lan-addr')
    put('n/' + PLAIN_USER + '@gmail' + '.com.txt', 'clean body\n', 'email-personal')
    put('n/Users/' + PLAIN_USER + '/notes.md', 'peer ' + cg + '\n', 'home-mac', 'cgnat-addr')
    put('n/' + lan_real + '.png', _png([(b'tEXt', b'Software\0plain')]))
    put('media/clean.png', _png([]))
    put('media/loop.gif', _gif())
    put('media/mdat.mp4', _mp4(False))
    put('media/clean.jpg', _jpeg(_tiff_plain()))
    return files, exp


def _write_files(root, files):
    for path, data in files.items():
        full = os.path.join(root, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, 'wb') as fh:
            fh.write(data.encode('utf-8') if isinstance(data, str) else data)


def self_test(out, base_env, argv0):
    failures = []
    arms = [0]
    surfaces = set()

    def arm(name, ok, detail=''):
        arms[0] += 1
        if not ok:
            failures.append('%s %s' % (name, detail))

    private_env = ('word:' + PW + ' @host\nword:' + SU + ' @login\nword:' + PERSON
                   + ' @person\n')
    with tempfile.TemporaryDirectory() as tmp:
        home = os.path.join(tmp, 'home')
        os.makedirs(home)
        env = {k: v for k, v in base_env.items() if not k.startswith('LEAK_')}
        env.update({'HOME': home, 'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_CONFIG_NOSYSTEM': '1',
                    'GIT_AUTHOR_NAME': 'Self Test', 'GIT_COMMITTER_NAME': 'Self Test',
                    'GIT_AUTHOR_EMAIL': 'self-test@' + FORGE_NOREPLY,
                    'GIT_COMMITTER_EMAIL': 'self-test@' + FORGE_NOREPLY})
        penv = dict(env, LEAK_PATTERNS=private_env)
        repo = os.path.join(tmp, 'repo')
        os.makedirs(repo)
        git = Git(repo, env)
        git.run(['init', '-q'])
        git.run(['symbolic-ref', 'HEAD', 'refs/heads/main'])
        files, expected = _fixture_tree()
        _write_files(repo, files)
        os.symlink(P_LINUX + SU + '/target', os.path.join(repo, 'link'))
        expected += [('home-linux', 'link'), ('private#2@login', 'link')]
        git.run(['add', '-A'])
        git.run(['commit', '-q', '-m', 'fixture'])

        def run(argv, e=None, cwd=None):
            lines = []
            rc = main_inner(argv, cwd or repo, e or env, lines.append, argv0)
            return rc, lines

        def hits(lines, word='HIT'):
            found = set()
            for ln in lines:
                m = re.match(word + r' (\S+) (.+?):(\d+)(?=:|\s\[|$)', ln)
                if m:
                    found.add((m.group(1), m.group(2), int(m.group(3))))
            return found

        def bare_private(lines):
            """True when every private HIT line is label, location and an optional
            bracket tag and NOTHING else: no matched text, no surrounding line."""
            mine = [ln for ln in lines if ln.startswith('HIT private#')]
            shape = re.compile(r'^HIT private#\S+ .+:\d+(?: \[[a-z]+\])?$')
            return bool(mine) and all(shape.match(ln) and ': ' not in ln for ln in mine)

        # --- tree: every class on its surface, worktree, ref and staged readers ----
        for label, argv in (('worktree', ['tree', '--no-allow']),
                            ('ref', ['tree', '--ref', 'HEAD', '--no-allow']),
                            ('staged', ['tree', '--staged', '--no-allow'])):
            rc, lines = run(argv, penv)
            got = hits(lines)
            gotcp = set((c, p) for c, p, _ in got)
            missing = [e for e in expected if e not in gotcp]
            arm('tree-%s-expected' % label, rc == EXIT_HIT and not missing,
                'rc=%d missing=%s' % (rc, missing))
            clean_hits = [c for c, p, _ in got if p == 'clean/ok.md']
            arm('tree-%s-negative-controls' % label, not clean_hits, str(clean_hits))
            for c, p in expected:
                surfaces.add(os.path.splitext(p)[1] or p.split('/')[0])
        rc, lines = run(['tree', '--no-allow'], penv)
        got = hits(lines)
        arm('forms-joined-dashed-underscored-upper',
            all(('private#1@host', 'forms.txt', n) in got for n in (1, 2, 3, 4)))
        arm('json-line-number-real', ('host-field', 'results/run.json', 1) in got)
        arm('json-leaf-walk-reaches-an-escaped-newline',
            ('login-at-host', 'results/run.json', 1) in got)
        arm('guard-file-dash-hard', any(ln.startswith('HIT style-dash tools/hooks/x')
                                        for ln in lines))
        arm('report-dash-not-hard', any(ln.startswith('REPORT style-dash style.md')
                                        for ln in lines)
            and any(ln.startswith('REPORT home-tilde clean/ok.md') for ln in lines))
        # contract 1: no private text anywhere in the output
        arm('private-output-redacted', not any(PW.lower() in ln.lower() or SU in ln
                                               or home in ln for ln in lines))
        # contract 1b: a private hit line carries no text at all, not even the
        # surrounding line with the match redacted
        arm('private-hit-line-carries-no-text', bare_private(lines))
        # contract 1c: an identity-bearing generic hit prints its VALUE only where
        # the private tier is loaded and the output is a terminal (text format).
        # With no tier (the lint step, a fork pull request) or in a CI log (github
        # format) it prints a masked shape; style-dash and overlay-word keep their
        # text everywhere. `clear` holds every identity value the fixture plants.
        lan_real, cg = _addr(10, 77, 13, 9), _addr(100, 64, 0, 1)
        clear = (SU, SH, lan_real, cg, 'cdefgh123456', 'lab-runner', PLAIN_USER)

        def value_free(lines):
            return not any(ln.startswith(('HIT', 'REPORT', '::error'))
                           and any(v in ln for v in clear) for ln in lines)

        home_mac = P_MAC + SU + '/'
        masked_mac = '(value masked, %d chars: %s)' % (len(home_mac),
                                                       MASK_RX.sub('x', home_mac))
        ov = OVERLAY_WORDS[0]
        rc, lines_nt = run(['tree', '--no-allow'])
        arm('b1-no-tier-text-format-masks-identity-values', value_free(lines_nt)
            and any(ln == 'HIT home-mac src/lib.rs:2: ' + masked_mac for ln in lines_nt)
            and any(ln.startswith('HIT overlay-word tools/gen.py:3: ' + ov) for ln in lines_nt)
            and any(ln.startswith('REPORT style-dash style.md:1: ' + DASH_EN) for ln in lines_nt),
            str([ln for ln in lines_nt if not value_free([ln])][:3]))
        rc, lines_nt = run(['tree', '--no-allow', '--format', 'github'])
        arm('b1-no-tier-github-format-masks-identity-values', value_free(lines_nt)
            and any(ln == '::error file=src/lib.rs,line=2::home-mac: ' + gh_escape(masked_mac)
                    for ln in lines_nt)
            and any(ln.startswith('::error file=tools/gen.py,line=3::overlay-word: ' + ov)
                    for ln in lines_nt),
            str([ln for ln in lines_nt if not value_free([ln])][:3]))
        arm('b1-mask-shape-oracle', mask_shape(P_MAC + 'ab1/x') == '(value masked, 12 chars: '
            '/xxxxx/xxx/x)' and mask_shape('r' + chr(0xE9) + 's-9@h.ts') == '(value masked, '
            '10 chars: xxx-x@x.xx)')
        # contract 1e: the key path of an unlocated JSON leaf is a location that can
        # carry a name, so it prints only where a value would (tier loaded, text),
        # whatever the class of the hit that landed on the leaf.
        rc, lines_jt = run(['tree', '--no-allow'])
        rc, lines_jg = run(['tree', '--no-allow', '--format', 'github'])
        arm('b1-json-key-path-tail-follows-the-context-not-the-hit-class',
            any(ln == 'HIT overlay-word tools/rt4/keys.json:0: ' + ov + ' [json]'
                for ln in lines_jt)
            and any(ln.startswith('::error file=tools/rt4/keys.json,line=1::overlay-word: ' + ov)
                    and ln.endswith(' [json]') for ln in lines_jg)
            and not any(lan_real in ln for ln in lines_jt + lines_jg)
            and any(ln == 'HIT overlay-word tools/rt4/keys.json:0: ' + ov + ' [json $.'
                    + lan_real + '.cmd]' for ln in lines),
            str([ln for ln in lines_jt + lines_jg + lines if 'keys.json' in ln][:4]))
        # contract 1d: a value in a FILE or DIRECTORY name prints in the location
        # only where the value itself would (tier loaded, text); everywhere else
        # the location carries <class:shape> in its place, on the name hit, on a
        # content hit inside that file, and in a CI log where the masked location
        # rides the message (a file property names a file the forge anchors to)
        home_plain = P_MAC + PLAIN_USER + '/'
        mail_plain = PLAIN_USER + '@gmail' + '.com'
        masked_lan_md = 'n/<lan-addr:' + shape_of(lan_real) + '>.md'
        masked_mail_txt = 'n/<email-personal:' + shape_of(mail_plain) + '>.txt'
        masked_home_dir = 'n<home-mac:' + shape_of(home_plain) + '>notes.md'
        arm('b1-tier-loaded-text-shows-a-file-name-control',
            any(ln == 'HIT lan-addr n/' + lan_real + '.md:0: ' + lan_real + ' [name]'
                for ln in lines)
            and any(ln == 'HIT cgnat-addr n/Users/' + PLAIN_USER + '/notes.md:1: ' + cg
                    for ln in lines))
        rc, lines_nt = run(['tree', '--no-allow'])
        arm('b1-no-tier-text-masks-a-value-in-a-file-or-directory-name',
            any(ln == 'HIT lan-addr ' + masked_lan_md + ':0: ' + mask_shape(lan_real)
                + ' [name]' for ln in lines_nt)
            and any(ln == 'HIT email-personal ' + masked_mail_txt + ':0: '
                    + mask_shape(mail_plain) + ' [name]' for ln in lines_nt)
            and any(ln == 'HIT home-mac ' + masked_home_dir + ':0: ' + mask_shape(home_plain)
                    + ' [name]' for ln in lines_nt)
            and not any(lan_real in ln or PLAIN_USER in ln for ln in lines_nt),
            str([ln for ln in lines_nt if ln.startswith('HIT') and 'n/' in ln][:4]))
        arm('b1-no-tier-a-content-hit-location-is-masked-too',
            any(ln == 'HIT cgnat-addr ' + masked_home_dir + ':1: ' + mask_shape(cg)
                for ln in lines_nt))
        rc, lines_nt = run(['tree', '--no-allow', '--format', 'github'])
        arm('b1-no-tier-github-a-masked-location-rides-the-message-not-the-file-property',
            any(ln == '::error ::lan-addr ' + masked_lan_md + ':0: ' + mask_shape(lan_real)
                + ' [name]' for ln in lines_nt)
            and any(ln == '::error ::cgnat-addr ' + masked_home_dir + ':1: ' + mask_shape(cg)
                    for ln in lines_nt)
            and not any(ln.startswith('::error file=n/') or lan_real in ln or PLAIN_USER in ln
                        for ln in lines_nt),
            str([ln for ln in lines_nt if 'n/' in ln or 'n<' in ln][:4]))
        # contract 2: a private match inside a path is redacted on every printed line
        arm('private-path-redacted', any(ln.startswith('REPORT style-dash docs/' + RED1
                                                       + '-notes.md:2') for ln in lines)
            and any(ln.startswith('HIT private#1@host docs/' + RED1 + '-notes.md:0 [name]')
                    for ln in lines)
            and not any(PW in ln for ln in lines))
        rc, lines = run(['tree', '--no-allow', '--format', 'github'], penv)
        arm('github-format-private', any(
            ln == '::error ::private pattern #1 docs/' + RED1 + '-notes.md:0 [name]'
            for ln in lines)
            and any('::error file=src/lib.rs,line=1::private pattern #1' == ln
                    for ln in lines) and not any(PW in ln for ln in lines))
        arm('b1-tier-loaded-github-masks-a-file-name-the-tier-does-not-know',
            any(ln == '::error ::lan-addr ' + masked_lan_md + ':0: ' + mask_shape(lan_real)
                + ' [name]' for ln in lines)
            and not any(lan_real in ln or PLAIN_USER in ln for ln in lines),
            str([ln for ln in lines if 'n/' in ln or 'n<' in ln][:4]))
        # contract 3: missing private source
        rc, lines = run(['tree', '--no-allow'])
        arm('private-missing-loud', rc == EXIT_HIT and any(
            ln.startswith('PRIVATE PATTERNS NOT LOADED') for ln in lines))
        rc, lines = run(['tree', '--no-allow', '--require-private'])
        arm('require-private-norun', rc == EXIT_NORUN)
        rc, lines = run(['tree', '--no-allow', '--no-private'], penv)
        arm('no-private-ignores-loaded-patterns', not any(ln.startswith('HIT private')
                                                          for ln in lines)
            and any('private=NOT-LOADED' in ln for ln in lines))
        arm('no-private-and-require-usage', run(['tree', '--no-private', '--require-private'])[0]
            == EXIT_USAGE)
        rc, lines = run(['tree', '--no-allow', '--require-private'],
                        dict(env, LEAK_PATTERNS_FILE=os.path.join(tmp, 'absent')))
        arm('private-file-env-missing-norun', rc == EXIT_NORUN)
        # contract 4: malformed pattern names the index only
        bad = 'word:' + PW + '\n[[:alpha:]]\n'
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS=bad))
        arm('private-malformed-index-only', rc == EXIT_NORUN and any(
            'private pattern #2 does not compile' in ln for ln in lines)
            and not any('alpha' in ln for ln in lines))
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS='(a|\n'))
        arm('private-unbalanced-norun', rc == EXIT_NORUN)
        # a tagged hit reports its category, an untagged one does not
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS='word:' + PW))
        arm('private-untagged-label', any(ln.startswith('HIT private#1 src/lib.rs:1')
                                          for ln in lines))
        # the default file under the home directory is the third source
        cfg = os.path.join(home, '.config', 'cerulion')
        os.makedirs(cfg)
        with open(os.path.join(cfg, 'leak-patterns.txt'), 'w') as fh:
            fh.write('word:' + PW + ' @device\n')
        os.chmod(os.path.join(cfg, 'leak-patterns.txt'), 0o644)
        rc, lines = run(['tree', '--no-allow'])
        arm('private-default-file-and-mode-warning',
            any('private=LOADED(1,default-file)' in ln for ln in lines)
            and any(ln.startswith('WARNING') and 'chmod 600' in ln for ln in lines)
            and not any(home in ln for ln in lines))
        os.remove(os.path.join(cfg, 'leak-patterns.txt'))
        # contract 5: bad ref, zero units
        rc, lines = run(['tree', '--ref', 'no-such-ref', '--no-allow'])
        arm('bad-ref-norun', rc == EXIT_NORUN)
        with open(os.path.join(tmp, 'list'), 'w') as fh:
            fh.write('absent/file.txt\n')
        rc, lines = run(['tree', '--files-from', os.path.join(tmp, 'list'), '--no-allow'])
        arm('zero-units-norun', rc == EXIT_NORUN)
        rc, lines = run(['tree', '--no-allow'], cwd=tmp)
        arm('outside-worktree-norun', rc == EXIT_NORUN)
        # contract 6: unknown argument, anything after --self-test
        arm('unknown-arg-usage', run(['tree', '--bogus'])[0] == EXIT_USAGE)
        arm('unknown-mode-usage', run(['sweep'])[0] == EXIT_USAGE)
        arm('self-test-trailing-usage', run(['--self-test', 'x'])[0] == EXIT_USAGE)
        arm('no-mode-usage', run([])[0] == EXIT_USAGE)
        # contract 7: a dead control exits 3
        lines = []
        rc = main_inner(['tree', '--no-allow'], repo, env, lines.append, argv0,
                        neuter='home-mac')
        arm('dead-control-norun', rc == EXIT_NORUN and any('CONTROL FAILED' in ln and
                                                          'home-mac' in ln for ln in lines))
        lines = []
        rc = main_inner(['tree', '--no-allow'], repo, penv, lines.append, argv0,
                        neuter='private-canary')
        arm('dead-private-canary-norun', rc == EXIT_NORUN and any(
            'CONTROL FAILED' in ln and 'private-canary' in ln for ln in lines))
        # --- untracked -------------------------------------------------------------
        with open(os.path.join(repo, 'untracked.md'), 'w') as fh:
            fh.write('scratch on ' + PW + '\n')
        got0 = hits(run(['tree', '--no-allow'], penv)[1])
        got1 = hits(run(['tree', '--no-allow', '--untracked'], penv)[1])
        arm('untracked-only-with-flag', ('private#1@host', 'untracked.md', 1) in got1
            and not any(p == 'untracked.md' for _, p, _ in got0))
        surfaces.add('untracked')
        os.remove(os.path.join(repo, 'untracked.md'))
        # --- diff: staged added line, new-side line number; range ---------------------
        with open(os.path.join(repo, 'src/lib.rs'), 'a') as fh:
            fh.write('// later: measured on ' + PW + '\n')
        git.run(['add', 'src/lib.rs'])
        got = hits(run(['diff', '--staged', '--no-allow'], penv)[1])
        arm('diff-staged-added-line', ('private#1@host', 'src/lib.rs', 5) in got
            and not any(p == 'README.md' for _, p, _ in got))
        surfaces.add('staged')
        git.run(['commit', '-q', '-m', 'second'])
        git.run(['checkout', '-q', '-b', 'feature'])
        with open(os.path.join(repo, 'new.md'), 'w') as fh:
            fh.write('line one\nsee ' + P_MAC + SU + '/x\n')
        with open(os.path.join(repo, 'blob/c.bin'), 'wb') as fh:
            fh.write(b'\0' + PW.encode() + b' in a new blob\0')
        git.run(['add', '-A'])
        git.run(['commit', '-q', '-m', 'feature work'])
        got = hits(run(['diff', '--range', 'main..feature', '--no-allow'], penv)[1])
        arm('diff-range-added-lines', ('home-mac', 'new.md', 2) in got
            and ('private#1@host', 'blob/c.bin', 0) in got
            and not any(p == 'src/lib.rs' for _, p, _ in got))
        rc, lines = run(['diff', '--no-allow'])
        arm('diff-needs-selector-usage', rc == EXIT_USAGE)
        # --- names: paths, symlink target, branch, range -----------------------------
        rc, lines = run(['names', '--no-allow', '--branch', 'fix/' + PW + '-thing'], penv)
        got = hits(lines)
        arm('names-path-dir-branch', ('private#1@host', 'docs/' + RED1 + '-notes.md', 0) in got
            and ('private#1@host', RED1 + '/x.txt', 0) in got
            and ('private#1@host', 'branch', 0) in got
            and ('private#2@login', 'link', 0) in got)
        arm('names-content-not-scanned', not any(p == 'README.md' for _, p, _ in got))
        surfaces.update(('file-name', 'dir-name', 'symlink', 'branch'))
        with open(os.path.join(repo, PW + '-b.txt'), 'w') as fh:
            fh.write('x\n')
        with open(os.path.join(repo, 'n', cg + '-b.md'), 'w') as fh:
            fh.write('x\n')
        git.run(['add', '-A'])
        git.run(['commit', '-q', '-m', 'add a file'])
        got = hits(run(['names', '--range', 'main..feature', '--no-allow'], penv)[1])
        arm('names-range-added-paths', ('private#1@host', RED1 + '-b.txt', 0) in got
            and not any(p == 'docs/' + RED1 + '-notes.md' for _, p, _ in got))
        rc, lines = run(['names', '--range', 'main..feature', '--no-allow', '--format',
                         'github'])
        arm('b1-names-range-github-masks-an-added-file-name-without-the-tier',
            rc == EXIT_HIT
            and any(ln == '::error ::cgnat-addr n/<cgnat-addr:' + shape_of(cg) + '>-b.md:0: '
                    + mask_shape(cg) + ' [name]' for ln in lines)
            and not any(cg in ln for ln in lines), str(lines[:4]))
        rc, lines = run(['names', '--no-allow', '--branch-env', 'LG_BRANCH'],
                        dict(penv, LG_BRANCH='topic/' + PW))
        arm('names-branch-env', ('private#1@host', 'branch', 0) in hits(lines))
        # --- messages: subject, body with a pasted log line, identities ---------------
        mail_env = dict(env, GIT_AUTHOR_EMAIL=SU + '@gmail' + '.com',
                        GIT_AUTHOR_NAME=PERSON,
                        GIT_COMMITTER_EMAIL=PERSON.lower().replace(' ', '.') + '@'
                        + PROJECT_DOMAIN)
        Git(repo, mail_env).run(['commit', '-q', '--allow-empty', '-m',
                                 'perf: measured on ' + PW + '\n\nSquashed:\n\n* fix '
                                 + DASH_EM + ' thing\n* log: robot=' + ESC + '[1m' + SH + '.local'
                                 + ESC + '[0m ok\n* path ' + TILDE + '/' + SU + '-notes\n'
                                 '* reviewed with ' + PERSON + '\n'])
        rc, lines = run(['messages', '--range', 'main..feature', '--no-allow'], penv)
        got = hits(lines)
        arm('messages-subject-body', any(c == 'private#1@host' and p.startswith('commit:')
                                         and n == 1 for c, p, n in got)
            and any(c == 'style-dash' and n == 5 for c, p, n in got)
            and any(c == 'mdns-local' and n == 6 for c, p, n in got)
            and any(c == 'home-tilde' and n == 7 for c, p, n in got))
        arm('messages-identity-policy', any(c == 'email-personal' and p.endswith('author email')
                                            for c, p, n in got)
            and any(c == 'email-personal' and p.endswith('committer email') for c, p, n in got)
            and any(c == 'identity-email' and p.endswith('author email') for c, p, n in got)
            and any(c == 'private#3@person' and p.endswith('committer email')
                    for c, p, n in got))
        arm('messages-author-name-is-not-a-person-leak', not any(
            c == 'private#3@person' and p.endswith(' name') for c, p, n in got)
            and any(c == 'private#3@person' and n == 8 and p.startswith('commit:')
                    and ' ' not in p for c, p, n in got))
        ident_hits = set(p for c, p, n in got if c == 'identity-email')
        arm('messages-noreply-identity-passes', len(ident_hits) == 2 and all(
            p.split()[0] == sorted(ident_hits)[0].split()[0] for p in ident_hits),
            str(ident_hits))
        surfaces.update(('commit-subject', 'commit-body', 'author-email', 'committer-email'))
        # --- private entry FORMS: every arm above loads word: entries only, while a real
        # list is mostly text: and regex entries. A second scratch repository drives
        # both forms through the tree scan (the combined slow regex) and through the
        # identity name scan (the per-pattern slow loop, which skips person entries).
        repo2 = os.path.join(tmp, 'repo2')
        os.makedirs(repo2)
        opt = SL + 'opt' + SL + SU
        _write_files(repo2, {
            't.md': ('cd ' + opt + '-scratch\nx' + opt + 'y\nnothing here\n').encode('utf-8'),
            'r.md': ('ssh ' + SU + '@' + SH + '\nwarning: rmw_' + SU + '@0.1.0: text\n'
                     + SU + '@0.2.0\n' + SU + '@' + _addr(10, 77, 13, 9) + '\n').encode('utf-8')})
        name_env = dict(env, GIT_AUTHOR_NAME=PERSON)
        git2 = Git(repo2, name_env)
        git2.run(['init', '-q'])
        git2.run(['add', '-A'])
        git2.run(['commit', '-q', '-m', 'one'])
        git2.run(['commit', '-q', '--allow-empty', '-m', 'two'])
        forms_env = dict(env, LEAK_PATTERNS=(
            'text:' + opt + ' @login\n'
            + '(?<![A-Za-z0-9_-])' + SU + '@(?![0-9]+[.][0-9]+[.][0-9]+(?![0-9.])) @login\n'
            + '(?<![a-z])' + PERSON.split()[0] + '(?![a-z]) @host\n'))
        rc, lines = run(['tree', '--no-allow'], forms_env, repo2)
        got = hits(lines)
        arm('private-text-entry-substring', rc == EXIT_HIT
            and ('private#1@login', 't.md', 1) in got and ('private#1@login', 't.md', 2) in got
            and ('private#1@login', 't.md', 3) not in got)
        arm('private-regex-entry-bounded-login', ('private#2@login', 'r.md', 1) in got
            and ('private#2@login', 'r.md', 4) in got
            and ('private#2@login', 'r.md', 2) not in got
            and ('private#2@login', 'r.md', 3) not in got)
        arm('private-text-regex-output-redacted', not any(
            opt in ln or (SU + '@' + SH) in ln for ln in lines)
            and any(ln.startswith('HIT login-at-host r.md:1: ' + WITHHELD) for ln in lines))
        tree_bare = bare_private(lines)
        rc, lines = run(['messages', '--range', 'HEAD~1..HEAD', '--no-allow'], forms_env, repo2)
        arm('private-text-regex-hit-line-carries-no-text', tree_bare and bare_private(lines))
        arm('private-regex-entry-in-identity-name', rc == EXIT_HIT and any(
            c == 'private#3@host' and p.endswith('author name') for c, p, n in hits(lines))
            and not any(PERSON.split()[0] in ln for ln in lines))
        with open(os.path.join(tmp, 'allow-mail'), 'w') as fh:
            fh.write(SU + '@gmail' + '.com\n')
        got2 = hits(run(['messages', '--range', 'main..feature', '--no-allow',
                         '--allow-email-file', os.path.join(tmp, 'allow-mail')], penv)[1])
        arm('messages-allow-email-file', not any(
            c == 'identity-email' and p.endswith('author email') for c, p, n in got2))
        rc, lines = run(['messages', '--range', 'main..feature', '--pr-title-env', 'T',
                         '--pr-body-env', 'B', '--no-allow'],
                        dict(penv, T='fix on ' + PW, B='body\nbox ' + P_LINUX + SU + '/z\n'))
        got = hits(lines)
        arm('messages-pr-title-body-env', ('private#1@host', 'pr-title', 1) in got
            and ('home-linux', 'pr-body', 2) in got)
        surfaces.update(('pr-title', 'pr-body'))
        with open(os.path.join(tmp, 'msg'), 'w') as fh:
            fh.write('# comment with ' + PW + '\nfeat: ok\n\nbody ' + PW + '\n'
                     '# ------------------------ >8 ------------------------\n'
                     'diff ' + P_MAC + SU + '/x\n')
        rc, lines = run(['messages', '--message-file', os.path.join(tmp, 'msg'), '--no-allow'],
                        penv)
        got = hits(lines)
        # a message FILE outside the hook has no editor signal: it is scanned as
        # written, comment lines included, and a pasted scissors line followed by
        # anything but git's own block is ordinary content
        arm('messages-file-verbatim-comments-and-pasted-scissors',
            ('private#1@host', 'message', 1) in got and ('private#1@host', 'message', 4) in got
            and ('home-mac', 'message', 6) in got and ('private#2@login', 'message', 6) in got
            and len(got) == 4)
        rc, lines = run(['messages', '--no-allow'])
        arm('messages-nothing-norun', rc == EXIT_NORUN)
        rc, lines = run(['messages', '--range', 'feature..feature', '--require-commits',
                         '--no-allow'])
        arm('messages-zero-commits-norun', rc == EXIT_NORUN)
        rc, lines = run(['messages', '--range', 'main..nowhere', '--no-allow'])
        arm('messages-bad-range-norun', rc == EXIT_NORUN)
        rc, lines = run(['messages', '--range', 'main..feature', '--no-allow'],
                        dict(penv, LEAK_PATTERNS=private_env))
        arm('messages-private-redacted', not any(PW.lower() in ln.lower() for ln in lines))
        # --- media -----------------------------------------------------------------------
        git.run(['checkout', '-q', 'main'])
        rc, lines = run(['media', '--no-allow'], penv)
        got = hits(lines)
        want = {('media-meta', 'media/t.png'), ('media-meta', 'media/c.gif'),
                ('media-meta', 'media/app.gif'),
                ('media-meta', 'media/g.jpg'), ('media-meta', 'media/l.mp4'),
                ('private#1@host', 'media/t.png'), ('private#1@host', 'media/c.gif'),
                ('private#1@host', 'media/g.jpg')}
        gotcp = set((c, p) for c, p, _ in got)
        arm('media-walkers-find-metadata', rc == EXIT_HIT and want <= gotcp,
            str(want - gotcp))
        arm('media-clean-controls', not any(p in ('media/clean.png', 'media/loop.gif',
                                                  'media/mdat.mp4', 'media/clean.jpg')
                                            for _, p, _ in got))
        arm('media-summary-counts', any('media=10' in ln and 'gif=3' in ln and 'png=3' in ln
                                        and 'jpeg=2' in ln and 'bmff=2' in ln for ln in lines))
        surfaces.update(('png-text', 'gif-comment', 'jpeg-exif', 'mp4-box'))
        # a container's info line and its hit both name the file: masked without
        # the tier and in a CI log, in clear on a terminal with the tier loaded
        masked_png = 'n/<lan-addr:' + shape_of(lan_real) + '>.png'
        arm('b1-media-tier-loaded-text-shows-the-container-name-control',
            any(ln.startswith('media png n/' + lan_real + '.png: png-tEXt') for ln in lines)
            and ('media-meta', 'n/' + lan_real + '.png', 0) in got
            and rc == EXIT_HIT)
        rc, lines_m = run(['media', '--no-allow'])
        arm('b1-media-info-line-and-hit-location-mask-a-name-without-the-tier',
            any(ln.startswith('media png ' + masked_png + ': png-tEXt') for ln in lines_m)
            and any(ln == 'HIT media-meta ' + masked_png + ':0: png-tEXt len=14 [container]'
                    for ln in lines_m)
            and not any(lan_real in ln for ln in lines_m), str(lines_m[:6]))
        rc, lines_m = run(['media', '--no-allow', '--format', 'github'])
        arm('b1-media-github-a-masked-container-location-rides-the-message',
            any(ln == '::error ::media-meta ' + masked_png + ':0: png-tEXt len=14 [container]'
                for ln in lines_m)
            and not any(ln.startswith('::error file=n/') or lan_real in ln for ln in lines_m),
            str(lines_m[:6]))
        with open(os.path.join(repo, 'media/x.pdf'), 'wb') as fh:
            fh.write(b'%PDF-1.4 fake\n')
        with open(os.path.join(repo, 'media/bad.png'), 'wb') as fh:
            fh.write(b'not a png at all')
        git.run(['add', '-A'])
        rc, lines = run(['media', '--staged', '--no-allow'])
        got = hits(lines)
        arm('media-unparsed-inventory', ('media-unparsed', 'media/x.pdf', 0) in got
            and ('media-unparsed', 'media/bad.png', 0) in got)
        git.run(['rm', '-q', '--cached', 'media/x.pdf', 'media/bad.png'])
        os.remove(os.path.join(repo, 'media/x.pdf'))
        os.remove(os.path.join(repo, 'media/bad.png'))
        # --- allowlist and pragma ------------------------------------------------------
        def allow_run(text, extra=(), e=None):
            with open(os.path.join(tmp, 'allow'), 'w') as fh:
                fh.write(text)
            return run(['tree', '--allow', os.path.join(tmp, 'allow')] + list(extra), e or env)

        rc, lines = allow_run('cfg/x.xml | mdns-local\n')
        arm('allow-two-fields-refused', rc == EXIT_USAGE)
        rc, lines = allow_run('cfg/x.xml | mdns-local | short\n')
        arm('allow-short-reason-refused', rc == EXIT_USAGE)
        rc, lines = allow_run('cfg/x.xml | no-such-class | a long enough reason\n')
        arm('allow-unknown-class-refused', rc == EXIT_USAGE)
        rc, lines = allow_run('cfg/x.xml | private | a long enough reason\n')
        arm('allow-bare-private-refused', rc == EXIT_USAGE)
        rc, lines = allow_run('** | home-mac | a long enough reason\n')
        arm('allow-match-all-refused', rc == EXIT_USAGE)
        rc, lines = allow_run('cfg/x.xml | mdns-local | a test fixture host name\n')
        arm('allow-suppresses', not any(p == 'cfg/x.xml' for _, p, _ in hits(lines))
            and any('allowlist=1' in ln and 'suppressed=1' in ln for ln in lines))
        rc, lines = allow_run('gone/*.xml | mdns-local | a test fixture host name\n')
        arm('allow-stale-fails', any('(stale)' in ln for ln in lines) and rc == EXIT_HIT)
        rc, lines = allow_run('README.md | mdns-local | a test fixture host name\n')
        arm('allow-unused-fails', any('(unused)' in ln for ln in lines))
        rc, lines = allow_run('README.md | private@person | the names stay by ruling\n', (),
                              penv)
        arm('allow-private-category', not any(c == 'private#3@person' for c, _, _ in hits(lines))
            and any(c == 'private#2@login' and p == 'README.md' for c, p, _ in hits(lines)))
        rc, lines = allow_run('README.md | private@person | the names stay by ruling\n')
        arm('allow-private-entry-inert-without-patterns', not any('(unused)' in ln
                                                                for ln in lines))
        rc, lines = allow_run('README.md | private@person | the names stay by ruling\n',
                              (), penv)
        arm('allow-private-entry-not-judged-in-media', not any('(unused)' in ln for ln in run(
            ['media', '--allow', os.path.join(tmp, 'allow')], penv)[1]))
        # --- a private@<tag> waiver whose tag no loaded entry carries cannot be
        # judged unused: what the secret holds must not decide whether every push
        # to main is red. Its path check still runs (it depends on the tree, not
        # on the secret); a waiver for a generic class, and a private waiver whose
        # tag IS loaded, keep both checks. A clean scratch repository, so rc is exact.
        repo4 = os.path.join(tmp, 'repo4')
        os.makedirs(repo4)
        _write_files(repo4, {'README.md': 'contact the maintainers\n', 'ok.md': 'clean\n'})
        git4 = Git(repo4, env)
        git4.run(['init', '-q'])
        git4.run(['add', '-A'])
        git4.run(['commit', '-q', '-m', 'one'])
        host_only = dict(env, LEAK_PATTERNS='word:' + PW + ' @host\n')

        def allow_run4(text, e):
            with open(os.path.join(tmp, 'allow4'), 'w') as fh:
                fh.write(text)
            return run(['tree', '--allow', os.path.join(tmp, 'allow4')], e, repo4)

        person_waiver = 'README.md | private@person | the names stay by ruling\n'
        rc, lines = allow_run4(person_waiver, host_only)
        arm('n1-private-waiver-with-no-entry-of-its-tag-is-a-note-not-a-failure',
            rc == EXIT_OK
            and any(ln == 'ALLOWLIST NOTE: allowlist line 1 unused: no person entry loaded'
                    for ln in lines)
            and not any(ln.startswith('ALLOWLIST:') for ln in lines),
            'rc=%d %s' % (rc, [ln for ln in lines if 'ALLOWLIST' in ln]))
        rc, lines = allow_run4(person_waiver, penv)
        arm('n1-private-waiver-whose-tag-is-loaded-and-excused-nothing-still-fails',
            rc == EXIT_HIT and any('allowlist line 1 excused nothing (unused)' in ln
                                   for ln in lines))
        rc, lines = allow_run4(person_waiver + 'ok.md | mdns-local | a test fixture host name\n',
                               host_only)
        arm('n1-generic-waiver-keeps-the-unused-check-beside-a-noted-private-one',
            rc == EXIT_HIT
            and any('allowlist line 2 excused nothing (unused)' in ln for ln in lines)
            and any('allowlist line 1 unused: no person entry loaded' in ln for ln in lines))
        rc, lines = allow_run4('gone.md | private@person | the names stay by ruling\n',
                               host_only)
        arm('n1-private-waiver-for-a-missing-file-is-still-stale', rc == EXIT_HIT
            and any('allowlist line 1 matches no file in the tree (stale)' in ln
                    for ln in lines))
        with open(os.path.join(repo, 'prag.md'), 'w') as fh:
            fh.write('a ' + P_MAC + SU + '/x  <!-- ' + PRAGMA_WORD + ' allow home-mac a doc'
                     ' example path -->\n'
                     'b ' + P_MAC + SU + '/y  <!-- ' + PRAGMA_WORD + ' allow home-mac short -->\n'
                     'c ' + PW + ' <!-- ' + PRAGMA_WORD + ' allow private#1@host a doc example'
                     ' path -->\n'
                     'd ' + P_MAC + SU + '/z  <!-- ' + PRAGMA_WORD + ' allow lan-addr a doc'
                     ' example path -->\n')
        git.run(['add', 'prag.md'])
        rc, lines = run(['tree', '--no-allow'], penv)
        got = hits(lines)
        arm('pragma-valid-suppresses', ('home-mac', 'prag.md', 1) not in got
            and ('home-mac', 'prag.md', 2) in got and ('home-mac', 'prag.md', 4) in got
            and any('pragmas=1' in ln and 'pragmas_refused=2' in ln for ln in lines))
        arm('pragma-private-refused', ('private#1@host', 'prag.md', 3) in got)
        Git(repo, env).run(['commit', '-q', '-m', 'pragma ' + P_MAC + SU + '/x <!-- '
                            + PRAGMA_WORD + ' allow home-mac a long enough reason -->'])
        got = hits(run(['messages', '--range', 'main~1..main', '--no-allow'])[1])
        arm('pragma-ignored-in-messages', any(c == 'home-mac' for c, _, _ in got))
        # --hard escalates a report class
        rc, lines = run(['tree', '--no-allow', '--hard', 'style-dash'])
        arm('hard-escalation', any(ln.startswith('HIT style-dash style.md') for ln in lines))
        # --- oversize stream --------------------------------------------------------------
        global TEXT_CAP
        saved_cap = TEXT_CAP
        TEXT_CAP = 64
        try:
            rc, lines = run(['tree', '--no-allow'], penv)
            got = hits(lines)
            arm('oversize-stream-raw-view', ('private#1@host', 'forms.txt', 1) in got
                and any('oversize=' in ln for ln in lines))
        finally:
            TEXT_CAP = saved_cap
        # --- hook mode ----------------------------------------------------------------------
        with open(os.path.join(repo, 'hooked.md'), 'w') as fh:
            fh.write('on ' + PW + '\n')
        git.run(['add', 'hooked.md'])
        rc, lines = run(['hook', 'pre-commit', '--no-allow'], penv)
        arm('hook-pre-commit-blocks', rc == EXIT_HIT and any(
            ln.startswith('HIT private#1@host hooked.md:1') for ln in lines))
        git.run(['reset', '-q', 'hooked.md'])
        os.remove(os.path.join(repo, 'hooked.md'))
        with open(os.path.join(repo, 'clean2.md'), 'w') as fh:
            fh.write('clean\n')
        git.run(['add', 'clean2.md'])
        rc, lines = run(['hook', 'pre-commit', '--no-allow'], penv)
        arm('hook-pre-commit-passes', rc == EXIT_OK and any('leak-guard pre-commit: OK' in ln
                                                            for ln in lines))
        rc, lines = run(['hook', 'pre-commit', '--no-allow'])
        arm('hook-note-when-private-missing', rc == EXIT_OK and any(
            'private patterns NOT loaded' in ln for ln in lines))
        # a staged file whose NAME is a value, through the hook with no list installed
        with open(os.path.join(repo, 'n', cg + '-staged.md'), 'w') as fh:
            fh.write('clean\n')
        git.run(['add', 'n/' + cg + '-staged.md'])
        rc, lines = run(['hook', 'pre-commit', '--no-allow'])
        arm('b1-hook-pre-commit-masks-a-staged-file-name-without-the-tier', rc == EXIT_HIT
            and any(ln == 'HIT cgnat-addr n/<cgnat-addr:' + shape_of(cg) + '>-staged.md:0: '
                    + mask_shape(cg) + ' [name]' for ln in lines)
            and not any(cg in ln for ln in lines), str(lines[:4]))
        git.run(['reset', '-q', 'n/' + cg + '-staged.md'])
        os.remove(os.path.join(repo, 'n', cg + '-staged.md'))
        rc, lines = run(['hook', 'commit-msg', os.path.join(tmp, 'msg'), '--no-allow'], penv)
        arm('hook-commit-msg-blocks', rc == EXIT_HIT)
        with open(os.path.join(tmp, 'msg2'), 'w') as fh:
            fh.write('feat: clean message\n')
        rc, lines = run(['hook', 'commit-msg', os.path.join(tmp, 'msg2'), '--no-allow'], penv)
        arm('hook-commit-msg-passes', rc == EXIT_OK)
        rc, lines = run(['hook', 'commit-msg', os.path.join(tmp, 'msg2'), '--no-allow'],
                        dict(penv, GIT_AUTHOR_EMAIL=SU + '@gmail' + '.com'))
        arm('hook-commit-msg-identity', rc == EXIT_HIT and any(
            ln.startswith('HIT identity-email author email:1') for ln in lines))
        guard_root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(argv0)),
                                                   '..', '..'))
        # --- redaction is decided on the LINE, escapes and format characters are
        # decoded, the message hook mirrors git, lists and printed lines are safe ----
        OV_SHORT = 'vex' + 'moor'
        OV_LONG = OV_SHORT + '-' + 'kessel4'
        PLAIN = PLAIN_USER
        ZWSP, SHY, WJ = chr(0x200B), chr(0xAD), chr(0x2060)
        repo3 = os.path.join(tmp, 'repo3')
        os.makedirs(repo3)
        penv3 = dict(env, LEAK_PATTERNS=private_env + 'word:' + OV_SHORT + ' @host\nword:'
                     + OV_LONG + ' @host\n')
        ansi_mac = 'path=' + P_MAC + ESC + '[1m' + SU + ESC + '[0m/run'
        pct_users = '%2F' + 'Users' + '%2F'
        _write_files(repo3, {
            'r1/ansi-mac.log': ansi_mac + '\n',
            'r1/ansi-linux.log': 'cwd=' + P_LINUX + ESC + '[32m' + SU + ESC + '[0m/work\n',
            'r1/ansi-win.txt': 'C:' + BSL + 'Users' + BSL + ESC + '[1m' + SU + ESC + '[0m' + BSL
                               + 'x\n',
            'r1/overlap.md': 'run: ssh qzop@' + OV_LONG + ' uptime\n',
            'r1/person.md': 'path=' + P_LINUX + PERSON + '/run\n',
            'r1/bracket.md': 'ping [' + SH[0] + ']' + SH[1:] + '.local\n',
            'r1/plain.md': 'see ' + P_MAC + PLAIN + '/src\n',
            'r1/email.md': 'mail ' + ESC + '[4m' + SU + '.thandry' + '@gmail' + '.com\n',
            OV_LONG + '/notes.md': 'clean\n',
            'r1/' + PW[:3] + ZWSP + PW[3:] + '.md': 'clean\n',
            'r2/lit.rs': 'pub const M: &str = "measured on:' + BSL + 'n' + PW + ' overnight";\n',
            'r2/tab.py': "x = 'a" + BSL + 't' + PW + "'\n",
            'r2/color.sh': 'printf "' + BSL + '033[1m' + PW + BSL + '033[0m"\n',
            'r2/hex.rs': 'let s = "' + BSL + 'x1b[32m' + PW + '";\n',
            'r2/pct.md': '?path=' + pct_users + SU + '%2Fwork\n',
            'r2/url.md': 'x%20' + PW + '%20y\n',
            'r2/boot.log': '{"msg":"boot ok' + BSL + 'n' + PW + '"}\n',
            'r2/rx.rs': 'let r = Regex::new(r"' + BSL + 'b' + PW + BSL + 'b").unwrap();\n',
            'r2/nested.log': 'j={"a":"' + BSL + BSL + 'n' + PW + '"}\n',
            'r2/glued.txt': 'q' + PW + '\n',
            'r3/zwsp.md': 'Measured on ' + PW[:3] + ZWSP + PW[3:] + ' overnight.\n',
            'r3/shy.md': 'Measured on ' + PW[:3] + SHY + PW[3:] + ' overnight.\n',
            'r3/wj.md': 'Measured on ' + PW[:3] + WJ + PW[3:] + ' overnight.\n',
            'r3/home.md': 'see ' + P_MAC + SU[:2] + ZWSP + SU[2:] + '/x\n',
        })
        git3 = Git(repo3, env)
        git3.run(['init', '-q'])
        git3.run(['symbolic-ref', 'HEAD', 'refs/heads/main'])
        git3.run(['add', '-A'])
        git3.run(['commit', '-q', '-m', 'fixture'])
        rc, lines = run(['tree', '--no-allow'], penv3, repo3)
        got = hits(lines)
        leaked = [ln for ln in lines if SU in ln or PW.lower() in ln.lower()
                  or PERSON.split()[0] in ln or 'kessel4' in ln or SH[1:] in ln
                  or ESC in ln or ZWSP in ln or SHY in ln or WJ in ln]
        arm('r1-no-private-text-on-any-line', not leaked, str(leaked[:3]))

        def withheld(cls, path, n=1):
            return any(ln.startswith('HIT %s %s:%d: %s' % (cls, path, n, WITHHELD))
                       for ln in lines)

        arm('r1-ansi-inside-home-segment-withheld',
            withheld('home-mac', 'r1/ansi-mac.log') and withheld('home-linux', 'r1/ansi-linux.log')
            and withheld('home-win', 'r1/ansi-win.txt')
            and all(('private#2@login', p, 1) in got for p in (
                'r1/ansi-mac.log', 'r1/ansi-linux.log', 'r1/ansi-win.txt')))
        arm('r1-overlapping-entries-shorter-first-withheld',
            withheld('login-at-host', 'r1/overlap.md')
            and ('private#5@host', 'r1/overlap.md', 1) in got)
        arm('r1-multi-token-entry-cut-by-match-withheld',
            withheld('home-linux', 'r1/person.md') and ('private#3@person', 'r1/person.md', 1)
            in got)
        arm('r1-bracket-spelling-inside-match-withheld',
            withheld('mdns-local', 'r1/bracket.md') and ('private#1@host', 'r1/bracket.md', 1)
            in got)
        arm('r1-email-with-escape-withheld', withheld('email-personal', 'r1/email.md'))
        arm('r1-generic-text-still-shown-when-nothing-private-touches-the-line',
            any(ln.startswith('HIT home-mac r1/plain.md:1: ' + P_MAC + PLAIN + '/')
                for ln in lines))
        arm('r1-path-masked-by-merged-spans-not-entry-order',
            ('private#5@host', '<private#5@host>/notes.md', 0) in got)
        arm('r1-path-split-by-format-character-withheld-per-component',
            ('private#1@host', 'r1/' + RED1, 0) in got)
        rc, lines_gh = run(['tree', '--no-allow', '--format', 'github'], penv3, repo3)
        arm('r1-github-format-withheld', any(
            ln == '::error file=r1/ansi-mac.log,line=1::home-mac: ' + gh_escape(WITHHELD)
            for ln in lines_gh) and not any(SU in ln or ESC in ln for ln in lines_gh))
        # contract 1c over a line NO private pattern touches: a CI log masks the
        # value even with the tier loaded, and the negative control shows that the
        # same detector sees the value where it is allowed (tier loaded, text)
        plain_mac = P_MAC + PLAIN + '/'
        masked_plain = mask_shape(plain_mac)
        arm('b1-tier-loaded-github-format-masks-identity-values', any(
            ln == '::error file=r1/plain.md,line=1::home-mac: ' + gh_escape(masked_plain)
            for ln in lines_gh) and not any(PLAIN in ln for ln in lines_gh))
        arm('b1-tier-loaded-text-format-shows-the-value-control', any(
            ln == 'HIT home-mac r1/plain.md:1: ' + plain_mac for ln in lines)
            and any(PLAIN in ln for ln in lines)
            and not any(masked_plain in ln for ln in lines))
        arm('r2-literal-escape-before-name-is-a-boundary', all(
            ('private#1@host', 'r2/' + f, 1) in got for f in (
                'lit.rs', 'tab.py', 'color.sh', 'hex.rs', 'url.md', 'boot.log', 'rx.rs',
                'nested.log')))
        arm('r2-percent-encoded-home-path', ('private#2@login', 'r2/pct.md', 1) in got
            and withheld('home-mac', 'r2/pct.md'))
        arm('r2-name-glued-to-a-letter-is-still-no-word',
            not any(p == 'r2/glued.txt' for _, p, _ in got))
        arm('r3-format-characters-do-not-split-a-name', all(
            ('private#1@host', 'r3/' + f, 1) in got for f in ('zwsp.md', 'shy.md', 'wj.md')))
        arm('r3-format-character-inside-a-home-segment',
            ('private#2@login', 'r3/home.md', 1) in got and withheld('home-mac', 'r3/home.md'))
        # the same shapes through the hook's diff mode and through messages
        with open(os.path.join(repo3, 'staged.log'), 'w') as fh:
            fh.write(ansi_mac + '\n' + 'seen ' + BSL + 'n' + PW + '\n')
        git3.run(['add', 'staged.log'])
        rc, lines = run(['hook', 'pre-commit', '--no-allow'], penv3, repo3)
        arm('r1-r2-hook-diff-withheld-and-decoded', rc == EXIT_HIT
            and any(ln.startswith('HIT home-mac staged.log:1: ' + WITHHELD) for ln in lines)
            and ('private#1@host', 'staged.log', 2) in hits(lines)
            and not any(SU in ln or ESC in ln for ln in lines))
        git3.run(['reset', '-q', 'staged.log'])
        os.remove(os.path.join(repo3, 'staged.log'))
        git3.run(['commit', '-q', '--allow-empty', '-m',
                  'chore: x\n\n' + ansi_mac + '\nlog: a' + BSL + 't' + PW + '\n'])
        rc, lines = run(['messages', '--range', 'HEAD~1..HEAD', '--pr-body-env', 'B',
                         '--no-allow'],
                        dict(penv3, B='body\n' + ansi_mac + '\nseen ' + PW[:3] + ZWSP + PW[3:]
                             + '\n'), repo3)
        got = hits(lines)
        arm('r1-r2-r3-messages-withheld-decoded-unsplit', rc == EXIT_HIT
            and any(ln.startswith('HIT home-mac commit:') and ln.endswith(':3: ' + WITHHELD)
                    for ln in lines)
            and any(c == 'private#1@host' and p.startswith('commit:') and n == 4
                    for c, p, n in got)
            and any(ln.startswith('HIT home-mac pr-body:2: ' + WITHHELD) for ln in lines)
            and ('private#1@host', 'pr-body', 3) in got
            and not any(SU in ln or ESC in ln or ZWSP in ln for ln in lines))
        # --- the commit-msg hook mirrors what git keeps of the message file ---------
        hash_only = os.path.join(tmp, 'msg-hash')
        with open(hash_only, 'w') as fh:
            fh.write('chore: add a file\n\n# Measured on ' + PW + '\n')
        no_editor = dict(penv, GIT_EDITOR=':')
        rc_m, lines = run(['hook', 'commit-msg', hash_only, '--no-allow'], no_editor)
        rc_e, _ = run(['hook', 'commit-msg', hash_only, '--no-allow'], penv)
        arm('r4-hash-line-scanned-when-git-keeps-it', rc_m == EXIT_HIT
            and ('private#1@host', 'message', 3) in hits(lines))
        arm('r4-hash-line-dropped-when-the-editor-strips-it', rc_e == EXIT_OK)
        git.run(['config', 'commit.cleanup', 'whitespace'])
        rc_w, _ = run(['hook', 'commit-msg', hash_only, '--no-allow'], penv)
        git.run(['config', '--unset', 'commit.cleanup'])
        arm('r4-hash-line-scanned-under-a-keeping-cleanup', rc_w == EXIT_HIT)
        scissors = '# ------------------------ >8 ------------------------\n'
        own_block = os.path.join(tmp, 'msg-own')
        with open(own_block, 'w') as fh:
            fh.write('chore: ok\n\n' + scissors + '# Do not modify or remove the line above.\n'
                     '# Everything below it will be ignored.\ndiff --git a/x b/x\n+see '
                     + P_MAC + SU + '/x\n')
        rc_own_e, _ = run(['hook', 'commit-msg', own_block, '--no-allow'], penv)
        rc_own_m, _ = run(['hook', 'commit-msg', own_block, '--no-allow'], no_editor)
        arm('r4-below-gits-own-scissors-block-dropped-in-the-editor', rc_own_e == EXIT_OK)
        arm('r4-below-scissors-scanned-when-git-keeps-it', rc_own_m == EXIT_HIT)
        pasted = os.path.join(tmp, 'msg-pasted')
        with open(pasted, 'w') as fh:
            fh.write('chore: ok\n\n' + scissors + 'notes from ' + P_MAC + SU + '/x\n')
        rc_p, lines = run(['hook', 'commit-msg', pasted, '--no-allow'], penv)
        arm('r4-pasted-scissors-then-prose-is-content', rc_p == EXIT_HIT
            and ('home-mac', 'message', 4) in hits(lines))
        # --- a private list that is malformed at the entry level fails closed ------
        docs_style = ('word:' + PW + ' @host          # case-insensitive, alphanumeric\n'
                      '                                # boundaries\n'
                      'text:' + P_LINUX + SU + ' @login  # substring, no boundaries\n'
                      '(?<![a-z])lab[0-9] @device      # anything else is a regex\n')
        rc, lines = run(['tree', '--no-allow', '--require-private'],
                        dict(env, LEAK_PATTERNS=docs_style))
        got = hits(lines)
        arm('f1-inline-comments-are-not-part-of-the-entry', rc == EXIT_HIT
            and ('private#1@host', 'src/lib.rs', 1) in got
            and ('private#2@login', 'tools/gen.py', 1) in got
            and any('private=LOADED(3,env)' in ln for ln in lines))
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS='word:' + PW
                                                       + ' @hots\n'))
        arm('f1-tag-outside-the-vocabulary-is-a-no-run-naming-the-index', rc == EXIT_NORUN
            and any('private pattern #1 ends in a tag outside the vocabulary' in ln
                    for ln in lines) and not any('hots' in ln or PW in ln for ln in lines))
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS='word:' + PW
                                                       + ' @Host\n'))
        arm('f1-capitalised-tag-is-the-tag', ('private#1@host', 'src/lib.rs', 1)
            in hits(lines))
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS='﻿word:' + PW
                                                       + ' @host\n'))
        arm('f1-byte-order-mark-does-not-kill-the-first-entry',
            ('private#1@host', 'src/lib.rs', 1) in hits(lines))
        with open(os.path.join(tmp, 'latin'), 'wb') as fh:
            fh.write(b'word:' + PW.encode() + b' @host \xe9\n')
        rc, lines = run(['tree', '--no-allow'], dict(env, LEAK_PATTERNS_FILE=os.path.join(
            tmp, 'latin')))
        arm('f1-non-utf8-list-is-a-clean-no-run', rc == EXIT_NORUN
            and any('is not UTF-8' in ln for ln in lines))
        doc = os.path.join(guard_root, 'docs', 'internals', 'leak-guard.md')
        doc_ok = True
        if os.path.isfile(doc):
            with open(doc, 'r', encoding='utf-8') as fh:
                body = fh.read()
            m = re.search(r'One entry per line.*?```\n(.*?)```', body, re.S)
            plants = ('on examplebox7 today\n', 'cd ' + P_LINUX + 'examplelogin/run\n',
                      'device lab4 up\n')
            with open(os.path.join(tmp, 'docs-list'), 'w') as fh:
                fh.write(m.group(1) if m else '')
            with open(os.path.join(repo3, 'docs-plants.md'), 'w') as fh:
                fh.write(''.join(plants))
            git3.run(['add', 'docs-plants.md'])
            rc, lines = run(['tree', '--no-allow', '--require-private'],
                            dict(env, LEAK_PATTERNS_FILE=os.path.join(tmp, 'docs-list')),
                            repo3)
            got = hits(lines)
            doc_ok = m is not None and rc == EXIT_HIT and all(
                (c, 'docs-plants.md', n) in got for c, n in (
                    ('private#1@host', 1), ('private#2@login', 2), ('private#3@device', 3)))
            git3.run(['rm', '-q', '--cached', 'docs-plants.md'])
            os.remove(os.path.join(repo3, 'docs-plants.md'))
        arm('f1-the-docs-example-list-loads-and-finds-its-own-stand-ins', doc_ok)
        # --- the entry grammar is strict: a line that is not blank, a comment or an
        # exactly well-formed entry refuses the load naming the entry index, the
        # list line and the accepted forms, never the line. Every shape rides
        # SECOND behind a well-formed entry, because a good neighbour used to mask
        # a dead one as LOADED(2), and the plants must appear on no printed line.
        good = 'word:' + PW + ' @host'
        plants = (PW, SU, 'lab[0-9]', 'the lab box', 'extra words')
        shapes = (
            ('words-after-the-tag-word', 'word:' + PW + ' @host the lab box'),
            ('comment-without-a-space-word', 'word:' + PW + ' #lab'),
            ('tag-glued-word', 'word:' + PW + '@host'),
            ('slash-comment-word', 'word:' + PW + ' // lab'),
            ('words-after-the-tag-text', 'text:' + P_LINUX + SU + ' @login extra words'),
            ('comment-without-a-space-text', 'text:' + P_LINUX + SU + ' #lab'),
            ('words-after-the-tag-regex', '(?<![a-z])lab[0-9] @device extra words'),
            ('comment-without-a-space-regex', '(?<![a-z])lab[0-9] #lab'),
            ('prefix-case', 'Word:' + PW + ' @host'),
            ('whitespace-after-the-colon', 'word: ' + PW + ' @host'),
            ('tag-without-its-at', 'word:' + PW + ' host'),
            ('carriage-return', 'word:' + PW + ' @host\r'),
            ('tag-glued-text', 'text:' + P_LINUX + SU + '@login'),
            ('tag-glued-regex', '(?<![a-z])lab[0-9]@device'),
            ('two-tags', 'word:' + PW + ' @host @device'),
            ('tab-inside-the-literal', 'word:' + PW[:3] + '\t' + PW[3:] + ' @host'),
            ('at-inside-a-word-literal', 'word:' + PW + '@x'),
        )

        def refused(lines):
            said = any('private pattern #2 is malformed' in ln and 'list line 2' in ln
                       and 'accepted forms' in ln for ln in lines)
            return said and not any(p in ln for ln in lines for p in plants)

        for name, shape in shapes:
            rc, lines = run(['tree', '--no-allow', '--require-private'],
                            dict(env, LEAK_PATTERNS=good + '\n' + shape + '\n'))
            arm('f2-entry-shape-refused-' + name, rc == EXIT_NORUN and refused(lines),
                'rc=%d %s' % (rc, [ln for ln in lines if ln.startswith('leak_scan')][:1]))
        with open(os.path.join(tmp, 'shape-file'), 'w') as fh:
            fh.write(good + '\n' + shapes[0][1] + '\n')
        rc, lines = run(['tree', '--no-allow', '--require-private'],
                        dict(env, LEAK_PATTERNS_FILE=os.path.join(tmp, 'shape-file')))
        arm('f2-entry-shape-refused-through-the-file-source', rc == EXIT_NORUN
            and refused(lines))
        with open(os.path.join(repo, 'shape.md'), 'w') as fh:
            fh.write('on ' + PW + '\n')
        git.run(['add', 'shape.md'])
        rc, lines = run(['hook', 'pre-commit', '--no-allow'],
                        dict(env, LEAK_PATTERNS=good + '\n' + shapes[0][1] + '\n'))
        arm('f2-entry-shape-refused-in-the-hook', rc == EXIT_NORUN and refused(lines))
        git.run(['reset', '-q', 'shape.md'])
        os.remove(os.path.join(repo, 'shape.md'))
        # the control: every accepted form loads, counts and finds its plant, with
        # and without a tag, with a trailing comment, leading whitespace, a tab
        # before the tag, blank and comment lines between
        forms = ('# a comment line\n\n   \n'
                 'word:' + PW + ' @host    # a comment after the tag\n'
                 '  text:' + P_LINUX + SU + '\t@login\n'
                 'contact\\s+' + PERSON.split()[0] + ' @person\n'
                 'word:' + SH + '\n'
                 'text:' + P_TEMP + 'ab/ # a comment after a body with no tag\n'
                 '(?<![a-z])lab-runner(?![a-z])\n')
        rc, lines = run(['tree', '--no-allow', '--require-private'],
                        dict(env, LEAK_PATTERNS=forms))
        got = hits(lines)
        arm('f2-every-accepted-form-loads-and-matches', rc == EXIT_HIT
            and any('private=LOADED(6,env)' in ln for ln in lines)
            and all(e in got for e in (
                ('private#1@host', 'src/lib.rs', 1), ('private#2@login', 'tools/gen.py', 1),
                ('private#3@person', 'README.md', 3), ('private#4', 'README.md', 1),
                ('private#4', 'cfg/x.xml', 1), ('private#5', 'scripts/run.sh', 1),
                ('private#6', '.github/workflows/x.yml', 2))),
            'rc=%d got=%s' % (rc, sorted(g for g in got if g[0].startswith('private'))))
        # --- a listed path that the scan source does not hold is a NO RUN, and a
        # NUL separated list is never unquoted ----------------------------------------
        quoted = '"n.md"'
        with open(os.path.join(repo3, quoted), 'w') as fh:
            fh.write('measured at ' + P_MAC + SU + '/bench on ' + PW + '\n')
        git3.run(['add', quoted])
        git3.run(['commit', '-q', '-m', 'quoted name'])
        with open(os.path.join(tmp, 'zlist'), 'wb') as fh:
            fh.write(quoted.encode() + b'\0')
        rc, lines = run(['tree', '--files-from', os.path.join(tmp, 'zlist'), '--no-allow'],
                        penv3, repo3)
        arm('w3-nul-list-is-taken-verbatim', rc == EXIT_HIT
            and ('private#2@login', quoted, 1) in hits(lines)
            and not any('skipped=' in ln for ln in lines))
        rc, lines = run(['tree', '--ref', 'HEAD', '--files-from', os.path.join(tmp, 'zlist'),
                         '--no-allow'], penv3, repo3)
        arm('w3-nul-list-against-a-ref', rc == EXIT_HIT
            and ('private#2@login', quoted, 1) in hits(lines))
        with open(os.path.join(tmp, 'zlist2'), 'wb') as fh:
            fh.write(quoted.encode() + b'\0gone/away.md\0')
        for extra in ([], ['--ref', 'HEAD'], ['--staged']):
            rc, lines = run(['tree'] + extra + ['--files-from', os.path.join(tmp, 'zlist2'),
                                                '--no-allow', '--allow-empty'], penv3, repo3)
            arm('w3-listed-path-missing-is-a-no-run%s' % ''.join(extra), rc == EXIT_NORUN
                and any('1 listed path(s) are not in the scan source' in ln for ln in lines)
                and ('private#2@login', quoted, 1) in hits(lines))
        rc, lines = run(['media', '--files-from', os.path.join(tmp, 'zlist2'), '--no-allow',
                         '--allow-empty'], penv3, repo3)
        arm('w3-media-listed-path-missing-is-a-no-run', rc == EXIT_NORUN)
        # --- no printed line can start a workflow command ----------------------------
        forged = 'w2/p\n::warning::forged from a file name\n.png'
        forged2 = 'w2/a' + DASH_EM + 'b\n::notice::forged notice\n.txt'
        forged3 = 'w2/x##[warning]y' + DASH_EM + '.txt'
        forged4 = 'w2' + P_MAC + PLAIN + '/p\n::warning::forged\n.txt'
        forged5 = 'w2/q\n::warning::forged\n.md'
        _write_files(repo3, {forged: _png([]), forged2: b'clean\n', forged3: b'clean\n',
                             forged4: b'clean\n',
                             forged5: ('see ' + P_MAC + PLAIN + '/src\n').encode('utf-8')})
        git3.run(['add', '-A'])
        git3.run(['commit', '-q', '-m', 'forged names'])
        rc, lines = run(['media', '--no-allow'], penv3, repo3)
        rc2, lines2 = run(['names', '--no-allow', '--format', 'github'], penv3, repo3)
        rc3, lines3 = run(['tree', '--no-allow', '--format', 'github'], penv3, repo3)
        every = lines + lines2 + lines3
        arm('w2-no-line-break-and-no-command-marker-in-any-printed-line',
            not any('\n' in ln or '\r' in ln or ln.lstrip().startswith('::warning')
                    or ln.lstrip().startswith('::notice') or '##[' in ln for ln in every)
            and any('forged from a file name' in ln for ln in lines)
            and any('forged notice' in ln for ln in lines2)
            and any('# #[warning]' in ln for ln in lines2))
        arm('w2-own-error-lines-keep-their-escaped-file-property', any(
            ln == '::error file=w2/q%0A%3A%3Awarning%3A%3Aforged%0A.md,line=1::home-mac: '
            + mask_shape(P_MAC + PLAIN + '/') for ln in lines3))
        # a masked location leaves the file property and rides the message, where
        # the line breaks in it are already gone and the marker cannot start a line
        arm('w2-a-masked-location-in-an-error-line-names-no-file-and-cannot-break-the-line',
            any(ln == '::error ::home-mac w2<home-mac:' + shape_of(P_MAC + PLAIN + '/')
                + '>p?::warning::forged?.txt:0: ' + mask_shape(P_MAC + PLAIN + '/') + ' [name]'
                for ln in lines3)
            and not any(ln.startswith('::error file=w2' + P_MAC) for ln in lines3),
            str([ln for ln in lines3 if 'w2' in ln][:4]))
        # the two sanitisers as pure oracles: every printed fragment goes through
        # clean() and every printed line through safe_line(), and each must hold on
        # its own (the surfaces above cannot tell which of the two did the work)
        hostile = 'a\n::warning::x\r##[error]y' + ESC + '[1m' + ZWSP + 'z'
        arm('w2-sanitiser-oracles', safe_line(hostile) == 'a?::warning::x?# #[error]y?[1mz'
            and clean(hostile) == 'a?::warning::x?# #[error]yz')
        # the private masker as a pure oracle: merged spans of every entry (the
        # shorter entry first cannot leave the tail of the longer one in clear) and
        # a match found only in a normalised view withholds the whole string
        ps = PrivateSet(parse_private('word:' + OV_SHORT + ' @host\nword:' + OV_LONG + ' @host\n'
                                      'word:' + PW + ' @host\n'))
        arm('r1-masker-oracles', ps.mask('ssh qzop@' + OV_LONG + ' up')
            == 'ssh qzop@<private#2@host> up'
            and ps.mask('[' + PW[0] + ']' + PW[1:]) == '<private#3@host>'
            and ps.mask('nothing here') == 'nothing here'
            and ps.mask_path('docs/' + OV_LONG + '/' + PW[:3] + ZWSP + PW[3:] + '.md')
            == 'docs/<private#2@host>/<private#3@host>')
        # the location masker as a pure oracle: a value in a name is masked per span
        # with no tier; a published example is not a value; a match found only in
        # a view of one component withholds the component, one found only in a
        # view of the whole path withholds the path (counted per class, so a
        # second encoded home path beside a plain one is not explained away by
        # the first); two home paths sharing a slash are both masked, as one
        # span whose shape covers both; with a tier in a CI log the
        # private and generic spans are merged on the RAW path, so a login before
        # a private host cannot ride out in clear; the same path on a terminal
        # with the tier carries the private label only
        cls_all = build_classes()
        tier1 = PrivateSet(parse_private('word:' + PW + ' @host\n'))
        sc_nt = Scanner('tree', cls_all, None, [], lambda s: None)
        sc_gh = Scanner('tree', cls_all, tier1, [], lambda s: None, fmt='github')
        sc_tx = Scanner('tree', cls_all, tier1, [], lambda s: None)
        lan_real = _addr(10, 77, 13, 9)
        bracket = '[1]' + lan_real[1:] + '.md'
        encoded = 'docs/Users%2F' + PLAIN + '%2Fx.md'
        twice = 'docs' + P_MAC + PLAIN + '/Users%2F' + PLAIN + '%2Fx.md'
        double = 'docs' + P_MAC + PLAIN + P_MAC + 'ab1/x.md'
        login_name = 'n/ssh ' + PLAIN + '@' + SH + '.md'
        arm('b1-location-masker-oracles',
            sc_nt.redact_path('docs/' + lan_real + '.md')
            == 'docs/<lan-addr:' + shape_of(lan_real) + '>.md'
            and sc_nt.redact_path('docs/' + _addr(10, 0, 0, 1) + '.md')
            == 'docs/' + _addr(10, 0, 0, 1) + '.md'
            and sc_nt.redact_path('docs/' + bracket) == 'docs/<lan-addr:' + shape_of(bracket) + '>'
            and sc_nt.redact_path(encoded) == '<home-mac:' + shape_of(encoded) + '>'
            and sc_nt.redact_path(twice) == '<home-mac:' + shape_of(twice) + '>'
            and sc_nt.redact_path(double)
            == 'docs<home-mac:' + shape_of(P_MAC + PLAIN + P_MAC + 'ab1/') + '>x.md'
            and sc_gh.redact_path(login_name)
            == 'n/<login-at-host:' + shape_of('ssh ' + PLAIN + '@' + SH + '.md') + '>'
            and sc_tx.redact_path(login_name) == 'n/ssh ' + PLAIN + '@' + RED1 + '.md'
            and sc_nt.redact_path('docs/plain.md') == 'docs/plain.md',
            str([sc_nt.redact_path('docs/' + bracket), sc_nt.redact_path(encoded),
                 sc_nt.redact_path(twice), sc_nt.redact_path(double),
                 sc_gh.redact_path(login_name),
                 sc_tx.redact_path(login_name)]))
        # --- contract 9: the guard's own files scan clean; the control does not ----------
        guard_root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(argv0)),
                                                   '..', '..'))
        own = [p for p in sorted(GUARD_FILES) if os.path.isfile(os.path.join(guard_root, p))]
        hooks_dir = os.path.join(guard_root, 'tools', 'hooks')
        if os.path.isdir(hooks_dir):
            own += ['tools/hooks/' + f for f in sorted(os.listdir(hooks_dir))
                    if os.path.isfile(os.path.join(hooks_dir, f))]
        if not own:
            own_root = os.path.dirname(os.path.abspath(argv0))
            own = [os.path.basename(argv0)]
        else:
            own_root = guard_root
        with open(os.path.join(tmp, 'own'), 'w') as fh:
            fh.write('\n'.join(own) + '\n')
        lines = []
        rc = main_inner(['tree', '--files-from', os.path.join(tmp, 'own'), '--no-allow',
                         '--hard', 'style-dash', '--hard', 'overlay-word'], own_root, penv,
                        lines.append, argv0, root_override=own_root)
        own_hits = [ln for ln in lines if ln.startswith('HIT')]
        arm('self-scan-clean', rc == EXIT_OK and not own_hits,
            'rc=%d files=%s hits=%s' % (rc, own, own_hits[:5]))
        ctrl_dir = os.path.join(tmp, 'ctrl')
        os.makedirs(ctrl_dir)
        with open(os.path.abspath(argv0), 'rb') as fh:
            src = fh.read()
        with open(os.path.join(ctrl_dir, 'joined.py'), 'wb') as fh:
            fh.write(src + b'\n# ' + '\n# '.join(c.sample for c in build_classes()).encode(
                'utf-8') + b'\n')
        with open(os.path.join(tmp, 'ctrl-list'), 'w') as fh:
            fh.write('joined.py\n')
        lines = []
        rc = main_inner(['tree', '--files-from', os.path.join(tmp, 'ctrl-list'), '--no-allow'],
                        ctrl_dir, penv, lines.append, argv0, root_override=ctrl_dir)
        arm('self-scan-control-hits', rc == EXIT_HIT and sum(
            1 for ln in lines if ln.startswith('HIT')) >= CLASS_FLOOR - 2)
        # --- contract 10 and 11 -------------------------------------------------------------
        classes = build_classes()
        arm('class-floor', len(classes) >= CLASS_FLOOR, str(len(classes)))
        arm('list-classes-runs', run(['--list-classes'])[0] == EXIT_OK)
        arm('list-lan-values-runs', run(['--list-lan-values'])[0] == EXIT_OK)
        standins = {PW, SU, SH, SH.replace('-', ''), PERSON.lower(), PLAIN_USER}
        arm('standins-disjoint-from-vocabularies',
            not (standins & (PH_USER | PH_HOST | ROLE_LOCAL))
            and not any(s.startswith(('example', 'your', 'my', 'robot-', 'box-', 'mac-'))
                        for s in standins))
        arm('word-expansion-shapes', all(
            re.compile(_expand_word(PW)[0], re.I).search(s) for s in (
                PW, PW[:6] + '-' + PW[6:], PW[:6] + '_' + PW[6:], PW.upper(), PW[:6] + ' '
                + PW[6:])) and not re.compile(_expand_word(PW)[0], re.I).search('x' + PW))
        arm('word-expansion-address', _expand_word(_addr(10, 1, 1, 1))[0].startswith(NB_L))
        arm('network-definition-rule', _network_definition('x/24 y', 1, _addr(10, 1, 2, 0))
            and not _network_definition('x/24 y', 1, _addr(10, 1, 2, 3))
            and not _network_definition('x/32', 1, _addr(10, 1, 2, 3)))
        arm('glob-rules', glob_to_rx('docs/**/*.md').match('docs/a/b/c.md') is not None
            and glob_to_rx('docs/*.md').match('docs/a/b.md') is None
            and glob_to_rx('**/x.txt').match('x.txt') is not None)
    arm('arm-count-pin', arms[0] == EXPECTED_ARMS, 'arms=%d expected=%d' % (arms[0],
                                                                           EXPECTED_ARMS))
    for f in failures:
        out('SELF-TEST FAILED: ' + f)
    if failures:
        return EXIT_NORUN
    out('leak_scan --self-test: OK (classes=%d arms=%d surfaces=%d)' % (
        len(build_classes()), arms[0], len(surfaces)))
    return EXIT_OK


# ---------------------------------------------------------------------------
# Entry.
# ---------------------------------------------------------------------------
def main_inner(argv, cwd, env, raw_out, argv0, neuter=None, root_override=None):
    def out(line):
        raw_out(safe_line(line))

    try:
        args = parse_args(argv)
        if args.self_test:
            return self_test(out, env, argv0)
        if args.list_classes:
            list_classes(out)
            return EXIT_OK
        root = root_override or find_root(cwd, env)
        if args.list_lan_values:
            list_lan_values(root, env, out)
            return EXIT_OK
        return run_mode(args, root, env, out, neuter=neuter, home=env.get('HOME'))
    except Usage as e:
        out('leak_scan: usage error: %s' % e)
        return EXIT_USAGE
    except NoRun as e:
        out('leak_scan: NO RUN: %s' % e)
        return EXIT_NORUN
    except Exception as e:  # never a traceback: it could carry a private match
        out('leak_scan: internal error (%s)' % type(e).__name__)
        if env.get('LEAK_SCAN_DEBUG'):
            import traceback
            traceback.print_exc()
        return EXIT_NORUN


def main():
    def out(line):
        sys.stdout.buffer.write(line.encode('utf-8', 'replace') + b'\n')
        sys.stdout.buffer.flush()
    return main_inner(sys.argv[1:], os.getcwd(), dict(os.environ), out, sys.argv[0])


if __name__ == '__main__':
    sys.exit(main())
