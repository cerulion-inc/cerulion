#!/usr/bin/env python3
"""ci_selected_packages.py: the workspace packages a change can reach.

Reads a `cargo metadata --format-version 1` document and prints, as a sorted
JSON array, the packages whose tests could observe a change to the packages
named on the command line: the named packages themselves plus every workspace
member that depends on one of them, transitively.

DIRECTION, and why it decides everything else. The walk runs along REVERSE
dependency edges, so the output is the set a workflow may narrow its test steps
to. A set that is too LARGE costs runner minutes; a set that is too SMALL skips
a test that could have caught the change, and skips it silently. Every rule
below therefore errs towards selecting more:

  * DEV-DEPENDENCY EDGES ARE FOLLOWED. A crate that only dev-depends on another
    compiles its integration tests against it, and those tests are exactly the
    ones the change can break. Dropping these edges is what makes selection
    look cheap while it stops running the tests that matter, so `dependents`
    takes `include_dev` and the self-test pins both answers on the real
    workspace: the two numbers differ, which is the proof the edges are live.
  * BUILD-DEPENDENCY EDGES ARE FOLLOWED, for the same reason: a build script
    that links a workspace crate re-runs when that crate changes.
  * A NAME THAT IS NOT A WORKSPACE MEMBER IS AN ERROR, never an empty
    selection. Names reach here from changed paths, so a name the workspace
    does not carry means the caller's path rules and the manifest disagree, and
    a refusal is the answer rather than "nothing to run".
  * ANY PATH A CALLER CANNOT CLASSIFY selects everything, which is what `all`
    and `--all` are for. Either spelling is the WHOLE selection, so combining
    it with a package name is a refusal too: the caller that wrote both meant
    one of them, and picking for it would run the wrong set.

A dependency is resolved to a workspace member BY NAME, and `dependencies[].name`
is the real package name rather than the local alias, so a renamed dependency
still resolves; the alias is not a member and naming it is a refusal. A
dependency whose name no member carries is a registry crate and is ignored, and
so is a self-edge (a crate may dev-depend on itself, and it adds nothing to its
own closure). A package that appears in `packages` without being in
`workspace_members` is never selected, whatever it depends on, so a document
produced with or without `--no-deps` gives the same answer.

WHAT THIS CLOSURE PROVES, AND WHAT IT DOES NOT. It is the reverse CARGO
DEPENDENCY closure over normal, build and dev edges: a package whose tests can
observe a change through a dependency edge is in the output. A test can also
observe another package WITHOUT an edge to it — by opening a path literal into
that package's tree, by walking the whole repository, or by loading an artifact
built from it at run time (`dlopen`) — and NONE of those classes is covered
here. One of them is covered elsewhere: the doc-pin walk in
`crates/cerulion_cli_engine/tests/ci_doc_pin_walk_test.rs` pins every test
binary that opens the shared documentation and tool trees. Cross-crate source
literals and dlopen fixtures are an OPEN class, and they have to be pinned
before any CI step is gated on this selection.

Usage:
  ci_selected_packages.py [--metadata FILE] PACKAGE...
  ci_selected_packages.py [--metadata FILE] all
  ci_selected_packages.py [--metadata FILE] --all
  ci_selected_packages.py --self-test

`--metadata` defaults to `-`, standard input.

Exit codes:
  0  the selection is on stdout, or the self-test passed
  1  the self-test found a miss
  2  malformed invocation, unreadable metadata, or a name that is not a
     workspace member

Stdlib only. Python 3.8 or newer.
"""
import argparse
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

METADATA_FORMAT_VERSION = 1
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class SelectionError(Exception):
    """A refusal this script reports as exit 2.

    Every subclass carries the offending values as fields and compares by those
    fields, so a caller and a test read the refusal rather than parse its text.
    """


class UnknownPackages(SelectionError):
    """Names that no workspace member carries."""

    def __init__(self, names, members):
        self.names = tuple(sorted(names))
        self.members = tuple(sorted(members))
        super().__init__(str(self))

    def __str__(self):
        return ('ci_selected_packages: not a workspace member: %s (the document lists %d members); '
                'a name reaches here from a changed path, so either the path rule that produced it '
                'or the workspace manifest is wrong'
                % (', '.join(self.names), len(self.members)))

    def __eq__(self, other):
        return type(other) is type(self) and (other.names, other.members) == (self.names, self.members)

    def __hash__(self):
        return hash((type(self).__name__, self.names, self.members))


class UnsupportedMetadata(SelectionError):
    """A metadata document in a format version this script does not read."""

    def __init__(self, version):
        self.version = version
        super().__init__(str(self))

    def __str__(self):
        return ('ci_selected_packages: metadata format version %r, expected %d; '
                'produce the document with `cargo metadata --format-version %d`'
                % (self.version, METADATA_FORMAT_VERSION, METADATA_FORMAT_VERSION))

    def __eq__(self, other):
        return type(other) is type(self) and other.version == self.version

    def __hash__(self):
        return hash((type(self).__name__, self.version))


def workspace_members(document):
    """The names of the workspace members in a format-version 1 document."""
    version = document.get('version')
    if version != METADATA_FORMAT_VERSION:
        raise UnsupportedMetadata(version)
    member_ids = set(document.get('workspace_members') or ())
    return {package['name'] for package in document.get('packages') or () if package['id'] in member_ids}


def dependents(document, include_dev=True):
    """Member name -> the member names that depend on it, one hop.

    `include_dev` exists for the self-test's positive control: with it False the
    walk drops dev-dependency edges, which is the mutation that shrinks a
    closure without failing anything else. Callers leave it True.
    """
    members = workspace_members(document)
    member_ids = set(document.get('workspace_members') or ())
    table = {name: set() for name in members}
    for package in document.get('packages') or ():
        if package['id'] not in member_ids:
            continue
        for dependency in package.get('dependencies') or ():
            if not include_dev and dependency.get('kind') == 'dev':
                continue
            name = dependency['name']
            if name in table and name != package['name']:
                table[name].add(package['name'])
    return table


def selected_packages(document, touched, include_dev=True):
    """The touched packages plus their reverse-dependency closure, sorted."""
    table = dependents(document, include_dev)
    unknown = set(touched) - set(table)
    if unknown:
        raise UnknownPackages(unknown, table)
    selected = set()
    pending = list(touched)
    while pending:
        name = pending.pop()
        if name in selected:
            continue
        selected.add(name)
        pending.extend(table[name])
    return sorted(selected)


def read_metadata(source):
    """The document at `source`, or standard input when it is `-`."""
    if source == '-':
        return json.load(sys.stdin)
    with open(source, encoding='utf-8') as handle:
        return json.load(handle)


# ---------------------------------------------------------------------------
# Self-test.
#
# Every document below is built by hand and every expected closure is written
# out by hand beside it, so no arm compares the walk against another run of the
# walk. The live arm is the other half: it runs the walk over the real
# workspace and checks it against counts measured off the manifests by other
# means, which is the only arm that can catch a walk that is self-consistently
# wrong about this workspace.
# ---------------------------------------------------------------------------

# LIVE WORKSPACE ORACLE. Each row is `package: (closure size, closure size with
# dev-dependency edges dropped)`, and LIVE_MEMBER_COUNT is the workspace member
# count, all measured off this workspace independently of this script. The two
# sizes in a row differ wherever a dev-dependency edge carries the closure, and
# `cerulion_bag` is the extreme case: 52 packages can observe a change to it,
# six of them without dev-dependency edges. That gap is what proves the edges
# are followed on the real graph rather than only on a fixture.
#
# The member count is NOT the length of the root manifest's `members` list: a
# path dependency inside the workspace directory is a member whether or not the
# list names it, which is why the closure is read from `cargo metadata` rather
# than from the manifests.
#
# LIVE_ORACLE_MEASURED is False while a measurement is outstanding, and the live
# arm then reports a miss rather than passing on numbers nobody measured.
LIVE_ORACLE_MEASURED = True
LIVE_MEMBER_COUNT = 66
LIVE_ORACLE = {
    'cerulion_bag': (52, 6),
    'cerulion_cli': (1, 1),
    'cerulion_cli_engine': (52, 4),
    'cerulion_core': (52, 52),
    'cerulion_viz': (2, 2),
    'cerulion_vizd': (1, 1),
    'go2_tf': (4, 4),
    'native_ros2_messages': (52, 40),
}


def _dependency(entry):
    """One `dependencies` entry.

    `name`, `name:dev` / `name:build` for a kind, and `name=alias` for a
    dependency RENAMED in the manifest: `cargo metadata` reports the real
    package name in `name` and the local alias in `rename`.
    """
    entry, _, rename = entry.partition('=')
    name, _, kind = entry.partition(':')
    out = {'name': name, 'req': '*', 'kind': kind or None}
    if rename:
        out['rename'] = rename
    return out


def _document(members, outsiders=None):
    """A format-version 1 document over hand-written packages.

    `members` maps each workspace member name to its dependency entries.
    `outsiders` maps packages that appear in the document WITHOUT being
    workspace members, the shape `cargo metadata` produces for a registry crate.
    """
    packages = []
    member_ids = []
    for name, deps in members.items():
        package_id = 'path+file:///w/%s#%s@0.1.0' % (name, name)
        member_ids.append(package_id)
        packages.append({'id': package_id, 'name': name,
                         'dependencies': [_dependency(entry) for entry in deps]})
    for name, deps in (outsiders or {}).items():
        packages.append({'id': 'registry+https://example.invalid#%s@1.0.0' % name, 'name': name,
                         'dependencies': [_dependency(entry) for entry in deps]})
    return {'version': METADATA_FORMAT_VERSION, 'packages': packages,
            'workspace_members': member_ids}


# A crate whose only route to `leaf` is a dev-dependency.
DEV_ONLY = _document({'leaf': [], 'user': ['leaf:dev']})
# A build script that links a workspace crate.
BUILD_ONLY = _document({'codegen': [], 'user': ['codegen:build']})
# A registry crate as a dependency, and a registry crate that depends on a
# member: neither is a member, so neither is ever selected.
OUTSIDE = _document({'core': [], 'app': ['serde', 'core']},
                    {'serde': [], 'downstream': ['core']})
# base <- left (normal) and base <- right (dev), both <- top.
DIAMOND = _document({'base': [], 'left': ['base'], 'right': ['base:dev'], 'top': ['left', 'right']})
# A crate that dev-depends on itself, and a pair that depend on each other.
LOOPS = _document({'solo': ['solo:dev'], 'ping': ['pong:dev'], 'pong': ['ping']})
# `user` depends on `base` under a local alias. The edge is keyed by the real
# package name, which is what the docstring above claims and nothing exercised.
RENAMED = _document({'base': [], 'user': ['base=alias']})

# `name | document | touched | dev edges followed | expected closure`.
CLOSURE_CASES = [
    ('dev-edge-followed', DEV_ONLY, ['leaf'], True, ['leaf', 'user']),
    ('dev-edge-dropped-shrinks-the-same-closure', DEV_ONLY, ['leaf'], False, ['leaf']),
    ('build-edge-followed', BUILD_ONLY, ['codegen'], True, ['codegen', 'user']),
    ('build-edge-survives-dropping-dev-edges', BUILD_ONLY, ['codegen'], False, ['codegen', 'user']),
    ('registry-dependency-ignored', OUTSIDE, ['app'], True, ['app']),
    ('registry-dependent-never-selected', OUTSIDE, ['core'], True, ['app', 'core']),
    ('diamond-from-the-base', DIAMOND, ['base'], True, ['base', 'left', 'right', 'top']),
    ('diamond-base-without-dev-edges', DIAMOND, ['base'], False, ['base', 'left', 'top']),
    ('diamond-from-one-side', DIAMOND, ['left'], True, ['left', 'top']),
    ('diamond-from-the-dev-side', DIAMOND, ['right'], True, ['right', 'top']),
    ('diamond-two-touched-packages', DIAMOND, ['left', 'right'], True, ['left', 'right', 'top']),
    ('leaf-closure-is-itself', DIAMOND, ['top'], True, ['top']),
    ('touched-package-is-always-selected', LOOPS, ['solo'], True, ['solo']),
    ('cycle-terminates', LOOPS, ['ping'], True, ['ping', 'pong']),
    ('cycle-terminates-from-the-other-end', LOOPS, ['pong'], True, ['ping', 'pong']),
    ('repeated-name-selects-once', DIAMOND, ['top', 'top'], True, ['top']),
    ('renamed-dependency-edge-is-keyed-by-the-package-name', RENAMED, ['base'], True,
     ['base', 'user']),
]

# `name | document | touched | expected error`.
ERROR_CASES = [
    ('registry-crate-is-not-a-member', OUTSIDE, ['serde'],
     UnknownPackages(['serde'], ['app', 'core'])),
    ('package-outside-the-workspace-is-not-a-member', OUTSIDE, ['downstream'],
     UnknownPackages(['downstream'], ['app', 'core'])),
    ('several-unknown-names-are-all-named', DIAMOND, ['nope', 'base', 'gone'],
     UnknownPackages(['gone', 'nope'], ['base', 'left', 'right', 'top'])),
    # The other side of the renamed edge: the ALIAS is nobody's package name,
    # so a caller that reaches it from a changed path is refused rather than
    # handed an empty selection.
    ('renamed-dependency-alias-is-not-a-member', RENAMED, ['alias'],
     UnknownPackages(['alias'], ['base', 'user'])),
]

def _run(argv, stdin_text=''):
    """Run the command line in-process: returns (exit code, stdout, stderr)."""
    out, err = io.StringIO(), io.StringIO()
    saved_stdin = sys.stdin
    sys.stdin = io.StringIO(stdin_text)
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = run(argv)
            except SystemExit as exit_request:
                code = exit_request.code
    finally:
        sys.stdin = saved_stdin
    return code, out.getvalue(), err.getvalue()


def live_metadata():
    """The workspace's own metadata document."""
    output = subprocess.check_output(
        ['cargo', 'metadata', '--format-version', str(METADATA_FORMAT_VERSION), '--no-deps'],
        cwd=REPO_ROOT, text=True)
    return json.loads(output)


def self_test():
    """The table above, then the live workspace. Returns an exit code."""
    failures = []
    arms = [0]

    def arm(name, ok, detail=''):
        arms[0] += 1
        if not ok:
            failures.append('%s %s' % (name, detail))

    for name, document, touched, include_dev, expected in CLOSURE_CASES:
        got = selected_packages(document, touched, include_dev)
        arm(name, got == expected, '-> %s, wanted %s' % (got, expected))

    for name, document, touched, expected in ERROR_CASES:
        try:
            got = selected_packages(document, touched)
        except SelectionError as error:
            arm(name, error == expected, '-> %s, wanted %s' % (error, expected))
        else:
            arm(name, False, '-> %s, wanted a refusal' % (got,))

    # Both sides of the supported format version, each document built from the
    # constant rather than from a typed-in number.
    supported = dict(DIAMOND, version=METADATA_FORMAT_VERSION)
    arm('format-version-at-the-supported-version',
        selected_packages(supported, ['top']) == ['top'])
    above = METADATA_FORMAT_VERSION + 1
    for name, document, expected in (
            ('format-version-above-the-supported-version-refused',
             dict(DIAMOND, version=above), UnsupportedMetadata(above)),
            ('format-version-absent-refused', {'packages': [], 'workspace_members': []},
             UnsupportedMetadata(None))):
        try:
            selected_packages(document, [])
        except SelectionError as error:
            arm(name, error == expected, '-> %s, wanted %s' % (error, expected))
        else:
            arm(name, False, 'the document was read as version %d' % METADATA_FORMAT_VERSION)

    # `name | argv | stdin | the whole (exit code, stdout, stderr)`.
    document_text = json.dumps(DIAMOND)
    every_member = '["base", "left", "right", "top"]\n'
    for name, argv, stdin_text, expected in (
            ('cli-reads-stdin', ['base'], document_text, (0, every_member, '')),
            ('cli-sorts-and-deduplicates', ['top', 'left', 'top'], document_text,
             (0, '["left", "top"]\n', '')),
            ('cli-all-sentinel', ['all'], document_text, (0, every_member, '')),
            ('cli-all-flag', ['--all'], document_text, (0, every_member, '')),
            # `all` is read as the sentinel even where a member carries that
            # name, which selects that member's dependents along with the rest.
            ('cli-all-sentinel-wins-over-a-member-of-that-name', ['all'],
             json.dumps(_document({'all': [], 'spare': [], 'user': ['all']})),
             (0, '["all", "spare", "user"]\n', '')),
            ('cli-unknown-name-refused-in-full', ['nope'], document_text,
             (2, '', str(UnknownPackages(['nope'], ['base', 'left', 'right', 'top'])) + '\n'))):
        got = _run(argv, stdin_text)
        arm(name, got == expected, '-> %r, wanted %r' % (got, expected))

    # argparse writes its own usage, so these pin the exit code and the silence
    # on stdout: a refused invocation never prints something a caller can read
    # as a selection.
    for name, argv in (('cli-no-package-exits-2', []),
                       ('cli-self-test-takes-no-package', ['--self-test', 'base']),
                       # `all` and `--all` are the WHOLE selection, so a caller
                       # that also names a package contradicted itself and gets
                       # a refusal rather than one of the two readings.
                       ('cli-all-flag-with-a-package-exits-2', ['--all', 'base']),
                       ('cli-all-sentinel-with-a-package-exits-2', ['all', 'base']),
                       ('cli-all-sentinel-before-a-package-exits-2', ['base', 'all'])):
        code, out, _ = _run(argv, document_text)
        arm(name, (code, out) == (2, ''), '-> %s %r' % (code, out))

    with tempfile.TemporaryDirectory() as scratch:
        path = os.path.join(scratch, 'metadata.json')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(document_text)
        got = _run(['--metadata', path, 'left'])
        arm('cli-reads-a-path', got == (0, '["left", "top"]\n', ''), '-> %r' % (got,))
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write('{"version": 1, "packages": [{"name": "base"}]}')
        # Pinned WHOLE, like its sibling below: the line a caller reads has to
        # name the document it could not read AND the field that was missing.
        expected_field = (2, '', 'ci_selected_packages: cannot read %s: %r\n'
                          % (path, KeyError('id')))
        got = _run(['--metadata', path, 'base'])
        arm('cli-document-missing-a-field-refused-in-full', got == expected_field,
            '-> %r, wanted %r' % (got, expected_field))
        # The absent-file refusal is pinned WHOLE, not by a prefix: the line a
        # caller reads has to name the path it could not open and why.
        absent = os.path.join(scratch, 'absent.json')
        expected_absent = (2, '', 'ci_selected_packages: cannot read %s: %r\n'
                           % (absent, FileNotFoundError(2, 'No such file or directory')))
        got = _run(['--metadata', absent, 'base'])
        arm('cli-absent-document-refused-in-full', got == expected_absent,
            '-> %r, wanted %r' % (got, expected_absent))

    arm('live-oracle-measured', LIVE_ORACLE_MEASURED,
        'LIVE_ORACLE_MEASURED is False: measure the workspace, fill in '
        'LIVE_ORACLE and LIVE_MEMBER_COUNT, and set it True')
    if LIVE_ORACLE_MEASURED:
        document = live_metadata()
        members = workspace_members(document)
        arm('live-member-count', len(members) == LIVE_MEMBER_COUNT,
            '-> %d, wanted %d' % (len(members), LIVE_MEMBER_COUNT))
        for name in sorted(LIVE_ORACLE):
            with_dev, without_dev = LIVE_ORACLE[name]
            got = len(selected_packages(document, [name]))
            arm('live-%s' % name, got == with_dev, '-> %d, wanted %d' % (got, with_dev))
            got = len(selected_packages(document, [name], include_dev=False))
            arm('live-%s-without-dev-edges' % name, got == without_dev,
                '-> %d, wanted %d' % (got, without_dev))
        arm('live-dev-edges-change-the-answer',
            any(with_dev != without_dev for with_dev, without_dev in LIVE_ORACLE.values()),
            'every row has the same closure with and without dev edges')
        # The mutant this kills on the real graph: a walk that reports a
        # package's dependents without the package itself.
        arm('live-every-member-is-in-its-own-closure',
            all(name in selected_packages(document, [name]) for name in sorted(members)))

    for failure in failures:
        print('SELF-TEST FAILED: ' + failure, file=sys.stderr)
    if failures:
        print('ci_selected_packages: %d of %d self-test arm(s) failed' % (len(failures), arms[0]),
              file=sys.stderr)
        return 1
    print('ci_selected_packages: self-test OK (%d arms, %d live rows)' % (arms[0], len(LIVE_ORACLE)))
    return 0


def run(argv):
    """The command line. Returns an exit code; never raises SelectionError."""
    parser = argparse.ArgumentParser(
        prog='ci_selected_packages.py',
        description='Print the touched workspace packages plus their reverse-dependency closure.')
    parser.add_argument('--metadata', default='-', metavar='FILE',
                        help='a `cargo metadata --format-version 1` document (default: stdin)')
    parser.add_argument('--all', action='store_true', help='print every workspace member')
    parser.add_argument('--self-test', action='store_true', help='run the self-test and exit')
    parser.add_argument('packages', nargs='*', metavar='PACKAGE',
                        help='touched workspace package names, or the single word `all`')
    args = parser.parse_args(argv)

    if args.self_test:
        if args.packages or args.all or args.metadata != '-':
            parser.error('--self-test takes no other argument')
        return self_test()

    if not args.packages and not args.all:
        parser.error('name at least one touched package, or `all`, or pass --all')
    if args.all and args.packages:
        parser.error('--all is the whole selection and takes no package name')
    if 'all' in args.packages and len(args.packages) > 1:
        parser.error('`all` is the whole selection and takes no other package name')

    try:
        document = read_metadata(args.metadata)
        if args.all or args.packages == ['all']:
            selected = sorted(workspace_members(document))
        else:
            selected = selected_packages(document, args.packages)
    except SelectionError as error:
        print(error, file=sys.stderr)
        return 2
    except (OSError, ValueError, KeyError, TypeError) as error:
        # A document missing a field this walk needs is unreadable, not empty:
        # exit 2 so a caller never reads a truncated selection as the answer.
        print('ci_selected_packages: cannot read %s: %r' % (args.metadata, error), file=sys.stderr)
        return 2

    print(json.dumps(selected))
    return 0


if __name__ == '__main__':
    sys.exit(run(sys.argv[1:]))
