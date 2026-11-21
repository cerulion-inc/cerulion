#!/usr/bin/env python3
"""ci_built_tests.py: build a CI leg's package tests once, then run each binary.

A `cargo test -p X` step per package resolves features for X alone, so each step
rebuilds the crates X shares with the leg's other packages. `build` compiles a
group of packages in ONE `cargo test --no-run`, then runs `cargo test` over the
same packages (it compiles nothing) with this script as cargo's target runner,
recording the environment and working directory cargo gives each test binary;
cargo then runs the packages' doctests against the libraries the build compiled,
and rustdoc hands each doctest it runs to the same runner, which runs it. `run -- cargo test -p X ...` then runs the prebuilt binaries
that command selects: one process per binary, cargo's order, cargo's
environment, the same arguments, cargo's fail-fast rule. `audit` fails when a
built binary had no unfiltered run, when a full leg built fewer binaries than
its floor, when `--not-run` names a binary the leg does not build, or when cargo
compiled a unit or ran a build script outside a declared build step (`phase`).
`check-workflow FILE` holds a workflow's wiring to these rules. State lives in
`$RUNNER_TEMP/ci-built-tests`. The rules: `docs/internals/ci-and-gates.md`,
"Build once per shard".
"""

import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SELF = os.path.abspath(__file__)
SKIP_ENV = {'PWD', 'OLDPWD', 'SHLVL', '_'}
SLACK_NS = 200_000_000
# what `reclaim` never deletes: libraries (shared ones by name, whatever their
# mode, `.so.N` included) and cargo's metadata
KEPT_FILE = re.compile(r'\.(so|dylib|dll)(\.\d+)*$|\.(rlib|rmeta|d|a|lib)$')
# libtest arguments that still run every test of a binary
WHOLE_RUN_ARGS = {'--nocapture', '--show-output', '--include-ignored', '-q', '--quiet'}
# The one condition an audit step may carry: the classifier's `code` output, in
# a job whose runner follows that output and whose every step is skipped when
# no package is reached, so a skipped audit audits a leg that built and ran
# nothing.
CODE_GATE = "needs.changes.outputs.code == 'true'"


def step_condition(step):
    """The `if:` of one step block, or None: the step's own key, as its first
    key (`- if:`) or at the step's key indent, never a line inside a `run:`
    script. A folded (`>-`, `>`) or literal (`|`) value is its lines joined
    onto one, read up to the first line not indented under the key."""
    m = re.search(r'^(?:      - |        )if:[ ]*(.*?)\r?$', step, re.M)
    if not m:
        return None
    value = m.group(1).strip()
    if value in ('>-', '>', '|', '>+', '|-', '|+'):
        deeper = []
        for line in step[m.end():].split('\n')[1:]:
            if not line.startswith('          '):
                break
            deeper.append(line.strip())
        value = ' '.join(deeper)
    return value


def top_level_terms(cond):
    """`cond` split at each `&&` outside parentheses and quotes."""
    terms, depth, quote, start, i = [], 0, None, 0, 0
    while i < len(cond):
        c = cond[i]
        if quote:
            quote = None if c == quote else quote
        elif c in "'\"":
            quote = c
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
        elif depth == 0 and cond.startswith('&&', i):
            terms.append(cond[start:i].strip())
            start = i + 2
            i += 1
        i += 1
    terms.append(cond[start:].strip())
    return terms


def holds_code_gate(cond):
    """Whether `cond` is an `&&` chain with CODE_GATE as one of its terms outside
    every parenthesis, so it is false whenever CODE_GATE is. Any `||` is
    refused: `a && b || c` holds with `a` false. A gate inside a negation or
    parentheses is no term of the chain."""
    if cond is None:
        return False
    cond = cond.strip()
    if cond.startswith('${{') and cond.endswith('}}'):
        cond = cond[3:-2].strip()
    if '||' in cond:
        return False
    return CODE_GATE in top_level_terms(cond)


def die(msg):
    print(f'ci_built_tests: {msg}', file=sys.stderr)
    sys.exit(2)


def target_dir():
    return os.path.abspath(os.environ.get('CARGO_TARGET_DIR') or os.path.join(ROOT, 'target'))


def state_path(name):
    base = os.environ.get('RUNNER_TEMP') or target_dir()
    os.makedirs(os.path.join(base, 'ci-built-tests'), exist_ok=True)
    return os.path.join(base, 'ci-built-tests', name)


def load_groups():
    path = state_path('groups.json')
    return json.load(open(path)) if os.path.exists(path) else {}


def append(name, record):
    with open(state_path(name), 'a') as f:
        f.write(json.dumps(record, sort_keys=True) + '\n')


def read_lines(name):
    path = state_path(name)
    return [json.loads(l) for l in open(path) if l.strip()] if os.path.exists(path) else []


def rustc(*args):
    return subprocess.run(['rustc', *args], capture_output=True, text=True, check=True).stdout


def host_triple():
    return next(l[6:].strip() for l in rustc('-vV').splitlines() if l.startswith('host: '))


# ------------------------------------------------------------------ pure rules

def selected_packages(specs, selection):
    """(built, deselected). A gated package builds exactly when the classifier's
    list names it, as its step's `contains(fromJSON(pkgs), 'X')` reads; unset
    or empty is a local run and selects everything."""
    names = None
    if selection and selection.strip():
        try:
            names = json.loads(selection)
        except ValueError:
            die(f'CI_SELECTED_PACKAGES is not JSON: {selection!r}')
        if not isinstance(names, list) or not all(isinstance(n, str) for n in names):
            die(f'CI_SELECTED_PACKAGES is not a JSON list of names: {selection!r}')
    built, deselected = [], []
    for spec in specs:
        name = spec.lstrip('+')
        if not name or name in built + deselected:
            die(f'bad or repeated package spec {spec!r}')
        (built if spec.startswith('+') or names is None or name in names else deselected).append(name)
    return built, deselected


def parse_cargo_test(argv):
    """The selection a `cargo test` command line makes. Anything this runner
    cannot honour exactly is refused rather than ignored."""
    if argv[:2] != ['cargo', 'test']:
        die(f'run expects `cargo test ...`, got {" ".join(argv[:2])!r}')
    sel = {'package': None, 'lib': False, 'bins': False, 'tests': [], 'features': [],
           'no_fail_fast': False, 'filter': None, 'args': []}
    rest, i = argv[2:], 0
    while i < len(rest):
        a = rest[i]
        if a == '--':
            sel['args'] = rest[i + 1:]
            break
        if a in ('-p', '--package', '--test', '--features'):
            if i + 1 >= len(rest):
                die(f'{a} takes a value')
            v = rest[i + 1]
            if a == '--test':
                sel['tests'].append(v)
            elif a == '--features':
                sel['features'] += v.replace(',', ' ').split()
            elif sel['package'] is not None:
                die('run takes exactly one -p package')
            else:
                sel['package'] = v
            i += 2
            continue
        if a in ('--lib', '--bins', '--no-fail-fast'):
            sel[a[2:].replace('-', '_')] = True
        elif a == '--locked':
            pass
        elif a.startswith('-'):
            die(f'unsupported cargo test argument {a!r}: the prebuilt runner cannot honour it')
        elif sel['filter'] is None:
            sel['filter'] = a
        else:
            die(f'a second test name filter {a!r}; cargo takes one')
        i += 1
    if sel['package'] is None:
        die('run needs -p <package>')
    sel['features'] = [f if '/' in f else f'{sel["package"]}/{f}' for f in sel['features']]
    return sel


def select_binaries(manifest, sel):
    """The manifest entries `cargo test` with `sel` runs, in manifest order."""
    pkg = sel['package']
    mine = [b for b in manifest if b['package'] == pkg]
    if not mine:
        die(f'{pkg} was not built by this leg\'s build step: add it there')
    if not (sel['lib'] or sel['bins'] or sel['tests']):
        return mine
    keep = set()
    if sel['lib']:
        libs = [b['executable'] for b in mine if b['kind'] == 'lib']
        if not libs:
            die(f'no library targets found in package `{pkg}`')
        keep.update(libs)
    if sel['bins']:
        keep.update(b['executable'] for b in mine if b['kind'] == 'bin')
    for name in sel['tests']:
        hits = [b['executable'] for b in mine if b['kind'] == 'test' and b['target'] == name]
        if not hits:
            die(f'no test target named `{name}` in package `{pkg}`')
        keep.update(hits)
    return [b for b in mine if b['executable'] in keep]


def env_delta(base, recorded, ignore):
    """What cargo set or removed for one binary, against the build step's env."""
    skip = SKIP_ENV | set(ignore)
    changed = {k: v for k, v in recorded.items()
               if k not in skip and not k.startswith('CI_BUILT_TESTS_') and base.get(k) != v}
    return {'changed': changed, 'removed': sorted(k for k in base if k not in recorded and k not in skip)}


def replay_env(current, delta, base_keys, soft_keys):
    """The env cargo would give the binary in this step: a cargo `[env]` key
    without `force` yields to a value the step itself sets, as in cargo."""
    env = {k: v for k, v in current.items() if k not in delta['removed']}
    for k, v in delta['changed'].items():
        if not (k in soft_keys and k in current and k not in base_keys):
            env[k] = v
    return env


def soft_env_keys(text):
    """The `[env]` keys of a `.cargo/config.toml` that carry no `force = true`."""
    soft, section = set(), None
    for raw in text.splitlines():
        line = raw.split('#', 1)[0].strip()
        if line.startswith('['):
            section = line.strip('[] ')
        elif section == 'env' and '=' in line:
            key, value = (part.strip() for part in line.split('=', 1))
            if not re.search(r'\bforce\s*=\s*true\b', value):
                soft.add(key)
    return soft


def cargo_dylib_path(native_dirs, tdir, sysroot, host, existing, home):
    """DYLD_FALLBACK_LIBRARY_PATH as cargo under the rustup proxy sets it
    (measured, cargo 1.98), for a recording that could not see it: macOS strips
    the variable from a system interpreter."""
    tail = existing.split(':') if existing else [os.path.join(home, 'lib'), '/usr/local/lib', '/usr/lib']
    inside = sorted(d for d in set(native_dirs) if d.startswith(tdir + os.sep))
    return ':'.join(inside + [os.path.join(tdir, 'debug'), os.path.join(tdir, 'debug', 'deps'),
                              os.path.join(sysroot, 'lib', 'rustlib', host, 'lib'),
                              os.path.join(sysroot, 'lib')] + tail)


def artifact_kind(kinds):
    if 'bin' in kinds or 'test' in kinds:
        return 'bin' if 'bin' in kinds else 'test'
    return None if ('example' in kinds or 'bench' in kinds) else 'lib'


def unit_kind(files):
    for prefix, kind in (('run-build-script-', 'build script run'), ('build-script-', 'build script'),
                         ('test-', 'test')):
        if any(f.startswith(prefix) for f in files):
            return kind
    return 'build'


def compiled_units(profile_dir, since):
    """[('crate (kind)', mtime_ns)] for each unit cargo compiled or build script
    cargo ran since `since`: `.fingerprint/*` and `build/*` each date one in an
    `invoked.timestamp`."""
    units = []
    for sub in ('.fingerprint', 'build'):
        top = os.path.join(profile_dir, sub)
        for d in sorted(os.listdir(top)) if os.path.isdir(top) else []:
            stamp = os.path.join(top, d, 'invoked.timestamp')
            if os.path.exists(stamp) and os.stat(stamp).st_mtime_ns >= since:
                kind = 'build script run' if sub == 'build' else unit_kind(os.listdir(os.path.join(top, d)))
                units.append((f'{d.rsplit("-", 1)[0]} ({kind})', os.stat(stamp).st_mtime_ns))
    return units


def attribute(units, phases, slack_ns):
    """(units per declared step, library builds per crate, units outside every step).
    A file's mtime can trail the clock, so a unit just before a step begins counts
    as the step's; nothing after a step ends does."""
    counts, libs, stray = {p['name']: 0 for p in phases}, {}, []
    for name, t in units:
        home = ([p for p in phases if p['begin'] <= t <= p['end']]
                or [p for p in phases if p['begin'] - slack_ns <= t <= p['end']])
        if not home:
            stray.append(name)
            continue
        counts[home[0]['name']] += 1
        if name.endswith(' (build)'):
            libs[name[:-8]] = libs.get(name[:-8], 0) + 1
    return counts, libs, sorted(stray)


def job_blocks(text):
    """{job id: [step text]} for every job holding a `build` step of this script."""
    jobs, name, cur = {}, None, None
    for line in text.splitlines():
        m = re.match(r'^  ([A-Za-z0-9_-]+):$', line)
        if m:
            name, cur = m.group(1), None
            continue
        if name is None or line.lstrip().startswith('#'):
            continue
        if re.match(r'^      - ', line):
            cur = []
            jobs.setdefault(name, []).append(cur)
        if cur is not None:
            cur.append(line.split(' #', 1)[0])
    steps = {j: ['\n'.join(st) for st in v] for j, v in jobs.items()}
    return {j: v for j, v in steps.items() if any('ci_built_tests.py build' in st for st in v)}


def workflow_complaints(text):
    """Each leg builds exactly the packages its steps run, `+` exactly where the
    step has no selection condition, the build comes first, every cargo command
    that can compile is a `run` or a declared `phase`, and the audit carries no
    condition but, at most, CODE_GATE, and that only in a job whose runner
    follows `code` and whose every build step carries the gate too."""
    out = []
    for job, steps in sorted(job_blocks(text).items()):
        legs = re.search(r'shard: \[([\d, ]+)\]', '\n'.join(steps)) or re.search(
            r'shard: \[([\d, ]+)\]', text.split(f'  {job}:', 1)[1])
        legs = [int(x) for x in legs.group(1).split(',')] if legs else [0]
        built, ran, seen_build = {l: {} for l in legs}, {l: {} for l in legs}, False
        audits = [i for i, st in enumerate(steps) if 'ci_built_tests.py audit' in st]
        runs = [i for i, st in enumerate(steps) if 'ci_built_tests.py run --' in st]
        audit_if = step_condition(steps[audits[-1]]) if audits else None
        code_gated = audit_if == CODE_GATE
        if not audits or (audit_if is not None and not code_gated) or (runs and audits[-1] < runs[-1]):
            out.append(f'{job}: needs an audit step after its last run step with no condition, or with '
                       f'exactly `{CODE_GATE}`')
        if code_gated:
            head = re.search(rf'^  {re.escape(job)}:[ \t]*\r?$(.*?)^    steps:', text, re.M | re.S)
            runner = head and re.search(r'^    runs-on:[ ]*(.*?)\r?$', head.group(1), re.M)
            if not (runner and 'needs.changes.outputs.code' in runner.group(1)):
                out.append(f'{job}: the audit is gated on `code`, but the runner does not follow `code`')
            for st in steps:
                if 'ci_built_tests.py build' in st and not holds_code_gate(step_condition(st)):
                    out.append(f'{job}: the audit is gated on `code`, so the build step must carry that gate '
                               f'as a whole `&&` term, with no `||`')
        for st in steps:
            if 'ci_built_tests.py build' in st:
                seen_build = True
                for leg, arm in re.findall(r'^\s+(\d+)\) (.*?);;', st, re.M | re.S):
                    for cmd in re.findall(r'ci_built_tests\.py build (.*)', arm):
                        toks = cmd.split()
                        feats = [f for i, t in enumerate(toks) if t == '--features'
                                 for f in toks[i + 1].replace(',', ' ').split()]
                        for t in toks:
                            if (not t.startswith('-') and toks[toks.index(t) - 1] not in ('--group', '--features')):
                                built[int(leg)][t.lstrip('+')] = (t.startswith('+'), feats)
            for line in st.splitlines():
                if re.search(r'\bcargo (test|build|nextest|check)\b', line) and not re.match(r'\s+- name:', line) \
                        and 'ci_built_tests.py run --' not in line and 'ci_built_tests.py phase' not in line:
                    out.append(f'{job}: cargo outside a declared build step: {line.strip()}')
            for cmd in re.findall(r'ci_built_tests\.py run -- (cargo test\b.*)', st):
                # The runner's own reader, so the check accepts exactly the
                # command lines `run` does.
                quiet, sys.stderr = sys.stderr, io.StringIO()
                try:
                    sel = parse_cargo_test(cmd.split())
                except SystemExit:
                    out.append(f'{job}: the runner refuses `{cmd.strip()}`: {sys.stderr.getvalue().strip()}')
                    continue
                finally:
                    sys.stderr = quiet
                pkg = sel['package']
                if not seen_build:
                    out.append(f'{job}: `{pkg}` runs before the build step')
                cond = step_condition(st) or ''
                gate = re.findall(r"contains\(fromJSON\(needs\.changes\.outputs\.pkgs\), '([^']+)'\)", cond)
                if gate and gate != [pkg]:
                    out.append(f'{job}: the `{pkg}` step is gated on {gate}')
                feats = sel['features']
                for leg in [int(x) for x in re.findall(r'matrix\.shard == (\d+)', cond)] or legs:
                    ran[leg][pkg] = (not gate, feats)
        for leg in legs:
            for pkg in sorted(set(built[leg]) | set(ran[leg])):
                b, r = built[leg].get(pkg), ran[leg].get(pkg)
                if b is None or r is None:
                    out.append(f'{job} leg {leg}: `{pkg}` is {"run but not built" if b is None else "built but run by no step"}')
                elif b[0] != r[0]:
                    out.append(f'{job} leg {leg}: `{pkg}` needs `+` exactly when its step has no selection condition')
                elif set(r[1]) - set(b[1]):
                    out.append(f'{job} leg {leg}: `{pkg}` runs with features the build lacks: {sorted(set(r[1]) - set(b[1]))}')
    return out


# ------------------------------------------------------------------ modes

def cmd_phase(name, cmd):
    begin = time.time_ns()
    rc = subprocess.run(cmd).returncode
    append('phases.jsonl', {'name': name, 'begin': begin, 'end': time.time_ns()})
    return rc


def cmd_record(argv):
    """Record a binary under the target directory; run anything else for real.
    Cargo hands rustdoc the same runner (`--test-runtool`), so each doctest
    rustdoc runs arrives here from its temporary directory, and it runs. A
    target binary the build did not list is recorded, and the build refuses it."""
    exe, seq = os.path.realpath(argv[0]), time.monotonic_ns()
    if not exe.startswith(os.path.realpath(target_dir()) + os.sep):
        env = {k: v for k, v in os.environ.items() if not k.startswith('CI_BUILT_TESTS_')}
        if os.environ.get('CI_BUILT_TESTS_DYLD') and 'DYLD_FALLBACK_LIBRARY_PATH' not in env:
            env['DYLD_FALLBACK_LIBRARY_PATH'] = os.environ['CI_BUILT_TESTS_DYLD']
        sys.stdout.flush()
        os.execve(argv[0], argv, env)
    with open(os.path.join(os.environ['CI_BUILT_TESTS_RECORD_DIR'], f'{seq}.json'), 'w') as f:
        json.dump({'executable': exe, 'cwd': os.getcwd(), 'env': dict(os.environ), 'seq': seq}, f)
    return 0


def cmd_build(group, features, specs):
    built, deselected = selected_packages(specs, os.environ.get('CI_SELECTED_PACKAGES'))
    groups = load_groups()
    if group in groups or any(set(g['packages']) & set(built) for g in groups.values()):
        die(f'group {group!r} or one of its packages is already built')
    for name in deselected:
        print(f'selection: not building {name} (not in the selection)')
    features = [f for f in features if f.split('/')[0] in built]
    entry = {'packages': built, 'deselected': deselected, 'features': features, 'binaries': [],
             'base_keys': sorted(os.environ), 'native_dirs': []}
    if built:
        args = ['--locked'] + [a for p in built for a in ('-p', p)]
        args += ['--features', ','.join(features)] if features else []
        begin = time.time_ns()
        print(f'group {group}: cargo test --no-run {" ".join(args)}', flush=True)
        # `--timings` writes the group's compile report under the target's
        # `cargo-timings`, where the job's collect step picks it up; the
        # recording run below compiles nothing, so it records no report.
        proc = subprocess.Popen(['cargo', 'test', '--no-run', '--timings',
                                 '--message-format=json-render-diagnostics'] + args,
                                stdout=subprocess.PIPE, text=True, cwd=ROOT)
        artifacts, native = {}, set()
        for line in proc.stdout:
            msg = json.loads(line) if line.startswith('{') else {}
            if msg.get('reason') == 'build-script-executed':
                native |= {p.split('=', 1)[-1] for p in msg.get('linked_paths', [])}
            kind = artifact_kind(msg.get('target', {}).get('kind', []))
            if msg.get('reason') == 'compiler-artifact' and msg.get('executable') \
                    and msg.get('profile', {}).get('test') and kind:
                exe = os.path.realpath(msg['executable'])
                artifacts[exe] = {'executable': exe, 'kind': kind, 'target': msg['target']['name'],
                                  'src_path': msg['target']['src_path']}
        if proc.wait() != 0:
            die(f'group {group}: the build failed')
        # The recording run is outside the declared step, so `audit` refuses it
        # if it compiles anything. It is a whole `cargo test`, so cargo runs the
        # group's doctests after the recorder, over the same units.
        append('phases.jsonl', {'name': f'build {group}', 'begin': begin, 'end': time.time_ns()})
        record_dir = state_path(f'record-{group}')
        shutil.rmtree(record_dir, ignore_errors=True)
        os.makedirs(record_dir)
        runner = 'CARGO_TARGET_' + host_triple().upper().replace('-', '_') + '_RUNNER'
        if runner in os.environ or any(c.isspace() for c in sys.executable + SELF):
            die(f'{runner} is already set, or a path holds whitespace a cargo runner cannot carry')
        env = dict(os.environ, CI_BUILT_TESTS_RECORD_DIR=record_dir)
        if sys.platform == 'darwin':
            env['CI_BUILT_TESTS_DYLD'] = cargo_dylib_path(
                sorted(native), target_dir(), rustc('--print', 'sysroot').strip(), host_triple(),
                os.environ.get('DYLD_FALLBACK_LIBRARY_PATH', ''), os.environ.get('HOME', ''))
        env[runner] = f'{sys.executable} -B {SELF} record'
        if subprocess.run(['cargo', 'test'] + args, env=env, cwd=ROOT).returncode != 0:
            die(f'group {group}: the recording run or the doctests failed')
        records = sorted((json.load(open(os.path.join(record_dir, f))) for f in os.listdir(record_dir)),
                         key=lambda r: r['seq'])
        seen = [r['executable'] for r in records]
        if set(seen) != set(artifacts) or len(seen) != len(set(seen)):
            differ = sorted(os.path.basename(e) for e in set(seen) ^ set(artifacts))
            die(f'group {group}: the build listed {len(artifacts)} test binaries and cargo ran '
                f'{len(seen)}; these differ: {", ".join(differ) or "(one ran twice)"}')
        for r in records:
            entry['binaries'].append(dict(artifacts[r['executable']], package=r['env'].get('CARGO_PKG_NAME'),
                                          cwd=r['cwd'], delta=env_delta(os.environ, r['env'], [runner])))
        entry['native_dirs'] = sorted(native)
    else:
        # Nothing to build is still a declared step, so a job whose selection
        # names none of its packages audits clean.
        now = time.time_ns()
        append('phases.jsonl', {'name': f'build {group}', 'begin': now, 'end': now})
    groups[group] = entry
    with open(state_path('groups.json'), 'w') as f:
        json.dump(groups, f, indent=1, sort_keys=True)
    print(f'group {group}: {len(entry["binaries"])} test binaries built for {len(built)} package(s)')
    return 0


def cmd_run(argv):
    sel = parse_cargo_test(argv)
    home = [g for g in load_groups().values() if sel['package'] in g['packages']]
    if not home:
        die(f'{sel["package"]} was not built by this leg\'s build step: add it there')
    group = home[0]
    missing = sorted(set(sel['features']) - set(group['features']))
    if missing:
        die(f'the build did not enable {missing}; add them to its --features')
    config = os.path.join(ROOT, '.cargo', 'config.toml')
    soft = soft_env_keys(open(config).read()) if os.path.exists(config) else set()
    dyld = None
    if sys.platform == 'darwin':
        dyld = cargo_dylib_path(group['native_dirs'], target_dir(), rustc('--print', 'sysroot').strip(),
                                host_triple(), os.environ.get('DYLD_FALLBACK_LIBRARY_PATH', ''),
                                os.environ.get('HOME', ''))
    args = ([sel['filter']] if sel['filter'] else []) + sel['args']
    # A run with a name filter runs part of the binary, so it does not count as
    # the binary's run for `audit`.
    whole = not sel['filter'] and all(a in WHOLE_RUN_ARGS or a.startswith('--test-threads=') for a in sel['args'])
    failed = []
    for b in select_binaries(group['binaries'], sel):
        env = replay_env(os.environ, b['delta'], set(group['base_keys']), soft)
        if dyld and 'DYLD_FALLBACK_LIBRARY_PATH' not in b['delta']['changed']:
            env['DYLD_FALLBACK_LIBRARY_PATH'] = dyld
        src = os.path.relpath(b['src_path'], b['cwd'])
        label = src if b['kind'] == 'test' else f'unittests {src}'
        print(f'     Running {label} ({os.path.relpath(b["executable"], ROOT)})', flush=True)
        rc = subprocess.run([b['executable']] + args, env=env, cwd=b['cwd']).returncode
        append('ran.jsonl', {'executable': b['executable'], 'rc': rc, 'whole': whole})
        if rc != 0:
            how = f'signal: {-rc}' if rc < 0 else f'exit status: {rc}'
            print(f'error: test failed: {b["executable"]} ({how})', file=sys.stderr, flush=True)
            failed.append(b['target'])
            if not sel['no_fail_fast']:
                break
    if failed:
        print(f'error: {len(failed)} target(s) failed: {", ".join(failed)}', file=sys.stderr)
        return 101
    return 0


def cmd_reclaim():
    """The pre-existing `find -executable -delete`, keeping binaries still to run."""
    ran = {r['executable'] for r in read_lines('ran.jsonl') if r['whole']}
    keep = {b['executable'] for g in load_groups().values() for b in g['binaries']} - ran
    deps, removed = os.path.join(target_dir(), 'debug', 'deps'), 0
    for name in sorted(os.listdir(deps)) if os.path.isdir(deps) else []:
        path = os.path.join(deps, name)
        if (os.path.isfile(path) and not os.path.islink(path) and os.access(path, os.X_OK)
                and not KEPT_FILE.search(name)
                and not name.startswith('build_script_build') and os.path.realpath(path) not in keep):
            os.remove(path)
            removed += 1
    print(f'reclaim: removed {removed} executable(s), kept {len(keep)} still to run')
    return 0


def cmd_audit(floor, not_run):
    groups, phases = load_groups(), read_lines('phases.jsonl')
    if not groups or not phases:
        die('nothing was built through this script in this job')
    ran = {r['executable'] for r in read_lines('ran.jsonl') if r['whole']}
    built = [b for g in groups.values() for b in g['binaries']]
    names = {f'{b["package"]}:{b["target"]}' for b in built}
    never = sorted(f'{b["package"]}:{b["target"]}' for b in built
                   if b['executable'] not in ran and f'{b["package"]}:{b["target"]}' not in not_run)
    full = not any(g['deselected'] for g in groups.values())
    bad = []
    if never:
        bad.append(f'{len(never)} built test binaries ran unfiltered in no step: {", ".join(never)}')
    if full and set(not_run) - names:
        bad.append(f'--not-run names binaries this leg does not build: {", ".join(sorted(set(not_run) - names))}')
    print(f'binaries: {len(built)} built, {len(ran)} run')
    if floor is not None and full and len(built) < floor:
        bad.append(f'{len(built)} test binaries were built, below this leg\'s floor of {floor}')
    elif floor is not None and not full:
        print('floor: not applied, the selection left packages out of this leg')
    profile = os.path.join(target_dir(), 'debug')
    if not os.path.isdir(os.path.join(profile, '.fingerprint')):
        bad.append(f'{profile}/.fingerprint does not exist, so no compile could be dated')
    units = compiled_units(profile, min(p['begin'] for p in phases))
    counts, libs, stray = attribute(units, phases, SLACK_NS)
    for name, n in counts.items():
        print(f'compiled units, {name}: {n}')
    multi = sorted(((n, c) for c, n in libs.items() if n > 1), reverse=True)
    print(f'compiled units, total: {len(units)}; library builds of {len(libs)} crates, '
          f'{len(multi)} built more than once: ' + ', '.join(f'{c} x{n}' for n, c in multi[:15]))
    if stray:
        bad.append(f'{len(stray)} unit(s) compiled outside every declared build step, so a step '
                   f'rebuilt what the leg had built: {", ".join(stray[:40])}')
    for msg in bad:
        print(f'::error::{msg}')
    return 1 if bad else 0


# ------------------------------------------------------------------ self-test

def self_test():
    fails = []

    def check(name, got, want):
        if got != want:
            fails.append(f'{name}: got {got!r}, wanted {want!r}')

    def refused(name, fn):
        quiet, sys.stderr = sys.stderr, io.StringIO()
        try:
            fn()
            fails.append(f'{name}: was not refused')
        except SystemExit as e:
            if e.code != 2:
                fails.append(f'{name}: exited {e.code}')
        finally:
            sys.stderr = quiet

    check('selection names one', selected_packages(['a', '+b', 'c'], '["c"]'), (['b', 'c'], ['a']))
    check('an empty list builds only the + packages', selected_packages(['a', '+b'], '[]'), (['b'], ['a']))
    check('no classifier builds all', selected_packages(['a', '+b'], ''), (['a', 'b'], []))
    check('a longer name is no match', selected_packages(['cerulion_viz'], '["cerulion_vizd"]'), ([], ['cerulion_viz']))
    refused('malformed selection', lambda: selected_packages(['a'], '[a'))
    refused('selection of non-names', lambda: selected_packages(['a'], '[1]'))
    refused('repeated spec', lambda: selected_packages(['a', '+a'], '[]'))
    refused('a bare +', lambda: selected_packages(['+'], '[]'))
    refused('a JSON string is no list', lambda: selected_packages(['cerulion_viz'], '"cerulion_vizd"'))

    p = lambda line: parse_cargo_test(line.split())
    s = p('cargo test -p cerulion_cli --test credit_death_e2e_test c7_')
    check('test and filter', (s['package'], s['tests'], s['filter'], s['args']),
          ('cerulion_cli', ['credit_death_e2e_test'], 'c7_', []))
    s = p('cargo test -p cerulion_netd --features wan -- --test-threads=1')
    check('features qualify, args pass', (s['features'], s['args']), (['cerulion_netd/wan'], ['--test-threads=1']))
    s = p('cargo test -p x --no-fail-fast --lib openh264_fetch')
    check('lib with filter', (s['lib'], s['no_fail_fast'], s['filter']), (True, True, 'openh264_fetch'))
    s = p('cargo test --locked -p x --features y/z,w')
    check('locked passes, features keep or gain a package', s['features'], ['y/z', 'x/w'])
    for bad_line in ('cargo test -p x --doc', 'cargo test -p x -p y', 'cargo test --lib', 'cargo test -p',
                     'cargo nextest run -p x', 'cargo test -p x a b', 'cargo test -p x --release'):
        refused(f'refuses `{bad_line}`', lambda: p(bad_line))

    m = [{'package': 'p', 'kind': 'lib', 'target': 'p', 'executable': '/l'},
         {'package': 'p', 'kind': 'bin', 'target': 'p-cli', 'executable': '/b'},
         {'package': 'p', 'kind': 'test', 'target': 'one_test', 'executable': '/t1'},
         {'package': 'p', 'kind': 'test', 'target': 'two_test', 'executable': '/t2'},
         {'package': 'q', 'kind': 'test', 'target': 'one_test', 'executable': '/q1'}]
    pick = lambda line: [b['executable'] for b in select_binaries(m, p(line))]
    check('whole package in order', pick('cargo test -p p'), ['/l', '/b', '/t1', '/t2'])
    check('a shared test name stays in its package', pick('cargo test -p q --test one_test'), ['/q1'])
    check('selectors keep manifest order', pick('cargo test -p p --test two_test --lib'), ['/l', '/t2'])
    check('bins only', pick('cargo test -p p --bins'), ['/b'])
    refused('unknown test target', lambda: pick('cargo test -p p --test three_test'))
    refused('no lib', lambda: pick('cargo test -p q --lib'))
    refused('unbuilt package', lambda: pick('cargo test -p r'))
    check('kinds', [artifact_kind(k) for k in (['lib'], ['cdylib', 'rlib'], ['proc-macro'], ['bin'], ['test'],
                                               ['example'], ['bench'])],
          ['lib', 'lib', 'lib', 'bin', 'test', None, None])

    base = {'HOME': '/h', 'RUST_LOG': 'info', 'GONE': '1', 'PWD': '/x'}
    rec = {'HOME': '/h', 'RUST_LOG': 'info', 'CARGO_PKG_NAME': 'p', 'PWD': '/y', 'R': 'run',
           'CI_BUILT_TESTS_RECORD_DIR': '/r', 'IOX2_LOG_LEVEL': 'error'}
    check('delta', env_delta(base, rec, ['R']),
          {'changed': {'CARGO_PKG_NAME': 'p', 'IOX2_LOG_LEVEL': 'error'}, 'removed': ['GONE']})
    delta = {'changed': {'CARGO_PKG_NAME': 'p', 'RUST_LOG': 'warn', 'CERULION_LOGIN_GATE': 'off'}, 'removed': ['GONE']}
    check('a soft key yields to the step, a forced one does not',
          replay_env({'RUST_LOG': 'debug', 'CERULION_LOGIN_GATE': 'on', 'GONE': '1'}, delta, set(), {'RUST_LOG'}),
          {'RUST_LOG': 'debug', 'CERULION_LOGIN_GATE': 'off', 'CARGO_PKG_NAME': 'p'})
    check('a soft key the step does not set is replayed',
          replay_env({}, delta, set(), {'RUST_LOG'})['RUST_LOG'], 'warn')
    check('a soft key the build env held is replayed',
          replay_env({'RUST_LOG': 'debug'}, delta, {'RUST_LOG'}, {'RUST_LOG'})['RUST_LOG'], 'warn')
    check('soft config keys', soft_env_keys('[env]\nA = "x"\nB = { value = "off", force = true }\n'
                                            'C = { value = "w", force = false }  # note\n[alias]\nD = "t"\n'),
          {'A', 'C'})
    check('dylib path', cargo_dylib_path(['/t/debug/build/z/out', '/elsewhere', '/t/debug/build/a/out'], '/t',
                                          '/s', 'h', '', '/u'),
          '/t/debug/build/a/out:/t/debug/build/z/out:/t/debug:/t/debug/deps:/s/lib/rustlib/h/lib:'
          '/s/lib:/u/lib:/usr/local/lib:/usr/lib')
    check('dylib path keeps an existing value', cargo_dylib_path([], '/t', '/s', 'h', '/e', '/u'),
          '/t/debug:/t/debug/deps:/s/lib/rustlib/h/lib:/s/lib:/e')

    phases = [{'name': 'w', 'begin': 100, 'end': 200}, {'name': 'b', 'begin': 300, 'end': 400}]
    check('attribution', attribute([('x (build)', 150), ('y (test)', 350), ('z (build)', 250),
                                    ('x (build)', 295), ('w (build)', 205)], phases, 10),
          ({'w': 1, 'b': 2}, {'x': 2}, ['w (build)', 'z (build)']))
    close = [{'name': 'a', 'begin': 100, 'end': 200}, {'name': 'b', 'begin': 205, 'end': 300}]
    check('a unit inside a step belongs to it, not to a neighbour within the slack',
          attribute([('x (build)', 210)], close, 50)[0], {'a': 0, 'b': 1})
    check('unit kinds', [unit_kind(f) for f in (['lib-x', 'lib-x.json'], ['test-lib-x'],
                                                 ['build-script-build-script-build'],
                                                 ['run-build-script-build-script-build'])],
          ['build', 'test', 'build script', 'build script run'])
    with tempfile.TemporaryDirectory() as tmp:
        for d, t in (('.fingerprint/old-1', 50), ('.fingerprint/new-2', 150), ('.fingerprint/build-dep-3', 160),
                     ('.fingerprint/fresh-4', None), ('build/ring-5', 170), ('build/zstd-6', 20)):
            os.makedirs(os.path.join(tmp, d))
            open(os.path.join(tmp, d, 'lib-x'), 'w').close()
            if t is not None:
                open(os.path.join(tmp, d, 'invoked.timestamp'), 'w').close()
                os.utime(os.path.join(tmp, d, 'invoked.timestamp'), ns=(t, t))
        check('compiled units', sorted(compiled_units(tmp, 100)),
              [('build-dep (build)', 160), ('new (build)', 150), ('ring (build script run)', 170)])

    good = ("""jobs:
  t:
    strategy:
      matrix:
        shard: [0, 1]
    steps:
      - name: Build
        run: |
          case "${{ matrix.shard }}" in
            0) python3 ci_built_tests.py build --group g +a b --features b/w ;;
            1) python3 ci_built_tests.py build --group g c ;;
          esac
      - name: a
        if: matrix.shard == 0
        run: python3 ci_built_tests.py run -- cargo test -p a
      - name: b
        if: matrix.shard == 0 && contains(fromJSON(needs.changes.outputs.pkgs), 'b')
        run: python3 ci_built_tests.py run -- cargo test -p b --features w -- --test-threads=1
      - name: c
        if: matrix.shard == 1 && contains(fromJSON(needs.changes.outputs.pkgs), 'c')
        run: |
          python3 ci_built_tests.py phase x -- cargo build -p y
          python3 ci_built_tests.py run -- cargo test -p c
      - name: Audit
        run: python3 ci_built_tests.py audit --floor 3
""")
    check('wiring: a consistent job', workflow_complaints(good), [])
    for spelling in ('--package a', '--locked -p a'):
        check(f'wiring: the runner\'s spelling `{spelling}` reads as `-p a`',
              workflow_complaints(good.replace('run -- cargo test -p a', f'run -- cargo test {spelling}')), [])
    check('wiring: a run line the runner refuses', len(workflow_complaints(
        good.replace('run -- cargo test -p a', 'run -- cargo test -p a --release'))), 2)
    check('wiring: comma feature lists compare as sets', workflow_complaints(
        good.replace('--features b/w', '--features b/v,b/w').replace('--features w --', '--features v,w --')), [])
    for name, bad, want in (
            ('a + on a gated step', good.replace('+a b', 'a +b'), 2),
            ('a package no step runs', good.replace('build --group g c', 'build --group g c d'), 1),
            ('a step on an unbuilt package', good.replace('--group g c ;;', '--group g ;;'), 1),
            ('a feature the build lacks', good.replace('--features b/w', ''), 1),
            ('an undeclared cargo build', good.replace('phase x -- cargo build', 'cargo build'), 1),
            ('a run before the build', good.replace('      - name: Build', '      - name: e\n        run: python3 '
                                                     'ci_built_tests.py run -- cargo test -p a\n      - name: Build'), 2),
            ('no audit', good.split('      - name: Audit')[0], 1),
            ('a feature list the build splits but lacks one of', good.replace('--features w -- --test-threads=1',
                                                                              '--features w,v -- --test-threads=1'), 1),
            ('an audit before the last run', good.replace('      - name: c\n', '      - name: Audit\n        run: '
                                                          'python3 ci_built_tests.py audit\n      - name: c\n')
             .split('      - name: Audit\n        run: python3 ci_built_tests.py audit --floor 3')[0], 1),
            ('a gated audit', good.replace('      - name: Audit\n', '      - name: Audit\n        if: always()\n'), 1)):
        check(f'wiring: {name}', len(workflow_complaints(bad)), want)
    # A job whose runner follows `code`, as `test-macos`'s does.
    routed = good.replace('  t:\n', "  t:\n    runs-on: ${{ needs.changes.outputs.code == 'true' && 'macos-latest'"
                                    " || 'ubuntu-latest' }}\n")
    audit_on_code = '      - name: Audit\n        if: %s\n' % CODE_GATE

    def build_on(cond):
        return '      - name: Build\n        if: %s\n' % cond
    check('wiring: an audit and a build both gated on code in a routed job', workflow_complaints(
        routed.replace('      - name: Audit\n', audit_on_code).replace('      - name: Build\n', build_on(CODE_GATE))), [])
    check('wiring: a build gated on code and a leg in a routed job', workflow_complaints(
        routed.replace('      - name: Audit\n', audit_on_code)
        .replace('      - name: Build\n', build_on('matrix.shard != 3 && ' + CODE_GATE))), [])
    check('wiring: a build gated on code in an expression wrapper', workflow_complaints(
        routed.replace('      - name: Audit\n', audit_on_code)
        .replace('      - name: Build\n', build_on('${{ matrix.shard != 3 && %s }}' % CODE_GATE))), [])
    check('wiring: an `if:` inside a run script is no step condition', workflow_complaints(
        good.replace('python3 ci_built_tests.py audit --floor 3', '|\n          if: x\n          python3 '
                     'ci_built_tests.py audit --floor 3')), [])
    check('wiring: a build gated on code in a folded condition', workflow_complaints(
        routed.replace('      - name: Audit\n', audit_on_code).replace(
            '      - name: Build\n', '      - name: Build\n        if: >-\n          matrix.shard != 3\n'
            '          && %s\n' % CODE_GATE)), [])
    for name, bad, want in (
            ('an audit gated on code beside another term', routed.replace(
                '      - name: Audit\n', f'      - name: Audit\n        if: always() && {CODE_GATE}\n')
             .replace('      - name: Build\n', build_on(CODE_GATE)), 1),
            ('an audit gated on code after a build that is not', routed.replace('      - name: Audit\n', audit_on_code), 1),
            ('an audit gated on another term after a build gated on code', routed.replace(
                '      - name: Audit\n', '      - name: Audit\n        if: always()\n')
             .replace('      - name: Build\n', build_on(CODE_GATE)), 1),
            ('a build gated on code or a leg', routed.replace('      - name: Audit\n', audit_on_code)
             .replace('      - name: Build\n', build_on('matrix.shard == 0 || ' + CODE_GATE)), 1),
            ('a build gated on the code gate and then or', routed.replace('      - name: Audit\n', audit_on_code)
             .replace('      - name: Build\n', build_on(CODE_GATE + ' && matrix.shard == 0 || matrix.shard == 1')), 1),
            ('a build gated on the negated code gate', routed.replace('      - name: Audit\n', audit_on_code)
             .replace('      - name: Build\n', build_on('!(%s)' % CODE_GATE)), 1),
            ('a code-gated audit in a job whose runner does not follow code', good.replace(
                '      - name: Audit\n', audit_on_code).replace('      - name: Build\n', build_on(CODE_GATE)), 1),
            ('a code-gated audit in a job on a fixed runner', good.replace('  t:\n', '  t:\n    runs-on: macos-latest\n')
             .replace('      - name: Audit\n', audit_on_code).replace('      - name: Build\n', build_on(CODE_GATE)), 1),
            ('a build gated on code inside a negation', routed.replace('      - name: Audit\n', audit_on_code)
             .replace('      - name: Build\n', build_on('!(matrix.shard != 3 && %s && matrix.shard != 4)' % CODE_GATE)), 1),
            ('an audit whose condition is its first key', good.replace(
                '      - name: Audit\n', '      - if: always()\n        name: Audit\n'), 1)):
        check(f'wiring: {name}', len(workflow_complaints(bad)), want)
    end_to_end(check, refused)
    for f in fails:
        print(f'FAIL {f}', file=sys.stderr)
    print('ci_built_tests --self-test: ' + ('FAILED' if fails else 'OK'))
    return 1 if fails else 0


FAKE_CARGO = """import json, os, subprocess, sys
a, tmp = sys.argv[1:], os.environ['FAKE_TMP']
with open(os.path.join(tmp, 'cargo-args'), 'a') as f:
    f.write(' '.join(a) + '\\n')
exes = [os.path.join(tmp, 'target', 'debug', 'deps', n) for n in ('pkg-lib', 'one_test')]
if a[:2] == ['test', '--no-run']:
    for exe, kind, name in zip(exes, (['lib'], ['test']), ('pkg', 'one_test')):
        print(json.dumps({'reason': 'compiler-artifact', 'executable': exe, 'profile': {'test': True},
                          'target': {'kind': kind, 'name': name, 'src_path': os.path.join(tmp, 'pkg', 'src', name)}}))
    print(json.dumps({'reason': 'compiler-artifact', 'executable': exe + '-bin', 'profile': {'test': False},
                      'target': {'kind': ['bin'], 'name': 'pkg', 'src_path': 'x'}}))
elif a[:1] == ['test']:
    runner = [v for k, v in os.environ.items() if k.endswith('_RUNNER')][0].split()
    for exe in exes:
        env = dict(os.environ, CARGO_PKG_NAME='pkg', FROM_CARGO='yes')
        subprocess.run(runner + [exe, 'x'], env=env, cwd=os.path.join(tmp, 'pkg'), check=True)
    doctest = os.path.join(tmp, 'rustdoctest', 'rust_out')
    os.makedirs(os.path.dirname(doctest))
    with open(doctest, 'w') as f:
        f.write('#!/bin/sh\\necho "doctest $CI_BUILT_TESTS_RECORD_DIR" >> "$FAKE_TMP/trace"\\n'
                '[ -z "$FAKE_DOCTEST_FAILS" ]\\n')
    os.chmod(doctest, 0o755)
    sys.exit(101 if subprocess.run(runner + [doctest]).returncode else 0)
"""


def end_to_end(check, refused):
    """build, run, reclaim and audit over a fake cargo and fake test binaries."""
    saved = dict(os.environ)
    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, 'bin'))
        os.makedirs(os.path.join(tmp, 'target', 'debug', 'deps'))
        os.makedirs(os.path.join(tmp, 'pkg', 'src'))
        for name, body in (('cargo', f'#!{sys.executable}\n' + FAKE_CARGO),
                           ('rustc', '#!/bin/sh\n[ "$1" = -vV ] && echo "host: fake-host" || echo /sysroot\n'),
                           ('pkg-lib', '#!/bin/sh\necho "lib $* $CARGO_PKG_NAME $FROM_CARGO $(pwd)" >> "$FAKE_TMP/trace"\n'
                                       '[ "$1" != fail ]\n'),
                           ('one_test', '#!/bin/sh\necho "one_test $*" >> "$FAKE_TMP/trace"\n')):
            path = os.path.join(tmp, 'bin' if name in ('cargo', 'rustc') else 'target/debug/deps', name)
            with open(path, 'w') as f:
                f.write(body)
            os.chmod(path, 0o755)
        os.makedirs(os.path.join(tmp, 'target', 'debug', '.fingerprint'))
        os.environ.update(PATH=os.path.join(tmp, 'bin') + os.pathsep + saved['PATH'], FAKE_TMP=tmp,
                          RUNNER_TEMP=tmp, CARGO_TARGET_DIR=os.path.join(tmp, 'target'), CI_SELECTED_PACKAGES='["pkg"]')
        trace = lambda: open(os.path.join(tmp, 'trace')).read().splitlines() if os.path.exists(
            os.path.join(tmp, 'trace')) else []
        quiet = sys.stdout, sys.stderr
        sys.stdout, sys.stderr = io.StringIO(), io.StringIO()
        try:
            cmd_phase('workspace', ['true'])
            check('end to end: build', cmd_build('g', ['pkg/f'], ['pkg']), 0)
            grouped = [l.split() for l in open(os.path.join(tmp, 'cargo-args')).read().splitlines()
                       if l.startswith('test --no-run')]
            check('end to end: the grouped build records cargo timings',
                  [len(grouped), all('--timings' in g for g in grouped)], [1, True])
            check('end to end: the recording ran no test, and a doctest ran without the recorder state',
                  trace(), ['doctest '])
            check('end to end: one test, serial', cmd_run('cargo test -p pkg --test one_test -- --test-threads=1'
                                                         .split()), 0)
            check('end to end: whole package, filter, cargo env and cwd',
                  (cmd_run('cargo test -p pkg c7_'.split()), trace()[-2:]),
                  (0, [f'lib c7_ pkg yes {os.path.realpath(os.path.join(tmp, "pkg"))}', 'one_test c7_']))
            check('end to end: fail fast stops at the first failure', (cmd_run('cargo test -p pkg fail'.split()),
                                                                     trace()[-1].split()[:2]), (101, ['lib', 'fail']))
            check('end to end: filtered runs alone leave the lib unrun', cmd_audit(2, set()), 1)
            check('end to end: the whole package', cmd_run('cargo test -p pkg'.split()), 0)
            check('end to end: audit passes', cmd_audit(2, set()), 0)
            check('end to end: audit floor', cmd_audit(3, set()), 1)
            check('end to end: an unknown --not-run entry', cmd_audit(2, {'pkg:gone'}), 1)
            refused('end to end: a feature the build lacks', lambda: cmd_run('cargo test -p pkg --features g'.split()))
            deps = os.path.join(tmp, 'target', 'debug', 'deps')
            for name in ('stale-1', 'lib.rlib', 'one_test-2', 'libfix.dylib', 'libfix.so.1'):
                path = os.path.join(deps, name)
                with open(path, 'w') as f:
                    f.write('#!/bin/sh\n')
                os.chmod(path, 0o755)
            groups = load_groups()
            groups['g']['binaries'][1]['executable'] = os.path.realpath(os.path.join(deps, 'one_test-2'))
            with open(state_path('groups.json'), 'w') as f:
                json.dump(groups, f)
            check('end to end: reclaim keeps what is still to run', (cmd_reclaim(), sorted(os.listdir(deps))),
                  (0, ['lib.rlib', 'libfix.dylib', 'libfix.so.1', 'one_test-2']))
            check('end to end: a binary run only under a filter has not run', cmd_audit(2, set()), 1)
            check('end to end: a run with --ignored is no whole run', cmd_run(
                'cargo test -p pkg --test one_test -- --ignored'.split()), 0)
            check('end to end: so the binary is still unrun', cmd_audit(2, set()), 1)
            open(state_path('ran.jsonl'), 'a').write(json.dumps({'executable': os.path.realpath(
                os.path.join(deps, 'one_test-2')), 'rc': 0, 'whole': True}) + '\n')
            check('end to end: a whole run clears it', cmd_audit(2, set()), 0)
            stray = os.path.join(tmp, 'target', 'debug', '.fingerprint', 'stray-1')
            os.makedirs(stray)
            open(os.path.join(stray, 'invoked.timestamp'), 'w').close()
            check('end to end: a unit compiled after every step', cmd_audit(2, set()), 1)
            refused('end to end: a floor that is no number', lambda: main(['audit', '--floor', 'x']))
            os.environ.update(RUNNER_TEMP=os.path.join(tmp, 'second'), FAKE_DOCTEST_FAILS='1')
            shutil.rmtree(os.path.join(tmp, 'rustdoctest'))
            refused('end to end: a failing doctest fails the build', lambda: cmd_build('g', [], ['pkg']))
            os.environ.update(RUNNER_TEMP=os.path.join(tmp, 'third'), CI_SELECTED_PACKAGES='[]')
            check('end to end: a selection naming no package builds nothing',
                  (cmd_build('g', [], ['pkg']), load_groups()['g']['binaries']), (0, []))
            check('end to end: and audits clean', cmd_audit(2, set()), 0)
        except SystemExit:
            check('end to end: no mode stopped the run', sys.stderr.getvalue().strip()[-300:], '')
        finally:
            sys.stdout, sys.stderr = quiet
            os.environ.clear()
            os.environ.update(saved)


def main(argv):
    mode, rest = (argv[0], argv[1:]) if argv else ('', [])
    if mode == '--self-test':
        return self_test()
    if mode == 'record':
        return cmd_record(rest)
    if mode == 'run' and rest[:1] == ['--']:
        return cmd_run(rest[1:])
    if mode == 'phase' and len(rest) > 2 and rest[1] == '--':
        return cmd_phase(rest[0], rest[2:])
    if mode == 'check-workflow' and len(rest) == 1:
        found = workflow_complaints(open(rest[0]).read())
        for msg in found:
            print(f'::error::{msg}')
        print(f'check-workflow: {len(job_blocks(open(rest[0]).read()))} job(s), {len(found)} complaint(s)')
        return 1 if found or not job_blocks(open(rest[0]).read()) else 0
    if mode == 'reclaim' and not rest:
        return cmd_reclaim()
    if mode == 'build':
        group, features, specs, i = None, [], [], 0
        while i < len(rest):
            if rest[i] in ('--group', '--features') and i + 1 < len(rest):
                if rest[i] == '--group':
                    group = rest[i + 1]
                else:
                    features += rest[i + 1].replace(',', ' ').split()
                i += 2
            else:
                specs += rest[i].split()
                i += 1
        if group and specs and all('/' in f for f in features):
            return cmd_build(group, features, specs)
    if mode == 'audit' and len(rest) % 2 == 0:
        floor, not_run = None, set()
        for flag, value in zip(rest[::2], rest[1::2]):
            if flag == '--floor':
                floor = int(value) if value.isdigit() else die(f'--floor takes a number, got {value!r}')
            elif flag == '--not-run':
                not_run.add(value)
            else:
                die(f'unknown audit flag {flag!r}')
        return cmd_audit(floor, not_run)
    die('usage: ' + ' | '.join(('build --group NAME [--features PKG/F] SPEC...', 'run -- cargo test ...',
                                'reclaim', 'phase NAME -- CMD...', 'audit [--floor N] [--not-run P:T]',
                                'check-workflow FILE', '--self-test')))


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
