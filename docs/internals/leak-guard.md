# The leak guard

Everything this repository publishes is public: code, comments, docs, logs, data files,
file and branch names, commit messages, commit identities, pull request titles and
bodies, and the images and clips under `docs/media`. The leak guard is one scanner,
`tools/scripts/leak_scan.py` (stdlib Python 3, no dependencies), that keeps machine
names, addresses, home paths, logins, people and location metadata out of all of it.
It runs in three places: the git hooks on your machine, the `lint` job of the main CI
workflow, and the `Leak guard` workflow on every pull request, merge queue batch and
push to `main`.

## What it checks

Two tiers.

**Generic classes** ship in the tree and are shapes only, so nothing in the scanner
names a real machine, address, login or person. `--list-classes` prints the table with
its severity per mode. The shapes: a macOS, Linux or Windows home path with a real
segment; a per-user temp root; an address inside the RFC 6598 shared range; the overlay
vendor's DNS suffix; an RFC 1918 address that is not one of the example values already
in the tree (new text reuses one of those or a documentation range such as
`192.0.2.0/24`); a login at a host after `ssh`, `scp` or `rsync`; a multicast DNS host
name; a personal mail address, or a non-role address at the project domain; a host
identity field (`uname`, `hostname`, `nodename` and friends) carrying a value; an
overlay access control tag; the overlay product words in CI files; and, as a
report-only style class, en and em dashes. Placeholders such as `/Users/someone/`,
`/home/ubuntu/` and `robot-a.local` are published vocabularies, never
length rules.

**Private patterns** never ship. They hold the real names: machine hostnames and short
names, overlay device names, real LAN addresses, logins, people, the private
repository slug, the internal tracker prefix. They load from, in order, the
`LEAK_PATTERNS` environment variable (what CI passes from a secret), the file named by
`LEAK_PATTERNS_FILE`, or `~/.config/cerulion/leak-patterns.txt`. The first source that
yields a pattern wins. Private classes are HARD in every mode and honour no pragma.

Surfaces in `tree` mode: every text file in full (comments, strings, help text, YAML,
Markdown, lock files), path names, symlink targets, binary files through their
printable strings, JSON string leaves and keys (nested and escaped), ANSI-wrapped log
fields, bracket-obfuscated spellings (`[x]yz`), base64 data URIs inside SVG and
Markdown, and compressed PNG text chunks. Every surface is read in several views of
the same text, and a hit in any view counts: the raw text; the text with escape
sequences stripped and Unicode format characters dropped (a zero width space, a soft
hyphen or a word joiner splits a name on a screen and must not split it here); with
`[x]` brackets removed; with JSON escapes decoded; and with backslash escapes (`\n`,
`\t`, `\033[1m`, `\x1b`, `\u{..}`, a backslash before any letter) and `%HH` percent
escapes decoded, twice for a once-nested string. So a name written right after a
literal `\n` in a Rust string, after `\033[1m` in a shell script, or inside a
percent-encoded path has the boundary it has on screen. `diff` mode scans added lines
only (the hooks use it). `messages` mode scans commit messages, author and committer
names and emails (the identity policy accepts the forge noreply address, the forge
web-flow address, project role addresses, or an address listed in
`--allow-email-file`), and pull request title and body. `names` mode scans paths and
the branch name. `media` mode parses PNG, JPEG, GIF, WebP and the MP4 and MOV family
for comments, EXIF, XMP, location boxes and trailing bytes; a document, archive or
container no walker can read is a hit until an allowlist entry names it.

## Install the hooks

```bash
tools/scripts/install_hooks.sh            # once per clone
tools/scripts/install_hooks.sh --status   # what is active
tools/scripts/install_hooks.sh --uninstall
```

The installer copies `tools/hooks` and the scanner to an absolute directory under the
shared git directory and points `core.hooksPath` at it, so every linked worktree on
every branch is covered (a relative hooks path runs nothing in a worktree whose branch
does not carry that directory). The hooks chain to whatever hook was active before, so a
global identity hook keeps running. When the tree's hooks change, each installed hook
says so and names the installer. `--in-tree` uses the relative `tools/hooks` for a
single checkout instead.

`pre-commit` scans the staged added lines, the staged file names, the branch name and
the metadata of staged media. `commit-msg` scans the message and your author and
committer identity. It scans exactly what git will keep of the message file: with
`git commit -m` or `-F` (git tells the hook that no editor ran) every line is part of
the commit, a line starting with `#` included, and so is anything below a scissors
line; when an editor ran, the `#` lines are dropped the way git's default cleanup
drops them (unless `commit.cleanup` keeps them), and the text below git's own
scissors block (the marker, git's two comment lines, then the diff that `-v` appends)
is dropped because git truncates there. A pasted scissors line followed by anything
else is ordinary content. A hook that cannot find the scanner or `python3` prints one
line and lets the commit through; CI is the floor. Bypass once with
`git commit --no-verify` when a hit is a reviewed false positive you are about to
allowlist.

## Supply private patterns locally

Create `~/.config/cerulion/leak-patterns.txt` outside the tree and `chmod 600` it (the
scanner warns when it is wider). One entry per line, `#` comments:

```
word:examplebox7 @host          # case-insensitive, alphanumeric boundaries; joined,
                                # dashed, underscored and spaced spellings all match
text:/home/examplelogin @login  # case-insensitive substring, no boundaries
(?<![a-z])lab[0-9] @device      # anything else is a Python regex
```

**Nicknames go in this list too.** A machine is as often written about by a nickname
as by its host name (by its model, by a colour, or as "the big one"). That shape
carries no host name, no address and no login, so no generic class can see it and
no in-tree regex may carry it: put one entry per nickname here, tagged `@nickname`,
alongside the host names of the same machines. A nickname a writer invents on the
spot is the one leak this guard cannot close by itself, which is why the shipped-text
gate refuses the machine vocabulary it CAN name (`tools/scripts/public_surface_workstate.txt`,
key `our-machines`) and reviewers read for the rest.

A `text:` entry has no boundaries, so keep it for a string that cannot sit inside an
ordinary token. A home directory prefix is safe. A login followed by an at sign is not:
a package id of the shape `<crate>@<version>` ends in the same characters whenever a
crate name ends in that login, and every build log that prints one would read as a hit.
Write that entry as a regex with a left token boundary that also refuses a three-part
version after the at sign, so an address after the at sign still matches:

```
(?<![A-Za-z0-9_-])examplelogin@(?![0-9]+[.][0-9]+[.][0-9]+(?![0-9.])) @login
```

The scanner self-test drives this exact recipe, the `text:` form and a regex entry in
an author name, so all three forms are pinned, not only `word:`.

The trailing tag is optional and comes from a fixed vocabulary (`@host` `@device`
`@person` `@login` `@lan` `@nickname` `@slug` `@hygiene`, in any letter case); it
labels a hit's category without naming anything. The grammar is strict and there is exactly one
shape per line: blank; a comment (`#` first); or one entry, `word:<literal>`,
`text:<literal>` or `<regex>`, then optionally whitespace and the tag, then optionally
whitespace, `#` and a comment. Anything else refuses the whole run with exit 3, naming
the entry's index, the list line and the accepted forms and never the line itself: a
tag outside the vocabulary, words after the tag, a tag glued to the entry or written
without its `@`, a second tag, `#` with no space after it, a `//` comment, `Word:` or
`word: ` (a capital, or whitespace after the colon), a tab inside the entry, a carriage
return (Windows line endings), a `word:` literal holding `@` or `#`. Loading any of those
would give an entry that can match nothing yet counts as LOADED, and a well-formed
neighbour would hide the dead one. A byte order mark is dropped; a list that is not UTF-8
refuses the run. A file that keeps the list inside the checkout is gitignored as
`.leak-patterns.local`; point `LEAK_PATTERNS_FILE` at it. The maintainers hold the
canonical list; ask them for it rather than reconstructing it.

## Read a hit

```
HIT home-mac docs/setup.md:12: /Users/<a real login>/
HIT home-mac docs/setup.md:12: (value masked, 21 chars: /xxxxx/xxxxxxxxxxxxx/)
HIT home-mac docs/setup.md:13: (text withheld: a private pattern touches this line)
HIT lan-addr docs/<lan-addr:xx.xx.xx.x>.md:0: (value masked, 10 chars: xx.xx.xx.x) [name]
HIT private#3@host crates/netd/src/lib.rs:88
HIT private#1@host docs/<private#1@host>-notes.md:0 [name]
REPORT style-dash README.md:40: en dash
```

A generic hit prints the class, the location, and then either the matched text or a
masked shape of it. The first two lines are one hit in its two contexts. For the
classes whose text is itself an address, a host, a login or a path segment (every
generic class except `style-dash` and `overlay-word`) the text prints only where the
private tier is loaded and the output is a terminal: the hooks, a local `tree` run.
With no tier loaded the scanner cannot tell that the value is not a name the tier
would have withheld, and a CI log (`--format github`) outlives the force-push that
scrubs a branch, so the `lint` step of the main workflow, a fork pull request and every
`Leak guard` job print the value with each letter and digit replaced (the second
example): the length and the punctuation stay, which is enough to find the token on
the line, and the value stays out of the log. When a private pattern touches the same
line in any view of it, the generic hit prints no matched text at all (the third
example): the decision is made on the whole line, never on the extracted match, so a
name that sits half inside the match, is split by an escape the generic class ignored,
or is shadowed by a shorter entry cannot ride out on the generic line. The LOCATION
follows the same rule: wherever a value prints as a shape, a value that is the file or
directory name itself prints as `<class:shape>` in its place (the fourth example), on
the name hit, on every content hit inside that file, and on the `media` line that
names a container, so the location beside a masked value cannot carry the value. A CI
annotation whose location had to be masked carries it in the message rather than in
the file property, which the forge would anchor to a file that does not exist. A
private hit prints the class as `private#<index>` plus its tag, and the location, and
nothing else: no matched text, no pattern, no surrounding line. The index is the line
number of the entry in the private list, counting only entries. Open the file at that
line and look for the name the tag describes. A private match inside a PATH is
redacted in every printed line (a match found only in a normalised view of a path
component withholds the whole component), which is why the sixth example reads the way
it does; `[name]` says the hit is in the file name itself. Every printed line is
stripped of line breaks, escape sequences and format characters, so no path or match
can start a workflow command in a CI log. `REPORT` lines count and do not block;
`--hard <class>` escalates one for a run.

Exit codes keep "found something" apart from "could not run": `0` clean, `1` at least
one HARD hit, `2` a malformed invocation, `3` the scan could not run (a bad ref, zero
units, a private pattern that does not compile, a dead built-in control, or
`--require-private` with nothing loaded). The last line is always a summary; the
`private=` field says whether the private tier ran, and when it did not a loud line
above it says what was not checked.

## False positives

Two mechanisms, both reviewed like code. A line pragma inside that language's comment:
`leak-scan: allow <class> <reason of at least 12 characters>` on the hit line; it never
accepts a private class and is ignored in `messages` mode. A path entry in
`tools/scripts/leak_scan_allow.txt`: `glob | class | reason`, where the class is a
generic class or a private CATEGORY such as `private@person` (a bare private waiver is
refused, so a waiver for a person's name can never excuse a machine name in the same
file). Prefer the pragma for a fixture line: a path entry waives the class over the
whole file, so a real name added to that file later would be hidden by it. Path
entries are for what a pragma cannot express, such as the authors' names in the
citation and legal files. In a full-tree run an entry that matches no file or excused
nothing fails the run, so waivers cannot go stale. One exception, so that what the
secret holds cannot decide whether a push to `main` is red: a `private@<tag>` entry is
judged on use only when a loaded private entry carries that tag; when none does, the
run prints `ALLOWLIST NOTE: ... unused: no <tag> entry loaded` and goes on (its path
check still runs, since that depends on the tree alone). Every summary line prints
`allowlist=N` and `pragmas=N`.

## Run it yourself

```bash
python3 -B tools/scripts/leak_scan.py --self-test        # every class and surface, planted
python3 -B tools/scripts/leak_scan.py tree               # the whole worktree
python3 -B tools/scripts/leak_scan.py tree --require-private   # refuse to run without the private tier
python3 -B tools/scripts/leak_scan.py tree --ref HEAD --files-from changed.zlist   # a NUL separated list, taken verbatim; a listed path the ref lacks is a NO RUN
python3 -B tools/scripts/leak_scan.py messages --range origin/main..HEAD
python3 -B tools/scripts/leak_scan.py names --branch "$(git rev-parse --abbrev-ref HEAD)"
python3 -B tools/scripts/leak_scan.py media
```

What the guard does not see: a name on no list is invisible until it is
listed; pixels are not read (look at every frame of a changed image or clip, both
sides); compressed payloads inside bags and video streams are not opened; forge
surfaces that no check sees before they are public (review comments, CI logs of other
jobs) stay a habit.
