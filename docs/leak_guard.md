# The leak guard

Everything this repository publishes is public: code, comments, docs, logs, data files,
file and branch names, commit messages, commit identities, pull request titles and
bodies, and the images and clips under `docs/media`. The leak guard is one scanner,
`tools/scripts/leak_scan.py` (stdlib Python 3, no dependencies), that keeps machine
names, addresses, home paths, logins, people and location metadata out of all of it.
It runs in four places: the git hooks on your machine, the `lint` job of the main CI
workflow, the tree, names, messages and media scans in the `Leak guard` workflow
(`.github/workflows/leak-guard.yml`) on every pull request, merge queue batch and push
to `main`, and the issue and comment body scan in the `Leak guard (issue and comment
bodies)` job of `.github/workflows/leak-guard-conversation.yml` as a body is written or
edited. The messages job scans in two runs that both always happen and whose results
combine: the commit range and the pull request title under the full hard set, and the
pull request body under the conversation hard set.

A body is public the moment it is written and no check runs before it is, so that last
job cannot block anything. It reads EVERY author, bots included. A bot restates a diff and
quotes what it found, so an address, a home path, a private name or a closed reference
reaches a public thread through a bot exactly as it does through a person, and both of the
genuine findings this guard has made were in text a review app wrote. What made bot text
unbearable was style classes counting as hard on a conversation body, which the narrower
hard set fixes for every author, so an author rule has nothing left to buy. The one body
never read is the guard's OWN ask, matched on its account and its marker together, so it
cannot read itself while a marker pasted by anyone else is scanned like any other text.

A conversation body is also judged by a narrower hard set than a file or a commit message:
only the identity classes and the two reference classes, the ones whose finding is a value
a reader should not have been shown. A style class reports there and no more. House style
is a rule about text this project writes, and a label, an ask and a red run on someone
else's thread over a dash is the guard crying wolf on a page that leaks nothing.

On a HARD finding, which on that surface means an identity class or a reference class, the
job applies the `leak` label, asks the author once to edit the text (one ask per body, so
an unchanged body is never asked twice), and goes red. A style class prints a `REPORT` line
and does none of the three. The ask names what it found: a value asks to have that value
taken out, a reference asks for a link anyone can follow in its place.

The label tracks the THREAD: a clean body takes back the ask left for it, and the label
comes off when the last outstanding ask on that thread is gone, so a clean comment cannot
clear a label another body still deserves.

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
overlay access control tag; the overlay product words in CI files; a reference nobody
outside this project can open (below); and a style class, en and em
dashes, whose severity the SURFACE decides: hard in a guard file, a commit message and a
pull request title, a report on a CONVERSATION surface. The conversation surfaces are an
issue body, an issue comment, a pull request review comment, and a pull request BODY. A
body is there because many hands edit it: a review app appends a summary, the author
pushes, and what arrives is no longer only the author's writing, so a dash somebody else
left must not red a required check. A title is not there, because it is the author's own
and nobody else rewrites it. Placeholders such as `/Users/someone/`, `/home/ubuntu/` and
`robot-a.local` are published vocabularies, never length rules.

**Private patterns** never ship. They hold the real names: machine hostnames and short
names, overlay device names, real LAN addresses, logins, people, the private
repository slug, the internal tracker prefix, the tracker host. They load from, in order, the
`LEAK_PATTERNS` environment variable (what CI passes from a secret), the file named by
`LEAK_PATTERNS_FILE`, or `~/.config/cerulion/leak-patterns.txt`. The first source that
yields a pattern wins. Private classes are HARD in every mode and honour no pragma.

### References a stranger cannot open

A reference is a leak surface of its own. The shape carries no address, no host and no
login, so no other class can see it, and the harm runs both ways: a reader outside this
project cannot open what the text points at, and when the target is closed its NAME is
the secret the text just published. An internal ticket quoted in a public issue as a
`#` shorthand is how this class came to exist.

Nothing in the tree lists a repository. The scanner RESOLVES what the text points at and
asks the forge's repository endpoint (`api.github.com/repos/<owner>/<repo>`, a `HEAD`, up
to three of them) whether the credential the question carries is served that repository.
The question carries `GITHUB_TOKEN` when the environment holds one, which raises the
budget the forge prices it under from 60 an hour to 5,000; the token is sent on that
request and is never printed, written or passed as an argument. The page endpoint answers
the same 200 and 404 but publishes no budget header, so a throttle there cannot be told
from a server fault.

WHO ASKS DECIDES THE ANSWER. The Actions job token is scoped to the repository the
workflow runs in, so for every other repository it is a stranger and a 404 there is the
not-found verdict. A personal token is not a stranger: a local online run carrying one
reads a repository its owner is a member of as served, so it can read clean exactly where
the job reads `ref-unopenable`. A disagreement between a local run and the job is those
two identities, and the job's reading is the one a stranger gets.

One forge is resolved, the one this repository lives on (`github.com`). A link to any other
forge is not a candidate and is not checked; a reference there is a reviewer's job.

| shape | example | resolved as |
| --- | --- | --- |
| a forge link | `https://github.com/<owner>/<repo>/...` | that owner and repository |
| a qualified shorthand | `<owner>/<repo>#<n>` | that owner and repository |
| a bare shorthand | `<repo>#<n>` | `<repo>` under this repository's own owner |
| a tracker link | `https://<tracker host>/...`, shipped or private-tier | nothing: a tracker is closed to a stranger already |

`200` means public and is clean. `404` and `410` mean private, renamed away, taken down or
never there, and a stranger is given nothing in every one of those cases, so all of them
are one finding, `ref-unopenable`.

AN ANSWER IS A VERDICT; NO ANSWER IS A TOOL FAILURE. A throttle (`429`, or a `403`
carrying a rate limit header), a server error, a transport error and a timeout say nothing
about the reference, so each one is retried: three attempts per reference, the two waits
between them 1.5 s then 3.0 s, or the wait a short `Retry-After` asked for, all inside the
run's own time budget. A reference still unanswered after that is recorded as NOT QUERIED,
and so is one the run never got to inside its bounds. Such a reference carries no row at
all.

The hits and the verdict summary print first, whatever the forge answered. Then comes one
`NOT QUERIED ref-unqueried:` line per unanswered reference, masked the way a finding's
value is masked and carrying the status or the error kind the last attempt saw, so a stale
credential reads as `(status 401)` and a throttle as `(status 429)`. The run then says
`unqueried=N reference(s) got no answer`: a run that otherwise reads OK exits `3`
(NON-RUN) and asserts no leak at all, and a run that already reads `FAIL` on its own
evidence, a HARD hit or a waiver that matched or excused nothing, keeps that `FAIL`
summary and exit `1` and asserts nothing about these references alone. The same unchanged
content used
to read 0, 2, 2 and 3 `ref-unverified` findings across four runs, which is the defect this
split closes.

`--require-private` and `--hard ref-unverified` raise nothing for a reference nobody
answered about, which has no row for them to raise. They do raise the one
`ref-unverified` row an online run still carries: a bare `<repo>#<n>` with no owner known
to resolve it against, which is a reference the text did not finish writing. And they
raise the whole class under `--offline`, which is the operator's own choice not to ask and
keeps reporting every candidate as a scoped `ref-unverified` note. The hooks always run
offline and name neither flag, so there the class reports and no more.

The asking is bounded twice over: at most 300 distinct repositories per run, and at most
two minutes of asking in total from the first question. Past either bound no further
repository is asked about at all, and each one is NOT QUERIED on the same terms, so a scan
behind a dead network still finishes and still refuses to call what it did not check
clean.

One question per distinct repository per run, cached in memory (the no-answer verdict
included, so a throttled repository costs its bounded attempts once), to the forge the
link names and to no other host. That question carries the owner and repository the text
already wrote, and it goes to the forge that owns the name; no text reaches anywhere
else, and a tracker link is judged with no request at all. A reference to THIS repository
is never a finding, and neither is a bare `#<n>`, which the forge already reads as this
repository.

Two shapes are deliberately out of reach. The first path segment of a forge URL is
treated as an owner only when it is not one of the forge's own reserved words, and a bare
`<repo>#<n>` counts only when the token is SLUG shaped, that is when it carries a `-`,
`_` or `.` between alphanumerics: `issue#5` and `demand#1` are prose, and a guard that
reds on prose is a guard nobody keeps. A repository whose name is a single word with no
separator is therefore invisible to the shape, which is exactly what the private tier's
`repo-slug:` key is for.

What the slug shape cannot tell apart is a closed repository from a name that was never a
repository: the forge answers 404 to both. A slug shaped token that is not a reference at
all, an `api_v2` written straight against a `#8`, therefore reads as one. Across the
tracked tree that shape occurs zero times, and the fix where it does occur is one
character: a space before the number sign (`api_v2 #8`) is prose, and the ask the
conversation job posts says so. In a file, the line pragma says it once and for good.

A finding prints the reference stripped to what a reader needs to find it on the line, an
owner and a repository or a tracker host, never the full link, and both classes are
identity bearing: the value prints only on a terminal with the private tier loaded, and
everywhere else a masked shape takes its place, in the LOCATION as well as the value, so a
slug that is itself a file or directory name cannot ride out in the place that says where.

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
`--allow-email-file`), pull request title and body, and any further body handed to it by
name (`--body-env VAR --body-label issue-body`), which is how the workflow reaches an
issue body, an issue comment and a pull request review comment. `names` mode scans paths and
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

**Two keys, for what no shape can see.** A key is an entry like any other: it counts in
the index, it is HARD in every mode, and its value never prints. It carries its own
category and takes no tag.

```
tracker-host:tickets.example.invalid    # a tracker whose links a stranger cannot open
repo-slug:exampleoldname                # a repository name that must not appear at all
```

`tracker-host:` names a tracker host beside the hosts the scanner ships with: a link to
it becomes a `ref-unopenable` finding, and the host itself is refused anywhere in the
tree. `repo-slug:` names a repository whose NAME is the secret, in the spelling the forge
uses (`<repo>`, or `<owner>/<repo>`): the reference classes see a slug-shaped reference
generically, but a repository named by a single word with no separator reads as prose,
and this is the only place that can be refused. Both values are matched the way `word:`
entries are, so a separator at any letter or digit seam still matches, and both are
reported as `private#<index>@host` and `private#<index>@slug`. Neither value ships in the
tree.

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
HIT ref-unopenable docs/notes.md:4: (value masked, 17 chars: xxxxxxxx-xxxx#xxx)
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
units, a private pattern that does not compile, a dead built-in control,
`--require-private` with nothing loaded, or a reference nobody answered about on a run
that otherwise reads OK). The last line is always a summary; the
`private=` field says whether the private tier ran, and when it did not a loud line
above it says what was not checked.

## False positives

Two mechanisms, both reviewed like code. A line pragma inside that language's comment:
`leak-scan: allow <class> <reason of at least 12 characters>` on the hit line; it never
accepts a private class and is ignored in `messages` mode. `ref-unopenable` and
`ref-unverified` are one name for this purpose: they are two verdicts on the same
reference, and a pragma judges the reference, so a line excused as one is excused as the
other, which is what keeps a pragma written against an online `404` from going hard the
moment the same line is scanned with `--offline`. A path entry in
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
python3 -B tools/scripts/leak_scan.py tree --offline           # resolve no reference; every candidate reads unverified, and no run is a NON-RUN for it
python3 -B tools/scripts/leak_scan.py tree --self-repo OWNER/REPO   # name this repository yourself
python3 -B tools/scripts/leak_scan.py tree --ref HEAD --files-from changed.zlist   # a NUL separated list, taken verbatim; a listed path the ref lacks is a NO RUN
python3 -B tools/scripts/leak_scan.py messages --range origin/main..HEAD
python3 -B tools/scripts/leak_scan.py names --branch "$(git rev-parse --abbrev-ref HEAD)"
python3 -B tools/scripts/leak_scan.py media
```

What the guard does not see: a name on no list is invisible until it is
listed; pixels are not read (look at every frame of a changed image or clip, both
sides); compressed payloads inside bags and video streams are not opened; a repository
named by a single word with no separator, written bare before a `#`, is invisible to the
reference shape until a `repo-slug:` entry names it; a reference on a host the classes do
not know is not resolved at all; and CI logs of other jobs stay a habit.
