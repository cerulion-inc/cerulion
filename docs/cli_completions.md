# Shell Tab-Completion

> **The command is the authoritative surface.** `cerulion completions <shell>`
> emits the completion script, and `cerulion completions --help` prints the
> installation commands for the version you have installed. The two are pinned
> against each other by a test. Where this page disagrees with `--help`,
> `--help` describes your version.

`cerulion` completes **everything**: every subcommand and flag from the command
tree, and the live names you actually type: topics, node types, graph names,
schemas, robots.

```
$ cerulion topic hz <TAB>
/camera/image   /imu/data   /utlidar/cloud

$ cerulion graph run per<TAB>
perception

$ cerulion node build <TAB>
imu_fusion   lidar_filter

$ cerulion schema info geometry_msgs/Tw<TAB>
geometry_msgs/Twist                       geometry_msgs/TwistStamped
geometry_msgs/TwistWithCovariance         geometry_msgs/TwistWithCovarianceStamped

$ cerulion viz --robot <TAB>
go2 -- paired      orin_nano -- tcp/192.168.1.9 7683
```

## Install

One line, once, per shell. Add it to your shell's startup file:

| Shell | Line to add | Where |
|---|---|---|
| **zsh** | `autoload -Uz compinit && (( $+functions[compdef] )) \|\| compinit`<br>`source <(COMPLETE=zsh cerulion)` | `~/.zshrc` |
| **bash** | `source <(COMPLETE=bash cerulion)` | `~/.bashrc` |
| **fish** | `COMPLETE=fish cerulion \| source` | `~/.config/fish/completions/cerulion.fish` |
| **elvish** | `eval (E:COMPLETE=elvish cerulion \| slurp)` | `~/.elvish/rc.elv` |
| **powershell** | `$env:COMPLETE = "powershell"; cerulion \| Out-String \| Invoke-Expression; Remove-Item Env:\COMPLETE` | `$PROFILE` |

**zsh needs both lines.** The completion script ends in `compdef`, which is a
function `compinit` defines, not a builtin. macOS's system `/etc/zshrc` never
calls `compinit`, so on a stock `~/.zshrc` the second line alone prints
`command not found: compdef` at every shell start and nothing ever completes.
The guard is a no-op when a framework (oh-my-zsh, prezto) has already run
`compinit`, so it is safe to add unconditionally.

**fish** puts its line in a file under `completions/`, not `config.fish`: fish
autoloads that file the first time you complete `cerulion`, so it regenerates
itself on upgrade and costs nothing at shell start.

Or let the CLI print it for you:

```bash
cerulion completions zsh          # prints the shell code + the install line
```

`cerulion completions <shell>` writes the same code to stdout, so you can also
install it as a file:

```bash
cerulion completions zsh > ~/.zsh/completions/_cerulion
```

**Prefer the one-liner.** It regenerates the shell code on every shell start,
so it self-corrects when `cerulion` is upgraded or moved. A file written by
`cerulion completions ... > FILE` hard-codes the path of the binary that wrote
it: re-run the command after an upgrade or a move.

## What completes

| Where you press TAB | What you get | Source |
|---|---|---|
| any position | subcommands, flags, `--time-source real\|virtual\|external`, … | the clap command tree |
| `topic echo\|info\|hz <TAB>`, `bag record <TAB>` | topic names | the local iceoryx2 service directory |
| `viz <TAB>` | topic names | same |
| `node delete\|modify\|build\|stage\|run\|info <TAB>` | node types | `nodes/*` in your workspace |
| `graph run\|validate\|levels\|chains\|profile\|partition <TAB>`, `node stage -g <TAB>` | graph names | `graphs/*.yaml` stems |
| `schema delete\|info <TAB>` | workspace schemas, the `.msg` store, and all 254 built-in ROS 2 types | `schemas/`, `native_ros2_messages` |
| `viz --robot <TAB>`, `connect <TAB>`, `pair <TAB>` | robot names | `~/.cerulion/robots.toml` (paired) + `~/.cerulion/peers.json` (seen on the LAN) |
| `bag play\|info\|migrate <TAB>` | `.mcap` bags and directories | the filesystem |
| `bag play --resim <TAB>` | `all`, the only selection that resolves | a constant |
| any other path argument | files or directories, as appropriate | the filesystem |

The three `create` verbs (`node create`, `graph create`, `schema create`)
complete nothing on purpose. A create argument names something that does not
exist yet, and the existing names are exactly what those verbs reject.

Outside a Cerulion workspace, the node/graph/schema-file completers have
nothing to offer and stay silent; built-in ROS 2 schemas still complete
anywhere, which is why `cerulion schema info sensor_msgs/Image` works from any
directory.

## Guarantees

**A TAB press never hangs, and never touches the network.**

Every source is an instant local read: a directory listing, a file in
`~/.cerulion`, a compile-time static, or the local shared-memory service
directory. A completion:

- **never starts a process.** `cerulion-netd` in particular is never spawned:
  its client blocks for up to ten seconds polling a freshly forked daemon for
  readiness, which would be a long stare at a keystroke.
- **never opens the network.** No zenoh session, no mDNS browse, no TCP probe,
  no DNS lookup. The discovery ladder that `cerulion topic list` runs is
  off-limits here.
- **never prints anything but candidates.** Errors are silent; the shell simply
  falls back to its own default.
- **is bounded at 150 ms.** Anything slower is abandoned and yields nothing.

That budget has a cost worth knowing:

**Remote topics complete only while a mirror of them is held.** Learning a
remote topic's name needs a network round-trip, so completion can only see the
ones `cerulion-netd` is currently mirroring into local shared memory. Mirrors
are held per PROCESS and retired when the last holder goes away:

| What you run | Mirror held | Remote topics complete? |
|---|---|---|
| `cerulion viz --robot NAME` | by the long-lived `cerulion-vizd` daemon | **yes**, and it stays that way |
| `cerulion topic echo/info/hz /some/topic` | only while that command runs | only during that command |
| `cerulion topic list` | none: it is a service-directory scan plus a liveliness gather, and demands nothing | no |
| nothing | none | **no** |

So on a desk with no `viz --robot` attach, a remote robot's topics do not
complete. That is a real limit, not a warm-up step: `topic list` will show you
the names, but it does not make them completable.

**Topic candidates carry no origin label.** A mirrored remote topic and a
genuine local producer are indistinguishable in the local service directory,
and reading the registry that distinguishes them does not fit the 150 ms
budget. Rather than print a possibly-wrong `local` next to a mirror, completion
prints no label at all. `cerulion topic list` shows the attribution.

## Troubleshooting

**`command not found: compdef` when the shell starts (zsh).** You added the
`source` line without the `compinit` guard. `compdef` is a function the zsh
completion system defines, and macOS's system zshrc never initialises it, so
the completion script has nothing to register with, and nothing completes. Add
the first zsh line from the Install table above, before the `source` line.

**Nothing completes at all.** Confirm the shell code is loaded. `echo
$COMPLETE` inside the completion is not observable, so instead check that
`cerulion` is on your `PATH` and re-run the install line in a fresh shell. If
you installed by writing a file, the binary may have moved since; regenerate
it, or switch to the one-liner.

**Subcommands complete but topic names do not.** For LOCAL topics, that is the
expected shape when no Cerulion graph is running: there are no topics to offer.
For a REMOTE robot's topics, see "Remote topics complete only while a mirror of
them is held" above: you most likely need a running `cerulion viz --robot
NAME`.

**Node or graph names do not complete.** You are probably outside the
workspace. Completion resolves the workspace by walking up from the current
directory exactly as the real commands do: `cd` into it.

**A robot is missing from `--robot`.** Robot names come from robots you have
paired (`cerulion pair`) or that were seen on the LAN within the last 7 days.
Run `cerulion topic list` to refresh the peer cache.
