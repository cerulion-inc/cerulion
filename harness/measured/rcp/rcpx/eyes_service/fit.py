# SPDX-License-Identifier: AGPL-3.0-only
"""What the vision service holds in memory, read when asked (for /describe). Standard library only; a backend's
framework is imported only when the service already runs on a graphics processor.

  resident_mb()               this process's resident memory in MB
  gpu_mb(device, command)     (MB, how it was read): this process's memory on the graphics processor, None on the CPU
  weights_mb(model)           the size of a loaded model's own numbers in MB (0 for no model)

How a computer reports graphics memory per process differs by computer, so the installation's equipment file says
it: "memory_command" is a command (a list of words, run without a shell) that prints one "pid, MB" line per
process. Where the file gives none, or the command has no line for this process (a computer with shared memory
has no figure per process), the figure is what the framework has reserved, and the answer says so.
"""
import os
import subprocess
import sys

MB = 1024 * 1024


def resident_mb():
    try:
        with open("/proc/self/status", encoding="ascii") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return max(1, int(line.split()[1]) // 1024)
    except (OSError, ValueError, IndexError):
        pass
    try:   # no /proc: the peak resident size (bytes on one family of systems, kB on the other)
        import resource
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return max(1, int(peak // (MB if sys.platform == "darwin" else 1024)))
    except Exception:
        return 1


def _command_mb(command, pid):
    """The figure the installation's own command prints for this process, or None (no command, no line, a failure)."""
    if not isinstance(command, list) or not command or not all(isinstance(w, str) for w in command):
        return None
    try:
        out = subprocess.run(command, capture_output=True, text=True, timeout=3, check=False).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        parts = [x.strip() for x in line.split(",")]
        if len(parts) == 2 and parts[0] == str(pid) and parts[1].isdigit():
            return int(parts[1])
    return None


def gpu_mb(device, command=None):
    if str(device) == "cpu":
        return None, "no graphics processor is used"
    mb = _command_mb(command, os.getpid())
    if mb is not None:
        return mb, "the computer's own figure for this process (the equipment file's memory_command)"
    try:
        import torch
        return (int(torch.cuda.memory_reserved() // MB),
                "reserved by the framework (no figure per process from the computer: less than the process holds)")
    except Exception:
        return None, "could not be read"


def weights_mb(model):
    try:
        return int(round(sum(p.numel() * p.element_size() for p in model.parameters()) / MB))
    except Exception:
        return 0
