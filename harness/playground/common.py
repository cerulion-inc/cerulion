# SPDX-License-Identifier: AGPL-3.0-only
"""Small helpers every module of the sim server shares."""
import json
import os
import re
import time

ENDED = ("ended", "failed")
EID_RE = re.compile(r"^[A-Za-z0-9_-]{6,40}$")   # an episode id: what a viewer link and a kept folder are named
OWNER_FILE = ".playground_owner"                  # in each work folder: the pid of the server that made it
LICENSE = "AGPL-3.0-only"
DAY_S = 86400.0
# What a client sends when it connects: the session handshake, or, with no session, the discovery and the first
# tool listing. One of these, for a start state whose episode has ended, starts a new episode.
NEW_CONNECTION = ("initialize", "server/discover", "tools/list")


def utc(epoch=None):
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(epoch))


def read_json(path):
    """The JSON document in path, or {} when the file is missing or is not JSON."""
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return {}
    return doc if isinstance(doc, dict) else {}


def span(seconds):
    """A keep time in words: whole days, else whole hours, else seconds."""
    for unit, name in ((DAY_S, "day"), (3600.0, "hour")):
        if seconds >= unit and seconds % unit == 0:
            n = int(seconds // unit)
            return f"{n} {name}{'' if n == 1 else 's'}"
    return f"{seconds:g} seconds"


def cputime(text):
    """ps's cumulative processor time ([dd-]hh:mm:ss or mm:ss.cc) as seconds."""
    days, _, rest = text.rpartition("-")
    total = 0.0
    for part in rest.split(":"):
        total = total * 60.0 + float(part)
    return total + (int(days) * 86400 if days else 0)


def pid_alive(pid):
    try:
        os.kill(int(pid), 0)
    except PermissionError:
        return True          # it runs, under another user
    except (OSError, ValueError):
        return False
    return True


def source_offer(settings):
    """The offer of source to every user of the running server (GNU AGPL version 3, section 13)."""
    return (f"Source code: this server runs {settings.name}'s robot tools under the GNU Affero General Public "
            f"License, version 3 ({LICENSE}). The source is offered to every user: {settings.source_url}")


def source_link(settings):
    """source_url when it is a link, else None."""
    return settings.source_url if settings.source_url.startswith(("https://", "http://")) else None
