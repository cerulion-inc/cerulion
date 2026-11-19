# SPDX-License-Identifier: AGPL-3.0-only
"""urllib follows no redirect in a process of a run, so a request that carries the model key goes to the URL it
names and to no other. Standard library only.

A redirect answer (301, 302, 303, 307 or 308) is raised as urllib.error.HTTPError with its code and its Location
header, and nothing is sent to the address it names. refuse_everywhere() does that for every urllib opener of the
process that uses urllib's own redirect handler, built before the call or after it (urllib.request.urlopen and
build_opener alike). run-cell calls it for itself, and episode_site/sitecustomize.py calls it at the start of
each Python process of an episode that starts with run-cell's PYTHONPATH, the episode's model client among
them."""
import os
import urllib.request

SITE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "episode_site")
REDIRECT_CODES = (301, 302, 303, 307, 308)


class Refuse(urllib.request.HTTPRedirectHandler):
    """A redirect handler that follows nothing: urllib raises the redirect answer as an HTTPError instead."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def refuse_everywhere():
    """From now on no urllib redirect handler of this process follows a redirect."""
    urllib.request.HTTPRedirectHandler.redirect_request = Refuse.redirect_request
