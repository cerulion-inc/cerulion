"""``python -m cerulion.msgs [--out DIR]``: write the bound workspace's type
stubs (default ``<workspace>/.cerulion/python``) and print each file written.
"""

import argparse
import os
import sys

from cerulion.msgs import _workspace_root
from cerulion.msgs._stubs import write_workspace_stubs


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python -m cerulion.msgs",
        description="Write the type stubs of the workspace's cerulion.msgs classes.",
    )
    parser.add_argument("--out", help="output directory (default <workspace>/.cerulion/python)")
    args = parser.parse_args(argv)
    root = _workspace_root()
    if root is None and args.out is None:
        print(
            "no Cerulion workspace: set CERULION_WORKSPACE or run inside one, "
            "or pass --out for the built-in messages alone",
            file=sys.stderr,
        )
        return 2
    out = args.out or os.path.join(root, ".cerulion", "python")
    for path in write_workspace_stubs(out):
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
