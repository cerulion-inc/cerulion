# SPDX-License-Identifier: AGPL-3.0-only
"""Round two plug-ins (SPEC-R2.md): one improvement per package, joined through hooks, never by editing each other.

    from rcpx import r2
    mods = r2.load()                         # every package that imports; RCPX_R2 = all | none | a,b,c
    ex = r2.make_executor(model, backend, journal=journal, world=world, mods=mods)
    surface, resources, handlers, box = r2.assemble(ex, mods, base_surface, base_resources, base_handlers)
    session = Session(ex, surface, resources, handlers=handlers, ...); box.session = session
"""

from rcpx.r2.core import (PACKAGES, HookMixin, assemble, executor_class, load, make_executor,  # noqa: F401
                          round_one_surface)
