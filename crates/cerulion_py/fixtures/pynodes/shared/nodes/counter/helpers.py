"""Helper module private to the counter fixture: a second node type that ships
its own `helpers.py` must not see this one through the shared `sys.modules`."""


def transform(value):
    return value * 2 + 1
