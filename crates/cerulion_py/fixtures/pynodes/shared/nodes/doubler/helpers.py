"""Helper module private to the doubler fixture, named exactly as the counter's
so a host that cached one node's `helpers` would run the wrong code here."""


def transform(value):
    return value * 2
