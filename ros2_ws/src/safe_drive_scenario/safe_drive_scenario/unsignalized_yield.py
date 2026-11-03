"""Yield decision from a configured pedestrian scenario state."""


def pedestrian_requires_yield(state: str | None) -> bool:
    """Keep the approach stopped until a known clear scenario state."""
    return state not in ('ARMED', 'COMPLETED')
