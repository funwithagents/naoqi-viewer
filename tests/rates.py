"""Checking a source's `rate()` against its target on any machine, a slow CI runner included."""

import time
from collections.abc import Callable


def assert_rate_settles(
    rate: Callable[[], float], low: float, high: float, timeout: float = 5.0
) -> None:
    """`rate()` reaches `low` within `timeout`, and no reading exceeds `high`, during that wait
    or for one second after.

    `rate()` counts what arrived over the last second, and a polling loop skips the slots a
    late tick missed rather than catching up. So a machine whose timers run late (CI's macOS
    runner) can have a second below the target, and a single reading after a fixed sleep fails
    for the machine, not the code. Polling until the floor is reached lets one slow second
    pass. The ceiling stays strict: no slowness can make a fixed schedule poll faster.
    """
    deadline = time.monotonic() + timeout
    highest = 0.0
    while (current := rate()) < low:
        highest = max(highest, current)
        assert time.monotonic() < deadline, (
            f"rate() stayed below {low} for {timeout} s (last reading {current})"
        )
        time.sleep(0.05)
    ceiling_until = time.monotonic() + 1.0
    while time.monotonic() < ceiling_until:
        highest = max(highest, rate())
        time.sleep(0.05)
    assert highest <= high, f"rate() reached {highest}, above {high}"
