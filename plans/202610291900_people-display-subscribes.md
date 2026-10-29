# The people display subscribes to ALPeoplePerception

**Status:** Done

Implements the settled behavior in [people.md](../specs/people.md) ("The subscription"): `PeopleSource` subscribes to `ALPeoplePerception` at 200 ms while it runs, again after a reconnect, and unsubscribes at close, so a NAOqi whose people perception runs only while subscribed (a NAO's, naoqi-sim's since its Oct 29, 2026 replacement) computes the people the display shows. Leaves out a toggle key and any other module.

## Scope

- `src/naoqi_viewer/people.py` — `SUBSCRIBER_NAME`, `SUBSCRIBE_PERIOD_MS`, `UNSUBSCRIBE_TIMEOUT`; `PeopleSource._subscribe` at each connection (a missing module or refused call logged once, the keys still read) and `_unsubscribe` at close, bounded.
- `tests/mock_naoqi.py` — an `ALPeoplePerception` service recording its subscriptions (`people_subscribers()`), lost at `restart()`, left out with `people_perception=False`.
- `tests/test_people.py` — the subscription while running and gone at close, again after a restart, the keys read on a target without the module.
- `specs/people.md`, `specs/_overview.md`, `specs/_index.md` — the subscription.

## Steps

1. The mock's service, then the source's subscription and unsubscription.
2. The fast tests; the spec.

## Verification

`uv run ruff check .`, `uv run ruff format .`, `uv run pyright`, `uv run pytest` (the fast tier). The live tier was not run with this change: the only NAOqi on 9559 was a user's naoqi-sim run, left alone.
