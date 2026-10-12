# Fast tier shared fixtures.
#
# If the package has process-global or singleton state (a module-level registry,
# a cached client, a configured logger), add an `autouse=True` fixture here that
# resets it before and after each test so state can't leak between tests. Keep
# this tier deterministic and network-free — anything that hits a real service
# belongs in tests-e2e/ instead.

from collections.abc import Iterator

import pytest
from mock_naoqi import MockNaoqi

from naoqi_viewer import meshes


@pytest.fixture(autouse=True, scope="session")
def no_installed_meshes(tmp_path_factory: pytest.TempPathFactory) -> Iterator[None]:
    """Keep the developer's own Aldebaran meshes out: "auto" means placeholder here, and
    no test writes to the real data directory. Mesh tests install a release of their own.

    Session-wide, so it is in place before the module-scoped worlds are loaded."""
    empty = tmp_path_factory.mktemp("no-meshes")
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(meshes, "data_dir", lambda: empty)
        yield


@pytest.fixture
def mock_naoqi() -> Iterator[tuple[str, MockNaoqi]]:
    """A virtual-robot mock NAOqi on a loopback port: yields (url, mock)."""
    mock = MockNaoqi().start()
    try:
        yield mock.url, mock
    finally:
        mock.close()
