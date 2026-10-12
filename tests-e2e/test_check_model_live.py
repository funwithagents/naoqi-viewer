"""check-model against a live NAOqi (needs NAOQI_URL): the model's acceptance test."""

import pytest
from support import require_env

from naoqi_viewer import check_model


def test_the_model_matches_naoqis_kinematics():
    url = require_env("NAOQI_URL")
    try:
        report = check_model.run(url, samples=20)
    except check_model.TargetRefused:
        pytest.skip(
            "NAOQI_URL is a real robot; check-model only moves simulated ones here"
        )
    assert report.passed, report.format()
