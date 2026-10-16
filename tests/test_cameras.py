import logging
import time
from collections.abc import Callable
from contextlib import closing

import numpy as np
import pytest
from mock_naoqi import MockNaoqi

from naoqi_viewer import cameras
from naoqi_viewer.cameras import (
    DISPLAY_COLORSPACE,
    DISPLAY_FPS,
    DISPLAY_RESOLUTION,
    PICTURE_BORDER,
    SUBSCRIBER_NAME,
    CameraImage,
    CameraSource,
    decode_images,
    framed,
    picture_rects,
    placeholder,
    resample_nearest,
)
from naoqi_viewer.source import connect

QVGA = (320, 240)


def wait_until(condition: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


def gradient(blue: int, size: tuple[int, int] = QVGA) -> np.ndarray:
    """An RGB image whose pixel (row, col) is (row, col, blue) mod 256: every pixel says
    where it is, so a flip or a transposition shows."""
    width, height = size
    rows, cols = np.mgrid[0:height, 0:width]
    return np.stack([rows % 256, cols % 256, np.full_like(rows, blue)], axis=-1).astype(
        np.uint8
    )


def latest(src: CameraSource, camera: str) -> CameraImage:
    image = src.latest(camera)  # type: ignore[arg-type]
    assert image is not None
    return image


def both_arrived(src: CameraSource) -> bool:
    return src.latest("top") is not None and src.latest("bottom") is not None


# --- CameraSource on the mock -----------------------------------------------------------


def test_subscribes_both_cameras_and_decodes_their_images(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    mock.set_image("bottom", gradient(2))
    with closing(CameraSource(url)) as src:
        wait_until(lambda: both_arrived(src))
        top, bottom = latest(src, "top"), latest(src, "bottom")
        assert top.image.shape == (240, 320, 3) and top.image.dtype == np.uint8
        assert top.image[5, 9].tolist() == [
            5,
            9,
            1,
        ]  # row 0 is the top row, not flipped
        assert bottom.image[200, 300].tolist() == [200, 300 % 256, 2]
        assert top.camera == "top" and bottom.camera == "bottom"
        assert top.seq == bottom.seq >= 1  # one poll, both images
        assert top.naoqi_time is not None and abs(top.naoqi_time - time.time()) < 5
        assert time.monotonic() - top.received_at < 5

        assert mock.calls_to("subscribeCameras") == [
            (
                SUBSCRIBER_NAME,
                [0, 1],
                [DISPLAY_RESOLUTION, DISPLAY_RESOLUTION],
                [DISPLAY_COLORSPACE, DISPLAY_COLORSPACE],
                DISPLAY_FPS,
            )
        ]
        assert mock.subscribers() == [f"{SUBSCRIBER_NAME}_0"]
        assert mock.calls_to("unsubscribeAllInstances") == [(SUBSCRIBER_NAME,)]
    assert mock.subscribers() == []  # unsubscribed on close


def test_images_follow_the_robot(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url)) as src:
        wait_until(lambda: src.latest("top") is not None)
        first = latest(src, "top")
        mock.set_image("top", gradient(9))
        wait_until(lambda: latest(src, "top").image[0, 0, 2] == 9)
        assert latest(src, "top").seq > first.seq


@pytest.mark.parametrize(("fps", "low", "high"), [(5, 4, 6), (15, 13, 16)])
def test_rate_follows_fps(
    mock_naoqi: tuple[str, MockNaoqi], fps: int, low: float, high: float
):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url, fps=fps)) as src:
        wait_until(lambda: src.latest("top") is not None)
        time.sleep(1.2)
        assert low <= src.rate() <= high
    assert mock.calls_to("subscribeCameras")[0][-1] == fps


def test_a_camera_with_nothing_to_serve_has_no_image(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url)) as src:
        wait_until(lambda: latest_seq(src) >= 3)
        assert src.latest("bottom") is None
        assert src.rate() > 0  # polls with one image count

        mock.set_image("bottom", gradient(2))
        wait_until(lambda: src.latest("bottom") is not None)
        assert latest(src, "bottom").image[0, 0, 2] == 2


def latest_seq(src: CameraSource) -> int:
    image = src.latest("top")
    return 0 if image is None else image.seq


def test_nothing_before_naoqi_serves_images():
    mock = MockNaoqi().start()
    try:
        with closing(CameraSource(mock.url)) as src:
            time.sleep(0.5)
            assert src.latest("top") is None and src.latest("bottom") is None
            assert src.rate() == 0.0
            assert mock.subscribers() == [
                f"{SUBSCRIBER_NAME}_0"
            ]  # subscribed all the same
    finally:
        mock.close()


def test_startup_clears_what_a_crashed_viewer_left_but_not_other_names(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    session = connect(url)
    try:
        video = session.service("ALVideoDevice")
        for _ in range(3):  # three crashed viewers' leftovers
            video.subscribeCameras(SUBSCRIBER_NAME, [0, 1], [1, 1], [11, 11], 10)
        video.subscribeCameras("someone_else", [0], [1], [11], 5)
    finally:
        session.close()
    assert len(mock.subscribers()) == 4

    with closing(CameraSource(url)) as src:
        wait_until(lambda: mock.calls_to("subscribeCameras")[-1][0] == SUBSCRIBER_NAME)
        assert sorted(mock.subscribers()) == [f"{SUBSCRIBER_NAME}_0", "someone_else_0"]
        del src
    assert mock.subscribers() == ["someone_else_0"]


def test_failing_calls_keep_the_last_image_then_recover_without_clearing_again(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url)) as src:
        wait_until(lambda: src.latest("top") is not None)
        mock.fail_calls("ALVideoDevice is gone")
        time.sleep(0.2)
        stalled = latest(src, "top")
        time.sleep(0.4)
        assert src.latest("top") is stalled
        assert time.monotonic() - stalled.received_at >= 0.4

        mock.fail_calls(None)
        mock.set_image("top", gradient(5))
        wait_until(lambda: latest(src, "top").seq > stalled.seq, timeout=10)
        assert latest(src, "top").image[0, 0, 2] == 5
        assert mock.calls_to("unsubscribeAllInstances") == [(SUBSCRIBER_NAME,)]
        assert mock.subscribers() == [f"{SUBSCRIBER_NAME}_0"]  # the old one was removed


def test_subscribes_again_after_naoqi_restarts(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url)) as src:
        wait_until(lambda: src.latest("top") is not None)
        before = latest(src, "top")
        mock.disconnect()
        time.sleep(0.3)
        assert src.latest("top") is before

        mock.restart()
        mock.set_image("top", gradient(7))
        wait_until(lambda: latest(src, "top").image[0, 0, 2] == 7, timeout=10)
        assert mock.subscribers() == [f"{SUBSCRIBER_NAME}_0"]
        assert len(mock.calls_to("unsubscribeAllInstances")) == 1


def test_two_viewers_on_one_naoqi_both_get_images(
    mock_naoqi: tuple[str, MockNaoqi], monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setattr(cameras, "SUBSCRIPTION_CHECK", 0.2)
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with closing(CameraSource(url)) as first:
        wait_until(lambda: first.latest("top") is not None)
        with closing(CameraSource(url)) as second:
            wait_until(lambda: second.latest("top") is not None)
            # The second cleared the first's subscription and got its suffix back, so the
            # first's polls are answered through the second's subscription meanwhile.
            seq = latest(first, "top").seq
            wait_until(lambda: latest(first, "top").seq > seq + 2, timeout=10)
            assert mock.subscribers() == [f"{SUBSCRIBER_NAME}_0"]
            assert len(mock.calls_to("unsubscribeAllInstances")) == 2  # once each
            assert len(mock.calls_to("subscribeCameras")) == 2
        # The second closed and unsubscribed: the first notices and subscribes again.
        seq = latest(first, "top").seq
        wait_until(lambda: latest(first, "top").seq > seq + 2, timeout=10)
        assert mock.subscribers() == [f"{SUBSCRIBER_NAME}_0"]
        assert len(mock.calls_to("subscribeCameras")) == 3
        assert len(mock.calls_to("unsubscribeAllInstances")) == 2  # not cleared again
    assert mock.subscribers() == []


def test_close_is_prompt_when_naoqi_is_gone(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    src = CameraSource(url)
    wait_until(lambda: src.latest("top") is not None)
    mock.disconnect()
    time.sleep(0.2)
    started = time.monotonic()
    src.close()
    assert time.monotonic() - started < 3.0


def test_the_source_only_subscribes_polls_and_unsubscribes(
    mock_naoqi: tuple[str, MockNaoqi], caplog: pytest.LogCaptureFixture
):
    url, mock = mock_naoqi
    mock.set_image("top", gradient(1))
    with (
        caplog.at_level(logging.INFO, logger="naoqi_viewer.cameras"),
        closing(CameraSource(url)) as src,
    ):
        wait_until(lambda: latest_seq(src) >= 3)
    video_calls = {
        method for service, method, _ in mock.calls if service == "ALVideoDevice"
    }
    assert video_calls == {
        "unsubscribeAllInstances",
        "subscribeCameras",
        "getImagesRemote",
        "unsubscribe",
    }
    assert not [r for r in caplog.records if r.levelno >= logging.WARNING]
    assert any("subscribed" in r.getMessage() for r in caplog.records)


# --- decode_images (pure) ---------------------------------------------------------------


def naoqi_entry(image: np.ndarray, index: int, data: object = None) -> list[object]:
    height, width = image.shape[:2]
    pixels = image.tobytes() if data is None else data
    return [width, height, 3, 11, 1700000000, 250000, pixels, index, 0.0, 0.0, 0.0, 0.0]


def test_decode_images_reads_naoqi_layout_and_skips_what_is_not_an_image():
    top, bottom = gradient(1, (4, 3)), gradient(2, (4, 3))
    reply = [
        naoqi_entry(top, 0),
        tuple(
            naoqi_entry(bottom, 1, bytearray(bottom.tobytes()))
        ),  # a raw buffer arrives so
    ]
    images = decode_images(reply, received_at=12.5, seq=7)
    assert set(images) == {"top", "bottom"}
    np.testing.assert_array_equal(images["top"].image, top)
    np.testing.assert_array_equal(images["bottom"].image, bottom)
    assert images["top"].naoqi_time == pytest.approx(1700000000.25)
    assert (images["bottom"].seq, images["bottom"].received_at) == (7, 12.5)
    assert (
        images["top"].image.flags["C_CONTIGUOUS"] and images["top"].image.base is None
    )

    empty = [0, 0, 0, 11, 0, 0, b"", 1, 0.0, 0.0, 0.0, 0.0]  # nothing to serve
    short = naoqi_entry(top, 0, top.tobytes()[:-1])  # a byte count off
    unknown = naoqi_entry(top, 3)  # not a head camera
    text = naoqi_entry(top, 0, "x" * top.size)  # a string, not a buffer
    assert (
        decode_images([empty, short, unknown, text, [], None], received_at=0, seq=1)
        == {}
    )
    assert decode_images(None, received_at=0, seq=1) == {}
    assert decode_images([], received_at=0, seq=1) == {}


# --- The pictures (pure) ----------------------------------------------------------------


def test_picture_rects_stack_two_4_3_pictures_in_the_top_right_corner():
    rects = picture_rects((0, 0, 1600, 1000))
    assert rects is not None
    top, bottom = rects
    assert (top[2], top[3]) == (400, 300) and (bottom[2], bottom[3]) == (400, 300)
    inset = 32  # 2 % of 1600
    assert (top[0], top[1]) == (1600 - 400 - inset, 1000 - inset - 300)
    assert (bottom[0], bottom[1]) == (top[0], top[1] - inset - 300)

    shifted = picture_rects((100, 50, 1600, 1000))
    assert shifted is not None
    assert [(r[0] - 100, r[1] - 50, r[2], r[3]) for r in shifted] == rects


def test_picture_rects_have_even_sides_and_keep_the_aspect():
    rects = picture_rects((0, 0, 1366, 768))
    assert rects is not None
    for _, _, width, height in rects:
        assert width % 2 == 0 and height % 2 == 0
        assert abs(height / width - 0.75) < 0.01


def test_picture_rects_none_when_the_view_is_too_small_or_too_short():
    assert picture_rects((0, 0, 120, 100)) is None  # a 30 px picture
    assert picture_rects((0, 0, 1600, 600)) is None  # 2 x 300 + 3 x 32 does not fit
    assert picture_rects((0, 0, 1600, 700)) is not None


def test_resample_nearest_keeps_the_corners_and_the_size():
    image = gradient(3)
    bigger = resample_nearest(image, (400, 300))
    assert bigger.shape == (300, 400, 3) and bigger.flags["C_CONTIGUOUS"]
    assert bigger[0, 0].tolist() == image[0, 0].tolist()
    assert bigger[-1, -1].tolist() == image[-1, -1].tolist()
    assert bigger[0, -1].tolist() == image[0, -1].tolist()
    smaller = resample_nearest(image, (64, 48))
    assert smaller.shape == (48, 64, 3)
    assert smaller[-1, 0].tolist() == image[-1, 0].tolist()
    same = resample_nearest(image, (320, 240))
    assert same is not image and np.array_equal(same, image)


def test_framed_and_placeholder():
    picture = framed(np.zeros((30, 40, 3), dtype=np.uint8))
    b = PICTURE_BORDER
    assert picture[:b].min() == picture[-b:].min() == 220
    assert picture[:, :b].min() == picture[:, -b:].min() == 220
    assert picture[b:-b, b:-b].max() == 0  # the inside untouched

    slot = placeholder((40, 30))
    assert slot.shape == (30, 40, 3) and slot.dtype == np.uint8
    assert slot[15, 20].tolist() == [40, 40, 40] and slot[0, 0].tolist() == [
        220,
        220,
        220,
    ]
