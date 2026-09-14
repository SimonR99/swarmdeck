import numpy as np
from types import SimpleNamespace as NS

from autonomy.capture_color import (
    message_stamp_ns,
    qualified_rgbd_frame,
    retain_bounded_image,
    select_geometry_and_color,
    select_rgbd_observation,
    validated_image_bytes,
)


def test_qualified_raw_capture_is_colored_in_its_actual_order():
    frontend = np.array([[9.0, 9.0, 9.0]])
    raw_xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    raw = (raw_xyz, "raw-mount", "lidar", "qualified")
    seen = []

    result = select_geometry_and_color(
        frontend, ("frontend-mount", "base"), raw,
        lambda points: seen.append(points) or np.full((len(points), 4), 255, np.uint8),
    )

    assert seen == [raw_xyz]
    assert result[0] is raw_xyz
    assert result[1:4] == ("raw-mount", "lidar", "qualified")
    assert result[4].shape == (2, 4)


def test_frontend_geometry_is_colored_when_no_raw_capture_qualified():
    frontend = np.array([[1.0, 0.0, 0.0]])
    result = select_geometry_and_color(
        frontend, ("mount", "base"), None,
        lambda points: np.zeros((len(points), 4), np.uint8),
    )
    assert result[0] is frontend
    assert result[1:4] == ("mount", "base", None)


def test_rgbd_requires_matching_frames_and_capture_timestamps():
    args = (1_000_000_000, 1_010_000_000, 1_020_000_000)
    assert qualified_rgbd_frame(*args, "optical", "optical", "optical") == "optical"
    assert qualified_rgbd_frame(*args, "color", "depth", "color") is None
    assert qualified_rgbd_frame(1_000_000_000, 1_300_000_000, 1_300_000_000,
                                "optical", "optical", "optical") is None
    assert qualified_rgbd_frame(*args, "optical", "optical", "optical", "other") is None


def _message(stamp_ns, frame="camera"):
    return NS(
        header=NS(
            stamp=NS(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id=frame,
        )
    )


def _image_message(stamp_ns=1, *, encoding="rgb8", width=2, height=2,
                   step=6, data=b"\0" * 12):
    message = _message(stamp_ns)
    message.encoding = encoding
    message.width = width
    message.height = height
    message.step = step
    message.data = data
    return message


def test_image_cache_input_validation_is_fail_closed_and_byte_bounded():
    image = _image_message()
    assert message_stamp_ns(image) == 1
    assert validated_image_bytes(image, "color", 12) == 12
    assert validated_image_bytes(image, "color", 11) is None
    assert validated_image_bytes(_image_message(data=b"short"), "color", 12) is None
    assert validated_image_bytes(
        _image_message(encoding="unsupported"), "color", 12,
    ) is None
    assert validated_image_bytes(
        _image_message(encoding="32FC1", step=8, data=b"\0" * 16),
        "depth",
        16,
    ) == 16
    image.header.stamp.nanosec = 1_000_000_000
    try:
        message_stamp_ns(image)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid ROS nanoseconds accepted")


def test_image_cache_bounds_bytes_count_and_refreshes_repeated_stamp():
    cache = {}
    assert retain_bounded_image(cache, _image_message(1), "color", 2, 24)
    assert retain_bounded_image(cache, _image_message(2), "color", 2, 24)
    assert retain_bounded_image(cache, _image_message(3), "color", 2, 24)
    assert list(cache) == [2, 3]
    assert retain_bounded_image(cache, _image_message(2), "color", 2, 24)
    assert list(cache) == [3, 2]
    assert retain_bounded_image(cache, _image_message(4), "color", 2, 12)
    assert list(cache) == [4]
    before = dict(cache)
    assert not retain_bounded_image(
        cache, _image_message(5, data=b"short"), "color", 2, 24,
    )
    assert cache == before


def test_rolling_rgbd_caches_keep_a_pair_for_the_latest_scan():
    images, depths = {}, {}
    for tick in range(1, 6):
        stamp = tick * 100_000_000
        assert retain_bounded_image(
            images, _image_message(stamp), "color", 3, 36,
        )
        assert retain_bounded_image(
            depths,
            _image_message(
                stamp,
                encoding="32FC1",
                step=8,
                data=b"\0" * 16,
            ),
            "depth",
            3,
            48,
        )
    assert list(images) == [300_000_000, 400_000_000, 500_000_000]
    assert list(depths) == [300_000_000, 400_000_000, 500_000_000]
    selected = select_rgbd_observation(
        500_000_000, images.values(), depths.values(), _message(0),
    )
    assert selected == (images[500_000_000], depths[500_000_000], "camera")


def test_rgbd_selection_recovers_pair_hidden_by_latest_only_callback_order():
    old_image, new_image = _message(1_000_000_000), _message(1_200_000_000)
    old_depth = _message(1_000_000_000)
    info = _message(0)

    selected = select_rgbd_observation(
        1_010_000_000,
        [old_image, new_image],
        [old_depth],
        info,
    )

    assert selected == (old_image, old_depth, "camera")


def test_rgbd_selection_prefers_tight_pair_and_keeps_quality_gates():
    close_image = _message(1_010_000_000)
    exact_image = _message(1_100_000_000)
    offset_depth = _message(1_040_000_000)
    exact_depth = _message(1_100_000_000)
    info = _message(0)
    assert select_rgbd_observation(
        1_000_000_000,
        [close_image, exact_image],
        [offset_depth, exact_depth],
        info,
    ) == (exact_image, exact_depth, "camera")
    assert select_rgbd_observation(
        1_000_000_000,
        [_message(1_300_000_000)],
        [_message(1_300_000_000)],
        info,
    ) is None
