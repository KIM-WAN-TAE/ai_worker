import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from ffw_bringup.color_zoom import crop_bounds, zoom_camera_info, zoom_image


@pytest.mark.parametrize('rotation,expected', [
    (0, (160, 240, 320, 240)),
    (90, (320, 120, 320, 240)),
    (180, (160, 0, 320, 240)),
    (270, (0, 120, 320, 240)),
])
def test_display_bottom_crop(rotation, expected):
    assert crop_bounds(640, 480, 2.0, rotation) == expected


def test_zoom_matches_bottom_half_of_rotated_display():
    frame = np.random.default_rng(7).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    displayed = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    # The 480x640 displayed image is cropped to its bottom-center 240x320.
    expected = cv2.resize(displayed[320:640, 120:360], (480, 640))
    actual = cv2.rotate(zoom_image(frame, 2.0, 270), cv2.ROTATE_90_COUNTERCLOCKWISE)
    # OpenCV's separable integer interpolation can round differently by one
    # intensity level when resize and rotation are performed in opposite orders.
    np.testing.assert_allclose(actual, expected, atol=1, rtol=0)


def test_calibration_preserves_projection_after_crop():
    info = CameraInfo(width=640, height=480)
    info.header.frame_id = 'camera_left_color_optical_frame'
    info.header.stamp.sec = 123
    info.k = [400., 0., 319., 0., 401., 239., 0., 0., 1.]
    info.p = [400., 0., 319., -20., 0., 401., 239., 0., 0., 0., 1., 0.]
    info.d = [0.1, -0.1, 0., 0., 0.]
    result = zoom_camera_info(info, 2.0, 270)
    np.testing.assert_allclose(result.k, [800., 0., 638., 0., 802., 238., 0., 0., 1.])
    np.testing.assert_allclose(result.p, [800., 0., 638., -40., 0., 802., 238., 0., 0., 0., 1., 0.])
    assert result.header == info.header
    assert result.d == info.d
    assert (result.width, result.height) == (640, 480)
    assert info.k[0] == 400.  # The source calibration is untouched.


def test_one_times_zoom_is_identity():
    frame = np.random.default_rng(1).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    np.testing.assert_array_equal(zoom_image(frame, 1.0, 270), frame)


def test_hd_crop_keeps_four_by_three_output():
    assert crop_bounds(1280, 720, 2.0, 270, (640, 480)) == (160, 180, 480, 360)
    frame = np.random.default_rng(2).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
    actual = zoom_image(frame, 2.0, 270, (640, 480))
    expected = cv2.resize(frame[180:540, 160:640], (640, 480))
    np.testing.assert_array_equal(actual, expected)


def test_hd_calibration_maps_same_rays_as_vga():
    old = CameraInfo(width=640, height=480)
    old.k = [400., 0., 319., 0., 401., 239., 0., 0., 1.]
    old.p = [400., 0., 319., 0., 0., 401., 239., 0., 0., 0., 1., 0.]
    hd = CameraInfo(width=1280, height=720)
    hd.k = [600., 0., 638.5, 0., 601.5, 358.5, 0., 0., 1.]
    hd.p = [600., 0., 638.5, 0., 0., 601.5, 358.5, 0., 0., 0., 1., 0.]
    expected = zoom_camera_info(old, 2.0, 270)
    actual = zoom_camera_info(hd, 2.0, 270, (640, 480))
    np.testing.assert_allclose(actual.k, expected.k)
    np.testing.assert_allclose(actual.p, expected.p)
    assert (actual.width, actual.height) == (640, 480)


def test_calibrated_d405_hd_crop_preserves_measured_vga_framing():
    old = CameraInfo(width=640, height=480)
    old.k = [392.9569091796875, 0., 319.38250732421875,
             0., 392.4443359375, 240.80543518066406, 0., 0., 1.]
    hd = CameraInfo(width=1280, height=720)
    hd.k = [653.22705078125, 0., 638.9735107421875,
            0., 652.375, 361.33892822265625, 0., 0., 1.]
    expected = zoom_camera_info(old, 2.0, 270)
    actual = zoom_camera_info(hd, 2.0, 270, (640, 480), (108, 161, 532, 399))
    # Integer cropping retains the same ray projection within one output pixel.
    np.testing.assert_allclose(actual.k, expected.k, atol=0.61, rtol=0)


def test_calibrated_crop_pixels():
    frame = np.random.default_rng(3).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
    actual = zoom_image(frame, 2.0, 270, (640, 480), (108, 161, 532, 399))
    expected = cv2.resize(frame[161:560, 108:640], (640, 480))
    np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize('crop', [(0, 0, 1300, 720), (-1, 0, 532, 399),
                                 (108, 161, 0, 399), (0, 0, 640, 360)])
def test_invalid_calibrated_crop_is_rejected(crop):
    with pytest.raises(ValueError):
        crop_bounds(1280, 720, 2., 270, (640, 480), crop)


@pytest.mark.parametrize('zoom', [0., 0.5, 9., float('nan'), float('inf')])
def test_invalid_zoom_is_rejected(zoom):
    with pytest.raises(ValueError):
        crop_bounds(640, 480, zoom, 270)
