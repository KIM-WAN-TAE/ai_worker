"""Publish a cropped color stream and its matching camera calibration."""

import copy
import math

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, RegionOfInterest


def crop_bounds(width, height, zoom, display_rotation, output_size=None, source_crop=None):
    """Return a crop anchored at the bottom center after display rotation."""
    if not math.isfinite(zoom) or not 1.0 <= zoom <= 8.0:
        raise ValueError('zoom must be finite and between 1 and 8')
    if display_rotation not in (0, 90, 180, 270):
        raise ValueError('display_rotation must be 0, 90, 180 or 270')
    if width <= 0 or height <= 0:
        raise ValueError('image dimensions must be positive')
    if source_crop is not None:
        if len(source_crop) != 4:
            raise ValueError('source_crop must contain x, y, width, height')
        x, y, crop_width, crop_height = source_crop
        if (x < 0 or y < 0 or crop_width <= 0 or crop_height <= 0
                or x + crop_width > width or y + crop_height > height):
            raise ValueError('source_crop must fit within the source image')
        if output_size and crop_width * output_size[1] != crop_height * output_size[0]:
            raise ValueError('source_crop and output must have the same aspect ratio')
        return x, y, crop_width, crop_height
    base_width, base_height = width, height
    if output_size is not None:
        out_width, out_height = output_size
        if out_width <= 0 or out_height <= 0:
            raise ValueError('output dimensions must be positive')
        # Match the original 4:3 field within a wider sensor profile before
        # applying the bottom-anchored zoom; never stretch a 16:9 crop to 4:3.
        if width * out_height > height * out_width:
            base_width = round(height * out_width / out_height)
        else:
            base_height = round(width * out_height / out_width)
    offset_x = (width - base_width) // 2
    offset_y = (height - base_height) // 2
    crop_width = max(1, round(base_width / zoom))
    crop_height = max(1, round(base_height / zoom))
    x = (width - crop_width) // 2
    y = (height - crop_height) // 2
    if display_rotation == 0:
        y = offset_y + base_height - crop_height
    elif display_rotation == 90:
        x = offset_x + base_width - crop_width
    elif display_rotation == 180:
        y = offset_y
    else:
        x = offset_x
    return x, y, crop_width, crop_height


def zoom_image(frame, zoom, display_rotation, output_size=None, source_crop=None):
    height, width = frame.shape[:2]
    x, y, crop_width, crop_height = crop_bounds(
        width, height, zoom, display_rotation, output_size, source_crop)
    return cv2.resize(
        frame[y:y + crop_height, x:x + crop_width],
        output_size or (width, height), interpolation=cv2.INTER_LINEAR,
    )


def zoom_camera_info(msg, zoom, display_rotation, output_size=None, source_crop=None):
    """Apply the crop/resize pixel transform to K and P, preserving extrinsics."""
    result = copy.deepcopy(msg)
    x, y, width, height = crop_bounds(
        msg.width, msg.height, zoom, display_rotation, output_size, source_crop)
    result.width, result.height = output_size or (msg.width, msg.height)
    sx, sy = result.width / width, result.height / height
    # A @ K and A @ P, where A maps source pixels into the resized crop.
    for values, columns in ((result.k, 3), (result.p, 4)):
        original = list(values)
        for col in range(columns):
            values[col] = sx * (original[col] - x * original[2 * columns + col])
            values[columns + col] = sy * (
                original[columns + col] - y * original[2 * columns + col])
    result.binning_x = 0
    result.binning_y = 0
    result.roi = RegionOfInterest()
    return result


class ColorZoom(Node):
    def __init__(self):
        super().__init__('color_zoom')
        descriptor = ParameterDescriptor(read_only=True)
        defaults = {
            'input_image': '/camera_left/camera_left/color/unzoomed/image_rect_raw',
            'input_info': '/camera_left/camera_left/color/unzoomed/camera_info',
            'output_image': '/camera_left/camera_left/color/image_rect_raw',
            'output_info': '/camera_left/camera_left/color/camera_info',
            'zoom': 2.0,
            'display_rotation': 270,
            'jpeg_quality': 95,
            'output_width': 640,
            'output_height': 480,
            # Optional calibrated rectangle for matching a previous sensor mode.
            # All zeros select automatic aspect-preserving, bottom-anchored zoom.
            'source_crop': [0, 0, 0, 0],
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, descriptor)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.zoom = values['zoom']
        self.rotation = values['display_rotation']
        self.jpeg_quality = values['jpeg_quality']
        self.output_size = (values['output_width'], values['output_height'])
        self.source_crop = tuple(values['source_crop']) if any(values['source_crop']) else None
        crop_bounds(640, 480, self.zoom, self.rotation, self.output_size)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')
        if values['input_image'] == values['output_image']:
            raise ValueError('input_image and output_image must differ')
        self.bridge = CvBridge()
        qos = QoSProfile(depth=1)
        self.raw_pub = self.create_publisher(Image, values['output_image'], qos)
        self.jpeg_pub = self.create_publisher(
            CompressedImage, values['output_image'] + '/compressed', qos)
        self.info_pub = self.create_publisher(CameraInfo, values['output_info'], qos)
        self.create_subscription(
            Image, values['input_image'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, values['input_info'], self.on_info, qos_profile_sensor_data)
        self.get_logger().info(
            f"Color zoom {self.zoom}x, displayed bottom center at {self.rotation} degrees: "
            f"{values['input_image']} -> {values['output_image']}")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = zoom_image(frame, self.zoom, self.rotation, self.output_size, self.source_crop)
        raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw.header = msg.header
        self.raw_pub.publish(raw)
        if self.jpeg_pub.get_subscription_count():
            success, encoded = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if success:
                compressed = CompressedImage()
                compressed.header = msg.header
                compressed.format = 'bgr8; jpeg compressed bgr8'
                compressed.data = encoded.tobytes()
                self.jpeg_pub.publish(compressed)

    def on_info(self, msg):
        if msg.width and msg.height:
            self.info_pub.publish(zoom_camera_info(
                msg, self.zoom, self.rotation, self.output_size, self.source_crop))


def main(args=None):
    rclpy.init(args=args)
    node = ColorZoom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
