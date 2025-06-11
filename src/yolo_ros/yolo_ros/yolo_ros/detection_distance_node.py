#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from yolo_msgs.msg import DetectionArray, DetectionDistance
from visualization_msgs.msg import Marker

class DistanceNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('detection_distance_node')

        # parameters for topic names and depth scaling
        self.declare_parameter('depth_image_topic', 'depth_image')
        self.declare_parameter('camera_info_topic', 'depth_info')
        self.declare_parameter('detections_topic', 'detections')
        self.declare_parameter('depth_image_units_divisor', 1000)
        self.declare_parameter('depth_image_reliability', QoSReliabilityPolicy.BEST_EFFORT)

        # helpers
        self.cv_bridge = CvBridge()
        self.camera_info = None  # will be set in callback

    def on_configure(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Configuring…')

        # load parameters
        self.depth_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.depth_image_units_divisor = (
            self.get_parameter('depth_image_units_divisor')
                .get_parameter_value().integer_value
        )
        dimg_rel = (
            self.get_parameter('depth_image_reliability')
                .get_parameter_value().integer_value
        )
        self.depth_qos = QoSProfile(
            reliability=dimg_rel,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # publishers
        self.distance_pub = self.create_publisher(
            DetectionDistance, 'detection_distances', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, 'detection_markers', 10
        )

        super().on_configure(state)
        self.get_logger().info(f'[{self.get_name()}] Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Activating…')

        # subscribe to CameraInfo separately
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile=self.depth_qos
        )

        # message_filters for depth + detections
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=self.depth_qos
        )
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, self.det_topic
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.depth_sub, self.detections_sub),
            queue_size=10,
            slop=0.5
        )
        self.sync.registerCallback(self.on_detection)

        super().on_activate(state)
        self.get_logger().info(f'[{self.get_name()}] Activated')
        return TransitionCallbackReturn.SUCCESS

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # store latest camera_info
        self.camera_info = msg

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Deactivating…')
        self.destroy_subscription(self.depth_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)
        del self.sync

        super().on_deactivate(state)
        self.get_logger().info(f'[{self.get_name()}] Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Cleaning up…')
        self.destroy_publisher(self.distance_pub)
        self.destroy_publisher(self.marker_pub)

        super().on_cleanup(state)
        self.get_logger().info(f'[{self.get_name()}] Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Shutting down…')
        super().on_cleanup(state)
        self.get_logger().info(f'[{self.get_name()}] Shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def on_detection(
        self,
        depth_msg: Image,
        detections_msg: DetectionArray
    ) -> None:
        # convert ROS Image to OpenCV
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        h, w = depth_image.shape[:2]

        # publish for each 2D detection
        marker_id = 0
        for det in detections_msg.detections:
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            if not (0 <= u < w and 0 <= v < h):
                continue

            raw = depth_image[v, u]
            dist = float(raw) / self.depth_image_units_divisor
            if dist <= 0.0:
                continue

            # publish scalar distance
            dist_msg = DetectionDistance()
            dist_msg.label = str(det.class_id)
            dist_msg.confidence = det.score
            dist_msg.distance = dist
            self.distance_pub.publish(dist_msg)

            # only visualize marker if camera_info received
            if self.camera_info:
                k = self.camera_info.k
                fx, fy = k[0], k[4]
                cx, cy = k[2], k[5]

                # project to 3D
                X = (u - cx) * dist / fx
                Y = (v - cy) * dist / fy
                Z = dist

                # publish visualization marker
                marker = Marker()
                marker.header.frame_id = self.camera_info.header.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'detections'
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = X
                marker.pose.position.y = Y
                marker.pose.position.z = Z
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05

                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                self.marker_pub.publish(marker)
                marker_id += 1


def main():
    rclpy.init()
    node = DistanceNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
