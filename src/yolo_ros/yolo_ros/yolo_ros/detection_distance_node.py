#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image, CameraInfo
from yolo_msgs.msg import DetectionArray, DetectionDistance

import numpy as np

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
        self.tf_broadcaster = None  # initialized on activation

        # store last positions for selected classes
        self.last_positions = {}
        # mapping COCO IDs to frame base names
        self.class_map = {0: 'person', 56: 'chair', 65: 'remote'}

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

        # publisher for scalar distances
        self.distance_pub = self.create_publisher(
            DetectionDistance, 'detection_distances', 10
        )

        super().on_configure(state)
        self.get_logger().info(f'[{self.get_name()}] Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Activating…')

        # subscribe to CameraInfo
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

        # initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # timer to log positions every 100ms
        self.log_timer = self.create_timer(0.1, self.timer_callback)

        super().on_activate(state)
        self.get_logger().info(f'[{self.get_name()}] Activated')
        return TransitionCallbackReturn.SUCCESS

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def timer_callback(self):
        # print last positions for target classes
        for frame_id, (x, y, z) in self.last_positions.items():
            self.get_logger().info(
                f"{frame_id}: X={x:.3f} m, Y={y:.3f} m, Z={z:.3f} m"
            )

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Deactivating…')
        self.destroy_subscription(self.depth_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)
        del self.sync
        self.log_timer.cancel()

        super().on_deactivate(state)
        self.get_logger().info(f'[{self.get_name()}] Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        self.get_logger().info(f'[{self.get_name()}] Cleaning up…')
        self.destroy_publisher(self.distance_pub)

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
        # clear last positions and reset counters
        self.last_positions.clear()
        occurrence_counter = {}

        # convert ROS Image to OpenCV
        depth_image = self.cv_bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )
        h, w = depth_image.shape[:2]

        for det in detections_msg.detections:
            class_id = det.class_id
            if class_id not in self.class_map:
                continue

            # update occurrence count
            count = occurrence_counter.get(class_id, 0) + 1
            occurrence_counter[class_id] = count
            frame_id = f"{self.class_map[class_id]}{count}"

            # bounding box ROI
            u_c = int(det.bbox.center.position.x)
            v_c = int(det.bbox.center.position.y)
            box_w = det.bbox.size.x
            box_h = det.bbox.size.y
            u0 = max(int(u_c - box_w/2), 0)
            u1 = min(int(u_c + box_w/2), w-1)
            v0 = max(int(v_c - box_h/2), 0)
            v1 = min(int(v_c + box_h/2), h-1)

            # extract depths in ROI
            window = depth_image[v0:v1+1, u0:u1+1]
            valid = window[window > 0]
            if valid.size == 0:
                continue

            # select nearest half of pixels
            sorted_vals = np.sort(valid.flatten())
            half_count = sorted_vals.size // 2
            if half_count > 0:
                chosen = sorted_vals[:half_count]
            else:
                chosen = sorted_vals
            median_raw = np.median(chosen)
            dist = float(median_raw) / self.depth_image_units_divisor

            # publish scalar distance
            dist_msg = DetectionDistance()
            dist_msg.label = str(class_id)
            dist_msg.confidence = det.score
            dist_msg.distance = dist
            self.distance_pub.publish(dist_msg)

            if self.camera_info:
                k = self.camera_info.k
                fx, fy = k[0], k[4]
                cx, cy = k[2], k[5]

                X = (u_c - cx) * dist / fx
                Y = (v_c - cy) * dist / fy
                Z = dist

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.camera_info.header.frame_id
                t.child_frame_id = frame_id
                t.transform.translation.x = X
                t.transform.translation.y = Y
                t.transform.translation.z = Z
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

                self.last_positions[frame_id] = (X, Y, Z)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
