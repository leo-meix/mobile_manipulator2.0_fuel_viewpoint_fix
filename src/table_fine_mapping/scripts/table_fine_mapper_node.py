#!/usr/bin/env python3
"""Local color-TSDF mapping around a YOLO-detected table.

The table segmentation mask is used only to estimate the tabletop and lock a
3-D region of interest.  All valid RGB-D pixels inside that region are fused,
so unlabeled cups, bottles and other tabletop objects remain in the map.
"""

import math
import os
import threading
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

try:
    import open3d as o3d
except ImportError as exc:
    o3d = None
    OPEN3D_IMPORT_ERROR = exc
else:
    OPEN3D_IMPORT_ERROR = None


class TableFineMapper:
    def __init__(self):
        rospy.init_node("table_fine_mapper")
        if o3d is None:
            rospy.logfatal("Open3D is required: %s. Install it with: pip3 install open3d",
                           OPEN3D_IMPORT_ERROR)
            raise RuntimeError("open3d is not installed")

        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.rng = np.random.default_rng(7)

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.camera_frame = rospy.get_param(
            "~camera_frame", "robot1_camera_depth_optical_frame")
        self.color_topic = rospy.get_param(
            "~color_topic", "/robot1_camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/robot1_camera/depth/image_raw")
        self.info_topic = rospy.get_param(
            "~camera_info_topic", "/robot1_camera/depth/camera_info")
        self.mask_topic = rospy.get_param(
            "~mask_topic", "/yolo_table_detector/mask")
        self.capture_topic = rospy.get_param(
            "~capture_topic", "/table_fine_mapping/capture")

        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 40))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.10))
        self.localization_rate = float(rospy.get_param("~localization_rate", 5.0))
        self.min_mask_pixels = int(rospy.get_param("~min_mask_pixels", 1200))
        self.max_plane_points = int(rospy.get_param("~max_plane_points", 12000))
        self.ransac_iterations = int(rospy.get_param("~ransac_iterations", 100))
        self.plane_threshold = float(
            rospy.get_param("~plane_distance_threshold", 0.018))
        self.max_table_tilt = math.radians(
            float(rospy.get_param("~max_table_tilt_deg", 18.0)))
        self.min_plane_inliers = int(rospy.get_param("~min_plane_inliers", 350))
        self.min_table_extent = float(rospy.get_param("~min_table_extent", 0.30))
        self.stable_frames_required = int(rospy.get_param("~stable_frames", 5))
        self.stable_center_tolerance = float(
            rospy.get_param("~stable_center_tolerance", 0.12))
        self.roi_alpha = float(rospy.get_param("~roi_smoothing_alpha", 0.35))
        self.roi_padding_xy = float(rospy.get_param("~roi_padding_xy", 0.18))
        self.roi_below = float(rospy.get_param("~roi_below_table", 0.85))
        self.roi_above = float(rospy.get_param("~roi_above_table", 0.65))

        self.voxel_length = float(rospy.get_param("~voxel_length", 0.008))
        self.sdf_trunc = float(rospy.get_param("~sdf_trunc", 0.032))
        self.depth_min = float(rospy.get_param("~depth_min", 0.25))
        self.depth_max = float(rospy.get_param("~depth_max", 2.50))
        self.integration_rate = float(rospy.get_param("~integration_rate", 4.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 0.5))
        self.min_roi_depth_pixels = int(
            rospy.get_param("~min_roi_depth_pixels", 1000))
        self.output_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~output_directory", "/tmp/table_fine_maps")))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.roi_min = None
        self.roi_max = None
        self.table_height = None
        self.stable_count = 0
        self.roi_locked = False
        self.capture_enabled = False
        self.integrated_frames = 0
        self.last_localization_time = rospy.Time(0)
        self.last_integration_time = rospy.Time(0)
        self.last_publish_time = rospy.Time(0)
        self.last_status = "waiting for synchronized RGB-D and YOLO mask"
        self.uv_cache = {}
        self.volume = self._new_volume()

        self.cloud_pub = rospy.Publisher(
            "~surface_cloud", PointCloud2, queue_size=1, latch=True)
        self.roi_pub = rospy.Publisher("~table_roi", Marker, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.capture_state_pub = rospy.Publisher(
            "~capture_state", Bool, queue_size=1, latch=True)

        self.capture_sub = rospy.Subscriber(
            self.capture_topic, Bool, self._capture_topic_callback, queue_size=1)
        self.save_srv = rospy.Service("~save", Trigger, self._save_callback)
        self.reset_srv = rospy.Service("~reset", Trigger, self._reset_callback)
        self.capture_srv = rospy.Service("~set_capture", SetBool, self._capture_service)

        self.color_sub = message_filters.Subscriber(self.color_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.mask_sub = message_filters.Subscriber(self.mask_topic, Image)
        self.info_sub = message_filters.Subscriber(self.info_topic, CameraInfo)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.mask_sub, self.info_sub],
            self.sync_queue_size, self.sync_slop, allow_headerless=False)
        self.sync.registerCallback(self._rgbd_callback)

        rospy.Timer(rospy.Duration(0.5), self._status_timer)
        rospy.loginfo("[table_fine_mapper] ready: voxel=%.3f m, topics=(%s, %s, %s)",
                      self.voxel_length, self.color_topic, self.depth_topic,
                      self.mask_topic)
        rospy.loginfo("[table_fine_mapper] move around the table with capture OFF; "
                      "enable capture only after the robot has settled")

    def _new_volume(self):
        return o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    @staticmethod
    def _transform_matrix(transform):
        q = transform.rotation
        matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[0, 3] = transform.translation.x
        matrix[1, 3] = transform.translation.y
        matrix[2, 3] = transform.translation.z
        return matrix

    def _lookup_camera_pose(self, frame_id, stamp):
        source_frame = self.camera_frame or frame_id
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame, source_frame, stamp,
                rospy.Duration(self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2.0, "[table_fine_mapper] exact TF unavailable: %s", exc)
            return None
        return self._transform_matrix(tf_msg.transform)

    @staticmethod
    def _depth_metres(depth_image, encoding):
        if encoding in ("16UC1", "mono16") or depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * 0.001
        if encoding == "32FC1" or depth_image.dtype == np.float32:
            return depth_image.astype(np.float32)
        raise ValueError("unsupported depth encoding: %s" % encoding)

    def _pixel_grid(self, height, width):
        key = (height, width)
        if key not in self.uv_cache:
            vv, uu = np.indices((height, width), dtype=np.float32)
            self.uv_cache[key] = (uu, vv)
        return self.uv_cache[key]

    def _masked_world_points(self, depth_m, mask, info, world_from_camera):
        valid = ((mask > 127) & np.isfinite(depth_m) &
                 (depth_m >= self.depth_min) & (depth_m <= self.depth_max))
        indices = np.flatnonzero(valid)
        if indices.size > self.max_plane_points:
            indices = self.rng.choice(indices, self.max_plane_points, replace=False)
        if indices.size < self.min_plane_inliers:
            return np.empty((0, 3), dtype=np.float64)

        height, width = depth_m.shape
        uu, vv = self._pixel_grid(height, width)
        z = depth_m.ravel()[indices].astype(np.float64)
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        if fx <= 0.0 or fy <= 0.0:
            return np.empty((0, 3), dtype=np.float64)
        x = (uu.ravel()[indices] - cx) * z / fx
        y = (vv.ravel()[indices] - cy) * z / fy
        camera_points = np.column_stack((x, y, z))
        return camera_points.dot(world_from_camera[:3, :3].T) + world_from_camera[:3, 3]

    def _estimate_horizontal_plane(self, points):
        if points.shape[0] < self.min_plane_inliers:
            return None
        best_indices = None
        min_vertical_dot = math.cos(self.max_table_tilt)

        for _ in range(self.ransac_iterations):
            sample = points[self.rng.choice(points.shape[0], 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            if abs(normal[2]) < min_vertical_dot:
                continue
            distances = np.abs((points - sample[0]).dot(normal))
            inliers = np.flatnonzero(distances < self.plane_threshold)
            if best_indices is None or inliers.size > best_indices.size:
                best_indices = inliers

        if best_indices is None or best_indices.size < self.min_plane_inliers:
            return None

        inliers = points[best_indices]
        center = np.median(inliers, axis=0)
        xy_min = np.percentile(inliers[:, :2], 2.0, axis=0)
        xy_max = np.percentile(inliers[:, :2], 98.0, axis=0)
        extent = xy_max - xy_min
        if np.min(extent) < self.min_table_extent:
            return None

        roi_min = np.array([xy_min[0] - self.roi_padding_xy,
                            xy_min[1] - self.roi_padding_xy,
                            center[2] - self.roi_below], dtype=np.float64)
        roi_max = np.array([xy_max[0] + self.roi_padding_xy,
                            xy_max[1] + self.roi_padding_xy,
                            center[2] + self.roi_above], dtype=np.float64)
        return roi_min, roi_max, float(center[2]), int(best_indices.size)

    def _update_roi(self, estimate):
        roi_min, roi_max, table_height, inlier_count = estimate
        if self.roi_locked:
            return

        new_center = 0.5 * (roi_min + roi_max)
        if self.roi_min is None:
            self.roi_min = roi_min
            self.roi_max = roi_max
            self.table_height = table_height
            self.stable_count = 1
        else:
            old_center = 0.5 * (self.roi_min + self.roi_max)
            stable = (np.linalg.norm(new_center[:2] - old_center[:2]) <=
                      self.stable_center_tolerance and
                      abs(table_height - self.table_height) <=
                      self.stable_center_tolerance * 0.5)
            self.stable_count = self.stable_count + 1 if stable else 1
            alpha = max(0.0, min(1.0, self.roi_alpha))
            self.roi_min = (1.0 - alpha) * self.roi_min + alpha * roi_min
            self.roi_max = (1.0 - alpha) * self.roi_max + alpha * roi_max
            self.table_height = ((1.0 - alpha) * self.table_height +
                                 alpha * table_height)

        self.last_status = "table ROI stabilizing %d/%d (%d plane points)" % (
            self.stable_count, self.stable_frames_required, inlier_count)
        if self.stable_count >= self.stable_frames_required:
            self.roi_locked = True
            self.last_status = "table ROI locked; press SPACE to start capture"
            rospy.loginfo("[table_fine_mapper] ROI locked: min=%s max=%s tabletop_z=%.3f",
                          np.array2string(self.roi_min, precision=3),
                          np.array2string(self.roi_max, precision=3),
                          self.table_height)
        self._publish_roi_marker()

    def _crop_depth_to_roi(self, depth_m, info, world_from_camera):
        valid = (np.isfinite(depth_m) & (depth_m >= self.depth_min) &
                 (depth_m <= self.depth_max))
        indices = np.flatnonzero(valid)
        if indices.size == 0:
            return None, 0

        height, width = depth_m.shape
        uu, vv = self._pixel_grid(height, width)
        z = depth_m.ravel()[indices].astype(np.float64)
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        x = (uu.ravel()[indices] - cx) * z / fx
        y = (vv.ravel()[indices] - cy) * z / fy
        camera_points = np.column_stack((x, y, z))
        world_points = (camera_points.dot(world_from_camera[:3, :3].T) +
                        world_from_camera[:3, 3])
        inside = np.all(world_points >= self.roi_min, axis=1) & np.all(
            world_points <= self.roi_max, axis=1)
        kept_indices = indices[inside]

        masked_depth = np.zeros(depth_m.size, dtype=np.uint16)
        clipped_mm = np.clip(depth_m.ravel()[kept_indices] * 1000.0, 1, 65535)
        masked_depth[kept_indices] = clipped_mm.astype(np.uint16)
        return masked_depth.reshape(depth_m.shape), int(kept_indices.size)

    def _integrate(self, color_bgr, depth_m, info, world_from_camera, stamp):
        if not self.capture_enabled or not self.roi_locked:
            return
        min_period = 1.0 / max(self.integration_rate, 0.1)
        if (stamp - self.last_integration_time).to_sec() < min_period:
            return

        masked_depth, kept_count = self._crop_depth_to_roi(
            depth_m, info, world_from_camera)
        if masked_depth is None or kept_count < self.min_roi_depth_pixels:
            self.last_status = "capture ON, but only %d ROI depth pixels" % kept_count
            return

        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        color_o3d = o3d.geometry.Image(np.ascontiguousarray(color_rgb))
        depth_o3d = o3d.geometry.Image(np.ascontiguousarray(masked_depth))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, depth_scale=1000.0,
            depth_trunc=self.depth_max, convert_rgb_to_intensity=False)
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            int(info.width), int(info.height),
            float(info.K[0]), float(info.K[4]),
            float(info.K[2]), float(info.K[5]))
        camera_from_world = np.linalg.inv(world_from_camera)
        self.volume.integrate(rgbd, intrinsic, camera_from_world)
        self.integrated_frames += 1
        self.last_integration_time = stamp
        self.last_status = "capture ON: %d frames, %d ROI pixels" % (
            self.integrated_frames, kept_count)

        publish_period = 1.0 / max(self.publish_rate, 0.05)
        if (stamp - self.last_publish_time).to_sec() >= publish_period:
            self._publish_surface_cloud(stamp)
            self.last_publish_time = stamp

    def _rgbd_callback(self, color_msg, depth_msg, mask_msg, info_msg):
        if info_msg.width != depth_msg.width or info_msg.height != depth_msg.height:
            rospy.logerr_throttle(
                2.0, "[table_fine_mapper] CameraInfo size %dx%d != depth %dx%d",
                info_msg.width, info_msg.height, depth_msg.width, depth_msg.height)
            return
        if color_msg.width != depth_msg.width or color_msg.height != depth_msg.height:
            rospy.logerr_throttle(
                2.0, "[table_fine_mapper] RGB and depth are not pixel-aligned: %dx%d vs %dx%d",
                color_msg.width, color_msg.height, depth_msg.width, depth_msg.height)
            return

        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
            depth_m = self._depth_metres(depth_raw, depth_msg.encoding)
        except (CvBridgeError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "[table_fine_mapper] image conversion failed: %s", exc)
            return

        if mask.shape != depth_m.shape:
            mask = cv2.resize(mask, (depth_m.shape[1], depth_m.shape[0]),
                              interpolation=cv2.INTER_NEAREST)
        world_from_camera = self._lookup_camera_pose(depth_msg.header.frame_id,
                                                     depth_msg.header.stamp)
        if world_from_camera is None:
            return

        with self.lock:
            localization_period = 1.0 / max(self.localization_rate, 0.1)
            if (not self.roi_locked and
                    (depth_msg.header.stamp - self.last_localization_time).to_sec() >=
                    localization_period):
                self.last_localization_time = depth_msg.header.stamp
                if int(np.count_nonzero(mask)) >= self.min_mask_pixels:
                    table_points = self._masked_world_points(
                        depth_m, mask, info_msg, world_from_camera)
                    estimate = self._estimate_horizontal_plane(table_points)
                    if estimate is not None:
                        self._update_roi(estimate)
                    else:
                        self.stable_count = 0
                        self.last_status = "table mask found, tabletop plane not stable"
                else:
                    self.stable_count = 0
                    self.last_status = "waiting for a sufficiently large table mask"

            self._integrate(color_bgr, depth_m, info_msg, world_from_camera,
                            depth_msg.header.stamp)

    def _publish_roi_marker(self):
        if self.roi_min is None:
            return
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "table_fine_mapping"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        center = 0.5 * (self.roi_min + self.roi_max)
        size = self.roi_max - self.roi_min
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r = 0.1
        marker.color.g = 0.9 if self.roi_locked else 0.5
        marker.color.b = 0.2
        marker.color.a = 0.20
        self.roi_pub.publish(marker)

    def _publish_surface_cloud(self, stamp=None):
        cloud = self.volume.extract_point_cloud()
        points = np.asarray(cloud.points)
        if points.size == 0:
            return
        colors = np.asarray(cloud.colors)
        if colors.shape != points.shape:
            colors = np.zeros_like(points)
        rgb8 = np.clip(colors * 255.0, 0, 255).astype(np.uint32)
        packed = (rgb8[:, 0] << 16) | (rgb8[:, 1] << 8) | rgb8[:, 2]
        packed_float = packed.astype(np.uint32).view(np.float32)
        cloud_rows = np.empty(points.shape[0], dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        cloud_rows["x"] = points[:, 0]
        cloud_rows["y"] = points[:, 1]
        cloud_rows["z"] = points[:, 2]
        cloud_rows["rgb"] = packed_float
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]
        header = Header(stamp=stamp or rospy.Time.now(), frame_id=self.world_frame)
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = bool(np.isfinite(points).all())
        msg.data = cloud_rows.tobytes()
        self.cloud_pub.publish(msg)

    def _set_capture(self, enabled):
        self.capture_enabled = bool(enabled)
        self.capture_state_pub.publish(Bool(data=self.capture_enabled))
        if self.capture_enabled and not self.roi_locked:
            self.last_status = "capture requested; waiting for locked table ROI"
        elif self.capture_enabled:
            self.last_status = "capture ON"
        else:
            self.last_status = "capture OFF; move to the next view"
        rospy.loginfo("[table_fine_mapper] %s", self.last_status)

    def _capture_topic_callback(self, msg):
        with self.lock:
            self._set_capture(msg.data)

    def _capture_service(self, request):
        with self.lock:
            self._set_capture(request.data)
        return SetBoolResponse(success=True, message=self.last_status)

    def _reset_callback(self, _request):
        with self.lock:
            self.volume = self._new_volume()
            self.roi_min = None
            self.roi_max = None
            self.table_height = None
            self.stable_count = 0
            self.roi_locked = False
            self.capture_enabled = False
            self.integrated_frames = 0
            self.last_localization_time = rospy.Time(0)
            self.last_integration_time = rospy.Time(0)
            self.last_publish_time = rospy.Time(0)
            self.last_status = "map reset; waiting for table"
            self.capture_state_pub.publish(Bool(data=False))
            delete_marker = Marker()
            delete_marker.header.frame_id = self.world_frame
            delete_marker.ns = "table_fine_mapping"
            delete_marker.id = 0
            delete_marker.action = Marker.DELETE
            self.roi_pub.publish(delete_marker)
        return TriggerResponse(success=True, message="TSDF and table ROI reset")

    def _save_callback(self, _request):
        with self.lock:
            if self.integrated_frames == 0:
                return TriggerResponse(success=False, message="no TSDF frames integrated")
            os.makedirs(self.output_directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base = os.path.join(self.output_directory, "table_%s" % timestamp)
            mesh_path = base + ".ply"
            cloud_path = base + ".pcd"
            mesh = self.volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            cloud = self.volume.extract_point_cloud()
            mesh_ok = o3d.io.write_triangle_mesh(mesh_path, mesh, write_ascii=False)
            cloud_ok = o3d.io.write_point_cloud(cloud_path, cloud, write_ascii=False)
            self._publish_surface_cloud()
            if not mesh_ok or not cloud_ok:
                return TriggerResponse(success=False,
                                       message="Open3D failed to write one or more files")
            message = "saved %s and %s (%d frames)" % (
                mesh_path, cloud_path, self.integrated_frames)
            rospy.loginfo("[table_fine_mapper] %s", message)
            return TriggerResponse(success=True, message=message)

    def _status_timer(self, _event):
        with self.lock:
            status = "%s | roi_locked=%s capture=%s frames=%d" % (
                self.last_status, self.roi_locked, self.capture_enabled,
                self.integrated_frames)
            self.status_pub.publish(String(data=status))


if __name__ == "__main__":
    try:
        TableFineMapper()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError):
        pass
