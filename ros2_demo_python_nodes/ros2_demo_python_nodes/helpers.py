
import struct
import math
from transforms3d.euler import quat2euler
import sys

from rclpy.time import Duration
import tf2_ros
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import PoseStamped
import std_msgs.msg as std_msgs
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
import numpy as np

sys.dont_write_bytecode = True


def generate_points(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()
    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


# The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later:
# https://github.com/ros2/common_interfaces
_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(
        cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                'Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


class SimpleNavHelpers():
    def __init__(self, node):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            buffer=self.tf_buffer,  node=node)

    def pose_euclidean_dist(self, a, b):

        return math.sqrt((a.position.x - b.position.x) ** 2 +
                         (a.position.y - b.position.y) ** 2 +
                         (a.position.z - b.position.z) ** 2)

    def get_curr_robot_pose(self, now, logger):
        curr_robot_pose = PoseStamped()
        curr_robot_pose.header.frame_id = "map"
        curr_robot_pose.header.stamp = now.to_msg()
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", time=now, timeout=Duration(seconds=0.5))
            curr_robot_pose.pose.position.x = transform.transform.translation.x
            curr_robot_pose.pose.position.y = transform.transform.translation.y
            curr_robot_pose.pose.position.z = transform.transform.translation.z
            curr_robot_pose.pose.orientation = transform.transform.rotation

        except (tf2_ros.TypeException, tf2_ros.NotImplementedException):
            logger.info("Failed to get current robot pose")
        return curr_robot_pose

    def clip(self, val, min_, max_):
        return min_ if val < min_ else max_ if val > max_ else val


class PurePursuitController():
    def __init__(self, linear_k, angular_k, linear_max, angular_max):
        self.linear_k = linear_k
        self.angular_k = angular_k
        self.linear_max = linear_max
        self.angular_max = angular_max

    def clip(self, val, min_, max_):
        return min_ if val < min_ else max_ if val > max_ else val

    def compute_error(self, curr_robot_pose,  curr_goal_pose, dist_to_goal_satisfied):

        robot_quat_exp = [curr_robot_pose.pose.orientation.w, curr_robot_pose.pose.orientation.x,
                          curr_robot_pose.pose.orientation.y, curr_robot_pose.pose.orientation.z]
        robot_euler = quat2euler(robot_quat_exp)

        goal_quat_exp = [curr_goal_pose.pose.orientation.w, curr_goal_pose.pose.orientation.x,
                         curr_goal_pose.pose.orientation.y,  curr_goal_pose.pose.orientation.z]
        goal_euler = quat2euler(goal_quat_exp)

        robot_roll, robot_pitch, robot_yaw = robot_euler[0], robot_euler[1], robot_euler[2]
        goal_roll, goal_pitch, goal_yaw = goal_euler[0], goal_euler[1], goal_euler[2]

        err_local = [curr_goal_pose.pose.position.x - curr_robot_pose.pose.position.x,
                     curr_goal_pose.pose.position.y - curr_robot_pose.pose.position.y,
                     robot_yaw - goal_yaw]

        if dist_to_goal_satisfied:
            rot_error = goal_yaw - robot_yaw
        else:
            rot_error = math.atan2(err_local[1], err_local[0]) - robot_yaw

        dist_error = math.sqrt(err_local[0]**2 + err_local[1]**2)

        return dist_error, rot_error

    def compute_velocities(self, curr_robot_pose,  curr_goal_pose, dist_to_goal_satisfied):
        robot_quat_exp = [curr_robot_pose.pose.orientation.w, curr_robot_pose.pose.orientation.x,
                          curr_robot_pose.pose.orientation.y, curr_robot_pose.pose.orientation.z]
        robot_euler = quat2euler(robot_quat_exp)
        goal_quat_exp = [curr_goal_pose.pose.orientation.w, curr_goal_pose.pose.orientation.x,
                         curr_goal_pose.pose.orientation.y,  curr_goal_pose.pose.orientation.z]
        goal_euler = quat2euler(goal_quat_exp)

        robot_roll, robot_pitch, robot_yaw = robot_euler[0], robot_euler[1], robot_euler[2]
        goal_roll, goal_pitch, goal_yaw = goal_euler[0], goal_euler[1], goal_euler[2]

        err_local = [curr_goal_pose.pose.position.x - curr_robot_pose.pose.position.x,
                     curr_goal_pose.pose.position.y - curr_robot_pose.pose.position.y,
                     robot_yaw - goal_yaw]

        k1 = self.linear_k
        k2 = self.angular_k
        max_v = self.linear_max
        max_w = self.angular_max

        v_in = k1 * math.sqrt(err_local[0]**2 + err_local[1]**2)

        if dist_to_goal_satisfied:
            w_in = k2 * (goal_yaw - robot_yaw)
        else:
            w_in = k2 * (math.atan2(err_local[1], err_local[0]) - robot_yaw)

        v_in = self.clip(v_in, -max_v, max_v)
        w_in = self.clip(w_in, -max_w, max_w)

        return v_in, w_in
