#!/usr/bin/env python3
"""
shape_drawer.py — ROS 2 node that commands an xArm7 to trace 2D shapes in 3D space.

Uses MoveIt 2 ROS 2 service/action clients directly (no moveit_commander):
  - /compute_cartesian_path  (GetCartesianPath service)  — Cartesian path planning
  - /execute_trajectory      (ExecuteTrajectory action)  — trajectory execution
  - /move_action             (MoveGroup action)          — single-pose joint-space planning

Pipeline:
  1. Load shape definitions from a YAML config file
  2. Transform each shape's 2D vertices into 3D waypoints via rotation + translation
  3. Publish RViz LINE_STRIP markers for visualisation
  4. For each shape: move to start → compute Cartesian path → execute trajectory
"""

# ---------------------------------------------------------------------------
#  Standard library imports
# ---------------------------------------------------------------------------
import math          # Trigonometric functions, pi
import os            # File path operations
import threading     # Timer thread for EE trail
import time          # Sleep / delay for timing
import traceback     # Full exception stack traces
import numpy as np   # Matrix operations (rotation, translation)
import yaml          # YAML config file parser

# ---------------------------------------------------------------------------
#  ROS 2 core
# ---------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
#  ROS 2 message types
# ---------------------------------------------------------------------------
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
#  MoveIt 2 messages, services, and actions
# ---------------------------------------------------------------------------
from moveit_msgs.msg import (
    RobotState,
    Constraints,
    MoveItErrorCodes,
    MotionPlanRequest,
    PlanningOptions,
    RobotTrajectory,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup, ExecuteTrajectory


# ===================================================================
#  Math helpers
# ===================================================================

def rpy_to_rotation_matrix(roll, pitch, yaw):
    """Convert Roll-Pitch-Yaw Euler angles to a 3x3 rotation matrix (ZYX order)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr            ],
    ])


def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w).

    Uses the Shepperd method to avoid numerical instability near 180-degree
    rotations by selecting the largest diagonal element.
    """
    t = R[0,0] + R[1,1] + R[2,2]  # matrix trace
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s; x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s; z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s; x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s; z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s; x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s; z = 0.25 * s
    return (x, y, z, w)


def compute_ee_orientation(rpy):
    """Compute end-effector orientation for the drawing plane.

    The EE must face the drawing surface, so we apply an additional 180-degree
    rotation about X (R_flip) to point the tool downward into the plane.
    """
    R_shape = rpy_to_rotation_matrix(*rpy)
    R_flip  = rpy_to_rotation_matrix(math.pi, 0.0, 0.0)
    return rotation_matrix_to_quaternion(R_shape @ R_flip)


def interpolate_arc(center, radius, start_angle, end_angle, n=48):
    """Generate evenly spaced 2D points along a circular arc.

    Args:
        center:      [u, v] arc centre on the 2D plane
        radius:      arc radius in metres
        start_angle: start angle in radians
        end_angle:   end angle in radians (2*pi for a full circle)
        n:           number of interpolation points (higher = smoother)

    Returns:
        List of [u, v] coordinate pairs.
    """
    return [
        [center[0] + radius * math.cos(a),
         center[1] + radius * math.sin(a)]
        for a in np.linspace(start_angle, end_angle, n)
    ]


# ===================================================================
#  Constants — xArm7 robot configuration
# ===================================================================

GROUP_NAME     = 'xarm7'       # MoveIt planning group (7-DOF arm)
EE_LINK        = 'link_eef'    # End-effector link name
BASE_FRAME     = 'link_base'   # Robot base frame
PLANNING_FRAME = 'link_base'   # Motion planning reference frame


# ===================================================================
#  ShapeDrawer ROS 2 Node
# ===================================================================

class ShapeDrawer(Node):
    """ROS 2 node that reads YAML shape definitions and controls the xArm7
    to draw each shape in 3D space using MoveIt 2 interfaces."""

    # Colours for RViz markers (cycled for each shape)
    COLOURS = [
        ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),  # red
        ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0),  # green
        ColorRGBA(r=0.3, g=0.3, b=1.0, a=1.0),  # blue
        ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),  # yellow
        ColorRGBA(r=0.9, g=0.2, b=0.9, a=1.0),  # magenta
    ]

    def __init__(self):
        super().__init__('shape_drawer')

        # ROS parameter: path to YAML config file
        self.declare_parameter('config_file', '')

        # Reentrant callback group allows concurrent callbacks
        # (e.g. joint state updates while waiting for service responses)
        self.cbg = ReentrantCallbackGroup()

        # Publisher: RViz shape markers
        self.marker_pub = self.create_publisher(MarkerArray, 'shape_markers', 10)

        # Publisher: EE trail marker (shows where the end-effector has been)
        self.trail_pub = self.create_publisher(Marker, 'ee_trail', 10)
        self._trail_points = []   # collected EE positions
        self._trail_active = False

        # Subscriber: current joint positions (needed to build RobotState)
        self._joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10,
            callback_group=self.cbg)

        # Service client: Cartesian path computation
        self.cartesian_cli = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self.cbg)

        # Action client: trajectory execution
        self.execute_cli = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.cbg)

        # Action client: MoveGroup (joint-space single-pose planning)
        self.movegroup_cli = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.cbg)

        self.get_logger().info('ShapeDrawer: waiting for MoveIt services …')

    def _js_cb(self, msg: JointState):
        """Callback: store the latest joint state and record EE trail."""
        self._joint_state = msg

    # ------------------------------------------------------------------
    #  Helper methods
    # ------------------------------------------------------------------

    def _wait_services(self, timeout=30.0):
        """Block until all MoveIt services and action servers are available."""
        ok = self.cartesian_cli.wait_for_service(timeout_sec=timeout)
        if not ok:
            self.get_logger().error('GetCartesianPath service not available!')
            return False
        ok = self.execute_cli.wait_for_server(timeout_sec=timeout)
        if not ok:
            self.get_logger().error('ExecuteTrajectory action not available!')
            return False
        ok = self.movegroup_cli.wait_for_server(timeout_sec=timeout)
        if not ok:
            self.get_logger().error('MoveGroup action not available!')
            return False
        self.get_logger().info('All MoveIt services/actions are ready.')
        return True

    def _current_robot_state(self) -> RobotState:
        """Build a RobotState message from the latest joint state."""
        rs = RobotState()
        if self._joint_state:
            rs.joint_state = self._joint_state
        return rs

    def _make_pose(self, pos, quat) -> Pose:
        """Create a Pose message from position [x,y,z] and quaternion (x,y,z,w)."""
        p = Pose()
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        return p

    # ------------------------------------------------------------------
    #  Cartesian path planning
    # ------------------------------------------------------------------

    def _compute_cartesian(self, waypoints, eef_step=0.005):
        """Call GetCartesianPath service to plan a straight-line EE path.

        Args:
            waypoints: list of Pose messages (3D waypoints)
            eef_step:  max distance between interpolated points (metres)

        Returns:
            (RobotTrajectory, fraction) — the planned trajectory and the
            fraction of the path that was successfully planned (1.0 = 100%).
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = GROUP_NAME
        req.link_name = EE_LINK
        req.start_state = self._current_robot_state()
        req.waypoints = waypoints
        req.max_step = eef_step
        req.jump_threshold = 0.0      # disable jump detection
        req.avoid_collisions = True

        future = self.cartesian_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            return None, 0.0
        res = future.result()
        return res.solution, res.fraction

    # ------------------------------------------------------------------
    #  Trajectory execution
    # ------------------------------------------------------------------

    def _execute(self, trajectory: RobotTrajectory):
        """Send a planned trajectory to the ExecuteTrajectory action server.

        Returns True if execution succeeds.
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        # Step 1: send goal asynchronously
        future = self.execute_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected')
            return False

        # Step 2: wait for execution to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error('ExecuteTrajectory returned None')
            return False

        ok = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().error(
                f'ExecuteTrajectory error: {result.result.error_code.val}')
        return ok

    # ------------------------------------------------------------------
    #  Single-pose motion (MoveGroup action)
    # ------------------------------------------------------------------

    def _move_to_pose(self, pose: Pose):
        """Plan and execute a joint-space motion to the target pose.

        Uses the MoveGroup action with OMPL RRTConnect planner.
        Supports automatic replanning on failure.
        """
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        # Define workspace bounds (2m cube centred at origin)
        ws = WorkspaceParameters()
        ws.header.frame_id = PLANNING_FRAME
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  1.0
        req.workspace_parameters = ws

        # Build goal constraints (position + orientation)
        from moveit_msgs.msg import (
            PositionConstraint, OrientationConstraint,
            BoundingVolume,
        )
        from shape_msgs.msg import SolidPrimitive

        # Position constraint: EE must reach target within 1mm sphere
        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = EE_LINK
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.001]  # 1mm tolerance
        bv = BoundingVolume()
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        # Orientation constraint: EE must match target within 0.01 rad
        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = EE_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)
        goal_constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(goal_constraints)
        req.start_state = self._current_robot_state()

        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False   # plan and execute
        goal.planning_options.replan = True        # auto-replan on failure
        goal.planning_options.replan_attempts = 3

        future = self.movegroup_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error('MoveGroup goal rejected')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None:
            return False
        return res.result.error_code.val == MoveItErrorCodes.SUCCESS

    # ------------------------------------------------------------------
    #  YAML loading and shape processing
    # ------------------------------------------------------------------

    def load_shapes(self, path):
        """Load shape definitions from a YAML file.

        Expected format:
            shapes:
              - name: "..."
                type: "polygon" | "arc"
                ...
        """
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        shapes = data.get('shapes', [])
        if not isinstance(shapes, list):
            raise ValueError('Config file must contain a top-level "shapes" list')
        return shapes

    def _validate_shape(self, shape, index):
        """Validate a single shape definition, raising ValueError on problems."""
        if not isinstance(shape, dict):
            raise ValueError(f'Shape #{index} must be a mapping')

        name = shape.get('name', f'shape_{index}')
        stype = shape.get('type', 'polygon')
        if stype not in ('polygon', 'arc'):
            raise ValueError(f'Shape "{name}" has unsupported type "{stype}"')

        if 'position' not in shape or len(shape['position']) != 3:
            raise ValueError(f'Shape "{name}" must define position: [x, y, z]')

        rpy = shape.get('orientation', [0.0, 0.0, 0.0])
        if len(rpy) != 3:
            raise ValueError(
                f'Shape "{name}" must define orientation as [roll, pitch, yaw]')

        if stype == 'arc':
            num_points = int(shape.get('arc_num_points', 48))
            if num_points < 2:
                raise ValueError(f'Shape "{name}" arc_num_points must be >= 2')
            if float(shape.get('arc_radius', 0.0)) <= 0.0:
                raise ValueError(f'Shape "{name}" arc_radius must be > 0')
            center = shape.get('arc_center', [0.0, 0.0])
            if len(center) != 2:
                raise ValueError(f'Shape "{name}" arc_center must be [u, v]')
            return

        vertices = shape.get('vertices')
        if not isinstance(vertices, list) or len(vertices) < 2:
            raise ValueError(
                f'Shape "{name}" must define at least two 2D vertices')
        if len(vertices[0]) != 2 or any(len(v) != 2 for v in vertices):
            raise ValueError(f'Shape "{name}" vertices must be [u, v] pairs')
        if tuple(vertices[0]) != (0, 0):
            raise ValueError(
                f'Shape "{name}" must start at vertex [0, 0] per challenge spec')

    def transform_vertices(self, shape):
        """Transform 2D shape vertices into 3D waypoints.

        For each 2D point (u, v):
            p_3d = origin + R_shape @ [u, v, 0]

        where origin is the shape's 3D position and R_shape is the rotation
        matrix built from the [roll, pitch, yaw] orientation.

        Returns a list of dicts: [{'pos': [x,y,z], 'quat': (qx,qy,qz,qw)}, ...]
        """
        origin = np.array(shape['position'], dtype=float)
        rpy    = shape.get('orientation', [0.0, 0.0, 0.0])
        stype  = shape.get('type', 'polygon')

        # Generate 2D points depending on shape type
        if stype == 'arc':
            pts2d = interpolate_arc(
                shape.get('arc_center', [0, 0]),
                shape.get('arc_radius', 0.05),
                shape.get('arc_start_angle', 0),
                shape.get('arc_end_angle', 2 * math.pi),
                shape.get('arc_num_points', 48))
        else:
            pts2d = [list(v) for v in shape['vertices']]

        # Auto-close shape if needed
        if shape.get('closed', True) and pts2d and pts2d[0] != pts2d[-1]:
            pts2d.append(pts2d[0])

        R    = rpy_to_rotation_matrix(*rpy)
        quat = compute_ee_orientation(rpy)

        return [{'pos': (origin + R @ np.array([u, v, 0.0])).tolist(),
                 'quat': quat}
                for u, v in pts2d]

    # ------------------------------------------------------------------
    #  RViz marker publishing
    # ------------------------------------------------------------------

    def publish_markers(self, all_shapes):
        """Publish LINE_STRIP markers in RViz for all shapes."""
        ma = MarkerArray()
        for i, (name, wps, _blend) in enumerate(all_shapes):
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'shapes'; m.id = i
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.005  # line width 5mm
            m.color = self.COLOURS[i % len(self.COLOURS)]
            m.pose.orientation.w = 1.0
            for wp in wps:
                m.points.append(Point(
                    x=wp['pos'][0], y=wp['pos'][1], z=wp['pos'][2]))
            ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info(f'Published {len(ma.markers)} shape markers')

    # ------------------------------------------------------------------
    #  Execute a single shape
    # ------------------------------------------------------------------

    def _publish_trail(self):
        """Publish the accumulated EE trail as a LINE_STRIP marker."""
        if not self._trail_points:
            return
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'ee_trail'; m.id = 999
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.006  # 6mm bright trail line
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # bright green
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0  # persist forever
        for pt in self._trail_points:
            m.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))
        self.trail_pub.publish(m)

    def _add_trail_waypoints(self, waypoints):
        """Add waypoint positions to the trail for visualization."""
        for wp in waypoints:
            self._trail_points.append(wp['pos'])
        self._publish_trail()

    def _clear_trail(self):
        """Clear the EE trail marker."""
        self._trail_points = []
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'ee_trail'; m.id = 999
        m.action = Marker.DELETE
        self.trail_pub.publish(m)

    def execute_shape(self, name, waypoints, blend=False):
        """Draw one complete shape.

        1. Move to the first waypoint (joint-space planning)
        2. Compute Cartesian path through remaining waypoints
        3. Execute the trajectory
        4. If Cartesian coverage < 80%, fall back to per-waypoint moves
        """
        self.get_logger().info(
            f'▶ "{name}" — {len(waypoints)} waypoints')

        poses = [self._make_pose(wp['pos'], wp['quat']) for wp in waypoints]
        if not poses:
            self.get_logger().warn(f'  Skipping "{name}" — no waypoints')
            return False

        # Clear previous trail, start fresh for this shape
        self._clear_trail()

        # Step 1: move to start position
        self.get_logger().info(f'  Moving to start of "{name}" …')
        if not self._move_to_pose(poses[0]):
            self.get_logger().error(f'  ✗ Cannot reach start of "{name}"')
            return False
        time.sleep(1.0)  # allow joint_state to update

        # Record first waypoint in trail
        self._trail_points.append(waypoints[0]['pos'])

        # Step 2: Cartesian path through remaining waypoints
        if len(poses) > 1:
            # Pre-publish full trail so user can see the target shape
            self._add_trail_waypoints(waypoints)

            eef_step = 0.002 if blend else 0.005
            traj, frac = self._compute_cartesian(poses[1:], eef_step)
            self.get_logger().info(f'  Cartesian plan: {frac*100:.1f}%')

            if traj and frac > 0.80:
                ok = self._execute(traj)
                self.get_logger().info(
                    f'  {"✓" if ok else "✗"} "{name}" executed')
                return ok
            else:
                # Fallback: move to each waypoint individually
                self.get_logger().warn(
                    f'  Low fraction ({frac:.0%}); falling back …')
                for j, p in enumerate(poses[1:], 1):
                    if not self._move_to_pose(p):
                        self.get_logger().error(
                            f'  ✗ Fallback move {j} failed for "{name}"')
                        return False
                    time.sleep(0.3)
                self.get_logger().info(f'  ✓ "{name}" done (fallback)')
        return True

    # ------------------------------------------------------------------
    #  Main entry point
    # ------------------------------------------------------------------

    def run(self):
        """Main logic: load → validate → transform → visualise → execute."""
        # Resolve config file path
        cfg = self.get_parameter('config_file').value
        if not cfg:
            try:
                pkg = get_package_share_directory('avatar_challenge')
                cfg = os.path.join(pkg, 'config', 'shapes.yaml')
            except Exception:
                cfg = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    '..', 'config', 'shapes.yaml')

        self.get_logger().info(f'Config: {cfg}')
        shapes = self.load_shapes(cfg)
        self.get_logger().info(f'Loaded {len(shapes)} shape(s)')

        # Validate and transform all shapes
        all_shapes = []
        for index, s in enumerate(shapes, 1):
            self._validate_shape(s, index)
            nm  = s.get('name', 'unnamed')
            wps = self.transform_vertices(s)
            all_shapes.append((nm, wps, s.get('blend', False)))
            self.get_logger().info(f'  → "{nm}": {len(wps)} waypoints')

        # Wait for MoveIt services and joint state
        if not self._wait_services(timeout=30.0):
            return
        while self._joint_state is None:
            self.get_logger().info('Waiting for /joint_states …')
            rclpy.spin_once(self, timeout_sec=1.0)

        # Publish RViz markers
        self.publish_markers(all_shapes)

        # Execute each shape
        failed_shapes = []
        for nm, wps, blend in all_shapes:
            if not self.execute_shape(nm, wps, blend):
                failed_shapes.append(nm)

        if failed_shapes:
            self.get_logger().error(
                f'Completed with failures: {", ".join(failed_shapes)}')
            return

        self.get_logger().info('✓ All shapes executed!')


# ===================================================================
#  Entry point
# ===================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDrawer()
    time.sleep(2.0)  # allow other nodes to initialise
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f'Fatal: {exc}')
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
