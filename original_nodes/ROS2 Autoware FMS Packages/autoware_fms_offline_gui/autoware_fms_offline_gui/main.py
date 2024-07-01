#!/usr/bin/env python3
# Add all files in directory to PATH
import sys, pathlib
import typing
lib_path = pathlib.Path(__file__).parent.__str__()
sys.path.append(lib_path)

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import LaneletRoute
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_fms_msgs.msg import Schedule, Task, Place, Tag, PlaceArray
from tier4_map_msgs.msg import MapProjectorInfo

# GUI Library Imports
from PyQt5.QtCore import Qt, QObject, QThread, pyqtSignal, QRunnable, QThreadPool, QTimer
from PyQt5.QtGui import QCursor
from PyQt5 import QtWidgets, uic

import pathlib, csv, time, math, random
import geometric_utils, py_geographic_lib

#def iso_timestamp_to_ros(iso_timestamp: str) -> Time:
#    datetime_obj = dateutil.parser.isoparse(iso_timestamp)
#    output_msg = Time()
#    output_msg.sec = int(datetime_obj.timestamp())
#    output_msg.nanosec = datetime_obj.microsecond * 1000 # convert from microsecond to nanosecond
#    return output_msg

class ROSNode(Node):

    def __init__(self):
        super().__init__('autoware_fms_offline_gui')
        qos_profile = QoSProfile( # allow subscribers to receive last message, even if not actively publishing
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.route_subscriber_ = self.create_subscription(LaneletRoute, '/planning/mission_planning/route', self.route_callback_, qos_profile)
        self.operation_mode_subscriber_ = self.create_subscription(OperationModeState, '/system/operation_mode/state', self.operation_mode_callback_, qos_profile)
        self.route_state_subscriber_ = self.create_subscription(RouteState, '/planning/mission_planning/route_state', self.route_state_callback_, qos_profile)
        self.kinematic_state_subscriber_ = self.create_subscription(Odometry, '/localization/kinematic_state', self.kinematic_state_callback_, qos_profile)
        self.operation_mode_client_ = None

        # FMS emulation
        self.map_projector_info_ = None
        self.map_projector_info_subscriber_ = self.create_subscription(MapProjectorInfo, '/map/map_projector_info', self.map_projector_info_callback_, qos_profile=qos_profile)
        self.place_array_publisher_ = self.create_publisher(PlaceArray, '~/places', qos_profile=qos_profile)
        self.schedule_publisher_ = self.create_publisher(Schedule, '~/current_schedule', qos_profile=qos_profile)

    def map_projector_info_callback_(self, msg: MapProjectorInfo) -> None:
        self.map_projector_info_ = msg

    def convert_to_lat_lng(self, x: float, y: float) -> tuple[float, float]:
        if self.map_projector_info_ is None:
            return 0.0, 0.0
        lat, lon = py_geographic_lib.x_y_to_mgrs(x, y, self.map_projector_info_.mgrs_grid, '')
        return lat, lon
    
    def create_task(self, stop_id: int) -> Task:
        msg = Task()

        destination_msg = Place()
        destination_msg.point_id = stop_id
        msg.destination = destination_msg

        return msg
    
    def create_schedule(self, stop_id: int) -> Schedule:
        msg = Schedule()
        msg.tasks = [self.create_task(stop_id)]
        return msg
    
    def publish_schedule(self, stop_id: int) -> None:
        schedule = self.create_schedule(stop_id)
        self.schedule_publisher_.publish(schedule)

    def enable_autonomous_driving(self) -> None:
        if type(self.operation_mode_client_) is None:
            self.operation_mode_client_ = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
            while not self.operation_mode_client_.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
        self.operation_mode_client_request_ = ChangeOperationMode.Request()
        self.future_ = self.operation_mode_client_.call_async(self.operation_mode_client_request_) # switch to autonomous

    def publish_goal(self, x: float, y: float, z: float, yaw: float) -> None:
        send_msg = PoseStamped()
        send_msg.header.stamp = self.get_clock().now().to_msg()
        send_msg.header.frame_id = "map"

        send_msg.pose.position.x = x
        send_msg.pose.position.y = y
        send_msg.pose.position.z = z
        q_x, q_y, q_z, q_w = geometric_utils.rpy_to_quaternion(0.0, 0.0, yaw)
        send_msg.pose.orientation.z = q_z
        send_msg.pose.orientation.w = q_w

        self.goal_publisher_.publish(send_msg)
        #self.get_logger().info(f'Publishing goal #{self.current_goal_}@{time.time()}: {goal_pose}')

    # stubs (will be overwritten)
    def route_callback_(self, msg: LaneletRoute) -> None:
        pass

    def operation_mode_callback_(self, msg: OperationModeState) -> None:
        pass

    def kinematic_state_callback_(self, msg: Odometry) -> None:
        pass

    def route_state_callback_(self, msg: Odometry) -> None:
        pass

class StopPoint:
    def __init__(self, csv_line: list[str]):
        name, x, y, z, yaw, lane_id, stop_point_id = csv_line
        self.name = str(name)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)
        self.lane_id = int(lane_id)
        self.stop_point_id = int(stop_point_id)
        self.qtreewidgetitem = self.create_qtreewidgetitem()
        self.point = geometric_utils.Point(self.x, self.y)

    def create_qtreewidgetitem(self) -> QtWidgets.QTreeWidgetItem:
        row_order = [
            self.stop_point_id,
            self.name,
            self.x,
            self.y, 
            self.z,
            self.yaw
        ]
        row_order = [str(x) for x in row_order] # QTreeWidgetItem expects list of strings
        return QtWidgets.QTreeWidgetItem(row_order)

    def __str__(self) -> str:
        return f"<StopPoint#{self.stop_point_id}({self.name})@{self.x},{self.y}>"

    def __repr__(self) -> str:
        return self.__str__()

class ROSWorkerThread(QObject):
    def __init__(self, parent = None, node = None) -> None:
        super().__init__(parent)
        self.executor_ = MultiThreadedExecutor()
        self.executor_.add_node(node)
    def run(self):
        self.executor_.spin()
        self.executor_.shutdown()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        main_window_path = pathlib.Path(__file__).with_name('main.ui')
        uic.loadUi(main_window_path, self)

        self.statusBar().showMessage("Idle")
        self.stop_points: list[StopPoint] = list()
        
        # these are loaded from main.ui, but are also placed here for development convenience
        self.routes_widget: QtWidgets.QTreeWidget
        self.publish_button: QtWidgets.QPushButton
        self.current_pose_label: QtWidgets.QLabel
        self.current_velocity_label: QtWidgets.QLabel
        self.closest_goal_label: QtWidgets.QLabel
        self.next_goal_label: QtWidgets.QLabel
        self.auto_publisher_state_label: QtWidgets.QLabel
        self.autoware_state_label: QtWidgets.QLabel
        self.route_state_label: QtWidgets.QLabel
        self.auto_publisher_checkbox: QtWidgets.QCheckBox
        self.auto_publisher_enable_driving: QtWidgets.QCheckBox

        self.routes_widget_selection = None
        self.routes_widget.itemClicked.connect(self.routes_widget_item_clicked_callback_)
        self.publish_button.clicked.connect(self.publish_button_callback_)
        self.actionOpen_stop_points_csv.triggered.connect(self.open_file_menu_callback_)

        self.auto_publisher_checkbox.stateChanged.connect(self.auto_publisher_checkbox_callback_)
        self.show()

        self.last_operation_mode_ = None
        self.last_route_state_ = None
        self.current_goal = None
        self.next_goal = None
        
        # Initialize ROS node
        rclpy.init()
        ROSNode.kinematic_state_callback_ = self.kinematic_state_callback_
        ROSNode.operation_mode_callback_ = self.operation_mode_callback_
        ROSNode.route_state_callback_ = self.route_state_callback_
        ROSNode.route_callback_ = self.route_callback_
        self.ros_node_ = ROSNode()
        self.worker_ = ROSWorkerThread(node=self.ros_node_)

        self.auto_publisher_state_label.setText("<font color='green'>Enabled</font>")

        # Initialize ROS executor thread
        self.thread_ = QThread()
        self.worker_.moveToThread(self.thread_)
        self.thread_.started.connect(self.worker_.run)
        self.thread_.start()

    def open_file_menu_callback_(self) -> None:
        new_path = QtWidgets.QFileDialog.getOpenFileName(self, "Open File", "./", "stop_points (*.csv)")
        if len(new_path) < 1:
            return
        if new_path[0] == '':
            return
        path = new_path[0]
        self.load_routes(path)
        pass

    def publish_button_callback_(self) -> None:
        self.publish_button.setDisabled(True)
        self.ros_node_.publish_goal(self.routes_widget_selection.x, self.routes_widget_selection.y, self.routes_widget_selection.z, self.routes_widget_selection.yaw)
        self.ros_node_.publish_schedule(self.routes_widget_selection.stop_point_id)

    def auto_publisher_checkbox_callback_(self, state: int) -> None:
        match state:
            case 0: # Unchecked
                self.auto_publisher_state_label.setText("<font color='red'>Disabled</font>")
            case 2: # Checked
                self.auto_publisher_state_label.setText("<font color='green'>Waiting for arrival</font>")

    def routes_widget_item_clicked_callback_(self, item: QtWidgets.QTreeWidgetItem, column: int) -> None:
        if item is None:
            self.publish_button.setDisabled(True)
            return
        current_selection_idx = self.routes_widget.indexOfTopLevelItem(item)
        self.routes_widget_selection = self.stop_points[current_selection_idx]
        self.publish_button.setDisabled(False)

    def load_routes(self, path: str) -> None:
        stop_points_file = pathlib.Path(path)
        lines = list()
        with stop_points_file.open('r') as fp:
            lines = fp.readlines()
        rows_reader = csv.reader(lines)
        # parse and create list of stop points, minus header
        self.stop_points = [StopPoint(x) for x in rows_reader if rows_reader.line_num != 1]

        # populate selection list
        self.routes_widget.clear()
        self.routes_widget.addTopLevelItems([x.qtreewidgetitem for x in self.stop_points])

    def find_closest_goal(self, current_position: geometric_utils.Point) -> StopPoint:
        distances = [x.point.fast_distance_to_point(current_position) for x in self.stop_points]
        closest_idx = distances.index(min(distances))
        return self.stop_points[closest_idx]
    
    def find_next_goal(self, current_goal: StopPoint) -> StopPoint:
        current_idx = self.stop_points.index(current_goal)
        if (current_idx+1) > (len(self.stop_points)-1):
            return self.stop_points[0]
        return self.stop_points[current_idx+1]
    
    def operation_mode_callback_(self, msg: OperationModeState) -> None:
        if msg.is_in_transition: # ignore transition states
            return
        match msg.mode:
            case OperationModeState.UNKNOWN:
                self.autoware_state_label.setText("<font color='red'>unknown</font>")
            case OperationModeState.STOP:
                self.autoware_state_label.setText("<font color='blue'>Stopped</font>")
            case OperationModeState.AUTONOMOUS:
                self.autoware_state_label.setText("<font color='green'>Autonomous</font>")
        self.last_operation_mode_ = msg.mode
        self.operation_mode_route_state_callback_()
    
    def route_state_callback_(self, msg: RouteState) -> None:
        match msg.state:
            case RouteState.UNKNOWN:
                self.route_state_label.setText("<font color='red'>unknown</font>")
            case RouteState.UNSET:
                self.route_state_label.setText("<font color='yellow'>Unset</font>")
            case RouteState.SET:
                self.route_state_label.setText("<font color='green'>Set</font>")
            case RouteState.ARRIVED:
                self.route_state_label.setText("<font color='blue'>Arrived</font>")
            case RouteState.CHANGING:
                self.route_state_label.setText("<font color='yellow'>Changing</font>")
        self.last_route_state_ = msg.state
        self.operation_mode_route_state_callback_()
        
    def operation_mode_route_state_callback_(self) -> None:
        if self.last_route_state_ is not RouteState.ARRIVED:
            return
        if self.last_operation_mode_ is not OperationModeState.STOP:
            return
        if self.auto_publisher_checkbox.checkState() != 2:
            return
        if self.next_goal is None:
            return
        self.auto_publisher_state_label.setText("<font color='blue'>Publishing goal</font>")
        self.ros_node_.publish_goal(self.next_goal.x, self.next_goal.y, self.next_goal.z, self.next_goal.yaw)
        self.ros_node_.publish_schedule(self.next_goal.stop_point_id)
        if self.auto_publisher_enable_driving.checkState() == 2:
            self.auto_publisher_state_label.setText("<font color='blue'>Switching to autonomous</font>")
        self.auto_publisher_state_label.setText("<font color='green'>Waiting for arrival</font>")

    def route_callback_(self, msg: LaneletRoute) -> None:
        current_goal_position = geometric_utils.Point(msg.goal_pose.position.x, msg.goal_pose.position.y)
        self.current_goal = self.find_closest_goal(current_goal_position)
        self.closest_goal_label.setText(f"<font color='green'>{self.current_goal.stop_point_id}: {self.current_goal.name}</font>")
        self.next_goal = self.find_next_goal(self.current_goal)
        self.next_goal_label.setText(f"<font color='green'>{self.next_goal.stop_point_id}: {self.next_goal.name}</font>")

    def kinematic_state_callback_(self, msg: Odometry) -> None:
        self.current_pose_label.setText(f"<font color='green'>{msg.pose.pose.position.x:0.4f}</font>, <font color='green'>{msg.pose.pose.position.y:0.4f}</font>")
        self.current_velocity_label.setText(f"<font color='green'>{msg.twist.twist.linear.x}</font> m/s")


#if __name__ == '__main__':
#    main()


def main():
    try:
        import sys
        app = QtWidgets.QApplication([])
        mainWindow = MainWindow()
        
        #mainWindow.load_routes("stop_points.csv")
        sys.exit(app.exec_())
    except Exception as e:
        print(e)