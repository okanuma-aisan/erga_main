import rclpy
import time
from canlib import canlib
from rclpy.node import Node

from std_msgs.msg import String, Int8, Float32, Float64, Bool
from geometry_msgs.msg import Vector3, Twist, TwistWithCovariance, TwistWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport

def can_id_to_pgn(can_id):
    return (can_id >> 8) & 0x3FFFF

class VehicleInfoPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_info_publisher')
        self.declare_parameter("serial_no", value=97692)
        self.declare_parameter("use_selected_gear", value=True)
        self.use_selected_gear = self.get_parameter("use_selected_gear").value

        self.selected_gear_publisher_ = self.create_publisher(Int8, 'selected_gear', 10)
        self.current_gear_publisher_ = self.create_publisher(Int8, 'current_gear', 10)
        self.engine_rpm_publisher_ = self.create_publisher(Float32, 'engine_rpm', 10)
        self.wheel_velocity_avg_publisher_ = self.create_publisher(TwistWithCovarianceStamped, 'absolute_speed/rear_axle', 10)
        self.wheel_velocity_front_publisher_ = self.create_publisher(TwistWithCovarianceStamped, 'absolute_speed/front_axle', 10)
        #self.wheel_velocity_rear_publisher_ = self.create_publisher(TwistWithCovarianceStamped, 'absolute_speed/rear_axle', 10)
        self.velocity_report_publisher_ = self.create_publisher(VelocityReport, 'velocity_status', 10)
        self.velocity_report_front_publisher_ = self.create_publisher(VelocityReport, 'velocity_status_front', 10)
        self.velocity_report_rear_publisher_ = self.create_publisher(VelocityReport, 'velocity_status_rear', 10)

        self.wheel_fl_publisher_ = self.create_publisher(Float32, 'relative_speed/front/left', 10)
        self.wheel_fr_publisher_ = self.create_publisher(Float32, 'relative_speed/front/right', 10)
        self.wheel_rl_publisher_ = self.create_publisher(Float32, 'relative_speed/rear/left', 10)
        self.wheel_rr_publisher_ = self.create_publisher(Float32, 'relative_speed/rear/right', 10)

        serial_no = self.get_parameter("serial_no").value
        while True:
            self._logger.info(f"Starting search for channel with serial no. {serial_no}")
            self.my_channel = -1
            for i in range(0, canlib.getNumberOfChannels()):
                chd = canlib.ChannelData(i)
                if chd.card_serial_no == serial_no:
                    self.my_channel = i
                    break
            if self.my_channel == -1:
                self._logger.error(f"Unable to find channel with device that matches serial no. {serial_no}, retrying after 3s ...")
                time.sleep(3)
                continue
            self._logger.info(f"Found matching device at channel no. {self.my_channel}")
            break

        with canlib.openChannel(channel=self.my_channel) as ch:
            self._logger.info("Setting baud rate to 500k")
            ch.setBusParams(canlib.canBITRATE_500K)
            self._logger.info("Opening bus and starting data acquisition ...")
            ch.busOn()

            self.last_received_gear = 0
            while True:
                frame = ch.read(timeout=100)
                pgn = can_id_to_pgn(frame.id)
                data = frame.data

                if pgn == 61445:
                    selected_gear = data[0] - 125
                    selected_gear_msg = Int8()
                    selected_gear_msg.data = selected_gear
                    self.selected_gear_publisher_.publish(selected_gear_msg)

                    current_gear = data[3] - 125
                    current_gear_msg = Int8()
                    current_gear_msg.data = current_gear
                    self.current_gear_publisher_.publish(current_gear_msg)

                    if self.use_selected_gear: # we use selected gear by default here because "current gear" switch is very delayed; bus is already moving rearwards before it switches
                        self.last_received_gear = selected_gear
                    else:
                        self.last_received_gear = current_gear
                        
                    continue

                if pgn == 61444:
                    engine_speed = int.from_bytes(data[3:5], 'little', signed=False) * 0.125
                    engine_speed_msg = Float32()
                    engine_speed_msg.data = engine_speed
                    self.engine_rpm_publisher_.publish(engine_speed_msg)
                    continue

                if pgn == 65215:
                    front_axle_left = (data[2] - 125) * 0.0625
                    front_axle_right = (data[3] - 125) * 0.0625

                    rear_axle_left = (data[4] - 125) * 0.0625
                    rear_axle_right = (data[5] - 125) * 0.0625

                    front_axle = int.from_bytes(data[0:2], 'little', signed=False) * 0.00390625
                    rear_axle = (rear_axle_left + rear_axle_right) / 2

                    speed_avg = front_axle + rear_axle
                    speed_avg_m = speed_avg / 3.6 # convert from km/h to m/s
                    front_axle_m = front_axle / 3.6
                    if self.last_received_gear < 0:
                        speed_avg_m = speed_avg_m * -1
                        front_axle_m = front_axle_m * -1

                    wheel_velocity_msg = TwistWithCovarianceStamped()
                    wheel_velocity_msg.header.stamp = self.get_clock().now().to_msg()
                    wheel_velocity_msg.twist.twist.linear = Vector3(x = speed_avg_m)
                    wheel_velocity_msg.twist.covariance[0] = 0.062
                    wheel_velocity_msg.twist.covariance[7] = 0.062
                    wheel_velocity_msg.twist.covariance[14] = 0.062
                    wheel_velocity_msg.twist.covariance[21] = -1
                    self.wheel_velocity_avg_publisher_.publish(wheel_velocity_msg)

                    velocity_report_msg = VelocityReport()
                    velocity_report_msg.header.stamp = self.get_clock().now().to_msg()
                    velocity_report_msg.longitudinal_velocity = speed_avg_m
                    velocity_report_msg.lateral_velocity = 0.0
                    velocity_report_msg.heading_rate = 0.0
                    self.velocity_report_publisher_.publish(velocity_report_msg)

                    velocity_report_front_msg = VelocityReport()
                    velocity_report_front_msg.header.stamp = self.get_clock().now().to_msg()
                    velocity_report_front_msg.longitudinal_velocity = front_axle / 3.6
                    velocity_report_front_msg.lateral_velocity = 0.0
                    velocity_report_front_msg.heading_rate = 0.0
                    self.velocity_report_front_publisher_.publish(velocity_report_front_msg)

                    velocity_report_rear_msg = VelocityReport()
                    velocity_report_rear_msg.header.stamp = self.get_clock().now().to_msg()
                    velocity_report_rear_msg.longitudinal_velocity = rear_axle / 3.6
                    velocity_report_rear_msg.lateral_velocity = 0.0
                    velocity_report_rear_msg.heading_rate = 0.0
                    self.velocity_report_rear_publisher_.publish(velocity_report_rear_msg)

                    wheel_velocity_front_msg = TwistWithCovarianceStamped()
                    wheel_velocity_front_msg.header.stamp = self.get_clock().now().to_msg()
                    wheel_velocity_front_msg.twist.twist.linear = Vector3(x = front_axle_m)
                    wheel_velocity_front_msg.twist.covariance[0] = 0.062
                    wheel_velocity_front_msg.twist.covariance[7] = 0.062
                    wheel_velocity_front_msg.twist.covariance[14] = 0.062
                    wheel_velocity_front_msg.twist.covariance[21] = -1
                    self.wheel_velocity_front_publisher_.publish(wheel_velocity_front_msg)

                    front_axle_left_msg = Float32()
                    front_axle_left_msg.data = front_axle_left
                    self.wheel_fl_publisher_.publish(front_axle_left_msg)

                    front_axle_right_msg = Float32()
                    front_axle_right_msg.data = front_axle_right
                    self.wheel_fr_publisher_.publish(front_axle_right_msg)

                    rear_axle_left_msg = Float32()
                    rear_axle_left_msg.data = rear_axle_left
                    self.wheel_rl_publisher_.publish(rear_axle_left_msg)

                    rear_axle_right_msg = Float32()
                    rear_axle_right_msg.data = rear_axle_right
                    self.wheel_rr_publisher_.publish(rear_axle_right_msg)
                    continue
     
def main(args=None):
    rclpy.init(args=args)

    vehicle_info_publisher = VehicleInfoPublisher()
    rclpy.spin(vehicle_info_publisher)

    vehicle_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
