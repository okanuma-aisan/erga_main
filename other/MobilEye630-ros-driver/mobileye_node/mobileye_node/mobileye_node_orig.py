import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from mobileye_msgs.msg import *

class MobileyeNode(Node):
    def __init__(self):
        super().__init__('dbw_mobileye_node')
        self.mobPub = self.create_publisher(MobileyeInfo, 'Mobileye_Info', 20)
        #self.canSub = self.create_subscription(Frame, 'received_messages', self.data_pub, 10)
        self.canSub = self.create_subscription(Frame, 'can_tx', self.data_pub, 10)
        self.frame_ok = 0
        self.mobmsg = MobileyeInfo()
        self.obstacle_data = [ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData(),ObstacleData()]
        self.next_lane = [LKAlane(),LKAlane(),LKAlane(),LKAlane(),LKAlane(),LKAlane(),LKAlane(),LKAlane()]
        self.tsr = [TSR(),TSR(),TSR(),TSR(),TSR(),TSR(),TSR()]
        self.tsr_num = 0

    def data_parser(self, data):
        if data.id == 0x766:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
        
            _data = self.frame_data[2]*256+self.frame_data[1]
            if _data > 32767:
                signed_data = (_data - 65536)
            else:
                signed_data = _data
            self.mobmsg.left_lane.position_parameter_c0 = float(signed_data)/256
            self.mobmsg.left_lane.curvature_parameter_c2 = float((self.frame_data[4]*256+self.frame_data[3])-0x7FFF)/1024/1000
            self.mobmsg.left_lane.curvature_derivative_c3 = float((self.frame_data[6]*256+self.frame_data[5])-0x7FFF)/(1<<28)
            self.mobmsg.left_lane.width_marking = self.frame_data[7]*0.01
            self.mobmsg.left_lane.model_degree = self.frame_data[0]//64
            self.mobmsg.left_lane.quality = (self.frame_data[0]%64)//16
            self.mobmsg.left_lane.lane_type = self.frame_data[0]%16
    
        elif data.id == 0x767:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.left_lane.heading_angle_parameter_c1 = float((self.frame_data[1]*256+self.frame_data[0])-0x7FFF)/1024
            self.mobmsg.left_lane.view_range = float((self.frame_data[3]%128)*256+self.frame_data[2])/256
            self.mobmsg.left_lane.view_range_availability = bool(self.frame_data[3]//128)

        elif data.id == 0x768:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            _data = self.frame_data[2]*256+self.frame_data[1]
            if _data > 32767:
                signed_data = (_data - 65536)
            else:
                signed_data = _data
            self.mobmsg.right_lane.position_parameter_c0 = float(signed_data)/256
            self.mobmsg.right_lane.curvature_parameter_c2 = float((self.frame_data[4]*256+self.frame_data[3])-0x7FFF)/1024/1000
            self.mobmsg.right_lane.curvature_derivative_c3 = float((self.frame_data[6]*256+self.frame_data[5])-0x7FFF)/(1<<28)
            self.mobmsg.right_lane.width_marking = self.frame_data[7]*0.01
            self.mobmsg.right_lane.model_degree = self.frame_data[0]//64
            self.mobmsg.right_lane.quality = (self.frame_data[0]%64)//16
            self.mobmsg.right_lane.lane_type = self.frame_data[0]%16
    
        elif data.id == 0x769:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.right_lane.heading_angle_parameter_c1 = float((self.frame_data[1]*256+self.frame_data[0])-0x7FFF)/1024
            self.mobmsg.right_lane.view_range = float((self.frame_data[3]%128)*256+self.frame_data[2])/256
            self.mobmsg.right_lane.view_range_availability = bool(self.frame_data[3]//128)

        elif data.id == 0x76a:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.reference_points.ref_point1_position = float((self.frame_data[1]*256+self.frame_data[0])-0x7FFF)/256
            self.mobmsg.reference_points.ref_point1_distance = float((self.frame_data[3]%128)*256+self.frame_data[2])/256
            self.mobmsg.reference_points.ref_point1_validity = bool(self.frame_data[3]//128)
            self.mobmsg.reference_points.ref_point2_position = float((self.frame_data[5]*256+self.frame_data[4])-0x7FFF)/256
            self.mobmsg.reference_points.ref_point2_distance = float((self.frame_data[7]%128)*256+self.frame_data[6])/256
            self.mobmsg.reference_points.ref_point2_validity = bool(self.frame_data[7]//128)

        elif data.id == 0x76b:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.number_of_next_lane_markers = self.frame_data[0]

        elif data.id >= 0x76c and data.id < 0x76c + 16:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            lane_index = (data.id - 0x76c)//2
            type_of_info = (data.id - 0x76c)%2
            if type_of_info == 0:
                _data = self.frame_data[2]*256+self.frame_data[1]
                if _data > 32767:
                    signed_data = (_data - 65536)
                else:
                    signed_data = _data
                self.next_lane[lane_index].position_parameter_c0 = float(signed_data)/256
                self.next_lane[lane_index].curvature_parameter_c2 = float((self.frame_data[4]*256+self.frame_data[3])-0x7FFF)/1024/1000
                self.next_lane[lane_index].curvature_derivative_c3 = float((self.frame_data[6]*256+self.frame_data[5])-0x7FFF)/(1<<28)
                self.next_lane[lane_index].width_marking = self.frame_data[7]*0.01
                self.next_lane[lane_index].model_degree = self.frame_data[0]//64
                self.next_lane[lane_index].quality = (self.frame_data[0]%64)//16
                self.next_lane[lane_index].lane_type = self.frame_data[0]%16
            else:
                self.next_lane[lane_index].heading_angle_parameter_c1 = float((self.frame_data[1]*256+self.frame_data[0])-0x7FFF)/1024
                self.next_lane[lane_index].view_range = float((self.frame_data[3]%128)*256+self.frame_data[2])/256
                self.next_lane[lane_index].view_range_availability = bool(self.frame_data[3]//128)

        elif data.id == 0x700:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.system_warnings.sound_type = self.frame_data[0]%8
            self.mobmsg.system_warnings.time_indicator = (self.frame_data[0]%32)//8
            self.mobmsg.system_warnings.zero_speed = bool((self.frame_data[1]%64)//32)
            self.mobmsg.system_warnings.headway_valid = bool(self.frame_data[2]%2)
            self.mobmsg.system_warnings.error_valid = bool(self.frame_data[3]%2)
            self.mobmsg.system_warnings.error_code = self.frame_data[3]//2
            self.mobmsg.system_warnings.headway_measurement = (self.frame_data[2]//2)*0.1
            self.mobmsg.system_warnings.ldw_off = bool(self.frame_data[4]%2)
            self.mobmsg.system_warnings.left_ldw_on = bool((self.frame_data[4]%4)//2)
            self.mobmsg.system_warnings.right_ldw_on = bool((self.frame_data[4]%8)//4)
            self.mobmsg.system_warnings.fcw_on = bool((self.frame_data[4]%16)//8)
            self.mobmsg.system_warnings.maintenance = bool((self.frame_data[4]%128)//64)
            self.mobmsg.system_warnings.failsafe = bool(self.frame_data[4]//128)
            self.mobmsg.system_warnings.peds_fcw = bool((self.frame_data[5]%4)//2)
            self.mobmsg.system_warnings.peds_in_dz = bool((self.frame_data[5]%8)//4)
            self.mobmsg.system_warnings.tamper_alert = bool((self.frame_data[5]%64)//32)
            self.mobmsg.system_warnings.tsr_enabled = bool(self.frame_data[5]//128)
            self.mobmsg.system_warnings.tsr_warning_level = self.frame_data[6]%8
            self.mobmsg.system_warnings.headway_warning_level = self.frame_data[7]%4
            self.mobmsg.system_warnings.hw_repeatable_enabled = bool((self.frame_data[7]%8)//4)

        elif data.id == 0x760:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.car_info.high_beam = bool((self.frame_data[0]%64)//32)
            self.mobmsg.car_info.low_beam = bool((self.frame_data[0]%32)//16)
            self.mobmsg.car_info.wipers = bool((self.frame_data[0]%16)//8)
            self.mobmsg.car_info.right_signal = bool((self.frame_data[0]%8)//4)
            self.mobmsg.car_info.left_signal = bool((self.frame_data[0]%4)//2)
            self.mobmsg.car_info.brake_signal = bool(self.frame_data[0]%2)
            self.mobmsg.car_info.high_beam_available = bool((self.frame_data[1]%64)//32)
            self.mobmsg.car_info.low_beam_available = bool((self.frame_data[1]%32)//16)
            self.mobmsg.car_info.wipers_available = bool((self.frame_data[1]%16)//8)
            self.mobmsg.car_info.right_blink_available = bool((self.frame_data[1]%8)//4)
            self.mobmsg.car_info.left_blink_available = bool((self.frame_data[1]%4)//2)
            self.mobmsg.car_info.brake_available = bool(self.frame_data[1]%2)
            self.mobmsg.car_info.speed_available = bool(self.frame_data[1]//128)
            self.mobmsg.car_info.speed = self.frame_data[2]
            self.mobmsg.car_info.shield_plus_settings = self.frame_data[7]

        elif data.id == 0x737:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            _data = self.frame_data[1]*256+self.frame_data[0]
            if _data > 32767:
                signed_data = (_data - 65536)
            else:
                signed_data = _data
            self.mobmsg.lane.lane_curvature = signed_data*3.81*0.000001
            _data = (self.frame_data[3]%16)*256+self.frame_data[2]
            if _data > 2047:
                signed_data = (_data - 4096)
            else:
                signed_data = _data
            self.mobmsg.lane.lane_heading = signed_data*0.0005
            self.mobmsg.lane.ca = bool((self.frame_data[3]%32)//16)
            self.mobmsg.lane.pitch_angle = float((self.frame_data[7]*256+self.frame_data[6])-0x7FFF)/1024/512
            self.mobmsg.lane.yaw_angle = float((self.frame_data[5]*256+self.frame_data[4])-0x7FFF)/1024
            self.mobmsg.lane.right_ldw_availability = bool((self.frame_data[3]%64)//32)
            self.mobmsg.lane.left_ldw_availability = bool((self.frame_data[3]%128)//64)

        elif data.id == 0x738:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            if self.frame_data[0] > 13:
                obstacle_num = 13
            else:
                obstacle_num = self.frame_data[0]
            self.mobmsg.obstacle_status.number_of_obstacles = obstacle_num
            self.mobmsg.obstacle_status.timestamp = self.frame_data[1]
            self.mobmsg.obstacle_status.left_close_rang_cut_in = bool((self.frame_data[2]%8)//4)
            self.mobmsg.obstacle_status.right_close_rang_cut_in = bool((self.frame_data[2]%16)//8)
            self.mobmsg.obstacle_status.go = self.frame_data[2]//16
            self.mobmsg.obstacle_status.close_car = bool(self.frame_data[5]%2)
            self.mobmsg.obstacle_status.failsafe = bool((self.frame_data[5]%32)//2)

        elif data.id >= 0x739 and data.id <= 0x73b+(self.mobmsg.obstacle_status.number_of_obstacles-1)*3:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            obstacle_idx = (data.id - 0x739)//3
            type_of_data = (data.id - 0x739)%3
            if type_of_data == 0:
                self.obstacle_data[obstacle_idx].obstacle_id = self.frame_data[0]
                self.obstacle_data[obstacle_idx].obstacle_position_x = ((self.frame_data[2]%16)*256+self.frame_data[1])*0.0625
                _data = (self.frame_data[4]%4)*256 + self.frame_data[3]
                if _data > 511:
                    signed_data = (_data - 1024)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_position_y = signed_data*0.0625
                _data = (self.frame_data[6]%16)*256+self.frame_data[5]
                if _data > 2047:
                    signed_data = (_data - 4096)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_relative_velocity_x = signed_data*0.0625
                self.obstacle_data[obstacle_idx].obstacle_type = (self.frame_data[6]%128)//16
                self.obstacle_data[obstacle_idx].obstacle_status = self.frame_data[7]%8
                self.obstacle_data[obstacle_idx].obstacle_brake_lights = bool((self.frame_data[7]%16)//8)
                self.obstacle_data[obstacle_idx].cut_in_and_out = self.frame_data[4]//32
                self.obstacle_data[obstacle_idx].blinker_info = (self.frame_data[4]%32)//4
                self.obstacle_data[obstacle_idx].obstacle_valid = self.frame_data[7]//64
            elif type_of_data == 1:
                self.obstacle_data[obstacle_idx].obstacle_length = self.frame_data[0]*0.5
                self.obstacle_data[obstacle_idx].obstacle_width = self.frame_data[1]*0.05
                self.obstacle_data[obstacle_idx].obstacle_age = self.frame_data[2]
                self.obstacle_data[obstacle_idx].obstacle_lane = self.frame_data[3]%4
                self.obstacle_data[obstacle_idx].cipv_flag = (self.frame_data[3]%8)//4
                self.obstacle_data[obstacle_idx].radar_position_x = (self.frame_data[4]*16+self.frame_data[3]//16)*0.0625
                _data = (self.frame_data[6]%16)*256+self.frame_data[5]
                if _data > 2047:
                    signed_data = (_data - 4096)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].radar_velocity_x = signed_data*0.0625
                self.obstacle_data[obstacle_idx].radar_match_confidence = (self.frame_data[6]%128)//16
                self.obstacle_data[obstacle_idx].matched_radar_id = self.frame_data[7]%128
            else:
                _data = self.frame_data[1]*256+self.frame_data[0]
                if _data > 32767:
                    signed_data = (_data - 65536)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_angle_rate = signed_data*0.01
                _data = self.frame_data[3]*256+self.frame_data[2]
                if _data > 32767:
                    signed_data = (_data - 65536)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_scale_change = signed_data*0.0002
                _data = (self.frame_data[5]%4)*256+self.frame_data[4]
                if _data > 511:
                    signed_data = (_data - 1024)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_object_accel_x = signed_data*0.03
                self.obstacle_data[obstacle_idx].obstacle_replaced = bool((self.frame_data[5]%32)//16)
                _data = self.frame_data[7]*256+self.frame_data[6]
                if _data > 32767:
                    signed_data = (_data - 65536)
                else:
                    signed_data = _data
                self.obstacle_data[obstacle_idx].obstacle_angle = signed_data*0.01

        elif data.id == 0x703:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.gyro.gyro_sensor_data_available = bool(self.frame_data[0]//128)
            _data = self.frame_data[1]*256+self.frame_data[2]
            if _data > 32767:
                signed_data = (_data - 65536)
            else:
                signed_data = _data
            self.mobmsg.gyro.x_axis_data = signed_data*(-0.00875)

        elif data.id >= 0x720 and data.id <= 0x726:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            tsr_idx = data.id - 0x720
            self.tsr_num = tsr_idx
            self.tsr[tsr_idx].vision_only_sign_type = self.frame_data[0]
            self.tsr[tsr_idx].supplementary_sign_type = self.frame_data[1]
            self.tsr[tsr_idx].sign_position_x = self.frame_data[2]*0.5
            _data = self.frame_data[3]%128
            if _data > 63:
                signed_data = _data - 128
            else:
                signed_data = _data
            self.tsr[tsr_idx].sign_position_y = signed_data*0.5
            _data = self.frame_data[4]%64
            if _data > 31:
                signed_data = _data - 64
            else:
                signed_data = _data
            self.tsr[tsr_idx].sign_position_z = signed_data*0.5
            self.tsr[tsr_idx].filter_type = self.frame_data[5]

        elif data.id == 0x727:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.tsr_vision_only.vision_only_sign_type_display_1 = self.frame_data[0]
            self.mobmsg.tsr_vision_only.supplementary_sign_type_display_1 = self.frame_data[1]
            self.mobmsg.tsr_vision_only.vision_only_sign_type_display_2 = self.frame_data[2]
            self.mobmsg.tsr_vision_only.supplementary_sign_type_display_2 = self.frame_data[3]
            self.mobmsg.tsr_vision_only.vision_only_sign_type_display_3 = self.frame_data[4]
            self.mobmsg.tsr_vision_only.supplementary_sign_type_display_3 = self.frame_data[5]
            self.mobmsg.tsr_vision_only.vision_only_sign_type_display_4 = self.frame_data[6]
            self.mobmsg.tsr_vision_only.supplementary_sign_type_display_4 = self.frame_data[7]

        elif data.id == 0x400:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.smart_adas.persistent_on_off = (self.frame_data[0]%32)//8
            self.mobmsg.smart_adas.volume_level = self.frame_data[0]//32
            self.mobmsg.smart_adas.hmw_level = self.frame_data[1]%16
            self.mobmsg.smart_adas.ldw_volume_level = self.frame_data[2]%8
            self.mobmsg.smart_adas.hmw_volume_level = (self.frame_data[2]%64)//8
            self.mobmsg.smart_adas.ldw_level = self.frame_data[2]//64
            self.mobmsg.smart_adas.pedestrian_warning_level = self.frame_data[3]%4
            self.mobmsg.smart_adas.sli_warning_level = self.frame_data[4]%8
            self.mobmsg.smart_adas.blinker_reminder_level = self.frame_data[5]%8
            self.mobmsg.smart_adas.virtual_bumper_level = self.frame_data[6]%16
            self.mobmsg.smart_adas.hw_repeatable_level = self.frame_data[7]%16

        elif data.id == 0x401:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.smart_adas.buzzer_max_volume = self.frame_data[0]%8
            self.mobmsg.smart_adas.buzzer_min_volume = (self.frame_data[0]%64)//8
            self.mobmsg.smart_adas.ewiii_speed_indication = self.frame_data[0]//64
            self.mobmsg.smart_adas.buzzer_hmw_max_volume = self.frame_data[1]%8
            self.mobmsg.smart_adas.buzzer_hmw_min_volume = (self.frame_data[1]%64)//8
            self.mobmsg.smart_adas.disable_system_off = self.frame_data[1]//64
            self.mobmsg.smart_adas.buzzer_ldw_max_volume = self.frame_data[2]%8
            self.mobmsg.smart_adas.buzzer_ldw_min_volume = (self.frame_data[2]%64)//8
            self.mobmsg.smart_adas.calibration_source = self.frame_data[2]//64
            self.mobmsg.smart_adas.ldw_max_value = self.frame_data[3]%4
            self.mobmsg.smart_adas.ldw_min_value = (self.frame_data[3]%16)//4
            self.mobmsg.smart_adas.ldw_speed = (self.frame_data[3]%128)//16
            self.mobmsg.smart_adas.ped_max_value = self.frame_data[4]%4
            self.mobmsg.smart_adas.ped_min_value = (self.frame_data[4]%16)//4
            self.mobmsg.smart_adas.speed_for_high_low_beam_control = self.frame_data[4]//64
            self.mobmsg.smart_adas.virtual_bumper_max_value = self.frame_data[6]%16
            self.mobmsg.smart_adas.virtual_bumper_min_value = self.frame_data[6]//16
            self.mobmsg.smart_adas.blinker_reminder_max_value = self.frame_data[7]%16
            self.mobmsg.smart_adas.blinker_reminder_min_value = self.frame_data[7]//16

        elif data.id == 0x402:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.smart_adas.hmw_max_value = self.frame_data[0]%16
            self.mobmsg.smart_adas.hmw_min_value = self.frame_data[1]%16
            self.mobmsg.smart_adas.hmw_repeatable_max_value = self.frame_data[2]%16
            self.mobmsg.smart_adas.hmw_repeatable_min_value = self.frame_data[3]%16
            self.mobmsg.smart_adas.sli_max_value = self.frame_data[4]%8
            self.mobmsg.smart_adas.sli_min_value = (self.frame_data[4]%64)//8
            self.mobmsg.smart_adas.sli_delta_round_step = self.frame_data[5]%16
            self.mobmsg.smart_adas.sli_delta_round_upwards = (self.frame_data[5]%64)//16
            self.mobmsg.smart_adas.sli_unit_speed = (self.frame_data[6]%16)//4
            self.mobmsg.smart_adas.country_code = (self.frame_data[6]%64)//16
            self.mobmsg.smart_adas.tamper_alert_on_frames = self.frame_data[7]%4
            self.mobmsg.smart_adas.tamper_alert_off_frames = (self.frame_data[7]%16)//4
            self.mobmsg.smart_adas.tamper_alert_enable_j1939 = (self.frame_data[7]%64)//16

        elif data.id == 0x403:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            menuticks = self.frame_data[1]*256+self.frame_data[0]
            ad_menuticks = self.frame_data[3]*256+self.frame_data[2]
            for i in range(16):
                self.mobmsg.smart_adas.menu_ticks[i] = menuticks%2
                self.mobmsg.smart_adas.advanced_menu_ticks[i] = ad_menuticks%2
                menuticks = menuticks//2
                ad_menuticks = ad_menuticks//2

        elif data.id == 0x410:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.seeq.serial_number = self.frame_data[2]*256*256+self.frame_data[1]*256+self.frame_data[0]

        elif data.id == 0x411:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.seeq.production_date = self.frame_data[3]*256*256*256+self.frame_data[2]*256*256+self.frame_data[1]*256+self.frame_data[0]

        elif data.id == 0x412:
            self.frame_ok = 1
            self.frame_data = bytearray(data.data)
            self.mobmsg.seeq.brain_version_major = self.frame_data[0]
            self.mobmsg.seeq.brain_version_minor = self.frame_data[1]
            self.mobmsg.seeq.mest_version_major = self.frame_data[2]
            self.mobmsg.seeq.mest_version_minor = self.frame_data[3]
            self.mobmsg.seeq.mest_version_subminor = self.frame_data[4]
            self.mobmsg.seeq.mest_version_patch_number = self.frame_data[5]

        else:
            self.frame_ok = 0

    def data_pub(self, data):
        self.data_parser(data)
        if self.frame_ok == 1:
            self.mobmsg.obstacle_data = self.obstacle_data[:self.mobmsg.obstacle_status.number_of_obstacles]
            self.mobmsg.next_lane = self.next_lane[:self.mobmsg.number_of_next_lane_markers]
            self.mobmsg.tsr = self.tsr[:self.tsr_num]
        self.mobPub.publish(self.mobmsg)

def main(args=None):
    rclpy.init(args=args)
    main_node = MobileyeNode()
    rclpy.spin(main_node)

    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
