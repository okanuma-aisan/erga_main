#!/bin/bash

source ~/.bashrc

# Rainbow
#sudo chmod 666 /dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU18180091Y-if00-port0
# Erga
sudo chmod 666 /dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU21470314E-if00-port0

/home/sit/prepare_sysdiag.sh
python3 /home/sit/auto_aichi_logger/aichi_logger.py &


#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r15.osm pointcloud_map_file:=okabe_MGRS_0.5.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml
# 2023-12-18 FMS連携用 - Yuugo Takano
ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r15.osm pointcloud_map_file:=okabe_MGRS_0.5.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml launch_deprecated_api:=true
#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/opt/autoware/maps/ vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit launch_deprecated_api:=true

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan lanelet2_map_file:=okabe_mgrs/lanelet2_map_okabe_r10.osm pointcloud_map_file:=okabe_mgrs/okabe_MGRS_0.5.pcd vehicle_model:=rainbow_vehicle sensor_model:=rainbow_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/kokusoken lanelet2_map_file:=LL2_NILIM_COURSE_30kph_3.5m.osm pointcloud_map_file:="PCD/output_0.5.pcd" vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/kokusoken/map_projector_info.yaml

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/kokusoken lanelet2_map_file:=LL2_NILIM_COURSE_60kph_4m.osm pointcloud_map_file:="PCD/output_0.5.pcd" vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/kokusoken/map_projector_info.yaml

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/kokusoken lanelet2_map_file:=LL2_NILIM_COURSE_60kph_4m.osm pointcloud_map_file:="PCD/output_0.5.pcd" vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/kokusoken/map_projector_info.yaml

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan lanelet2_map_file:=sit_lanelet2_erga_ver9_13.osm pointcloud_map_file:=okabe_2023_09_05/okabe_0.5.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit
#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/ogose_zidousyagakkou lanelet2_map_file:=ogose_ll2_ver2.osm pointcloud_map_file:=pointcloud_map0.5.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/moriyama lanelet2_map_file:=lanelet2_map_moriyama_lane1.osm pointcloud_map_file:=moriyama_MGRS0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/moriyama/map_projector_info.yaml

#ros2 launch autoware_erga_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/tusima lanelet2_map_file:=tsushima_outline_1122.osm pointcloud_map_file:=pointcloud_map0.2_mgrs.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/tusima/map_projector_info.yaml


### rainbow ###

#ros2 launch autoware_rainbow_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r12.osm pointcloud_map_file:=okabe_MGRS_0.5.pcd vehicle_model:=rainbow_vehicle sensor_model:=rainbow_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml launch_deprecated_api:=true

#渋沢
#ros2 launch autoware_rainbow_launch autoware.yolov5.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r12.osm pointcloud_map_file:=sibusawa_all0.5_add.pcd vehicle_model:=rainbow_vehicle sensor_model:=rainbow_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml
