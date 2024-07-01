#!/bin/bash

source ~/.bashrc


#sudo chmod 666 /dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU21470314E-if00-port0
#/home/sit/prepare_sysdiag.sh
#python3 /home/sit/auto_aichi_logger/aichi_logger.py &


### rainbow ###
#ros2 launch autoware_rainbow_launch logging_simulator.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r15.osm pointcloud_map_file:=okabe_MGRS_0.5.pcd vehicle_model:=rainbow_vehicle sensor_model:=rainbow_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml

### erga ###
ros2 launch autoware_erga_launch logging_simulator.launch.xml map_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs lanelet2_map_file:=lanelet2_map_okabe_r15.osm pointcloud_map_file:=okabe_MGRS_0.5.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/okabe/pcd/create_wan/okabe_mgrs/map_projector_info.yaml
