#!/bin/bash

source ~/.bashrc

# Rainbow
#sudo chmod 666 /dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU18180091Y-if00-port0
# Erga
sudo chmod 666 /dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_DMMU21470314E-if00-port0

/home/sit/prepare_sysdiag.sh

python $HOME/auto_aichi_logger/aichi_logger.py &

mkdir -p /home/sit/autoware_stdoutstderr/


#ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/home/sit/load_data/moriyama lanelet2_map_file:=MORIYAMA_TESTCOURSE_20240220.osm pointcloud_map_file:=moriyama_MGRS0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=map_projector_info.yaml launch_deprecated_api:=true 2>&1 | tee /home/sit/autoware_stdoutstderr/`date +"%Y-%m-%d_%H-%M-%S"`.log

# FMS
ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/opt/autoware/maps lanelet2_map_file:=lanelet2_map.osm pointcloud_map_file:=pcd/pointcloud0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/aichi_sky_expo/map_projector_info.yaml launch_deprecated_api:=true 2>&1 | tee /home/sit/autoware_stdoutstderr/`date +"%Y-%m-%d_%H-%M-%S"`.log

# FMS not use
#ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/home/sit/load_data/tokoname/FMS lanelet2_map_file:=lanelet2_map.osm pointcloud_map_file:=pcd/pointcloud0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/tokoname/FMS/map_projector_info.yaml launch_deprecated_api:=true 2>&1 | tee /home/sit/autoware_stdoutstderr/`date +"%Y-%m-%d_%H-%M-%S"`.log

#ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/home/sit/load_data/morikoro_2024 lanelet2_map_file:=lanelet2_map.osm pointcloud_map_file:=pcd/pointcloud_map0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=/home/sit/load_data/aichi_sky_expo/map_projector_info.yaml launch_deprecated_api:=true 2>&1 | tee /home/sit/autoware_stdoutstderr/`date +"%Y-%m-%d_%H-%M-%S"`.log

#ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/home/sit/load_data/tokoname/FMS lanelet2_map_file:=lanelet2_map.osm pointcloud_map_file:=pcd/pointcloud_map0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=../map_projector_info.yaml launch_deprecated_api:=true 1>/home/sit/autoware_stdoutstderr/"$(date +%Y-%m-%d%H-%M-%S)_stdout.log" 2>/home/sit/autoware_stdoutstderr/"$(date +%Y-%m-%d%H-%M-%S)_stderr.log"

#ros2 launch autoware_erga_launch autoware.launch.xml map_path:=/home/sit/load_data/tokoname/FMS lanelet2_map_file:=we1L5Yvi_tokoname.osm pointcloud_map_file:=pcd/TAMOKUTEKI_AEON_MGRS0.2.pcd vehicle_model:=erga_vehicle sensor_model:=erga_sensor_kit map_projector_info_path:=map_projector_info.yaml launch_deprecated_api:=true 2>&1 | tee /home/sit/autoware_stdoutstderr/`date +"%Y-%m-%d_%H-%M-%S"`.log


