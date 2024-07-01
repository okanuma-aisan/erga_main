# mgrs_gcs_converter
## Description
Subscribes to `~/input_map_info` and `~/input_odometry_mgrs`.
Converts the x, y and z coordinates found in `~/input_odometry_mgrs.pose.pose.position` from MGRS to GCS (Longitute & Latitude), using the data found in `~/input_map_info.mgrs_grid`
Publishes the result to `~/output_odometry_gcs`

## Usage
### Building
1. Install GeographicLib
```
sudo apt install libgeographic-dev
```

2. Clone to workspace
```
cd autoware
git clone git@github.com:saikocar/mgrs_gcs_converter.git
```

3. Build package
```
colcon build --symlink-install --continue-on-error --cmake-clean-first --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select mgrs_gcs_converter
```

4. Launch node
```
ros2 launch mgrs_gcs_converter convert_autoware.launch.xml
```

By default, the remappings are as follows:
| Remap From              | Remap To                              |
| :---------------------- | :------------------------------------ |
| `~/input_map_info`      | `/map/map_projector_info`             |
| `~/input_odometry_mgrs` | `/localization/kinematic_state`       |
| `~/output_odometry_gcs` | `/localization/kinematic_state_gcs`   |

This default configuration uses autoware-standard topic names for the inputs, and will publish the GCS-converted coordinates to `/localization/kinematic_state_gcs`.

## Topics
### Subscriptions
| Topic Name              | Type                                  | Description                                                                      |
| :---------------------- | :------------------------------------ | :------------------------------------------------------------------------------- |
| `~/input_map_info`      | tier4_map_msgs::msg::MapProjectorInfo | MapProjectorInfo topic containng `vertical_datum` and `mgrs_grid` information.   |
| `~/input_odometry_mgrs` | nav_msgs::msg::Odometry               | Odometry topic containing pose coordinates in MGRS coordinate system.            |
### Publications
| Topic Name              | Type                    | Description                                                                               |
| :---------------------- | :---------------------- | :---------------------------------------------------------------------------------------- |
| `~/output_odometry_gcs` | nav_msgs::msg::Odometry | Output odometry topic with pose coordinates converted from MGRS to GCS coordinate system. |
