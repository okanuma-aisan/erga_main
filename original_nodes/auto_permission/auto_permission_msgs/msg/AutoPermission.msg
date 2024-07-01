uint32 PERM_OK = 0
uint32 PERM_NDT_TP = 1
uint32 PERM_NDT_NVTL = 2
uint32 PERM_GNSS_DEV = 4
uint32 PERM_WAY_DISTANCE = 8
uint32 PERM_WAY_ANGLE = 16
uint32 PERM_KINEMATIC_TIME_DIFF_OVER = 32
uint32 PERM_STEER_SPEED_OVER = 64
uint32 PERM_NOT_STEER_MODE = 128
uint32 PERM_NOT_PEDAL_MODE = 256
uint32 PERM_TRAJECTORY_TIME_DIFF_OVER = 512
uint32 PERM_PATH_TIME_DIFF_OVER = 1024
uint32 PERM_SCENARIO_TRAJECTORY_DISTANCE = 2048
uint32 PERM_SCENARIO_TRAJECTORY_ANGULAR = 4096

bool forcibly #このフラグがtrueの場合、強制てきにpermission okにする

uint32 steer_permission
uint32 pedal_permission

bool use_ndt_tp
float32 ndt_tp #NDT評価値
float32 ndt_tp_min
bool use_ndt_nvtl
float32 ndt_nvtl #NDT評価値
float32 ndt_nvtl_min
float32 ndt_nvtl_max
int32 ndt_tp_over_count_th #ndt精度低下がこの回数発生したらエラー
int32 position_distance_count_th #経路との誤差がこの回数発生したらエラー

bool use_gnss_dev
float64 gnss_lat_dev #緯度誤差
float64 gnss_lat_dev_th
float64 gnss_lon_dev #軽度誤差
float64 gnss_lon_dev_th
float64 gnss_alt_dev #高度誤差
float64 gnss_alt_dev_th
float64 gnss_yaw_dev_th #設定だけ、使用はしていない interface用

bool use_kinematic_stat
float64 kinematic_time_diff #自車位置更新時間差分
float64 kinematic_time_diff_th

bool use_position_and_traj
bool use_angle_and_traj
float64 position_and_traj_distance #車両位置と経路との距離
float64 position_and_traj_distance_th
float64 position_and_traj_augular_deg #車両位置と経路との向き
float64 position_and_traj_augular_th_deg

bool use_trajectory_time_diff #経路(Trajectory)置更新時間しきい値を使用するか？
float64 trajectory_time_diff #経路(Trajectory)置更新時間
float64 trajectory_time_diff_th #経路(Trajectory)置更新時間しきい値

bool pedal_mode #ペダル自動運転モード
bool steer_mode #ステア自動運転モード
