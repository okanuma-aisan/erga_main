# Overview

NDT, GNSS, GMPSを観測として、EKFを走らせる。


## Nodes

### Subscribed Topics
- in_ndt_pose (geometry_msgs/PoseWithCovarianceStamped)

  NDTによる観測座標

- in_gnss_pose (geometry_msgs/PoseWithCovarianceStamped)

  GNSSによる観測座標

- in_gmps_pose (geometry_msgs/PoseWithCovarianceStamped)

  GMPSによる観測座標

- in_velocity (geometry_msgs/TwistWithCovarianceStamped)

  後軸中心の速度

- in_delta (wada_vmc_msgs/Can502_20221111)

  タイヤ操舵角

- in_gamma (sensor_msgs/Imu)

  IMUによるヨーレート

- in_ndt_tp (tier4_debug_msgs/Float32Stamped)

  transformation probability

- in_ndt_nvtl (tier4_debug_msgs/Float32Stamped)

  nearest voxel transformation likelihood

- in_gnss_gpgga (nmea_msgs/Gpgga)

  NMEAのGPGGAセンテンス




### Published Topics
- ekf_aichi_pose (geometry_msgs/PoseWithCovarianceStamped)

  推定位置

- debug_ekf_log (gmps_msgs/DebugEkfLog)

  解析用の情報を集めたメッセージ

### Pulished TF
- ekf_aichi_link

  TF from "map" coordinate to estimated pose.


## Functions
### time_update (predict)
状態の前回値$x_{k-1}$と入力$u_k$により、
状態の事前推定値$\hat{x}_k^-$を計算する。
もし観測が無い場合は、事前推定値$\hat{x}_k^-$が位置推定結果として出力される。


### measurement_update (correct)
NDT, GNSS, GMPSの観測$z_k$があった場合に、カルマンゲイン$G_k$と残余$z_k - h(\hat{x}_k^-)$により事後推定値$\hat{x}_k$を計算する。
各観測について同じ処理を繰り返すため、1タイムステップの間に最大で3回の観測更新が行われる場合がある。



## Parameter description

The parameters are set in `launch/ekf_aichi_localizer.launch.xml` .

### For Node

| Name                       | Type   | Description                                                                               | Default value |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------- | :------------ |
| show_debug_info            | bool   | Flag to display debug info                                                                | false         |
| timer_freq          | double | Frequency for filtering and publishing [Hz]                                               | 50.0          |



### For time_update (vehiicle parameter)

| Name                          | Type   | Description                                                   | Default value |
| :---------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| lr | double | 重心から後軸までの距離 [m]                      |     1.5       |
| lw  | double    | ホイールベース [m]                              | 4.0            |
| Ksf        | double | スタビリティファクタ | 0.001       |
| Kbeta0        | double | 横滑り係数 | -0.001       |



### For covariance matrix

| Name                          | Type   | Description                                                   | Default value |
| :---------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| sigma_x_ndt | double | NDTのX座標の標準偏差 [m]           |  0.07  |
| sigma_y_ndt | double | NDTのY座標の標準偏差 [m]           | 0.07  |
| sigma_theta_ndt | double | NDTのYaw角の標準偏差 [rad] | 0.10   |
| sigma_x_gnss | double | GNSSのX座標の標準偏差 [m]           |  0.07  |
| sigma_y_gnss | double | GNSSのY座標の標準偏差 [m]           | 0.07  |
| sigma_theta_gnss | double | GNSSのYaw角の標準偏差 [rad] | 0.10   |
| sigma_x_gmps | double | GMPSのX座標の標準偏差 [m]           |  0.07  |
| sigma_y_gmps | double | GMPSのY座標の標準偏差 [m]           | 0.07  |
| sigma_theta_gmps | double | GMPSのYaw角の標準偏差 [rad] | 0.10   |
| sigma_v | double | 速度の標準偏差 [m/s]           |  0.03  |
| sigma_delta | double | タイヤ操舵角の標準偏差 [rad]           | 0.04  |
| sigma_gamma | double | ヨーレートの標準偏差 [rad/s] | 0.005   |


### For EKF

| Name                          | Type   | Description                                                   | Default value |
| :---------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| limit_rate_kv | double | 速度スケールファクタ$k_v$のレートリミット          |  0.001  |
| limit_rate_gb | double | ヨーレートバイアス$\gamma_b$のレートリミット  | 0.0001  |
| limit_rate_kb | double | 横滑り角スケールファクタ$k_\beta$のレートリミット| 0.001   |
| min_kv | double | 速度スケールファクタ$k_v$の最小値           |  0.9  |
| max_kv | double | 速度スケールファクタ$k_v$の最大値          | 1.1  |
| min_gb | double | ヨーレートバイアス$\gamma_b$の最小値 | -0.001   |
| max_gb | double | ヨーレートバイアス$\gamma_b$の最大値          |  +0.001  |
| min_kb | double | 横滑り角スケールファクタ$k_\beta$の最小値           | 0.9  |
| max_kb | double | 横滑り角スケールファクタ$k_\beta$の最大値 | 1.1   |

### For measurement ready

| Name                          | Type   | Description                                                   | Default value |
| :---------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| min_ndt_tp | double | NDT TPの最小値          |  3.0  |
| min_ndt_nvtl | double | NDT NVTLの最小値  | 1.5  |
| min_gnss_fix_quality | int | GPGGAセンテンスに含まれるFixQualityの最小値 | 3   |
| max_gnss_hdop | double | GPGGAセンテンスに含まれるHDOPの最大値           |  2.0  |
| enable_ndt | bool | NDTの観測を有効にするかどうか          | true  |
| enable_gnss | bool | GNSSの観測を有効にするかどうか          | true  |
| enable_gmps | bool | GMPSNDTの観測を有効にするかどうか          | true  |


## Kalman Filter Model

### kinetic model in time update

$$ x_k = x_{k-1}  +  k_{v,k-1}V_k T_s \cos(\theta_{k-1} + k_{\beta,k-1} \beta_{r,k})$$
$$ y_k =  y_{k-1}  +  k_{v,k-1}V_k T_s \sin(\theta_{k-1} + k_{\beta,k-1} \beta_{r,k})$$
$$ \theta_k = \theta_{k-1} + (\gamma_k - \gamma_{b,k})T_s$$
$$ k_{v,k} = k_{v,k-1} $$
$$\gamma_{b,k} = \gamma_{b,k-1}$$
$$k_{\beta,k} = k_{\beta,k-1}$$

where $k_v$ is the scale factor of velocity $V$,
 $\gamma_b$ is the bias of yawrate $\gamma$ , and 
 $k_\beta$ is the scale factor of sideslip angle $\beta$ .

sideslip angle at rear axle $\beta_r$ is defined as following, 

$$ \beta_{r,k} = \frac{K_{\beta_0} V_k^2}{1+K_{sf}V_k^2} \frac{l_r}{l_w}\delta_k  $$


## How to tune EKF parameters

### 1. vehicle parameters
ホイールベース$l_w$はメーカー車両諸元から転記する。
重心から後軸中心までの距離$l_r$は、車検証の前軸荷重と後軸荷重により内分比を求める。
スタビリティファクタ$K_{sf}$と横滑り係数$K_{\beta_0}$は定常円旋回の実験により求める。


## Known issues
- 外れ値のゲート処理
- readyの条件判定は別nodeにすべきか
  - subscribeが増えるため
