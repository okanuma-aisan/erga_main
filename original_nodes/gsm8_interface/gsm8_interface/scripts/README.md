# 車両精度検証及び Map 作成について

## 1. 車速センサー精度検証

手動運転で、車両の CAN データとドップラー速度計で計測した車速を比較して、速度精度を検証する。

```bash
# CANデータの値
/gsm8/can/status/m0_status
# ドップラー速度計の値(真値)
/sensing/ldvs/data/speed
```

## 2. 操舵角センサー精度検証

手動運転で、車両の CAN データから算出したヨーレートと IMU で計測したヨーレートを比較して、操舵角精度を検証する。

```bash
ros2 run gsm8_interface yawrate_estimator.py
# CANデータから算出したヨーレート
/kinematic_bicycle_model/twist
```

```bash
ros2 run gsm8_interface imu_reverse.py
# IMUで計測したヨーレート(真値)
/sensing/imu/imu_data
```

## 3. By-wire 操舵精度検証(EPS Map + PID Controller)

横方向のみ自動運転モードで、目標ヨーレートを実現する EPS 電圧を FF ＋ FB コントローラーから算出する。

### 3-1. EPS Map 作成(FF Control)

EPS Map を作成するために、Keyboard から[−12.0V, 12.0V]の電圧ごとを入力して、操舵角と操舵角速度を確認する。

```bash
ros2 run gsm8_interface eps_voltage_publisher.py
ros2 run gsm8_interface eps_voltage_tester.py
```

### 3-2. Steering 精度検証(FB Control)

EPS Map を用い、Keyboard から目標操舵角と目標操舵角速度を入力して、出力電圧を確認する。

```bash
ros2 run gsm8_interface eps_ff_voltage_publisher.py
ros2 run gsm8_interface eps_ff_voltage_tester.py
```

## 4. By-wire 加速精度検証

縦方向のみ自動運転モードで、目標加速度を実現するスロットル値を算出する。

### 4-1. Accel Map 作成

Accel Map を作成するために、Keyboard から[0.0, 1.0]の値を入力して、車速と加速度を確認する。

```bash
ros2 run gsm8_interface throttle_publisher.py
ros2 run gsm8_interface throttle_tester.py
```

### 4-2. Accel 精度検証

Accel Map を用い、Keyboard から目標加速度を入力して、アクセルの出力を確認する。

```bash
ros2 run gsm8_interface acceleration_publisher.py
ros2 run gsm8_interface acceleration_tester.py
```

## 5. By-wire 減速精度検証

縦方向のみ自動運転モードで、目標加速度を実現するブレーキ油圧を算出する。

### 5-1. Brake Map 作成

Brake Map を作成するために、Keyboard から[0.0, 5.0]MPa の油圧を入力して、車速と加速度を確認する。

```bash
ros2 run gsm8_interface brake_pressure_publisher.py
ros2 run gsm8_interface brake_pressure_tester.py
```
