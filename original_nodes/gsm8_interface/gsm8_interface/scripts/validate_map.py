#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import argparse

import matplotlib.pyplot as plt
import pandas as pd

value_change_ratio_threshold_acc = 10.0
value_change_ratio_threshold_brk = 10.0
value_change_ratio_threshold_eps = 70.0


def check_bag():
    # Accel Map
    print("Accel map is created on weighing 780kg?")
    choice = input("Answer with 'yes' or 'no' [y/N]: ").lower()
    if choice in ["y", "yes", ""]:
        pass
    else:
        print("[NG] Accel map should created on weighing 780kg.")
        exit(1)
    # Brake Map
    print("Brake map is created on weighing 780kg?")
    choice = input("Answer with 'yes' or 'no' [y/N]: ").lower()
    if choice in ["y", "yes", ""]:
        pass
    else:
        print("[NG] Brake map should created on weighing 780kg.")
        exit(1)
    # Steer Map
    print("Accel map is created on weighing 120kg?")
    choice = input("Answer with 'yes' or 'no' [y/N]: ").lower()
    if choice in ["y", "yes", ""]:
        pass
    else:
        print("[NG] Steer map should created on weighing 780kg.")
        exit(1)


def draw_map(accel_map, brake_map, steer_map):
    color_list = [
        "#e41a1c",
        "#377eb8",
        "#4daf4a",
        "#984ea3",
        "#ff7f00",
        "#ffff33",
        "#a65628",
        "#f781bf",
        "#ff8105",
        "#f70100",
        "#37ff21",
        "#33aa00",
        "#25f0f0",
    ]

    plt.title("Accel Map")
    acc_x_ticks = [0, 0.69, 1.39, 2.08, 2.78, 3.47, 4.17, 4.86, 5.56, 6.25]
    for i in range(accel_map.shape[0]):
        plt.plot(acc_x_ticks, accel_map.iloc[i, 1:11], color_list[i], label="0." + str(i) + "[V]")
    plt.xticks(acc_x_ticks)
    plt.xlabel("Target Velocity")
    plt.ylabel("Target Acceleration")
    plt.legend()
    plt.show()

    plt.title("Brake Map")
    brk_x_ticks = [0, 0.69, 1.39, 2.08, 2.78, 3.47, 4.17, 4.86, 5.56, 6.25]
    brk_list = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]
    for i in range(brake_map.shape[0]):
        plt.plot(
            brk_x_ticks, brake_map.iloc[i, 1:13], color_list[i], label=str(brk_list[i]) + "[MPa]"
        )
    plt.xticks(brk_x_ticks)
    plt.xlabel("Target Velocity")
    plt.ylabel("Target Acceleration")
    plt.legend()
    plt.show()

    plt.title("Steer Map")
    eps_x_ticks = [-0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    eps_list = [-8, -6, -4, -2, 0, 2, 4, 6, 8]
    for i in range(steer_map.shape[0]):
        plt.plot(
            eps_x_ticks, steer_map.iloc[i, 1:14], color_list[i], label=str(eps_list[i]) + "[V]"
        )
    plt.xticks(eps_x_ticks)
    plt.xlabel("Target Steer")
    plt.ylabel("Target Steer Velocity")
    plt.legend()
    plt.show()


def check_acc_equal_brk(accel_map, brake_map):
    accel_map_equal_brake_map = True
    for i in range(1, accel_map.shape[0]):
        if accel_map.iloc[0, i] == brake_map.iloc[0, i]:
            pass
        else:
            accel_map_equal_brake_map = False
            print(f"[NG] Check the Accel map at {0, i}, value: {accel_map.iloc[0, i]}")
            print(f"[NG] Check the Brake map at {0, i}, value: {brake_map.iloc[0, i]}")
            break
    if accel_map_equal_brake_map is True:
        print("[OK] Accel map is same as Brake map when target voltage is 0[V].")
    else:
        print(
            "[NG] Accel map should same as Brake map when target voltage is 0[V], check map please."
        )


def check_value_monotonicity_acc(accel_map):
    map_is_ok = True
    for i in range(1, accel_map.shape[1]):
        for j in range(accel_map.shape[0] - 1):
            if accel_map.iloc[j, i] < accel_map.iloc[j + 1, i]:
                pass
            else:
                map_is_ok = False
                print(f"[NG] Check the Accel map at {j + 2, i}, value: {accel_map.iloc[j + 1, i]}.")
                break
    if map_is_ok is True:
        print("[OK] Accel map values is increase monotonously with the target voltage value.")
    else:
        print("[NG] Accel map values is not increase monotonously with the target voltage value.")


def check_value_monotonicity_brk(brake_map):
    map_is_ok = True
    for i in range(1, brake_map.shape[1]):
        for j in range(brake_map.shape[0] - 1):
            if brake_map.iloc[j, i] > brake_map.iloc[j + 1, i]:
                pass
            else:
                map_is_ok = False
                print(f"[NG]Check the Brake map at {j + 2, i}, value: {brake_map.iloc[j + 1, i]}.")
                break
    if map_is_ok is True:
        print("[OK] Brake map values is increase monotonously with the target voltage value.")
    else:
        print("[NG] Brake map values is not increase monotonously with the target voltage value.")


def check_value_monotonicity_eps(steer_map):
    map_is_ok = True
    for i in range(1, steer_map.shape[1] - 1):  # cols
        for j in range(steer_map.shape[0] - 1):  # rows
            if steer_map.iloc[j + 1, i] > steer_map.iloc[j, i]:  # > steer_map.iloc[j, i + 1]:
                pass
            else:
                map_is_ok = False
                print(f"[NG] Check the Steer map at {j + 1, i}, value: {steer_map.iloc[j, i]}.")
                break
    if map_is_ok is True:
        print("[OK] Steer map values is increase monotonously with the target voltage value.")
    else:
        print("[NG] Steer map values is not increase monotonously with the target voltage value.")


def check_value_change_acc(accel_map, value_change_ratio_threshold):
    map_is_ok = True
    for i in range(1, accel_map.shape[1]):
        for j in range(accel_map.shape[0] - 1):
            value_change_ratio = abs(accel_map.iloc[j + 1, 0] - accel_map.iloc[j, 0]) / abs(
                accel_map.iloc[j + 1, i] - accel_map.iloc[j, i]
            )
            if value_change_ratio <= value_change_ratio_threshold:
                pass
            else:
                map_is_ok = False
                print(
                    f"[NG] Check the Accel map at {j + 1, i}, value: {accel_map.iloc[j, i]}, "
                    f"and the value changed ratio is {value_change_ratio:.2f}"
                )
                break
    if map_is_ok is True:
        print(
            f"[OK] Accel map values changed ratio under the threshold: {value_change_ratio_threshold}"
        )
    else:
        print(
            f"[NG] Accel map values changed ratio over the threshold: {value_change_ratio_threshold}"
        )


def check_value_change_brk(brake_map, value_change_ratio_threshold):
    map_is_ok = True
    for i in range(1, brake_map.shape[1]):
        for j in range(brake_map.shape[0] - 1):
            value_change_ratio = abs(brake_map.iloc[j + 1, 0] - brake_map.iloc[j, 0]) / abs(
                brake_map.iloc[j + 1, i] - brake_map.iloc[j, i]
            )
            if value_change_ratio <= value_change_ratio_threshold:
                pass
            else:
                map_is_ok = False
                print(
                    f"[NG] Check the Brake map at {j+1, i}, value: {brake_map.iloc[j, i]}, "
                    f"and the value changed ratio is {value_change_ratio:.2f}"
                )
                break
    if map_is_ok is True:
        print(
            f"[OK] Brake map values changed ratio under the threshold: {value_change_ratio_threshold}"
        )
    else:
        print(
            f"[NG] Brake map values changed ratio over the threshold: {value_change_ratio_threshold}"
        )


def check_value_change_eps(steer_map, value_change_ratio_threshold):
    map_is_ok = True
    for i in range(1, steer_map.shape[1]):
        for j in range(steer_map.shape[0] - 1):
            value_change_ratio = abs(steer_map.iloc[j + 1, 0] - steer_map.iloc[j, 0]) / abs(
                steer_map.iloc[j + 1, i] - steer_map.iloc[j, i]
            )
            if value_change_ratio <= value_change_ratio_threshold:
                pass
            else:
                map_is_ok = False
                print(
                    f"[NG] Check the Steer map at {j+1, i}, value: {steer_map.iloc[j, i]}, "
                    f"and the value changed ratio is {value_change_ratio:.2f}"
                )
                break
    if map_is_ok is True:
        print(
            f"[OK] Steer map values changed ratio under the threshold: {value_change_ratio_threshold}"
        )
    else:
        print(
            f"[NG] Steer map values changed ratio over the threshold: {value_change_ratio_threshold}"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check accel, brake and steer map.")
    parser.add_argument("map_path", help="/path/to/individual_params/config/ps/vehicle/")
    args = parser.parse_args()

    # Check the map is created on 780kg or 120kg
    check_bag()

    # Read csv file and save as dictionary
    accel_map_data = pd.read_csv(args.map_path + "accel_map.csv")
    brake_map_data = pd.read_csv(args.map_path + "brake_map.csv")
    steer_map_data = pd.read_csv(args.map_path + "steer_map.csv")
    # print("Read accel map successes!\n", accel_map_data)
    # print("Read brake map successes!\n", brake_map_data)
    # print("Read steer map successes!\n", steer_map_data)

    # Check accel map is same as brake map when voltage is 0
    check_acc_equal_brk(accel_map_data, brake_map_data)
    # Check whether the value is monotonically increasing or decreasing
    check_value_monotonicity_acc(accel_map_data)
    check_value_monotonicity_brk(brake_map_data)
    check_value_monotonicity_eps(steer_map_data)
    # Check whether the acceleration change is too small to cause the target voltage change
    check_value_change_acc(accel_map_data, value_change_ratio_threshold_acc)
    check_value_change_brk(brake_map_data, value_change_ratio_threshold_brk)
    check_value_change_eps(steer_map_data, value_change_ratio_threshold_eps)

    draw_map(accel_map_data, brake_map_data, steer_map_data)
