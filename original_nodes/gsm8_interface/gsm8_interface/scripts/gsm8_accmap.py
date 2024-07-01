#!/usr/bin/env python2

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

import csv
import pickle

import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy import interpolate
from scipy import signal

SAMPLING_RATE = 100  # [Hz]
CAR_NO = "pv1"
BASE_DIR = "/home/makoto/projects/autoware.proj/data/210201_brake_test_course2/"
USE_PICKLE = False
BLKP_THRESHOULD = 0.8


def read_rosbag(bagfile):
    print("start reading:", bagfile)
    bag = rosbag.Bag(bagfile)

    accx = []  # [m/s^2]
    time_accx = []  # [sec]
    velx = []  # [m/s]
    time_velx = []  # [sec]

    i = 0
    time_first = 0.0
    ntime_first = 0.0
    enable = False
    ep = 0.01

    for topic, msg, t in bag.read_messages():
        if i == 0:
            time_first = t.secs
            ntime_first = round(t.nsecs / 1000.0 / 1000.0 / 1000.0, 3)
            i = i + 1
        if topic == "/vehicle/gsm8/can/command/CMD_M2":
            if msg.C_TRTL > ep or msg.C_BRKP > ep:
                enable = True
            elif msg.C_TRTL < ep or msg.C_BRKP < ep:
                enable = False
        if topic == "/vehicle/gsm8/can/status/STTS_M4":
            if msg.S_BWM.data == 0:
                enable = False
        if topic == "/sensing/imu/imu_data" and enable:
            accx.append(msg.linear_acceleration.x)
            time_accx.append(
                (t.secs - time_first) + (round(t.nsecs / 1000.0 / 1000.0 / 1000.0, 3) - ntime_first)
            )
        if topic == "/vehicle/gsm8/can/status/STTS_M0" and enable:
            velx.append(msg.S_SPINVA)
            time_velx.append(
                (t.secs - time_first) + (round(t.nsecs / 1000.0 / 1000.0 / 1000.0, 3) - ntime_first)
            )

    print("accx:", len(accx))
    print("time_accx:", len(time_accx))
    print("velx:", len(velx))
    print("time_velx:", len(time_velx))

    return accx, time_accx, velx, time_velx


def read_files_from_csv(csvfile):
    bags = []
    with open(csvfile) as f:
        reader = csv.reader(f)
        for row in reader:
            if row[4] == "1":
                bag = {
                    "filename": BASE_DIR + row[0],
                    "type": row[1],
                    "acc": float(row[2]),
                    "weight": float(row[3]),
                    "offset": float(row[5]),
                    "return": row[6],
                    "vel_begin": row[7],
                }
                bags.append(bag)
    return bags


def apply_1dlpf(src, gain):
    dst = []
    prev = 0.0
    first = True

    for curr in src:
        if first:
            first = False
            dst.append(curr)
            prev = curr
        else:
            filtered = gain * prev + (1.0 - gain) * curr
            dst.append(filtered)
            prev = filtered

    return dst


def apply_butterworth(src, cutoff, numtaps):
    nyq = SAMPLING_RATE / 2.0
    cutoff_norm = cutoff / nyq

    lpf = signal.firwin(numtaps, cutoff_norm)

    return signal.lfilter(lpf, 1, src)


def compensate_offset(data, offset):
    dst = []
    for d in data:
        dst.append(d - offset)
    return dst


def analysis(bag):
    bagfile = bag["filename"]
    accx, time_accx, velx, time_velx = read_rosbag(bagfile)

    accx = compensate_offset(accx, bag["offset"])

    time_begin = max(time_accx[0], time_velx[0])
    time_end = min(time_accx[-1], time_velx[-1])
    time_interp = np.arange(time_begin, time_end, 1.0 / SAMPLING_RATE)

    fitted_curve_accx = interpolate.interp1d(time_accx, accx)
    fitted_curve_velx = interpolate.interp1d(time_velx, velx)
    accx_interp = fitted_curve_accx(time_interp)
    velx_interp = fitted_curve_velx(time_interp)

    accx_filtered = apply_1dlpf(accx_interp, 0.9)

    return time_interp, accx_interp, accx_filtered, velx_interp


def plot(data):
    for d in data:
        label = d["type"] + ": " + str(d["acc"])
        if d["return"] == "1":
            label += " return"
        if d["vel_begin"] != "-":
            add = " " + d["vel_begin"] + "kph"
            label += add
        plt.plot(
            d["velx_interp"] * 60.0 * 60.0 / 1000.0,  # [m/s] -> [km/h]
            d["accx_filtered"],
            label=label,
        )
    plt.title("Velocity vs Acceleration")
    plt.xlabel("Velocity [km/h]")
    plt.ylabel("Acceleration [m/s^2]")
    plt.xticks(np.arange(0, 31, 2.5))
    plt.yticks(np.arange(-4.5, 2.1, 0.1))
    # plt.xlim(0.0, 20.0)
    # plt.ylim(-3.0, 1.5)
    plt.grid()
    plt.legend()
    plt.show()


def main():
    bags = read_files_from_csv(BASE_DIR + "data_list.csv")

    if USE_PICKLE:
        with open(BASE_DIR + "accmap-data-heavy.pickle", "rb") as f:
            data = pickle.load(f)
    else:
        data = []
        for bag in bags:
            time_interp, accx_interp, accx_filtered, velx_interp = analysis(bag)
            d = {
                "type": bag["type"],
                "acc": bag["acc"],
                "return": bag["return"],
                "vel_begin": bag["vel_begin"],
                "time_interp": time_interp,
                "accx_interp": accx_interp,
                "accx_filtered": accx_filtered,
                "velx_interp": velx_interp,
            }
            data.append(d)

    with open(BASE_DIR + "accmap-data-gsm8-light.pickle", "wb") as f:
        pickle.dump(data, f)

    plot(data)


if __name__ == "__main__":
    main()
