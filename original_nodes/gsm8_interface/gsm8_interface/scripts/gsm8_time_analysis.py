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
import math
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


def calc_current_time(ros_time, time_begin, ntime_begin):
    return (ros_time.secs - time_begin) + (
        round(ros_time.nsecs / 1000.0 / 1000.0 / 1000.0, 3) - ntime_begin
    )


def quaternion_to_euler(x, y, z, w):
    q0 = w
    q1 = x
    q2 = y
    q3 = z
    q0q0 = q0 * q0
    q0q1 = q0 * q1
    q0q2 = q0 * q2
    q0q3 = q0 * q3
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q3q3 = q3 * q3

    roll = math.atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3)
    pitch = math.asin(2.0 * (q0q2 - q1q3))
    yaw = math.atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)

    return roll, pitch, yaw


def read_rosbag(bagfile):
    print("start reading:", bagfile)
    bag = rosbag.Bag(bagfile)

    accx = []  # [m/s^2]
    time_accx = []  # [sec]
    velx = []  # [m/s]
    time_velx = []  # [sec]
    accreq = []
    time_accreq = []
    # pitch = []
    # time_pitch = []
    # acccan = []
    # time_acccan = []

    i = 0
    time_begin = 0.0
    ntime_begin = 0.0
    # debug = False

    for topic, msg, t in bag.read_messages():
        if i == 0:
            time_begin = t.secs
            ntime_begin = round(t.nsecs / 1000.0 / 1000.0 / 1000.0, 3)
            i = i + 1
            accreq.append(0.0)
            time_accreq.append(0.0)
        # if topic == '/ovp_controller/debug_vehicle_cmd':
        #     if msg.enable == True:
        #         debug = True
        #         accreq.append(0.0)
        #         time_accreq.append(calc_current_time(t, time_begin, ntime_begin))
        #         accreq.append(msg.acc)
        #         time_accreq.append(calc_current_time(t, time_begin, ntime_begin))
        #     elif msg.enable == False:
        #         debug = False
        #         accreq.append(msg.acc)
        #         time_accreq.append(calc_current_time(t, time_begin, ntime_begin))
        #         accreq.append(0.0)
        #         time_accreq.append(calc_current_time(t, time_begin, ntime_begin))
        if topic == "/sensing/imu/imu_data":
            accx.append(msg.linear_acceleration.x)
            time_accx.append(calc_current_time(t, time_begin, ntime_begin))
        if topic == "/vehicle/gsm8/can/status/STTS_M0":
            velx.append(msg.S_SPINVA)
            time_velx.append(calc_current_time(t, time_begin, ntime_begin))
        # if topic == '/current_pose':
        #     r, p, y = quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        #     pitch.append(p)
        #     time_pitch.append(calc_current_time(t, time_begin, ntime_begin))
        # if topic == '/ovp_controller/req_acc':
        #     acccan.append(msg.data)
        #     time_acccan.append(calc_current_time(t, time_begin, ntime_begin))

    print("accx:", len(accx))
    print("velx:", len(velx))
    # print 'accreq:', len(accreq)
    # print 'pitch:', len(pitch)
    # print 'acccan:', len(acccan)

    data = {
        "accx": accx,
        "time_accx": time_accx,
        "velx": velx,
        "time_velx": time_velx,
        # 'accreq': accreq,
        # 'time_accreq': time_accreq,
        # 'pitch': pitch,
        # 'time_pitch': time_pitch,
        # 'acccan': acccan,
        # 'time_acccan': time_acccan,
    }

    return data


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
    data = read_rosbag(bagfile)

    data["accx"] = compensate_offset(data["accx"], bag["offset"])

    time_begin = max(data["time_accx"][0], data["time_velx"][0])
    time_end = min(data["time_accx"][-1], data["time_velx"][-1])
    time_interp = np.arange(time_begin, time_end, 1.0 / SAMPLING_RATE)

    fitted_curve_accx = interpolate.interp1d(data["time_accx"], data["accx"])
    fitted_curve_velx = interpolate.interp1d(data["time_velx"], data["velx"])
    accx_interp = fitted_curve_accx(time_interp)
    velx_interp = fitted_curve_velx(time_interp)

    accxs_filtered = []
    gains = [0.9]
    for gain in gains:
        accx_filtered = apply_1dlpf(accx_interp, gain)
        accxs_filtered.append({"filter": "1d", "gain": gain, "accx_filtered": accx_filtered})

    cutoffs = [1, 0.5]
    numtaps = [31]
    for cutoff in cutoffs:
        for numtap in numtaps:
            accx_filtered = apply_butterworth(accx_interp, cutoff, numtap)
            accxs_filtered.append(
                {
                    "filter": "butterworth",
                    "cutoff": cutoff,
                    "numtap": numtap,
                    "accx_filtered": accx_filtered,
                }
            )

    data.update(
        {
            "time_interp": time_interp,
            "accx_interp": accx_interp,
            "accxs_filtered": accxs_filtered,
            "velx_interp": velx_interp,
        }
    )

    return data


def plot(data):
    for d in data:
        fig, ax = plt.subplots(3, 1)

        # Velocity
        ax[0].plot(
            d["time_interp"], d["velx_interp"] * 60.0 * 60.0 / 1000.0, label="velocity [km/h]"
        )
        ax[0].set(
            xlabel="Time [sec]",
            ylabel="Velocity [km/h]",
            yticks=np.arange(0, 25, 2.5),
            title=(d["type"] + " " + str(d["acc"])),
        )
        ax[0].grid()
        ax[0].legend()

        # Acceleration
        for accx_filtered in d["accxs_filtered"]:
            label = ""
            if accx_filtered["filter"] == "1d":
                label = accx_filtered["filter"] + ": " + str(accx_filtered["gain"])
            elif accx_filtered["filter"] == "butterworth":
                label = (
                    accx_filtered["filter"]
                    + ": cutoff="
                    + str(accx_filtered["cutoff"])
                    + ", numtap="
                    + str(accx_filtered["numtap"])
                )
            ax[1].plot(d["time_interp"], accx_filtered["accx_filtered"], label=label)
        ax[1].plot(d["time_interp"], d["accx_interp"], label="acc no filter")
        # ax[1].plot(d['time_acccan'],
        #            d['acccan'],
        #            label='acc CAN')
        # ax[1].plot(d['time_accreq'],
        #            d['accreq'],
        #            label='acc req')
        # for check delay
        # ax[1].set(xlabel='Time [sec]', ylabel='Acceleration [m/s]')
        # for check offset
        ax[1].set(
            xlabel="Time [sec]",
            ylabel="Acceleration [m/s^2]",
            ylim=(-0.25, 0.25),
            yticks=np.arange(-0.25, 0.25, 0.01),
        )
        ax[1].grid()
        ax[1].legend()

        # Pitch
        # ax[2].plot(d['time_pitch'],
        #            d['pitch'],
        #            label='pitch')
        # ax[2].set(xlabel='Time [sec]', ylabel='Pitch [rad]')
        # ax[2].grid()
        # ax[2].legend()

        plt.show()


def main():
    bags = read_files_from_csv(BASE_DIR + "data_list.csv")

    if USE_PICKLE:
        print("reading pickle ...")
        with open(BASE_DIR + "time-analysis-data.pickle", "rb") as f:
            data = pickle.load(f)
    else:
        data = []
        # accxs_filtered = []
        for bag in bags:
            d = analysis(bag)
            d.update({"type": bag["type"], "acc": bag["acc"]})
            data.append(d)

    with open(BASE_DIR + "time-analysis-data.pickle", "wb") as f:
        pickle.dump(data, f)

    plot(data)


if __name__ == "__main__":
    main()
