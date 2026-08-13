# Phone IMU Based Orientation Control of AgileX LIMO Pro

A ROS 2 based implementation for controlling the orientation of an AgileX LIMO Pro mobile robot using the orientation of an Android smartphone.

## Overview

The system acquires accelerometer and gyroscope data from an Android smartphone using phyphox and transfers the data to a Linux computer over Wi-Fi.

The phone IMU data is published as a ROS 2 `sensor_msgs/msg/Imu` message.

The phone orientation is then used to generate a target heading for the LIMO Pro. The LIMO orientation is obtained from `/odom`, and a proportional controller generates the angular velocity command published to `/cmd_vel`.

## System Architecture

Android Phone
    |
    | IMU
    v
phyphox
    |
    | HTTP
    v
ROS 2 Foxy
    |
    | /phone/imu
    v
Phone Yaw Estimation
    |
    v
Target LIMO Heading
    |
    | heading error
    v
P Controller
    |
    | /cmd_vel
    v
AgileX LIMO Pro

## Hardware

- Android smartphone with accelerometer and gyroscope
- Linux computer
- AgileX LIMO Pro
- Wi-Fi network

## Software

- Ubuntu 20.04
- ROS 2 Foxy
- Python 3
- phyphox
- Git

## ROS 2 Topics

### Phone IMU

```text
/phone/imu

##Message type:

sensor_msgs/msg/Imu

## LIMO odometry

/odom


## Status


The current implementation successfully controls the LIMO Pro orientation using phone orientation.

## Future Work
Quaternion-based phone orientation estimation
IMU sensor fusion
Magnetometer integration
Filtering
PID control
ROS 2 bag recording
Experimental evaluation
Phone tilt based linear velocity control


