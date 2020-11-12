# ladybug_camera

## Overview

This repository contains a ROS package for interfacing with the Flir Ladybug 5P 360 degree camera. 

### License

The source code is released under a [BSD 3-Clause license](ladybug_camera/LICENSE).

**Author: Steve Phillips<br />
Affiliation: [SDIC Lab](http://www.civil.uwaterloo.ca/snarasim/)<br />
Maintainer: Steve Phillips, sjphilli@uwaterloo.ca**

The ladybug_camera package has been tested under [ROS] Kinetic and Ubuntu 16.04. 

[![Build Status](https://travis-ci.com/BEAMRobotics/ladybug_camera.svg?token=xsJrkSdtugQUZAqp6xuh&branch=master)](https://travis-ci.com/BEAMRobotics/ladybug_camera)

<p align="center">
    <img src ="https://i.imgur.com/dAH1m3e.png" width="500"/>
</p>

## Usage

This package uses nodelets to interface with and publish images from the camera. The camera nodelet can be launched with:

* **camera.launch:**
    -  **`frame_id`** (string, default: `ladybug_link`)
    -  **`camera_mode`** (string, default: `raw`) 
        -   Set to either 'raw' or 'rectified'
    -  **`enable_debug_logging`** (bool, default: `false`)
    - **`diagnostics_desired_freq`** (int, default: `10`)
    - **`diagnostics_min_freq`** (int, default: `10`)
    - **`diagnostics_max_freq`** (int, default: `10`)
    - **`diagnostics_freq_tolerance`** (int, default: `0.1`)
    - **`diagnostics_window_size`** (int, default: `10`)
    - **`diagnostics_min_acceptable_delay`** (int, default: `0.0`)
    - **`diagnostics_max_acceptable_delay`** (int, default: `0.01`)

## Nodelets

### LadybugNodelet

#### Published Topics

* **`/ladybug/camera_[1-6]/camera_info`** ([sensor_msgs/CameraInfo])

	Camera info topic for each of the 6 cameras.

* **`/ladybug/camera_[1-6]/image_raw`** ([sensor_msgs/Image])

	The raw bayer images from each camera are each published on their own topic via. image_transport.
	
* **`/image_tiles`** ([ladybug_msgs/LadybugTiles])

  The LadybugTiles.msg contains an array of 6 sensor_msgs/Image
  messages, corresponding to each of the 6 tiles.
	
* **`/diagnostics`** ([diagnostic_msgs/DiagnosticArray])

	Publishes diagnostics information from the ladybug (temperature, pressure).
