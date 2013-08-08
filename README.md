SENEKA
======

## Description
This stack contains several drivers and algorithms that have been developed at Fraunhofer IPA for the SENEKA project.

The different algorithms and drivers are described in more detail in their respective Readme's or the source code.

## Installation
As this stack contains dependencies to non-released packages and libraries, you need to put some extra effort into installing those correctly.

### Installing the optris_drivers package required for the ThermoVideoManager
Please call the following commands from the folder where you cloned the seneka repository in:
```bash
rosinstall . seneka/seneka.rosinstall
rosdep install seneka
rosmake seneka
```

### Building the `seneka_sony_camera node`
The `seneka_sony_camera` node requires the eBus SDK which is provided by Pleora (http://www.pleora.com/our-products/ebus-sdk).
Thus, the package is not automatically built using the ROS build system.

If you want to build it, you need to install the eBus SDK, `source set_puregev_env` in `/opt/pleora/ebus_sdk/bin` and build it by hand.
However, this is only required for the sensor nodes.
Thus, you do not need to do this for developing code other than that.

### Now you're good to go
If everything has been set up correctly, the SENEKA stack should now be built.

If not, check that you have checked out the optris_drivers package, that this package builds, that you can link to the respective libraries therein as well as to libudev-dev.
Also, if you want to build `seneka_sony_camera`, make sure you have installed the required libraries and that the respective environment variables are sourced.
