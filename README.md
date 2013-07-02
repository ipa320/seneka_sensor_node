SENEKA
======

## Description
This stack contains several drivers and algorithms that have been developed at Fraunhofer IPA for the SENEKA project.

The different algorithms and drivers are described in more detail in their respective Readme's

## Installation
As this stack contains dependencies to non-released stacks and packages, please call the following commands from the folder where you cloned the seneka repository in:
```bash
rosinstall . seneka/seneka.rosinstall
rosdep install seneka
rosmake seneka
```
If everything has been set up correctly, the SENEKA stack should now be built.
If not, check that you have checked out the optris_drivers package, that this package builds, that you can link to the respective libraries therein as well as to libudev-dev.
