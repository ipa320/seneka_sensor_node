#!/bin/bash

cp *.rules /etc/udev/rules.d/
/etc/init.d/udev restart
