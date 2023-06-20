#!/bin/bash

sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /sys/bus/i2c/drivers/ina3221/7-0040/hwmon/hwmon5/in2_input
sudo chmod 777 /sys/bus/i2c/drivers/ina3221/7-0040/hwmon/hwmon5/curr2_input
echo userspace | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 115200 | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
echo 1907200 | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
sudo chmod 777 /sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed
