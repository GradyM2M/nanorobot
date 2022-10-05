#!/usr/bin/env bash
BASE_TYOE=$1
LIDAR_TYPE=$2
echo "export BASE_TYPE=$BASE_TYOE" >> ~/.bashrc
echo "export LIDAR_TYPE=$LIDAR_TYPE" >> ~/.bashrc
source ~/.bashrc
