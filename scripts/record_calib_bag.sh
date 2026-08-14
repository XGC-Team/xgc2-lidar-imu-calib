#!/usr/bin/env bash
# Record a bag for offline LiDAR-IMU extrinsic identification.
# Requires the live vehicle: rslidar_sdk built with POINT_TYPE=XYZIRT.
set -euo pipefail

OUT="${1:-$HOME/xgc2-calib-$(date +%Y%m%d-%H%M%S).bag}"

echo "Recording to $OUT"
echo "Stand still 8-10 s after start, then:"
echo "  1) spin in place both ways"
echo "  2) figure-8 at 0.4-0.8 m/s"
echo "  3) a few hard accelerations and brakes"
echo "  4) if the body can pitch/roll even a little, do that"
echo "Need about 2-3 minutes. Ctrl-C to stop."

rosbag record -O "$OUT" \
  /rslidar_points \
  /imu/data_raw \
  /scout_status \
  /camera/color/image_raw/compressed \
  /tf \
  /tf_static
