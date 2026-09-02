# lidar_imu_calibration

Offline lidar-to-IMU extrinsics in Faster-LIO form:

```text
p_imu = R * p_lidar + t
```

Write `mapping/extrinsic_R` / `mapping/extrinsic_T`. Do not enable online
extrinsic estimation in the odometer.

```bash
rosrun lidar_imu_calibration record_calib_bag.sh ~/scout-helios16-calib.bag
rosrun lidar_imu_calibration identify_lidar_imu_extrinsic.py \
  ~/scout-helios16-calib.bag \
  --prior-t 0,0,0.307 \
  --output scout_extrinsic.yaml \
  --report scout_extrinsic.json
```

Require `POINT_TYPE=XYZIRT` (`ring` + `timestamp` on `/rslidar_points`) and
`observable_rotation: true` before copying into `scout_helios16.yaml`.

Recording recipe: `memory/field/agilex/lidar-imu-extrinsic-recording.md`
(`xgc2-dev-memory`).
