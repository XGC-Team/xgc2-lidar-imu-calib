# lidar_imu_calib

离线辨识雷达相对 IMU 的外参，输出和 Faster-LIO 相同的定义：

```
p_imu = R * p_lidar + t
```

对应 yaml 里的 `mapping/extrinsic_R`、`mapping/extrinsic_T`。几何关系是装上去就固定的，**不要**在里程计里在线估外参。

方法：相邻帧 ICP（Besl & McKay 1992）得到雷达角速度，再和 IMU 陀螺做时间互相关与 Wahba/SVD 求 `R`（与 LI-Init / Zhu IROS 2022 的角速度对齐同一类）。平移在平面车上往往激励不够，默认保留先验 `t`。

## 明天实车怎么录

1. `rslidar_sdk` 必须用 `POINT_TYPE=XYZIRT` 编译，让 `/rslidar_points` 带官方 `ring` 和包时间。
2. 先静止 8–10 秒，再原地左右转、走 8 字、加减速。尽量给一点俯仰/侧倾。录 2–3 分钟。
3. 执行：

```bash
rosrun lidar_imu_calib record_calib_bag.sh ~/scout-helios16-calib.bag
```

4. 辨识：

```bash
rosrun lidar_imu_calib identify_lidar_imu_extrinsic.py \
  ~/scout-helios16-calib.bag \
  --prior-t 0,0,0.307 \
  --output /tmp/scout_extrinsic.yaml \
  --report /tmp/scout_extrinsic.json
```

5. 看 `observable_rotation`。为真才把 `extrinsic_R` / `extrinsic_T` 抄进 Faster-LIO 的 `scout_helios16.yaml`，并保持 `extrinsic_est_en: false`。

当前这份场外 60 秒包主要是平面运动，**平移不可观**，旋转也可能激励不够。脚本会写明，不要强行当标定结果。
