# 明天实车：怎么录 Helios-16 + IMU 包

这份 2026-08-15 的场外包 **不能** 再用来估外参，也不要再在它上面猜偏航。包只有 60 s 平面运动，几乎只有 `ω_z`，Wahba 看不出 `R_z`，平移杠杆也看不出。下一步外参必须靠 **新录的激励包**。

机载驱动已经改成官方 `POINT_TYPE=XYZIRT`（`xgc2-robot-agilex` 里的 `rslidar_sdk/CMakeLists.txt`）。新包必须带 **ring** 和 **逐点 timestamp**，不要再录成只有 `x y z intensity` 的 XYZI。

## 1. 出发前确认驱动

车上重新编译并启动的必须是 XYZIRT，不是 XYZI。

```bash
# 编译日志里应有：
# -- POINT_TYPE is XYZIRT

rostopic echo -n 1 /rslidar_points/fields
```

`fields` 里必须同时有：

| 字段 | 来源 |
| --- | --- |
| `x y z intensity` | 一直都有 |
| `ring` | XYZIRT 才发布，官方通道号 |
| `timestamp` | XYZIRT 才发布，官方包时间，单位见驱动 |

没有 `ring` / `timestamp` 就停，不要录。不要事后自己补时间戳。

再看一眼：

```bash
rostopic hz /rslidar_points /imu/data_raw
# 雷达约 10 Hz，IMU 约 200 Hz
```

`rslidar_sdk` 配置应是 `lidar_type: RSHELIOS_16`，`frame_id: rslidar`，点云话题 `/rslidar_points`。IMU 话题 `/imu/data_raw`。

## 2. 录什么话题

```bash
rosrun lidar_imu_calib record_calib_bag.sh ~/scout-helios16-calib-$(date +%Y%m%d-%H%M).bag
```

脚本会录：

- `/rslidar_points`
- `/imu/data_raw`
- `/scout_status`
- `/camera/color/image_raw/compressed`（辅助看车在干什么，不参加标定）
- `/tf` `/tf_static`

需要底盘里程对照时再加 `/odom`（如果车上有）。

## 3. 车要怎么开（这是这份旧包缺的）

总时长 **2–3 分钟**，不要再录 60 秒直线/慢转。

1. **静置 8–10 s**  
   车完全停住。给 IMU 偏置和重力方向。
2. **原地左右转**  
   各转够 1–2 圈，角速度尽量上去（接近日常转弯上限）。这是 `ω_z`。
3. **8 字 / 来回转弯前进**  
   0.4–0.8 m/s，多转几个弯。让雷达 ICP 和陀螺在时间上对得上。
4. **加减速、急停几次**  
   给线加速度，否则平移杠杆不可观。
5. **尽量给一点俯仰/侧倾**  
   过减速带、压一块斜垫、短坡都行。  
   **没有 `ω_x、ω_y`，绕 Z 的安装偏航仍然不可观。**  
   平面车上这是最容易漏的一步。

不要只在走廊里慢慢开过去。那就是今天这份包。

## 4. 录完先查，再给里程计

```bash
rosbag info ~/scout-helios16-calib-*.bag

rosrun lidar_imu_calib identify_lidar_imu_extrinsic.py \
  ~/scout-helios16-calib-*.bag \
  --prior-t 0,0,0.307 \
  --output /tmp/scout_extrinsic.yaml \
  --report /tmp/scout_extrinsic.json
```

只有 `observable_rotation: true` 才把 `extrinsic_R` / `extrinsic_T` 抄进 Faster-LIO 的 `scout_helios16.yaml`，并保持 `extrinsic_est_en: false`。

`observable_rotation: false` 或脚本丢掉拟合 yaw 时，**不要**把结果当标定。再录一包，把俯仰/侧倾补上。

里程计里不要打开在线外参估计。几何关系是装上去就固定的。

## 5. 和今天这份包的差别

| | 今天 `agilex-all-20260815-000809` | 明天要的包 |
| --- | --- | --- |
| 点类型 | XYZI，事后只补了 organized `ring` | 驱动直接 XYZIRT |
| 逐点时间 | 没有，LIO 用 yaw 去畸变 | 官方 `timestamp` |
| 运动 | 约 60 s 平面 | 2–3 min，转 + 8 字 + 加减速 + 尽量俯仰 |
| 外参 | 不可观，已放弃用它拟合 | 旋转有机会可观；平移仍可能弱 |

仿真里雷达–IMU 外参用 URDF **真值**（`R = I`，`t = (0, 0, 0.187)`）做里程计对照。那不是实车外参。
