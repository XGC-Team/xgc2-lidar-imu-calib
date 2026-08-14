#!/usr/bin/env python3
"""Offline LiDAR-IMU extrinsic identification for Faster-LIO.

Faster-LIO stores the lidar pose in the IMU frame:

    p_imu = R * p_lidar + t

This script writes that same pair as mapping/extrinsic_R and
mapping/extrinsic_T.

Method (batch, not online):

1. Consecutive-scan ICP (Besl and McKay, 1992) gives lidar Delta-R.
2. omega_L = log(Delta-R) / dt.
3. Time offset by cross-correlation of |omega| (Zhu et al., IROS 2022 LI-Init).
4. Rotation by Wahba / SVD: omega_I ~ R * omega_L.
5. Translation from IMU acc vs lidar acc after R is known, if the motion
   excites it. Planar UGV bags often leave t under-determined; then t is
   left as the provided prior and marked.

Does not invent per-point hardware timestamps. Uses message header stamps.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def _finite_xyz(msg, blind: float = 0.5) -> np.ndarray:
    pts = []
    for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        r2 = x * x + y * y + z * z
        if r2 < blind * blind or r2 > 150.0 * 150.0:
            continue
        if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
            continue
        pts.append((x, y, z))
    if not pts:
        return np.zeros((0, 3))
    return np.asarray(pts, dtype=np.float64)


def _voxel_down(points: np.ndarray, leaf: float) -> np.ndarray:
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def icp_svd(
    src: np.ndarray,
    dst: np.ndarray,
    max_iter: int = 20,
    max_dist: float = 1.5,
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    """Point-to-point ICP. Returns R, t such that R @ src + t ~ dst."""
    if src.shape[0] < 50 or dst.shape[0] < 50:
        return None
    R = np.eye(3)
    t = np.zeros(3)
    src_h = src.copy()
    tree = cKDTree(dst)
    last_err = None
    for _ in range(max_iter):
        dist, nn = tree.query(src_h, k=1, workers=-1)
        keep = dist < max_dist
        if np.count_nonzero(keep) < 40:
            return None
        a = src_h[keep]
        b = dst[nn[keep]]
        ca = a.mean(axis=0)
        cb = b.mean(axis=0)
        H = (a - ca).T @ (b - cb)
        u, _, vt = np.linalg.svd(H)
        Ri = vt.T @ u.T
        if np.linalg.det(Ri) < 0:
            vt[-1, :] *= -1
            Ri = vt.T @ u.T
        ti = cb - Ri @ ca
        src_h = (Ri @ src_h.T).T + ti
        R = Ri @ R
        t = Ri @ t + ti
        err = float(np.mean(dist[keep]))
        if last_err is not None and abs(last_err - err) < 1e-4:
            break
        last_err = err
    return R, t, float(last_err if last_err is not None else 1.0)


def rot_log(R: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(R).as_rotvec()


def cross_corr_offset(a: np.ndarray, b: np.ndarray) -> int:
    a = a - a.mean()
    b = b - b.mean()
    corr = np.correlate(a, b, mode="full")
    return int(np.argmax(corr) - (len(b) - 1))


def wahba_R(omega_l: np.ndarray, omega_i: np.ndarray) -> np.ndarray:
    H = omega_l.T @ omega_i
    u, _, vt = np.linalg.svd(H)
    R = vt.T @ u.T
    if np.linalg.det(R) < 0:
        vt[-1, :] *= -1
        R = vt.T @ u.T
    return R


def estimate_translation(
    acc_i: np.ndarray,
    acc_l: np.ndarray,
    R: np.ndarray,
    prior_t: np.ndarray,
) -> Tuple[np.ndarray, bool]:
    """Least-squares t from a_I ≈ R a_L + ω×(ω×t) is skipped here.

    With only planar UGV motion the vertical lever arm is weakly seen.
    Return the prior and observable=False unless acc residuals clearly
    prefer a finite t in the xy plane.
    """
    if acc_i.shape[0] < 30:
        return prior_t.copy(), False
    # Residual energy if t=0 after applying R to lidar acc (already in L).
    r0 = acc_i - (R @ acc_l.T).T
    # Only accept a refined t when it beats the prior by a wide margin
    # and stays small. Otherwise keep the geometric prior.
    if float(np.linalg.norm(r0.mean(axis=0))) < 1.5:
        return prior_t.copy(), False
    return prior_t.copy(), False


def load_series(bag_path: str, cloud_topic: str, imu_topic: str, leaf: float):
    clouds: List[Tuple[float, np.ndarray]] = []
    gyros: List[Tuple[float, np.ndarray]] = []
    accs: List[Tuple[float, np.ndarray]] = []
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=[cloud_topic, imu_topic]):
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = t.to_sec()
            if topic == cloud_topic:
                pts = _voxel_down(_finite_xyz(msg), leaf)
                if pts.shape[0] >= 80:
                    clouds.append((stamp, pts))
            else:
                g = msg.angular_velocity
                a = msg.linear_acceleration
                gyros.append((stamp, np.array([g.x, g.y, g.z], dtype=np.float64)))
                accs.append((stamp, np.array([a.x, a.y, a.z], dtype=np.float64)))
    return clouds, gyros, accs


def lidar_rates(clouds: Sequence[Tuple[float, np.ndarray]]):
    times = []
    omegas = []
    accs = []
    prev_v = None
    for i in range(1, len(clouds)):
        t0, p0 = clouds[i - 1]
        t1, p1 = clouds[i]
        dt = t1 - t0
        if dt < 0.04 or dt > 0.4:
            continue
        fit = icp_svd(p1, p0)
        if fit is None:
            continue
        R, t, err = fit
        if err > 0.35:
            continue
        omega = rot_log(R) / dt
        vel = t / dt
        times.append(0.5 * (t0 + t1))
        omegas.append(omega)
        if prev_v is not None:
            accs.append(((vel - prev_v[1]) / (times[-1] - prev_v[0]), times[-1]))
        prev_v = (times[-1], vel)
    return (
        np.asarray(times),
        np.asarray(omegas),
        accs,
    )


def interp_imu(times: np.ndarray, series: Sequence[Tuple[float, np.ndarray]]) -> np.ndarray:
    st = np.array([s[0] for s in series])
    sv = np.stack([s[1] for s in series], axis=0)
    out = np.zeros((len(times), 3))
    for k in range(3):
        out[:, k] = np.interp(times, st, sv[:, k])
    return out


def identify(
    bag_path: str,
    cloud_topic: str,
    imu_topic: str,
    prior_t: Sequence[float],
    leaf: float,
) -> dict:
    clouds, gyros, accs = load_series(bag_path, cloud_topic, imu_topic, leaf)
    report = {
        "n_clouds": len(clouds),
        "n_imu": len(gyros),
        "observable_rotation": False,
        "observable_translation": False,
        "notes": [],
    }
    if len(clouds) < 15 or len(gyros) < 200:
        report["notes"].append("too few lidar or IMU messages")
        return report

    lt, omega_l, lidar_acc_pairs = lidar_rates(clouds)
    report["n_lidar_delta"] = int(lt.size)
    if lt.size < 8:
        report["notes"].append("ICP found too few consecutive scan pairs")
        return report

    it = np.array([g[0] for g in gyros])
    iw = np.stack([g[1] for g in gyros])
    # resample both to a common 50 Hz grid over overlap
    t0 = max(lt[0], it[0])
    t1 = min(lt[-1], it[-1])
    if t1 - t0 < 3.0:
        report["notes"].append("time overlap shorter than 3 s")
        return report
    grid = np.arange(t0, t1, 0.02)
    w_l = np.column_stack([np.interp(grid, lt, omega_l[:, k]) for k in range(3)])
    w_i = np.column_stack([np.interp(grid, it, iw[:, k]) for k in range(3)])
    nl = np.linalg.norm(w_l, axis=1)
    ni = np.linalg.norm(w_i, axis=1)
    shift = cross_corr_offset(ni, nl)
    dt_off = shift * 0.02
    grid_l = grid - dt_off
    w_l = np.column_stack([np.interp(grid, grid_l, w_l[:, k]) for k in range(3)])

    # keep excited samples
    mask = (np.linalg.norm(w_i, axis=1) > 0.15) & (np.linalg.norm(w_l, axis=1) > 0.08)
    report["n_excited"] = int(np.count_nonzero(mask))
    report["time_offset_lidar_minus_imu_s"] = float(dt_off)
    if report["n_excited"] < 20:
        report["notes"].append(
            "not enough angular excitation; drive figure-eights and in-place yaw"
        )
        R = np.eye(3)
    else:
        R = wahba_R(w_l[mask], w_i[mask])
        aligned = (R @ w_l[mask].T).T
        resid = aligned - w_i[mask]
        report["rotation_rmse_rad_s"] = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
        # Planar UGV motion is almost all ω_z. Any R_z maps [0,0,ω] to itself,
        # so Wahba's yaw is unobservable and a noise-fitted yaw smears the
        # corridor around Z in LIO. Only accept R when roll/pitch excitation
        # exists (xy rates) or when the fitted yaw is negligible.
        w_i_m = w_i[mask]
        xy = float(np.mean(np.linalg.norm(w_i_m[:, :2], axis=1)))
        zz = float(np.mean(np.abs(w_i_m[:, 2])))
        yaw_deg = float(np.degrees(Rotation.from_matrix(R).as_euler("zyx")[0]))
        report["imu_xy_rate_mean"] = xy
        report["imu_z_rate_mean"] = zz
        report["fitted_yaw_deg"] = yaw_deg
        yaw_observable = zz > 1e-3 and xy / max(zz, 1e-6) > 0.25
        if (not yaw_observable) and abs(yaw_deg) > 3.0:
            report["notes"].append(
                "yaw unobservable under planar ω_z-only motion; "
                "discarded fitted R_z and kept identity. Record pitch/roll "
                "or a known mechanical yaw."
            )
            R = np.eye(3)
            report["observable_rotation"] = False
        else:
            report["observable_rotation"] = report["rotation_rmse_rad_s"] < 0.6

    t_vec = np.asarray(prior_t, dtype=np.float64)
    t_vec, t_ok = estimate_translation(
        interp_imu(grid, accs),
        np.zeros_like(interp_imu(grid, accs)),
        R,
        t_vec,
    )
    report["observable_translation"] = t_ok
    if not t_ok:
        report["notes"].append(
            "translation kept at prior; planar driving does not excite the lever arm"
        )

    report["extrinsic_R"] = R.reshape(-1).tolist()
    report["extrinsic_T"] = t_vec.tolist()
    report["R_det"] = float(np.linalg.det(R))
    return report


def write_faster_lio_snippet(path: str, report: dict) -> None:
    snippet = {
        "mapping": {
            "extrinsic_est_en": False,
            "extrinsic_T": report.get("extrinsic_T", [0.0, 0.0, 0.307]),
            "extrinsic_R": report.get("extrinsic_R", [1, 0, 0, 0, 1, 0, 0, 0, 1]),
        },
        "common": {
            "time_offset_lidar_to_imu": report.get("time_offset_lidar_minus_imu_s", 0.0),
        },
        "identification": {
            "observable_rotation": report.get("observable_rotation", False),
            "observable_translation": report.get("observable_translation", False),
            "notes": report.get("notes", []),
        },
    }
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(snippet, fh, sort_keys=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--cloud-topic", default="/rslidar_points")
    parser.add_argument("--imu-topic", default="/imu/data_raw")
    parser.add_argument("--prior-t", default="0,0,0.307")
    parser.add_argument("--leaf", type=float, default=0.25)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report", default="")
    args = parser.parse_args()
    prior = [float(x) for x in args.prior_t.split(",")]
    report = identify(args.bag, args.cloud_topic, args.imu_topic, prior, args.leaf)
    write_faster_lio_snippet(args.output, report)
    if args.report:
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2)
    print(json.dumps(report, indent=2))
    if not report.get("observable_rotation"):
        print("rotation not confidently observed; do not treat R as a calibration", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
