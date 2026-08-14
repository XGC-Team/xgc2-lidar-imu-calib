#!/usr/bin/env python3
"""Synthetic check that Wahba recovers a known lidar-to-IMU rotation."""

import numpy as np
from identify_lidar_imu_extrinsic import wahba_R, rot_log
from scipy.spatial.transform import Rotation


def main() -> None:
    rng = np.random.default_rng(0)
    R_true = Rotation.from_euler("xyz", [10, -5, 35], degrees=True).as_matrix()
    omega_l = rng.normal(size=(200, 3))
    omega_i = (R_true @ omega_l.T).T + rng.normal(scale=0.02, size=omega_l.shape)
    R_hat = wahba_R(omega_l, omega_i)
    err = np.linalg.norm(rot_log(R_hat.T @ R_true))
    assert err < 0.05, err
    print("synthetic rotation error rad", err)


if __name__ == "__main__":
    main()
