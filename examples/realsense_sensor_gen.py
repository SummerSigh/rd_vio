#!/usr/bin/env python3
"""
realsense_sensor_gen.py
Robust EuroC‑style YAML generator for Intel® RealSense™ cameras.

• Auto‑detects best video stream (IR > fisheye > color)
• Uses IMU if present; falls back to cam‑only YAML otherwise
• Outputs plain‑number YAML (no NumPy tags) compatible with VIO / SLAM tools
"""

import argparse, math, yaml, numpy as np, pyrealsense2 as rs
from typing import Any


# ---------- utilities -------------------------------------------------------
def to_builtin(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return [to_builtin(x) for x in obj.tolist()]
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, (list, tuple)):
        return [to_builtin(x) for x in obj]
    if isinstance(obj, dict):
        return {k: to_builtin(v) for k, v in obj.items()}
    return obj


def quat_from_R(R: np.ndarray) -> list[float]:
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s,
                0.25 * s]
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [0.25 * s,
                (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s]
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 1] + R[1, 0]) / s,
                0.25 * s,
                (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s,
            0.25 * s,
            (R[1, 0] - R[0, 1]) / s]


def diag3(v):
    return [v[0], 0.0, 0.0,
            0.0, v[1], 0.0,
            0.0, 0.0, v[2]]


# ---------- main ------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default="realsense_sensor.yaml",
                    help="YAML file to write")
    args = ap.parse_args()

    ctx = rs.context()
    if not ctx.devices:
        raise RuntimeError("No RealSense device found")

    dev = ctx.devices[0]
    print("Using", dev.get_info(rs.camera_info.name),
          "FW", dev.get_info(rs.camera_info.firmware_version))

    # Enumerate profiles
    gyro_prof = accel_prof = video_prof = None
    video_priority = {rs.stream.infrared: 0, rs.stream.fisheye: 1, rs.stream.color: 2}

    for sensor in dev.sensors:
        for p in sensor.get_stream_profiles():
            st = p.stream_type()
            if st == rs.stream.gyro and (gyro_prof is None or p.fps() > gyro_prof.fps()):
                gyro_prof = p
            elif st == rs.stream.accel and (accel_prof is None or p.fps() > accel_prof.fps()):
                accel_prof = p
            elif st in video_priority:
                if (video_prof is None or
                        video_priority[st] < video_priority[video_prof.stream_type()]):
                    video_prof = p

    if video_prof is None:
        raise RuntimeError("No IR, fisheye or color stream found")

    vsp = video_prof.as_video_stream_profile()
    vid_st = video_prof.stream_type()
    print(f"Selected video stream: "
          f"{vid_st.name.lower()} {vsp.width()}×{vsp.height()} @ {vsp.fps()} Hz")

    imu_ok = gyro_prof is not None and accel_prof is not None
    if not imu_ok:
        print("⚠️  No IMU streams detected – YAML will have cam0 only.")

    # Configure pipeline exactly with chosen profiles
    cfg = rs.config()
    cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
    cfg.enable_stream(vid_st, vsp.width(), vsp.height(), vsp.format(), vsp.fps())

    if imu_ok:
        cfg.enable_stream(rs.stream.gyro,  gyro_prof.format(),  gyro_prof.fps())
        cfg.enable_stream(rs.stream.accel, accel_prof.format(), accel_prof.fps())

    pipe = rs.pipeline(ctx)
    prof = pipe.start(cfg)

    cam_p = prof.get_stream(vid_st).as_video_stream_profile()
    cim = cam_p.get_intrinsics()
    fu, fv, cu, cv = cim.fx, cim.fy, cim.ppx, cim.ppy
    resolution = [cim.width, cim.height]
    distortion = cim.coeffs[:4]
    dist_model = "radtan" if cim.model == rs.distortion.brown_conrady else "none"

    # Extrinsics IMU → cam0
    if imu_ok:
        gyro_active  = prof.get_stream(rs.stream.gyro ).as_motion_stream_profile()
        accel_active = prof.get_stream(rs.stream.accel).as_motion_stream_profile()
        ext = gyro_active.get_extrinsics_to(cam_p)
        R_bc = np.asarray(ext.rotation, dtype=float).reshape(3, 3)
        p_bc = list(map(float, ext.translation))
        q_bc = quat_from_R(R_bc)
    else:
        R_bc = np.eye(3)
        p_bc = [0.0, 0.0, 0.0]
        q_bc = [0.0, 0.0, 0.0, 1.0]

    T_BS = np.column_stack((R_bc, p_bc)).reshape(-1).tolist() + [0, 0, 0, 1]

    data = {
        "cam0": {
            "T_BS": {"cols": 4, "rows": 4, "data": T_BS},
            "resolution": resolution,
            "camera_model": "pinhole",
            "distortion_model": dist_model,
            "camera_distortion_flag": 1 if dist_model != "none" else 0,
            "intrinsics": [fu, fv, cu, cv],
            "distortion": distortion,
            "camera_readout_time": 0.0,
            "time_offset": 0.0,
            "extrinsic": {"q_bc": q_bc, "p_bc": p_bc},
            "noise": [0.5, 0.0, 0.0, 0.5],
        }
    }

    if imu_ok:
        gim = gyro_active.get_motion_intrinsics()
        aim = accel_active.get_motion_intrinsics()
        data["imu"] = {
            "gyroscope_noise_density":   math.sqrt(gim.noise_variances[0]),
            "gyroscope_random_walk":     math.sqrt(gim.bias_variances[0]),
            "accelerometer_noise_density": math.sqrt(aim.noise_variances[0]),
            "accelerometer_random_walk":   math.sqrt(aim.bias_variances[0]),
            "gyroscope_bias":     [0.0, 0.0, 0.0],
            "accelerometer_bias": [0.0, 0.0, 0.0],
            "noise": {
                "cov_g":  diag3(gim.noise_variances),
                "cov_a":  diag3(aim.noise_variances),
                "cov_bg": diag3(gim.bias_variances),
                "cov_ba": diag3(aim.bias_variances),
            },
            "extrinsic": {"q_bi": [0, 0, 0, 1], "p_bi": [0, 0, 0]},
        }

    pipe.stop()

    with open(args.output, "w") as f:
        yaml.safe_dump(to_builtin(data), f, default_flow_style=False)

    print("✅  YAML written to", args.output)


if __name__ == "__main__":
    main()
