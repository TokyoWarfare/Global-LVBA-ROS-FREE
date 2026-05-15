# Global-LVBA (ROS-free, Dockerised fork)

**Global LiDAR-Visual Bundle Adjustment** — globally consistent LiDAR + camera
BA refinement after LiDAR-inertial-visual odometry (e.g. FAST-LIVO2 or any
SLAM that produces per-frame camera + LiDAR poses).

This is a fork of [xuankuzcr/Global-LVBA](https://github.com/xuankuzcr/Global-LVBA)
with two changes:

1. **ROS dependency stripped.** The original is a catkin/ROS1 package; we
   replaced the few ROS APIs it actually used (NodeHandle::param,
   Publisher::publish, sensor_msgs::PointCloud2) with a header-only YAML-based
   shim (`include/ros_shim.h`). Source files are otherwise untouched. Builds
   with plain cmake, no ROS distribution required.
2. **Dockerised build environment.** All compile-time dependencies (Ceres,
   Eigen, OpenCV, PCL, GLEW/GLUT, legacy Sophus from source, etc.) are pinned
   in a `Dockerfile`. Build the image once, mount your source + dataset at
   runtime, iterate without polluting your host.

The algorithm itself (BALM LiDAR BA + visual BA via Ceres + point-plane
residual against the LiDAR cloud) is unchanged. See the [upstream paper /
repo](https://github.com/xuankuzcr/Global-LVBA) for the algorithmic details.

📬 Original authors: see upstream. Fork maintained by Pablo Vidaurre.

<p align="center">
  <img src="pics/lvba_cover.jpg" alt="Global-LVBA Cover" width="100%"/>
</p>

---

## Quick start

### Host prerequisites
- Linux with Docker installed
- For GPU runs: NVIDIA driver + `nvidia-container-toolkit`. CPU-only also works
  (drop `--gpus all` from the run script).

### One-time setup

```bash
# 1. Clone the repo + SiftGPU submodule
git clone https://github.com/TokyoWarfare/Global-LVBA-ROS-FREE.git ~/lvba_ws/Global-LVBA
cd ~/lvba_ws/Global-LVBA
git submodule update --init --recursive

# 2. Build the Docker image (~10-20 min first time; only deps install,
#    no source compiled in the image)
docker build -t lvba:dev .

# 3. Drop into the container, compile SiftGPU + LVBA (source is mounted
#    from your host, so build artefacts land in ~/lvba_ws/Global-LVBA/build/)
docker run --gpus all --rm -it \
    -v ~/lvba_ws/Global-LVBA:/opt/lvba \
    lvba:dev
# inside the container:
cd /opt/lvba/src/SiftGPU && mkdir -p build && cd build && cmake .. && make -j
cd /opt/lvba && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j
exit
```

After the second `make`, the LVBA binary is at `~/lvba_ws/Global-LVBA/build/lvba_run`
on your host (same path the container sees, because the source is bind-mounted).

### Running on a dataset

The cleanest workflow is **one bash script + one YAML per dataset**, both
living in the dataset directory:

```
/path/to/your/dataset/
├── all_image/                     # timestamped PNGs + image_poses.txt
├── all_pcd_body/                  # timestamped PCDs + lidar_poses.txt
├── Colmap/colmap_sub5.db          # COLMAP-format match.db
├── lvba_config.yaml               # intrinsics, extrinsics, BA params
└── run_lvba.sh                    # self-locating launcher (copied from scripts/)
```

Then from anywhere on your host:

```bash
/path/to/your/dataset/run_lvba.sh
```

The script bind-mounts the dataset folder into the container at
`/opt/lvba/dataset/<seq_name>/`, runs `lvba_run` with the YAML, tees the
output to `lvba_run.log` in the dataset folder, and pauses on exit so you
can read the result.

A template lives at `scripts/run_lvba.sh.template` — copy it into each
dataset folder, edit the path-to-repo and image-tag lines if they're not the
defaults. The template self-locates so it survives the dataset being moved.

---

## Dataset layout

Per the original Global-LVBA convention:

```
<dataset>/
├── all_image/
│   ├── <timestamp>.png       images named by capture timestamp
│   └── image_poses.txt        TUM-format poses: ts tx ty tz qx qy qz qw
├── all_pcd_body/
│   ├── <timestamp>.pcd        point clouds in body frame, timestamp-named
│   └── lidar_poses.txt        TUM-format poses (one row per PCD, in order)
├── Colmap/
│   ├── colmap_sub5.db         COLMAP match.db (images + keypoints +
│   │                          two_view_geometries tables)
│   └── sparse/                LVBA writes refined poses here (images.txt,
│                              points3D.txt) if colmap_output.enable: true
├── depth/                     LVBA writes per-image depth maps (optional)
├── reproj/                    LVBA writes reprojection visualisations
├── lvba_config.yaml           your per-dataset config
└── run_lvba.sh                copy of scripts/run_lvba.sh.template
```

`image_poses.txt` and `lidar_poses.txt` are TUM format, one row per pose:
```
<timestamp_secs>  <tx> <ty> <tz>  <qx> <qy> <qz> <qw>
```
Image rows must be sorted by timestamp; PCD rows must align row-for-row with
the sorted PCDs in `all_pcd_body/`. The match.db references images by name
(`<timestamp>.png`), so as long as those filenames are consistent across
all_image/ and the db, you're good.

---

## Per-dataset YAML

`config/config.yaml` is the upstream default (CBD_Building_01). For your own
sequences, create a new YAML alongside (the matching script's binary takes
the path as its first arg). Key fields:

```yaml
cam_model:
  cam_width, cam_height, scale
  cam_fx, cam_fy, cam_cx, cam_cy     # pinhole intrinsics
  cam_d0, cam_d1, cam_d2, cam_d3      # Brown-Conrady distortion

extrin_calib:
  extrinsic_T, extrinsic_R            # IMU-to-LiDAR (if used)
  Rcl, Pcl                            # LiDAR-to-camera

data_config:
  data_path: "dataset/<seq_name>/"    # relative to /opt/lvba
  colmap_db_path: "Colmap/colmap_sub5.db"
  image_sample_step: 5                # use every Nth image
  enable_lidar_ba: false              # BALM LiDAR-side optimisation
  enable_visual_ba: true              # main visual + point-plane BA

window_ba, BALM_stage1, BALM_stage2:  # tuning knobs; see upstream paper
track_fusion:                          # track filter thresholds
colmap_output:
  enable: true                         # write refined poses to
                                       # Colmap/sparse/images.txt and a
                                       # merged colored cloud
```

---

## Generating the `match.db`

Several options:

1. **COLMAP** (the upstream recommendation). Standard `feature_extractor` +
   `exhaustive_matcher` produces a compatible `.db`. Slow on large datasets.
2. **GLIM's LVBA match.db generator** (this fork's intended pairing, if you
   use the [GLIM](https://github.com/koide3/glim) mobile-mapping suite).
   The Tools > Utils > LVBA match.db generator panel produces a COLMAP-schema
   `.db` from any image folder via OpenCV-SIFT (COLMAP-tuned parameters) or
   SuperPoint+LightGlue, plus F-matrix RANSAC verification and spatial-radius
   pair selection. ~10× faster than COLMAP-exhaustive on typical MMS data
   and reaches equivalent or better post-BA reprojection. See the GLIM repo
   for the matcher tool documentation.
3. **LVBA's built-in SiftGPU** (set `enable: true` for the rebuild path in
   LVBA's code). Convenient fallback but quality is below COLMAP / GLIM's
   matcher per the upstream README.

LVBA reads only the `images`, `keypoints`, and `two_view_geometries` tables
from the `.db`. The schema is documented at
[COLMAP database docs](https://colmap.github.io/database.html).

---

## What got patched (for upstream consumers)

If you're cross-referencing this fork against upstream:

- `include/ros_shim.h` — new, header-only YAML-config + no-op publisher
  stubs. Replaces the 4-5 ROS APIs the original used.
- `src/main.cpp` — drops `ros::init`/`NodeHandle`, takes a YAML path as
  argv[1].
- `include/dataset_io.h` / `src/dataset_io.cpp` — `NodeHandle&` swapped for
  `Config&` from the shim. Otherwise identical.
- `include/lvba_system.h` / `src/lvba_system.cpp` — same swap. One algorithm
  edit: `EigenQuaternionManifold` (Ceres 2.2+) → `EigenQuaternionParameterization`
  (Ceres 2.0, which Ubuntu 22.04 ships) for compatibility.
- `CMakeLists.txt` — replaced catkin scaffolding with bare cmake +
  `find_package(yaml-cpp)`.
- `Dockerfile` (new) + `scripts/run_lvba.sh.template` (new) — build env + runner.
- `package.xml`, `launch/`, `rviz_cfg/` — deleted (ROS-only artefacts).

The visualization publishers were no-ops in the shim, so the algorithm's
output is identical to running the upstream version. Tested on
Retail_Street from the LVBA-Dataset.

---

## Reference dataset

The upstream **LVBA-Dataset**
([Google Drive](https://drive.google.com/drive/folders/19fYG4z666hcxyP6StVXs-ZOI2cbHsU5J?usp=drive_link))
is the easiest way to validate a build. Drop `Retail_Street/` into
`dataset/`, copy `scripts/run_lvba.sh.template` into it, and run.

---

## License

MIT, same as upstream. See [`LICENSE`](LICENSE).
