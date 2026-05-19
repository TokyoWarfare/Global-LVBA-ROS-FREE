# Global-LVBA (ROS-free, Dockerised fork)

**Global LiDAR-Visual Bundle Adjustment** — globally consistent LiDAR + camera
BA refinement after LiDAR-inertial-visual odometry (e.g. FAST-LIVO2 or any
SLAM that produces per-frame camera + LiDAR poses).

This is a fork of [xuankuzcr/Global-LVBA](https://github.com/xuankuzcr/Global-LVBA)
with several changes:

1. **ROS dependency stripped.** The original is a catkin/ROS1 package; we
   replaced the few ROS APIs it actually used (NodeHandle::param,
   Publisher::publish, sensor_msgs::PointCloud2) with a header-only YAML-based
   shim (`include/ros_shim.h`). Source files are otherwise untouched. Builds
   with plain cmake, no ROS distribution required.
2. **Dockerised build environment.** All compile-time dependencies (Ceres,
   Eigen, OpenCV, PCL, GLEW/GLUT, legacy Sophus from source, etc.) are pinned
   in a `Dockerfile`. Build the image once, mount your source + dataset at
   runtime, iterate without polluting your host.
3. **Pre-rendered depth-map support** (`use_existing_depthmap` YAML flag) to
   bypass LVBA's native voxel-projection depth gen when the upstream pipeline
   (e.g. GLIM's virtual-LiDAR-cameras tool) can produce richer per-camera
   depth maps with edge-aware gap-fill. Particularly useful for forward-facing
   MMS rigs with narrow-vertical-FOV LiDARs (Livox Horizon class, 25° V FOV)
   where native depth gen tops out at ~1.5% per-image pixel coverage. See
   [Patches in this fork](#patches-in-this-fork) for details.
4. **Multi-extension image-path resolver, configurable depth-merge window,
   diagnostic tracing**, and other small ergonomics — listed below.

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

### Build / install flow at a glance

There are **three distinct stages** here; the words "build" mean different
things at each one. Confusing the three is the most common gotcha for new
users, so it's worth being explicit:

| Stage | What it produces | Frequency | Where it happens |
|---|---|---|---|
| **1. Image build** (`docker build`) | A reusable Docker image (`lvba:dev`) containing the OS, Eigen/OpenCV/PCL/Ceres/Sophus/GLEW/etc. **No LVBA source** is compiled into the image. | **Once** (re-do only if deps change) | Host shell, takes ~10-20 min the first time |
| **2. Binary build** (`cmake .. && make -j`) | The `build/lvba_run` executable in your source tree, built against the image's dependencies. | **When source changes** | Inside an interactive container that bind-mounts your source |
| **3. Run** (`run_lvba.sh`) | LVBA actually executing on a dataset. | **Every time you process a dataset** | A fresh ephemeral container per run; container exits on completion |

The image is a sealed environment that the binary build + every run launch
from. Your **source code lives on the host** and is bind-mounted into the
container at `/opt/lvba`; the binary lands in `<repo>/build/lvba_run` on the
host filesystem because of the mount. **You do not rebuild the image when you
edit source code** — just the binary inside the container.

### One-time setup

```bash
# 1. Clone the repo + SiftGPU submodule
git clone https://github.com/TokyoWarfare/Global-LVBA-ROS-FREE.git ~/lvba_ws/Global-LVBA
cd ~/lvba_ws/Global-LVBA
git submodule update --init --recursive

# 2. Build the Docker image (stage 1 above; once)
docker build -t lvba:dev .

# 3. Build SiftGPU + LVBA binaries (stage 2 above; first time only).
#    The container bind-mounts your source so artefacts land on the host.
docker run --gpus all --rm -it \
    -v ~/lvba_ws/Global-LVBA:/opt/lvba \
    -v /data:/data \
    lvba:dev
# inside the container:
cd /opt/lvba/src/SiftGPU && mkdir -p build && cd build && cmake .. && make -j$(nproc)
cd /opt/lvba && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc)
exit
```

After the second `make`, `lvba_run` is at `~/lvba_ws/Global-LVBA/build/lvba_run`
on your host (same path the container sees through the bind mount).

### Re-building after source edits

You don't need an interactive shell; one-shot is fine:

```bash
docker run --gpus all --rm \
    -v ~/lvba_ws/Global-LVBA:/opt/lvba \
    -v /data:/data \
    lvba:dev \
    bash -c "cd /opt/lvba/build && make -j\$(nproc)"
```

If CMake's cache somehow points at a wrong path (rare; happens if a previous
build was launched from a different host-side path), nuke and reconfigure:

```bash
docker run --gpus all --rm \
    -v ~/lvba_ws/Global-LVBA:/opt/lvba \
    -v /data:/data \
    lvba:dev \
    bash -c "cd /opt/lvba && rm -rf build && mkdir build && cd build && \
             cmake -DCMAKE_BUILD_TYPE=Release .. && make -j\$(nproc)"
```

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
  colmap_db_path: "Colmap/match.db"   # or colmap_sub5.db for the upstream sample
  image_sample_step: 1                # 1 for GLIM-exported (already pruned), 5 upstream
  enable_lidar_ba: false              # BALM LiDAR-side optimisation
  enable_visual_ba: true              # main visual + point-plane BA
  # ---- this-fork additions: ----
  use_existing_depthmap: true         # read PNGs from <depthmap_dir> instead
                                       # of voxel-projecting
  depthmap_dir: "depth/"               # relative to data_path
  depthmap_scale_per_m: 100.0          # 16-bit PNG: depth_m = pixel_value / scale

depth_merge:
  half_window_s: 10.0                  # ±N s LiDAR-scan merge per image
                                        # (this-fork: upstream was hardcoded 0.5)

track_fusion:                          # track filter thresholds
  inlier_radius_m: 0.5                 # 3D scatter tolerance per track (m)
  reproj_mean_thr: 12.0                # post-fusion mean reproj cap (px)
  min_view_angle: 1.0                  # baseline angle gate (deg)

window_ba, BALM_stage1, BALM_stage2:  # tuning knobs; see upstream paper

colmap_output:
  enable: true                         # write refined poses to
                                       # Colmap/sparse/images.txt and a
                                       # merged colored cloud
```

The full example with all keys is at `config/config.yaml`; GLIM's Dataset
Export → LVBA target emits a populated YAML per-dataset, so you usually
don't write these by hand.

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

## Patches in this fork

If you're cross-referencing this fork against upstream:

### Initial port (ROS-free + Dockerised)

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
output is identical to running the upstream version on Retail_Street.

### Pipeline patches for MMS / GLIM integration

The original Global-LVBA targets handheld close-range capture (Retail_Street:
walker, ~1.5 m/s, facade at 4 m, dense LiDAR returns on every feature).
Mobile-mapping data (vehicle, 40 km/h, features at 20-30 m, narrow-V-FOV
LiDAR) breaks several upstream assumptions. Patches added during MMS
integration testing:

- **`use_existing_depthmap` short-circuit** in `generateDepthWithVoxel`
  (`src/lvba_system.cpp` ~line 847). When the YAML's
  `data_config/use_existing_depthmap: true` is set, the function reads
  per-image 16-bit PNGs from `<dataset>/<depthmap_dir>/<timestamp>.png`
  instead of running the voxel-projection pass. Decodes with
  `cv::imread(IMREAD_UNCHANGED)` and converts to `CV_32FC1` via
  `depthmap_scale_per_m` (units per metre; 100 = 1 cm quantum, max ~655 m
  range with 16-bit headroom). New YAML keys:
  ```yaml
  data_config:
    use_existing_depthmap: true
    depthmap_dir: "depth/"
    depthmap_scale_per_m: 100.0
  ```
  **Why**: native voxel projection on Livox-Horizon-class LiDARs (25° V FOV)
  gives ~1.5% per-image pixel coverage on a 4K camera at 4-5 m feature
  distances, because the LiDAR doesn't return on the upper half of the
  image and the camera's FOV is wider than the LiDAR's. Pre-rendered maps
  from GLIM's Virtual-LiDAR-Cameras pipeline (accumulated submap context,
  splat-tuned, edge-aware gap-fill) reach 55%+ coverage at the same scale,
  which makes `fetchDepthBilinear` actually return values at most keypoint
  pixels — the difference between LVBA producing tracks and not.

- **`getImagePath` multi-extension fallback** (`src/lvba_system.cpp` near
  line ~2287). Upstream hardcoded `.png`; GLIM exports `.jpg` by default
  (smaller, lossless-equivalent for SfM use). New behaviour:
  ```cpp
  for (const char* ext : {".png", ".jpg", ".jpeg", ".bmp"}) {
      if (std::filesystem::exists(base + ext)) return base + ext;
  }
  ```
  Falls through to `.png` for backward compatibility with Retail_Street.
  **Why**: makes the GLIM → LVBA round-trip work without renaming images.

- **Depth-merge half-window surfaced from hardcoded default to YAML key**
  (`src/lvba_system.cpp` ~line 24).
  ```yaml
  depth_merge:
    half_window_s: 10.0   # ±10s of LiDAR scans merged per image
  ```
  Upstream's `nh_.param` reads this key already, but the default 0.5 s is
  Retail_Street-tuned (close range, dense returns, no temporal accumulation
  needed). MMS rigs benefit from a much wider window — 10 s densifies
  per-image depth coverage by pulling in adjacent sweeps. Now consistently
  emitted by GLIM's LVBA-target YAML writer at the MMS-appropriate value.

- **Diagnostic tracing** throughout the pipeline. Stderr-flushing prints
  bracketing every major stage (`[Pipeline/trace]`, `[DB/trace]`,
  `[Frontend/trace]`, `[BuildTracks/trace]`, `[OptBA/trace]`) and inside
  the BALM `cut_voxel` / `recut` loops. **Why**: when LVBA misbehaves on
  unfamiliar data, it tends to silently exit with status 0 (e.g. SIGSEGV
  intercepted by GLIM's crash handler, tee gobbling the real exit code).
  The traces pinpoint the actual failure stage in minutes instead of
  bisecting via code edits + rebuild cycles. Negligible runtime cost.

- **Track filter breakdown counters** (`src/lvba_system.cpp`
  `BuildTracksAndFuse3D`). The per-gate counts (`small_comp`, `no_depth`,
  `dist_inl`, `dedup_img`, `rep_cnt`, `rep_mean`, `view_ang`) print at end
  of track building so the user can see which gate is killing the most
  candidates and tune accordingly. The same six gates exist in upstream
  but their drop counts weren't surfaced.

### Tuning thresholds (different from upstream defaults)

The upstream YAML defaults are tuned for Retail_Street. The defaults
emitted by GLIM's LVBA-target export are tuned for MMS regimes:

|  | Upstream | GLIM-emitted (MMS) | Rationale |
|---|---|---|---|
| `inlier_radius_m` | 0.12 | 0.5 | At 5-12 m feature distance with 1-2° calibration drift, real per-observation scatter is 15-50 cm. 0.12 kills genuine tracks; 0.5 absorbs the drift without letting outliers through. |
| `reproj_mean_thr` | 3.0 | 12.0 (was 5.0) | Pre-BA reprojection noise floor on depth-based 3D is 5-15 px at MMS distances. 3 px kills almost everything; 12 px lets BA receive enough constraints to refine cameras. |
| `min_view_angle` | 8.0 | 1.0 (was 5.0) | Forward-camera + forward motion produces near-zero parallax for on-axis features (the geometry where Retail_Street's walker-vs-facade had 10°+ per frame). Lowering the gate keeps these tracks. |
| `depth_merge/half_window_s` | 0.5 | 10.0 | See above. |
| `image_sample_step` | 5 | 1 | GLIM's export already prunes cameras by min-spacing in Colorize; LVBA doesn't need to subsample further. |

### Empirical results (MMS test sequence, May 2026)

Validation runs on a forward-facing 4K MMS sequence (40 km/h vehicle,
Livox Horizon LiDAR, ~200 cameras per tile after region clipping):

| Config | Kept tracks | Post-BA mean | BA status |
|---|---|---|---|
| Native voxel depth + upstream thresholds | <100 | — | DNF (small_comp >99%) |
| Pre-rendered depth (gap=0, /4) + GLIM thresholds | 1,235 | 9.5 px | CONVERGENCE |
| **+ depth-mask filtered match.db (LightGlue)** | **3,519** | **7 px** | **CONVERGENCE** (10 iters) |

The depth-mask filter (GLIM-side, drops pair-matches where neither
endpoint falls on a depth-covered pixel) removes ~40% of junk matches in
SIFT and proportionally more for LightGlue, while *increasing* surviving
track count and lowering BA cost. Combined with the pre-rendered depth
path, this is the canonical MMS workflow.

---

## Reference dataset

The upstream **LVBA-Dataset**
([Google Drive](https://drive.google.com/drive/folders/19fYG4z666hcxyP6StVXs-ZOI2cbHsU5J?usp=drive_link))
is the easiest way to validate a build. Drop `Retail_Street/` into
`dataset/`, copy `scripts/run_lvba.sh.template` into it, and run.

---

## License

MIT, same as upstream. See [`LICENSE`](LICENSE).
