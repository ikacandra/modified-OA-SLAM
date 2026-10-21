# modified-OA-SLAM: OA-SLAM on an ORB-SLAM3 backbone with YOLOv8n-seg

This repository is a modified version of [OA-SLAM-InstanceSeg](https://github.com/risetslamelins/OA-SLAM-InstanceSeg)
(itself based on [OA-SLAM](https://gitlab.inria.fr/tangram/oa-slam), Zins et al., ISMAR 2022):

1. **Instance segmentation front end: YOLOv8n-seg.** Detections and masks come from YOLOv8n-seg, either
   - a TensorRT engine (`.engine`, e.g. FP16 built on the Jetson with `trtexec`), see `src/TRTDetector.cc`, or
   - the ONNX model with the OpenCV DNN backend (`.onnx`), or
   - a JSON file of precomputed detections (`.json`).
2. **SLAM backbone: ORB-SLAM3 v1.0** ([UZ-SLAMLab/ORB_SLAM3@4452a3c](https://github.com/UZ-SLAMLab/ORB_SLAM3))
   instead of ORB-SLAM2. The OA-SLAM object layer (ellipsoid objects, object tracks, local object
   mapping, object-based relocalization) runs on top of the ORB-SLAM3 Atlas.

The git history keeps the steps separate:

| commit | content |
|---|---|
| `Hoan thanh save Ground Truth ...` | OA-SLAM-InstanceSeg as cloned (ORB-SLAM2) |
| `Add YOLOv8n-seg TensorRT detector ...` | YOLOv8n-seg TensorRT detector, RGB-D camera files, benchmark scripts (still ORB-SLAM2) |
| `Import ORB-SLAM3 v1.0 core ...` | pristine ORB-SLAM3 sources in place of ORB-SLAM2 (does not build on its own) |
| `Port the OA-SLAM object layer to ORB-SLAM3` | all the integration changes (`git show` that commit to review them against ORB-SLAM3) |

## What the OA-SLAM layer does in the ORB-SLAM3 backbone

Every change in the ORB-SLAM3 sources is marked with `[OA-SLAM]` or `[ADAPTIVE WEIGHT]`.

- **Dynamic objects** (`Tracking::PrepareDetections`, `ORBextractor::operator()`): the masks of the
  *person* detections (COCO id 0) are merged, dilated (15x15) and passed to the `Frame`; ORB keypoints
  inside the mask are discarded.
- **Adaptive weights** (`Optimizer::PoseOptimization`): RGB-D (stereo) observations inside a static object
  mask get a weight of 1.5, inside an AprilTag box 2.0 (AprilTag id 1000 is not a COCO class, so this is
  disabled with the stock model).
- **Object tracking and mapping** (`Tracking::TrackObjects`): detections with score > 0.5 are associated to
  object tracks (Hungarian algorithm on 2D IoU / projected-ellipsoid IoU, plus shared map points),
  ellipsoids are reconstructed, optimized and inserted in the map; `LocalObjectMapping` refines and fuses
  them in its own thread. The association parameters can be changed by environment variables, the defaults
  are the original values: `OA_IOU2D=0.2`, `OA_IOU3D=0.3`, `OA_MIN_OBS=10`.
- **Relocalization** (`Tracking::RelocalizationWithMode`): `points` (ORB-SLAM3), `objects` (P3P on
  ellipsoids/detections) or `points+objects` (objects when points fail). It is used in the
  `RECENTLY_LOST` state of ORB-SLAM3 and in localization mode.
- **Atlas (multi-map)**: objects are stored per `Map`. Only the tracks of the active map are associated;
  when tracking is lost and ORB-SLAM3 starts a new map, objects stay in their map. When LoopClosing merges
  the active map into an older one (`MergeLocal`), the objects are moved and their ellipsoids are
  transformed with the merge similarity. `ResetActiveMap` removes the objects of that map.
- **Viewer**: map window with the ellipsoids (quadrics) and their 3D oriented boxes, object points,
  keyframes; *Current Frame* window with the instance masks (persons in red), detection / track boxes and
  the projected ellipses (solid: in the map, dashed: being initialized); *Depth* window (0-5 m colormap with
  the track boxes). Menu entries: `Show Masks`, `Show Depth`, `Disp 3D Bboxes`, `Color by Cat`, `Pause`, `Quit`.
- **Mask crop fix** (`decodeYoloSeg`): the YOLOv8-seg masks are cropped to their boxes, as `crop_mask` in
  Ultralytics. OA-SLAM-InstanceSeg did not crop them, so person masks leaked over the background and removed
  static keypoints. The ORB-SLAM2 results (`OA-SLAM-Yolov8nSegFP16`) were obtained without this fix.

Not ported (tied to ORB-SLAM2 internals): Osmap map saving/loading, the `oa-slam_localization` example, the Dockerfile,
the AR viewer, the ROS 2 node, and objects in the local bundle adjustment (`use_objects_in_local_BA` 1/2;
the default 0, used for all the experiments, is what this version does). ORB-SLAM3 can save/load its
Atlas itself (`System.SaveAtlasToFile` / `System.LoadAtlasFromFile` in the settings file), without the objects.

## Dependencies

Tested on Jetson (JetPack 5.1.3 / L4T R35.5, CUDA 11.4, TensorRT 8.5, OpenCV 4.10, GCC 9.4, CMake 3.16).

- ORB-SLAM3 requirements: [Pangolin](https://github.com/stevenlovegrove/Pangolin), OpenCV >= 4.4 (with the
  `dnn` module), Eigen3, Boost (serialization), OpenSSL (libcrypto).
- [Dlib](https://github.com/davisking/dlib) (Hungarian algorithm).
- TensorRT + CUDA runtime (optional, for `.engine` models; detected by CMake).

Included in `Thirdparty`: DBoW2, g2o and Sophus (ORB-SLAM3 versions), [nlohmann/json](https://github.com/nlohmann/json).

## Building

```
./build.sh
```

It builds DBoW2 and g2o, extracts the vocabulary and builds `lib/libOA_SLAM3.so`,
`bin/oa-slam_rgb-d-offline` (RGB-D sequences) and `bin/oa-slam` (monocular / webcam).

### YOLOv8n-seg model

`models/yolov8n-seg.onnx` is the stock COCO model exported from Ultralytics (640x640). TensorRT engines are
specific to the device and TensorRT version, so build the engine on the target:

```
/usr/src/tensorrt/bin/trtexec --onnx=models/yolov8n-seg.onnx --saveEngine=models/yolov8n-seg_fp16.engine --fp16 --workspace=4096
```

## Camera files

`Cameras/*.yaml` use the ORB-SLAM3 settings format (`File.version: "1.0"`). The RGB-D files
(`TUM1_RGBD.yaml`, `TUM2_RGBD.yaml`, `TUM3_RGBD.yaml`, `Bonn_RGBD.yaml`) have the same intrinsics,
distortion, depth factor and ORB parameters as the ORB-SLAM2 files of OA-SLAM-InstanceSeg; the ORB-SLAM2
`Camera.bf` becomes `Stereo.b = bf / fx`.

## Running

### RGB-D sequence (TUM RGB-D / Bonn)

```
./bin/oa-slam_rgb-d-offline
      vocabulary_file
      camera_file
      path_to_sequence (root folder)
      path_to_association_file (.txt)
      detections_file (.json, .onnx or .engine)
      categories_to_ignore_file (one category id per line, or null)
      relocalization_mode ('points', 'objects' or 'points+objects')
      output_name
      [use_viewer (0: off, 1: on and closed at the end; default: on, waits for Quit)]
```

Example:

```
S=~/Riset_SLAM/Dataset/tum/rgbd_dataset_freiburg3_walking_xyz
./bin/oa-slam_rgb-d-offline Vocabulary/ORBvoc.txt Cameras/TUM3_RGBD.yaml $S $S/associations.txt \
    models/yolov8n-seg_fp16.engine null points+objects fr3_walking_xyz 1
```

Outputs in the folder `<output_name>/`:

| file | content |
|---|---|
| `camera_poses_<name>_tum.txt` | pose returned for every frame (TUM format; identity when not localized), used by the benchmark scripts as for the ORB-SLAM2 version |
| `CameraTrajectory_<name>_tum.txt` | ORB-SLAM3 `SaveTrajectoryTUM` (frames relative to their keyframes, after loop closure / map merging) |
| `keyframes_poses_<name>_tum.txt`, `.json` | keyframe poses |
| `map_objects_<name>.txt` / `.obj` | objects: `id category Q(4x4 dual quadric, column-major)` / ellipsoid point clouds |
| `map_points_<name>.obj` | map points |
| `frame_times_<name>.csv`, `statistics.txt` | per-frame detection / tracking time, FPS summary |

### Monocular

```
./bin/oa-slam Vocabulary/ORBvoc.txt Cameras/TUM2.yaml <rgb.txt | image folder | webcam_X> \
    <detections .json | .onnx | .engine> null points+objects <output_name>
```

### Benchmark scripts

`scripts/run_benchmark_trt.sh` (repeated runs on the 12 TUM/Bonn sequences) and `scripts/run_ablation_trt.sh`
(association ablation) are unchanged except for the method name: results go to
`Results/repeat_experiment/OA-SLAM3-Yolov8nSegFP16` and `Results/ablations/OA-SLAM3-Yolov8nSegFP16_ablation`,
next to the ORB-SLAM2 results (`OA-SLAM-Yolov8nSegFP16`). Set `METHOD=...` to change it.

## Smoke test

Single runs (Jetson, TensorRT FP16 engine, `points+objects`, viewer off), absolute trajectory error of
`camera_poses_<name>_tum.txt` (`evo_ape tum -a`):

| sequence | frames | ATE RMSE [m] `camera_poses` | ATE RMSE [m] `CameraTrajectory` | objects | ORB-SLAM2 version, 5 runs (ATE RMSE [m]) |
|---|---|---|---|---|---|
| fr3_walking_xyz | 827 | 0.018 | 0.019 | 4 | 0.068 – 0.258 |
| fr3_walking_rpy | 866 | 0.068 | 0.038 | 1 | 0.039 – 0.114 |
| fr2_desk_with_person | 3983 | 0.016 | 0.015 | 15 | 0.037 – 0.052 |
| fr1_room | 1352 | 0.161 | 0.068 | 4 | 0.059 – 0.186 |
| Bonn crowd | 927 | 0.041 | 0.021 | 0 | 0.044 – 0.058 |

Mean FPS (detection + tracking): 13.3 – 16.2.

These are single runs to check the port, not the benchmark (use `scripts/run_benchmark_trt.sh`).

## License

`LICENSE` is the license file of OA-SLAM-InstanceSeg (GNU AGPLv3); the OA-SLAM source headers state GPLv3.
ORB-SLAM3 is GPLv3 (`LICENSE_ORB_SLAM3`).

- M. Zins, G. Simon, M.-O. Berger, *OA-SLAM: Leveraging Objects for Camera Relocalization in Visual SLAM*, ISMAR 2022.
- C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós, *ORB-SLAM3: An Accurate
  Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM*, IEEE T-RO 2021.
