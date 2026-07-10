# Dense Point Cloud Mapping Migration: Issues and Solutions

## Overview
Migrate RGBD + Stereo dense point cloud mapping (including TensorRT inference) from `ORBSLAM3_Dense/` to `ORB_SLAM3/`.

---

## Migration Checklist

### Phase 1: Copy New Files
- [x] `include/PointCloudMapping.h` — copied from `with_trt` branch
- [x] `src/PointCloudMapping.cc` — copied from `with_trt` branch
- [x] `include/StereoMatch.h` — copied from `with_trt` branch (includes TensorRT IGEV/LiteAnyStereo)
- [x] `src/StereoMatch.cc` — copied from `with_trt` branch
- [x] `Thirdparty/elas/` — copied from `main` branch (ELAS stereo matching library)
- [x] `Thirdparty/TensorRTTemplate/` — copied from `with_trt` branch (TensorRT inference engine)

### Phase 2: Modify Existing Files
- [x] `include/System.h` — added PointCloudMapping include, forward decl, TrackStereo overload, member
- [x] `src/System.cc` — added PointCloudMapping construction, shutdown, TrackStereo(imDisp) overload
- [x] `include/Tracking.h` — added PointCloudMapping forward decl, constructor param, GrabImageStereo overload, dense members
- [x] `src/Tracking.cc` — added PointCloudMapping construction param, Q matrix loading, image storage, keyframe insertion
- [x] `CMakeLists.txt` — added PCL, Boost, optional TensorRT deps, ELAS link

### Phase 3: Build and Verify
- [x] Build test with cmake/make — **PASSED** (libORB_SLAM3.so 5.2MB, links PCL/ELAS/Boost)
- [ ] RGBD dense mapping test (requires dataset + YAML config with PointCloudMapping section)
- [ ] Stereo dense mapping test (requires dataset + YAML config with PointCloudMapping + Stereo section)
- [ ] TensorRT inference test (requires CUDA + TensorRT SDK + .engine model)

---

## Issues and Solutions

### Issue 1: CMakeLists.txt uses file(GLOB_RECURSE)
**Problem:** The ORB_SLAM3 CMakeLists.txt uses `file(GLOB_RECURSE)` to collect source files. Since we placed `PointCloudMapping.cc` and `StereoMatch.cc` directly in `src/`, they are auto-detected. No changes to the source listing needed.
**Solution:** No action needed — GLOB_RECURSE picks up the new files automatically.

### Issue 2: ORB_SLAM3 CMakeLists.txt uses cmake 2.8
**Problem:** The vanilla ORB_SLAM3 requires only cmake 2.8. PCL's CMake config needs cmake >= 3.x.
**Solution:** Bumped `cmake_minimum_required` to VERSION 3.15.

### Issue 3: PointCloudMapping depends on KeyFrame.h
**Problem:** `PointCloudMapping.cc` includes `<KeyFrame.h>` with angle brackets, which relies on the include path being set correctly. The ORB_SLAM3 CMakeLists.txt adds `${PROJECT_SOURCE_DIR}` to include_directories, so this resolves correctly.
**Solution:** Verified include paths are correct.

### Issue 4: TensorRT branch files need to be extracted from git
**Problem:** The `with_trt` branch contains the latest versions of PointCloudMapping and StereoMatch with TensorRT support, but it's not the currently checked-out branch.
**Solution:** Used `git show origin/with_trt:<path>` to extract individual files from the remote branch without switching branches. For directories, used `git archive origin/with_trt <dir> | tar x`.

### Issue 5: ELAS thirdparty library needs pre-built .so
**Problem:** The ELAS library must be pre-built before linking. The ORBSLAM3_Dense CMakeLists.txt links against `${PROJECT_SOURCE_DIR}/Thirdparty/elas/lib/libelas.so`.
**Solution:** The ELAS `lib/` directory with pre-built `.so` was copied along with the source. If it fails to link, rebuild with: `cd Thirdparty/elas && mkdir build && cd build && cmake .. && make`.

### Issue 6: Boost filesystem/program_options are new dependencies
**Problem:** ORBSLAM3_Dense examples use `boost::program_options` and `boost::filesystem` in their example executables. ORB_SLAM3 only linked `boost_serialization`.
**Solution:** Added `-lboost_filesystem -lboost_program_options` to the link libraries.

### Issue 7: PointCloudMapping.h includes System.h causing circular dependency
**Problem:** `PointCloudMapping.h` includes `System.h`, and `System.h` includes `PointCloudMapping.h`.
**Solution:** This was already handled in the with_trt branch version. The include guards prevent infinite recursion. The forward declaration in System.h breaks the circular dependency.

### Issue 8: TensorRT paths are hardcoded
**Problem:** The CMakeLists.txt has hardcoded paths `CUDA_ROOT_DIR "/usr/local/cuda"` and `TensorRT_ROOT_DIR "/root/TensorRT"`.
**Solution:** These are guarded by `option(WITH_TENSORRT OFF)` so they're only active when explicitly enabled. Users must adjust paths for their system.

### Issue 9: RGBD - depth image storage in GrabImageRGBD
**Problem:** The RGB-D GrabImageRGBD function in Tracking.cc needs to store `mImColor` and `mImdepth` for the dense mapping thread.
**Solution:** The ORBSLAM3_Dense Tracking.cc stores `mImColor = imRGB.clone()` and `mImdepth = imD.clone()` in GrabImageRGBD. This will be verified during the Tracking.cc modification phase.

### Issue 10: TensorRT headers cause compilation failure without CUDA
**Problem:** `StereoMatch.h` unconditionally includes `TRTinfer.h` which includes `NvInfer.h` (TensorRT SDK). On systems without TensorRT/CUDA installed, the build fails with `fatal error: NvInfer.h: No such file or directory`.
**Solution:** Added `#ifdef WITH_TENSORRT` guards around:
1. The `#include "Thirdparty/TensorRTTemplate/TRTInfer/TRTinfer.h"` in StereoMatch.h
2. The `IGEV` and `LiteAnyStereo` enum values in `AlgorithmType`
3. The `TensorRT_IGEV` and `TensorRT_LiteAnyStereo` class definitions
4. The factory cases for IGEV/LiteAnyStereo in StereoMatch.cc
5. The entire TensorRT_IGEV implementation block in StereoMatch.cc

### Issue 11: Settings class missing Q() method
**Problem:** The `newParameterLoader()` in Tracking.cc calls `settings->Q` but the vanilla ORB_SLAM3 Settings class doesn't have a Q() getter. Additionally, when the new Settings class is used (File.version "1.0"), `ParseCamParamFile` is never called, so `fSettings["Q"].mat()` never executes either.
**Solution (final):** Compute Q from stereo camera intrinsics and extrinsics using `cv::stereoRectify()`:
1. Extended camera2 loading from KannalaBrandt-only to ALL stereo sensors (Pinhole + KannalaBrandt)
2. Construct `K_l`, `K_r` from `mpCamera`/`mpCamera2` parameters
3. Get `D_l`, `D_r` from `settings->camera1DistortionCoef()` / `camera2DistortionCoef()`
4. Extract `R`, `t` from `mTlr` (Sophus::SE3f left-to-right transform)
5. Call `cv::stereoRectify(K_l, D_l, K_r, D_r, imgSize, R, t, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY)` to generate the proper 4×4 Q matrix

### Issue 12: OpenCV color conversion constant names
**Problem:** The new GrabImageStereo overload used deprecated OpenCV constants: `CV_RGB2GRAY`, `CV_BGR2GRAY`, `CV_RGBA2GRAY`, `CV_BGRA2GRAY`.
**Solution:** Replaced with the constants used by the original GrabImageStereo method in the same file (e.g., `cv::COLOR_RGB2GRAY`, `cv::COLOR_BGR2GRAY`).
