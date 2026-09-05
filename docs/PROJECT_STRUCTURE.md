# Project structure

| Path | Purpose |
| --- | --- |
| `demo.py` | Stereo depth and visualization entry point |
| `get_stereo_images.py` | Stereo image capture |
| `mono_camera_calibration.py` | Monocular calibration |
| `stereo_camera_calibration.py` | Stereo calibration |
| `core/` | Calibration, matching, image, and geometry utilities |
| `configs/` | Example calibration parameters |
| `data/` | Sample stereo images, checkerboard images, and videos |
| `demo/` | Checkerboard and animation helpers |
| `docs/` | Documentation and example output images |
| `scripts/` | Bash usage examples and legacy installation references |

`core/utils_3d/` also contains auxiliary pose and Human3.6M tools. These are not part of the main stereo demo and may depend on external modules not included in this repository.

Generated captures in `data/temp/` and `data/camera/`, Python caches, and local editor settings are excluded by `.gitignore`.
