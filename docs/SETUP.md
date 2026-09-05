# Setup and usage

## Environment

Use the project's previously working environment if available. A reproducible environment specification has not yet been established.

The main pipeline imports NumPy, OpenCV with contrib modules, Matplotlib, Pillow, requests, and Open3D. Additional utilities have separate dependencies. PCL is an optional visualization backend.

`requirements-original.txt` preserves the historical dependency list. The root `requirements.txt` references this list for archival compatibility; review it before installation. It includes overlapping OpenCV distributions and packages not used by the main stereo pipeline.

WLS filtering requires `cv2.ximgproc`. The Open3D code uses legacy APIs such as `open3d.Visualizer` and `open3d.PointCloud`; the historical list specifies `open3d-python==0.7.0.0`. Installing the latest Open3D release without adapting the code is not a supported setup procedure.

## Video demo

From the repository root:

```bash
python demo.py
```

To compare output without WLS filtering:

```bash
python demo.py --filter false
```

Default inputs:

| Input | Path |
| --- | --- |
| Left video | `data/lenacv-video/left_video.avi` |
| Right video | `data/lenacv-video/right_video.avi` |
| Stereo calibration | `configs/lenacv-camera/stereo_cam.yml` |

Open3D is enabled by default. The program may proceed to the image example after the video finishes. Display windows require a graphical desktop session.

Press `q` to exit. Press `s` or `c` to save images to `data/temp/`.

## Image pair

The existing Python interface can process a pair without opening the point cloud viewer:

```python
from demo import StereoDepth

stereo = StereoDepth(
    'configs/lenacv-camera/stereo_cam.yml',
    use_open3d=False,
)
stereo.test_pair_image_file('docs/left.png', 'docs/right.png')
```

This still opens OpenCV image windows and requires the image-processing dependencies.

## Camera input

`StereoDepth.capture1(0)` reads a single stream containing side-by-side views. `StereoDepth.capture2(0, 1)` reads two devices. Device IDs are examples: select the correct devices and match the image dimensions to the calibration configuration. Sequential reads from two devices do not provide hardware synchronization.

## Calibration

- `get_stereo_images.py`: capture calibration images.
- `mono_camera_calibration.py`: estimate individual camera parameters.
- `stereo_camera_calibration.py`: estimate stereo parameters.

The Bash examples in `scripts/` use an 8 × 11 checkerboard corner grid and a square size of 20 mm. Check these values against the physical target before using them. Bash syntax cannot be run directly in PowerShell.

`scripts/pcl.sh` is a legacy platform-specific installation reference, not a supported automatic installer.

## Validation status

Runtime dependencies, live capture, point cloud rendering, measurement accuracy, and processing speed are not covered by a verified benchmark. Color/depth alignment and invalid-disparity handling should be checked before using the output for measurement.
