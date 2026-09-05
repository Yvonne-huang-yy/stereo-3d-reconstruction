# Stereo geometry notes

## Disparity and depth

For a rectified stereo pair, depth can be computed as:

```text
depth = focal_length_pixels * baseline / disparity_pixels
```

Depth and baseline use the same physical unit. The calibration scale determines whether the resulting coordinates are in millimeters, meters, or another unit.

The project uses OpenCV's reprojection matrix `Q` to recover 3D coordinates. SGBM fixed-point disparity is divided by 16 before reprojection. Invalid or zero disparities require care when interpreting depth.

## WLS filtering

The optional WLS filter refines the left disparity using the image and a right disparity estimate. Compare filtered and unfiltered results on the same inputs to assess the quality and computational cost.

## Environment

See [Setup and usage](SETUP.md). PCL installation notes are retained in `../scripts/pcl.sh` as a legacy reference.

## Further reading

The original documentation referenced these installation notes:

- https://zhuanlan.zhihu.com/p/162277657
- https://blog.csdn.net/weixin_47047999/article/details/119088321
