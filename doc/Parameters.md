## Parameters

Description of the node parameters and how to tune them.

**This document is work-in-progress.**

## Likelihood calculation

- **match_dist_min** (float, default: `0.2`, unit: meters):\
    Point to map distance larger than *match_dist_min* is ignored in the likelihood calculation.
- **match_dist_flat** (float, default: `0.05`, unit: meters):\
    Point-to-map distance smaller than *match_dist_flat* is clipped to *match_dist_flat*. Set to three-sigma of the input pointcloud noise or larger to avoid fixing to local minimums.

## Expansion resetting

- **match_ratio_thresh** (float, default: `0.0`, unit: ratio):\
    Trigger the expansion resetting if the ratio of the measured points which have corresponding map points is smaller than *match_ratio_thresh*. Set 0.0 to disable the expansion resetting.
- **expansion_var_(x|y|z)** (float, default: `0.2`, unit: meters):\
    Gaussian noise distribution to expand the particle distribution on the expansion resetting.
- **expansion_var_(roll|pitch|yaw)** (float, default: `0.05`, unit: rad):\
    Gaussian noise distribution to expand the particle distribution on the expansion resetting.
