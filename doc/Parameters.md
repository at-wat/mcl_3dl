## Parameters

Description of the node parameters and how to tune them.

**This document is work-in-progress.**

- **match_dist_min** (float, default: `0.2`, unit: meters):\
    Point to map distance larger than *match_dist_min* is ignored in the likelihood calculation.
- **match_dist_flat** (float, default: `0.05`, unit: meters):\
    Point-to-map distance smaller than *match_dist_flat* is clipped to *match_dist_flat*. Set to two or three-sigma of the input pointcloud noise.
