^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcl_3dl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove CMake warning message (`#134 <https://github.com/at-wat/mcl_3dl/issues/134>`_)
* Contributors: Atsushi Watanabe

0.1.0 (2018-04-23)
------------------
* Migrate to ROS recommended namespace model (`#130 <https://github.com/at-wat/mcl_3dl/issues/130>`_)
* Minor CI setting updates (`#129 <https://github.com/at-wat/mcl_3dl/issues/129>`_)
* Fix package deps (`#127 <https://github.com/at-wat/mcl_3dl/issues/127>`_)
* Fix dockerfile style (`#125 <https://github.com/at-wat/mcl_3dl/issues/125>`_)
* Load CI cache from docker hub registry (`#124 <https://github.com/at-wat/mcl_3dl/issues/124>`_)

  * also add build matrix

* Add raycast performance benchmark (`#123 <https://github.com/at-wat/mcl_3dl/issues/123>`_)
* Fix GLOBAL_LOCALIZATION status (`#122 <https://github.com/at-wat/mcl_3dl/issues/122>`_)
* Add localization status output (`#120 <https://github.com/at-wat/mcl_3dl/issues/120>`_)
* Fix nodehandle usage (`#121 <https://github.com/at-wat/mcl_3dl/issues/121>`_)
* Update demo without odometry (`#119 <https://github.com/at-wat/mcl_3dl/issues/119>`_)

  * Update demo without odometry
  * Update README
  * Add document of the demo without odometry

* Move sample parameters in launch into yamls (`#72 <https://github.com/at-wat/mcl_3dl/issues/72>`_)
* Fix time jump back (`#117 <https://github.com/at-wat/mcl_3dl/issues/117>`_)

  * Fix time jump back
  * Add warning of time jump
  * Fix tf error check

* Add unit tests for Raycast (`#116 <https://github.com/at-wat/mcl_3dl/issues/116>`_)

  * Add unit tests for Raycast
  * Fix raycast grid handling

* Chunked kd-tree (`#113 <https://github.com/at-wat/mcl_3dl/issues/113>`_)

  * Add chunked kd-tree to remove map truncation
  * Remove unused params
  * Remove unused debug output
  * Add unit test for ChunkedKdtree

* Update test reference checksum (`#114 <https://github.com/at-wat/mcl_3dl/issues/114>`_)
* Fix raycast collision tolerance (`#112 <https://github.com/at-wat/mcl_3dl/issues/112>`_)

  * Tolerance of the end of the raycast was too small in 1a758c0 because of the increase of the search range.

* Add integral angular odometry error constraint (`#111 <https://github.com/at-wat/mcl_3dl/issues/111>`_)
* Fix raycast (`#110 <https://github.com/at-wat/mcl_3dl/issues/110>`_)

  * Hit was checked by using range search with (grid/2.0) which make a lot of miss detection. (sqrt(2.0) * grid / 2.0) should be good approximation.

* Add rule based expansion resetting (`#109 <https://github.com/at-wat/mcl_3dl/issues/109>`_)
* Fix integral odom error debug output (`#108 <https://github.com/at-wat/mcl_3dl/issues/108>`_)
* Add landmark measurement input (`#107 <https://github.com/at-wat/mcl_3dl/issues/107>`_)
* Fix map update timer (`#105 <https://github.com/at-wat/mcl_3dl/issues/105>`_)
* Fix CI bot (`#104 <https://github.com/at-wat/mcl_3dl/issues/104>`_)

  * Fix repository url
  * Use pip version of the bot

* Remove spinOnce polling and waitForTransform (`#102 <https://github.com/at-wat/mcl_3dl/issues/102>`_)

  * Use ros::Timer instead of ros::spinOnce polling
  * Remove waitForTransform for static transforms
  * Remove waitForTransform for buffered (delayed) objects

* Fix particle initialization (`#101 <https://github.com/at-wat/mcl_3dl/issues/101>`_)
* Reset integral odometry error if jumped (`#100 <https://github.com/at-wat/mcl_3dl/issues/100>`_)
* Add constraint on the integral of odometry error (`#99 <https://github.com/at-wat/mcl_3dl/issues/99>`_)

  - odom_err_integ_tc: time constant to hold the integral of the odometry error
  - odom_err_integ_sigma: acceptable range of the integral of the odometry error

* Visualize sampled points and raycasting result (`#97 <https://github.com/at-wat/mcl_3dl/issues/97>`_)

  * Visualize sampled points and raycasting result
  * Remove duplicated code around raycasting

* Fix raycasting accuracy (`#96 <https://github.com/at-wat/mcl_3dl/issues/96>`_)
* Fix odometry noise function in prediction (`#95 <https://github.com/at-wat/mcl_3dl/issues/95>`_)
* Add global localization (`#91 <https://github.com/at-wat/mcl_3dl/issues/91>`_)
* Fix particle resize (`#92 <https://github.com/at-wat/mcl_3dl/issues/92>`_)

  * same fix as `#90 <https://github.com/at-wat/mcl_3dl/issues/90>`_

* Fix resampling for huge particle size (`#90 <https://github.com/at-wat/mcl_3dl/issues/90>`_)

  * All-zero particles have appeared on resampling if the particle size is very large.
  * Also, add iterator.

* Add test for pf::ParticleFilter. (`#89 <https://github.com/at-wat/mcl_3dl/issues/89>`_)
* Build test with -Wall -Werror. (`#88 <https://github.com/at-wat/mcl_3dl/issues/88>`_)

  * Build test with -Wall -Werror.
  * Workaround for invalid macro name bug in PCL(<1.8.1).

* Fix odometry noise function. (`#87 <https://github.com/at-wat/mcl_3dl/issues/87>`_)

  - wrong: `nd(mean = 1.0, sigma = sigma_trans_trans) * nd(mean = 1.0, sigma = sigma_rot_trans)`
  - corrected: `nd(mean = 0.0, sigma = sigma_trans_trans) + nd(mean = 0.0, sigma = sigma_rot_trans)`

* Skip random points sampling if all points are filtered out. (`#86 <https://github.com/at-wat/mcl_3dl/issues/86>`_)
* Fix build on indigo. (`#84 <https://github.com/at-wat/mcl_3dl/issues/84>`_)
* Add map_clip_far param. (`#85 <https://github.com/at-wat/mcl_3dl/issues/85>`_)
* Support variable particle size. (`#78 <https://github.com/at-wat/mcl_3dl/issues/78>`_)

  * Support variable particle size.
  * Add service to change particle size.
  * Add test for resizeParticle.

* Check input cloud size. (`#82 <https://github.com/at-wat/mcl_3dl/issues/82>`_)

  * Check for empty cloud to avoid failure on kdtree build.
  * Fix usage of point size of pcl::PointCloud.

* Remove debug outputs. (`#81 <https://github.com/at-wat/mcl_3dl/issues/81>`_)
* Use online version of test result comment bot. (`#80 <https://github.com/at-wat/mcl_3dl/issues/80>`_)
* Fix const function attributes. (`#77 <https://github.com/at-wat/mcl_3dl/issues/77>`_)
* Remove dummy dep to system_lib. (`#76 <https://github.com/at-wat/mcl_3dl/issues/76>`_)
* Add unit tests for mathematical classes. (`#74 <https://github.com/at-wat/mcl_3dl/issues/74>`_)

  * Add unit tests for Vec3, Quat, NormalLikelihood, Filter classes.
  * Fix scaling of the NormalLikelihood distribution.
  * Fix Filter::set in angle mode.

* Fix naming styles. (`#73 <https://github.com/at-wat/mcl_3dl/issues/73>`_)

  * Names of the classes and their members now get compatible with ROS recommended coding styles.
  * Public member variables are kept without underscore postfix.

* Fix package install. (`#71 <https://github.com/at-wat/mcl_3dl/issues/71>`_)
* Fix assert of sampled point amount check. (`#70 <https://github.com/at-wat/mcl_3dl/issues/70>`_)
* Fix quaternion average and use expectation as estimation result. (`#67 <https://github.com/at-wat/mcl_3dl/issues/67>`_)
* Fix bot's test result posting on fail. (`#68 <https://github.com/at-wat/mcl_3dl/issues/68>`_)
* Include test result on bot post. (`#66 <https://github.com/at-wat/mcl_3dl/issues/66>`_)
* Fix a bug where all particle probabilities get zero. (`#65 <https://github.com/at-wat/mcl_3dl/issues/65>`_)

  - fix number of selected points for likelihood calculation
  - add error recovering / asserts

* fixes coding styles (`#64 <https://github.com/at-wat/mcl_3dl/issues/64>`_)
* adds parameter to accumulate input clouds (`#60 <https://github.com/at-wat/mcl_3dl/issues/60>`_)
* syncs tf timestamp with last odometry (`#61 <https://github.com/at-wat/mcl_3dl/issues/61>`_)
* adds example without odometry (`#57 <https://github.com/at-wat/mcl_3dl/issues/57>`_)
* updates default params and demo (`#55 <https://github.com/at-wat/mcl_3dl/issues/55>`_)
* adds option to disable tf publish and test for tf output (`#46 <https://github.com/at-wat/mcl_3dl/issues/46>`_)
* adds test result notifier bot (`#53 <https://github.com/at-wat/mcl_3dl/issues/53>`_)
* fixes possibly invalid memory access (`#52 <https://github.com/at-wat/mcl_3dl/issues/52>`_)
* changes docker storage driver to overlay2 (`#51 <https://github.com/at-wat/mcl_3dl/issues/51>`_)
* adds pcd file output of all pointcloud (`#50 <https://github.com/at-wat/mcl_3dl/issues/50>`_)
* limits minimum beam_model likelihood (`#49 <https://github.com/at-wat/mcl_3dl/issues/49>`_)
* separates point ranges of beam model and fixes total ref reduction (`#48 <https://github.com/at-wat/mcl_3dl/issues/48>`_)
* makes acc measurement variance configurable (`#47 <https://github.com/at-wat/mcl_3dl/issues/47>`_)
* fixes published tf timestamps to have a future date (`#45 <https://github.com/at-wat/mcl_3dl/issues/45>`_)
* fixes docker caching on travis (`#43 <https://github.com/at-wat/mcl_3dl/issues/43>`_)
* updates default parameters (`#42 <https://github.com/at-wat/mcl_3dl/issues/42>`_)
* adds debug visualization output of casted ray (`#41 <https://github.com/at-wat/mcl_3dl/issues/41>`_)
* fixes total reflection reduction (`#40 <https://github.com/at-wat/mcl_3dl/issues/40>`_)
* rejects total reflection points in beam_model (`#37 <https://github.com/at-wat/mcl_3dl/issues/37>`_)
* fixes test result handling and playback rate (`#38 <https://github.com/at-wat/mcl_3dl/issues/38>`_)
* ignores travis run on non-master branch (`#36 <https://github.com/at-wat/mcl_3dl/issues/36>`_)
* caches test dataset outside of docker (`#34 <https://github.com/at-wat/mcl_3dl/issues/34>`_)

  * caches test dataset outside docker
  * changes script path

* adds travis settings for a test in docker container (`#33 <https://github.com/at-wat/mcl_3dl/issues/33>`_)
* adds localization accuracy test (`#32 <https://github.com/at-wat/mcl_3dl/issues/32>`_)
* makes beam_model likelihood configurable (`#30 <https://github.com/at-wat/mcl_3dl/issues/30>`_)
* removes ad-hoc map filter (`#27 <https://github.com/at-wat/mcl_3dl/issues/27>`_)
* updates sample launch file (`#28 <https://github.com/at-wat/mcl_3dl/issues/28>`_)

  * The commit enables:

    * IMU measurement
    * loading map from pcd file

* adds imu measurement (`#26 <https://github.com/at-wat/mcl_3dl/issues/26>`_)
* adds hysteresis on final estimation (`#24 <https://github.com/at-wat/mcl_3dl/issues/24>`_)
* updates parameters in sample launch file (`#23 <https://github.com/at-wat/mcl_3dl/issues/23>`_)

  * removes map offset parameters
  * specifies jump detection distance

* fixes axis-angle value range (`#22 <https://github.com/at-wat/mcl_3dl/issues/22>`_)
* updates parameters in sample launch file (`#19 <https://github.com/at-wat/mcl_3dl/issues/19>`_)
* fixes odometry error parameter handling (`#18 <https://github.com/at-wat/mcl_3dl/issues/18>`_)
* fixes beam_model raycast origin (`#17 <https://github.com/at-wat/mcl_3dl/issues/17>`_)
* adds parameter to specify odometry error
* adds sample launch file (`#14 <https://github.com/at-wat/mcl_3dl/issues/14>`_)

  * This fixes `#3 <https://github.com/at-wat/mcl_3dl/issues/3>`_.
  * A dataset for testing will be supplied in future.

* adds documentation (`#10 <https://github.com/at-wat/mcl_3dl/issues/10>`_)
* fixes init_yaw/pitch/roll setting (`#12 <https://github.com/at-wat/mcl_3dl/issues/12>`_)
* ad hoc fix to a bug on PCL-1.7 with C++11

  * fixes `#9 <https://github.com/at-wat/mcl_3dl/issues/9>`_

* adds matched/unmatched pointclouds output (`#7 <https://github.com/at-wat/mcl_3dl/issues/7>`_)
* fixes filter resetting in angular mode

  * This commit fixes `#2 <https://github.com/at-wat/mcl_3dl/issues/2>`_.

* makes map clipping parameters configurable
* fixes roll and pitch motion in prediction phase
* adds /amcl_pose output

  * This commit fixes `#1 <https://github.com/at-wat/mcl_3dl/issues/1>`_.

* applies LPF on debugging output pointcloud coordinate
* changes default map frame to 'map' instead of 'map_ground'
* outsources map update
* adds beam model
* makes z clipping parameters configurable
* adds parameter to skip measurement
* reduces almost invisible points in map
* checks localization covariance on map update
* detects pose jump and reset LPF
* makes some parameters configurable
* adds covariance calculation
* uses rpy variance instead of quat
* supports jump back
* fixes PointRepresentation dimension
* speed up by using radiusSearch instead of nearestKSearch
* improves prediction phase
* adds flexible particle operators
* removes garbage semicolons
* makes matching related parameters configurable
* makes several parameters configurable
* adds output filter
* adds weight in matching
* adds some parameters
* reduces number of points of updated map cloud
* adds particleBase::operator+
* clips and updates maps
* adds vec3::operator*
* adds arg to specify sigma to resampling
* avoids memory access error in max()
* supports tf and initialpose
* supports quat::inverse
* supports vec3::operator-
* updates test parameters
* update map cloud
* accumulates clouds
* fixes resampling
* first test version
* Contributors: Atsushi Watanabe
