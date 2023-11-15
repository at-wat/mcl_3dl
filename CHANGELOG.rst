^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcl_3dl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2023-11-14)
------------------
* Fix reported entropy (`#408 <https://github.com/at-wat/mcl_3dl/issues/408>`_)
* Update assets to v0.6.1 (`#407 <https://github.com/at-wat/mcl_3dl/issues/407>`_)
* Update assets to v0.6.0 (`#406 <https://github.com/at-wat/mcl_3dl/issues/406>`_)
* Update assets to v0.5.2 (`#405 <https://github.com/at-wat/mcl_3dl/issues/405>`_)
* Update assets to v0.5.1 (`#404 <https://github.com/at-wat/mcl_3dl/issues/404>`_)
* Update assets to v0.4.2 (`#402 <https://github.com/at-wat/mcl_3dl/issues/402>`_)
* Contributors: Atsushi Watanabe, f-fl0

0.6.1 (2023-01-04)
------------------
* Update assets to v0.4.1 (`#400 <https://github.com/at-wat/mcl_3dl/issues/400>`_)
* Support PCL 1.11 and later (`#397 <https://github.com/at-wat/mcl_3dl/issues/397>`_)
* Update assets to v0.4.0 (`#395 <https://github.com/at-wat/mcl_3dl/issues/395>`_)
* Update assets to v0.3.4 (`#392 <https://github.com/at-wat/mcl_3dl/issues/392>`_)
* Remove old workarounds for PCL<1.8 (`#389 <https://github.com/at-wat/mcl_3dl/issues/389>`_)
* Update assets to v0.3.3 (`#388 <https://github.com/at-wat/mcl_3dl/issues/388>`_)
* Update assets to v0.3.2 (`#387 <https://github.com/at-wat/mcl_3dl/issues/387>`_)
* Update code format (`#386 <https://github.com/at-wat/mcl_3dl/issues/386>`_)
* Contributors: Atsushi Watanabe

0.6.0 (2021-05-12)
------------------
* Add option to load cloud through "load_pcd" service (`#381 <https://github.com/at-wat/mcl_3dl/issues/381>`_)
* Update assets to v0.3.1 (`#382 <https://github.com/at-wat/mcl_3dl/issues/382>`_)
* Update assets to v0.3.0 (`#380 <https://github.com/at-wat/mcl_3dl/issues/380>`_)
* Update assets to v0.2.0 (`#379 <https://github.com/at-wat/mcl_3dl/issues/379>`_)
* Contributors: Atsushi Watanabe, Remco

0.5.4 (2021-03-07)
------------------
* Add odom/imu/cloud_queue_size params (`#375 <https://github.com/at-wat/mcl_3dl/issues/375>`_)
* Fix flaky tests (`#376 <https://github.com/at-wat/mcl_3dl/issues/376>`_)
* Contributors: Atsushi Watanabe

0.5.3 (2021-02-26)
------------------
* Fix cloud accum reset on map update (`#371 <https://github.com/at-wat/mcl_3dl/issues/371>`_)
* Contributors: Atsushi Watanabe

0.5.2 (2021-01-15)
------------------
* Fix potential "Time is out of dual 32-bit range" error (`#367 <https://github.com/at-wat/mcl_3dl/issues/367>`_)
* Update assets to v0.1.4 (`#365 <https://github.com/at-wat/mcl_3dl/issues/365>`_)
* Improve test stability (`#363 <https://github.com/at-wat/mcl_3dl/issues/363>`_)
* Update assets to v0.1.3 (`#362 <https://github.com/at-wat/mcl_3dl/issues/362>`_)
* Update assets to v0.1.2 (`#361 <https://github.com/at-wat/mcl_3dl/issues/361>`_)
* Migrate to GitHub Actions (`#357 <https://github.com/at-wat/mcl_3dl/issues/357>`_)
* Update assets to v0.0.10 (`#356 <https://github.com/at-wat/mcl_3dl/issues/356>`_)
* Contributors: Atsushi Watanabe

0.5.1 (2020-10-26)
------------------
* Make hit_range independent from grid size and fix DDA hit/miss state (`#350 <https://github.com/at-wat/mcl_3dl/issues/350>`_)
* Fix crushing when lidar poses are out of map (`#351 <https://github.com/at-wat/mcl_3dl/issues/351>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.5.0 (2020-10-15)
------------------
* Fix crushing when new map is received (`#347 <https://github.com/at-wat/mcl_3dl/issues/347>`_)
* Ease condition for test of PointCloudSamplerWithNormal (`#344 <https://github.com/at-wat/mcl_3dl/issues/344>`_)
* Add faster raycast algorithm using DDA (`#343 <https://github.com/at-wat/mcl_3dl/issues/343>`_)
* Contributors: Naotaka Hatao

0.4.0 (2020-10-07)
------------------
* Fix typos of license (`#340 <https://github.com/at-wat/mcl_3dl/issues/340>`_)
* Add PointCloudSamplerWithNormal (`#339 <https://github.com/at-wat/mcl_3dl/issues/339>`_)
* Contributors: Naotaka Hatao

0.3.0 (2020-09-07)
------------------
* Switch beam model by map label field (`#334 <https://github.com/at-wat/mcl_3dl/issues/334>`_)
* Update test script for latest catkin (`#333 <https://github.com/at-wat/mcl_3dl/issues/333>`_)
* Remove references to sensor_msgs::PointCloud (`#332 <https://github.com/at-wat/mcl_3dl/issues/332>`_)
* Update assets to v0.0.9 (`#331 <https://github.com/at-wat/mcl_3dl/issues/331>`_)
* Improve expansion resetting/global localization test stability (`#330 <https://github.com/at-wat/mcl_3dl/issues/330>`_)
* Fix global localization test parameter (`#328 <https://github.com/at-wat/mcl_3dl/issues/328>`_)
* Avoid rate limit when fetching gh-ph-comment (`#329 <https://github.com/at-wat/mcl_3dl/issues/329>`_)
* Update gh-pr-comment (`#327 <https://github.com/at-wat/mcl_3dl/issues/327>`_)
* Retry codecov script download (`#326 <https://github.com/at-wat/mcl_3dl/issues/326>`_)
* Improve test coverage (`#325 <https://github.com/at-wat/mcl_3dl/issues/325>`_)
* Merge rostest coverage profiles (`#324 <https://github.com/at-wat/mcl_3dl/issues/324>`_)
* Contributors: Atsushi Watanabe, f-fl0

0.2.5 (2020-05-27)
------------------
* Add validation for orientation of initial pose (`#317 <https://github.com/at-wat/mcl_3dl/issues/317>`_)
* Update CI scripts (`#318 <https://github.com/at-wat/mcl_3dl/issues/318>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.2.4 (2020-05-08)
------------------
* Fix resampling failure of last particle (`#313 <https://github.com/at-wat/mcl_3dl/issues/313>`_)
* Retry gpg keyserver on prerelease test (`#312 <https://github.com/at-wat/mcl_3dl/issues/312>`_)
* Add filter class for Vec3 (`#311 <https://github.com/at-wat/mcl_3dl/issues/311>`_)
* Refactor math functions (`#310 <https://github.com/at-wat/mcl_3dl/issues/310>`_)
* Fix deprecation warning (`#309 <https://github.com/at-wat/mcl_3dl/issues/309>`_)
* Split parameter loader code (`#307 <https://github.com/at-wat/mcl_3dl/issues/307>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.2.3 (2020-04-07)
------------------
* Update assets to v0.0.8 (`#303 <https://github.com/at-wat/mcl_3dl/issues/303>`_)
* Fix flaky rostest nodes (`#302 <https://github.com/at-wat/mcl_3dl/issues/302>`_)
* Update E2E test parameters (`#301 <https://github.com/at-wat/mcl_3dl/issues/301>`_)
* Refactor CI scripts (`#300 <https://github.com/at-wat/mcl_3dl/issues/300>`_)
* Add Noetic CI job (`#296 <https://github.com/at-wat/mcl_3dl/issues/296>`_)
* Fix initialization of accumulated cloud header (`#299 <https://github.com/at-wat/mcl_3dl/issues/299>`_)
* Support Noetic (`#297 <https://github.com/at-wat/mcl_3dl/issues/297>`_)
* Contributors: Atsushi Watanabe

0.2.2 (2020-03-30)
------------------
* Make average number of accumulated clouds accurate (`#293 <https://github.com/at-wat/mcl_3dl/issues/293>`_)
* Fix latching flag in demo bag (`#294 <https://github.com/at-wat/mcl_3dl/issues/294>`_)
* Fix cloud accumulation logic (`#290 <https://github.com/at-wat/mcl_3dl/issues/290>`_)
* Contributors: Atsushi Watanabe

0.2.1 (2020-02-03)
------------------
* Set DiagnosticStatus::OK as default (`#283 <https://github.com/at-wat/mcl_3dl/issues/283>`_)
* Update assets to v0.0.7 (`#282 <https://github.com/at-wat/mcl_3dl/issues/282>`_)
* Contributors: Atsushi Watanabe, Daiki Maekawa

0.2.0 (2020-01-18)
------------------
* Install consistent version of ros_buildfarm (`#281 <https://github.com/at-wat/mcl_3dl/issues/281>`_)
* Run prerelease test with latest msgs package (`#278 <https://github.com/at-wat/mcl_3dl/issues/278>`_)
* Expose internal errors and convergence status (`#265 <https://github.com/at-wat/mcl_3dl/issues/265>`_)
* Document motion prediction model parameters (`#277 <https://github.com/at-wat/mcl_3dl/issues/277>`_)
* Contributors: Atsushi Watanabe, Daiki Maekawa

0.1.7 (2020-01-06)
------------------
* Update assets to v0.0.6 (`#273 <https://github.com/at-wat/mcl_3dl/issues/273>`_)
* Update assets to v0.0.5 (`#272 <https://github.com/at-wat/mcl_3dl/issues/272>`_)
* Add catkin/bloom release actions (`#269 <https://github.com/at-wat/mcl_3dl/issues/269>`_)
* Fix codecov setting (`#270 <https://github.com/at-wat/mcl_3dl/issues/270>`_)
* Fix codecov config (`#268 <https://github.com/at-wat/mcl_3dl/issues/268>`_)
* Migrate C math functions to C++ (`#267 <https://github.com/at-wat/mcl_3dl/issues/267>`_)
* Enable particle initialization using covariances (`#259 <https://github.com/at-wat/mcl_3dl/issues/259>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.1.6 (2019-10-21)
------------------
* Clear odometry integration error on global localization (`#257 <https://github.com/at-wat/mcl_3dl/issues/257>`_)
* Accelerate CI and prerelease test (`#254 <https://github.com/at-wat/mcl_3dl/issues/254>`_)
* Contributors: Atsushi Watanabe

0.1.5 (2019-08-15)
------------------
* Split prerelease test job to avoid CI time-limit (`#251 <https://github.com/at-wat/mcl_3dl/issues/251>`_)
* Speed-up covariance calculation during global localization (`#248 <https://github.com/at-wat/mcl_3dl/issues/248>`_)
* Run prerelease test on release preparation PR (`#246 <https://github.com/at-wat/mcl_3dl/issues/246>`_)
* Add a test for landmark measurement (`#242 <https://github.com/at-wat/mcl_3dl/issues/242>`_)
* Add test case for expansion resetting service (`#241 <https://github.com/at-wat/mcl_3dl/issues/241>`_)
* Test matched/unmatched debug outputs (`#240 <https://github.com/at-wat/mcl_3dl/issues/240>`_)
* Add basic tests for State6DOF class (`#239 <https://github.com/at-wat/mcl_3dl/issues/239>`_)
* Fix demo configuration and update README (`#238 <https://github.com/at-wat/mcl_3dl/issues/238>`_)
* Add no-imu and no-odometry modes (`#234 <https://github.com/at-wat/mcl_3dl/issues/234>`_)
* Add test case for obsolated compatibility mode (`#237 <https://github.com/at-wat/mcl_3dl/issues/237>`_)
* Fix document file permission (`#236 <https://github.com/at-wat/mcl_3dl/issues/236>`_)
* Add test for compatibility level check (`#235 <https://github.com/at-wat/mcl_3dl/issues/235>`_)
* Fix ParticleFilter::resample() (`#233 <https://github.com/at-wat/mcl_3dl/issues/233>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#230 <https://github.com/at-wat/mcl_3dl/issues/230>`_)
* Disable CI build for indigo (`#229 <https://github.com/at-wat/mcl_3dl/issues/229>`_)
* Refactor motion prediction models (`#227 <https://github.com/at-wat/mcl_3dl/issues/227>`_)
* Refactor IMU measurement models (`#226 <https://github.com/at-wat/mcl_3dl/issues/226>`_)
* Fix include directory priority (`#225 <https://github.com/at-wat/mcl_3dl/issues/225>`_)
* Custom point type (`#206 <https://github.com/at-wat/mcl_3dl/issues/206>`_)
* Fix transform object constness (`#224 <https://github.com/at-wat/mcl_3dl/issues/224>`_)
* Add LICENSE file (`#220 <https://github.com/at-wat/mcl_3dl/issues/220>`_)
* Add post-release test script (`#218 <https://github.com/at-wat/mcl_3dl/issues/218>`_)
* Reduce memcpy in point cloud transform (`#216 <https://github.com/at-wat/mcl_3dl/issues/216>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.1.4 (2018-12-20)
------------------
* Fix IO figure (`#212 <https://github.com/at-wat/mcl_3dl/issues/212>`_)
* Fix tf timestamp (`#214 <https://github.com/at-wat/mcl_3dl/issues/214>`_)
* Add pf::ParticleFilter::appendParticle (`#207 <https://github.com/at-wat/mcl_3dl/issues/207>`_)
* Fix pointer alignment style (`#210 <https://github.com/at-wat/mcl_3dl/issues/210>`_)
* Migrate tf to tf2 (`#208 <https://github.com/at-wat/mcl_3dl/issues/208>`_)
* Fix class member naming style (`#205 <https://github.com/at-wat/mcl_3dl/issues/205>`_)
* Make lidar measurement model class (`#195 <https://github.com/at-wat/mcl_3dl/issues/195>`_)
* Add I/O diagram to the document (`#199 <https://github.com/at-wat/mcl_3dl/issues/199>`_)
* Update Algorithms.md (`#198 <https://github.com/at-wat/mcl_3dl/issues/198>`_)
* Add apt-get upgrade to test Dockerfiles (`#197 <https://github.com/at-wat/mcl_3dl/issues/197>`_)
* Add document for expansion resetting (`#193 <https://github.com/at-wat/mcl_3dl/issues/193>`_)
* Add test for expansion resetting (`#192 <https://github.com/at-wat/mcl_3dl/issues/192>`_)
* Add test for global localization (`#188 <https://github.com/at-wat/mcl_3dl/issues/188>`_)
* Refactor likelihood calculation (`#189 <https://github.com/at-wat/mcl_3dl/issues/189>`_)
* Add a comment to test_transform_failure (`#184 <https://github.com/at-wat/mcl_3dl/issues/184>`_)
* Build mcl_3dl_msgs from source on CI (`#185 <https://github.com/at-wat/mcl_3dl/issues/185>`_)
* Fix resampling (`#183 <https://github.com/at-wat/mcl_3dl/issues/183>`_)
* Fix test failure on ROS buildfarm (`#181 <https://github.com/at-wat/mcl_3dl/issues/181>`_)
* Fix catkin package definitions (`#180 <https://github.com/at-wat/mcl_3dl/issues/180>`_)
* Add tf exception handling and change message level (`#177 <https://github.com/at-wat/mcl_3dl/issues/177>`_)
* Relax codecov patch threshold (`#179 <https://github.com/at-wat/mcl_3dl/issues/179>`_)
* Allow small coverage drop (`#178 <https://github.com/at-wat/mcl_3dl/issues/178>`_)
* Fix test names (`#176 <https://github.com/at-wat/mcl_3dl/issues/176>`_)
* Add build id to CI bot comment (`#174 <https://github.com/at-wat/mcl_3dl/issues/174>`_)
* Fold CI bot comment (`#173 <https://github.com/at-wat/mcl_3dl/issues/173>`_)
* Decrease bag playback rate in integration test (`#172 <https://github.com/at-wat/mcl_3dl/issues/172>`_)
* Add test for NormalLikelihoodNd (`#171 <https://github.com/at-wat/mcl_3dl/issues/171>`_)
* Report coverage only after successful test (`#170 <https://github.com/at-wat/mcl_3dl/issues/170>`_)
* Add CI badges (`#169 <https://github.com/at-wat/mcl_3dl/issues/169>`_)
* Add codecov covarage test (`#168 <https://github.com/at-wat/mcl_3dl/issues/168>`_)
* Fix bot comment target slug (`#167 <https://github.com/at-wat/mcl_3dl/issues/167>`_)
* Contributors: Atsushi Watanabe, So Jomura

0.1.3 (2018-06-23)
------------------
* Fix install of demo launch and config (`#164 <https://github.com/at-wat/mcl_3dl/issues/164>`_)
* Update CI and add test on ROS Melodic (`#155 <https://github.com/at-wat/mcl_3dl/issues/155>`_)
* Ignore gh-pr-comment failure (`#162 <https://github.com/at-wat/mcl_3dl/issues/162>`_)
* Compile with PCL_NO_PRECOMPILE (`#161 <https://github.com/at-wat/mcl_3dl/issues/161>`_)
* Fix rostest dependency (`#160 <https://github.com/at-wat/mcl_3dl/issues/160>`_)
* Fix roslint dependency (`#159 <https://github.com/at-wat/mcl_3dl/issues/159>`_)
* Update install instructions in README (`#158 <https://github.com/at-wat/mcl_3dl/issues/158>`_)
* Update manifest format and fix CMakeLists (`#157 <https://github.com/at-wat/mcl_3dl/issues/157>`_)
* Use mcl_3dl_msgs package (`#152 <https://github.com/at-wat/mcl_3dl/issues/152>`_)
* Test with shadow-fixed repository (`#154 <https://github.com/at-wat/mcl_3dl/issues/154>`_)
* Update CI bot environments (`#150 <https://github.com/at-wat/mcl_3dl/issues/150>`_)
* Add encrypted token for image caching (`#149 <https://github.com/at-wat/mcl_3dl/issues/149>`_)
* Fix migration instruction message (`#147 <https://github.com/at-wat/mcl_3dl/issues/147>`_)
* Fix match ratio min/max check (`#146 <https://github.com/at-wat/mcl_3dl/issues/146>`_)
* Add interfaces to ChunkedKdtree for external usages (`#145 <https://github.com/at-wat/mcl_3dl/issues/145>`_)
* Install headers (`#143 <https://github.com/at-wat/mcl_3dl/issues/143>`_)
* Contributors: Atsushi Watanabe

0.1.2 (2018-04-27)
------------------
* Workaround for debian stretch build (`#140 <https://github.com/at-wat/mcl_3dl/issues/140>`_)
* Contributors: Atsushi Watanabe

0.1.1 (2018-04-25)
------------------
* Update CI settings (`#136 <https://github.com/at-wat/mcl_3dl/issues/136>`_)
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
