# Post-release test script for mcl_3dl

This script tests shadow-fixed binary release.

NVIDIA graphic board is not supported in the script at now.
Use Intel or AMD graphic board.

```shell
$ xhost +
$ ./post-release-test.sh kinetic
$ ./post-release-test.sh lunar
$ ./post-release-test.sh melodic
```
