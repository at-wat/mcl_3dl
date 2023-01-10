#!/bin/bash

# This file is automatically deployed from https://github.com/at-wat/.rospkg-assets.
# Please don't directly edit; update at-wat/.rospkg-assets instead.

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /catkin_ws

md_codeblock='```'

if [ -d /catkin_ws/src/self/.cached-dataset ]
then
  mkdir -p /catkin_ws/build/self/test/
  mv /catkin_ws/src/self/.cached-dataset/* /catkin_ws/build/self/test/
  ls -lh /catkin_ws/build/self/test/
fi

sed -i -e "/^set(CATKIN_TOPLEVEL TRUE)$/a set(CMAKE_C_FLAGS \"-Wall -Werror -O2 -coverage\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e "/^set(CATKIN_TOPLEVEL TRUE)$/a set(CMAKE_CXX_FLAGS \"-Wall -Werror -O2 -coverage\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

echo "--- catkin cmake hook ---"
grep -A5 -B1 "set(CATKIN_TOPLEVEL TRUE)" /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
echo "-------------------------"

CM_OPTIONS=${CATKIN_MAKE_OPTIONS:-}

catkin_make ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ]
then
  result_text="
${md_codeblock}
$(catkin_test_results --all | grep -v Skipping || true)
${md_codeblock}
"
else
  result_text="
${md_codeblock}
$(catkin_test_results --all | grep -v Skipping || true)
${md_codeblock}
$(find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;')
"
fi
catkin_test_results || (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" "<details><summary>Test failed</summary>

$result_text</details>"; false)

if [ x${COVERAGE_TEST:-true} == "xtrue" ]
then
  set -o pipefail

  echo
  echo "Generated gcda files"
  find /catkin_ws/build -name "*.gcda" | xargs -n1 echo "  -"

  if [ -d /tmp/gcov/ ]
  then
    # Find and copy renamed gcda files
    echo
    echo "Renamed gcda files"
    find /tmp/gcov/ -name "*.gcda" | sed 's|^/tmp/gcov||' | while read file
    do
      id=$(echo ${file} | cut -d'/' -f2) # /id/path/to/gcda
      gcda=$(echo ${file} | sed 's|^/[^/]*/|/|')
      new_gcda=$(echo ${gcda} | sed "s|/\(\S*\)\.gcda$|/\1.${id}.gcda|")
      gcno=$(echo ${gcda} | sed 's|\.gcda|.gcno|')
      new_gcno=$(echo ${gcda} | sed "s|\.gcda|.${id}.gcno|")
      cp /tmp/gcov/${id}/${gcda} ${new_gcda}
      cp ${gcno} ${new_gcno}
      echo "  - ${new_gcda}"
    done
  fi

  cd src/self/
  cp -r /catkin_ws/build ./

  if [ -n "$(find . -name "*.gcda")" ]
  then
    mkdir -p /tmp/gcov-out/
    find . -name "*.gcda" | while read gcda
    do
      echo "processing ${gcda}"
      name="${gcda%.gcda}"
      exec_name="$(echo "${name}" | sed -n 's|^.*/\([^/]\+\)\.dir/.*|.\1|p')"
      gcov -p ${name}.gcda -s ./ > /dev/null
      for file in $(find . -name "*.gcov")
      do
        base="$(basename ${file})"
        mv "${file}" "/tmp/gcov-out/${base%.gcov}.${RANDOM}${exec_name}.gcov"
      done
    done
    mv /tmp/gcov-out/* ./

    rm -rf $(find build -type d -maxdepth 1 -mindepth 1 | grep -v -e "/self$")
    find . -name "*.gcov" | while read file
    do
      mkdir -p /orig-src/$(dirname ${file})
      cp ${file} /orig-src/${file}
    done
  fi
fi

gh-pr-comment "${BUILD_LINK} PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
