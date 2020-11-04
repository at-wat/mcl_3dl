#!/bin/bash

set -o errexit

source /opt/ros/${ROS_DISTRO}/setup.bash
cd /catkin_ws

md_codeblock='```'

pkgs=$(find . -name package.xml | xargs -n1 dirname)
catkin_lint $pkgs \
  || (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" \
      "<details><summary>catkin_lint failed</summary>

${md_codeblock}
$(catkin_lint $pkgs 2>&1)
${md_codeblock}
</details>"; false)

mkdir -p /catkin_ws/build/mcl_3dl/test/
mv /catkin_ws/src/mcl_3dl/.cached-dataset/* /catkin_ws/build/mcl_3dl/test/
ls -lh /catkin_ws/build/mcl_3dl/test/

sed -i -e "/^set(CATKIN_TOPLEVEL TRUE)$/a set(CMAKE_C_FLAGS \"-Wall -Werror -O2 -coverage\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
sed -i -e "/^set(CATKIN_TOPLEVEL TRUE)$/a set(CMAKE_CXX_FLAGS \"-Wall -Werror -O2 -coverage\")" \
  /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake

echo "--- catkin cmake hook ---"
grep -A5 -B1 "set(CATKIN_TOPLEVEL TRUE)" /opt/ros/${ROS_DISTRO}/share/catkin/cmake/toplevel.cmake
echo "-------------------------"

CM_OPTIONS=''

catkin_make -DMCL_3DL_EXTRA_TESTS=ON ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make tests -DMCL_3DL_EXTRA_TESTS=ON ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make run_tests -DMCL_3DL_EXTRA_TESTS=ON ${CM_OPTIONS} || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
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

  cd src/mcl_3dl/
  cp -r /catkin_ws/build ./

  gcov $(find . -name "*.gcda") -p -c -l > /dev/null

  rm -rf build/mcl_3dl_msgs
  download_codecov='wget --timeout=10 -O /tmp/codecov https://codecov.io/bash'
  ${download_codecov} || ${download_codecov} || ${download_codecov}
  bash /tmp/codecov \
    -Z \
    -X gcov
fi

gh-pr-comment "${BUILD_LINK} PASSED on ${ROS_DISTRO}" "<details><summary>All tests passed</summary>

$result_text</details>" || true
