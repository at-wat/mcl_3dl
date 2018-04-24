#!/bin/bash

set -o errexit
set -o verbose

DATASET_BASE_URI=https://openspur.org/~atsushi.w/dataset/mcl_3dl

if [ ! -e ${DATASET_CACHE_DIR} ]; then
	mkdir -p ${DATASET_CACHE_DIR}
fi

cd ${DATASET_CACHE_DIR}

DATASET_REF_MD5=`md5sum ${DATASET_REF_FILE} | cut -f1 -d' ' || true`
DATASET_MD5=`md5sum ${DATASET_FILE} | cut -f1 -d' ' || true`
DATASET_REF_MD5_SERVER=`wget --quiet ${DATASET_BASE_URI}/${DATASET_REF_FILE}.md5 -O -`
DATASET_MD5_SERVER=`wget --quiet ${DATASET_BASE_URI}/${DATASET_FILE}.md5 -O -`

if [ ! -f ${DATASET_REF_FILE} ] || 
		[[ ${DATASET_REF_MD5} != ${DATASET_REF_MD5_SERVER} ]]; then
	echo "${DATASET_REF_FILE}: md5 checksum unmatched (${DATASET_REF_MD5}, ${DATASET_REF_MD5_SERVER})"
	wget --quiet ${DATASET_BASE_URI}/${DATASET_REF_FILE} -O ${DATASET_REF_FILE}
else
	echo "${DATASET_REF_FILE}: md5 checksum matched"
fi

if [ ! -f ${DATASET_FILE} ] || 
		[[ ${DATASET_MD5} != ${DATASET_MD5_SERVER} ]]; then
	echo "${DATASET_FILE}: md5 checksum unmatched (${DATASET_MD5}, ${DATASET_MD5_SERVER})"
	wget --quiet ${DATASET_BASE_URI}/${DATASET_FILE} -O ${DATASET_FILE}
else
	echo "${DATASET_FILE}: md5 checksum matched"
fi

mkdir -p ${TRAVIS_BUILD_DIR}/.cached-dataset
cp ${DATASET_FILE} ${DATASET_REF_FILE} ${TRAVIS_BUILD_DIR}/.cached-dataset

