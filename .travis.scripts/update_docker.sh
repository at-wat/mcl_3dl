#!/bin/bash

set -o errexit
set -o verbose

echo 'DOCKER_OPTS="-H tcp://127.0.0.1:2375 -H unix:///var/run/docker.sock -s overlay2"' \
		 | sudo tee /etc/default/docker > /dev/null
sudo service docker restart

sleep 5

