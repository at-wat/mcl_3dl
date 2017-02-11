FROM ros:kinetic
#RUN apt-get update && apt-get install -y sudo

COPY script.docker/init.sh mcl_3dl
RUN mcl_3dl/script.docker/init.sh

COPY script.docker/test.sh mcl_3dl
RUN mcl_3dl/script.docker/test.sh


