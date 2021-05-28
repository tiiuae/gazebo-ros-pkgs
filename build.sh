#!/bin/bash

set -euxo pipefail

THISDIR="$(dirname "$(readlink -f "$0")")"

output_dir=$1
output="${output_dir}/gazebo_ros_pkgs"
mkdir -p "${output}"

pushd "${THISDIR}"
git_version=$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)

iname=gazebo-ros-pkgs
docker build -t "${iname}" -f ./Dockerfile .
container_id=$(docker create "${iname}" "")
docker cp "${container_id}":/packages .
docker rm "${container_id}"
cp -r packages/* "${output}"
rm -Rf packages

pushd "${output_dir}"
tar -zcvf "gazebo_ros_pkgs-0.1.0${git_version}.tar.gz" ./gazebo_ros_pkgs
rm -rf ./gazebo_ros_pkgs
popd
popd
