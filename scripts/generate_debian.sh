#!/usr/bin/env bash
#
# Generate a debian file for the project
# Copyright 2017-2024 Honda Research Institute Japan. All rights reserved.
#
function find_in_list()
{
   [[ "$2" == *"$1"* ]] && return 0 || return 1;
}
 
function add_path_to_rules()
{
   # The idea was to pass an extra path
   # to CMAKE_PREFIX_PATH pointing to the install folder in the workspace, so the
   # debian generator was able to compile it. I was not able to pass the flag using dh
   # so I had to modify the rules file that gets generated and append the path
   # at the end of the line...
   sed -i "s+CMAKE_PREFIX_PATH=\"+CMAKE_PREFIX_PATH=\"$1;+g" debian/rules
}
 
function get_current_commit()
{
   git rev-parse --short HEAD
}
 
function add_commit_to_control()
{
   sed -i "/^Description:/ s/$/ [$(get_current_commit)]/" debian/control
}
 
function get_package_paths()
{
   # Find all packages in the workspace (folders containing both a 'package.xml' and a 'CMakeLists.txt')
   # -prune is used to exclude subfolders after a match has been found (packages inside packages)
   find "$1" -type d -exec test -f '{}'/package.xml -a -f '{}'/CMakeLists.txt \; -printf "%p\n" -prune | sort | uniq
}
 
function generate_binary_package()
{
   local directory=$1
 
   # Extract the package name from the CMakeLists.txt file. This is just to get an unique
   # id for each build folder. It could have been a number or a random string.
   # It doesn't have to be the package name
   package_name=$(grep -m 1 "project(" "${directory}/CMakeLists.txt" | cut -d"(" -f2 | cut -d" " -f1 | cut -d")" -f1)
 
   # Check if the package should be processed (because it's in the list or because)
   # all the packages should be processed
 
   if [[ -v PACKAGES ]] && ! find_in_list "$package_name" "$PACKAGES"; then
       echo "$package_name not found in package list. Skipping..."
       return 0
   fi
 
   # It seems the bloom-generate has to be executed in the package folder itself or
   # or in a parent folder (aka, it's not valid to try to generate it from /tmp)
   cd "$directory" 
   local os_release=$(cat /etc/os-release | grep UBUNTU_CODENAME= | sed 's/=/\n/g' | tail -1)
   bloom-generate rosdebian --os-name ubuntu --os-version "$os_release" --ros-distro "$ROS_DISTRO" "$directory"
 
   add_path_to_rules "${WORKSPACE_FOLDER}"/install
   add_commit_to_control
 
   # Replace previous postint scripts if any
   test -f debian/postinst && rm --force debian/postinst
   test -f "$directory"/postinst && cp --force "$directory"/postinst debian/
   
   # Replace previous postrm scripts if any
   test -f debian/postrm && rm --force debian/postrm
   test -f "$directory"/postrm && cp --force "$directory"/postrm debian/
 
   # Replace previous preinst scripts if any
   test -f debian/preinst && rm --force debian/preinst
   test -f "$directory"/preinst && cp --force "$directory"/preinst debian/
 
   # Replace previous prerm scripts if any
   test -f debian/prerm && rm --force debian/prerm
   test -f "$directory"/prerm && cp --force "$directory"/prerm debian/
 
   # I didn't manage to pass pamaters to dh by calling the rules script directly, but
   # it seems it's possible to call dh by hand and add the options. The rules script
   # will be executed automatically
   fakeroot dh binary --buildsystem=cmake --parallel \
                      --sourcedirectory="$directory" \
                      --builddirectory="${BUILD_PREFIX}/bloom_build/${package_name}" \
                      --tmpdir="${BUILD_PREFIX}/bloom_tmp/${package_name}" \
                      --dpkg-shlibdeps-params="--ignore-missing-info -l${WORKSPACE_FOLDER}/install/lib/
                          -l${WORKSPACE_FOLDER}/install/lib/${package_name}/lib"
   if [ ! -z "$NOTIFY" ]; then                 
      if [ $? -eq 0 ]
      then
        echo "Debian file built succesfully"
      else
        echo "Could not create debian file" >&2
        exit 1
      fi
   fi
}


NEED_ROOT=0
PY_PROJECT="$(dirname "$0")/../"
pushd ${PY_PROJECT}

# Include the common set of functions
DIR="${BASH_SOURCE%/*}"
if [[ ! -d "$DIR" ]]; then DIR="$PWD"; fi
. "$DIR/common.sh"

new_command "Cleaning the build folders"
rm -rf ${PY_PROJECT}/bloom_build ${PY_PROJECT}/bloom_tmp ${PY_PROJECT}/debian
ret_ok "Cleaning the build folders ... done"

new_command "Generating a debian file"
BUILD_PREFIX="$(pwd)"
generate_binary_package ${PY_PROJECT}
ret_ok "Generating a debian file ... done"

new_command "Cleaning the build folders"
rm -rf ${PY_PROJECT}/bloom_build ${PY_PROJECT}/bloom_tmp ${PY_PROJECT}/debian
ret_ok "Cleaning the build folders ... done"

popd