#!/bin/bash
if test -z "$1"
then
  echo "Pls Provide your deployer xml as the first argument"
else
  rosrun ocl deployer-gnulinux --start $1
fi
