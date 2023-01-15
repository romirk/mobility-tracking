#!/bin/bash
pushd /mnt/mobility-tracking
git pull
pushd ..
/mnt/envs/3.6/bin/python -m mobility-tracking -vw 1280 -s Tonys-MBP.lan -d v 0.7 -a 5000 --debug
popd
popd
