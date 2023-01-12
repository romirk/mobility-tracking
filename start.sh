#!/bin/bash
cd server
./env/bin/python backend.py &
cd ../sensorbox
../env/bin/python sensorbox.py -d v 0.5 &
