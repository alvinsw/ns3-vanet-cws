#!/bin/bash -l

ct="$@"
cd ..
./waf --visualize --run="vanet $ct"

