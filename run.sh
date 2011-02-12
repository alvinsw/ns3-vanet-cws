#!/bin/bash -l

ct="$@"
cd ..
./waf --run="vanet $ct"

