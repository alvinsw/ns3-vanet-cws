#!/bin/bash -l

cd ..
ct="gdb --args %s $@"
#echo $ct
./waf --run=vanet --command-template="$ct"

