#!/bin/bash -l

cd ..
ct="valgrind --leak-check=full %s $@"
#echo $ct
./waf --run=vanet --command-template="$ct"

