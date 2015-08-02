#!/bin/bash
# must use GCC, CLANG does not seem build working output.
g++ power_meter.cpp -o power_meter `pkg-config uhd --cflags --libs`  -lboost_system -lboost_thread -lrt -lboost_program_options -std=c++11
