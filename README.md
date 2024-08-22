# TLE7230pi
Driver for TLE7230 infineon relay driver, for raspberry pi, in c++ requires pigpiod is running. Code should be correct, but possible typos in comments from rushed ctrl+c ctrl+v

To Build 

g++ -pthread [main.cpp] -o [main] -lpigpiod_if2 -lrt -std=c++20 

**make sure pigpio daemon is running before running program: sudo pigpiod 
