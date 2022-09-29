# Changelog

## v6.0.1
- Fixed bug in converting from deg to rad where precision could be lost with lat / lon

## v6.0.0
- Removed dependency on Eigen and Units libraries to better support Arduino AVR
- Added CMake support for Teensy MMOD

## v5.1.0
- Added default constructor and a config method to configure the receiver

## v5.0.0
- Removed configuration methods (for now), since they weren't always reliable
- Added relative position output
- Added survey-in output

## v4.1.0
- Added relative position output to support GNSS RTK and GNSS yaw

## v4.0.1
- Fixing issue with linker in versions older than C++17

## v4.0.0
- Merging ublox and ublox-arduino
- Using mcu_support repo for CMake builds
- Autoconfiguring receiver

## v2.0.0
- Updated to match our [Ublox](https://github.com/bolderflight/ublox-arduino) library for flight software
- Updated license to MIT

## v1.1.0
- Updated license to GPLV3.

## v1.0.0
- Modified to work with Arduino 1.5 format and creating a baseline release now.
