# Changelog

## v1.2.3
- Updated to support Teensy 4.x

## v1.2.2
- Updated CONTRIBUTING
- Updated *fetch_content* to use https instead of ssh
- Updated *flash_mcu.cmake* to use local loader on Linux

## v1.2.1
- Updated to MIT license.
- Specified versions for dependencies.

## v1.2.0
- Added MSL and WGS84 altitude. Added ground speed, ground track, and track accuracy.

## v1.1.1
- Modified sensors::Ublox::Begin to flush the serial buffer after initializing communication and sensors::Ublox::Read to read through all available bytes, keeping the most recent parsed data. This should help the library keep up when run at lower rates.

## v1.1.0
- Added LLA vector output

## v1.0.0
- Initial baseline
