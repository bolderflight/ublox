# Changelog

## v1.1.1
- Modified sensors::Ublox::Begin to flush the serial buffer after initializing communication and sensors::Ublox::Read to read through all available bytes, keeping the most recent parsed data. This should help the library keep up when run at lower rates.

## v1.1.0
- Added LLA vector output

## v1.0.0
- Initial baseline
