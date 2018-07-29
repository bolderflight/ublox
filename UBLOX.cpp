/*
UBLOX.cpp
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Arduino.h"
#include "UBLOX.h"

/* uBlox object, input the serial bus and baud rate */
UBLOX::UBLOX(HardwareSerial& bus,uint32_t baud)
{
  _bus = &bus;
	_baud = baud;
}

/* starts the serial communication */
void UBLOX::begin()
{
	// initialize parsing state
	_parserState = 0;
	// begin the serial port for uBlox
	_bus->begin(_baud);
}

/* reads packets from the uBlox receiver */
bool UBLOX::readSensor()
{
	if (_parse(_ubxNavPvt_msgClass,_ubxNavPvt_msgId,_ubxNavPvt_msgLen)) {
		return true;
	} else {
		return false;
	}
}

/* GPS time of week of nav solution, ms */
uint32_t UBLOX::getTow_ms()
{
	return _validPacket.iTOW;
}

/* UTC year */
uint16_t UBLOX::getYear()
{
	return _validPacket.year;
}

/* UTC month */
uint8_t UBLOX::getMonth()
{
	return _validPacket.month;
}

/* UTC day */
uint8_t UBLOX::getDay()
{
	return _validPacket.day;
}

/* UTC hour */
uint8_t UBLOX::getHour()
{
	return _validPacket.hour;
}

/* UTC minute */
uint8_t UBLOX::getMin()
{
	return _validPacket.min;
}

/* UTC second */
uint8_t UBLOX::getSec()
{
	return _validPacket.sec;
}

/* UTC fraction of a second, ns */
int32_t UBLOX::getNanoSec()
{
	return _validPacket.nano;
}

/* number of satellites used in nav solution */
uint8_t UBLOX::getNumSatellites()
{
	return _validPacket.numSV;
}

/* longitude, deg */
double UBLOX::getLongitude_deg()
{
	return (double)_validPacket.lon * 1e-7;
}

/* latitude, deg */
double UBLOX::getLatitude_deg()
{
	return (double)_validPacket.lat * 1e-7;
}

/* height above the ellipsoid, ft */
double UBLOX::getEllipsoidHeight_ft()
{
	return (double)_validPacket.height * 1e-3 * _m2ft;
}

/* height above mean sea level, ft */
double UBLOX::getMSLHeight_ft()
{
	return (double)_validPacket.hMSL * 1e-3 * _m2ft;
}

/* horizontal accuracy estimate, ft */
double UBLOX::getHorizontalAccuracy_ft()
{
	return (double)_validPacket.hAcc * 1e-3 * _m2ft;
}

/* vertical accuracy estimate, ft */
double UBLOX::getVerticalAccuracy_ft()
{
	return (double)_validPacket.vAcc * 1e-3 * _m2ft;
}

/* NED north velocity, ft/s */
double UBLOX::getNorthVelocity_fps()
{
	return (double)_validPacket.velN * 1e-3 * _m2ft;
}

/* NED east velocity, ft/s */
double UBLOX::getEastVelocity_fps()
{
	return (double)_validPacket.velE * 1e-3 * _m2ft;
}

/* NED down velocity ft/s */
double UBLOX::getDownVelocity_fps()
{
	return (double)_validPacket.velD * 1e-3 * _m2ft;
}

/* 2D ground speed, ft/s */
double UBLOX::getGroundSpeed_fps()
{
	return (double)_validPacket.gSpeed * 1e-3 * _m2ft;
}

/* speed accuracy estimate, ft/s */
double UBLOX::getSpeedAccuracy_fps()
{
	return (double)_validPacket.sAcc * 1e-3 * _m2ft;
}

/* 2D heading of motion, deg */
double UBLOX::getMotionHeading_deg()
{
	return (double)_validPacket.headMot * 1e-5;
}

/* 2D vehicle heading, deg */
double UBLOX::getVehicleHeading_deg()
{
	return (double)_validPacket.headVeh * 1e-5;
}

/* heading accuracy estimate, deg */
double UBLOX::getHeadingAccuracy_deg()
{
	return (double)_validPacket.headAcc * 1e-5;
}

/* magnetic declination, deg */
float UBLOX::getMagneticDeclination_deg()
{
	return (float)_validPacket.magDec * 1e-2;
}

/* magnetic declination accuracy estimate, deg */
float UBLOX::getMagneticDeclinationAccuracy_deg()
{
	return (float)_validPacket.magAcc * 1e-2;
}

/* longitude, rad */
double UBLOX::getLongitude_rad()
{
	return (double)_validPacket.lon * 1e-7 * _deg2rad;
}

/* latitude, rad */
double UBLOX::getLatitude_rad()
{
	return (double)_validPacket.lat * 1e-7 * _deg2rad;
}

/* height above the ellipsoid, m */
double UBLOX::getEllipsoidHeight_m()
{
	return (double)_validPacket.height * 1e-3;
}

/* height above mean sea level, m */
double UBLOX::getMSLHeight_m()
{
	return (double)_validPacket.hMSL * 1e-3;
}

/* horizontal accuracy estimate, m */
double UBLOX::getHorizontalAccuracy_m()
{
	return (double)_validPacket.hAcc * 1e-3;
}

/* vertical accuracy estimate, m */
double UBLOX::getVerticalAccuracy_m()
{
	return (double)_validPacket.vAcc * 1e-3;
}

/* NED north velocity, m/s */
double UBLOX::getNorthVelocity_ms()
{
	return (double)_validPacket.velN * 1e-3;
}

/* NED east velocity, m/s */
double UBLOX::getEastVelocity_ms()
{
	return (double)_validPacket.velE * 1e-3;
}

/* NED down velocity, m/s */
double UBLOX::getDownVelocity_ms()
{
	return (double)_validPacket.velD * 1e-3;
}

/* 2D ground speed, m/s */
double UBLOX::getGroundSpeed_ms()
{
	return (double)_validPacket.gSpeed * 1e-3;
}

/* speed accuracy estimate, m/s */
double UBLOX::getSpeedAccuracy_ms()
{
	return (double)_validPacket.sAcc * 1e-3;
}

/* 2D heading of motion, rad */
double UBLOX::getMotionHeading_rad()
{
	return (double)_validPacket.headMot * 1e-5 * _deg2rad;
}

/* 2D vehicle heading, rad */
double UBLOX::getVehicleHeading_rad()
{
	return (double)_validPacket.headVeh * 1e-5 * _deg2rad;
}

/* heading accuracy estimate, rad */
double UBLOX::getHeadingAccuracy_rad()
{
	return (double)_validPacket.headAcc * 1e-5 * _deg2rad;
}

/* magnetic declination, rad */
float UBLOX::getMagneticDeclination_rad()
{
	return (float)_validPacket.magDec * 1e-2 * _deg2rad;
}

/* magnetic declination accuracy estimate, rad */
float UBLOX::getMagneticDeclinationAccuracy_rad()
{
	return (float)_validPacket.magAcc * 1e-2 * _deg2rad;
}

/* position dilution of precision */
float UBLOX::getpDOP()
{
	return (float)_validPacket.pDOP * 1e-2;
}

/* fix type */
enum UBLOX::FixType UBLOX::getFixType()
{
	return (FixType)_validPacket.fixType;
}

/* power save mode */
enum UBLOX::PowerSaveMode UBLOX::getPowerSaveMode()
{
	return (PowerSaveMode)((_validPacket.flags >> 2) & 0x07);
}

/* carrier phase status */
enum UBLOX::CarrierPhaseStatus UBLOX::getCarrierPhaseStatus()
{
	return (CarrierPhaseStatus)((_validPacket.flags >> 6) & 0x03);
}

/* valid fix, within DOP and accuracy masks */
bool UBLOX::isGnssFixOk()
{
	return _validPacket.flags & 0x01;
}

/* differential corrections were applied */
bool UBLOX::isDiffCorrApplied()
{
	return _validPacket.flags & 0x02;
}

/* heading of vehicle is valid */
bool UBLOX::isHeadingValid()
{
	return _validPacket.flags & 0x20;
}

/* UTC date validity could be confirmed */
bool UBLOX::isConfirmedDate()
{
	return _validPacket.flags & 0x40;
}

/* UTC time validity could be confirmed */
bool UBLOX::isConfirmedTime()
{
	return _validPacket.flags & 0x80;
}

/* info about UTC date and time validity confirmation is available */
bool UBLOX::isTimeDateConfirmationAvail()
{
	return _validPacket.flags2 & 0x20;
}

/* valid UTC date */
bool UBLOX::isValidDate()
{
	return _validPacket.valid & 0x01;
}

/* valid UTC time */
bool UBLOX::isValidTime()
{
	return _validPacket.valid & 0x02;
}

/* UTC time of day has been fully resolved, no seconds uncertainty */
bool UBLOX::isTimeFullyResolved()
{
	return _validPacket.valid & 0x04;
}

/* valid magnetic declination estimate */
bool UBLOX::isMagneticDeclinationValid()
{
	return _validPacket.valid & 0x08;
}

/* parse the uBlox data */
bool UBLOX::_parse(uint8_t msg_class,uint8_t msg_id,uint16_t msg_length)
{
	// read a byte from the serial port
	while (_bus->available()) {
		_byte = _bus->read();
		// identify the packet header
		if (_parserState < 2) {
			if (_byte == _ubxHeader[_parserState]) {
				_parserState++;
			} else {
				_parserState = 0;
			}
		} else {
			if ((_parserState - 2) < msg_length) {
				*((uint8_t *) &_tempPacket + _parserState - 2) = _byte;
			}
			_parserState++;
			// compute checksum
			if ((_parserState - 2) == msg_length) {
				_calcChecksum(_checksum,((uint8_t *) &_tempPacket),msg_length);
			} else if ((_parserState - 2) == (msg_length + 1)) {
				if (_byte != _checksum[0]) {
					_parserState = 0;
				}
			} else if ((_parserState - 2) == (msg_length + 2)) {
				_parserState = 0;
				if (_byte == _checksum[1]) {
					memcpy(&_validPacket,&_tempPacket,sizeof(_validPacket));
					return true;
				}
			} else if (_parserState > (msg_length + 4) ) {
				_parserState = 0;
			}
		}
	}
	return false;
}

/* uBlox checksum */
void UBLOX::_calcChecksum(uint8_t* CK, uint8_t* payload, uint16_t length)
{
	CK[0] = 0;
  CK[1] = 0;
	for (uint8_t i = 0; i < length; i++) {
		CK[0] += payload[i];
		CK[1] += CK[0];
	}
}
