/*
UBLOX.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2016-11-03

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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC 
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "UBLOX.h"

/* default constructor */
UBLOX::UBLOX(){}

/* uBlox object, input the serial bus */
UBLOX::UBLOX(uint8_t bus){
  _bus = bus; // serial bus
}

void UBLOX::configure(uint8_t bus) {
    _bus = bus; // serial bus
}

/* starts the serial communication */
void UBLOX::begin(int baud){

  	// initialize parsing state
  	_fpos = 0;

	// select the serial port
	#if defined(__MK20DX128__) || defined(__MK20DX256__) ||  defined(__MKL26Z64__) // Teensy 3.0 || Teensy 3.1/3.2 || Teensy LC

		if(_bus == 3){
			_port = &Serial3;
		}
		else if(_bus == 2){
			_port = &Serial2;
		}
		else{
			_port = &Serial1;
		}

	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 || Teensy 3.6

		if(_bus == 6){
			_port = &Serial6;
		}
		else if(_bus == 5){
			_port = &Serial5;
		}
		else if(_bus == 4){
			_port = &Serial4;
		}
		else if(_bus == 3){
			_port = &Serial3;
		}
		else if(_bus == 2){
			_port = &Serial2;
		}
		else{
			_port = &Serial1;
		}

	#endif

  	// begin the serial port for uBlox
  	_port->begin(baud);
}

/* read the uBlox data */
bool UBLOX::read(gpsData *gpsData_ptr){

    const double mm2m = 1.0e-3;
    const double en7 = 1.0e-7;
    const double en5 = 1.0e-5;

	union{
		unsigned long val;
		uint8_t b[4];
	}iTOW;

	union{
		unsigned short val;
		uint8_t b[2];
	}utcYear;

	union{
		unsigned long val;
		uint8_t b[4];
	}tAcc;

	union{
		long val;
		uint8_t b[4];
	}utcNano;

	union{
		long val;
		uint8_t b[4];
	}lon;

	union{
		long val;
		uint8_t b[4];
	}lat;

	union{
		long val;
		uint8_t b[4];
	}height;

	union{
		long val;
		uint8_t b[4];
	}hMSL;

	union{
		unsigned long val;
		uint8_t b[4];
	}hAcc;

	union{
		unsigned long val;
		uint8_t b[4];
	}vAcc;

	union{
		long val;
		uint8_t b[4];
	}velN;

	union{
		long val;
		uint8_t b[4];
	}velE;

	union{
		long val;
		uint8_t b[4];
	}velD;

	union{
		long val;
		uint8_t b[4];
	}gSpeed;

	union{
		long val;
		uint8_t b[4];
	}heading;

	union{
		unsigned long val;
		uint8_t b[4];
	}sAcc;

	union{
		unsigned long val;
		uint8_t b[4];
	}headingAcc;

	union{
		unsigned short val;
		uint8_t b[2];
	}pDOP;

	union{
		unsigned long val;
		uint8_t b[4];
	}headVeh;

  // parse the uBlox packet
  if(parse()){

    	iTOW.b[0] = _gpsPayload[4];
    	iTOW.b[1] = _gpsPayload[5];
    	iTOW.b[2] = _gpsPayload[6];
    	iTOW.b[3] = _gpsPayload[7];
    	gpsData_ptr->iTOW = iTOW.val;

    	utcYear.b[0] = _gpsPayload[8];
    	utcYear.b[1] = _gpsPayload[9];
    	gpsData_ptr->utcYear = utcYear.val;

    	gpsData_ptr->utcMonth = _gpsPayload[10];
    	gpsData_ptr->utcDay = _gpsPayload[11];
    	gpsData_ptr->utcHour = _gpsPayload[12];
    	gpsData_ptr->utcMin = _gpsPayload[13];
    	gpsData_ptr->utcSec = _gpsPayload[14];
    	gpsData_ptr->valid = _gpsPayload[15];

    	tAcc.b[0] = _gpsPayload[16];
    	tAcc.b[1] = _gpsPayload[17];
    	tAcc.b[2] = _gpsPayload[18];
    	tAcc.b[3] = _gpsPayload[19];
    	gpsData_ptr->tAcc = tAcc.val;

    	utcNano.b[0] = _gpsPayload[20];
    	utcNano.b[1] = _gpsPayload[21];
    	utcNano.b[2] = _gpsPayload[22];
    	utcNano.b[3] = _gpsPayload[23];
    	gpsData_ptr->utcNano = utcNano.val;

    	gpsData_ptr->fixType = _gpsPayload[24];
    	gpsData_ptr->flags = _gpsPayload[25];
    	gpsData_ptr->flags2 = _gpsPayload[26];
    	gpsData_ptr->numSV = _gpsPayload[27];

    	lon.b[0] = _gpsPayload[28];
    	lon.b[1] = _gpsPayload[29];
    	lon.b[2] = _gpsPayload[30];
    	lon.b[3] = _gpsPayload[31];
    	gpsData_ptr->lon = lon.val * en7;

    	lat.b[0] = _gpsPayload[32];
    	lat.b[1] = _gpsPayload[33];
    	lat.b[2] = _gpsPayload[34];
    	lat.b[3] = _gpsPayload[35];
    	gpsData_ptr->lat = lat.val * en7;

    	height.b[0] = _gpsPayload[36];
    	height.b[1] = _gpsPayload[37];
    	height.b[2] = _gpsPayload[38];
    	height.b[3] = _gpsPayload[39];
    	gpsData_ptr->height = height.val * mm2m;

    	hMSL.b[0] = _gpsPayload[40];
    	hMSL.b[1] = _gpsPayload[41];
    	hMSL.b[2] = _gpsPayload[42];
    	hMSL.b[3] = _gpsPayload[43];
    	gpsData_ptr->hMSL = hMSL.val * mm2m;

    	hAcc.b[0] = _gpsPayload[44];
    	hAcc.b[1] = _gpsPayload[45];
    	hAcc.b[2] = _gpsPayload[46];
    	hAcc.b[3] = _gpsPayload[47];
    	gpsData_ptr->hAcc = hAcc.val * mm2m;

    	vAcc.b[0] = _gpsPayload[48];
    	vAcc.b[1] = _gpsPayload[49];
    	vAcc.b[2] = _gpsPayload[50];
    	vAcc.b[3] = _gpsPayload[51];
    	gpsData_ptr->vAcc = vAcc.val * mm2m;

    	velN.b[0] = _gpsPayload[52];
    	velN.b[1] = _gpsPayload[53];
    	velN.b[2] = _gpsPayload[54];
    	velN.b[3] = _gpsPayload[55];
    	gpsData_ptr->velN = velN.val * mm2m;

    	velE.b[0] = _gpsPayload[56];
    	velE.b[1] = _gpsPayload[57];
    	velE.b[2] = _gpsPayload[58];
    	velE.b[3] = _gpsPayload[59];
    	gpsData_ptr->velE = velE.val * mm2m;

    	velD.b[0] = _gpsPayload[60];
    	velD.b[1] = _gpsPayload[61];
    	velD.b[2] = _gpsPayload[62];
    	velD.b[3] = _gpsPayload[63];
    	gpsData_ptr->velD = velD.val * mm2m;

    	gSpeed.b[0] = _gpsPayload[64];
    	gSpeed.b[1] = _gpsPayload[65];
    	gSpeed.b[2] = _gpsPayload[66];
    	gSpeed.b[3] = _gpsPayload[67];
    	gpsData_ptr->gSpeed = gSpeed.val * mm2m;

    	heading.b[0] = _gpsPayload[68];
    	heading.b[1] = _gpsPayload[69];
    	heading.b[2] = _gpsPayload[70];
    	heading.b[3] = _gpsPayload[71];
    	gpsData_ptr->heading = heading.val * en5;

    	sAcc.b[0] = _gpsPayload[72];
    	sAcc.b[1] = _gpsPayload[73];
    	sAcc.b[2] = _gpsPayload[74];
    	sAcc.b[3] = _gpsPayload[75];
    	gpsData_ptr->sAcc = sAcc.val  * mm2m;

    	headingAcc.b[0] = _gpsPayload[76];
    	headingAcc.b[1] = _gpsPayload[77];
    	headingAcc.b[2] = _gpsPayload[78];
    	headingAcc.b[3] = _gpsPayload[79];
    	gpsData_ptr->headingAcc = headingAcc.val * en5;

    	pDOP.b[0] = _gpsPayload[80];
    	pDOP.b[1] = _gpsPayload[81];
    	gpsData_ptr->pDOP = pDOP.val * 0.01L;

    	headVeh.b[0] = _gpsPayload[88];
    	headVeh.b[1] = _gpsPayload[89];
    	headVeh.b[2] = _gpsPayload[90];
    	headVeh.b[3] = _gpsPayload[91];
    	gpsData_ptr->headVeh = headVeh.val * en5;

    // return true on receiving a full packet
    return true;
  }
  else{

    // return false if a full packet is not received
    return false;
  }

}

/* parse the uBlox data */
bool UBLOX::parse(){
    // uBlox UBX header definition
    const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

    // checksum calculation
    static unsigned char checksum[2];

    // read a byte from the serial port
    while ( _port->available() ) {
		uint8_t c = _port->read();

        // identify the packet header
        if ( _fpos < 2 ) {
            if ( c == UBX_HEADER[_fpos] ) {
                _fpos++;
            }
            else
                _fpos = 0;
        }
        else {

            // grab the payload
            if ( (_fpos-2) < _payloadSize )
                ((unsigned char*)_gpsPayload)[_fpos-2] = c;
                _fpos++;

            // compute checksum
            if ( (_fpos-2) == _payloadSize ) {
                calcChecksum(checksum,_gpsPayload,_payloadSize);
            }
            else if ( (_fpos-2) == (_payloadSize+1) ) {
                if ( c != checksum[0] )
                    _fpos = 0;
            }
            else if ( (_fpos-2) == (_payloadSize+2) ) {
                _fpos = 0;
                if ( c == checksum[1] ) {
                    return true;
                }
            }
            else if ( _fpos > (_payloadSize+4) ) {
                _fpos = 0;
            }
        }
    }
    return false;
}

/* uBlox checksum */
void UBLOX::calcChecksum(unsigned char* CK, unsigned char* payload, uint8_t length){
	CK[0] = 0;
    CK[1] = 0;
    for (uint8_t i = 0; i < length; i++) {
        CK[0] += payload[i];
        CK[1] += CK[0];
    }
}

#endif
