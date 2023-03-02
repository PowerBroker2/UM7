#include "UM7.h"




void UM7::begin(Stream& _port, const bool& _debug, Stream& _debug_port)
{
    port       = &_port;
    debug      = _debug;
    debug_port = &_debug_port;

    reset();
}




bool UM7::available()
{
    bool valid   = false;
	byte recChar = 0xFF;

	if (port->available())
	{
		valid = true;

		while (port->available())
		{
			recChar   = port->read();
			bytesRead = parse(recChar, valid);

			if (status != UM7_IMU::CONTINUE)
			{
				if (status < 0)
					reset();

				break;
			}
		}
	}
	else
	{
		bytesRead = parse(recChar, valid);

		if (status < 0)
			reset();
	}

    if (status == UM7_IMU::NEW_DATA)
    {
        if (hasData)
        {
            if (isBatch)
            {
                int dataOffset = 0;
                int addrOffset = 0;

                while (dataOffset < bytesRead)
                {
                    dataOffset += parseData(addrOffset, dataOffset);
                    addrOffset++;
                }
            }
            else
                parseData();
        }

        status = UM7_IMU::CONTINUE;
        reset();

        return true;
    }
    else
        return false;
}




int UM7::parse(const byte& recChar, const bool& valid)
{
    if (valid)
    {
		switch (state)
		{
            case UM7_IMU::PARSE_S: /////////////////////////////////////////
            {
                if (recChar == 's')
                {
                    if (debug)
                        debug_port->println("Found s");

                    state  = UM7_IMU::PARSE_N;
                    status = UM7_IMU::CONTINUE;
                }
                else
                {
                    if (debug)
                    {
                        debug_port->println("Did NOT find s");
                        debug_port->print("Found: "); debug_port->println(recChar);
                    }
                }
                break;
            }

            case UM7_IMU::PARSE_N: /////////////////////////////////////////
            {
                if (recChar == 'n')
                {
                    if (debug)
                        debug_port->println("Found n");

                    state  = UM7_IMU::PARSE_P;
                    status = UM7_IMU::CONTINUE;
                }
                else
                {
                    if (debug)
                    {
                        debug_port->println("Did NOT find n");
                        debug_port->print("Found: "); debug_port->println(recChar);
                    }
                }
                break;
            }

            case UM7_IMU::PARSE_P: /////////////////////////////////////////
            {
                if (recChar == 'p')
                {
                    if (debug)
                        debug_port->println("Found p");

                    state  = UM7_IMU::PARSE_PT;
                    status = UM7_IMU::CONTINUE;
                }
                else
                {
                    if (debug)
                    {
                        debug_port->println("Did NOT find p");
                        debug_port->print("Found: "); debug_port->println(recChar);
                    }
                }
                break;
            }

            case UM7_IMU::PARSE_PT: ////////////////////////////////////////
            {
                pt = recChar;
                
                hasData        = pt & (1 << UM7_IMU::HAS_DATA);
                isBatch        = pt & (1 << UM7_IMU::IS_BATCH);
                bl3            = pt & (1 << UM7_IMU::BL3);
                bl2            = pt & (1 << UM7_IMU::BL2);
                bl1            = pt & (1 << UM7_IMU::BL1);
                bl0            = pt & (1 << UM7_IMU::BL0);
                isHidden       = pt & (1 << UM7_IMU::HIDDEN);
                _commandFailed = pt & (1 << UM7_IMU::CF);

                if (debug)
                {
                    debug_port->print("pt: ");             debug_port->println(pt, BIN);
                    debug_port->print("hasData: ");        debug_port->println(hasData);
                    debug_port->print("isBatch: ");        debug_port->println(isBatch);
                    debug_port->print("bl3: ");            debug_port->println(bl3);
                    debug_port->print("bl2: ");            debug_port->println(bl2);
                    debug_port->print("bl1: ");            debug_port->println(bl1);
                    debug_port->print("bl0: ");            debug_port->println(bl0);
                    debug_port->print("isHidden: ");       debug_port->println(isHidden);
                    debug_port->print("_commandFailed: "); debug_port->println(_commandFailed);
                }

                if (hasData)
                {
                    byte bl = (bl0 << 0) +
                              (bl1 << 1) +
                              (bl2 << 2) +
                              (bl3 << 3);
                    
                    if (debug)
                    {
                        debug_port->print("bl: "); debug_port->println(bl);
                        debug_port->print("bl (BIN): "); debug_port->println(bl, BIN);
                    }

                    if (!isBatch && bl)
                    {
                        if (debug)
                            debug_port->println("BATCH ERROR");

                        status = UM7_IMU::BATCH_ERROR;
                        reset();
                        return bytesRead;
                    }

                    bytesToRec = 4 * bl;

                    if (!bytesToRec)
                        bytesToRec = 4;
                }
                else
                    bytesToRec = 0;
                
                if (debug)
                {
                    debug_port->print("bytesToRec: "); debug_port->println(bytesToRec);
                }

                state = UM7_IMU::PARSE_ADDR;
                break;
            }

            case UM7_IMU::PARSE_ADDR: ////////////////////////////////////////////
            {
                addr = recChar;
                
                if (debug)
                {
                    debug_port->print("addr: "); debug_port->println(addr);
                }
                
                if (bytesToRec)
                    state = UM7_IMU::PARSE_DATA;
                else
                    state = UM7_IMU::PARSE_CS1;
                break;
            }

            case UM7_IMU::PARSE_DATA: ////////////////////////////////////////////
            {
                if (payIndex < bytesToRec)
                {
                    if (debug)
                    {
                        debug_port->print(payIndex); debug_port->print(" data: "); debug_port->println(recChar);
                    }

                    buff[payIndex] = recChar;
                    payIndex++;

                    if (payIndex == bytesToRec)
                    {
                        bytesRead = bytesToRec;
                        state     = UM7_IMU::PARSE_CS1;
                    }
                }
                else
                {
                    if (debug)
                        debug_port->println("AAAAAAHHHHH");
                }
                break;
            }

            case UM7_IMU::PARSE_CS1: ////////////////////////////////////////////
            {
                if (debug)
                {
                    debug_port->print("cs1 (BIN): "); debug_port->println(recChar, BIN);
                }

                cs = recChar << 8;

                state = UM7_IMU::PARSE_CS0;
                break;
            }

            case UM7_IMU::PARSE_CS0: ///////////////////////////////////////////
            {
                cs += recChar;

                uint16_t calcCs = calcChecksum();
                
                if (debug)
                {
                    debug_port->print("cs0 (BIN): ");    debug_port->println(recChar, BIN);
                    debug_port->print("cs: ");           debug_port->println(cs);
                    debug_port->print("cs (BIN): ");     debug_port->println(cs, BIN);
                    debug_port->print("calcCs: ");       debug_port->println(calcCs);
                    debug_port->print("calcCs (BIN): "); debug_port->println(calcCs, BIN);
                }

                if (calcCs == cs)
                {
                    if (debug)
                        debug_port->println("Packet parse successful\n");

                    state  = UM7_IMU::PARSE_S;
                    status = UM7_IMU::NEW_DATA;
                    return bytesRead;
                }
                else
                {
                    if (debug)
                        debug_port->println("CS_ERROR");
                    
                    status = UM7_IMU::CS_ERROR;
                    reset();
                    return bytesRead;
                }

                break;
            }

            default:
            {
                reset();
                break;
            }
        }
    }
    else
    {
        status = UM7_IMU::NO_DATA;
        return bytesRead;
    }
    
    status = UM7_IMU::CONTINUE;
    return bytesRead;
}




uint16_t UM7::calcChecksum()
{
    uint16_t calcCs = 's' + 'n' + 'p' + pt + addr;

    for (int i = 0; i < bytesRead; i++)
        calcCs += buff[i];

    return calcCs;
}




void UM7::reset()
{
	memset(buff, 0, sizeof(buff));

    state = UM7_IMU::PARSE_S;

    payIndex  = 0;
	bytesRead = 0;

    _commandFailed = false;
}




int UM7::parseData(const int& addrOffset, const int& dataOffset)
{
    int address = addr + addrOffset;

    switch (address)
    {
        case UM7_IMU::CREG_COM_SETTINGS:
        {
            _baud      = UM7_IMU::BAUD_LOOKUP[buff[dataOffset + 0]];
            _gpsBaud   = UM7_IMU::BAUD_LOOKUP[buff[dataOffset + 1]];
            _gpsAutoTx = buff[dataOffset + 2] & 0x01;
            _satAutoTx = (buff[dataOffset + 3] & 0x01) >> 4;

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES1:
        {
            _rawAccelRate = buff[dataOffset + 0];
            _rawGyroRate  = buff[dataOffset + 1];
            _rawMagRate   = buff[dataOffset + 2];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES2:
        {
            _tempRate   = buff[dataOffset + 0];
            _allRawRate = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES3:
        {
            _procAccelRate = buff[dataOffset + 0];
            _procGyroRate  = buff[dataOffset + 1];
            _procMagRate   = buff[dataOffset + 2];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES4:
        {
            _allProcRate = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES5:
        {
            _quatRate  = buff[dataOffset + 0];
            _eulerRate = buff[dataOffset + 1];
            _posRate   = buff[dataOffset + 2];
            _velRate   = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES6:
        {
            _poseRate     = buff[dataOffset + 0];
            _healthRate   = UM7_IMU::HEALTH_RATE_LOOKUP[buff[dataOffset + 1] & 0x0F];
            _gyroBiasRate = buff[dataOffset + 2];

            return 4;
            break;
        }

        case UM7_IMU::CREG_COM_RATES7: // TODO: lookup tables
        {
            _healthNMEARate  = buff[dataOffset + 0] >> 4;
            _poseNMEARate    = buff[dataOffset + 0] & 0xF;
            _attNMEARate     = buff[dataOffset + 1] >> 4;
            _sensorNMEARate  = buff[dataOffset + 1] & 0xF;
            _ratesNMEARate   = buff[dataOffset + 2] >> 4;
            _gpsPoseNMEARate = buff[dataOffset + 2] & 0xF;
            _quatNMEARate    = buff[dataOffset + 3] >> 4;

            return 4;
            break;
        }

        case UM7_IMU::CREG_MISC_SETTINGS:
        {
            _pps = buff[dataOffset + 2] & 0x1;
            _zg  = buff[dataOffset + 3] & 0x4;
            _q   = buff[dataOffset + 3] & 0x2;
            _mag = buff[dataOffset + 3] & 0x1;

            return 4;
            break;
        }

        case UM7_IMU::CREG_HOME_NORTH:
        {
            uint8_t* ptr = (uint8_t*)&_northHomeLat;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_HOME_EAST:
        {
            uint8_t* ptr = (uint8_t*)&_eastHomeLon;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_HOME_UP:
        {
            uint8_t* ptr = (uint8_t*)&_homeUp;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_GYRO_TRIM_X:
        {
            uint8_t* ptr = (uint8_t*)&_gyroTrimX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_GYRO_TRIM_Y:
        {
            uint8_t* ptr = (uint8_t*)&_gyroTrimY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_GYRO_TRIM_Z:
        {
            uint8_t* ptr = (uint8_t*)&_gyroTrimZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL1_1:
        {
            uint8_t* ptr = (uint8_t*)&_magCal11;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL1_2:
        {
            uint8_t* ptr = (uint8_t*)&_magCal12;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }
            
            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL1_3:
        {
            uint8_t* ptr = (uint8_t*)&_magCal13;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL2_1:
        {
            uint8_t* ptr = (uint8_t*)&_magCal21;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL2_2:
        {
            uint8_t* ptr = (uint8_t*)&_magCal22;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL2_3:
        {
            uint8_t* ptr = (uint8_t*)&_magCal23;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL3_1:
        {
            uint8_t* ptr = (uint8_t*)&_magCal31;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL3_2:
        {
            uint8_t* ptr = (uint8_t*)&_magCal32;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_CAL3_3:
        {
            uint8_t* ptr = (uint8_t*)&_magCal33;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_BIAS_X:
        {
            uint8_t* ptr = (uint8_t*)&_magBiasX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_BIAS_Y:
        {
            uint8_t* ptr = (uint8_t*)&_magBiasY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_MAG_BIAS_Z:
        {
            uint8_t* ptr = (uint8_t*)&_magBiasZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL1_1:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal11;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL1_2:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal12;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL1_3:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal13;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL2_1:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal21;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL2_2:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal22;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL2_3:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal23;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL3_1:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal31;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL3_2:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal32;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_CAL3_3:
        {
            uint8_t* ptr = (uint8_t*)&_accelCal33;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_BIAS_X:
        {
            uint8_t* ptr = (uint8_t*)&_accelBiasX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }
            
            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_BIAS_Y:
        {
            uint8_t* ptr = (uint8_t*)&_accelBiasY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::CREG_ACCEL_BIAS_Z:
        {
            uint8_t* ptr = (uint8_t*)&_accelBiasZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_HEALTH:
        {
            _satsUsed   = buff[dataOffset + 0] >> 2;
            _hdop       = ((buff[dataOffset + 0] & 0x3) << 8) + buff[dataOffset + 1];
            _satsInView = buff[dataOffset + 2] >> 2;
            _ovf        = buff[dataOffset + 2] & 0x1;
            _mgN        = (buff[dataOffset + 3] >> 5) & 0x1;
            _accN       = (buff[dataOffset + 3] >> 4) & 0x1;
            _accelFail  = (buff[dataOffset + 3] >> 3) & 0x1;
            _gyroFail   = (buff[dataOffset + 3] >> 2) & 0x1;
            _magFail    = (buff[dataOffset + 3] >> 1) & 0x1;
            _gps        = (buff[dataOffset + 3] >> 0) & 0x1;

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_RAW_XY:
        {
            _gyroX = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 1] << (0 * 8));
            
            _gyroY = ((int16_t)buff[dataOffset + 2] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 3] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_RAW_Z:
        {
            _gyroZ = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 1] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_gyroT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_RAW_XY:
        {
            _acclX = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 1] << (0 * 8));
            
            _acclY = ((int16_t)buff[dataOffset + 2] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 3] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_RAW_Z:
        {
            _acclZ = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 1] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_acclT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_RAW_XY:
        {
            _magX = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                    ((int16_t)buff[dataOffset + 1] << (0 * 8));
            
            _magY = ((int16_t)buff[dataOffset + 2] << (1 * 8)) +
                    ((int16_t)buff[dataOffset + 3] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_RAW_Z:
        {
            _magZ = ((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                    ((int16_t)buff[dataOffset + 1] << (0 * 8));

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_RAW_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_magT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_TEMPERATURE:
        {
            uint8_t* ptr = (uint8_t*)&_temp;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_TEMPERATURE_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_tempT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_PROC_X:
        {
            uint8_t* ptr = (uint8_t*)&_gyroProcX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_PROC_Y:
        {
            uint8_t* ptr = (uint8_t*)&_gyroProcY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_PROC_Z:
        {
            uint8_t* ptr = (uint8_t*)&_gyroProcZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_PROC_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_gyroProcT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_PROC_X:
        {
            uint8_t* ptr = (uint8_t*)&_accelProcX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_PROC_Y:
        {
            uint8_t* ptr = (uint8_t*)&_accelProcY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_PROC_Z:
        {
            uint8_t* ptr = (uint8_t*)&_accelProcZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_ACCEL_PROC_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_accelProcT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_PROC_X:
        {
            uint8_t* ptr = (uint8_t*)&_magProcX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_PROC_Y:
        {
            uint8_t* ptr = (uint8_t*)&_magProcY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_PROC_Z:
        {
            uint8_t* ptr = (uint8_t*)&_magProcZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_MAG_PROC_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_magProcT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_QUAT_AB:
        {
            _quatA = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                      ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::QUAT_SCALER;

            _quatB = (((int16_t)buff[dataOffset + 2] << (1 * 8)) +
                      ((int16_t)buff[dataOffset + 3] << (0 * 8))) * UM7_IMU::QUAT_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_QUAT_CD:
        {
            _quatC = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                      ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::QUAT_SCALER;

            _quatD = (((int16_t)buff[dataOffset + 2] << (1 * 8)) +
                      ((int16_t)buff[dataOffset + 3] << (0 * 8))) * UM7_IMU::QUAT_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_QUAT_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_quatT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_EULER_PHI_THETA:
        {
            _roll = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                     ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_SCALER;

            _pitch = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                      ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_EULER_PSI:
        {
            _yaw = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                    ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_EULER_PHI_THETA_DOT:
        {
            _rollRate = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                         ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_RATE_SCALER;

            _pitchRate = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                          ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_RATE_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_EULER_PSI_DOT:
        {
            _yawRate = (((int16_t)buff[dataOffset + 0] << (1 * 8)) +
                        ((int16_t)buff[dataOffset + 1] << (0 * 8))) * UM7_IMU::EULER_RATE_SCALER;

            return 4;
            break;
        }

        case UM7_IMU::DREG_EULER_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_eulerT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_POSITION_NORTH:
        {
            uint8_t* ptr = (uint8_t*)&_posNorth;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_POSITION_EAST:
        {
            uint8_t* ptr = (uint8_t*)&_posEast;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_POSITION_UP:
        {
            uint8_t* ptr = (uint8_t*)&_posUp;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_POSITION_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_posT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_VELOCITY_NORTH:
        {
            uint8_t* ptr = (uint8_t*)&_velNorth;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_VELOCITY_EAST:
        {
            uint8_t* ptr = (uint8_t*)&_velEast;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_VELOCITY_UP:
        {
            uint8_t* ptr = (uint8_t*)&_velUp;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_VELOCITY_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_velT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_LATITUDE:
        {
            uint8_t* ptr = (uint8_t*)&_gpsLat;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_LONGITUDE:
        {
            uint8_t* ptr = (uint8_t*)&_gpsLon;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_ALTITUDE:
        {
            uint8_t* ptr = (uint8_t*)&_gpsAlt;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_COURSE:
        {
            uint8_t* ptr = (uint8_t*)&_gpsCOG;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SPEED:
        {
            uint8_t* ptr = (uint8_t*)&_gpsSOG;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_TIME:
        {
            uint8_t* ptr = (uint8_t*)&_gpsT;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_1_2:
        {
            _sat1ID  = buff[dataOffset + 0];
            _sat1SNR = buff[dataOffset + 1];
            _sat2ID  = buff[dataOffset + 2];
            _sat2SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_3_4:
        {
            _sat3ID  = buff[dataOffset + 0];
            _sat3SNR = buff[dataOffset + 1];
            _sat4ID  = buff[dataOffset + 2];
            _sat4SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_5_6:
        {
            _sat5ID  = buff[dataOffset + 0];
            _sat5SNR = buff[dataOffset + 1];
            _sat6ID  = buff[dataOffset + 2];
            _sat6SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_7_8:
        {
            _sat7ID  = buff[dataOffset + 0];
            _sat7SNR = buff[dataOffset + 1];
            _sat8ID  = buff[dataOffset + 2];
            _sat8SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_9_10:
        {
            _sat9ID   = buff[dataOffset + 0];
            _sat9SNR  = buff[dataOffset + 1];
            _sat10ID  = buff[dataOffset + 2];
            _sat10SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GPS_SAT_11_12:
        {
            _sat11ID  = buff[dataOffset + 0];
            _sat11SNR = buff[dataOffset + 1];
            _sat12ID  = buff[dataOffset + 2];
            _sat12SNR = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_BIAS_X:
        {
            uint8_t* ptr = (uint8_t*)&_gyroBiasX;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_BIAS_Y:
        {
            uint8_t* ptr = (uint8_t*)&_gyroBiasY;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::DREG_GYRO_BIAS_Z:
        {
            uint8_t* ptr = (uint8_t*)&_gyroBiasZ;

            for (int i = 3; i >= 0; i--)
            {
                *ptr = buff[dataOffset + i];
                ptr++;
            }

            return 4;
            break;
        }

        case UM7_IMU::GET_FW_REVISION:
        {
            memset(_fwRevision, 0, sizeof(_fwRevision));

            _fwRevision[0] = buff[dataOffset + 0];
            _fwRevision[1] = buff[dataOffset + 1];
            _fwRevision[2] = buff[dataOffset + 2];
            _fwRevision[3] = buff[dataOffset + 3];

            return 4;
            break;
        }

        case UM7_IMU::FLASH_COMMIT:
        {
            break;
        }

        case UM7_IMU::RESET_TO_FACTORY:
        {
            break;
        }

        case UM7_IMU::ZERO_GYROS:
        {
            break;
        }

        case UM7_IMU::SET_HOME_POSITION:
        {
            break;
        }

        case UM7_IMU::RESERVED_1:
        {
            break;
        }

        case UM7_IMU::SET_MAG_REFERENCE:
        {
            break;
        }

        case UM7_IMU::CALIBRATE_ACCELEROMETERS:
        {
            break;
        }

        case UM7_IMU::RESERVED_2:
        {
            break;
        }

        case UM7_IMU::RESET_EKF:
        {
            break;
        }
        
        default:
        {
            break;
        }
    } 

    return 1;            
}
