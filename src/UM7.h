#pragma once
#include "Arduino.h"




namespace UM7_IMU 
{
    const char START[] = "snp";

    const byte CF       = 0; // Used by the autopilot to report when a command has failed. Must be set to zero for all packets written to the UM7.
    const byte HIDDEN   = 1; // If set, then the packet address specified in the “Address” field is a “hidden” address. Hidden registers are used to store factory calibration and filter tuning coefficients that do not typically need to be viewed or modified by the user. This bit should always be set to 0 to avoid altering factory configuration. 
    const byte BL0      = 2; // Four bits specifying the length of the batch operation. Unused if bit 7 is cleared. The maximum batch length is therefore 2^4 = 16
    const byte BL1      = 3; // ''
    const byte BL2      = 4; // ''
    const byte BL3      = 5; // ''
    const byte IS_BATCH = 6; // If the packet is a batch operation, this bit is set (1). If not, this bit is cleared (0)
    const byte HAS_DATA = 7; // If the packet contains data, this bit is set (1). If not, this bit is cleared (0).

    const int MAX_PACKET_SIZE = 3 + 1 + 1 + 64 + 1 + 1; // Start bytes + PT + Addr + Data + CS1 + CS0

    const int CONTINUE           = 3;
    const int NEW_DATA           = 2;
    const int NO_DATA            = 1;
    const int CS_ERROR           = 0;
    const int BATCH_ERROR        = -1;
    const int PAYLOAD_ERROR      = -2;
    const int STALE_PACKET_ERROR = -3;

    enum parse_state { PARSE_S,
                       PARSE_N,
                       PARSE_P,
                       PARSE_PT,
                       PARSE_ADDR,
                       PARSE_DATA,
                       PARSE_CS1,
                       PARSE_CS0 };
    
    const int BAUD_LOOKUP[] = { 9600,
                                4400,
                                9200,
                                8400,
                                7600,
                                15200,
                                28000,
                                53600,
                                30400,
                                56000,
                                460800,
                                921600 };
    
    const float HEALTH_RATE_LOOKUP[] = { 0,
                                         0.125,
                                         0.25,
                                         0.5,
                                         1,
                                         2,
                                         4 };
    
    const float QUAT_SCALER       = 1.0 / 29789.09091;
    const float EULER_SCALER      = 1.0 / 91.02222;
    const float EULER_RATE_SCALER = 1.0 / 16.0;
    
    const int CREG_COM_SETTINGS        = 0x00; // General communication settings 
    const int CREG_COM_RATES1          = 0x01; // Broadcast rate settings 
    const int CREG_COM_RATES2          = 0x02; // Broadcast rate settings 
    const int CREG_COM_RATES3          = 0x03; // Broadcast rate settings 
    const int CREG_COM_RATES4          = 0x04; // Broadcast rate settings 
    const int CREG_COM_RATES5          = 0x05; // Broadcast rate settings 
    const int CREG_COM_RATES6          = 0x06; // Broadcast rate settings 
    const int CREG_COM_RATES7          = 0x07; // Broadcast rate settings 
    const int CREG_MISC_SETTINGS       = 0x08; // Misc. settings 
    const int CREG_HOME_NORTH          = 0x09; // GPS north position to consider position 0 
    const int CREG_HOME_EAST           = 0x0A; // GPS east position to consider position 0 
    const int CREG_HOME_UP             = 0x0B; // GPS altitude to consider position 0 
    const int CREG_GYRO_TRIM_X         = 0x0C; // Bias trim for x-axis rate gyro 
    const int CREG_GYRO_TRIM_Y         = 0x0D; // Bias trim for y-axis rate gyro 
    const int CREG_GYRO_TRIM_Z         = 0x0E; // Bias trim for z-axis rate gyro 
    const int CREG_MAG_CAL1_1          = 0x0F; // Row 1, Column 1 of magnetometer calibration matrix 
    const int CREG_MAG_CAL1_2          = 0x10; // Row 1, Column 2 of magnetometer calibration matrix 
    const int CREG_MAG_CAL1_3          = 0x11; // Row 1, Column 3 of magnetometer calibration matrix 
    const int CREG_MAG_CAL2_1          = 0x12; // Row 2, Column 1 of magnetometer calibration matrix 
    const int CREG_MAG_CAL2_2          = 0x13; // Row 2, Column 2 of magnetometer calibration matrix 
    const int CREG_MAG_CAL2_3          = 0x14; // Row 2, Column 3 of magnetometer calibration matrix 
    const int CREG_MAG_CAL3_1          = 0x15; // Row 3, Column 1 of magnetometer calibration matrix 
    const int CREG_MAG_CAL3_2          = 0x16; // Row 3, Column 2 of magnetometer calibration matrix 
    const int CREG_MAG_CAL3_3          = 0x17; // Row 3, Column 3 of magnetometer calibration matrix 
    const int CREG_MAG_BIAS_X          = 0x18; // Magnetometer X-axis bias 
    const int CREG_MAG_BIAS_Y          = 0x19; // Magnetometer Y-axis bias 
    const int CREG_MAG_BIAS_Z          = 0x1A; // Magnetometer Z-axis bias 
    const int CREG_ACCEL_CAL1_1        = 0x1B; // Row 1, Column 1 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL1_2        = 0x1C; // Row 1, Column 2 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL1_3        = 0x1D; // Row 1, Column 3 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL2_1        = 0x1E; // Row 2, Column 1 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL2_2        = 0x1F; // Row 2, Column 2 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL2_3        = 0x20; // Row 2, Column 3 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL3_1        = 0x21; // Row 3, Column 1 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL3_2        = 0x22; // Row 3, Column 2 of accelerometer calibration matrix 
    const int CREG_ACCEL_CAL3_3        = 0x23; // Row 3, Column 3 of accelerometer calibration matrix 
    const int CREG_ACCEL_BIAS_X        = 0x24; // Accelerometer X-axis bias 
    const int CREG_ACCEL_BIAS_Y        = 0x25; // Accelerometer Y-axis bias 
    const int CREG_ACCEL_BIAS_Z        = 0x26; // Accelerometer Z-axis bias
    const int DREG_HEALTH              = 0x55; // Contains information about the health and status of the UM7 
    const int DREG_GYRO_RAW_XY         = 0x56; // Raw X and Y rate gyro data 
    const int DREG_GYRO_RAW_Z          = 0x57; // Raw Z rate gyro data 
    const int DREG_GYRO_TIME           = 0x58; // Time at which rate gyro data was acquired 
    const int DREG_ACCEL_RAW_XY        = 0x59; // Raw X and Y accelerometer data 
    const int DREG_ACCEL_RAW_Z         = 0x5A; // Raw Z accelerometer data 
    const int DREG_ACCEL_TIME          = 0x5B; // Time at which accelerometer data was acquired 
    const int DREG_MAG_RAW_XY          = 0x5C; // Raw X and Y magnetometer data 
    const int DREG_MAG_RAW_Z           = 0x5D; // Raw Z magnetometer data 
    const int DREG_MAG_RAW_TIME        = 0x5E; // Time at which magnetometer data was acquired 
    const int DREG_TEMPERATURE         = 0x5F; // Temperature data 
    const int DREG_TEMPERATURE_TIME    = 0x60; // Time at which temperature data was acquired 
    const int DREG_GYRO_PROC_X         = 0x61; // Processed x-axis rate gyro data 
    const int DREG_GYRO_PROC_Y         = 0x62; // Processed y-axis rate gyro data 
    const int DREG_GYRO_PROC_Z         = 0x63; // Processed z-axis rate gyro data 
    const int DREG_GYRO_PROC_TIME      = 0x64; // Time at which rate gyro data was acquired 
    const int DREG_ACCEL_PROC_X        = 0x65; // Processed x-axis accel data 
    const int DREG_ACCEL_PROC_Y        = 0x66; // Processed y-axis accel data 
    const int DREG_ACCEL_PROC_Z        = 0x67; // Processed z-axis accel data 
    const int DREG_ACCEL_PROC_TIME     = 0x68; // Time at which accelerometer data was acquired 
    const int DREG_MAG_PROC_X          = 0x69; // Processed x-axis magnetometer data 
    const int DREG_MAG_PROC_Y          = 0x6A; // Processed y-axis magnetometer data 
    const int DREG_MAG_PROC_Z          = 0x6B; // Processed z-axis magnetometer data 
    const int DREG_MAG_PROC_TIME       = 0x6C; // Time at which magnetometer data was acquired
    const int DREG_QUAT_AB             = 0x6D; // Quaternion elements A and B 
    const int DREG_QUAT_CD             = 0x6E; // Quaternion elements C and D 
    const int DREG_QUAT_TIME           = 0x6F; // Time at which the sensor was at the specified quaternion rotation 
    const int DREG_EULER_PHI_THETA     = 0x70; // Roll and pitch angles 
    const int DREG_EULER_PSI           = 0x71; // Yaw angle 
    const int DREG_EULER_PHI_THETA_DOT = 0x72; // Roll and pitch angle rates 
    const int DREG_EULER_PSI_DOT       = 0x73; // Yaw rate 
    const int DREG_EULER_TIME          = 0x74; // Time of computed Euler attitude and rates 
    const int DREG_POSITION_NORTH      = 0x75; // North position in meters 
    const int DREG_POSITION_EAST       = 0x76; // East position in meters 
    const int DREG_POSITION_UP         = 0x77; // Altitude in meters 
    const int DREG_POSITION_TIME       = 0x78; // Time of estimated position 
    const int DREG_VELOCITY_NORTH      = 0x79; // North velocity 
    const int DREG_VELOCITY_EAST       = 0x7A; // East velocity 
    const int DREG_VELOCITY_UP         = 0x7B; // Altitude velocity 
    const int DREG_VELOCITY_TIME       = 0x7C; // Time of velocity estimate 
    const int DREG_GPS_LATITUDE        = 0x7D; // GPS latitude 
    const int DREG_GPS_LONGITUDE       = 0x7E; // GPS longitude 
    const int DREG_GPS_ALTITUDE        = 0x7F; // GPS altitude 
    const int DREG_GPS_COURSE          = 0x80; // GPS course 
    const int DREG_GPS_SPEED           = 0x81; // GPS speed 
    const int DREG_GPS_TIME            = 0x82; // GPS time (UTC time of day in seconds) 
    const int DREG_GPS_SAT_1_2         = 0x83; // GPS satellite information 
    const int DREG_GPS_SAT_3_4         = 0x84; // GPS satellite information 
    const int DREG_GPS_SAT_5_6         = 0x85; // GPS satellite information 
    const int DREG_GPS_SAT_7_8         = 0x86; // GPS satellite information 
    const int DREG_GPS_SAT_9_10        = 0x87; // GPS satellite information 
    const int DREG_GPS_SAT_11_12       = 0x88; // GPS satellite information 
    const int DREG_GYRO_BIAS_X         = 0x89; // Gyro x-axis bias estimate
    const int DREG_GYRO_BIAS_Y         = 0x8A; // Gyro y-axis bias estimate 
    const int DREG_GYRO_BIAS_Z         = 0x8B; // Gyro z-axis bias estimate
    const int GET_FW_REVISION          = 0xAA; // Causes the autopilot to respond with a packet containing the current firmware revision. 
    const int FLASH_COMMIT             = 0xAB; // Writes all current configuration settings to flash 
    const int RESET_TO_FACTORY         = 0xAC; // Reset all settings to factory defaults 
    const int ZERO_GYROS               = 0xAD; // Causes the rate gyro biases to be calibrated.  
    const int SET_HOME_POSITION        = 0xAE; // Sets the current GPS location as position
    const int RESERVED_1               = 0xAF; // RESERVED 
    const int SET_MAG_REFERENCE        = 0xB0; // Sets the magnetometer reference vector 
    const int CALIBRATE_ACCELEROMETERS = 0xB1; // Calibrates the accelerometer biases 
    const int RESERVED_2               = 0xB2; // RESERVED 
    const int RESET_EKF                = 0xB3; // Resets the EKF 
}




class UM7
{
public: // <<---------------------------------------//public
	void begin(Stream& _port, const bool& _debug = true, Stream& _debug_port = Serial);
    bool available();
    bool commandFailed() { return _commandFailed; };

    // void flashCommit();
    // void resetToFactory();
    // void zeroGyros();
    // void setHomePos();
    // void setMagRef();
    // void calAccels();
    // void resetEKF();

    int16_t baud()            { return _baud; };
    int16_t gpsBaud()         { return _gpsBaud; };
    uint8_t gpsAutoTx()       { return _gpsAutoTx; };
    uint8_t satAutoTx()       { return _satAutoTx; };
    int16_t rawAccelRate()    { return _rawAccelRate; };
    int16_t rawGyroRate()     { return _rawGyroRate; };
    int16_t rawMagRate()      { return _rawMagRate; };
    int16_t tempRate()        { return _tempRate; };
    int16_t allRawRate()      { return _allRawRate; };
    int16_t procAccelRate()   { return _procAccelRate; };
    int16_t procGyroRate()    { return _procGyroRate; };
    int16_t procMagRate()     { return _procMagRate; };
    int16_t allProcRate()     { return _allProcRate; };
    int16_t quatRate()        { return _quatRate; };
    int16_t eulerRate()       { return _eulerRate; };
    int16_t posRate()         { return _posRate; };
    int16_t velRate()         { return _velRate; };
    int16_t poseRate()        { return _poseRate; };
    int16_t healthRate()      { return _healthRate; };
    int16_t gyroBiasRate()    { return _gyroBiasRate; };
    int16_t healthNMEARate()  { return _healthNMEARate; };
    int16_t poseNMEARate()    { return _poseNMEARate; };
    int16_t attNMEARate()     { return _attNMEARate; };
    int16_t sensorNMEARate()  { return _sensorNMEARate; };
    int16_t ratesNMEARate()   { return _ratesNMEARate; };
    int16_t gpsPoseNMEARate() { return _gpsPoseNMEARate; };
    int16_t quatNMEARate()    { return _quatNMEARate; };
    uint8_t pps()             { return _pps; };
    uint8_t zg()              { return _zg; };
    uint8_t q()               { return _q; };
    uint8_t mag()             { return _mag; };
    float   northHomeLat()    { return _northHomeLat; };
    float   eastHomeLon()     { return _eastHomeLon; };
    float   homeUp()          { return _homeUp; };
    float   gyroTrimX()       { return _gyroTrimX; };
    float   gyroTrimY()       { return _gyroTrimY; };
    float   gyroTrimZ()       { return _gyroTrimZ; };
    float   magCal11()        { return _magCal11; };
    float   magCal12()        { return _magCal12; };
    float   magCal13()        { return _magCal13; };
    float   magCal21()        { return _magCal21; };
    float   magCal22()        { return _magCal22; };
    float   magCal23()        { return _magCal23; };
    float   magCal31()        { return _magCal31; };
    float   magCal32()        { return _magCal32; };
    float   magCal33()        { return _magCal33; };
    float   magBiasX()        { return _magBiasX; };
    float   magBiasY()        { return _magBiasY; };
    float   magBiasZ()        { return _magBiasZ; };
    float   accelCal11()      { return _accelCal11; };
    float   accelCal12()      { return _accelCal12; };
    float   accelCal13()      { return _accelCal13; };
    float   accelCal21()      { return _accelCal21; };
    float   accelCal22()      { return _accelCal22; };
    float   accelCal23()      { return _accelCal23; };
    float   accelCal31()      { return _accelCal31; };
    float   accelCal32()      { return _accelCal32; };
    float   accelCal33()      { return _accelCal33; };
    float   accelBiasX()      { return _accelBiasX; };
    float   accelBiasY()      { return _accelBiasY; };
    float   accelBiasZ()      { return _accelBiasZ; };
    uint8_t satsUsed()        { return _satsUsed; };
    int16_t hdop()            { return _hdop; };
    uint8_t satsInView()      { return _satsInView; };
    uint8_t ovf()             { return _ovf; };
    uint8_t mgN()             { return _mgN; };
    uint8_t accN()            { return _accN; };
    uint8_t accelFail()       { return _accelFail; };
    uint8_t gyroFail()        { return _gyroFail; };
    uint8_t magFail()         { return _magFail; };
    uint8_t gps()             { return _gps; };
    int16_t gyroX()           { return _gyroX; };
    int16_t gyroY()           { return _gyroY; };
    int16_t gyroZ()           { return _gyroZ; };
    float   gyroT()           { return _gyroT; };
    int16_t acclX()           { return _acclX; };
    int16_t acclY()           { return _acclY; };
    int16_t acclZ()           { return _acclZ; };
    float   acclT()           { return _acclT; };
    int16_t magX()            { return _magX; };
    int16_t magY()            { return _magY; };
    int16_t magZ()            { return _magZ; };
    float   magT()            { return _magT; };
    float   temp()            { return _temp; };
    float   tempT()           { return _tempT; };
    float   gyroProcX()       { return _gyroProcX; };
    float   gyroProcY()       { return _gyroProcY; };
    float   gyroProcZ()       { return _gyroProcZ; };
    float   gyroProcT()       { return _gyroProcT; };
    float   accelProcX()      { return _accelProcX; };
    float   accelProcY()      { return _accelProcY; };
    float   accelProcZ()      { return _accelProcZ; };
    float   accelProcT()      { return _accelProcT; };
    float   magProcX()        { return _magProcX; };
    float   magProcY()        { return _magProcY; };
    float   magProcZ()        { return _magProcZ; };
    float   magProcT()        { return _magProcT; };
    float   quatA()           { return _quatA; };
    float   quatB()           { return _quatB; };
    float   quatC()           { return _quatC; };
    float   quatD()           { return _quatD; };
    float   quatT()           { return _quatT; };
    float   roll()            { return _roll; };
    float   pitch()           { return _pitch; };
    float   yaw()             { return _yaw; };
    float   rollRate()        { return _rollRate; };
    float   pitchRate()       { return _pitchRate; };
    float   yawRate()         { return _yawRate; };
    float   eulerT()          { return _eulerT; };
    float   posNorth()        { return _posNorth; };
    float   posEast()         { return _posEast; };
    float   posUp()           { return _posUp; };
    float   posT()            { return _posT; };
    float   velNorth()        { return _velNorth; };
    float   velEast()         { return _velEast; };
    float   velUp()           { return _velUp; };
    float   velT()            { return _velT; };
    float   gpsLat()          { return _gpsLat; };
    float   gpsLon()          { return _gpsLon; };
    float   gpsAlt()          { return _gpsAlt; };
    float   gpsCOG()          { return _gpsCOG; };
    float   gpsSOG()          { return _gpsSOG; };
    float   gpsT()            { return _gpsT; };
    uint8_t sat1ID()          { return _sat1ID; };
    uint8_t sat1SNR()         { return _sat1SNR; };
    uint8_t sat2ID()          { return _sat2ID; };
    uint8_t sat2SNR()         { return _sat2SNR; };
    uint8_t sat3ID()          { return _sat3ID; };
    uint8_t sat3SNR()         { return _sat3SNR; };
    uint8_t sat4ID()          { return _sat4ID; };
    uint8_t sat4SNR()         { return _sat4SNR; };
    uint8_t sat5ID()          { return _sat5ID; };
    uint8_t sat5SNR()         { return _sat5SNR; };
    uint8_t sat6ID()          { return _sat6ID; };
    uint8_t sat6SNR()         { return _sat6SNR; };
    uint8_t sat7ID()          { return _sat7ID; };
    uint8_t sat7SNR()         { return _sat7SNR; };
    uint8_t sat8ID()          { return _sat8ID; };
    uint8_t sat8SNR()         { return _sat8SNR; };
    uint8_t sat9ID()          { return _sat9ID; };
    uint8_t sat9SNR()         { return _sat9SNR; };
    uint8_t sat10ID()         { return _sat10ID; };
    uint8_t sat10SNR()        { return _sat10SNR; };
    uint8_t sat11ID()         { return _sat11ID; };
    uint8_t sat11SNR()        { return _sat11SNR; };
    uint8_t sat12ID()         { return _sat12ID; };
    uint8_t sat12SNR()        { return _sat12SNR; };
    float   gyroBiasX()       { return _gyroBiasX; };
    float   gyroBiasY()       { return _gyroBiasY; };
    float   gyroBiasZ()       { return _gyroBiasZ; };
    char*   fwRevision()      { return _fwRevision; };




private: // <<---------------------------------------//private
    Stream* port;
    Stream* debug_port;

    bool debug;

    byte buff[UM7_IMU::MAX_PACKET_SIZE];

    byte pt;
    byte addr;
    int bytesToRec;
    uint16_t cs;

    bool hasData;
    bool isBatch;
    bool bl3;
    bool bl2;
    bool bl1;
    bool bl0;
    bool isHidden;

    int startCnt;
    int bytesRead;
    int numDataBytes;
    int payIndex;
    int status = UM7_IMU::CONTINUE;
    UM7_IMU::parse_state state = UM7_IMU::PARSE_S;

    bool _commandFailed;



    int16_t _baud;
    int16_t _gpsBaud;
    uint8_t _gpsAutoTx;
    uint8_t _satAutoTx;
    int16_t _rawAccelRate;
    int16_t _rawGyroRate;
    int16_t _rawMagRate;
    int16_t _tempRate;
    int16_t _allRawRate;
    int16_t _procAccelRate;
    int16_t _procGyroRate;
    int16_t _procMagRate;
    int16_t _allProcRate;
    int16_t _quatRate;
    int16_t _eulerRate;
    int16_t _posRate;
    int16_t _velRate;
    int16_t _poseRate;
    int16_t _healthRate;
    int16_t _gyroBiasRate;
    int16_t _healthNMEARate;
    int16_t _poseNMEARate;
    int16_t _attNMEARate;
    int16_t _sensorNMEARate;
    int16_t _ratesNMEARate;
    int16_t _gpsPoseNMEARate;
    int16_t _quatNMEARate;
    uint8_t _pps;
    uint8_t _zg;
    uint8_t _q;
    uint8_t _mag;
    float   _northHomeLat;
    float   _eastHomeLon;
    float   _homeUp;
    float   _gyroTrimX;
    float   _gyroTrimY;
    float   _gyroTrimZ;
    float   _magCal11;
    float   _magCal12;
    float   _magCal13;
    float   _magCal21;
    float   _magCal22;
    float   _magCal23;
    float   _magCal31;
    float   _magCal32;
    float   _magCal33;
    float   _magBiasX;
    float   _magBiasY;
    float   _magBiasZ;
    float   _accelCal11;
    float   _accelCal12;
    float   _accelCal13;
    float   _accelCal21;
    float   _accelCal22;
    float   _accelCal23;
    float   _accelCal31;
    float   _accelCal32;
    float   _accelCal33;
    float   _accelBiasX;
    float   _accelBiasY;
    float   _accelBiasZ;
    uint8_t _satsUsed;
    int16_t _hdop;
    uint8_t _satsInView;
    uint8_t _ovf;
    uint8_t _mgN;
    uint8_t _accN;
    uint8_t _accelFail;
    uint8_t _gyroFail;
    uint8_t _magFail;
    uint8_t _gps;
    int16_t _gyroX;
    int16_t _gyroY;
    int16_t _gyroZ;
    float   _gyroT;
    int16_t _acclX;
    int16_t _acclY;
    int16_t _acclZ;
    float   _acclT;
    int16_t _magX;
    int16_t _magY;
    int16_t _magZ;
    float   _magT;
    float   _temp;
    float   _tempT;
    float   _gyroProcX;
    float   _gyroProcY;
    float   _gyroProcZ;
    float   _gyroProcT;
    float   _accelProcX;
    float   _accelProcY;
    float   _accelProcZ;
    float   _accelProcT;
    float   _magProcX;
    float   _magProcY;
    float   _magProcZ;
    float   _magProcT;
    float   _quatA;
    float   _quatB;
    float   _quatC;
    float   _quatD;
    float   _quatT;
    float   _roll;
    float   _pitch;
    float   _yaw;
    float   _rollRate;
    float   _pitchRate;
    float   _yawRate;
    float   _eulerT;
    float   _posNorth;
    float   _posEast;
    float   _posUp;
    float   _posT;
    float   _velNorth;
    float   _velEast;
    float   _velUp;
    float   _velT;
    float   _gpsLat;
    float   _gpsLon;
    float   _gpsAlt;
    float   _gpsCOG;
    float   _gpsSOG;
    float   _gpsT;
    uint8_t _sat1ID;
    uint8_t _sat1SNR;
    uint8_t _sat2ID;
    uint8_t _sat2SNR;
    uint8_t _sat3ID;
    uint8_t _sat3SNR;
    uint8_t _sat4ID;
    uint8_t _sat4SNR;
    uint8_t _sat5ID;
    uint8_t _sat5SNR;
    uint8_t _sat6ID;
    uint8_t _sat6SNR;
    uint8_t _sat7ID;
    uint8_t _sat7SNR;
    uint8_t _sat8ID;
    uint8_t _sat8SNR;
    uint8_t _sat9ID;
    uint8_t _sat9SNR;
    uint8_t _sat10ID;
    uint8_t _sat10SNR;
    uint8_t _sat11ID;
    uint8_t _sat11SNR;
    uint8_t _sat12ID;
    uint8_t _sat12SNR;
    float   _gyroBiasX;
    float   _gyroBiasY;
    float   _gyroBiasZ;
    char    _fwRevision[5];




    int parse(const byte& recByte, const bool& valid);
    uint16_t calcChecksum();
    void reset();
    int parseData(const int& addrOffset = 0, const int& dataOffset = 0);
};
