#ifndef __READ_PARSE_PACKAGE_H__
#define __READ_PARSE_PACKAGE_H__

#include <cstdint>
#include "HC_serial.h"

#define PACKAGESIZE 128

#ifndef BYTE
typedef unsigned char       BYTE;
#endif

struct rangedata
{
    bool flag;                          // 0->valid, 1->invalid
    float angle;                       // degree
    unsigned int dist;            // millimeter
    unsigned int speed;
    uint16_t syn_quality;
    rangedata() :
        flag(false),
        angle(0.),
        dist(0),
        speed(0),
        syn_quality(0) {}
};
//
//enum LiDarErrorCode
//{
//    LIDAR_SUCCESS = 0,
//    //??????????????
//    ERR_SERIAL_INVALID_HANDLE = -1001,//?????????
//    ERR_SERIAL_SETCOMMTIMEOUTS_FAILED = -1002,//???????????????????
//    ERR_SERIAL_READFILE_FAILED = -1003,//??????????
//    ERR_SERIAL_READFILE_ZERO = -1004,//?????????????
//    ERR_FIND_HEAD_TIMEOUT = -1005,//?????????
//    ERR_CHECKDATA_INVALID = -1006,//��??????
//    ERR_GETPACKAGE_FAILED = -1007,//????????
//    ERR_DATABYTELENGTH_INVALID = -1008,//???????????��????
//    ERR_RECEIVE_BUFFER_SMALL = -1009,//?????buffer??????��?��
//
//    //????????????
//    ERR_LIDAR_FPS_INVALID = -3001,//fps??
//    ERR_LIDAR_SPEED_LOW = -3002,//??????
//    ERR_LIDAR_SPEED_HIGH = -3003,//??????
//    ERR_LIDAR_NUMBER_INVALID = -3004,//??��??????
//	ERR_LIDAR_SAME_ANGLE = -3005, //????????????
//
//};

class ReadParsePackage
{
public:
    ReadParsePackage();
    ~ReadParsePackage();
    ///?????????????  ???�s?????????????????????????????
    int OpenSerial(char* port, unsigned int baud);
    void CloseSerial();
    int ReadDataSerial(unsigned int TimeOut_ms = 60);
    int ParseDataSerial(rangedata* dataPack, int& fps);
    ///--------------------?????????????

    void SetStartTimestamp()
    {
        startTimeStamp_ = GetTimeStamp();
    }
     //?????????
    int GetLastErrCode()
    {
        int err = m_lastErrorCode;
        if (m_lastErrorCode != 0)
            m_lastErrorCode = 0;
        return err;
    }

    void SetLastErrCode(int err)
    {
        m_lastErrorCode = err;
    }

    int data_num_per_pack_;

private:
    int64_t GetTimeStamp();
    bool CheckLidarData(BYTE* buffer, int len);
    void CheckInvalidFPS(int validNumber);

    void CheckInvalidLowSpeed(unsigned int speed);
    void CheckInvalidHighSpeed(unsigned int speed);

    int GetDataByteLength(char cInfo);
    void SetReadCharsError(int errCode);

    int ParseLidarData(BYTE* in_buffer, const int in_numData, rangedata* out_dataPack);

    int ReadSurplusData(unsigned int TimeOut_ms);
    int ReadLidarData(unsigned int TimeOut_ms, BYTE cInfo);

    BYTE rcvbuffer_[PACKAGESIZE];
    
    int unitDataSize_;

    int data_num_total_;
    int data_rate_hz_;
    //TimeOut data_timer_;
    int64_t startTimeStamp_;
	HC_serial serial_;

    int m_lastErrorCode = 0;

    int invalidFPSSecond_ = 0;

    int64_t startTimeLowSpeed_ = 0;
    int64_t startTimeHighSpeed_ = 0;
};

#endif