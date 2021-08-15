#pragma once
#include <TinyGPS++.h>
#include <HardwareSerial.h>

//#define GPS_TX 34
//#define GPS_RX 12

#define GPS_TX GPS_TX_PIN
#define GPS_RX GPS_RX_PIN

class gps
{
public:
    void init();
    bool checkGpsFix();
    void buildPacket(uint8_t txBuffer[9]);
    void encode();
    
    double getLatitude()
    {
        return tGps.location.lat();
    };
    double getLongitude()
    {
        return tGps.location.lng();
    };
    double distanceTo(double lat2, double lon2);

private:
    uint32_t LatitudeBinary, LongitudeBinary;
    uint16_t altitudeGps;
    uint8_t hdopGps;
    uint32_t sats;
    char t[32]; // used to sprintf for Serial output
    TinyGPSPlus tGps;
    double distance;
};
