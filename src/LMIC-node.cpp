/*******************************************************************************
 *
 *  File:          LMIC-node.cpp
 * 
 *  Function:      LMIC-node main application file.
 * 
 *  Copyright:     Copyright (c) 2021 Leonel Lopes Parente
 *                 Copyright (c) 2018 Terry Moore, MCCI
 *                 Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 *                 Permission is hereby granted, free of charge, to anyone 
 *                 obtaining a copy of this document and accompanying files to do, 
 *                 whatever they want with them without any restriction, including,
 *                 but not limited to, copying, modification and redistribution.
 *                 The above copyright notice and this permission notice shall be 
 *                 included in all copies or substantial portions of the Software.
 * 
 *                 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT ANY WARRANTY.
 * 
 *  License:       MIT License. See accompanying LICENSE file.
 * 
 *  Author:        Leonel Lopes Parente
 * 
 *  Description:   To get LMIC-node up and running no changes need to be made
 *                 to any source code. Only configuration is required
 *                 in platform-io.ini and lorawan-keys.h.
 * 
 *                 If you want to modify the code e.g. to add your own sensors,
 *                 that can be done in the two area's that start with
 *                 USER CODE BEGIN and end with USER CODE END. There's no need
 *                 to change code in other locations (unless you have a reason).
 *                 See README.md for documentation and how to use LMIC-node.
 * 
 *                 LMIC-node uses the concepts from the original ttn-otaa.ino 
 *                 and ttn-abp.ino examples provided with the LMIC libraries.
 *                 LMIC-node combines both OTAA and ABP support in a single example,
 *                 supports multiple LMIC libraries, contains several improvements
 *                 and enhancements like display support, support for downlinks,
 *                 separates LoRaWAN keys from source code into a separate keyfile,
 *                 provides formatted output to serial port and display
 *                 and supports many popular development boards out of the box.
 *                 To get a working node up and running only requires some configuration.
 *                 No programming or customization of source code required.
 * 
 *  Dependencies:  External libraries:
 *                 MCCI LoRaWAN LMIC library  https://github.com/mcci-catena/arduino-lmic
 *                 IBM LMIC framework         https://github.com/matthijskooijman/arduino-lmic  
 *                 U8g2                       https://github.com/olikraus/u8g2
 *                 EasyLed                    https://github.com/lnlp/EasyLed
 *
 ******************************************************************************/

#include "LMIC-node.h"

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

#include "gps.h"
gps gps1;
#include <driver/adc.h>
#define REF_VCCRAW_ESP32 949 // VCCraw read from ESP32 
#define REF_VBAT 3862 // Voltage in mV at battery

double lastLat = HOME_LATITUDE;
double lastLon = HOME_LONGITUDE;

const uint8_t payloadBufferLength = 12; // Adjust to fit max payload length

long ReadVBat()
{
  Serial.print("ReadVBat = ");
  long vccraw = 0;
  int read_raw;
  int samples_ok = 0;
  long raw = 0;

  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // Use ADC_ATTEN_DB_11 for higher voltage range

  for (int samples = 0; samples < 50; samples++) // Start from 0 for consistency
  {
    read_raw = adc1_get_raw(ADC1_CHANNEL_3);

    if (read_raw >= 0)
    {
      samples_ok++;
      raw += read_raw;
    }
    delayMicroseconds(500); // Reduce delay for faster execution
  }

  if (samples_ok > 0)
  {
    // Convert ADC reading to voltage in millivolts (assuming 3.9V max ADC input)
    float vccraw_f = ((float)raw / samples_ok) * (3.9 / 4095) * 1000; 
    float vBat_f = vccraw_f * ((float)REF_VBAT / (float)REF_VCCRAW_ESP32);
    float vBat_o = vBat_f;

    // Apply calibration formula
    vBat_f = (0.85* vBat_f) + 637;

    long vccraw = (long)vccraw_f;
    long vBat = (long)vBat_f;

    Serial.print("VCCraw: ");
    Serial.print(vccraw);
    Serial.print(" mV, vBat (corrected): ");
    Serial.print(vBat_f);
    //Serial.println(" mV, vBat (Original):");
    //Serial.print(vBat_o);
    //Serial.print(" mV");
    return vBat;
  }
  else
  {
    Serial.println("No valid samples.");
    return 0;
  }
}




//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

uint8_t payloadBuffer[payloadBufferLength];
static osjob_t doWorkJob;
static osjob_t doWorkJobLong;  // Add a second job for long intervals
uint32_t doWorkIntervalSeconds = DO_WORK_INTERVAL_SECONDS; // Change value in platformio.ini
uint32_t doWorkIntervalLongSeconds = DO_WORK_INTERVAL_LONG_SECONDS; // Change value in platformio.ini

// Note: LoRa module pin mappings are defined in the Board Support Files.

// Set LoRaWAN keys defined in lorawan-keys.h.
#ifdef OTAA_ACTIVATION
static const u1_t PROGMEM DEVEUI[8] = {OTAA_DEVEUI};
static const u1_t PROGMEM APPEUI[8] = {OTAA_APPEUI};
static const u1_t PROGMEM APPKEY[16] = {OTAA_APPKEY};
// Below callbacks are used by LMIC for reading above values.
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#else
    // ABP activation
static const u4_t DEVADDR = ABP_DEVADDR;
static const PROGMEM u1_t NWKSKEY[16] = {ABP_NWKSKEY};
static const u1_t PROGMEM APPSKEY[16] = {ABP_APPSKEY};
// Below callbacks are not used be they must be defined.
void os_getDevEui(u1_t *buf) {}
void os_getArtEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#endif

int16_t getSnrTenfold()
{
    // Returns ten times the SNR (dB) value of the last received packet.
    // Ten times to prevent the use of float but keep 1 decimal digit accuracy.
    // Calculation per SX1276 datasheet rev.7 §6.4, SX1276 datasheet rev.4 §6.4.
    // LMIC.snr contains value of PacketSnr, which is 4 times the actual SNR value.
    return (LMIC.snr * 10) / 4;
}

int16_t getRssi(int8_t snr)
{
    // Returns correct RSSI (dBm) value of the last received packet.
    // Calculation per SX1276 datasheet rev.7 §5.5.5, SX1272 datasheet rev.4 §5.5.5.

#define RSSI_OFFSET 64
#define SX1276_FREQ_LF_MAX 525000000 // per datasheet 6.3
#define SX1272_RSSI_ADJUST -139
#define SX1276_RSSI_ADJUST_LF -164
#define SX1276_RSSI_ADJUST_HF -157

    int16_t rssi;

#ifdef MCCI_LMIC

    rssi = LMIC.rssi - RSSI_OFFSET;

#else
    int16_t rssiAdjust;
#ifdef CFG_sx1276_radio
    if (LMIC.freq > SX1276_FREQ_LF_MAX)
    {
        rssiAdjust = SX1276_RSSI_ADJUST_HF;
    }
    else
    {
        rssiAdjust = SX1276_RSSI_ADJUST_LF;
    }
#else
    // CFG_sx1272_radio
    rssiAdjust = SX1272_RSSI_ADJUST;
#endif

    // Revert modification (applied in lmic/radio.c) to get PacketRssi.
    int16_t packetRssi = LMIC.rssi + 125 - RSSI_OFFSET;
    if (snr < 0)
    {
        rssi = rssiAdjust + packetRssi + snr;
    }
    else
    {
        rssi = rssiAdjust + (16 * packetRssi) / 15;
    }
#endif

    return rssi;
}

void printEvent(ostime_t timestamp,
                const char *const message,
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true,
                bool eventLabel = false)
{
#ifdef USE_DISPLAY
    if (target == PrintTarget::All || target == PrintTarget::Display)
    {
        display.clearLine(TIME_ROW);
        display.setCursor(COL_0, TIME_ROW);
        display.print(F("Time:"));
        display.print(timestamp);
        display.clearLine(EVENT_ROW);
        if (clearDisplayStatusRow)
        {
            display.clearLine(STATUS_ROW);
        }
        display.setCursor(COL_0, EVENT_ROW);
        display.print(message);
    }
#endif

#ifdef USE_SERIAL
    // Create padded/indented output without using printf().
    // printf() is not default supported/enabled in each Arduino core.
    // Not using printf() will save memory for memory constrainted devices.
    String timeString(timestamp);
    uint8_t len = timeString.length();
    uint8_t zerosCount = TIMESTAMP_WIDTH > len ? TIMESTAMP_WIDTH - len : 0;

    if (target == PrintTarget::All || target == PrintTarget::Serial)
    {
        printChars(serial, '0', zerosCount);
        serial.print(timeString);
        serial.print(":  ");
        if (eventLabel)
        {
            serial.print(F("Event: "));
        }
        serial.println(message);
    }
#endif
}

void printEvent(ostime_t timestamp,
                ev_t ev,
                PrintTarget target = PrintTarget::All,
                bool clearDisplayStatusRow = true)
{
#if defined(USE_DISPLAY) || defined(USE_SERIAL)
    printEvent(timestamp, lmicEventNames[ev], target, clearDisplayStatusRow, true);
#endif
}

void printFrameCounters(PrintTarget target = PrintTarget::All)
{
#ifdef USE_DISPLAY
    if (target == PrintTarget::Display || target == PrintTarget::All)
    {
        display.clearLine(FRMCNTRS_ROW);
        display.setCursor(COL_0, FRMCNTRS_ROW);
        display.print(F("Up:"));
        display.print(LMIC.seqnoUp);
        display.print(F(" Dn:"));
        display.print(LMIC.seqnoDn);
    }
#endif

#ifdef USE_SERIAL
    if (target == PrintTarget::Serial || target == PrintTarget::All)
    {
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Up: "));
        serial.print(LMIC.seqnoUp);
        serial.print(F(",  Down: "));
        serial.println(LMIC.seqnoDn);
    }
#endif
}

void printSessionKeys()
{
#if defined(USE_SERIAL) && defined(MCCI_LMIC)
    u4_t networkId = 0;
    devaddr_t deviceAddress = 0;
    u1_t networkSessionKey[16];
    u1_t applicationSessionKey[16];
    LMIC_getSessionKeys(&networkId, &deviceAddress,
                        networkSessionKey, applicationSessionKey);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Network Id: "));
    serial.println(networkId, DEC);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Device Address: "));
    serial.println(deviceAddress, HEX);

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Application Session Key: "));
    printHex(serial, applicationSessionKey, 16, true, '-');

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Network Session Key:     "));
    printHex(serial, networkSessionKey, 16, true, '-');
#endif
}

void printDownlinkInfo(void)
{
#if defined(USE_SERIAL) || defined(USE_DISPLAY)

    uint8_t dataLength = LMIC.dataLen;
    // bool ackReceived = LMIC.txrxFlags & TXRX_ACK;

    int16_t snrTenfold = getSnrTenfold();
    int8_t snr = snrTenfold / 10;
    int8_t snrDecimalFraction = snrTenfold % 10;
    int16_t rssi = getRssi(snr);

    uint8_t fPort = 0;
    if (LMIC.txrxFlags & TXRX_PORT)
    {
        fPort = LMIC.frame[LMIC.dataBeg - 1];
    }

#ifdef USE_DISPLAY
    display.clearLine(EVENT_ROW);
    display.setCursor(COL_0, EVENT_ROW);
    display.print(F("RX P:"));
    display.print(fPort);
    if (dataLength != 0)
    {
        display.print(" Len:");
        display.print(LMIC.dataLen);
    }
    display.clearLine(STATUS_ROW);
    display.setCursor(COL_0, STATUS_ROW);
    display.print(F("RSSI"));
    display.print(rssi);
    display.print(F(" SNR"));
    display.print(snr);
    display.print(".");
    display.print(snrDecimalFraction);
#endif

#ifdef USE_SERIAL
    printSpaces(serial, MESSAGE_INDENT);
    serial.println(F("Downlink received"));

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("RSSI: "));
    serial.print(rssi);
    serial.print(F(" dBm,  SNR: "));
    serial.print(snr);
    serial.print(".");
    serial.print(snrDecimalFraction);
    serial.println(F(" dB"));

    printSpaces(serial, MESSAGE_INDENT);
    serial.print(F("Port: "));
    serial.println(fPort);

    if (dataLength != 0)
    {
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Length: "));
        serial.println(LMIC.dataLen);
        printSpaces(serial, MESSAGE_INDENT);
        serial.print(F("Data: "));
        printHex(serial, LMIC.frame + LMIC.dataBeg, LMIC.dataLen, true, ' ');
    }
#endif
#endif
}

void printHeader(void)
{
#ifdef USE_DISPLAY
    display.clear();
    display.setCursor(COL_0, HEADER_ROW);
    display.print(F("LMIC-node"));
#ifdef ABP_ACTIVATION
    display.drawString(ABPMODE_COL, HEADER_ROW, "ABP");
#endif
#ifdef CLASSIC_LMIC
    display.drawString(CLMICSYMBOL_COL, HEADER_ROW, "*");
#endif
    display.drawString(COL_0, DEVICEID_ROW, deviceId);
    display.setCursor(COL_0, INTERVAL_ROW);
    display.print(F("Interval:"));
    display.print(doWorkIntervalSeconds);
    display.print("s");
#endif

#ifdef USE_SERIAL
    serial.println(F("\n\nLMIC-node\n"));
    serial.print(F("Device-id:     "));
    serial.println(deviceId);
    serial.print(F("LMIC library:  "));
#ifdef MCCI_LMIC
    serial.println(F("MCCI"));
#else
    serial.println(F("Classic [Deprecated]"));
#endif
    serial.print(F("Activation:    "));
#ifdef OTAA_ACTIVATION
    serial.println(F("OTAA"));
#else
    serial.println(F("ABP"));
#endif
#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
    serial.print(F("LMIC debug:    "));
    serial.println(LMIC_DEBUG_LEVEL);
#endif
    serial.print(F("Interval:      "));
    serial.print(doWorkIntervalSeconds);
    serial.println(F(" seconds"));
    if (activationMode == ActivationMode::OTAA)
    {
        serial.println();
    }
#endif
}

#ifdef ABP_ACTIVATION
void setAbpParameters(dr_t dataRate = DR_SF7, s1_t txPower = 14)
{
// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
#elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
#endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(dataRate, txPower);
}
#endif //ABP_ACTIVATION

void initLmic(bit_t adrEnabled = 1,
              dr_t dataRate = DR_SF7,
              s1_t txPower = 14,
              bool setDrTxPowForOtaaExplicit = false)
{
    // Initialize LMIC runtime environment
    os_init();
    // Reset MAC state
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

#ifdef ABP_ACTIVATION
    setAbpParameters(dataRate, txPower);
#endif

    // Enable or disable ADR (data rate adaptation).
    // Should be turned off if the device is not stationary (mobile).
    // 1 is on, 0 is off.
    LMIC_setAdrMode(adrEnabled);

    if (activationMode == ActivationMode::OTAA)
    {
#if defined(CFG_us915) || defined(CFG_au915)
        // NA-US and AU channels 0-71 are configured automatically
        // but only one group of 8 should (a subband) should be active
        // TTN recommends the second sub band, 1 in a zero based count.
        // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
        LMIC_selectSubBand(1);
#endif

        // Optional: set/override data rate and transmit power for OTAA (only use if ADR is disabled).
        if (setDrTxPowForOtaaExplicit && !adrEnabled)
        {
            // Below dataRate will be overridden again when joined.
            LMIC_setDrTxpow(dataRate, txPower);
        }
    }

// Relax LMIC timing if defined
#if defined(LMIC_CLOCK_ERROR_PPM) && LMIC_CLOCK_ERROR_PPM > 0
#if defined(MCCI_LMIC) && LMIC_CLOCK_ERROR_PPM > 4000
// Allow clock error percentage to be > 0.4%
#define LMIC_ENABLE_arbitrary_clock_error 1
#endif
    uint32_t clockError = (LMIC_CLOCK_ERROR_PPM / 100) * (MAX_CLOCK_ERROR / 100) / 100;
    LMIC_setClockError(clockError);

#ifdef USE_SERIAL
    serial.print(F("Clock Error:   "));
    serial.print(LMIC_CLOCK_ERROR_PPM);
    serial.print(" ppm (");
    serial.print(clockError);
    serial.println(")");
#endif
#endif

#ifdef MCCI_LMIC
    // Register a custom eventhandler and don't use default onEvent() to enable
    // additional features (e.g. make EV_RXSTART available). User data pointer is omitted.
    LMIC_registerEventCb(&onLmicEvent, nullptr);
#endif
}

#ifdef MCCI_LMIC
void onLmicEvent(void *pUserData, ev_t ev)
#else
void onEvent(ev_t ev)
#endif
{
    // LMIC event handler
    ostime_t timestamp = os_getTime();

    switch (ev)
    {
#ifdef MCCI_LMIC
    // Only supported in MCCI LMIC library:
    case EV_RXSTART:
        // Do not print anything for this event or it will mess up timing.
        break;

    case EV_TXSTART:
        setTxIndicatorsOn();
        printEvent(timestamp, ev);
        break;

    case EV_JOIN_TXCOMPLETE:
    case EV_TXCANCELED:
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        break;
#endif
    case EV_JOINED:
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        printSessionKeys();

        // Disable link check validation.
        // Link check validation is automatically enabled
        // during join, but because slow data rates change
        // max TX size, it is not used in this example.
        LMIC_setLinkCheckMode(0);

        // The doWork job has probably run already (while
        // the node was still joining) and have rescheduled itself.
        // Cancel the next scheduled doWork job and re-schedule
        // for immediate execution to prevent that any uplink will
        // have to wait until the current doWork interval ends.
        os_clearCallback(&doWorkJob);
        os_setCallback(&doWorkJob, doWorkCallback);
        break;

    case EV_TXCOMPLETE:
        // Transmit completed, includes waiting for RX windows.
        setTxIndicatorsOn(false);
        printEvent(timestamp, ev);
        printFrameCounters();

        // Check if downlink was received
        if (LMIC.dataLen != 0 || LMIC.dataBeg != 0)
        {
            uint8_t fPort = 0;
            if (LMIC.txrxFlags & TXRX_PORT)
            {
                fPort = LMIC.frame[LMIC.dataBeg - 1];
            }
            printDownlinkInfo();
            processDownlink(timestamp, fPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        }
        break;

    // Below events are printed only.
    case EV_SCAN_TIMEOUT:
    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_RFU1: // This event is defined but not used in code
    case EV_JOINING:
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
#ifdef MCCI_LMIC
    // Only supported in MCCI LMIC library:
    case EV_SCAN_FOUND: // This event is defined but not used in code
#endif
        printEvent(timestamp, ev);
        break;

    default:
        printEvent(timestamp, "Unknown Event");
        break;
    }
}

static void doWorkCallback(osjob_t *job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWork() which is called below.

    ostime_t timestamp = os_getTime();
#ifdef USE_SERIAL
    serial.println();
    printEvent(timestamp, "doWork job started", PrintTarget::Serial);
#endif

    // Do the work that needs to be performed.
    processWork(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalSeconds);
    os_setTimedCallback(&doWorkJob, startAt, doWorkCallback);

}

static void doWorkCallbackLong(osjob_t *job)
{
    // Event hander for doWorkJob. Gets called by the LMIC scheduler.
    // The actual work is performed in function processWorkLong() which is called below.

    ostime_t timestamp = os_getTime();
#ifdef USE_SERIAL
    serial.println();
    printEvent(timestamp, "doWork long job started", PrintTarget::Serial);
#endif

    // Do the work that needs to be performed.
    processWorkLong(timestamp);

    // This job must explicitly reschedule itself for the next run.
    ostime_t startAt = timestamp + sec2osticks((int64_t)doWorkIntervalLongSeconds);
    os_setTimedCallback(&doWorkJobLong, startAt, doWorkCallbackLong);
}

lmic_tx_error_t scheduleUplink(uint8_t fPort, uint8_t *data, uint8_t dataLength, bool confirmed = false)
{
    // This function is called from the processWork() function to schedule
    // transmission of an uplink message that was prepared by processWork().
    // Transmission will be performed at the next possible time

    ostime_t timestamp = os_getTime();
    printEvent(timestamp, "Packet queued");

    lmic_tx_error_t retval = LMIC_setTxData2(fPort, data, dataLength, confirmed ? 1 : 0);
    timestamp = os_getTime();

    if (retval == LMIC_ERROR_SUCCESS)
    {
#ifdef CLASSIC_LMIC
        // For MCCI_LMIC this will be handled in EV_TXSTART
        setTxIndicatorsOn();
#endif
    }
    else
    {
        String errmsg;
#ifdef USE_SERIAL
        errmsg = "LMIC Error: ";
#ifdef MCCI_LMIC
        errmsg.concat(lmicErrorNames[abs(retval)]);
#else
        errmsg.concat(retval);
#endif
        printEvent(timestamp, errmsg.c_str(), PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
        errmsg = "LMIC Err: ";
        errmsg.concat(retval);
        printEvent(timestamp, errmsg.c_str(), PrintTarget::Display);
#endif
    }
    return retval;
}

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

void setUplinkInterval(uint8_t interval)
{
    doWorkIntervalSeconds = interval;
    Serial.print("Interval set to ");
    Serial.print(interval);
    Serial.println(" sec.");
}

void processWork(ostime_t doWorkJobTimeStamp)
{
    // This function is called from the doWorkCallback()
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWork if using OTAA and still joining.
    if (LMIC.devaddr != 0)
    {
        // Collect input data.
        ostime_t timestamp = os_getTime();

#ifdef USE_DISPLAY
        // Interval and Counter values are combined on a single row.
        // This allows to keep the 3rd row empty which makes the
        // information better readable on the small display.
        display.clearLine(INTERVAL_ROW);
        display.setCursor(COL_0, INTERVAL_ROW);
        display.print("I:");
        display.print(doWorkIntervalSeconds);
        display.print("s");
#endif
#ifdef USE_SERIAL
        printEvent(timestamp, "Input data collected", PrintTarget::Serial);
        printSpaces(serial, MESSAGE_INDENT);
        serial.println();
#endif

        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.
        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
            // TxRx is currently pending, do not send.
#ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
#endif
        }
        else
        {
            // Send uplink only if
            //   a) a valid gps fix
            //   b) outside "MIN_DISTANCE" (meter) from given location "HOME_LATITUDE", "HOME_LONGITUDE"
            //   c) new gps position differs more than "GPS_DEVIATION" (meter) from previous position
            // get gps data
            
            long vBat = ReadVBat(); // Read Battery Voltage
            uint16_t vbatx = (vBat / 10) - 200;

            // Check if vBat is greater than or equal to 4.16V (4160mV)
            if (vBat >= 4160) {
                led2.on(); // Turn LED on
            } else {
                led2.off(); // Turn LED off
            }
            
            if (gps1.checkGpsFix())
            {
                led3.off();

#ifdef USE_SERIAL
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println("Valid gps Fix (NORMAL INTERVAL)");
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println("Current Position:");
                char s[32];
                sprintf(s, "Lat: %f", gps1.getLatitude());
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println(s);
                sprintf(s, "Lon: %f", gps1.getLongitude());
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println(s);
                printSpaces(serial, MESSAGE_INDENT);
                Serial.print("Distance to start point = ");
                Serial.print(gps1.distanceTo(HOME_LATITUDE, HOME_LONGITUDE));
                Serial.print(" m. ");
                Serial.print("Set to: ");
                Serial.print(MIN_DISTANCE);
                Serial.println(" m.");
                printSpaces(serial, MESSAGE_INDENT);
                Serial.print("Difference between last position: ");
                Serial.print(gps1.distanceTo(lastLat, lastLon));
                Serial.print(" m.");
                Serial.print(" Set to: ");
                Serial.print(GPS_DEVIATION);
                Serial.println(" m.");
                //Serial.print("Debug: lastLat = ");
                //Serial.println(lastLat, 6);
                //Serial.print("Debug: lastLon = ");
                //Serial.println(lastLon, 6);
#endif           
                
                
                if (gps1.distanceTo(HOME_LATITUDE, HOME_LONGITUDE) >= MIN_DISTANCE)
                {
                    if (gps1.distanceTo(lastLat, lastLon) >= GPS_DEVIATION)
                    {

#ifdef USE_SERIAL
                        printSpaces(serial, MESSAGE_INDENT);
                        Serial.println("Update to TTN.");
#endif

                        uint8_t txBuffer[11];
                        gps1.buildPacket(txBuffer);

                        // Prepare uplink payload.
                        uint8_t fPort = 10;
                        uint8_t payloadLength = 12;

                        payloadBuffer[0] = txBuffer[0];
                        payloadBuffer[1] = txBuffer[1];
                        payloadBuffer[2] = txBuffer[2];
                        payloadBuffer[3] = txBuffer[3];
                        payloadBuffer[4] = txBuffer[4];
                        payloadBuffer[5] = txBuffer[5];
                        payloadBuffer[6] = txBuffer[6];
                        payloadBuffer[7] = txBuffer[7];
                        payloadBuffer[8] = txBuffer[8];
                        payloadBuffer[9] = txBuffer[9];

                        payloadBuffer[10] = (vbatx >> 8) & 0xFF;  // HIGH byte
                        payloadBuffer[11] = vbatx & 0xFF;         // LOW byte

                        scheduleUplink(fPort, payloadBuffer, payloadLength);
                        
#ifdef RESET_DO_WORK_INTERVAL_LONG 
                        // Reset and reschedule the long interval timer
                        os_clearCallback(&doWorkJobLong);
                        ostime_t startAtLong = timestamp + sec2osticks((int64_t)doWorkIntervalLongSeconds);
                        os_setTimedCallback(&doWorkJobLong, startAtLong, doWorkCallbackLong);
#endif
                        
                    }
                    else
                    {
#ifdef USE_SERIAL
                        printSpaces(serial, MESSAGE_INDENT);
                        Serial.println("Almost the same location, no update to TTN.");
#endif
                    }
                }
                else
                {
#ifdef USE_SERIAL
                    printSpaces(serial, MESSAGE_INDENT);
                    Serial.println("Too close to start point, no update to TTN.");
#endif
                }
                lastLat = gps1.getLatitude();
                lastLon = gps1.getLongitude();
            }
            else
            {
#ifdef USE_SERIAL
                printSpaces(serial, MESSAGE_INDENT);
                serial.println(F("No gps fix, no update to TTN."));
                led3.flash(255,10,2000); //LED on When no GPS...
#endif
            }
        }
    }
}

// No downlink procedure implemented.

void processWorkLong(ostime_t doWorkJobTimeStamp)
{
    // THIS FUNCTION SHOULD BE CALLED every doWorkIntervalLongSeconds!!!!!!!!!
    // This function is called from the doWorkCallback()
    // callback function when the doWork job is executed.

    // Uses globals: payloadBuffer and LMIC data structure.

    // This is where the main work is performed like
    // reading sensor and GPS data and schedule uplink
    // messages if anything needs to be transmitted.

    // Skip processWorkLong if using OTAA and still joining.
    if (LMIC.devaddr != 0)
    {
        // Collect input data.
        ostime_t timestamp = os_getTime();

#ifdef USE_DISPLAY
        // Interval and Counter values are combined on a single row.
        // This allows to keep the 3rd row empty which makes the
        // information better readable on the small display.
        display.clearLine(INTERVAL_ROW);
        display.setCursor(COL_0, INTERVAL_ROW);
        display.print("I:");
        display.print(doWorkIntervalSeconds);
        display.print("s");
#endif
#ifdef USE_SERIAL
        printEvent(timestamp, "Input data collected", PrintTarget::Serial);
        printSpaces(serial, MESSAGE_INDENT);
        serial.println();
#endif

        // For simplicity LMIC-node will try to send an uplink
        // message every time processWork() is executed.
        // Schedule uplink message if possible
        if (LMIC.opmode & OP_TXRXPEND)
        {
            // TxRx is currently pending, do not send.
#ifdef USE_SERIAL
            printEvent(timestamp, "Uplink not scheduled because TxRx pending", PrintTarget::Serial);
#endif
#ifdef USE_DISPLAY
            printEvent(timestamp, "UL not scheduled", PrintTarget::Display);
#endif
        }
        else
        {
            // Send uplink only if
            //   a) a valid gps fix
            //  IGNORES MIN_DISTANCE=10 & GPS_DEVIATION=10!!!!!!      
            
            long vBat = ReadVBat(); // Read Battery Voltage
            uint16_t vbatx = (vBat / 10) - 200;
            
            if (gps1.checkGpsFix())
            {
                led3.off();

#ifdef USE_SERIAL
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println("Valid gps Fix (LONG INTERVAL)");
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println("Current Position:");
                char s[32];
                sprintf(s, "Lat: %f", gps1.getLatitude());
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println(s);
                sprintf(s, "Lon: %f", gps1.getLongitude());
                printSpaces(serial, MESSAGE_INDENT);
                Serial.println(s);
                printSpaces(serial, MESSAGE_INDENT);
                Serial.print("Distance to start point = ");
                Serial.print(gps1.distanceTo(HOME_LATITUDE, HOME_LONGITUDE));
                Serial.print(" m. ");
                Serial.print("Set to: ");
                Serial.print(MIN_DISTANCE);
                Serial.println(" m.");
                printSpaces(serial, MESSAGE_INDENT);
                Serial.print("Difference between last position: ");
                Serial.print(gps1.distanceTo(lastLat, lastLon));
                Serial.print(" m.");
                Serial.print(" Set to: ");
                Serial.print(GPS_DEVIATION);
                Serial.println(" m.");
                //Serial.print("Debug: lastLat = ");
                //Serial.println(lastLat, 6);
                //Serial.print("Debug: lastLon = ");
                //Serial.println(lastLon, 6);
#endif           
                
                
                

#ifdef USE_SERIAL
                        printSpaces(serial, MESSAGE_INDENT);
                        Serial.println("Update to TTN.");
#endif

                        uint8_t txBuffer[11];
                        gps1.buildPacket(txBuffer);

                        // Prepare uplink payload.
                        uint8_t fPort = 10;
                        uint8_t payloadLength = 12;

                        payloadBuffer[0] = txBuffer[0];
                        payloadBuffer[1] = txBuffer[1];
                        payloadBuffer[2] = txBuffer[2];
                        payloadBuffer[3] = txBuffer[3];
                        payloadBuffer[4] = txBuffer[4];
                        payloadBuffer[5] = txBuffer[5];
                        payloadBuffer[6] = txBuffer[6];
                        payloadBuffer[7] = txBuffer[7];
                        payloadBuffer[8] = txBuffer[8];
                        payloadBuffer[9] = txBuffer[9];

                        payloadBuffer[10] = (vbatx >> 8) & 0xFF;  // HIGH byte
                        payloadBuffer[11] = vbatx & 0xFF;         // LOW byte

                        scheduleUplink(fPort, payloadBuffer, payloadLength);
                        
                lastLat = gps1.getLatitude();
                lastLon = gps1.getLongitude();
            }
            else
            {
#ifdef USE_SERIAL
                printSpaces(serial, MESSAGE_INDENT);
                serial.println(F("No gps fix, no update to TTN."));
                led3.flash(255,10,2000); //LED on When no GPS...
#endif
            }
        }
    }
}

// No downlink procedure implemented.

void processDownlink(ostime_t txCompleteTimestamp, uint8_t fPort, uint8_t *data, uint8_t dataLength)
{
    // This function is called from the onEvent() event handler
    // on EV_TXCOMPLETE when a downlink message was received.

    // Implements a 'setUplinkInterval' command that can be sent via a downlink message.
    // To set the uplink interval to the node, send a downlink message
    // (e.g. from the TTN Console) with single byte value resetCmd on port cmdPort.

    const uint8_t cmdPort = 100;
    const uint8_t intervalSec[4] = {0x1E, 0x3C, 0x78, 0xB4};

    if (fPort == cmdPort && dataLength == 1 && (data[0] == intervalSec[0] || data[0] == intervalSec[1] || data[0] == intervalSec[2] || data[0] == intervalSec[3]))
    {
        setUplinkInterval(data[0]);

#ifdef USE_SERIAL
        printSpaces(serial, MESSAGE_INDENT);
        serial.println(F("Interval cmd received"));
        ostime_t timestamp = os_getTime();
        printEvent(timestamp, "Interval changed", PrintTarget::All, false);
#endif

    }
}

//  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
//  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
//  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

void setup()
{
    setCpuFrequencyMhz(20);
    // boardInit(InitType::Hardware) must be called at start of setup() before anything else.
    bool hardwareInitSucceeded = boardInit(InitType::Hardware);
    
    

#ifdef USE_DISPLAY
    initDisplay();
#endif

#ifdef USE_SERIAL
    initSerial(MONITOR_SPEED, WAITFOR_SERIAL_S);
#endif

    boardInit(InitType::PostInitSerial);
    

#if defined(USE_SERIAL) || defined(USE_DISPLAY)
    printHeader();
    serial.println(__FILE__);
    serial.println("WROOM GPS V2");
#endif

    if (!hardwareInitSucceeded)
    {
#ifdef USE_SERIAL
        serial.println(F("Error: hardware init failed."));
        serial.flush();
#endif
#ifdef USE_DISPLAY
        // Following mesage shown only if failure was unrelated to I2C.
        display.setCursor(COL_0, FRMCNTRS_ROW);
        display.print(F("HW init failed"));
#endif
        abort();
    }

    initLmic();

    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀

    // Place code for initializing sensors etc. here.

    // resetCounter();
    // Set Extra LEDs too OFF
    pinMode(15, OUTPUT); //Green (led2)
    pinMode(22, OUTPUT); //Blue (led3)
    digitalWrite(15, LOW);
    digitalWrite(22, LOW); 
    
    gps1.init();

    led2.flash(3, 50, 50, 50, 50); //Flash Green Led 3 times rapidly to indicate successful setup

    
    //  █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
    //  █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
    //  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀

    //if (activationMode == ActivationMode::OTAA)
    //{
        //LMIC_startJoining();
    //}

    // Schedule initial doWork job for immediate execution.
    os_setCallback(&doWorkJob, doWorkCallback);
    os_setCallback(&doWorkJobLong, doWorkCallbackLong);
    
}

void loop()
{
    os_runloop_once();
}
