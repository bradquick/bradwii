//
//  frskytelemetry.h
//  Project
//
//  Created by Brad on 1/11/15.
//
//

#pragma once

void inittelemetry();
void checktelemetryforaction();


  // Frame protocol
  #define Protocol_Header    0x5E
  #define Protocol_Tail      0x5E

  // Data Ids  (bp = before point; af = after point)
  // Official data IDs
  #define ID_GPS_Altitude_bp    0x01
  #define ID_GPS_Altitude_ap    0x09
  #define ID_Temperature1       0x02
  #define ID_RPM                0x03
  #define ID_Fuel_level         0x04
  #define ID_Temperature2       0x05
  #define ID_Volt               0x06
  #define ID_Altitude_bp        0x10
  #define ID_Altitude_ap        0x21
  #define ID_GPS_speed_bp       0x11
  #define ID_GPS_speed_ap       0x19
  #define ID_Longitude_bp       0x12
  #define ID_Longitude_ap       0x1A
  #define ID_E_W                0x22
  #define ID_Latitude_bp        0x13
  #define ID_Latitude_ap        0x1B
  #define ID_N_S                0x23
  #define ID_Course_bp          0x14
  #define ID_Course_ap          0x1C
  #define ID_Date_Month         0x15
  #define ID_Year               0x16
  #define ID_Hour_Minute        0x17
  #define ID_Second             0x18
  #define ID_Acc_X              0x24
  #define ID_Acc_Y              0x25
  #define ID_Acc_Z              0x26
  #define ID_Voltage_Amp_bp     0x3A
  #define ID_Voltage_Amp_ap     0x3B
  #define ID_Current            0x28
  #define ID_Vario              0x30
  // User defined data IDs
  #define ID_Gyro_X             0x40
  #define ID_Gyro_Y             0x41
  #define ID_Gyro_Z             0x42
  //Multiwii EZ-GUI
  #define ID_Ang_X             0x50
  #define ID_Ang_Y             0x51
  #define ID_State             0x52 


// frysky smart port definitions: mostly copied from CleanFlight
enum
{
    FSSP_START_STOP = 0x7E,
    FSSP_DATA_FRAME = 0x10,

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67,
    // there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
};

// these data identifiers are obtained from http://diydrones.com/forum/topics/amp-to-frsky-x8r-sport-converter
enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_RPM        = 0x050F ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_CAP_USED   = 0x0600 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_GPS_SPEED  = 0x083f,
};

// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f
#define A3_FIRST_ID             0x0900
#define A3_LAST_ID              0x090f
#define A4_FIRST_ID             0x0910
#define A4_LAST_ID              0x091f
#define AIR_SPEED_FIRST_ID      0x0a00
#define AIR_SPEED_LAST_ID       0x0a0f
#define RSSI_ID                 0xf101
#define ADC1_ID                 0xf102
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105
#define XJT_VERSION_ID          0xf106

