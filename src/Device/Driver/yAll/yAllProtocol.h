#pragma once



#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4


// Logger-Message
#define MSP_LOGGER_SET           230   //in message          sets a logger time to send data in msecs (uint32_t)
                                       //out message				 sends a dat set of logger values
#define MSP_RESET                231   //in message          reset logger
#define MSP_IAS_CALIBRATION      232   //in message          no param

// Data for MSP_IDENT
typedef struct
{
  uint8_t version;
  uint8_t multiType;
  uint8_t MSPVersion;
  uint8_t capabilities;
} tIdent;


// Data for MSP_STATUS
typedef struct
{
  uint16_t cycleTime;
  uint16_t i2cError;
  uint16_t present;
  uint32_t mode;
} tStatus;


#define ETA_QR    0
#define ETA_HR    1
#define ETA_SR    2
#define ETA_FLAP  3


// Answerdata for MSP_LOGGER_SET
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  /*00*/	uint32_t 	millis;
  /*04*/	uint16_t 	cycle_time;
  /*06*/	int16_t 	accel[3];			// x, y, z (1/100g)
  /*12*/	int16_t 	angle[3];			// x-axis(bank), y-axis(pitch), z-axis(course) (1/10 degree)
  /*18*/	int16_t 	rot[3];				// x-axis(roll), y-axis(pitch), z-axis(yaw)
  /*24*/	int32_t 	pressure[3];	// cabin, static, differential in Pa
  /*36*/	int16_t 	eta[4];				// aileron, elevator, rudder, flap
  /*44*/	uint8_t 	GPS_FIX;
  /*45*/	uint8_t		GPS_numSat;
  /*46*/	int32_t  	GPS_latitude;
  /*50*/	int32_t  	GPS_longitude;
  /*54*/	int16_t		GPS_altitude;
  /*56*/	int16_t		GPS_speed;
  /*58*/  int16_t   GPS_heading;
  /*60*/  uint16_t  GPS_week;
  /*62*/  uint32_t  GPS_time;
  /*66*/
} tLoggerSetData ;
#pragma pack(pop)


typedef char assertion_on_mystruct[(   sizeof(tLoggerSetData)==66   )*2-1 ];


// Data for all messages
typedef struct
{
  tIdent					Ident;
  tStatus					Status;
  tLoggerSetData	LoggerSet;
} tMSP;
//tMSP MSPData;


