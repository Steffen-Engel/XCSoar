#pragma once

#include "stdint.h"
#include "CmdBuffer.h"
#include "Device/Port/Port.hpp"
#include "util/tstring.hpp"
#include "io/FileOutputStream.hxx"
#include "io/BufferedOutputStream.hxx"
#include "time/Stamp.hpp"

#include "yAllProtocol.h"

typedef struct
{
  int32_t max_q;  // in Pa
  int16_t max_g;  // in centi g
  int16_t min_g;  // in centi g
}tMaxValues;

extern tMaxValues MaxValues;


class cyAll
{
private:
	Port *port;
	cCmdBuffer CmdBuffer;
//	tstring path;   // Logger file
	bool flying;
	FileOutputStream *file;
	BufferedOutputStream *writer;
	TimeStamp last_time;

	uint8_t checksum;
	int dataSize;
	enum {
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
		HEADER_ERR
	} c_state;
	bool err_rcvd;


	void sendHeader();
	void sendByte(char value);
	void sendWord(uint16_t value);
	void sendLong(uint32_t value);
	void sendEnd();

	void StartLog(void);
	void EndLog(void);

	void evaluateCommand(uint8_t cmd, int dataSize, struct NMEAInfo &info);

public:

	tMSP MSPData;
	tIdent Ident;

	bool FlightState;  // copy from calculated data

	cyAll();
	~cyAll(){};

	void Open(Port &_port);

	void sendRequest(uint8_t value);

	void sendRequest(uint8_t value, uint8_t parm);
	void sendRequest(uint8_t value, uint16_t parm);
	void sendRequest(uint8_t value, uint32_t parm);
	

	void parse(uint8_t c, struct NMEAInfo &info);

};


extern tLoggerSetData LoggerData;
