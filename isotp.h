#ifndef __ISOTP_H__

/* isotp library by Mikael Ganehag Brorsson
 */

// This will load the definition for common Particle variable types
// #include "Particle.h"

/* ISO-TP Header (iso-15765-2) CANBus frame format

In ISO-TP (iso-15765-2) there are 4 different types of frames: Single, First, Consecutive and Flow.
For each frame the first 4 bits (7 .. 4) of the first byte (0) represents the frame type. Of which
there are only four and the remaining 12 possibilites are reserved. The second part of the first
byte (bits 3 .. 0) has a diffrent meaning depinding on the frame type.


| Bit offset  | 7 .. 4 (byte 0) | 3 .. 0 (byte 0) | 15 .. 8 (byte 1) | 23..16 (byte 2) |  ....  |
|-------------|-----------------|-----------------|------------------|-----------------|--------|
| Single      |               0 | size (0..7)     | Data A           | Data B          | Data C |
| First       |               1 | size (8..4095)                     | Data A          | Data B |
| Consecutive |               2 | index (0..15)   | Data A           | Data B          | Data C |
| Flow        |               3 | FC flag (0,1,2) | Block size       | Separation Time |        |


1) Single Frame

A ISO-TP frame with data size <= 7. This will fit into a standard CANBus frame (MAX_DATA_SIZE = 8).
Bits 3 .. 0 of the first byte represents the data size.


2) First Frame (in a multipart transmission)

The start of a ISO-TP frame with 8 <= data size <= 4095. Tranmitting this data will require several
CANBus frames. Bits 3 .. 0  of the first byte along with all eight bits of the second byte (total
12 bits), represents a data size between 8 and 4095. While the 12 bits can prepresent a data size of
0 to 4095. Only a size larger or equal to 8 uses this format, of the datasize is <= 7 a single frame
should be used.

**Although, any implementation should probably still be able to handle a situation where a "First"
frame has been sent with < 8 bytes. As this sort of will allow a "automatic" ACK of messages using
the "Flow" frames.**


3) Flow

Even though Flow is the forth kind of frame. Explaining it before the "Consecutive" frame makes more
sense, as it is a frame sent as a reply to a "First" frame. These are used to wait for acknowledgement
before "spamming" the CANBus with a "lot" of messages. A transmission of 4095 bytes will result in
a total of 586 CANBus frames and will tie of the bus for a short while.

Bits 3 .. 0 of byte 0 in a "Flow" frame represents Flow Control flags:
0 = Clear To Send, 1 = Wait, 2 = Overflow/abort

Byte 1 represents the number of frames to send before expecting a new Flow frame. A value of zero
means that no such frames are necessary.

The last byte is the minimum delay between frames. It is used as a way for the receiver to guide 
the sender not to overburden it. A value between 0 .. 127 (0x7F), represents a minimum delay in 
milliseconds. While values in the range 241 (0xF1) to 249 (0xF9) specify delays increasing from 
100 to 900 microseconds.


4) Consecutive Frame (in a multipart transmission)

Bits 3 .. 0 in a "Consecutive" frame represents a counter of sorts. Which goes from 0 .. 15, then wrapps
back to 0 and so on. Is is used as a very basic way to ensure that each frame is received in order.

*/

#include <mcp_can.h>
#include <mcp_can_dfs.h>

// #define ISOTP_SINGLE_FRAME 0
// #define ISOTP_FIRST_FRAME 1
// #define ISOTP_CONSECUTIVE_FRAME 2
// #define ISOTP_FLOWCTRL_FRAME 3

#define ISOTP_EXTENDED_MASK 0x80000000
#define ISOTP_EXTENDED_INVMASK 0x7fffffff
#define ISOTP_RTR_MASK 0x40000000
#define ISOTP_RTR_INVMASK 0xbfffffff
#define ISOTP_ADDR_UNMASK 0x3fffffff

#define IS_EXTENDED(rxId) ((rxId & 0x80000000) == 0x80000000)
#define IS_RTR(rxId) ((rxId & 0x40000000) == 0x40000000)

#define ISOTP_MAX_DATASIZE 4095

#define ISOTP_MAX_ADDR 0x7FF

#ifndef ISOTP_MAX_RECV_BUFFER
#define ISOTP_MAX_RECV_BUFFER 64
#endif

#define ISOTP_SINGLE_SIZE_MASK 0x0F  // 00001111
#define ISOTP_FIRST_SIZE_MASK_BYTE0 0x0F  // 00001111
#define ISOTP_FIRST_SIZE_MASK_BYTE1 0xFF  // 11111111
#define ISOTP_CONSECUTIVE_INDEX_MASK 0x0F  // 00001111

#define ISOTP_FLOW_CLEAR 0
#define ISOTP_FLOW_WAIT 1
#define ISOTP_FLOW_ABORT 2

#define ISOTP_DEFAULT_WAIT_TIME 20

// Timeout must be more than consecutive frame delay (max 127)
#define ISOTP_MACHINE_TIMEOUT 200 

#define ISOTP_INVALID_ADDR 0xFFFF

#define CAN_MAX_DATA_SIZE 8


#define FRAME_TYPE(x) (x >> 4)

/*

ẗypedef struct s_ISOTPMessage {
  uint32_t address; // Address / Priority (CAN bus specific)
  uint16_t length; // Expected length of message
  // uint8_t delay; // Delay between frames
  // uint8_t ffinterval; // Flow frame interval
  
  uint16_t buflen; // Length of the received data
  uint8_t buf[ISOTP_MAX_RECV_BUFFER]; // Buffer for the received data

  uint32_t timeout_timer; // Keep track of last frame timestamp
} ISOTPMessage;
*/

struct CANMessage {
   uint32_t id;
   bool     extended; // Not used currently
   bool     rtr;  // not used currently
   uint8_t  len;
   uint8_t  data[8];
};

struct ISOTPTransmission {
  uint32_t address;
  struct CANMessage can;
  uint16_t size;
  uint16_t message_pos;
  uint8_t message[ISOTP_MAX_RECV_BUFFER];
  uint16_t frameCounter;
};

enum isotpframe {
  ISOTP_SINGLE_FRAME = 0,
  ISOTP_FIRST_FRAME = 1,
  ISOTP_CONSECUTIVE_FRAME = 2,
  ISOTP_FLOWCTRL_FRAME = 3
};

enum state_m {
  DISABLED = -1,
  IDLE = 0,
  CHECKREAD = 1,
  READ = 2,
  PARSE_WHICH = 3,
  SEND_FLOW = 4,
  SEND_SINGLE = 5,
  SEND_FIRST = 6,
  SEND_CONSECUTIVE = 7,
  RECV_FLOW = 8,
  CHECK_COLLISION = 9  
};

class IsoTp {
public:
  /**
   * Constructor
   */
  IsoTp(MCP_CAN* bus, uint8_t mcp_int, uint32_t canAddr, void (*callback)(long unsigned int, uint8_t*, unsigned int));
  IsoTp(MCP_CAN* bus, uint8_t mcp_int, uint32_t canAddr, uint32_t timeout, void (*callback)(long unsigned int, uint8_t*, unsigned int));

  void begin();
  void loop();
  int send(uint8_t *payload, uint16_t length);
  int send(uint32_t addr, uint8_t* payload, uint16_t length);

private:
  enum state_m stateMachine;

  void initialize(MCP_CAN* bus, uint8_t mcp_int,  uint32_t canAddr, uint32_t timeout, void (*callback)(long unsigned int, uint8_t*, unsigned int));

  void parseSingle();
  void parseFirst();
  int parseConsecutive();
  void parseFlow();

  int sendSingle();
  int sendFirst();
  int sendConsecutive();

  int recvMessage();
  int sendMessage();

  bool hasOutgoingMessage();
  void clearTX();

  void changeState(enum state_m s);

  int sendFlowFrame(long unsigned int arbId, uint8_t fc_flag, uint8_t block_size, uint8_t separation_time);

  // MCP_CAN_lib stuff
  MCP_CAN* bus;
  uint8_t mcp_int;
  uint32_t canAddr;

  struct ISOTPTransmission rx;
  struct ISOTPTransmission tx;

  /*

  // Can Frame Stuff
  long unsigned int canTxId;
  long unsigned int canRxId;

  uint32_t frameRxId; // address for consecutive frames
  uint8_t canLen;
  uint8_t canBuf[8];

  // ISO-TP Stuff
  uint8_t mfBuf[ISOTP_MAX_RECV_BUFFER];
  uint16_t mfBufSize;
  uint16_t mfDataSize;

  */

  uint8_t wait_time;
  uint8_t block_size;

  int16_t flowExpected;
  uint16_t numFramesSent;

  uint32_t timeOutTimer;
  uint32_t timeout;

  void (*callback)(long unsigned int, uint8_t*, unsigned int);
};

#endif // isotp.h
