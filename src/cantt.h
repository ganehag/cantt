#ifndef __CANTT_H__

/*
 * CANTT library by Mikael Ganehag Brorsson
 */


#ifndef CANTT_MAX_RECV_BUFFER
#define CANTT_MAX_RECV_BUFFER 64
#endif

#define CANTT_MAX_DATASIZE 4095
#define CANTT_CAN_DATASIZE 8

#define CANTT_MAX_ADDR 0x7FF

#define CANTT_SINGLE_SIZE_MASK 0x0F  // 00001111
#define CANTT_FIRST_SIZE_MASK_BYTE0 0x0F  // 00001111
#define CANTT_FIRST_SIZE_MASK_BYTE1 0xFF  // 11111111
#define CANTT_CONSECUTIVE_INDEX_MASK 0x0F  // 00001111

#define CANTT_FLOW_CLEAR 0
#define CANTT_FLOW_WAIT 1
#define CANTT_FLOW_ABORT 2

#define CANTT_DEFAULT_WAIT_TIME 20
#define CANTT_DEFAULT_HOLDOFF_DELAY 100
#define CANTT_STATE_TIMEOUT 200 


#define CANTT_SINGLE_FRAME (0)
#define CANTT_FIRST_FRAME  (1)
#define CANTT_CONSECUTIVE_FRAME (2)
#define CANTT_FLOWCTRL_FRAME  (3)


#define FRAME_TYPE(x) (x >> 4)


#if(PLATFORM_ID == 0)

struct CANMessage {
   uint32_t id;
   bool     extended; // Not used currently
   bool     rtr;  // not used currently
   uint8_t  len;
   uint8_t  data[8];
};

#endif

struct CANTTbuf {
  uint32_t address;
  struct CANMessage can;
  uint16_t size;
  uint16_t message_pos;
  uint8_t message[CANTT_MAX_RECV_BUFFER];
  uint16_t frameCounter;
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

class CANTT {
public:
  CANTT(uint32_t canAddr,
        uint8_t (*canAvailable)(),
        uint8_t (*canRead)(CANMessage &msg),
        uint8_t (*canSend)(const CANMessage &msg), 
        void (*callback)(long unsigned int, uint8_t*, unsigned int));

  CANTT(uint32_t canAddr, uint32_t timeout,
        uint8_t (*canAvailable)(),
        uint8_t (*canRead)(CANMessage &msg),
        uint8_t (*canSend)(const CANMessage &msg), 
        void (*callback)(long unsigned int, uint8_t*, unsigned int));

  void begin();
  void loop();
  int send(uint8_t *payload, uint16_t length);
  int send(uint32_t addr, uint8_t* payload, uint16_t length);


private:
  enum state_m stateMachine;

  void initialize(uint32_t canAddr, uint32_t timeout,
                 uint8_t (*canAvailable)(),
                 uint8_t (*canRead)(CANMessage &msg),
                 uint8_t (*canSend)(const CANMessage &msg), 
                 void (*callback)(long unsigned int, uint8_t*, unsigned int));

  void parseSingle();
  void parseFirst();
  int  parseConsecutive();

  int sendSingle();
  int sendFirst();
  int sendConsecutive();

  int recvMessage();
  int sendMessage();

  bool hasOutgoingMessage();
  void clearTX();

  void changeState(enum state_m s);

  
  uint32_t canAddr;

  struct CANTTbuf rx;
  struct CANTTbuf tx;

/*
  void parseFlow();
  int sendFlowFrame(long unsigned int arbId, uint8_t fc_flag, uint8_t block_size, uint8_t separation_time);
  
  uint8_t block_size;
  int16_t flowExpected;
*/

  uint8_t wait_time;
  uint32_t timeOutTimer;
  uint32_t timeout;


  uint8_t (*canAvailable)();
  uint8_t (*canRead)(CANMessage &msg);
  uint8_t (*canSend)(const CANMessage &msg);
  void (*callback)(long unsigned int, uint8_t*, unsigned int);

};

#endif // cantt.h
