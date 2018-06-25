#include <SPI.h>
#include <mcp_can.h>
#include <isotp.h>

#define CAN0_INT 2
#define CAN0_CS 10
#define DEVICE_ID 0x101

uint8_t canAvailable();
uint8_t canRead(CANMessage &msg);
uint8_t canSend(const CANMessage &msg);
void callback(uint32_t, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len);

MCP_CAN CAN0(CAN0_CS);
CANTransport CANTR(canAvailable, canRead, canSend);
CANTT cantt(DEVICE_ID, CANTR, callback);

/******************************************************************************
  MCP2515 signaling new data
******************************************************************************/
uint8_t canAvailable() {
  return !digitalRead(CAN0_INT);
}

/******************************************************************************
  Read frame from MCP2515 into CANTT RX buffer
******************************************************************************/
uint8_t canRead(CANMessage &msg) {
  uint8_t ret = CAN0.readMsgBuf(&msg.id, &msg.len, msg.data);
  
  if(msg.id & 0x80000000) {
    msg.extended = true;
    msg.id &= 0x7fffffff;
  }

  if(msg.id & 0x40000000) {
    msg.rtr = true;
    msg.id &= 0x3fffffff;
  }

  return ret;
}

/******************************************************************************
  Send frame to MCP2515 from CANTT TX buffer
******************************************************************************/
uint8_t canSend(const CANMessage &msg) {
  uint32_t id = msg.id;

  if(msg.extended) {
    id |= 0x80000000; // CAN_MCP_lib mask for ext bit
  }

  if(msg.rtr) {
    id |= 0x40000000; // CAN_MCP_lib mask for rtr bit
  }

  return CAN0.sendMsgBuf(id, msg.len, (uint8_t *)msg.data);
}


/******************************************************************************
  Packet received callback
******************************************************************************/
void callback(uint32_t addr, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len) {
  // topic (not null terminated)
  // payload (not null terminated)
}


/******************************************************************************
  Setup
******************************************************************************/
void setup() {
  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
    pinMode(CAN0_INT, INPUT);
  }

  cantt.begin();
}


/******************************************************************************
  Main loop
******************************************************************************/
void loop() {
  cantt.loop(); // Run the state machine
  // cantt.publish("topic", "some message");
}

/******************************************************************************
  END FILE
******************************************************************************/
