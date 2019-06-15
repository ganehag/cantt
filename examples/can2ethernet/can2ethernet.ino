#include <SPI.h>
#include <mcp_can.h>

#include <cantt.h>

#include <EthernetUdp.h>
#include <Ethernet.h>

// Ethernet stuff
byte mac[] = {0x00, 0x55, 0x66, 0xEE, 0xFF, 0xFF};            // Device MAC 
IPAddress ip(10, 0, 0, 254);                                  // Device IP
IPAddress gateway(10, 0, 0, 1);                               // Gateway IP
IPAddress broadcast(10, 0, 0, 255);                           // Broadcast Addr
EthernetUDP UDP;

// CANTT stuff
#define DEVICE_ID 0x150
#define CAN0_INT 2                                            // Set INT to pin 2

void callback(uint32_t, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len);
uint8_t canAvailable();
uint8_t canRead(CANMessage &msg);
uint8_t canSend(const CANMessage &msg);

MCP_CAN CAN0(10);                                             // Set CS to pin 10
CANTransport CANTR(canAvailable, canRead, canSend);
CANTT cantt(DEVICE_ID, CANTR, callback);

void callback(uint32_t addr, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len) {
  if(!payload_len) {
    return;
  }
  
  if(payload[0] == 0x03) {   
    UDP.beginPacket(broadcast, 8889);
    UDP.write(payload, payload_len);
    UDP.endPacket();
  }

  // Serial debug
  Serial.print("ID: ");
  Serial.print(addr, HEX);
  Serial.print("    ");

  // Topic
  for(int i=0; i < topic_len; i++) {
    //Serial.print("0x");
    Serial.print((char)topic[i]);
    //Serial.print(" ");
  }
  
  Serial.print("  ");

  // Message
  for(int i=0; i < payload_len; i++) {
    // Serial.print("0x");
    Serial.print((char)payload[i]);
    // Serial.print(" ");
  }
  
  Serial.print("\n");
}

/*
 * Return 1 if there is data to read
 */
uint8_t canAvailable() {
  return !digitalRead(CAN0_INT);
}

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

void setup() {
  Serial.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) == CAN_OK) {
    CAN0.setMode(MCP_NORMAL);
  }
  Ethernet.begin(mac, ip);
  UDP.begin(8888); // Initialize the UDP listen port that is currently unused!

  // Blinky LED
  pinMode(8, OUTPUT);

  // ISO-TP
  cantt.begin();
}

void loop() {
  static int led_state = -10000;
  led_state++;
  if(led_state > 10000) {
    led_state = -10000;
  }
  
  digitalWrite(8, led_state > 0);
  
  cantt.loop();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
