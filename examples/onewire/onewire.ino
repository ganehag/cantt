#include <SPI.h>
#include <mcp_can.h>
#include <isotp.h>
#include <OneWire.h>

/* OneWire Scratchpad layout */
#define DS1920_SP_TEMP_LSB              0
#define DS1920_SP_TEMP_MSB              1
#define DS1920_SP_TH                    2
#define DS1920_SP_TL                    3
#define DS18B20_SP_CONFIG               4
#define DS1920_SP_COUNT_REMAIN          6
#define DS1920_SP_COUNT_PERC            7
#define DS1920_SP_CRC                   8

#define MAX_ONEWIRE_SENSORS 32

/* CANTT settings */
#define CAN0_INT 2
#define CAN0_CS 10
#define DEVICE_ID 0x201

uint8_t canAvailable();
uint8_t canRead(CANMessage &msg);
uint8_t canSend(const CANMessage &msg);
void callback(uint32_t, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len);

MCP_CAN CAN0(CAN0_CS);
CANTransport CANTR(canAvailable, canRead, canSend);
CANTT cantt(DEVICE_ID, CANTR, callback);

OneWire ds(3); // PIN 3
uint8_t oneWireAddr[MAX_ONEWIRE_SENSORS][8]; // OW registry

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
  Search for OneWire devices
******************************************************************************/
uint8_t scanOwDevices() {
  static const unsigned long REFRESH_INTERVAL = 15000; // ms
  static unsigned long lastRefreshTime = 0;
  uint8_t address[8];
  uint8_t count = 0;

  if(millis() - lastRefreshTime < 0) {
    // This will always occur after ~50days
    lastRefreshTime = 0;
  }

  if(millis() - lastRefreshTime < REFRESH_INTERVAL) {
    return 0;
  }

  lastRefreshTime += REFRESH_INTERVAL;

  ds.reset();

  if (ds.search(address)) {
    memset(oneWireAddr, 0, sizeof(oneWireAddr));
    
    do {
      if (ds.crc8(address, 7) == address[7]) {
        memcpy(&oneWireAddr[count][0], address, sizeof(address));
        count++;
      }
    } while (count < MAX_ONEWIRE_SENSORS && ds.search(address));
  }

  return count;
}


/******************************************************************************
  Query each OneWire sensor
******************************************************************************/
void oneWireQuery() {
  static const unsigned long REFRESH_INTERVAL = 5000; // ms
  static unsigned long lastRefreshTime = 0;
  static bool waitForConversion = false;

  char topic_buffer[CANTT_MAX_TOPIC_LEN];
  char payload_buffer[CANTT_MAX_PAYLOAD_LEN];

  double f = 0;

  if(millis() - lastRefreshTime < 0) {
    // This will always occur after ~50days
    lastRefreshTime = 0;
  }

  if(millis() - lastRefreshTime >= REFRESH_INTERVAL) {
    lastRefreshTime += REFRESH_INTERVAL;
    
    ds.reset();         // reset bus
    ds.skip();          // broadcast to all sensors
    ds.write(0x44, 0);  // start conversion
    waitForConversion = true;

    return;
  }

  // Will only trigger once the OW bus is ready
  // when the last sensor is done with its conversion
  if(waitForConversion && ds.read() != 0) {
    waitForConversion = false;
    
    for(uint8_t i = 0; i < MAX_ONEWIRE_SENSORS; i++) {
      if(oneWireAddr[i][0] == 0) {
        break;
      }
      
      ds.reset();                // reset bus
      ds.select(oneWireAddr[i]); // select sensor
      ds.write(0xBE);            // read scratchpad

      uint8_t owData[9];
      for (uint8_t j = 0; j < sizeof(owData); j++) {  // we need 9 bytes
        owData[j] = ds.read();
      }

      f = getTemp(oneWireAddr[i][0], owData);
    
      // Topic
      sprintf(topic_buffer, "ow/%02x%02x%02x%02x%02x%02x%02x%02x", 
              oneWireAddr[i][0], oneWireAddr[i][1], oneWireAddr[i][2], oneWireAddr[i][3],
              oneWireAddr[i][4], oneWireAddr[i][5], oneWireAddr[i][6], oneWireAddr[i][7]);

      // Payload
      // "%d.%02d" (int)f, (int)(f*100)%100 is a way to get float support in sprintf
      sprintf(payload_buffer, "%d.%02d", (int)f, (int)(f*100)%100);
      
      cantt.publish(topic_buffer, payload_buffer);
    }
  }
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
  scanOwDevices(); // Keeps track internaly so it only runs every 15000ms

  oneWireQuery();  // Keeps track internaly so it only runs every 5000ms

  cantt.loop(); // Run the state machine
}

/******************************************************************************
  END FILE
******************************************************************************/
