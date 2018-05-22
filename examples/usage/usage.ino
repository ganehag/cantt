#include <SPI.h>
#include <mcp_can.h>
#include <isotp.h>

#define CAN0_INT 2

void callback(long unsigned int, byte* payload, unsigned int length);

MCP_CAN CAN0(10); // 10 -> CS

IsoTp isotp(&CAN0, CAN0_INT, callback);


/******************************************************************************
  Packet received callback
******************************************************************************/
void callback(long unsigned int id, byte* payload, unsigned int length) {
    /* Do stuff with the payload data */
}


/******************************************************************************
  Setup
******************************************************************************/
void setup() {
    // Setup CANbus
    CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)
    CAN0.setMode(MCP_NORMAL);

    isotp.begin();
}


/******************************************************************************
  Main loop
******************************************************************************/
void loop() {
    isotp.loop(); // Run the state machine

    /* // Sending data

    uint8_t data[11] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A}
    isotp.send(0x100, data, sizeof(data));

    */
}

/******************************************************************************
  END FILE
******************************************************************************/
