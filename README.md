# ISO-TP (ISO-15765-2) library for Arduino and friends

A simple library with a state machine to handle sending and receiving of ISO-TP
frames over CANbus. This library requires MCP_CAN_lib as interface to the
MCP2515 hardware.

## Usage

See the [usage.ino](examples/usage/usage.ino) file in the 
[examples/usage](examples/usage) directory.


## License

[BSD 3-Clause License](LICENSE)

## Simple Example

```cpp
void callback(long unsigned int id, byte* payload, unsigned int length) {
  // Received frames arrives here
}

MCP_CAN CAN0(10); // Set CS to pin 10
IsoTp isotp(&CAN0, 2, callback); // Set INT to pin 2

void setup() {
  /*
    MCP_CAN_lib setup goes here
  */

  isotp.begin();
}

void loop {
  isotp.loop();
}

```
