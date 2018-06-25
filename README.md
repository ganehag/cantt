# CANTT, a Telemetry Transport for CAN bus

**While based upon ISO-TP (ISO-15765-2), this library does not retain
compatibility with ISO-TP. Read more about the explanation in the section
titled "Compatibility issues with ISO-TP".**

A topic and message protocol/library to send MQTT like messages over CAN bus.
This library is not dependant on the specific CAN bus implementation and has
been tested both with MCP_CAN_lib as well as the CAN implementation in the 
Particle Photon/Electron.

## Usage

See the [usage.ino](examples/usage/usage.ino) file in the
[examples/usage](examples/usage) directory.


## Compatibility issues with ISO-TP (ISO-15765-2)

While trying to build a library that was compatible with ISO-TP, significant 
issues arose from the *Flow Control* frames. These frames are supposed to 
follow a *First* frame as an acknowledge/acceptance from the other side. 
However, this library is supposed to broadcast messages from and to every 
node on the bus. Having each node sending a *Flow Control* frame whenever 
they receive a *First* frame results in lots of strange behaviour, and too 
much unnecessary overhead on the bus.

It was thus decided to skip the *Flow Control* frames and in favour of 
sending the *First* frame and directly after that all of the *Consecutive*
frames.

## License

[BSD 3-Clause License](LICENSE)

## Simple Example

```cpp
CANTransport CANTR0(canAvailable, canRead, canSend);
CANChannel CAN0(CAN_D1_D2); // Particle Photon/Electron
// MCP_CAN CAN0(10); // MCP_CAN LIbrary (Arduino)

uint8_t canAvailable() {
  return CAN0.available();
}
uint8_t canRead(CANMessage &msg) {
  CAN0.receive(msg);
  return 0;
}
uint8_t canSend(const CANMessage &msg) {
  CAN0.transmit(msg);
  return 0;
}
void callback(uint32_t addr, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len) {
  // Handle topic and payload
}

CANTT cantt(DEVICE_ID, CANTR0, callback);

void setup() {
  cantt.begin();
}

void loop {
  cantt.loop();
  // cantt.publish("some/topic", "some kind of data");
}
```
