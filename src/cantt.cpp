/**
    CANTT Library
    cantt.cpp
    Purpose: Sends and receives MQTT like messages over CAN bus.

    @author Mikael Ganehag Brorsson
    @version 0.5 2018-06-18
*/

/**

Example:

CANChannel CAN0(CAN_D1_D2);
CANTT CAN0(0x100, canAvailable, canRead, canSend, callback);

void callback(long unsigned int id, byte* payload, unsigned int length) {
    // handle payload
}

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

void setup() {
    cantt.begin();
}

void loop() {
    if(now_and_then) {
        cantt.publish("topic", "message");
    }

    cantt.loop();
}

*/


#include "Arduino.h"
#include "cantt.h"

/**
    Constructor for the class object.

    @param canAddr CAN bus address/priority
    @param canAvailable pointer to function that indicates the availibility of a new message on the CAN bus
    @param canRead pointer to function that reads the message from CAN bus into CANTT
    @param canSend pointer to function that sends a message from CANTT out over CAN bus
    @param callback pointer to callback function once a complete message has been received
*/
CANTT::CANTT(uint32_t canAddr,
            uint8_t (*canAvailable)(),
            uint8_t (*canRead)(CANMessage &msg),
            uint8_t (*canSend)(const CANMessage &msg), 
            void (*callback)(long unsigned int, uint8_t*, unsigned int)) {
    this->initialize(canAddr, CANTT_STATE_TIMEOUT, canAvailable, canRead, canSend, callback);
}

/**
    Constructor for the class object.

    @param canAddr CAN bus address/priority
    @param timeout value to configure internal timeouts
    @param canAvailable pointer to function that indicates the availibility of a new message on the CAN bus
    @param canRead pointer to function that reads the message from CAN bus into CANTT
    @param canSend pointer to function that sends a message from CANTT out over CAN bus
    @param callback pointer to callback function once a complete message has been received
*/
CANTT::CANTT(uint32_t canAddr, uint32_t timeout,
            uint8_t (*canAvailable)(),
            uint8_t (*canRead)(CANMessage &msg),
            uint8_t (*canSend)(const CANMessage &msg), 
            void (*callback)(long unsigned int, uint8_t*, unsigned int)) {
    this->initialize(canAddr, timeout, canAvailable, canRead, canSend, callback);
}

/**
    Initialises the class object.

    @param canAddr CAN bus address/priority
    @param timeout value to configure internal timeouts
    @param canAvailable pointer to function that indicates the availibility of a new message on the CAN bus
    @param canRead pointer to function that reads the message from CAN bus into CANTT
    @param canSend pointer to function that sends a message from CANTT out over CAN bus
    @param callback pointer to callback function once a complete message has been received
*/
void CANTT::initialize(uint32_t canAddr, uint32_t timeout,
                      uint8_t (*canAvailable)(),
                      uint8_t (*canRead)(CANMessage &msg),
                      uint8_t (*canSend)(const CANMessage &msg), 
                      void (*callback)(long unsigned int, uint8_t*, unsigned int)) {

    // Function pointers
    this->callback = callback;
    this->canAvailable = canAvailable;
    this->canRead = canRead;
    this->canSend = canSend;

    // Device CAN address/priority
    this->canAddr = canAddr;

    // TX Buffer
    this->tx.address = 0;
    this->tx.can.id = 0;
    this->tx.can.extended = false;
    this->tx.can.rtr = false;
    memset(this->tx.can.data, 0, CANTT_CAN_DATASIZE);
    this->tx.size = 0;
    this->tx.message_pos = 0;
    memset(this->tx.message, 0, CANTT_MAX_RECV_BUFFER);
    this->tx.frameCounter = 0;

    // RX Buffer
    this->rx.address = 0;
    this->rx.can.id = 0;
    this->rx.can.extended = false;
    this->rx.can.rtr = false;
    memset(this->rx.can.data, 0, CANTT_CAN_DATASIZE);
    this->rx.size = 0;
    this->rx.message_pos = 0;
    memset(this->rx.message, 0, CANTT_MAX_RECV_BUFFER);
    this->rx.frameCounter = 0;


    this->wait_time = CANTT_DEFAULT_WAIT_TIME;
    this->timeOutTimer = millis();
    this->timeout = timeout;

    this->stateMachine = DISABLED;

    /* Flow Control (not implemented)
    this->block_size = 0;
    this->flowExpected = -1; // Never
    */
}

/**
    Sets the state machine into IDLE state
*/
void CANTT::begin() {
    this->changeState(IDLE);
}

/**
    Empties the RX queue
*/
void CANTT::clearRX() {
    memset(this->rx.message, 0 , CANTT_MAX_RECV_BUFFER);
    this->rx.size = 0;
    this->rx.message_pos = 0;
    this->rx.address = 0;
    this->rx.frameCounter = 0;
}

/**
    Empties the TX queue
*/
void CANTT::clearTX() {
    memset(this->tx.message, 0, CANTT_MAX_RECV_BUFFER);
    this->tx.message_pos = 0;
    this->tx.size = 0;
    this->tx.address = 0;
    this->tx.frameCounter = 0;
};

/**
    Positions the TX buffer back to the beginning
*/
void CANTT::rewindTX() {
    this->tx.message_pos = 0;
    this->tx.frameCounter = 1;
};

/**
    Anything in the RX (receive) buffer
*/
bool CANTT::inReception() {
    return this->rx.size > 0;
}

/**
    Is the TX (transmission) buffer beeing sent
*/
bool CANTT::inTransmission() {
    return this->tx.message_pos > 0;
}

/**
    Anything in the TX (transmission) buffer
*/
bool CANTT::hasOutgoingMessage() {
    return this->tx.size > 0;
}

/**
    Switches the state machine to another state

    @param the new state for the machine
*/
void CANTT::changeState(enum state_m s) {
    this->stateMachine = s;

    if(this->stateMachine == IDLE) {
        this->timeOutTimer = 0;
    } else {
        this->timeOutTimer = millis();
    }
}

/**
    Waits in a loop for the state machine to return to IDLE state
    while still running the internal loop.

    @return error code
*/
int CANTT::waitUntilIdle() {
    uint32_t now = millis();

    // Busy block until any operation is done and the output buffer is available
    while(this->stateMachine != IDLE) {
        // Bail after TIMEOUT time
        if(now + CANTT_SEND_TIMEOUT < millis() ||
            millis() < now /* fail if millis wraps around to 0 */) {
            return -1;
        }

        this->loop();
    }

    return 0;
}

/**
    Parses a SINGLE_FRAME message and calls the callback function
*/
void CANTT::parseSingle() {
    uint8_t frameSize = this->rx.can.data[0] & CANTT_SINGLE_SIZE_MASK;

    // Ensure that the frame has the correct size
    if(frameSize == this->rx.can.len - 1 &&
       frameSize > 0 && frameSize < 8 &&
       this->callback != NULL) {
        // Use the data directly from the can buffer, no need to use the message buffer
        this->callback(this->rx.can.id, &this->rx.can.data[1], frameSize);

        this->clearRX();
    }
}

/**
    Parses the FIRST_FRAME in a long message
*/
void CANTT::parseFirst() {
    uint16_t frameSize = ((this->rx.can.data[0] & CANTT_SINGLE_SIZE_MASK) << 8) | this->rx.can.data[1];

    memset(this->rx.message, 0, CANTT_MAX_RECV_BUFFER);

    if(frameSize >= 8 && frameSize <= CANTT_MAX_DATASIZE) {
        this->rx.size = frameSize;

        memcpy(this->rx.message, &this->rx.can.data[2], 6); // All of the remaining data in this frame
        this->rx.message_pos = 6;
    }
}

/**
    Parses the CONSECUTIVE_FRAME(s) in a long message
    executes callback once the message is complete

    @return remaining length of message
*/
int CANTT::parseConsecutive() {
    uint16_t frameIndex = this->rx.can.data[0] & CANTT_CONSECUTIVE_INDEX_MASK;

    if(this->rx.size - this->rx.message_pos > 7) {
        // not the last frame
        memcpy(&this->rx.message[this->rx.message_pos], &this->rx.can.data[1], 7);
        this->rx.message_pos += 7;

    } else {
        // this (should be) the last frame
        memcpy(&this->rx.message[this->rx.message_pos], &this->rx.can.data[1], this->rx.size - this->rx.message_pos);
        this->rx.message_pos = this->rx.size;

        this->callback(this->rx.address, this->rx.message, this->rx.size);

        this->clearRX();
    }

    return this->rx.size - this->rx.message_pos;
}

/**
    Sends a SINGLE_FRAME message

    @return error code
*/
int CANTT::sendSingle() {
    memset(this->tx.can.data, 0, CANTT_CAN_DATASIZE);

    this->tx.can.data[0] = (CANTT_SINGLE_FRAME << 4) | this->tx.size;
    memcpy(&this->tx.can.data[1], this->tx.message, 7);
    this->tx.can.len = 1 + this->tx.size;
    this->tx.can.id = this->tx.address;

    return this->sendMessage();
}

/**
    Sends the FIRST_FRAME in a long message

    @return error code
*/
int CANTT::sendFirst() {
    memset(this->tx.can.data, 0, CANTT_CAN_DATASIZE);

    this->tx.can.data[0] = (CANTT_FIRST_FRAME << 4) | (this->tx.size >> 8);
    this->tx.can.data[1] = this->tx.size & CANTT_FIRST_SIZE_MASK_BYTE1;
    memcpy(&this->tx.can.data[2], this->tx.message, 6);
    this->tx.can.len = CANTT_CAN_DATASIZE;

    if(this->sendMessage() != 0) {
        this->changeState(IDLE);
        return 1;
    } else {
        this->tx.message_pos = 6;
        this->tx.frameCounter = 1;
    }

    return 0;
}

/**
    Sends the CONSECUTIVE_FRAME(s) in a long message

    @return the remaining amount of data to transmit
*/
int CANTT::sendConsecutive() {
    uint8_t maxSend = 7;

    memset(this->tx.can.data, 0, CANTT_CAN_DATASIZE);

    // Set frame type and counter
    this->tx.can.data[0] = (CANTT_CONSECUTIVE_FRAME << 4) | (this->tx.frameCounter % 0x0F);

    // Copy some or remaining data
    if(this->tx.size - this->tx.message_pos <= 7) {
        maxSend = this->tx.size - this->tx.message_pos;
    }
    memcpy(&this->tx.can.data[1], &this->tx.message[this->tx.message_pos], maxSend);

    // FIXME: change to properly handle RTX & Extended
    this->tx.can.len = 1 + maxSend; // HDR + data
    this->tx.can.id = this->tx.address;

    if(this->sendMessage() != 0) {
        this->changeState(IDLE);
    } else {
        this->tx.message_pos += maxSend;
        this->tx.frameCounter++;
    }

    if(this->tx.size <= this->tx.message_pos) {
        this->changeState(IDLE);
    }

    return this->tx.size - this->tx.message_pos;
}

/**
    Wrapper for the external function pointed to by canRead

    @return error code
*/
int CANTT::recvMessage() {
    this->rx.can.extended = false;
    this->rx.can.rtr = false;
    memset(this->rx.can.data, 0, CANTT_CAN_DATASIZE);

    if(this->canRead == NULL) {
        return 1;
    }

    if(this->canRead(this->rx.can) != 0) {
        return 1;
    }

    this->rx.address = this->rx.can.id;

    return 0;
}

/**
    Wrapper for the external function pointed to by canSend

    @return error code
*/
int CANTT::sendMessage() {
    if(this->canSend == NULL) {
        return 1;
    }

    this->tx.can.id = this->tx.address;

    if(this->canSend(this->tx.can) != 0) {
        return 1;
    }

    return 0;
}

/**
    Publish a message on a topic

    @param priority address/priority of the message
    @param topic the topic
    @param topic_len the length of the topic
    @param payload the payload
    @param payload_len the length of the payload
    @return error code
*/
int CANTT::publish(uint32_t priority, uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len) {
    uint8_t buffer[CANTT_MAX_MESSAGE_SIZE];

    // (HDR byte + 2 * uint16_t) + topic_len + payload_len
    if(topic_len + payload_len + 5 > CANTT_MAX_MESSAGE_SIZE) {
        return -1;
    }

    buffer[0] = CANTT_MSG_PUBLISH;
    buffer[1] = (topic_len & 0xFF);
    buffer[2] = (topic_len >> 8);
    memcpy(&buffer[3], topic, topic_len);
    buffer[3 + topic_len + 0] = (payload_len & 0xFF);
    buffer[3 + topic_len + 1] = (payload_len >> 8);
    memcpy(&buffer[3 + topic_len + 2], payload, payload_len);

    return this->send(priority, buffer, topic_len + payload_len + 5);
}

/**
    Publish a message on a topic

    @param priority address/priority of the message
    @param topic the topic as a null terminated string
    @param payload the payload as a null terminated string
    @return error code
*/
int CANTT::publish(uint32_t priority, char *topic, char *payload) {
    uint16_t topic_len = strlen(topic);
    uint16_t payload_len = strlen(payload);

    return this->publish(priority, (uint8_t *)topic, topic_len, (uint8_t *)payload, payload_len);
}

/**
    Publish a message on a topic

    @param topic the topic
    @param topic_len the length of the topic
    @param payload the payload
    @param payload_len the length of the payload
    @return error code
*/
int CANTT::publish(uint8_t *topic, uint16_t topic_len, uint8_t *payload, uint16_t payload_len) {
    return this->publish(this->canAddr, topic, topic_len, payload, payload_len);
}

/**
    Publish a message on a topic

    @param topic the topic as a null terminated string
    @param payload the payload as a null terminated string
    @return error code
*/
int CANTT::publish(char *topic, char *payload) {
    uint16_t topic_len = strlen(topic);
    uint16_t payload_len = strlen(payload);

    return publish((uint8_t *)topic, topic_len, (uint8_t *)payload, payload_len);
}

/**
    Sends any long message

    @param payload the data to be sent
    @param length length of the data
    @return error code
*/
int CANTT::send(uint8_t * payload, uint16_t length) {
    return this->send(this->canAddr, payload, length);
}

/**
    Sends any long message

    @param addr address/priority
    @param payload the data to be sent
    @param length length of the data
    @return error code
*/
int CANTT::send(uint32_t addr, uint8_t* payload, uint16_t length) {
    if(length > CANTT_MAX_DATASIZE || payload == NULL) {
        return 1;
    }

    if(this->waitUntilIdle() != 0) {
        return -1;
    }

    this->tx.address = addr;
    this->tx.size = length;
    this->tx.message_pos = 0;
    memset(this->tx.message, 0, CANTT_MAX_RECV_BUFFER);
    memcpy(this->tx.message, payload, length);

    return 0;
}

/**
    Loop to run the internal state machine
*/
void CANTT::loop() {
    if(this->timeOutTimer > millis()) { // overflow after ~50days;
        this->timeOutTimer = millis();
    }

    if(this->timeOutTimer > 0 && this->timeOutTimer + CANTT_STATE_TIMEOUT < millis()) {
        this->changeState(IDLE);
        clearRX(); // TEST
    }

    switch(stateMachine) {
        case IDLE: // FIXME: not the correct name for this state
        case CHECKREAD:
            if(this->canAvailable()) {
                this->changeState(READ);

            } else if (this->inReception() == false) { // We can only send if we are not currently receiving...
                // As long as we ensure to not send anything when we are still receiving, there should not be any issues...
                this->changeState(CHECKSEND);
            }
        break;

        case CHECKSEND:
            if(this->hasOutgoingMessage()) {
                if(this->tx.size <= 7) {
                    this->changeState(SEND_SINGLE);

                } else if(this->tx.message_pos == 0) {
                    this->changeState(SEND_FIRST);

                } else if(this->inTransmission()) {
                    this->changeState(SEND_CONSECUTIVE);
                }
            } else {
                this->changeState(IDLE);
            }
        break;

        case READ:
            if(this->recvMessage() == 0) {
                this->changeState(PARSE_WHICH);

                if(this->hasOutgoingMessage()) {
                    // We need to resend the current outgoing message
                    // as it will have collided on the bus with this new message

                    this->rewindTX();

                    if(this->rx.address > this->canAddr) { // lower is more important
                        this->clearRX();
                    } else {
                        delay(CANTT_DEFAULT_HOLDOFF_DELAY);
                    }

                    this->changeState(CHECKREAD);
                }
            } else {
                this->changeState(CHECKREAD);
            }
        break;

        case PARSE_WHICH:
            if(FRAME_TYPE(this->rx.can.data[0]) == CANTT_SINGLE_FRAME) {
                this->parseSingle();
                this->changeState(IDLE);

            } else if (FRAME_TYPE(this->rx.can.data[0]) == CANTT_FIRST_FRAME) {
                this->parseFirst();
                this->changeState(CHECKREAD);

            } else if (FRAME_TYPE(this->rx.can.data[0]) == CANTT_CONSECUTIVE_FRAME) {
                if(this->parseConsecutive() == 0) {
                    this->changeState(IDLE);
                } else {
                    this->changeState(CHECKREAD);  // Fetch a new frame
                }

            } /* else if (FRAME_TYPE(this->canBuf[0]) == CANTT_FLOWCTRL_FRAME) {

                if(this->canBuf[0] & 0x0F == CANTT_FLOW_CLEAR) { // mask for 0000 1111
                    if(this->canBuf[1] >= this->block_size) {
                        this->block_size = this->canBuf[1];
                    }

                    if(this->canBuf[2] >= this->wait_time) {
                        this->wait_time = this->canBuf[2];
                    }
                } else {
                    // Some other kind of Flow Ctrl Frame
                }

            } */ else {
                // Ignore frame and switch to CHECKREAD state
                this->changeState(CHECKREAD);
            }
        break;

        case SEND_SINGLE:
            if(this->sendSingle() == 0) {
                this->clearTX();
                this->changeState(IDLE);
            }
        break;

        case SEND_FIRST:
            if(this->sendFirst() == 0) {
                this->changeState(CHECKREAD); // To check for collision
            }
        break;

        case SEND_CONSECUTIVE:
            if(this->sendConsecutive() == 0) { // Done with sending the multiframe
                this->clearTX();
                this->changeState(IDLE);
            } else {
                this->changeState(CHECKREAD); // To check for collision
            }
        break;
        /*
        case SEND_FLOW:
            // FIXME: Where should the reply go? It can't be a fixed location.
            // The easiest way that I see, is to use the inverse of that address.
            // 0x000 <-> 0x7FF, 0x100 <-> 0x6FF, 0x160 <-> 0x69F etc.

            this->sendFlowFrame(this->canAddr,
                                CANTT_FLOW_CLEAR,
                                this->block_size, this->wait_time);

            this->changeState(BUSY);  // Fetch a new frame
        break;
        */
        /*
        case RECV_FLOW:
            this->recvMessage();
            if (FRAME_TYPE(this->canBuf[0]) == CANTT_FLOWCTRL_FRAME) {
                this->parseFlow();
                this->changeState(SEND_CONSECUTIVE);
            } else {
                // Someone else is speaking right now.
                // Abort
            }
        break;
        */
    }
}

/*

int CANTT::sendFlowFrame(long unsigned int arbId, uint8_t fc_flag, uint8_t block_size, uint8_t separation_time) {
    uint8_t data[3] = {
        (uint8_t)((CANTT_FLOWCTRL_FRAME << 4) | fc_flag),
        block_size,
        separation_time
    };

    uint8_t sndStat = this->bus->sendMsgBuf(arbId, sizeof(data), data);
    if (sndStat == 0) {
        return 0;
    }

    return 1;
}

void CANTT::parseFlow() {
    this->flowExpected = this->canBuf[1];
    if(this->flowExpected == 0) {
        this->flowExpected = -1; // Disable
    }
}
*/
