/* isotp library by Mikael Ganehag Brorsson
 */

#include "Arduino.h"
#include "isotp.h"

/**
 * Constructor.
 */
IsoTp::IsoTp(MCP_CAN* bus, uint8_t mcp_int,  uint32_t canAddr, void (*callback)(long unsigned int, uint8_t*, unsigned int)) {
    this->initialize(bus, mcp_int, canAddr, ISOTP_MACHINE_TIMEOUT, callback);
}

IsoTp::IsoTp(MCP_CAN* bus, uint8_t mcp_int,  uint32_t canAddr, uint32_t timeout, void (*callback)(long unsigned int, uint8_t*, unsigned int)) {
    this->initialize(bus, mcp_int, canAddr, timeout, callback);
}

void IsoTp::initialize(MCP_CAN* bus, uint8_t mcp_int, uint32_t canAddr, uint32_t timeout, void (*callback)(long unsigned int, uint8_t*, unsigned int)) {
    // Arguments
    this->bus = bus;
    this->mcp_int = mcp_int;
    this->callback = callback;

    this->stateMachine = DISABLED;

    this->canAddr = canAddr;

    // TX
    this->tx.address = 0;
    this->tx.can.id = 0;
    this->tx.can.extended = false;
    this->tx.can.rtr = false;
    memset(this->tx.can.data, 0, CAN_MAX_DATA_SIZE);
    this->tx.size = 0;
    this->tx.message_pos = 0;
    memset(this->tx.message, 0, ISOTP_MAX_RECV_BUFFER);
    this->tx.frameCounter = 0;

    // RX
    this->rx.address = 0;
    this->rx.can.id = 0;
    this->rx.can.extended = false;
    this->rx.can.rtr = false;
    memset(this->rx.can.data, 0, CAN_MAX_DATA_SIZE);
    this->rx.size = 0;
    this->rx.message_pos = 0;
    memset(this->rx.message, 0, ISOTP_MAX_RECV_BUFFER);
    this->rx.frameCounter = 0;


    // this->canRxId = 0;
    // this->canTxId = 0;
    // this->frameRxId = ISOTP_INVALID_ADDR;
    // this->canLen = 0;
    // memset(this->canBuf, 0, sizeof(this->canBuf));

    // this->mfBufSize = 0;
    // this->mfDataSize = 0;
    // memset(this->mfBuf, 0, sizeof(this->mfBufSize));

    this->wait_time = ISOTP_DEFAULT_WAIT_TIME; // 5ms
    this->block_size = 0;
    this->flowExpected = -1; // Never
    // this->numFramesSent = 0;
    this->timeOutTimer = millis();
    this->timeout = timeout;
}

/*

int IsoTp::sendFlowFrame(long unsigned int arbId, uint8_t fc_flag, uint8_t block_size, uint8_t separation_time) {
    uint8_t data[3] = {
        (uint8_t)((ISOTP_FLOWCTRL_FRAME << 4) | fc_flag),
        block_size,
        separation_time
    };

    uint8_t sndStat = this->bus->sendMsgBuf(arbId, sizeof(data), data);
    if (sndStat == CAN_OK) {
        return 0;
    }

    return 1;
}

*/


void IsoTp::begin() {
    pinMode(this->mcp_int, INPUT);
    this->changeState(IDLE);
}

/*
void IsoTp::parseFlow() {
    this->flowExpected = this->canBuf[1];
    if(this->flowExpected == 0) {
        this->flowExpected = -1; // Disable
    }
}
*/

void IsoTp::changeState(enum state_m s) {
    // FIXME: Handle variable reset etc...

    if(s == IDLE && this->hasOutgoingMessage()) {
        delay(100); // FIXME

        // Can't IDLE when I have stuff to do.
        if(this->tx.size <= 7) {
            s = SEND_SINGLE;
        } else { // multiframe
            s = SEND_FIRST;
        }    
    }

    this->stateMachine = s;

    if(this->stateMachine == IDLE) {
        this->timeOutTimer = 0;
    } else {
        this->timeOutTimer = millis();
    }
}

void IsoTp::parseSingle() {
    uint8_t frameSize = this->rx.can.data[0] & ISOTP_SINGLE_SIZE_MASK;

    // Ensure that the frame has the correct size
    if(frameSize == this->rx.can.len - 1 &&
       frameSize > 0 && frameSize < 8 && 
       this->callback != NULL) {
        // Use the data directly from the can buffer, no need to use the message buffer
        this->callback(this->rx.address, &this->rx.can.data[1], frameSize);
    }
}

void IsoTp::parseFirst() {
    uint16_t frameSize = ((this->rx.can.data[0] & ISOTP_SINGLE_SIZE_MASK) << 8) | this->rx.can.data[1];

    memset(this->rx.message, 0, ISOTP_MAX_RECV_BUFFER);

    if(frameSize >= 8 && frameSize <= ISOTP_MAX_DATASIZE) {
        this->rx.size = frameSize;

        memcpy(this->rx.message, &this->rx.can.data[2], 6); // All of the remaining data in this frame
        this->rx.message_pos = 6;
    }
}

int IsoTp::parseConsecutive() {
    // FIXME: Support for some kind of timeout?
    // FIXME: keep track of block counts, so that we can send a new Flow frame if required.

    // if(this->frameRxId != ISOTP_INVALID_ADDR && this->canRxId != this->frameRxId) {
    //    return -1;
    //}

    uint16_t frameIndex = this->rx.can.data[0] & ISOTP_CONSECUTIVE_INDEX_MASK;

    if(this->rx.size - this->rx.message_pos > 7) {
        // not the last frame
        memcpy(&this->rx.message[this->rx.message_pos], &this->rx.can.data[1], 7);
        this->rx.message_pos += 7;

    } else {
        // this (should be) the last frame
        memcpy(&this->rx.message[this->rx.message_pos], &this->rx.can.data[1], this->rx.size - this->rx.message_pos);
        this->rx.message_pos = this->rx.size;

        this->callback(this->rx.address, this->rx.message, this->rx.size);
        
        memset(this->rx.message, 0 , ISOTP_MAX_RECV_BUFFER);
        this->rx.size = 0;
        this->rx.message_pos = 0;
    }

    return this->rx.size - this->rx.message_pos;

    // check for number of frames
    // send a new flow frame if this->block_size != 0
    // sendFlowFrame
}

int IsoTp::sendSingle() {
    memset(this->tx.can.data, 0, CAN_MAX_DATA_SIZE);

    this->tx.can.data[0] = (ISOTP_SINGLE_FRAME << 4) | this->tx.size;
    memcpy(&this->tx.can.data[1], this->tx.message, 7);
    this->tx.can.len = 1 + this->tx.size;

    return this->sendMessage();
}

int IsoTp::sendFirst() {
    memset(this->tx.can.data, 0, CAN_MAX_DATA_SIZE);

    this->tx.can.data[0] = (ISOTP_FIRST_FRAME << 4) | (this->tx.size >> 8);
    this->tx.can.data[1] = this->tx.size & ISOTP_FIRST_SIZE_MASK_BYTE1;
    memcpy(&this->tx.can.data[2], this->tx.message, 6);
    this->tx.can.len = CAN_MAX_DATA_SIZE;

    if(this->sendMessage() != 0) {
        this->changeState(IDLE);
        return 1;
    } else {
        this->tx.message_pos = 6;
        this->tx.frameCounter = 1;
    }

/*
    // Stay a while and wait...
    if(this->wait_time > 0 && this->tx.size - this->tx.message_pos > 0) {
        delay(this->wait_time); // Is this the correct way to do it
    }
*/

    return 0;
}

int IsoTp::sendConsecutive() {
    uint8_t maxSend = 7;

    memset(this->tx.can.data, 0, CAN_MAX_DATA_SIZE);

    // Set frame type and counter
    this->tx.can.data[0] = (ISOTP_CONSECUTIVE_FRAME << 4) | (this->tx.frameCounter % 0x0F);

    // Copy some or remaining data
    if(this->tx.size - this->tx.message_pos <= 7) {
        maxSend = this->tx.size - this->tx.message_pos;
    }
    memcpy(&this->tx.can.data[1], &this->tx.message[this->tx.message_pos], maxSend);

    // FIXME: change to properly handle RTX & Extended
    this->tx.can.len = 1 + maxSend; // HDR + data
    this->tx.can.id = this->tx.address;

    if(this->sendMessage() != CAN_OK) {
        this->changeState(IDLE);
    } else {
        this->tx.message_pos += maxSend;
        this->tx.frameCounter++;
    }

    if(this->tx.size <= this->tx.message_pos) {
        this->changeState(IDLE);
    }
/*
    // Stay a while and wait...
    if(this->wait_time > 0 && this->tx.size - this->tx.message_pos > 0) {
        delay(this->wait_time); // Is this the correct way to do it
    }
*/
    return this->tx.size - this->tx.message_pos;
}

int IsoTp::recvMessage() {
    memset(this->rx.can.data, 0, CAN_MAX_DATA_SIZE);

    if (this->bus->readMsgBuf(&this->rx.can.id, &this->rx.can.len, this->rx.can.data) != CAN_OK) {
        return 1;
    }

    this->rx.address = this->rx.can.id;
    // FIXME: can.id contains RTX and EXTENDED bits.

    return 0;
}

int IsoTp::sendMessage() {
    if (this->bus->sendMsgBuf(this->tx.can.id, this->tx.can.len, this->tx.can.data) != CAN_OK) {
        return 1;
    }

    return 0;
}

void IsoTp::loop() {
    if(this->timeOutTimer > millis()) { // overflow after ~50days;
        this->timeOutTimer = millis();
    }

    if(this->timeOutTimer > 0 && this->timeOutTimer + ISOTP_MACHINE_TIMEOUT < millis()) {
        this->changeState(IDLE);
    }


    // FIXME: timeout for multi frame recv.
    // this->inbox = false;

    switch(stateMachine) {
        case IDLE:
        case CHECKREAD:
            if(!digitalRead(this->mcp_int)) {
                this->changeState(READ);
            }
        break;

        case CHECK_COLLISION: // Check if something else arrived on the bus while sending multiframe message
            if(digitalRead(this->mcp_int) == 0) {
                if(this->recvMessage() == 0) {
                    if(FRAME_TYPE(this->rx.can.data[0]) == ISOTP_FIRST_FRAME || FRAME_TYPE(this->rx.can.data[0]) == ISOTP_CONSECUTIVE_FRAME) {
                        // Collision occued
                        // RX Result will be garbage without "per address frame buffer" support

                    
                        if(this->rx.address > this->canAddr) { // FrameId > MyId
                            // I have priority
                            // Go back to square A1
                            this->changeState(SEND_FIRST);
                        } else {
                            this->changeState(CHECKREAD); // Need to read and handle RX message first
                        }
                        
                    } else {
                        // No problem, not a collision
                        this->changeState(SEND_CONSECUTIVE);
                    }
                } else { // unable to recv message
                    // Failure
                    this->changeState(IDLE); // Retry?
                }    
            } else {
                // No problem, nothing on the RX line
                this->changeState(SEND_CONSECUTIVE);
            }
        break;

        case READ:
            if(this->recvMessage() == 0) {
                this->changeState(PARSE_WHICH);
            } else {
                this->changeState(CHECKREAD);
            }
        break;

        case PARSE_WHICH:
            if(FRAME_TYPE(this->rx.can.data[0]) == ISOTP_SINGLE_FRAME) {
                this->parseSingle();
                this->changeState(IDLE);

            } else if (FRAME_TYPE(this->rx.can.data[0]) == ISOTP_FIRST_FRAME) {
                // if(this->frameRxId == ISOTP_INVALID_ADDR) { // Not occupied with another frame
                    // this->frameRxId = this->canRxId;
                    this->parseFirst();
                    // this->changeState(SEND_FLOW);    
                    this->changeState(CHECKREAD);
                // }
            } else if (FRAME_TYPE(this->rx.can.data[0]) == ISOTP_CONSECUTIVE_FRAME) {
                if(this->parseConsecutive() == 0) {
                    // this->frameRxId = ISOTP_INVALID_ADDR;
                    this->changeState(IDLE);
                } else {
                    this->changeState(CHECKREAD);  // Fetch a new frame    
                }
                
            } /* else if (FRAME_TYPE(this->canBuf[0]) == ISOTP_FLOWCTRL_FRAME) {

                if(this->canBuf[0] & 0x0F == ISOTP_FLOW_CLEAR) { // mask for 0000 1111
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
/*
        case SEND_FLOW:
            // FIXME: Where should the reply go? It can't be a fixed location.
            // The easiest way that I see, is to use the inverse of that address.
            // 0x000 <-> 0x7FF, 0x100 <-> 0x6FF, 0x160 <-> 0x69F etc.

            this->sendFlowFrame(this->canAddr,
                                ISOTP_FLOW_CLEAR,
                                this->block_size, this->wait_time);
            
            this->changeState(BUSY);  // Fetch a new frame
        break;
        */

        case SEND_SINGLE:
            if(this->sendSingle() == 0) {
                this->clearTX();

                this->changeState(IDLE);
            }
        break;

        case SEND_FIRST:
            if(this->sendFirst() == 0) {
                this->changeState(CHECK_COLLISION);
                // this->changeState(RECV_FLOW);
            }
        break;

        case SEND_CONSECUTIVE:
            if(this->sendConsecutive() == 0) {
                // Done with sending the multiframe
                this->clearTX();

                this->changeState(IDLE);
            } else {
                this->changeState(CHECK_COLLISION);
            }
        break;
        /*
        case RECV_FLOW:
            this->recvMessage();
            if (FRAME_TYPE(this->canBuf[0]) == ISOTP_FLOWCTRL_FRAME) {
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

void IsoTp::clearTX() {
    memset(this->tx.message, 0, ISOTP_MAX_RECV_BUFFER);
    this->tx.message_pos = 0;
    this->tx.size = 0;
    this->tx.address = 0;
};

bool IsoTp::hasOutgoingMessage() {
    return this->tx.size > 0;
}

int IsoTp::send(uint8_t * payload, uint16_t length) {
    this->send(this->canAddr, payload, length);
}

int IsoTp::send(uint32_t addr, uint8_t* payload, uint16_t length) {
    if(length > ISOTP_MAX_DATASIZE || payload == NULL) {
        return 1;
    }

    // Busy block until recv is done and the buffer is available
    while(this->stateMachine != IDLE) {
        this->loop();
    }

    /*
    this->tx.can.id = addr & ISOTP_ADDR_UNMASK;
    this->tx.can.extended = IS_EXTENDED(addr);
    this->tx.can.rtr = IS_RTR(addr);
    this->tx.can.len = length;
    */

    this->tx.address = addr;
    this->tx.size = length;
    this->tx.message_pos = 0;
    memset(this->tx.message, 0, ISOTP_MAX_RECV_BUFFER);
    memcpy(this->tx.message, payload, length);

    if(this->tx.size <= 7) {  // Single frame
        this->changeState(SEND_SINGLE);
    } else { // multiframe
        this->changeState(SEND_FIRST);
    }

    return 0;
}
