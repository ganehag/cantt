/* isotp library by Mikael Ganehag Brorsson
 */

#include "Arduino.h"
#include "isotp.h"

/**
 * Constructor.
 */
IsoTp::IsoTp(MCP_CAN* bus, uint8_t mcp_int,  void (*callback)(long unsigned int, uint8_t*, unsigned int)) {

    // Arguments
    this->bus = bus;
    this->mcp_int = mcp_int;
    this->callback = callback;

    this->stateMachine = DISABLED;

    this->canRxId = 0;
    this->canTxId = 0;
    this->canLen = 0;
    memset(this->canBuf, 0, sizeof(this->canBuf));

    this->mfBufSize = 0;
    this->mfDataSize = 0;
    memset(this->mfBuf, 0, sizeof(this->mfBufSize));

    this->wait_time = 0x05; // 5ms
    this->block_size = 0;
    this->flowExpected = -1; // Never
    this->numFramesSent = 0;
    this->timeOut = millis();
}

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


void IsoTp::begin() {
    pinMode(this->mcp_int, INPUT);
    this->changeState(IDLE);
}


void IsoTp::parseSingle() {
    uint8_t frameSize = this->canBuf[0] & ISOTP_SINGLE_SIZE_MASK;

    // Ensure that the frame has the correct size
    if(frameSize == this->canLen - 1 &&
       frameSize > 0 && frameSize < 8 && 
       this->callback != NULL) {
        this->callback(this->canRxId, this->canBuf, this->canBuf[0] & ISOTP_SINGLE_SIZE_MASK);
    }
}

void IsoTp::parseFirst() {
    uint16_t frameSize = ((this->canBuf[0] & ISOTP_SINGLE_SIZE_MASK) << 8) | this->canBuf[1];

    if(frameSize >= 8 && frameSize <= ISOTP_MAX_DATASIZE) {
        this->mfDataSize = frameSize;

        memcpy(this->mfBuf, &this->canBuf[2], 6); // All of the remaining data in this frame
        this->mfBufSize = 6;
    }
}

int IsoTp::parseConsecutive() {
    // Ensure that all the frames comes to/from the same location (addr). Otherwise we will have a mixup
    // FIXME: keep track of block counts, so that we can send a new Flow frame if required.

    uint16_t frameIndex = this->canBuf[0] & ISOTP_CONSECUTIVE_INDEX_MASK;

    if(this->mfDataSize - this->mfBufSize > 7) { // not the last frame
        memcpy(&this->mfBuf[this->mfBufSize], &this->canBuf[1], 7);
         this->mfBufSize += 7;
    } else {  // this (should be) the last frame
        memcpy(&this->mfBuf[this->mfBufSize], &this->canBuf[1], this->mfDataSize - this->mfBufSize);
         this->mfBufSize = this->mfDataSize;

        this->callback(this->canRxId, this->mfBuf, this->mfBufSize);
        
        this->mfDataSize = 0;
        this->mfBufSize = 0;
    }

    return this->mfDataSize - this->mfBufSize;

    // check for number of frames
    // send a new flow frame if this->block_size != 0
    // sendFlowFrame
}

void IsoTp::parseFlow() {
    this->flowExpected = this->canBuf[1];
    if(this->flowExpected == 0) {
        this->flowExpected = -1; // Disable
    }
}

void IsoTp::changeState(enum state_m s) {
    // FIXME: Handle variable reset etc...

    // Serial.println(s);
    this->stateMachine = s;
    if(this->stateMachine == IDLE) {
        this->timeOut = 0;
    } else {
        this->timeOut = millis();
    }
}

int IsoTp::sendSingle() {
    memset(this->canBuf, 0, sizeof(this->canBuf));

    this->canBuf[0] = (ISOTP_SINGLE_FRAME << 4) | this->mfBufSize;
    memcpy(&this->canBuf[1], this->mfBuf, 7);

    if(this->bus->sendMsgBuf(this->canTxId, sizeof(this->canBuf), this->canBuf) != CAN_OK) {
        this->changeState(IDLE);
        return 1;
    }

    return 0;
}

int IsoTp::sendFirst() {
    memset(this->canBuf, 0, sizeof(this->canBuf));

    this->canBuf[0] = (ISOTP_FIRST_FRAME << 4) | (this->mfBufSize >> 8);
    this->canBuf[1] = this->mfBufSize & ISOTP_FIRST_SIZE_MASK_BYTE1;
    memcpy(&this->canBuf[2], this->mfBuf, 6);

    if(this->bus->sendMsgBuf(this->canTxId, sizeof(this->canBuf), this->canBuf) != CAN_OK) {
        this->changeState(IDLE);
        return 1;
    } else {
        this->mfDataSize = 6;
        this->numFramesSent = 1;
    }

    return 0;
}

int IsoTp::sendConsecutive() {
    uint8_t maxSend = 7;

    if(this->mfBufSize - this->mfDataSize <= 7) {
        maxSend = this->mfBufSize - this->mfDataSize;
    }

    this->canBuf[0] = (ISOTP_CONSECUTIVE_FRAME << 4) | (this->numFramesSent % 0x0F);
    memcpy(&this->canBuf[1], &this->mfBuf[this->mfDataSize], maxSend);

    if(this->bus->sendMsgBuf(this->canTxId, maxSend + 1, this->canBuf) != CAN_OK || 
       this->mfDataSize == this->mfBufSize) {
        /*
       this->mfDataSize = 0;
       this->mfBufSize = 0;
       this->outbox = 0;
       this->flowExpected = -1;
       */

        this->changeState(IDLE);
    } else {
        this->mfDataSize += maxSend;
        this->numFramesSent++;
    }

    return this->mfBufSize - this->mfDataSize;
}

void IsoTp::loop() {
    if(this->timeOut > millis()) { // overflow after ~50days;
        this->timeOut = millis();
    }

    if(this->timeOut > 0 && this->timeOut + ISOTP_MACHINE_TIMEOUT < millis()) {
        this->changeState(IDLE);
    }


    // FIXME: timeout for multi frame recv.
    // this->inbox = false;

    switch(stateMachine) {
        case IDLE:
        case BUSY:
            if(!digitalRead(this->mcp_int)) {
                this->changeState(READ);
            }
        break;

        case READ:
            this->bus->readMsgBuf(&this->canRxId, &this->canLen, this->canBuf);
            this->changeState(PARSE_WHICH);
        break;

        case PARSE_WHICH:
            if(FRAME_TYPE(this->canBuf[0]) == ISOTP_SINGLE_FRAME) {
                this->parseSingle();
                this->changeState(IDLE);

            } else if (FRAME_TYPE(this->canBuf[0]) == ISOTP_FIRST_FRAME) {
                this->parseFirst();
                this->changeState(SEND_FLOW);

            } else if (FRAME_TYPE(this->canBuf[0]) == ISOTP_CONSECUTIVE_FRAME) {
                if(this->parseConsecutive() == 0) {
                    this->changeState(IDLE);
                } else {
                    this->changeState(BUSY);  // Fetch a new frame    
                }
                
            } else {
                // Abort
            }
        break;

        case SEND_FLOW:
            // FIXME: Where should the reply go? It can't be a fixed location.
            // The easiest way that I see, is to use the inverse of that address.
            // 0x000 <-> 0x7FF, 0x100 <-> 0x6FF, 0x160 <-> 0x69F etc.
            this->sendFlowFrame(this->canRxId ^ ISOTP_MAX_ADDR,
                                ISOTP_FLOW_CLEAR,
                                this->block_size, this->wait_time);  
            this->changeState(BUSY);  // Fetch a new frame
        break;

        case SEND_SINGLE:
            if(this->sendSingle() == 0) {
                this->changeState(IDLE);
            }
        break;

        case SEND_FIRST:
            if(this->sendFirst() == 0) {
                this->changeState(RECV_FLOW);
            }
        break;

        case SEND_CONSECUTIVE:
            if(this->sendConsecutive() == 0) {
                this->changeState(IDLE);
            }
        break;

        case RECV_FLOW:
            this->bus->readMsgBuf(&this->canRxId, &this->canLen, this->canBuf);
            if (FRAME_TYPE(this->canBuf[0]) == ISOTP_FLOWCTRL_FRAME) {
                this->parseFlow();
                this->changeState(SEND_CONSECUTIVE);
            } else {
                // Abort
            }
        break;
    }
}

int IsoTp::send(long unsigned int destination, uint8_t* payload, unsigned int length) {
    if(length > ISOTP_MAX_DATASIZE || payload == NULL) {
        return 1;
    }

    // Busy block until recv is done and the buffer is available
    while(this->stateMachine != IDLE) {
        this->loop();
    }

    this->canTxId = destination;
    memcpy(this->mfBuf, payload, length);
    this->mfBufSize = length;

    if(this->mfBufSize <= 7) {  // Single frame
        this->changeState(SEND_SINGLE);
    } else { // multiframe
        this->changeState(SEND_FIRST);
    }

    return 0;
}
