//#ifndef NANOTEC_H
//#define NANOTEC_H
#pragma once
#include <framework/modules/can/can.h>



//Define CAN message structure
typedef struct
{
    uint32_t sid;
    bool ide;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
}CANMessage_t;

//Define CANopen message structure
typedef struct
{
    uint16_t functionCode;
    uint8_t nodeID;
    bool ide;
    bool rtr;
    uint8_t dlc;
    uint8_t data[8];
}CANopenMessage_t;

void CANopenTransmit(CANopenMessage_t msgOutCANopen);
CANopenMessage_t CANopenReceive(void);
void CAN_transmit(CANMessage_t msgOutCAN);
void SDO_writeObject(uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t arraySize, uint8_t data[]);
void SDO_readObject(uint8_t nodeID, uint16_t index, uint8_t subIndex);

//DIRECT MOTOR CONTROL
//void PD4C_setup(uint8_t nodeID);
void PD4C_openPetal(uint8_t nodeID);
void PD4C_closePetal(uint8_t nodeID);
//void PD4C_stateMachine(void);
    
    

/*test and unlock
nanotec(uint32_t baud);
void begin();
bool available();
void CANopenTransmit(CANopenMessage_t msgOutCANopen);
CANopenMessage_t CANopenReceive();
void CAN_transmit();
void SDO_writeObject(uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t arraySize, uint8_t data[]);
uint32_t SDO_readObject(uint8_t nodeID, uint16_t index, unit8_t subIndex);
*/



//ONLY APPLIES TO C++
/*
class nanotec
{
    public:
        nanotec(uint32_t baud);
        void begin();
        bool available();
        void CANopenTransmit(CANopenMessage_t msgOutCANopen);
        CANopenMessage_t CANopenReceive();
        void CAN_transmit();
        void SDO_writeObject(uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t arraySize, uint8_t data[]);
        uint32_t SDO_readObject(uint8_t nodeID, uint16_t index, unit8_t subIndex);


    private:
        uint32_t _baud;
};
*/
//#endif
