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

typedef struct
{
    uint16_t index;
    uint8_t subIndex;
    uint8_t arraySize;
    uint8_t data[4];
}dictionaryObject_t;


typedef struct
{
    bool errorFlag;         //Set if error is present, cleared when error confirmed cleared
    uint16_t index;         //Server OD index
    uint8_t subIndex;       //Server OD sub index
    uint32_t errorCode;     //Error code provided by server
}CANopenError_t;

void CANopenTransmit(CANopenMessage_t msgOutCANopen);
CANopenMessage_t CANopenReceive(void);
void CAN_transmit(CANMessage_t msgOutCAN);
void SDO_writeObject(uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t arraySize, uint8_t data[]);
void SDO_readObject(uint8_t nodeID, uint16_t index, uint8_t subIndex);
void SDO_writeObjectInit(uint8_t nodeID, dictionaryObject_t param);


//DIRECT MOTOR CONTROL
//void PD4C_setup(uint8_t nodeID);
void PD4C_openPetal(uint8_t nodeID);
void PD4C_closePetal(uint8_t nodeID);
//void PD4C_stateMachine(void);


/*
dictionaryObject_t PD4E_init_param[12];


//INIT PARAMS

//Motor Revolution (per shaft revolution)
PD4E_init_param[0].index = 0x6091;
PD4E_init_param[0].subIndex = 0x01;
PD4E_init_param[0].size = 0x04;
PD4E_init_param[0].data[0] = 0x01; PD4E_init_param[0].data[1] = 0; PD4E_init_param[0].data[2] = 0; PD4E_init_param[0].data[3] = 0;

//Shaft Revolution (per motor revolution)
PD4E_init_param[1].index = 0x6091;
PD4E_init_param[1].subIndex = 0x02;
PD4E_init_param[1].size = 0x04;
PD4E_init_param[1].data[0] = 0x01; PD4E_init_param[1].data[1] = 0; PD4E_init_param[1].data[2] = 0; PD4E_init_param[1].data[3] = 0;

//Feed
PD4E_init_param[2].index = 0x6092;
PD4E_init_param[2].subIndex = 0x01;
PD4E_init_param[2].size = 0x04;
PD4E_init_param[2].data[0] = 0x01; PD4E_init_param[2].data[1] = 0; PD4E_init_param[2].data[2] = 0; PD4E_init_param[2].data[3] = 0;

//Drive Axis Revs (to achieve that feed)
PD4E_init_param[3].index = 0x6092;
PD4E_init_param[3].subIndex = 0x02;
PD4E_init_param[3].size = 0x04;
PD4E_init_param[3].data[0] = 0x01; PD4E_init_param[3].data[1] = 0; PD4E_init_param[3].data[2] = 0; PD4E_init_param[3].data[3] = 0;

//Mode
PD4E_init_param[4].index = 0x6060;
PD4E_init_param[4].subIndex = 0x00;
PD4E_init_param[4].size = 0x01;
PD4E_init_param[4].data[0] = 0x04; PD4E_init_param[4].data[1] = 0; PD4E_init_param[4].data[2] = 0; PD4E_init_param[4].data[3] = 0;

//Target Torque
PD4E_init_param[5].index = 0x6071;
PD4E_init_param[5].subIndex = 0x00;
PD4E_init_param[5].size = 0x02;
PD4E_init_param[5].data[0] = 0x90; PD4E_init_param[5].data[1] = 0x01; PD4E_init_param[5].data[2] = 0; PD4E_init_param[5].data[3] = 0;

//Max Torque
PD4E_init_param[6].index = 0x6072;
PD4E_init_param[6].subIndex = 0x00;
PD4E_init_param[6].size = 0x02;
PD4E_init_param[6].data[0] = 0xF4; PD4E_init_param[6].data[1] = 0x01; PD4E_init_param[6].data[2] = 0; PD4E_init_param[6].data[3] = 0;

//Torque Slope (this is rise -- run = 1)
PD4E_init_param[7].index = 0x6087;
PD4E_init_param[7].subIndex = 0x00;
PD4E_init_param[7].size = 0x04;
PD4E_init_param[7].data[0] = 0xF4; PD4E_init_param[7].data[1] = 0x01; PD4E_init_param[7].data[2] = 0; PD4E_init_param[7].data[3] = 0;

//Closed Loop
PD4E_init_param[8].index = 0x3202;
PD4E_init_param[8].subIndex = 0x00;
PD4E_init_param[8].size = 0x04;
PD4E_init_param[8].data[0] = 0x01; PD4E_init_param[8].data[1] = 0; PD4E_init_param[8].data[2] = 0; PD4E_init_param[8].data[3] = 0;

//Velocity Numerator (1 rev)
PD4E_init_param[9].index = 0x604C;
PD4E_init_param[9].subIndex = 0x01;
PD4E_init_param[9].size = 0x04;
PD4E_init_param[9].data[0] = 0x01; PD4E_init_param[9].data[1] = 0; PD4E_init_param[9].data[2] = 0; PD4E_init_param[9].data[3] = 0;

//Velocity Denominator (per 60 seconds)
PD4E_init_param[10].index = 0x604C;
PD4E_init_param[10].subIndex = 0x02;
PD4E_init_param[10].size = 0x04;
PD4E_init_param[10].data[0] = 0x3C; PD4E_init_param[10].data[1] = 0; PD4E_init_param[10].data[2] = 0; PD4E_init_param[10].data[3] = 0;

//Max Speed (user units are RPM)
PD4E_init_param[11].index = 0x6080;
PD4E_init_param[11].subIndex = 0x00;
PD4E_init_param[11].size = 0x04;
PD4E_init_param[11].data[0] = 0x2C; PD4E_init_param[11].data[1] = 0x01; PD4E_init_param[11].data[2] = 0; PD4E_init_param[11].data[3] = 0;

//Nominal Current
PD4E_init_param[12].index = 0x203B;
PD4E_init_param[12].subIndex = 0x01;
PD4E_init_param[12].size = 0x04;
PD4E_init_param[12].data[0] = 0xA0; PD4E_init_param[12].data[1] = 0x0F; PD4E_init_param[12].data[2] = 0; PD4E_init_param[12].data[3] = 0;

//Max Current
PD4E_init_param[12].index = 0x203B;
PD4E_init_param[12].subIndex = 0x01;
PD4E_init_param[12].size = 0x04;
PD4E_init_param[12].data[0] = 0xA0; PD4E_init_param[12].data[1] = 0x0F; PD4E_init_param[12].data[2] = 0; PD4E_init_param[12].data[3] = 0;

*/


