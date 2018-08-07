#include <modules/worker_thread/worker_thread.h>
#include <modules/can/can.h>
#include "nanotec.h"

//nanotec(uint32_t baud);
//void begin();
//bool available();


//Subscriber!!!


#ifndef NANOTEC_WORKER_THREAD
#error Please define NANOTEC_WORKER_THREAD in framework_conf.h.
#endif

#define WT NANOTEC_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_listener_task_s rx_listener_task;
static void nanotec_can_rx_handler(size_t msg_size, const void* buf, void* ctx);
CANMessage_t motorStatus;
uint8_t masterCommand;

RUN_BEFORE(INIT_END) {
    struct can_instance_s* can = can_get_instance(0);
    struct pubsub_topic_s* can_rx_topic = can_get_rx_topic(can);
    if (!can_rx_topic) return;
    worker_thread_add_listener_task(&WT, &rx_listener_task, can_rx_topic, nanotec_can_rx_handler, NULL);
}

static void nanotec_can_rx_handler(size_t msg_size, const void* msg, void* ctx) 
{
    (void) msg_size;
    const struct can_rx_frame_s* frame = msg;
    
    if((frame->content.IDE == 0) && ((frame->content.SID == 0x30)))
    {
        masterCommand = frame->content.data[0];
        CANMessage_t rebound;
        rebound.sid = 0x40;
        rebound.ide = 0;
        rebound.rtr = 0;
        rebound.dlc = 8;
        rebound.data[0] = masterCommand;
        
        for(uint8_t i = 1; i < 8; i++)
        {
            rebound.data[i] = i;
        }
        CAN_transmit(rebound);
    }
    
    //Check if it's a CANopen frame
    else if((frame->content.IDE == 0) && (((frame->content.SID >> 4) & 0xF8) == 0x58))   //If not extended ID and function code is 0x58, this is CANopen message
    {
        CANopenMessage_t msgInCANopen;
    
        msgInCANopen.functionCode = ((frame->content.SID >> 4)&0xF8);
        msgInCANopen.nodeID = (frame->content.SID & 0x7F);
        msgInCANopen.ide = frame->content.IDE;
        msgInCANopen.rtr = frame->content.RTR;
        msgInCANopen.dlc = frame->content.DLC;
    
        for(uint8_t i = 0; i < msgInCANopen.dlc; i++)
        {
            msgInCANopen.data[i] = frame->content.data[i];
        }
        //DEBUG
        CANopenTransmit(msgInCANopen);
        
    }
    /*
    motorStatus.sid = frame->content.SID;
    motorStatus.data[0] = frame->content.data[0];
    motorStatus.data[1] = frame->content.data[1];
    motorStatus.data[2] = frame->content.data[2];
    motorStatus.data[3] = frame->content.data[3];
    motorStatus.data[4] = frame->content.data[4];
    motorStatus.data[5] = frame->content.data[5];
    motorStatus.data[6] = frame->content.data[6];
    motorStatus.data[7] = frame->content.data[7];*/
}

void CAN_transmit(CANMessage_t msgOutCAN)
{
    struct can_instance_s* can = can_get_instance(0);               //???
    struct can_tx_frame_s* frame = can_allocate_tx_frames(can, 1);  //???
    frame->content.RTR=msgOutCAN.rtr;       //NOT remote transmission request                     
    frame->content.IDE=msgOutCAN.ide;       //NO identified extension bit (11-bit ID)
    frame->content.DLC=msgOutCAN.dlc;       //1 data byte    
    frame->content.SID=msgOutCAN.sid;        //CAN message identifier
    
    for(uint8_t i=0; i<8; i++)
    {
       frame->content.data[i] = msgOutCAN.data[i];
    }
    
    can_enqueue_tx_frames(can, &frame, TIME_INFINITE, NULL);
}

void CANopenTransmit(CANopenMessage_t msgOutCANopen)
{
    CANMessage_t msgOutCAN;
    msgOutCAN.sid = (msgOutCANopen.functionCode << 4) | msgOutCANopen.nodeID;
    msgOutCAN.ide = msgOutCANopen.ide;
    msgOutCAN.rtr = msgOutCANopen.rtr;
    msgOutCAN.dlc = msgOutCANopen.dlc;
    
    for(uint8_t i=0; i<8; i++)
    {
        msgOutCAN.data[i] = msgOutCANopen.data[i];
    }
    
    CAN_transmit(msgOutCAN);
}

/*
CANopenMessage_t CANopenReceive(CANMessage_t msgInCAN)
{
    //CANMessage_t msgInCAN;
    CANopenMessage_t msgInCANopen;
    
    msgInCANopen.functionCode = ((frame->content.SID >> 4)&0xF8);
    msgInCANopen.nodeID = (frame->content.SID & 0x7F);
    msgInCANopen.ide = frame->content.IDE;
    msgInCANopen.rtr = frame->content.RTR;
    msgInCANopen.dlc = frame->content.DLC;
    
    for(uint8_t i = 0; i < msgInCANopen.dlc; i++)
    {
        msgInCANopen.data[i] = frame->content.data[i];
    }
    
    return msgInCANopen;
}*/

//CANopenMessage_t nanotec::CANopenReceive()

void SDO_writeObject(uint8_t nodeID, uint16_t index, uint8_t subIndex, uint8_t arraySize, uint8_t data[])
{
    CANopenMessage_t msgOutCANopen;
    msgOutCANopen.functionCode = 0x60;      //0x60 indicates CANopen master is making SDO request; first half of communication object identifier (COB_ID) 
    msgOutCANopen.nodeID = nodeID;
    msgOutCANopen.ide = 0;
    msgOutCANopen.rtr = 0;
    msgOutCANopen.dlc = 8;//arraySize + 4;  //Because first 4 bytes are command, index, sub-index
    
    //Byte 0 Command: 0x40 to Read Dictionary Object
    
    switch(arraySize)
    {
        case 1:
            msgOutCANopen.data[0] = 0x2F;
            break;
        
        case 2:
            msgOutCANopen.data[0] = 0x2B;
            break;
            
        case 3:
            msgOutCANopen.data[0] = 0x27;
            break;
            
        case 4:
            msgOutCANopen.data[0] = 0x23;
            break;
    }
    
    msgOutCANopen.data[1] = index & 0xFF;   //LSB of index, Little Endian
    msgOutCANopen.data[2] = index >> 8;     //MSB of index
    msgOutCANopen.data[3] = subIndex;
    
    //Currently expects 4 bytes...make this conditional
    msgOutCANopen.data[4] = data[0];
    msgOutCANopen.data[5] = data[1];
    msgOutCANopen.data[6] = data[2];
    msgOutCANopen.data[7] = data[3];
    
    CANopenTransmit(msgOutCANopen);
}

    
void SDO_readObject(uint8_t nodeID, uint16_t index, uint8_t subIndex)
{
    CANopenMessage_t msgOutCANopen;
    msgOutCANopen.functionCode = 0x60;      //0x60 indicates CANopen master is making SDO request; first half of communication object identifier (COB_ID) 
    msgOutCANopen.nodeID = nodeID;
    msgOutCANopen.ide = 0;
    msgOutCANopen.rtr = 0;
    msgOutCANopen.dlc = 8;//arraySize + 4;  //Because first 4 bytes are command, index, sub-index
    
    //Byte 0 Command: 0x40 to Read Dictionary Object
    msgOutCANopen.data[0] = 0x40;
    msgOutCANopen.data[1] = index & 0xFF;   //LSB of index, Little Endian
    msgOutCANopen.data[2] = index >> 8;   //MSB of index
    msgOutCANopen.data[3] = subIndex;
    msgOutCANopen.data[4] = 0;
    msgOutCANopen.data[5] = 0;
    msgOutCANopen.data[6] = 0;
    msgOutCANopen.data[7] = 0;
    
    CANopenTransmit(msgOutCANopen);
}
    

//DIRECT MOTOR CONTROL
/*void PD4C_setup(uint8_t nodeID)
{
    uint8_t data[4];    //Declare 4-byte data array to be populated for each object
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    SDO_writeObject(nodeID, 0x6091, 0x01, 4, data);     //Motor Rev
    SDO_writeObject(nodeID, 0x6091, 0x02, 4, data);     //Shaft Rev    
    SDO_writeObject(nodeID, 0x6092, 0x01, 4, data);     //Feed (pitch)
    SDO_writeObject(nodeID, 0x6092, 0x02, 4, data);     //Drive Axis Revs (to achieve that feed)
    
    SDO_writeObject(nodeID, 0x320B, 0x03, 4, data);     //Velocity ACtual (0x6044 sourced from encoder value)
    
    data[0] = 4;
    SDO_writeObject(nodeID, 0x6060, 0x00, 4, data);     //Set to torque mode
    data[0] = 100;
    SDO_writeObject(nodeID, 0x6071, 0x00, 4, data);     //Set target torque to 10% (this will increase to 40% when action is called)
    data[0] = 500;
    SDO_writeObject(nodeID, 0x6072, 0x00, 4, data);     //Set max torque to 50%
    data[0] = 100;
    SDO_writeObject(nodeID, 0x6087, 0x00, 4, data);     //Set torque slop rise to 10%/sec
    data[0] = 1;
    SDO_writeObject(nodeID, 0x3202, 0x00, 4, data);     //Set to Closed-Loop mode (0x21 no max velocity)
    
    data[0] = 0x12C;
    SDO_writeObject(nodeID, 0x2032, 0x00, 4, data);     //0x12C - 300rpm (this is subject to ramp up/down)
    
    //Velocity Dimensions
    data[0] = 0x01;
    SDO_writeObject(nodeID, 0x604C, 0x01, 4, data);     //Numerator - 1 rev
    data[0] = 0x3C;
    SDO_writeObject(nodeID, 0x604C, 0x02, 4, data);     //Denominator - 60 sec
    
    //Current Parameters
    data[0] = 0x157C;
    SDO_writeObject(nodeID, 0x2031, 0x00, 4, data);     //Set maximum current (0x157C - 5500 mA)
    data[0] = 0x0FA0;
    SDO_writeObject(nodeID, 0x203B, 0x01, 4, data);     //Set nominal current (0xFA0 - 4000mA)    
    data[0] = 0x3E8;
    SDO_writeObject(nodeID, 0x203B, 0x02, 4, data);     //Max duration of peak current (0x3E8 - 1000ms)
    
    //General Parameters
    data[0] = 0x2D0;
    SDO_writeObject(nodeID, 0x608F, 0x01, 4, data);     //Postion resolution (0x2D0 - 720 steps/rev) 
    data[0] = 0x01;
    SDO_writeObject(nodeID, 0x608F, 0x02, 4, data);     //1 encoder rev per motor turn   
    data[0] = 0xC0;
    SDO_writeObject(nodeID, 0x607E, 0x00, 4, data);     //Polartiy
}*/
    

void PD4C_openPetal(uint8_t nodeID)
{
    uint8_t data[4];
    data[0] = 0x70;
    data[1] = 0xFE;
    data[2] = 0;
    data[3] = 0;
    SDO_writeObject(nodeID, 0x6071, 0x00, 2, data);     //Set target torque to 40%, negative indicated opening direction
    
    data[0] = 0x0C;
    data[1] = 0xFE;
    SDO_writeObject(nodeID, 0x6072, 0x00, 2, data);     //Set max torque to 50%
    
    //stateMachine(void);
}

void PD4C_closePetal(uint8_t nodeID)
{
    uint8_t data[4];
    data[0] = 0x90;
    data[1] = 0x01;
    data[2] = 0;
    data[3] = 0;
    SDO_writeObject(nodeID, 0x6071, 0x00, 2, data);     //Set target torque to 40%, negative indicated opening direction
    
    data[0] = 0xF4;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x6072, 0x00, 2, data);     //Set max torque to 50%
    
    //stateMachine(void);
}

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    