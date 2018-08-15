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

static CANMessage_t motorStatus;
static uint8_t masterCommand;
static uint16_t setpointRPM;
static int16_t velocityActual;


//CANopen Variables
static bool SDO_downloadConfirmed;         //Cleared upon write, set upon write confirmation
static CANopenError_t SDO_downloadError;   //Populate when error raised
static uint8_t init_objectCounter;         //Increments for each object written during the init process

#define NUM_PD4_PARAMS (sizeof(PD4E_init_param)/sizeof(PD4E_init_param[0]))
static const dictionaryObject_t PD4E_init_param[] = {
    {0x6091, 0x01, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x6091, 0x02, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x6092, 0x01, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x6092, 0x02, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x6060, 0x00, 0x01, {0x04, 0x00, 0x00, 0x00}},
    {0x6071, 0x00, 0x02, {0x90, 0x01, 0x00, 0x00}},
    {0x6072, 0x00, 0x02, {0xF4, 0x01, 0x00, 0x00}},
    {0x6087, 0x00, 0x04, {0xF4, 0x01, 0x00, 0x00}},
    {0x3202, 0x00, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x604C, 0x01, 0x04, {0x01, 0x00, 0x00, 0x00}},
    {0x604C, 0x02, 0x04, {0x3C, 0x00, 0x00, 0x00}},
    {0x6080, 0x00, 0x04, {0x2C, 0x01, 0x00, 0x00}},
    {0x203B, 0x01, 0x04, {0xA0, 0x0F, 0x00, 0x00}}
};




RUN_BEFORE(INIT_END) {
    struct can_instance_s* can = can_get_instance(0);
    struct pubsub_topic_s* can_rx_topic = can_get_rx_topic(can);
    if (!can_rx_topic) return;
    worker_thread_add_listener_task(&WT, &rx_listener_task, can_rx_topic, nanotec_can_rx_handler, NULL);
    
    SDO_downloadConfirmed = 0; 
    SDO_downloadError = {0x0000, 0x00, 0x00, {0x00, 0x00, 0x00, 0x00}};
    init_objectCounter = 0;
    
}

static void nanotec_can_rx_handler(size_t msg_size, const void* msg, void* ctx) 
{
    (void) msg_size;
    const struct can_rx_frame_s* frame = msg;
    
    
    //Check if it's a command from Particle (11-bit SID = 0x30)
    if((frame->content.IDE == 0) && ((frame->content.SID == 0x30)))
    {
        masterCommand = frame->content.data[0];
        if(masterCommand == 0x05)
        {
            setpointRPM = ((frame->content.data[2] << 8) | (frame->content.data[1]));
        }
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
    
    //Check if it's a CANopen SDO reply frame
    else if((frame->content.IDE == 0) && (((frame->content.SID >> 4) & 0xF8) == 0x58))   //If not extended ID and function code is 0x58, this is CANopen message
    {
        CANopenMessage_t msgInCANopen;
        
        //Check node ID (we know this is an SDO response, no need to check function code, IDE, RTR)
        msgInCANopen.nodeID = (frame->content.SID & 0x7F);    
        msgInCANopen.dlc = frame->content.DLC;
    
        //Unnecessary...but keep for now 
        for(uint8_t i = 0; i < msgInCANopen.dlc; i++)
        {
            msgInCANopen.data[i] = frame->content.data[i];
        }
        
        //Check if Download SDO_downloadConfirmed
        if(msgInCANopen.data[0] == 0x60)
        {
            SDO_downloadConfirmed = 1;  //Set download confirm bit
        }
        
        //Check if there's been a download error
        else if(msgInCANopen.data[0] == 0x80)
        {
            SDO_downloadError.errorFlag = 1;  //Set error flag
            SDO_downloadError.index = ((msgInCANopen.data[2] << 8) | (msgInCANopen.data[1]));
            SDO_downloadError.subIndex = msgInCANopen.data[3];
            SDO_downloadError.errorCode = ((msgInCANopen.data[7] << 24) | (msgInCANopen.data[6] << 16) | (msgInCANopen.data[5] << 8) | (msgInCANopen.data[4] << 24));
        }
            
            
        
        
        if(msgInCANopen.data[1] == 0x44)
        {
            velocityActual = ((msgInCANopen.data[5] << 8) | msgInCANopen.data[4]);
            msgInCANopen.data[6] = (velocityActual >> 8);
            msgInCANopen.data[7] = (velocityActual & 0xFF);
        }
            
        //DEBUG
        CANopenTransmit(msgInCANopen);
        
    }

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

void SDO_writeInitParam(uint8_t nodeID, dictionaryObject_t param)
{
    CANopenMessage_t msgOutCANopen;
    msgOutCANopen.functionCode = 0x60;      //0x60 indicates CANopen master is making SDO request; first half of communication object identifier (COB_ID) 
    msgOutCANopen.nodeID = nodeID;
    msgOutCANopen.ide = 0;
    msgOutCANopen.rtr = 0;
    msgOutCANopen.dlc = 8;
    
    switch(param.arraySize)
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
    
    msgOutCANopen.data[1] = param.index & 0xFF;   //LSB of index, Little Endian
    msgOutCANopen.data[2] = param.index >> 8;     //MSB of index
    msgOutCANopen.data[3] = param.subIndex;
    
    //Currently expects 4 bytes...make this conditional
    msgOutCANopen.data[4] = param.data[0];
    msgOutCANopen.data[5] = param.data[1];
    msgOutCANopen.data[6] = param.data[2];
    msgOutCANopen.data[7] = param.data[3];
    
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
void PD4E_init(uint8_t nodeID) {
    for (size_t i = 0; i < NUM_PD4_PARAMS; i++) {
        SDO_writeObject(nodeID, PD4E_init_param[i]);
        SDO_downloadConfirmed = 0;
        while (SDO_downloadConfirmed != 1);
        {
            //Do nothing...
        }
    }
    
}
    

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

//CANopenMessage_t nanotec::CANopenReceive(
