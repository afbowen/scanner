#include <common/ctor.h>
#include <modules/uavcan/uavcan.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/timing/timing.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <com.matternet.equipment.scanner.BarcodeContent.h>
#include <modules/nanotec_controller/nanotec.h>
#include <string.h>

#if __GNUC__ != 6 || __GNUC_MINOR__ != 3 || __GNUC_PATCHLEVEL__ != 1
#error Please use arm-none-eabi-gcc 6.3.1.
#endif

#include <hal.h>

WORKER_THREAD_SPAWN(publish_thread, HIGHPRIO, 1024)

static struct worker_thread_timer_task_s my_task;                   //Create instance of timer_task struct called my_task
static void my_task_func(struct worker_thread_timer_task_s* task);  //Declare task function for worker thread

//UART Scanner Config
static void uart_char_recv(UARTDriver* uartp, uint16_t c);              //(ChibiOS) function called to handle received character (we name and define below)
static UARTConfig uart1conf = {.speed=9600, .rxchar_cb=uart_char_recv}; //Configure UART1: set baud to 9600, set callback (upon char received) to uart_char_recv

static uint32_t char_recv_ms;       //Time character received
static char scanned_code[255];      //256-byte buffer for QR string
static size_t scanned_code_len;     //Length of scanned QR string

extern CANMessage_t motorStatus;
extern uint8_t masterCommand;


typedef enum
{
    closed  = 0,
    opening = 1,
    open    = 2,
    closing = 3,
    setup   = 4
}   stationState_t;

stationState_t currentState;
bool active;

//NEW
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
bool beaconOn;
bool redLED;
bool whiteLED;
//TERM NEW

RUN_BEFORE(INIT_END)        //Is this framework?
{
    //UART Scanner Config
    uartStart(&UARTD1, &uart1conf);     //Start UART Driver 1, point to configuration                                                                                                                          
    worker_thread_add_timer_task(&publish_thread, &my_task, my_task_func, NULL, LL_MS2ST(10), true);
}

static void my_task_func(struct worker_thread_timer_task_s* task) 
{
    (void)task;

    //Publish Scanner Message
    chSysLock();    //Disable interrupts
    if (millis()-char_recv_ms > 100 && scanned_code_len > 0)
    {   
        //Scanner Handler
        struct com_matternet_equipment_scanner_BarcodeContent_s barcode_msg;    //DSDL stuct instance
        barcode_msg.content_string_len = scanned_code_len > sizeof(barcode_msg.content_string) ? sizeof(barcode_msg.content_string) : scanned_code_len; //If too long, clip size
        memcpy(barcode_msg.content_string, scanned_code, barcode_msg.content_string_len);   //copy scanned code to content_string
        scanned_code_len = 0;       //Reset scanned code length to zero once we've read and stored content
        chSysUnlock();  //Re-enable interrupts

        //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", scanned_code_copy);
        uavcan_broadcast(0, &com_matternet_equipment_scanner_BarcodeContent_descriptor, CANARD_TRANSFER_PRIORITY_LOW, &barcode_msg);

        //Blink LED to indicate message sent
        palSetLine(BOARD_PAL_LINE_LED1);
        chThdSleepMilliseconds(20);
        palClearLine(BOARD_PAL_LINE_LED1);
        
        //Nanotec Control
        /*
        uint8_t data[4];
        data[0] = 0x02;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        SDO_writeObject(0x03, 0x2400, 0x01, 4, data);*/
    }
    else 
    {
        chSysUnlock();
    }
    
    if(masterCommand)
    {
        CANMessage_t rebound;
        rebound.sid = 0x10;
        rebound.ide = 0;
        rebound.rtr = 0;
        rebound.dlc = 8;
        rebound.data[0] = masterCommand;
        
        for(uint8_t i = 7; i > 0; i--)
        {
            rebound.data[i] = i;
        }
        CAN_transmit(rebound);
        
    }
    
    switch (masterCommand)
    {
        case 0x02:
            currentState = opening;
            if(!active)
            {
                /*uint8_t data[4];
                data[0] = 0x02;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                SDO_writeObject(0x03, 0x2400, 0x01, 4, data);*/
                PD4C_stateMachine(0x03);
                active = 1;
            }
            break;
            
        case 0x03:
            currentState = closing;
            if(!active)
            {
                /*uint8_t data[4];
                data[0] = 0x03;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                SDO_writeObject(0x03, 0x2400, 0x01, 4, data);*/
                PD4C_stateMachine(0x03);
                active = 1;
            }
            break;
            
        case 0x01:
            currentState = closed;
            if(active)
            {
                /*uint8_t data[4];
                data[0] = 0x01;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                SDO_writeObject(0x03, 0x2400, 0x01, 4, data);*/
                PD4C_stateMachine(0x03);
                active = 0;
            }
            break;
            
        case 0x04:
            PD4C_setup(0x03);
            break;
        
        default:
            break;
    }
    masterCommand = 0;
       
       
       
    if(currentState == closed)
    {
        palClearLine(BOARD_PAL_LINE_PIN10);
        palSetLine(BOARD_PAL_LINE_PIN9);    //set white LEDs
    }
    else
    {
        palClearLine(BOARD_PAL_LINE_PIN9);
        palSetLine(BOARD_PAL_LINE_PIN10);
    }
    
    
            
    //palSetLine(BOARD_PAL_LINE_PIN10);
    //palClearLine(BOARD_PAL_LINE_PIN3);

    
    /*
    //Toggle Beacon 
    if(!beaconOn)
    {
        beaconOn = !beaconOn;
        palSetLine(BOARD_PAL_LINE_PIN3);
    }
    else
    {
        beaconOn = !beaconOn;
        palClearLine(BOARD_PAL_LINE_PIN3);
    }
    
    //Change LEDs
    if(!redLED)
    {
        redLED = !redLED;
        palSetLine(BOARD_PAL_LINE_PIN10);
        palClearLine(BOARD_PAL_LINE_PIN9);
    }
    else
    {
        redLED = !redLED;
        palSetLine(BOARD_PAL_LINE_PIN9);
        palClearLine(BOARD_PAL_LINE_PIN10);
    }
    */
    

    
    //START
    /*
    SDO_readObject(0x03, 0x6041, 0x00);
    chThdSleepMilliseconds(5);
    uint32_t msgSID = motorStatus.sid;
    uint8_t exData[4];
    exData[0] = motorStatus.data[4];
    exData[1] = motorStatus.data[5];
    exData[2] = motorStatus.data[6];
    exData[3] = motorStatus.data[7];
    SDO_writeObject(0x00, msgSID, motorStatus.data[0], 4, exData);*/
    
    //PD4C_setup(0x03);
    //PD4C_openPetal(0x03);
    //chThdSleepMilliseconds(100);
    //PD4C_stateMachine(0x03);
    //SDO_readObject(0x03, 0x6041, 0x00); 
    
    //Check Station State
}

//Serial Event
static void uart_char_recv(UARTDriver* uartp, uint16_t c)       //Called when char received
{
    (void)uartp;
    if (scanned_code_len < 255)
    {
        char_recv_ms = millis();                //Timestamps character
        scanned_code[scanned_code_len++] = c;   //Add character to array
        palToggleLine(BOARD_PAL_LINE_LED1);     //Toggle LED line
    }
}


//DIRECT MOTOR CONTROL
void PD4C_setup(uint8_t nodeID)
{
    uint8_t data[4];    //Declare 4-byte data array to be populated for each object
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    SDO_writeObject(nodeID, 0x6091, 0x01, 4, data);     //Motor Rev
    chThdSleepMilliseconds(3);
    SDO_writeObject(nodeID, 0x6091, 0x02, 4, data);     //Shaft Rev
    chThdSleepMilliseconds(3);    
    SDO_writeObject(nodeID, 0x6092, 0x01, 4, data);     //Feed (pitch)
    chThdSleepMilliseconds(3);
    SDO_writeObject(nodeID, 0x6092, 0x02, 4, data);     //Drive Axis Revs (to achieve that feed)
    chThdSleepMilliseconds(3);
    
    SDO_writeObject(nodeID, 0x320B, 0x03, 4, data);     //Velocity ACtual (0x6044 sourced from encoder value)
    chThdSleepMilliseconds(3);
    
    data[0] = 4;
    SDO_writeObject(nodeID, 0x6060, 0x00, 1, data);     //Set to torque mode
    chThdSleepMilliseconds(3);
    data[0] = 0x90;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x6071, 0x00, 2, data);     //Set target torque to 10% (this will increase to 40% when action is called)
    chThdSleepMilliseconds(3);
    data[0] = 0xF4;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x6072, 0x00, 2, data);     //Set max torque to 50%
    data[1] = 0x00;
    chThdSleepMilliseconds(3);
    data[0] = 100;
    SDO_writeObject(nodeID, 0x6087, 0x00, 4, data);     //Set torque slop rise to 10%/sec
    chThdSleepMilliseconds(3);
    data[0] = 1;
    SDO_writeObject(nodeID, 0x3202, 0x00, 4, data);     //Set to Closed-Loop mode (0x21 no max velocity)
    chThdSleepMilliseconds(3);
    
    data[0] = 0x2C;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x2032, 0x00, 4, data);     //0x12C - 300rpm (this is subject to ramp up/down)
    chThdSleepMilliseconds(3);
    
    //Velocity Dimensions
    data[0] = 0x01;
    SDO_writeObject(nodeID, 0x604C, 0x01, 4, data);     //Numerator - 1 rev
    chThdSleepMilliseconds(3);
    data[0] = 0x3C;
    SDO_writeObject(nodeID, 0x604C, 0x02, 4, data);     //Denominator - 60 sec
    chThdSleepMilliseconds(3);
    
    //Current Parameters
    data[0] = 0x7C;
    data[1] = 0x15;
    SDO_writeObject(nodeID, 0x2031, 0x00, 4, data);     //Set maximum current (0x157C - 5500 mA)
    chThdSleepMilliseconds(3);
    data[0] = 0xA0;
    data[1] = 0x0F;
    SDO_writeObject(nodeID, 0x203B, 0x01, 4, data);     //Set nominal current (0xFA0 - 4000mA)  
    chThdSleepMilliseconds(3);
    data[0] = 0xE8;
    data[1] = 0x03;
    SDO_writeObject(nodeID, 0x203B, 0x02, 4, data);     //Max duration of peak current (0x3E8 - 1000ms)
    chThdSleepMilliseconds(3);
    
    //General Parameters
    data[0] = 0xD0;
    data[1] = 0x02;
    SDO_writeObject(nodeID, 0x608F, 0x01, 4, data);     //Postion resolution (0x2D0 - 720 steps/rev) 
    chThdSleepMilliseconds(3);
    data[0] = 0x01;
    data[1] = 0x00;
    SDO_writeObject(nodeID, 0x608F, 0x02, 4, data);     //1 encoder rev per motor turn  
    chThdSleepMilliseconds(3);
    //data[0] = 0xC0;
    //SDO_writeObject(nodeID, 0x607E, 0x00, 4, data);     //Polartiy
}

void PD4C_stateMachine(uint8_t nodeID)
{
    uint8_t data[4];
    data[0] = 0x06;
    data[1] = 0x0;
    data[2] = 0x0;
    data[3] = 0x0;
    SDO_writeObject(nodeID, 0x6040, 0x00, 2, data);
    chThdSleepMilliseconds(3);
    SDO_readObject(nodeID, 0x6041, 0x00);
    chThdSleepMilliseconds(3);
    
    data[0] = 0x07;
    SDO_writeObject(nodeID, 0x6040, 0x00, 2, data);
    chThdSleepMilliseconds(3);
    SDO_readObject(nodeID, 0x6041, 0x00);
    chThdSleepMilliseconds(3);
    
    data[0] = 0x0F;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x6040, 0x00, 2, data);
    chThdSleepMilliseconds(3);
    SDO_readObject(nodeID, 0x6041, 0x00);
    chThdSleepMilliseconds(3);
    
    data[0] = 0x0F;
    data[1] = 0x01;
    SDO_writeObject(nodeID, 0x6040, 0x00, 2, data);
    chThdSleepMilliseconds(3);
    SDO_readObject(nodeID, 0x6041, 0x00);
    chThdSleepMilliseconds(3);
    
    data[0] = 0x0F;
    data[1] = 0;
    SDO_writeObject(nodeID, 0x6040, 0x00, 2, data);
    //chThdSleepMilliseconds(3);
    //data[0] = 0x02;
    //SDO_writeObject(nodeID, 0x2400, 0x01, 4, data);
    
}
    
    
    
    