#pragma once

#include <stdint.h>
#include <modules/platform_stm32f302x8/platform_stm32f302x8.h>

#define BOARD_PAL_LINE_LED1 PAL_LINE(GPIOA,6)       //On-Board LED
#define BOARD_PAL_LINE_LED2 PAL_LINE(GPIOA,7)       //On-Board LED


#define BOARD_PAL_LINE_USART1_TX PAL_LINE(GPIOA,9)
#define BOARD_PAL_LINE_USART1_RX PAL_LINE(GPIOA,10)

#define BOARD_PAL_LINE_CAN_RX PAL_LINE(GPIOA,11)
#define BOARD_PAL_LINE_CAN_TX PAL_LINE(GPIOA,12)


//

#define BOARD_PAL_LINE_PIN9 PAL_LINE(GPIOA,2)       //White LED
#define BOARD_PAL_LINE_PIN10 PAL_LINE(GPIOA,3)      //Red LED

#define BOARD_PAL_LINE_PIN3 PAL_LINE(GPIOB,3)       //Beacon