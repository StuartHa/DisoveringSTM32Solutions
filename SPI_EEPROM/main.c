#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include "spi.h"

uint8_t txbuf[4], rxbuf[4]; 

void csInit(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void main() {
  int i, j;
  csInit(); // Initialize chip select PC03 
  spiInit(SPI2);
    for (j = 0; j < 4; j++) {
      txbuf[j] = j + 5;
    }


  __IO uint8_t new_buf[4];
  eepromRead(new_buf, 4, 32);
}
