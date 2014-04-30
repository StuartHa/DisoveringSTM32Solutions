#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <stdlib.h>
#include "spi.h"

#define WIP(x) ((x) & 1)

static const uint16_t speeds[] = {
  [SPI_SLOW] = SPI_BaudRatePrescaler_64,
  [SPI_MEDIUM] = SPI_BaudRatePrescaler_8,
  [SPI_FAST] = SPI_BaudRatePrescaler_2};

static const uint16_t EEPROM_SPEED = SPI_SLOW;
static GPIO_TypeDef *EEPROM_PORT = GPIOC;
static const uint16_t EEPROM_CS = GPIO_Pin_3;

enum eepromCMD {
  cmdWRSR = 0x01,
  cmdWRITE = 0x02,
  cmdREAD = 0x03,
  cmdWRDI = 0x04,
  cmdRDSR = 0x05,
  cmdWREN = 0x06
};

uint8_t eepromReadStatus(void) {
  uint8_t cmd[] = {cmdRDSR, 0xff};
  uint8_t res[2];
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, res, cmd, 2, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
  return res[1];
}

void eepromWriteEnable(void) {
  while (WIP(eepromReadStatus()));

  uint8_t cmd = cmdWREN;
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, NULL, &cmd, 1, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
}

void eepromWriteDisable(void) {
  while (WIP(eepromReadStatus()));

  uint8_t cmd = cmdWRDI;
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, NULL, &cmd, 1, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
}

void eepromWriteStatus(uint8_t status) {
  while (WIP(eepromReadStatus()));

  uint8_t cmd[] = {cmdWRSR, status};
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, NULL, cmd, 2, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
}

static uint8_t prv_writable_bytes(uint16_t offset) {
  return 32 - (offset % 32);
}

void eepromWrite(uint8_t *buf, uint8_t cnt, uint16_t offset) {
  while (WIP(eepromReadStatus()));
  
  uint8_t writable_bytes = prv_writable_bytes(offset);
  if (cnt > writable_bytes) {
    cnt = writable_bytes;
  } 

  uint8_t cmd[3 + cnt];
  cmd[0] = cmdWRITE;
  cmd[1] = (offset >> 8) & 0xff;
  cmd[2] = offset & 0xff;
  for (int i = 0; i < cnt; i++) {
    cmd[3 + i] = buf[i];
  }

  eepromWriteEnable();
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, NULL, cmd, 3 + cnt, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
}

void eepromRead(uint8_t *buf, uint8_t cnt, uint16_t offset) {
  while (WIP(eepromReadStatus()));

  uint8_t cmd[3 + cnt];
  cmd[0] = cmdREAD;
  cmd[1] = offset & 0xff00;
  cmd[2] = offset & 0x00ff;

  uint8_t rxbuf[cnt + 3];
  eepromWriteEnable();
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, rxbuf, cmd, 3 + cnt, EEPROM_SPEED);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
  
  for (int i = 0; i < cnt; i++) {
    buf[i] = rxbuf[3 + i];
  } 
}

uint8_t eepromReadStatus_test_later(void) {
  uint8_t RDSR = 0x05;
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 0);
  spiReadWrite(SPI2, NULL, &RDSR, 1, SPI_SLOW);
  uint8_t eeprom_status;
  spiReadWrite(SPI2, &eeprom_status, NULL, 1, SPI_SLOW);
  GPIO_WriteBit(EEPROM_PORT, EEPROM_CS, 1);
  return eeprom_status;
}

void spiInit(SPI_TypeDef *SPIx)
{
  GPIO_InitTypeDef GPIO_InitStructureSCK;
  GPIO_InitTypeDef GPIO_InitStructureMISO;
  GPIO_InitTypeDef GPIO_InitStructureMOSI;
  SPI_InitTypeDef SPI_InitStructure;

  GPIO_StructInit(&GPIO_InitStructureSCK);
  GPIO_StructInit(&GPIO_InitStructureMISO);
  GPIO_StructInit(&GPIO_InitStructureMOSI);
  SPI_StructInit(&SPI_InitStructure);

  if (SPIx == SPI2) {  
    /* Enable clocks, configure pins */
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    // Configure pins
    GPIO_InitStructureSCK.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructureSCK.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureSCK.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructureSCK);

    GPIO_InitStructureMISO.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructureMISO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructureMISO);

    GPIO_InitStructureMOSI.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructureMOSI.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureMOSI.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructureMOSI);
   }  else  {  //  other SPI devices --
      return;
  }

  // Configure device
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = speeds[SPI_SLOW];
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPIx, &SPI_InitStructure);

  SPI_Cmd(SPIx, ENABLE);   
}

int spiReadWrite(SPI_TypeDef* SPIx, uint8_t *rbuf, 
         const uint8_t *tbuf, int cnt, enum spiSpeed speed)
{
  int i;

  SPIx->CR1 = (SPIx->CR1 & ~SPI_BaudRatePrescaler_256) | 
               speeds[speed];

  for (i = 0; i < cnt; i++){
    if (tbuf) {
      SPI_I2S_SendData(SPIx, *tbuf++);
    } else {
      SPI_I2S_SendData(SPIx, 0xff);
    }
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    if (rbuf) {
      *rbuf++ = SPI_I2S_ReceiveData(SPIx);
    } else {
      SPI_I2S_ReceiveData(SPIx);
    }
  }
  return i;
}

int spiReadWrite16(SPI_TypeDef* SPIx, uint16_t *rbuf, 
     const uint16_t *tbuf, int cnt, enum spiSpeed speed)
{
  SPI_DataSizeConfig(SPIx, SPI_DataSize_16b);
  int i;

  SPIx->CR1 = (SPIx->CR1 & ~SPI_BaudRatePrescaler_256) | 
               speeds[speed];

  for (i = 0; i < cnt; i++){
    if (tbuf) {
      SPI_I2S_SendData(SPIx, *tbuf++);
    } else {
      SPI_I2S_SendData(SPIx, 0xffff);
    }
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    if (rbuf) {
      *rbuf++ = SPI_I2S_ReceiveData(SPIx);
    } else {
      SPI_I2S_ReceiveData(SPIx);
    }
  }
  SPI_DataSizeConfig(SPIx, SPI_DataSize_8b);
  return i;
}

