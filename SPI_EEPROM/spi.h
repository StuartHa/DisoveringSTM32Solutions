#ifndef SPI_H
#define SPI_H

enum spiSpeed { SPI_SLOW , SPI_MEDIUM, SPI_FAST };

void spiInit(SPI_TypeDef* SPIx);
int spiReadWrite(SPI_TypeDef* SPIx, uint8_t *rbuf, 
		 const uint8_t *tbuf, int cnt, 
		 enum spiSpeed speed);
int spiReadWrite16(SPI_TypeDef* SPIx, uint16_t *rbuf, 
                   const uint16_t *tbuf, int cnt, 
                   enum spiSpeed speed);
uint8_t eepromReadStatus(void);
void eepromWriteEnable(void);
void eepromWriteDisable(void);
void eepromWriteStatus(uint8_t status);
void eepromWrite(uint8_t *buf, uint8_t cnt, uint16_t offset);
void eepromRead(uint8_t *buf, uint8_t cnt, uint16_t offset);
#endif
