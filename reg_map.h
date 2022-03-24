/** @file reg_map.h
 * 
 * @brief the register mapping for the RF95 LoRa Radio. 
 *
 */ 

#ifndef REG_MAP_H
#define REG_MAP_H

// Bits
//SD Card
#define CS    BIT0
#define CLK   BIT1
#define SOMI  BIT2
#define SIMO  BIT3
//GPS
#define RX    BIT6
#define TX    BIT7
//Sensor
#define SDA   BIT2
#define SCL   BIT3
//IO
#define ALG   BIT0
#define XB    BIT1

// Registers


// Constants
char BMECMD[]={0x74, 0x48, 0x72, 0x02, 0x75, 0xC, 0x74, 0x49};

#endif /* REG_MAP_H */

/*** end of file ***/
