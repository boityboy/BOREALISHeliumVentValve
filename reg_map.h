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
char BMECMD[]={0x74, 0x48, 0x72, 0x02, 0x75, 0xC, 0x74, 0x49, 0xEA,
               0xE9, 0x8B, 0x8A, 0x8C, 0x8F, 0x8E, 0x91, 0x90, 0x92,
               0x95, 0x94, 0x97, 0x96, 0x99, 0x98, 0x9D, 0x9C, 0x9F,
               0x9E, 0xA0, 0xE3, 0xE2, 0xE1, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8};

#endif /* REG_MAP_H */

/*** end of file ***/
