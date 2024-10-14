#ifndef MAX30003_H_
#define MAX30003_H_

#define DT_DRV_COMPAT ti_ads1299
#include <zephyr/device.h>


#define REG_ID 0x00
#define REG_CONFIG1 0x01
#define REG_CONFIG2 0x02
#define REG_CONFIG3 0x03
#define REG_LOFF 0x04
#define REG_CH1SET 0x05
#define REG_CH2SET 0x06
#define REG_CH3SET 0x07
#define REG_CH4SET 0x08
#define REG_CH5SET 0x09
#define REG_CH6SET 0x0A
#define REG_CH7SET 0x0B
#define REG_CH8SET 0x0C
#define REG_BIAS_SENSP 0x0D
#define REG_BIAS_SENSN 0x0E
#define REG_LOFF_SENSP 0x0F
#define REG_LOFF_SENSN 0x10
#define REG_LOFF_FLIP 0x11
#define REG_LOFF_STATP 0x12
#define REG_LOFF_STATN 0x13
#define REG_GPIO 0x14
#define REG_MISC1 0x15
#define REG_MISC2 0x16
#define REG_CONFIG4 0x17



struct ads1299_data
{
    uint8_t chip_id;

};

// #include <syscalls/ads1299.h>

#endif