#ifndef _SOFT_CRC_H_
#define _SOFT_CRC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 查找表在编译期生成，可直接计算整段数据的 CRC。 */
uint16_t CRC16_Modbus_calc(const uint8_t* data, uint32_t len);  //NOLINT
uint8_t CRC8_Modbus_calc(const uint8_t* data, uint32_t len);    //NOLINT

#ifdef __cplusplus
}
#endif

#endif
