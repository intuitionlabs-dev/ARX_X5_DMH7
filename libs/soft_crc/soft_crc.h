#ifndef _SOFT_CRC_H_
#define _SOFT_CRC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 不需要初始化：查找表是编译期的 const，直接调就行。
 *
 * 一次算完一整块：
 *   uint16_t crc = CRC16_Modbus_calc(buf, len);
 */
uint16_t CRC16_Modbus_calc(const uint8_t* data, uint32_t len);  //NOLINT
uint8_t CRC8_Modbus_calc(const uint8_t* data, uint32_t len);    //NOLINT

#ifdef __cplusplus
}
#endif

#endif
