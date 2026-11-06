#ifndef _CVECTOR_H
#define _CVECTOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MIN_LEN 5      /* cvector最小长度 */
#define EXPANED_VAL 1  /* cvector最小扩张倍数 */

#pragma pack(1)

typedef struct cvector_t {
    void* cv_pdata;     /* 指向数组所申请的内存 */
    size_t cv_len;      /* 数组中元素个数 */
    size_t cv_size;     /* 单个元素大小(字节) */
    size_t cv_tot_len;  /* 数组总大小(字节) */
} cvector;
#pragma pack()

cvector* cvector_create(const size_t size);
void cvector_destroy(cvector* cv);
size_t cvector_length(cvector* cv);
void* cvector_pushback(cvector* cv, void* memb);
void* cvector_val_at(cvector* cv, size_t index);

#endif
