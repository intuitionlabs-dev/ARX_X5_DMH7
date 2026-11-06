/* C语言实现动态数组 */
#include "cvector.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-arith"
/* size: 数组成员的大小 */
cvector* cvector_create(const size_t size) {
    cvector* cv = (cvector*)malloc(sizeof(cvector));

    if (!cv) return NULL;

    cv->cv_pdata = malloc(MIN_LEN * size);

    if (!cv->cv_pdata) {
        free(cv);
        return NULL;
    }

    cv->cv_size = size;
    cv->cv_tot_len = MIN_LEN;
    cv->cv_len = 0;

    return cv;
}

/* 删除动态数组 */
void cvector_destroy(cvector* cv) {
    free(cv->cv_pdata);
    free(cv);
    return;
}

size_t cvector_length(cvector* cv) { return cv->cv_len; }

/* 向动态数组中添加元素，并返回指向该元素的指针 */
void* cvector_pushback(cvector* cv, void* memb) {
    if (cv->cv_len >= cv->cv_tot_len) {
        /* 以cv_tot_len为最小单位进行扩张，避免反复realloc */
        cv->cv_tot_len <<= EXPANED_VAL;
        cv->cv_pdata = realloc(cv->cv_pdata, cv->cv_tot_len * cv->cv_size);
    }

    memcpy((char*)cv->cv_pdata + cv->cv_len * cv->cv_size, memb, cv->cv_size);
    cv->cv_len++;

    return cv->cv_pdata + (cv->cv_len - 1) * cv->cv_size;
}

void* cvector_val_at(cvector* cv, size_t index) {
    return cv->cv_pdata + index * cv->cv_size;
}

#pragma GCC diagnostic pop
