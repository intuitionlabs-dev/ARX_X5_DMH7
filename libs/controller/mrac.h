#ifndef _MRAC_H_
#define _MRAC_H_

#pragma pack(1)
typedef struct {
    float x1;
    float x2;
    float x;
    float r;
    float h;
    float aim;
    float dt;
} MTD_t;

typedef struct Mrac_config_t{
    float k1;
    float k2;
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
    float output_max;
    float integrator_ki;
    float integrator_sum_err_max;
}Mrac_config;

typedef struct mrac_2d_t{
    Mrac_config config;
    float x1_ref;
    float x1_fdb;
    float x2_ref;
    float x2_fdb;
    float x1_err;
    float x2_err;
    float gamma1;
    float gamma2;
    float gamma3;
    float gamma4;
    float output;
    float dt;
    MTD_t x1_td;
    MTD_t x2_td;

    float integrator_sum_err;
} mrac_2d;

#pragma pack()

void mrac_Init(mrac_2d* mrac,Mrac_config* config);
void mrac_2d_calc(mrac_2d *mrac, float ref, float x1_fdb, float x2_fdb, unsigned char enable);
#endif
