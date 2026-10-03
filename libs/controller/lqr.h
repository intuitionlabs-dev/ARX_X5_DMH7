#ifndef _LQR_H_
#define _LQR_H_

/*
yaw:
x   = [θ ω]^T
·x  = [0 1; 0 -b/J]x + [0; 1/J]u
u   = -kx + s
θ   表示角度
ω   表示角度相对时间一阶导数
b   表示电机的阻力，默认 f阻 = k*ω
k   = [k1 k2]由lqr确定
s   为积分项
u   为转矩, 对于6020有 u = 0.741*i(国际单位制) 对于电流与can信号,有 y = 1111i - 236(通过yaw轴测得,y为传入信号,i单位为A)
            对于2006有 u = 0.18*i(国际单位制)  
            对于3508有 u = 0.3*i

J = SW图中可测,数量级大致符合即可, b = 0.01(影响可忽略不计)
Q = diag[40000 100], R = 1
matlab 可得k = [k1 ，k2] 
*/

/*
pitch:
x   = [θ ω]^T
·x  = [0 1; m*g*L*sin(θ)/J -b/J]x + [0; 1/J]u
m   表示云台质量
L   表示质心距离转轴的距离
u   = -kx + s
θ   表示云台pitch与水平轴夹角
ω   表示角度相对时间一阶导数
b   表示电机的阻力，默认 f阻 = k*ω
k   = [k1 k2]由lqr确定
k   的整定方法 1、 k1 = f(θ) 以1°为一个单位，计算不同θ下的k1，将它们通过f(θ)拟合
               2、 增加前馈控制器
s   为积分项
u   为转矩, 对于6020有 u = 0.741i(国际单位制) 对于电流与can信号,有 y = 1111i - 236(通过yaw轴测得,y为传入信号,i单位为A)
            对于2006有 u = 0.18i(国际单位制)
            对于3508有 u = 0.3*i

J = SW图中可测,数量级大致符合即可, b = 0.01(影响可忽略不计)
Q = diag[40000 100], R = 1
matlab 可得k = [k1 ，k2] 
*/

enum Lqr_motor_e { MOTOR_6020 = 0,
                MOTOR_2006,    
                MOTOR_3508 };

#pragma pack(1)
typedef struct Lqr_config_t {
    float k1;
    float k2;
    float ki;//积分项参数
    float ki_limit;//积分项限幅
    float errsummax;
    float outputmax;
    enum Lqr_motor_e motor;
    float feedforward;
} Lqr_config;

typedef struct Lqr_t {
    Lqr_config config;
    float x1;  //
    float x2;  //·x1
    float x1_ref;
    float x1_err;
    double x1_errsum;
    float output;
    float output_full; //无输出限幅的计算输出
} Lqr;
#pragma pack()

void LQR_Init(Lqr* obj, Lqr_config* config);
void LQR_SetConfig(Lqr_config* obj, float k1, float k2, float ki, float errsummax, float outputmax,float ki_limit);
void LQR_Calc(Lqr* obj);

#endif
