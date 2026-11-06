#pragma once

/* 外部库 */
#include <stdint.h>
/* HAL层 */
#include "common.h"

/* 矩阵最大维度定义，可根据实际需求调整 */
#define MPC_MAX_STATE_DIM  2   /* 状态维度 nx */
#define MPC_MAX_INPUT_DIM  1   /* 输入维度 nu */
#define MPC_MAX_OUTPUT_DIM 2   /* 输出维度 ny */
#define MPC_MAX_PRED_HORIZ  40 /* 最大预测时域 Np */
#define MPC_MAX_CTRL_HORIZ 20   /* 最大控制时域 Nc（按需调大，主导内存/计算量） */

/* 派生维度（不可单独修改） */
#define MPC_N_DU_MAX    (MPC_MAX_CTRL_HORIZ * MPC_MAX_INPUT_DIM)   /* 决策变量数 */
#define MPC_N_Y_MAX     (MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM)  /* 预测输出总行 */
#define MPC_N_CONS_MAX  (2 * MPC_N_DU_MAX)
#define MPC_A_CONS_SIZE (MPC_N_CONS_MAX * MPC_N_DU_MAX)

/* 内部常量与辅助函数 */
#define MPC_FLOAT_EPS 1e-8f

static inline float mpc_fabsf(float x) { return (x >= 0.0f) ? x : -x; }

/* 配置离散状态空间模型、预测时域、权重和输入约束。 */
typedef struct Mpc_config_t {
    /* 状态空间模型: x(k+1) = A*x(k) + B*u(k), y(k) = C*x(k) */
    float A[MPC_MAX_STATE_DIM * MPC_MAX_STATE_DIM]; /* 状态转移矩阵 (nx × nx) */
    float B[MPC_MAX_STATE_DIM * MPC_MAX_INPUT_DIM]; /* 输入矩阵 (nx × nu) */
    float C[MPC_MAX_OUTPUT_DIM * MPC_MAX_STATE_DIM]; /* 输出矩阵 (ny × nx) */
    uint8_t nx;  /* 状态维度 */
    uint8_t nu;  /* 输入维度 */
    uint8_t ny;  /* 输出维度 */
    float Ts;    /* 采样周期 (s) */

    /* 时域 */
    uint8_t Np;  /* 预测时域 */
    uint8_t Nc;  /* 控制时域 */

    /* 权重矩阵 */
    float Q[MPC_MAX_OUTPUT_DIM * MPC_MAX_OUTPUT_DIM]; /* 输出权重 (ny × ny) */
    float R[MPC_MAX_INPUT_DIM * MPC_MAX_INPUT_DIM];   /* 控制权重 (nu × nu) */

    /* 约束 */
    float u_min[MPC_MAX_INPUT_DIM];  /* 输入下界 */
    float u_max[MPC_MAX_INPUT_DIM];  /* 输入上界 */
    uint8_t has_input_bound;         /* 是否有输入约束 */
} Mpc_config;

/* 保存 MPC 预测、QP 求解和控制输出工作区。 */
typedef struct Mpc_t {
    Mpc_config config;  /* 控制器配置（含模型、时域、权重、约束） */

    /* 预测矩阵 */
    float Phi[MPC_N_Y_MAX * MPC_MAX_STATE_DIM];     // Fx: Np*ny × nx
    float Omega[MPC_N_DU_MAX * MPC_N_Y_MAX];        // Theta_u: Np*ny × Nc*nu

    /* QP 矩阵 */
    float H[MPC_N_DU_MAX * MPC_N_DU_MAX];           // Hessian, n_du × n_du
    float H_inv[MPC_N_DU_MAX * MPC_N_DU_MAX];       // H^{-1}
    float G[MPC_N_DU_MAX];                          // gradient, n_du × 1
    float A_cons[MPC_A_CONS_SIZE];                  /* 约束矩阵, n_cons × n_du */
    float b_cons[MPC_N_CONS_MAX];                   /* 约束上界, n_cons × 1 */

    /* QP 求解中间变量 */
    float lambda[MPC_N_CONS_MAX];                   /* 拉格朗日乘子 */
    uint8_t num_cons;                               /* 有效约束数 */

    /* 状态缓存 */
    float x0[MPC_MAX_STATE_DIM];                    /* 当前状态 x(k) */
    float y_ref_traj[MPC_N_Y_MAX];                  /* 未来 Np 步参考轨迹 */
    float y_weight_traj[MPC_N_Y_MAX];               /* 未来 Np 步代价权重 */

    /* 结果 */
    float u_opt[MPC_N_DU_MAX];                      /* 最优控制量序列 U* (Nc·nu) */
    float u_pred[MPC_MAX_INPUT_DIM];                /* 当前拍最优控制量 u(k) */
    float y_pred[MPC_N_Y_MAX];                      /* 预测输出序列 */

    uint8_t initialized;                            /* 预测矩阵是否已构建 */
    uint8_t solve_ok;                               /* 本拍求解状态: 1=成功, 0=失败(Hessian奇异/非正定) */
} Mpc;

/* MPC 核心接口 */

/* 动态创建 MPC 对象，配置应完整初始化；失败返回 NULL。 */
Mpc* Create_Mpc(Mpc_config* config);

/* 设置状态和参考轨迹后求解；Hessian 无效时返回 0。 */
uint8_t MPC_Calc(Mpc* obj);

/* 加载长度为 nx 的当前状态向量。 */
void Mpc_Set_State(Mpc* obj, const float* x0);

/* 将长度为 ny 的参考输出填充到整个预测时域。 */
void Mpc_Set_Reference(Mpc* obj, const float* y_ref);

/* 加载长度为 ny×Np 的参考轨迹。 */
void Mpc_Set_Reference_Traj(Mpc* obj, const float* y_ref_traj);

/* 加载长度为 ny×Np 的逐步输出权重。 */
void Mpc_Set_Weight_Traj(Mpc* obj, const float* weight_traj);

/* 读取长度为 nu 的当前步控制量。 */
void Mpc_Get_Control(Mpc* obj, float* u_out);

/* 读取长度为 ny×Np 的预测输出序列。 */
void Mpc_Get_Prediction(Mpc* obj, float* y_pred_out);
