#include "mpc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 访问扁平化二维数组: A[rows][cols] → A[i * cols + j] */
#define MAT_ELEM(data, cols, i, j) ((data)[(i) * (cols) + (j)])

/* 对行优先的对称正定矩阵执行 H=L·Lᵀ 分解，非正定时返回 0。 */
static uint8_t cholesky_decomp(const float* H, uint8_t n, float* L) {
    memset(L, 0, sizeof(float) * n * n);
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            float sum = H[i * n + j];
            for (uint8_t k = 0; k < j; k++) {
                sum -= L[i * n + k] * L[j * n + k];
            }
            if (i == j) {
                if (sum <= MPC_FLOAT_EPS) return 0;  /* 非正定或奇异 */
                L[i * n + i] = sqrtf(sum);
            } else {
                L[i * n + j] = sum / L[j * n + j];
            }
        }
    }
    return 1;
}

/* 通过前代和回代求解 L·Lᵀ·x=b。 */
static void cholesky_solve(const float* L, uint8_t n, const float* b, float* x) {
    float y[MPC_N_DU_MAX];
    /* 前代: L·y = b */
    for (uint8_t i = 0; i < n; i++) {
        float sum = b[i];
        for (uint8_t j = 0; j < i; j++) {
            sum -= L[i * n + j] * y[j];
        }
        y[i] = sum / L[i * n + i];
    }
    /* 回代: Lᵀ·x = y */
    for (int8_t i = (int8_t)n - 1; i >= 0; i--) {
        float sum = y[i];
        for (uint8_t j = (uint8_t)i + 1; j < n; j++) {
            sum -= L[j * n + i] * x[j];
        }
        x[(uint8_t)i] = sum / L[i * n + i];
    }
}

/* 通过 Cholesky 分解求对称正定矩阵的逆，非正定时返回 0。 */
static uint8_t mat_inv_raw(const float* src, uint8_t n, float* dst) {
    float L[n * n];
    if (!cholesky_decomp(src, n, L)) return 0;

    /* 以单位向量为右端项逐列求解，利用对称性仅算下三角并镜像 */
    float e[MPC_N_DU_MAX];
    memset(e, 0, sizeof(float) * n);
    for (uint8_t j = 0; j < n; j++) {
        e[j] = 1.0f;
        float col_j[MPC_N_DU_MAX];
        cholesky_solve(L, n, e, col_j);
        /* H⁻¹ 对称：下三角 (i ≥ j) 直接填，上三角镜像 */
        for (uint8_t i = j; i < n; i++) {
            dst[i * n + j] = col_j[i];
            dst[j * n + i] = col_j[i];
        }
        e[j] = 0.0f;
    }
    return 1;
}

/* 模型或配置变化后重建状态与输入预测矩阵。 */
static void mpc_build_prediction_matrices(Mpc* obj) {
    Mpc_config* cfg = &obj->config;
    uint8_t nx = cfg->nx;
    uint8_t nu = cfg->nu;
    uint8_t ny = cfg->ny;
    uint8_t Np = cfg->Np;
    uint8_t Nc = cfg->Nc;

    /* 预计算 A 的幂次 A_pow[k] = A^k (k = 0..Np) */
    float A_pow[MPC_MAX_PRED_HORIZ + 1][MPC_MAX_STATE_DIM * MPC_MAX_STATE_DIM];
    memset(A_pow[0], 0, sizeof(float) * nx * nx);
    for (uint8_t i = 0; i < nx; i++) A_pow[0][i * nx + i] = 1.0f;  // A^0 = I
    memcpy(A_pow[1], cfg->A, sizeof(float) * nx * nx);              // A^1 = A
    for (uint8_t k = 2; k <= Np; k++) {
        for (uint8_t i = 0; i < nx; i++) {
            for (uint8_t j = 0; j < nx; j++) {
                float sum = 0.0f;
                for (uint8_t r = 0; r < nx; r++) {
                    sum += A_pow[k - 1][i * nx + r] * cfg->A[r * nx + j];
                }
                A_pow[k][i * nx + j] = sum;
            }
        }
    }

    /* Fx (Phi): 自由响应 × x(k),  size (Np·ny) × nx */
    for (uint8_t p = 0; p < Np; p++) {
        float* A_k = A_pow[p + 1];  // CA^{p+1}
        for (uint8_t i = 0; i < ny; i++) {
            for (uint8_t j = 0; j < nx; j++) {
                float sum = 0.0f;
                for (uint8_t r = 0; r < nx; r++) {
                    sum += cfg->C[i * nx + r] * A_k[r * nx + j];
                }
                obj->Phi[p * ny * nx + i * nx + j] = sum;
            }
        }
    }

    /* Theta_u (Omega): 受迫响应 × U,  size (Np·ny) × (Nc·nu) */
    uint8_t theta_cols = Nc * nu;
    uint8_t theta_rows = Np * ny;
    memset(obj->Omega, 0, sizeof(float) * theta_rows * theta_cols);
    for (uint8_t p = 0; p < Np; p++) {
        for (uint8_t c = 0; c < Nc; c++) {
            if (c > p) continue;  /* 因果性：未来控制不影响过去输出 */
            float* A_pc = A_pow[p - c];  // A^{p-c}
            for (uint8_t i = 0; i < ny; i++) {
                for (uint8_t j = 0; j < nu; j++) {
                    float sum = 0.0f;
                    for (uint8_t r1 = 0; r1 < nx; r1++) {
                        float c_a = 0.0f;
                        for (uint8_t r2 = 0; r2 < nx; r2++) {
                            c_a += cfg->C[i * nx + r2] * A_pc[r2 * nx + r1];
                        }
                        sum += c_a * cfg->B[r1 * nu + j];
                    }
                    MAT_ELEM(obj->Omega, theta_cols, p * ny + i, c * nu + j) = sum;
                }
            }
        }
    }
}

/* 构造 min 0.5·Uᵀ·H·U+gᵀ·U，并添加输入上下界约束。 */
static void mpc_build_qp(Mpc* obj) {
    Mpc_config* cfg = &obj->config;
    uint8_t nx = cfg->nx;
    uint8_t nu = cfg->nu;
    uint8_t ny = cfg->ny;
    uint8_t Np = cfg->Np;
    uint8_t Nc = cfg->Nc;
    uint8_t n_u = Nc * nu;   /* 决策变量维度 */
    uint8_t n_y = Np * ny;   /* 预测输出维度 */

    /* 1. 自由响应 Y_free = Fx * x(k) */
    float y_free[MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM];
    memset(y_free, 0, sizeof(float) * n_y);
    for (uint8_t i = 0; i < n_y; i++) {
        for (uint8_t j = 0; j < nx; j++) {
            y_free[i] += MAT_ELEM(obj->Phi, nx, i, j) * obj->x0[j];
        }
    }

    /* 2. 误差向量 E = Y_ref_traj - Y_free */
    float error[MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM];
    for (uint8_t p = 0; p < Np; p++) {
        for (uint8_t j = 0; j < ny; j++) {
            error[p * ny + j] = obj->y_ref_traj[p * ny + j] - y_free[p * ny + j];
        }
    }

    /* 3. 构建 H = Theta_u^T * Q_bar * Theta_u + R_bar（利用 H 对称性仅算上三角） */
    /* 先算 Q_bar * Theta_u（对角 Q 快速路径，消去 ny 内层循环） */
    float QTheta[MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM * MPC_MAX_CTRL_HORIZ * MPC_MAX_INPUT_DIM];
    uint8_t theta_cols = n_u;
    for (uint8_t i = 0; i < n_y; i++) {
        uint8_t blk = i / ny;        /* 预测步索引 */
        uint8_t loc_i = i % ny;      /* 输出通道索引 */
        float weight = obj->y_weight_traj[blk * ny + loc_i];
        if (weight <= 0.0f) weight = 1.0f;
        float wq = weight * cfg->Q[loc_i * ny + loc_i];  /* Q 对角占优，仅取对角元 */
        for (uint8_t j = 0; j < n_u; j++) {
            QTheta[i * n_u + j] = wq * MAT_ELEM(obj->Omega, theta_cols, blk * ny + loc_i, j);
        }
    }

    /* H = Omega^T * QTheta，仅计算上三角（H 对称，下三角镜像） */
    memset(obj->H, 0, sizeof(float) * n_u * n_u);
    for (uint8_t i = 0; i < n_u; i++) {
        for (uint8_t j = i; j < n_u; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < n_y; k++) {
                sum += MAT_ELEM(obj->Omega, theta_cols, k, i) * QTheta[k * n_u + j];
            }
            obj->H[i * n_u + j] = sum;
            obj->H[j * n_u + i] = sum;  /* 镜像 */
        }
    }
    /* 加 R_bar */
    for (uint8_t c = 0; c < Nc; c++) {
        for (uint8_t i = 0; i < nu; i++) {
            for (uint8_t j = 0; j < nu; j++) {
                uint8_t row = c * nu + i;
                uint8_t col = c * nu + j;
                obj->H[row * n_u + col] += cfg->R[i * nu + j];
            }
        }
    }

    /* 4. 构建 g = -Theta_u^T * Q_bar * E（对角 Q 快速路径，消去 ny 内层循环） */
    float QE[MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM];
    for (uint8_t p = 0; p < Np; p++) {
        for (uint8_t i = 0; i < ny; i++) {
            float weight = obj->y_weight_traj[p * ny + i];
            if (weight <= 0.0f) weight = 1.0f;
            QE[p * ny + i] = weight * cfg->Q[i * ny + i] * error[p * ny + i];
        }
    }
    memset(obj->G, 0, sizeof(float) * n_u);
    for (uint8_t i = 0; i < n_u; i++) {
        float sum = 0.0f;
        for (uint8_t k = 0; k < n_y; k++) {
            sum += MAT_ELEM(obj->Omega, theta_cols, k, i) * QE[k];
        }
        obj->G[i] = -sum;
    }

    /* 化为标准 QP 形式: H *= 2, G *= 2（利用 H 对称性仅算上三角） */
    for (uint8_t i = 0; i < n_u; i++) {
        for (uint8_t j = i; j < n_u; j++) {
            obj->H[i * n_u + j] *= 2.0f;
            obj->H[j * n_u + i] = obj->H[i * n_u + j];  /* 镜像保证一致性 */
        }
        obj->G[i] *= 2.0f;
    }

    /* 5. 构建输入约束 A_cons * U <= b_cons */
    uint8_t cons_cnt = 0;
    memset(obj->A_cons, 0, sizeof(obj->A_cons));
    memset(obj->b_cons, 0, sizeof(obj->b_cons));

    /* 5a. 输入幅值约束: u_min <= u <= u_max */
    if (cfg->has_input_bound) {
        for (uint8_t c = 0; c < Nc; c++) {
            for (uint8_t j = 0; j < nu; j++) {
                obj->A_cons[cons_cnt * n_u + c * nu + j] = 1.0f;
                obj->b_cons[cons_cnt] = cfg->u_max[j];
                cons_cnt++;

                obj->A_cons[cons_cnt * n_u + c * nu + j] = -1.0f;
                obj->b_cons[cons_cnt] = -cfg->u_min[j];
                cons_cnt++;
            }
        }
    }

    obj->num_cons = cons_cnt;
}

/* 使用 Hildreth 对偶迭代求解约束 QP，Hessian 无效时返回 0。 */
static uint8_t mpc_hildreth_solve(Mpc* obj) {
    Mpc_config* cfg = &obj->config;
    uint8_t n_var = cfg->Nc * cfg->nu;
    uint8_t n_cons = obj->num_cons;

    /* Step 1: 求 H^{-1} */
    if (!mat_inv_raw(obj->H, n_var, obj->H_inv)) return 0;

    /* Step 2: 无约束情况直接返回解析解 */
    if (n_cons == 0) {
        memset(obj->u_opt, 0, sizeof(float) * n_var);
        for (uint8_t i = 0; i < n_var; i++) {
            float sum = 0.0f;
            for (uint8_t j = 0; j < n_var; j++) {
                sum += obj->H_inv[i * n_var + j] * obj->G[j];
            }
            obj->u_opt[i] = -sum;
        }
        return 1;
    }

    /* 无约束最优解 x_unc = -H^{-1} * g */
    float x_unc[n_var];
    memset(x_unc, 0, sizeof(float) * n_var);
    for (uint8_t i = 0; i < n_var; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < n_var; j++) {
            sum += obj->H_inv[i * n_var + j] * obj->G[j];
        }
        x_unc[i] = -sum;
    }

    /* Step 3: 分析 A_cons 稀疏性 */
    /* 输入约束每行仅一个 ±1 元（col = c*nu + j） */
    int8_t cons_col[MPC_N_CONS_MAX];
    float cons_sign[MPC_N_CONS_MAX];
    for (uint8_t i = 0; i < n_cons; i++) {
        cons_col[i] = -1;
        cons_sign[i] = 0.0f;
        int8_t nz_count = 0;
        for (uint8_t j = 0; j < n_var; j++) {
            float a = obj->A_cons[i * n_var + j];
            if (mpc_fabsf(a) > MPC_FLOAT_EPS) {
                cons_col[i] = (int8_t)j;
                cons_sign[i] = a;
                nz_count++;
            }
        }
        if (nz_count != 1) cons_col[i] = -1;  /* 多非零元 → 按稠密处理 */
    }

    /* Step 4: 计算 d = b - A * x_unc（利用稀疏性），同时检查无约束解可行性 */
    float d[MPC_N_CONS_MAX];
    uint8_t feasible = 1;
    for (uint8_t i = 0; i < n_cons; i++) {
        float a_dot_x;
        if (cons_col[i] >= 0) {
            /* 稀疏行: A_i · x = s_i · x_unc[col_i] */
            a_dot_x = cons_sign[i] * x_unc[(uint8_t)cons_col[i]];
        } else {
            /* 稠密行: 完整点积 */
            a_dot_x = 0.0f;
            for (uint8_t j = 0; j < n_var; j++) {
                a_dot_x += obj->A_cons[i * n_var + j] * x_unc[j];
            }
        }
        d[i] = obj->b_cons[i] - a_dot_x;
        if (a_dot_x > obj->b_cons[i] + MPC_FLOAT_EPS) feasible = 0;
    }

    if (feasible) {
        memcpy(obj->u_opt, x_unc, sizeof(float) * n_var);
        return 1;
    }

    /* 复用上一周期的拉格朗日乘子作为迭代初值。 */

    /* Step 6: 预计算 P = A * H^{-1} * A^T（利用 A_cons 稀疏性，从 O(n_cons²·n_var²) 降至 O(n_cons²)） */
    float P[n_cons * n_cons];
    memset(P, 0, sizeof(float) * n_cons * n_cons);

    for (uint8_t i = 0; i < n_cons; i++) {
        for (uint8_t j = 0; j < n_cons; j++) {
            float sum;
            if (cons_col[i] >= 0 && cons_col[j] >= 0) {
                /* 双稀疏: P[i][j] = s_i · s_j · H^{-1}[col_i][col_j] */
                uint8_t ci = (uint8_t)cons_col[i];
                uint8_t cj = (uint8_t)cons_col[j];
                sum = cons_sign[i] * cons_sign[j] * obj->H_inv[ci * n_var + cj];
            } else if (cons_col[i] >= 0) {
                /* 行 i 稀疏, 行 j 稠密: P[i][j] = s_i · Σ_c H^{-1}[col_i][c] · A_j[c] */
                uint8_t ci = (uint8_t)cons_col[i];
                sum = 0.0f;
                for (uint8_t c = 0; c < n_var; c++) {
                    sum += obj->H_inv[ci * n_var + c] * obj->A_cons[j * n_var + c];
                }
                sum *= cons_sign[i];
            } else if (cons_col[j] >= 0) {
                /* 行 i 稠密, 行 j 稀疏: P[i][j] = s_j · Σ_r A_i[r] · H^{-1}[r][col_j] */
                uint8_t cj = (uint8_t)cons_col[j];
                sum = 0.0f;
                for (uint8_t r = 0; r < n_var; r++) {
                    sum += obj->A_cons[i * n_var + r] * obj->H_inv[r * n_var + cj];
                }
                sum *= cons_sign[j];
            } else {
                /* 稠密约束矩阵的四重循环乘积。 */
                sum = 0.0f;
                for (uint8_t r = 0; r < n_var; r++) {
                    float h_a = 0.0f;
                    for (uint8_t c = 0; c < n_var; c++) {
                        h_a += obj->H_inv[r * n_var + c] * obj->A_cons[j * n_var + c];
                    }
                    sum += obj->A_cons[i * n_var + r] * h_a;
                }
            }
            P[i * n_cons + j] = sum;
        }
    }

    /* Step 7: Hildreth 迭代（warm-start + 自适应上限） */
    /* 每轮更新全部约束乘子，迭代上限与约束数成比例。 */
    uint8_t max_iter = n_cons * 3;
    if (max_iter < 30) max_iter = 30;
    if (max_iter > 100) max_iter = 100;
    float lambda_prev[n_cons];
    for (uint8_t iter = 0; iter < max_iter; iter++) {
        memcpy(lambda_prev, obj->lambda, sizeof(float) * n_cons);
        float max_change = 0.0f;

        for (uint8_t i = 0; i < n_cons; i++) {
            /* 计算 sum_{j≠i} P_{ij} * λ_j */
            float sum_p_lambda = 0.0f;
            for (uint8_t j = 0; j < n_cons; j++) {
                if (j == i) continue;
                sum_p_lambda += P[i * n_cons + j] * obj->lambda[j];
            }
            float w = -(d[i] + sum_p_lambda) / P[i * n_cons + i];
            float lambda_old = obj->lambda[i];
            obj->lambda[i] = w > 0.0f ? w : 0.0f;

            float change = mpc_fabsf(obj->lambda[i] - lambda_old);
            if (change > max_change) max_change = change;
        }

        if (max_change < 1e-6f) break;
    }

    /* Step 8: 恢复原问题最优解: x_opt = x_unc - H^{-1} * A^T * λ（利用稀疏性加速 A^T·λ） */
    /* 先计算 A^T·λ: 稀疏行 j 仅在列 col_j 处贡献 s_j · λ[j] */
    float at_lambda[MPC_N_DU_MAX];
    memset(at_lambda, 0, sizeof(float) * n_var);
    for (uint8_t j = 0; j < n_cons; j++) {
        if (cons_col[j] >= 0) {
            at_lambda[(uint8_t)cons_col[j]] += cons_sign[j] * obj->lambda[j];
        } else {
            for (uint8_t k = 0; k < n_var; k++) {
                at_lambda[k] += obj->A_cons[j * n_var + k] * obj->lambda[j];
            }
        }
    }
    // x_opt = x_unc - H^{-1} · (A^T·λ)
    memset(obj->u_opt, 0, sizeof(float) * n_var);
    for (uint8_t i = 0; i < n_var; i++) {
        float correction = 0.0f;
        for (uint8_t k = 0; k < n_var; k++) {
            correction += obj->H_inv[i * n_var + k] * at_lambda[k];
        }
        obj->u_opt[i] = x_unc[i] - correction;
    }

    return 1;
}

/* 动态分配 MPC 对象并加载已初始化的配置。 */
Mpc* Create_Mpc(Mpc_config* config) {
    Mpc* obj = (Mpc*)malloc(sizeof(Mpc));
    if (!obj) return NULL;
    memset(obj, 0, sizeof(Mpc));
    obj->config = *config;
    /* 初始化权重轨迹为 1.0（均匀权重） */
    uint8_t ny = config->ny;
    uint8_t Np = config->Np;
    for (uint8_t p = 0; p < Np; p++) {
        for (uint8_t i = 0; i < ny; i++) {
            obj->y_weight_traj[p * ny + i] = 1.0f;
        }
    }
    /* 初始化参考轨迹为 0 */
    memset(obj->y_ref_traj, 0, sizeof(float) * ny * Np);
    obj->initialized = 0;
    return obj;
}

void Mpc_Set_State(Mpc* obj, const float* x0) {
    memcpy(obj->x0, x0, sizeof(float) * obj->config.nx);
}

void Mpc_Set_Reference(Mpc* obj, const float* y_ref) {
    uint8_t ny = obj->config.ny;
    uint8_t Np = obj->config.Np;
    /* 将当前参考填充整个预测时域 */
    for (uint8_t p = 0; p < Np; p++) {
        memcpy(&obj->y_ref_traj[p * ny], y_ref, sizeof(float) * ny);
    }
}

void Mpc_Set_Reference_Traj(Mpc* obj, const float* y_ref_traj) {
    uint8_t ny = obj->config.ny;
    uint8_t Np = obj->config.Np;
    memcpy(obj->y_ref_traj, y_ref_traj, sizeof(float) * ny * Np);
}

void Mpc_Set_Weight_Traj(Mpc* obj, const float* weight_traj) {
    uint8_t ny = obj->config.ny;
    uint8_t Np = obj->config.Np;
    memcpy(obj->y_weight_traj, weight_traj, sizeof(float) * ny * Np);
}

void Mpc_Get_Control(Mpc* obj, float* u_out) {
    memcpy(u_out, obj->u_pred, sizeof(float) * obj->config.nu);
}

void Mpc_Get_Prediction(Mpc* obj, float* y_pred_out) {
    uint8_t ny = obj->config.ny;
    uint8_t Np = obj->config.Np;
    memcpy(y_pred_out, obj->y_pred, sizeof(float) * ny * Np);
}

/* 构建预测和 QP 矩阵，求解最优输入序列并输出当前步控制量。 */
uint8_t MPC_Calc(Mpc* obj) {
    Mpc_config* cfg = &obj->config;
    uint8_t nu = cfg->nu;
    uint8_t ny = cfg->ny;
    uint8_t Np = cfg->Np;
    uint8_t Nc = cfg->Nc;
    uint8_t n_u = Nc * nu;

    /* 首次或模型变更后，重建预测矩阵（缓存，后续拍仅重建 QP） */
    if (!obj->initialized) {
        mpc_build_prediction_matrices(obj);
        obj->initialized = 1;
    }

    /* 构建 QP 问题 */
    mpc_build_qp(obj);

    /* 求解 QP */
    obj->solve_ok = 0;  /* 预设失败，成功后在下方置 1 */
    if (!mpc_hildreth_solve(obj)) {
        return 0;  /* Hessian 奇异/非正定，保留上一拍控制量 */
    }
    obj->solve_ok = 1;

    /* 提取当前拍最优控制量: u(k) = U*(0) */
    for (uint8_t i = 0; i < nu; i++) {
        obj->u_pred[i] = obj->u_opt[i];
    }

    /* 计算预测输出序列 Y_pred = Fx·x(k) + Theta_u·U* */
    float y_free[MPC_MAX_PRED_HORIZ * MPC_MAX_OUTPUT_DIM];
    memset(y_free, 0, sizeof(float) * Np * ny);
    for (uint8_t i = 0; i < Np * ny; i++) {
        for (uint8_t j = 0; j < cfg->nx; j++) {
            y_free[i] += MAT_ELEM(obj->Phi, cfg->nx, i, j) * obj->x0[j];
        }
    }
    for (uint8_t i = 0; i < Np * ny; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < n_u; j++) {
            sum += MAT_ELEM(obj->Omega, n_u, i, j) * obj->u_opt[j];
        }
        obj->y_pred[i] = y_free[i] + sum;
    }

    return obj->solve_ok;
}
