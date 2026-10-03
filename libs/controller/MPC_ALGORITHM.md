# MPC 算法文档 — 库解析

> 库文件：[mpc.h](mpc.h) / [mpc.c](mpc.c)

---

# 第一部分：核心算法原理

## 1. 核心思想

MPC（Model Predictive Control）三要素：**预测模型** → **滚动优化** → **反馈校正**。每个采样时刻，基于系统模型预测未来 $N_p$ 步的输出，求解一个带约束的有限时域最优控制问题，但**只施加第一个控制量**，下一时刻用新的状态反馈重复整个过程。

本实现采用**经典非增量形式**（决策变量 = 控制量 $u$ 本身），适用于嵌入式电机的直接扭矩控制场景。

---

## 2. 数学模型

### 2.1 被控对象状态空间模型

$$
\begin{aligned}
x(k+1) &= A \cdot x(k) + B \cdot u(k) \\
y(k)   &= C \cdot x(k)
\end{aligned}
$$

| 符号 | 维度 | 含义 |
|------|------|------|
| $x(k)$ | nx × 1 | 状态向量 |
| $u(k)$ | nu × 1 | 控制输入 |
| $y(k)$ | ny × 1 | 系统输出 |
| $A$ | nx × nx | 状态转移矩阵 |
| $B$ | nx × nu | 输入矩阵 |
| $C$ | ny × nx | 输出矩阵 |

### 2.2 决策变量

控制序列 $U$ 为待优化变量（共 $N_c \cdot n_u$ 个标量）：

$$
U = \begin{bmatrix} u(k|k) \\ u(k+1|k) \\ \vdots \\ u(k+N_c-1|k) \end{bmatrix} \in \mathbb{R}^{N_c n_u \times 1}
$$

> 非增量形式以 $u$ 本身为决策变量，求解结果直接为控制量，无需积分累积。

---

## 3. 预测方程 — 多步输出预测

### 3.1 递推推导

递推状态方程 $x(k+p|k) = A^p x(k) + \sum_{c=0}^{p-1} A^{p-1-c} B\, u(k+c|k)$，将未来 $N_p$ 步的输出 $Y$ 表示为当前状态 $x(k)$ 和决策变量 $U$ 的线性组合：

$$
\underbrace{\begin{bmatrix} y(k+1|k) \\ y(k+2|k) \\ \vdots \\ y(k+N_p|k) \end{bmatrix}}_{Y \in \mathbb{R}^{N_p n_y \times 1}}
= \underbrace{\begin{bmatrix} CA \\ CA^2 \\ \vdots \\ CA^{N_p} \end{bmatrix}}_{F_x \in \mathbb{R}^{N_p n_y \times n_x}} x(k)
\;+\;
\underbrace{\begin{bmatrix}
CB & 0 & \cdots & 0 \\
CAB & CB & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
CA^{N_p-1}B & CA^{N_p-2}B & \cdots & CA^{N_p-N_c}B
\end{bmatrix}}_{\Theta_u \in \mathbb{R}^{N_p n_y \times N_c n_u}}
\underbrace{\begin{bmatrix} u(k|k) \\ u(k+1|k) \\ \vdots \\ u(k+N_c-1|k) \end{bmatrix}}_{U \in \mathbb{R}^{N_c n_u \times 1}}
$$

简记为：

$$
\boxed{Y = F_x \cdot x(k) + \Theta_u \cdot U}
$$

### 3.2 时域参数：$N_p$ 与 $N_c$

递推预测涉及两个独立可调的时域参数：

| 参数 | 全称 | 含义 | 决定 |
|------|------|------|------|
| $N_p$ | 预测时域 | 向未来看多远 | $\Theta_u$ 行数 = $N_p \cdot n_y$ |
| $N_c$ | 控制时域 | 优化多少拍控制量 | $\Theta_u$ 列数 = $N_c \cdot n_u$，即决策变量 $U$ 的维度 |

#### $N_p > N_c$ 意味着什么？

直观类比：开车时，你**看向**前方 100 米（$N_p$），但只**规划**接下来 20 米的方向盘转角（$N_c$）。剩下的 80 米你只是在观察——让已规划的动作的后果自然展开，而不去干预。

数学上：决策变量 $U$ 只包含前 $N_c$ 步的控制量。对于第 $N_c, N_c+1, \dots, N_p-1$ 步，控制量假设为零——即"放手让系统自由滑行"。

$$
\underbrace{\begin{bmatrix} u(k) \\ u(k+1) \\ \vdots \\ u(k+N_c-1) \end{bmatrix}}_{\text{优化变量，Nc 个}}
\qquad
\underbrace{u(k+N_c) = u(k+N_c+1) = \cdots = 0}_{\text{优化范围外，固定为零}}
$$

反映在 $\Theta_u$ 矩阵上：有 $N_p$ 个块行（每预测步），但仅 $N_c$ 个块列（每控制步），形成了"高瘦"的结构。

#### 为什么取 $N_c \ll N_p$？

| 原因 | 说明 |
|------|------|
| **滚动时域天然补偿** | 下一拍会重新求解整个优化问题，当前规划的远端控制量本就不会被执行——优化它们是浪费 |
| **计算量由 $N_c$ 主导** | QP 的 $H$ 矩阵规模为 $(N_c n_u)^2$，约束数为 $2N_c n_u$。$N_c$ 增大 1 倍，$H$ 元素数增 4 倍 |
| **远端控制影响递减** | $A^k$ 随 $k$ 增大而衰减（稳定系统），第 $N_c$ 步之后的控制量对当前输出的杠杆效应远小于前几步 |
| **$N_p$ 必须足够长** | 预测太短会导致"近视"——优化器可能选择一条短期有利但远期发散的轨迹 |

#### $\Theta_u$ 矩阵形状

$$
\Theta_u = \underbrace{\begin{bmatrix}
CB & 0 & \cdots & 0 \\
CAB & CB & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
CA^{N_c-1}B & CA^{N_c-2}B & \cdots & CB \\[4pt]
\hline \\[-8pt]
CA^{N_c}B & CA^{N_c-1}B & \cdots & (CA+I)B \\
\vdots & \vdots & \ddots & \vdots \\
CA^{N_p-1}B & CA^{N_p-2}B & \cdots & \sum_{i=0}^{N_p-N_c} CA^i B
\end{bmatrix}}_{\text{共 } N_p \text{ 块行}}
\quad
\begin{aligned}
&\left.\rule{0pt}{20pt}\right\} N_c \text{ 块行 (有控制输入)}\\[30pt]
&\left.\rule{0pt}{20pt}\right\} N_p-N_c \text{ 块行 (无新控制，自由滑行)}
\end{aligned}
$$

注意分界线以下的行：每一行仍由 $u(k), \dots, u(k+N_c-1)$ 的加权和构成，但没有新的控制变量补充进来——系统仅凭惯性（$A$ 的幂次）向前传播。预测得越远（$N_p$ 越大），这部分就越多，但你始终只用 $N_c$ 个"旋钮"来塑造整个 $N_p$ 步的未来。

### 3.3 矩阵含义

| 矩阵 | 符号 | 名称 | 维度 | 含义 |
|------|------|------|------|------|
| $F_x$ | Phi | 自由响应矩阵 | Np·ny × nx | 零输入时系统的自然演化 |
| $\Theta_u$ | Omega | 受迫响应矩阵 | Np·ny × Nc·nu | 单位控制量对输出的影响 |

其中 $Y_{free} = F_x \cdot x(k)$ 称为**自由响应**，$\Theta_u \cdot U$ 称为**受迫响应**。

### 3.4 因果性约束

$\Theta_u$ 为下三角块矩阵：当控制步 $c >$ 预测步 $p$ 时对应块为零（未来的控制不影响过去的输出）。此性质在构造时显式跳过（`if (c > p) continue`）。

### 3.5 预测矩阵缓存

预测矩阵 $F_x$ 和 $\Theta_u$ 仅依赖于模型参数 $(A, B, C)$ 和时域 $(N_p, N_c)$，与实时状态无关。因此在首次求解时构建一次并缓存（`initialized` 标志），后续拍直接复用，大幅降低每拍计算量。

---

## 4. 代价函数与 QP 转化

### 4.1 代价函数

优化目标：使输出跟踪参考值，同时惩罚过大的控制量：

$$
J = \sum_{p=1}^{N_p} \big\| y_{ref}(k+p) - y(k+p|k) \big\|_{Q}^2 \;+\; \sum_{c=0}^{N_c-1} \big\| u(k+c|k) \big\|_{R}^2
$$

其中 $Q \in \mathbb{R}^{ny \times ny}$ 为输出误差权重矩阵，$R \in \mathbb{R}^{nu \times nu}$ 为控制量权重矩阵。

代入 $Y = Y_{free} + \Theta_u U$（其中 $Y_{free} = F_x \cdot x(k)$）：

$$
\begin{aligned}
J &= (Y_{ref} - Y_{free} - \Theta_u U)^T \bar{Q} (Y_{ref} - Y_{free} - \Theta_u U) + U^T \bar{R} U \\
  &= E^T \bar{Q} E \;-\; 2E^T \bar{Q} \Theta_u U \;+\; U^T (\Theta_u^T \bar{Q} \Theta_u + \bar{R}) U
\end{aligned}
$$

其中 $\bar{Q} = \text{blkdiag}(Q, Q, \dots, Q)$ 和 $\bar{R} = \text{blkdiag}(R, R, \dots, R)$ 为分块对角权重矩阵，$E = Y_{ref} - Y_{free}$ 为自由响应跟踪误差。

### 4.2 时变权重

每个预测步的每个输出通道可配置独立权重 $w_p^{(i)} \geq 0$。实际使用的 $Q$ 等效为 $\tilde{Q}^{(p)} = w_p^{(i)} \cdot Q$。若某步权重 ≤ 0，自动回退为 1.0。

### 4.3 标准 QP 形式

消去常数项 $E^T \bar{Q} E$，将剩余二次项和一次项整理为 $\min_U \frac{1}{2}U^T H U + g^T U$：

$$
\boxed{H = 2 \cdot (\Theta_u^T \bar{Q} \Theta_u + \bar{R})}
$$

$$
\boxed{g = -2 \cdot \Theta_u^T \bar{Q} \cdot (Y_{ref} - Y_{free})}
$$

> **因子 2 的作用**：确保 Hildreth 对偶推导中 $P = A H^{-1} A^T$ 与标准 QP 的 $\frac{1}{2}x^T H x + g^T x$ 代数形式一致。

---

## 5. 约束处理

支持**盒式约束**，统一化为 $A_{cons} \cdot U \leq b_{cons}$：

### 5.1 控制幅值约束

$u_{min} \leq u(k+c|k) \leq u_{max}$，$c = 0, \dots, N_c-1$

每步每个输入通道产生 2 条约束，共 $2 N_c n_u$ 条：

```
+u(c, j) ≤ u_max[j]        →  A_cons[row][c·nu + j] = +1,  b_cons[row] = u_max[j]
-u(c, j) ≤ -u_min[j]       →  A_cons[row][c·nu + j] = -1,  b_cons[row] = -u_min[j]
```

### 5.2 约束规模

| 约束来源 | 数量 | 启用条件 |
|----------|------|----------|
| 输入幅值 | $2 N_c n_u$ | `has_input_bound = 1` |

约束总数 $n_{cons}$ 受宏 `MPC_N_CONS_MAX` 限制。

---

## 6. Hildreth QP 求解器

求解问题：

$$
\min_{U} \quad \frac{1}{2}U^T H U + g^T U \quad \text{s.t.} \quad A_{cons} \cdot U \leq b_{cons}
$$

Hildreth 对偶算法分四步：

### Step 1: 预计算

- **求 $H^{-1}$**：Gauss-Jordan 列主元消元法，增广矩阵 $[H | I]$ → $[I | H^{-1}]$。$n_{du} \leq 8$（默认 Nc=8, nu=1）的小矩阵足够精确，栈上 VLA 安全
- **无约束最优解**：$U_{unc} = -H^{-1} g$
- **对偶矩阵**：$P = A_{cons} \cdot H^{-1} \cdot A_{cons}^T$（$n_{cons} \times n_{cons}$）
- **对偶向量**：$d = b_{cons} - A_{cons} \cdot U_{unc}$

### Step 2: 快速检查

若 $U_{unc}$ 已满足所有约束（$A_{cons} \cdot U_{unc} \leq b_{cons} + \epsilon$），直接返回解析解，无需迭代。

### Step 3: Hildreth 迭代（坐标上升法）

对偶变量（拉格朗日乘子）$\lambda_i \geq 0$ 逐个更新：

$$
\lambda_i^{(new)} = \max\left(0,\; -\frac{d_i + \sum_{j \neq i} P_{ij} \cdot \lambda_j^{(old)}}{P_{ii}}\right)
$$

收敛条件：$\max|\lambda_i^{(new)} - \lambda_i^{(old)}| < 10^{-6}$ 或达到 100 次迭代上限。

### Step 4: 恢复原问题最优解

$$
\boxed{U_{opt} = U_{unc} - H^{-1} \cdot A_{cons}^T \cdot \lambda}
$$

若 Hessian 奇异（`mat_inv_raw` 返回 0），求解失败，保留上一拍控制量不变。

---

## 7. 滚动时域 — Receding Horizon

求解得到最优控制序列 $U^* = [u^*(k), u^*(k+1), \dots, u^*(k+N_c-1)]^T$ 后，**仅施加第一个元素**：

$$
u(k) = u^*(k)
$$

下一周期用新的状态反馈 $x(k+1)$ 重新求解整个优化问题，形成反馈闭环。这体现了 MPC 的核心优势：虽然开环规划了 $N_c$ 步，但通过逐拍重优化引入了反馈校正。

---

# 第二部分：库实现与使用

## 8. 架构概览

MPC 库遵循控制器库统一范式（与 LQR、PID 一致）：

- **单一配置结构体** `Mpc_config` — 包含构建 MPC 所需的全部信息（模型、时域、权重、约束）
- **动态内存分配** — `Create_Mpc(&cfg)` 在堆上创建控制器对象，返回句柄 `Mpc*`
- **Set/Get 接口** — 状态、参考、权重通过独立函数设置，职责分离
- **`MPC_Calc(obj)` 入口** — 每拍调用一次，内部自动完成预测矩阵构建 → QP 构建 → Hildreth 求解 → 结果提取，返回求解成功与否

```
┌──────────────┐      Create_Mpc()       ┌──────────────────────┐
│  Mpc_config  │ ──────────────────────▶ │        Mpc*          │
│  (栈上临时)   │                         │  config + 全部运行时   │
│              │                         │  状态 (堆上持久)      │
└──────────────┘                         └──────────────────────┘
                                                  │
                      每拍调用前:                   │
                      Mpc_Set_State(obj, x0)       │
                      Mpc_Set_Reference_Traj(...)  │
                      Mpc_Set_Weight_Traj(...)     │
                                                  ▼
                                          MPC_Calc(obj)
                                                  │
                                                  ▼
                              Mpc_Get_Control(obj, &u)
                              Mpc_Get_Prediction(obj, &y)
```

---

## 9. 数据结构

### 9.1 Mpc_config — 配置结构体

创建 MPC 控制器时一次性传入，包含全部初始化信息。

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `A` | float[] | nx×nx | 状态转移矩阵 |
| `B` | float[] | nx×nu | 输入矩阵 |
| `C` | float[] | ny×nx | 输出矩阵 |
| `nx` | uint8_t | — | 状态维度 |
| `nu` | uint8_t | — | 输入维度 |
| `ny` | uint8_t | — | 输出维度 |
| `Ts` | float | — | 采样周期 (s) |
| `Np` | uint8_t | — | 预测时域 |
| `Nc` | uint8_t | — | 控制时域 |
| `Q` | float[] | ny×ny | 输出误差权重（对角矩阵） |
| `R` | float[] | nu×nu | 控制量权重（对角矩阵） |
| `u_min` | float[] | nu | 控制量下界 |
| `u_max` | float[] | nu | 控制量上界 |
| `has_input_bound` | uint8_t | — | 是否启用输入约束 |

### 9.2 Mpc — 控制器对象

`Create_Mpc()` 在堆上分配，包含 config 副本及全部运行时状态。

| 字段 | 维度 | 对应算法符号 | 含义 |
|------|------|-------------|------|
| `config` | — | — | 配置副本 |
| `Phi[]` | Np·ny × nx | $F_x$ | 自由响应预测矩阵 |
| `Omega[]` | Np·ny × Nc·nu | $\Theta_u$ | 受迫响应预测矩阵 |
| `H[]` | n_du × n_du | $H$ | QP Hessian 矩阵 |
| `H_inv[]` | n_du × n_du | $H^{-1}$ | H 的逆矩阵 |
| `G[]` | n_du | $g$ | QP 梯度向量 |
| `A_cons[]` | n_cons × n_du | $A_{cons}$ | 约束矩阵 |
| `b_cons[]` | n_cons | $b_{cons}$ | 约束上界 |
| `lambda[]` | n_cons | $\lambda$ | 拉格朗日乘子（对偶变量） |
| `num_cons` | — | $n_{cons}$ | 有效约束行数 |
| `x0[]` | nx | $x(k)$ | 当前状态 |
| `y_ref_traj[]` | Np·ny | $Y_{ref}$ | 未来 Np 步参考轨迹 |
| `y_weight_traj[]` | Np·ny | — | 时变权重轨迹 |
| `u_opt[]` | Nc·nu | $U^*$ | 最优控制序列 |
| `u_pred[]` | nu | $u(k)$ | 当前拍最优控制量 |
| `y_pred[]` | Np·ny | $Y_{pred}$ | 预测输出序列 |
| `initialized` | — | — | 预测矩阵是否已构建（缓存标志） |

---

## 10. API 参考与调用流程

### 10.1 创建

```c
Mpc* Create_Mpc(Mpc_config* config);
```

**使用前必须** `memset(&cfg, 0, sizeof(cfg))` 将配置置零，然后逐字段填充。函数内部：
1. `RT_MALLOC(sizeof(Mpc))` 在堆上分配对象
2. `memset(obj, 0, sizeof(Mpc))` 全量置零
3. `obj->config = *config` 拷贝配置
4. 初始化 `y_weight_traj` 为全 1.0（均匀权重）
5. 返回 `Mpc*` 句柄，失败返回 `NULL`

### 10.2 每拍设置接口

调用 `MPC_Calc` 前，按需调用以下 Set 函数：

```c
void Mpc_Set_State(Mpc* obj, const float* x0);
```
将当前状态 $x(k)$ 拷贝到 `obj->x0`，长度 `nx`。**每次 MPC_Calc 前必须调用**。

```c
void Mpc_Set_Reference(Mpc* obj, const float* y_ref);
```
将当前拍参考输出填充至整个预测时域（恒定参考假设），长度 `ny`。适用于参考时不变场景。

```c
void Mpc_Set_Reference_Traj(Mpc* obj, const float* y_ref_traj);
```
设置未来 Np 步的完整时变参考轨迹，长度 `ny × Np`，行优先排列：`[y₀(1), y₁(1), y₀(2), y₁(2), …]`。

```c
void Mpc_Set_Weight_Traj(Mpc* obj, const float* weight_traj);
```
设置未来 Np 步的时变权重，长度 `ny × Np`。值为通道缩放因子，乘到 Q 矩阵上。权重 ≤ 0 时自动回退为 1.0。

### 10.3 核心求解入口

```c
uint8_t MPC_Calc(Mpc* obj);
```

返回 `1` 表示求解成功，返回 `0` 表示失败（Hessian 奇异/非正定，保留上一拍控制量）。

`MPC_Calc` 内部的完整计算流程（[mpc.c:555-601](mpc.c#L555-L601)），对应核心算法各章节：

```
MPC_Calc(obj)
  │
  ├─① obj->initialized == 0 ?
  │    └─ YES → mpc_build_prediction_matrices(obj)    // 见 §3 — 仅首次/模型变更
  │              ├─ 预计算 A⁰, A¹, ..., A^Np
  │              ├─ 构建 Phi (F_x): (Np·ny) × nx
  │              └─ 构建 Omega (Θ_u): (Np·ny) × (Nc·nu)
  │
  ├─② mpc_build_qp(obj)                              // 见 §4, §5
  │    ├─ Y_free = Phi · x(k)                        // 自由响应
  │    ├─ E = Y_ref_traj - Y_free                    // 跟踪误差
  │    ├─ H = 2·(Omegaᵀ·Q̄·Omega + R̄)                 // QP Hessian
  │    ├─ g = -2·Omegaᵀ·Q̄·E                           // QP 梯度
  │    └─ 构建 A_cons·U ≤ b_cons                     // 输入约束
  │
  ├─③ mpc_hildreth_solve(obj)                        // 见 §6
  │    ├─ Gauss-Jordan 求 H⁻¹
  │    ├─ U_unc = -H⁻¹·g
  │    ├─ 快速检查 → 可行则直接返回
  │    └─ Hildreth 迭代 → U_opt = U_unc - H⁻¹·Aᵀ·λ
  │
  └─④ 提取结果                                        // 见 §7
       ├─ u_pred = U_opt[0:nu]                        // 仅第一步
       ├─ y_pred = Y_free + Omega·U_opt               // 预测序列
       └─ return obj->solve_ok                        // 1=成功, 0=失败
```

### 10.4 结果获取接口

```c
void Mpc_Get_Control(Mpc* obj, float* u_out);
```
获取当前拍最优控制量 $u(k) = U^*(0)$，长度 `nu`。

```c
void Mpc_Get_Prediction(Mpc* obj, float* y_pred_out);
```
获取预测输出序列 $Y_{pred} = F_x \cdot x(k) + \Theta_u \cdot U^*$，长度 `ny × Np`。

### 10.5 典型调用模式

```c
// 1. 创建（初始化时执行一次）
Mpc_config cfg;
memset(&cfg, 0, sizeof(cfg));
// ... 填充 cfg 各字段 ...
Mpc* mpc = Create_Mpc(&cfg);

// 2. 每拍循环
while (1) {
    float x0[2] = {pos_fdb, vel_fdb};      // 读取反馈
    Mpc_Set_State(mpc, x0);                // 设置状态
    Mpc_Set_Reference_Traj(mpc, ref_traj); // 设置参考轨迹
    Mpc_Set_Weight_Traj(mpc, w_traj);      // 设置权重（可选）

    if (MPC_Calc(mpc)) {                   // 求解，检查返回值
        float u;
        Mpc_Get_Control(mpc, &u);          // 获取控制量
        apply_torque(u);                   // 施加控制
    }
}
```

---

## 11. 可调维度宏

所有宏定义在 [mpc.h](mpc.h#L25-L35)：

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `MPC_MAX_STATE_DIM` | 2 | 最大状态维度 $n_x$ |
| `MPC_MAX_INPUT_DIM` | 1 | 最大输入维度 $n_u$ |
| `MPC_MAX_OUTPUT_DIM` | 2 | 最大输出维度 $n_y$ |
| `MPC_MAX_PRED_HORIZ` | 40 | 最大预测时域 $N_p$ |
| `MPC_MAX_CTRL_HORIZ` | 8 | 最大控制时域 $N_c$ |

派生宏（由基础宏自动计算，无需手动修改）：

| 派生宏 | 公式 | 含义 |
|--------|------|------|
| `MPC_N_DU_MAX` | Nc × nu | 决策变量总数 |
| `MPC_N_Y_MAX` | Np × ny | 预测输出总行数 |
| `MPC_N_CONS_MAX` | 2×(Nc·nu) | 约束最大行数 |
| `MPC_A_CONS_SIZE` | N_CONS_MAX × N_DU_MAX | 约束矩阵总元素数 |

---

## 12. 关键设计权衡

| 设计选择 | 理由 |
|----------|------|
| **非增量形式** | 决策变量为 $u$ 本身，扭矩直出，无需积分环节；适合电机等可直接控制力矩的场景 |
| **Hildreth QP** | 无需外部 QP 库，迭代简单，适合嵌入式环境 |
| **Gauss-Jordan 求逆** | $n_{du} \leq 8$ 的小矩阵足够精确；大维度需换 Cholesky |
| **因子 2 显式缩放** | 确保 Hildreth 对偶推导的 $P = A H^{-1} A^T$ 与 $\frac{1}{2}x^T H x + g^T x$ 一致 |
| **动态分配 + 句柄模式** | 符合控制器库统一范式（与 LQR/PID 一致），配置清晰，生命周期明确 |
| **预测矩阵缓存** | 模型/配置变更时仅重建一次（`initialized` 标志），后续只重建 QP |
| **时变权重** | 支持逐拍逐通道独立权重，可实现在特定区域（如跳变前）降低控制精度要求 |
| **求解失败保护** | Hessian 奇异时保留上一拍控制量，不更新 `u_pred` |
