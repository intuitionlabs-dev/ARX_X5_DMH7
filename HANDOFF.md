# 交接：接口文档规范化

## 状态

已提交并推送。主仓 `782c301`，sdk 子模块 `bdbfeb6`，两者的 origin 均已同步。

工作区残留三个 C 文件改动（`App/arm_auto.c`、`App/gimbal_arm.c`、`App/gimbal_cmd.c`），内容为夹爪找零判据、夹爪 MIT kp、TD 参数，与本项工作无关，也未提交。

## 全局规范（不在本仓）

1. `~/.claude/skills/interface-doc/`
   - `SKILL.md`：接口文档必须按 ASD-STE100 编写，适用协议说明、API 参考、码表、寄存器表、跨端约定。
   - `references/ste100-rules.md`：逐条规则、句式限制与违规改法。
   - `assets/interface-doc-template.md`：文档骨架。
2. `~/.claude/CLAUDE.md` 与 `/home/zhao/AGENTS.md`（两份逐字节相同）
   - 第四节「文档与注释：禁止口语化，用行业通用术语」：禁用口语、比喻、拟人、夸张；名词取源码与数据手册的命名；精简优先；判据为同领域工程师读到的是结论与约束。

## 本工程文档改动

| 文件 | 变更行数 | 说明 |
| --- | --- | --- |
| [libs/frame/protocol_doc.md](libs/frame/protocol_doc.md) | 478 | 帧格式、码表、校验、状态机、心跳超时 |
| sdk `README.md` | 412 | 概念、术语表、注意事项 |
| sdk `API.md` | 272 | 逐符号参考、异常树、速查 |
| [libs/encos_motor/README_ENCOS.md](libs/encos_motor/README_ENCOS.md) | 1 | 反馈新鲜度判据 |
| [libs/gravity_compensation/c/README.md](libs/gravity_compensation/c/README.md) | 1 | 角度归约范围 |
| [encos技术手册转写.md](encos技术手册转写.md) | 1 | Kt 校准枚举 |

改动性质：

- 口语、比喻、拟人替换为工程表述：`夹`（限幅）→ `限幅`，`喂帧` → `保活`／`输入`，`口子` → `链路`／`串口`，`踩坑` → `注意事项`，`兜得住` → `可检出`，`猛拽一下` → `目标阶跃`。
- 术语统一：协议文档正文 `姿态` → `位姿`，与 README／API 对齐。
- 删除无信息量的修饰语与第二人称表述。
- 保留 `夹爪`（域内名词）。

技术事实未变：帧布局、字段偏移、码表、量程、单位、阈值、十六进制常量、代码示例、代码围栏均保持原值。

## 验证

```bash
python3 -m sdk.selftest          # 通过 205，失败 0
python3 -m sdk.tui --selftest    # 通过 48，失败 0
```

- 逐文件与上一提交对拍：数字与十六进制常量无一减少，仅新增少量来自正文澄清的引用。
- 代码围栏成对：16（README）／20（API）／28（protocol_doc）。
- 全部文件 `file -b` 为 UTF-8。
- 两份全局指令文件 `diff -q` 一致。

## 尚未处理

以下文档未过口径，未纳入本次改动：

- [ARX_X5_CAN_FEEDBACK_ROUTING.md](ARX_X5_CAN_FEEDBACK_ROUTING.md)
- [encos电机pySDK通信流程.md](encos电机pySDK通信流程.md)
- [libs/controller/MPC_ALGORITHM.md](libs/controller/MPC_ALGORITHM.md)
- [libs/gravity_compensation/README.md](libs/gravity_compensation/README.md)
- [libs/gravity_compensation/PROVENANCE.md](libs/gravity_compensation/PROVENANCE.md)
- [libs/gravity_compensation/sdk_parameters/README.md](libs/gravity_compensation/sdk_parameters/README.md)
- sdk `config/README.md`
