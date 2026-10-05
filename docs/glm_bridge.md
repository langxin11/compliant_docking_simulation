# GLM 编码桥接

此桥接让当前 Codex 保持主代理，GLM-5.3 或 GLM-5.3-Flash 负责有明确文件边界的编码提案。
它是本项目的本地 MCP 服务，不是 ZCode 官方编码 MCP，也不替换当前 Codex 主模型。

## 本机接入

服务名为 `glm_coder`，入口 `scripts/glm_mcp.mjs`，需要 Node.js 22 或更新版本。
已在本机 Codex 用户配置中注册，并设置本项目的绝对路径 `GLM_WORKSPACE`。
重启 Codex 或重新加载 MCP 后，让主代理调用 `glm_status` 检查连接。
本会话启动时已加载的工具列表不会因文件配置修改而立即增加工具。

认证优先读取环境变量 `BIGMODEL_API_KEY`，否则读取本机 ZCode 的唯一 BigModel 编程套餐账户。
现有 ZCode `enc:v1` 凭据格式在内存解密；不输出、复制或保存明文密钥。
若凭据格式改变、存在多个账户或登录失效，明确报错，需要重新登录或在本机设置环境变量。
用户级 Codex 配置中的服务条目不含 API Key；原配置已备份到本机 `.codex/backups/`。

当前实现固定使用 BigModel Coding Plan 的 Chat Completions 接口，与 Codex 直接更换模型时的
Responses 接口是两条不同的路径。本桥接不使用 ZCode CLI 的工具执行循环。
参考：[ZCode 账号与接口](https://zcode.z.ai/cn/docs/configuration)、
[Codex MCP 配置](https://developers.openai.com/codex/mcp)。

## 三个工具

| 工具 | 用途 | 参数 |
|---|---|---|
| `glm_start_status` | 独立查询 Start Plan 有效权益，不调用模型 | 无参数 |
| `glm_status` | 检查 Coding Plan 认证；可发极小请求验证实际连接 | `probe` 默认 false；`model` 可选 |
| `glm_code` | 读取指定源码，生成完整替换提案 | `task`、`files` 必需；`model` 默认 `glm-5.3` |

例如将注释任务交给 Flash：

```json
{
  "task": "为指定函数补充中文说明，解释用途、单位与边界，保持执行逻辑不变",
  "files": ["src/compliant_docking/research/cases.py"],
  "model": "glm-5.3-flash"
}
```

主代理只提供任务所需的 1–12 个文件。输入同时附带根目录及对应子目录的 AGENTS 规则。
既有文件或新文件均须由调用者指定；桥接拒绝目录穿越、符号链接、凭据路径和历史运行/冻结目录。
指定文件和代理规则会发送到 BigModel 进行编码；其他项目文件和本机登录文件不会作为任务上下文发送。

GLM 返回中文摘要和文件完整内容。桥接校验提案未扩大文件范围、未重复文件、未截断，
并核对模型处理期间源码未变化。提案保存到 `runs/glm_bridge/<随机编号>.json`，
包含输入哈希、模型与 token 用量；原始源码不由桥接修改。
主代理读取提案、审查改动、应用并执行任务相关检查。桥接不会执行模型返回的命令。

## Start Plan 与 Coding Plan 分开处理

`glm_code` 当前使用 Coding Plan。`glm_start_status` 复用 ZCode 登录中的 JWT，
通过官方 `billing/current` 只读查询 Start Plan；它不使用 Coding Plan API Key。
有效权益、发放总量、剩余额度和模型调用成功分别判断，不能互相替代。

2026-10-04：Start Plan 权益查询成功，服务端返回有效的 GLM-5.3-Flash 权益。
返回的 100,000,000 token 是该权益的发放总量，接口没有给出实际剩余量；
工具因此返回 `remainingUnits: null` 和 `remainingUnitsKnown: false`。
本机模型目录中 Start Plan 包含 Flash，而 GLM-5.3 使用 Coding Plan，不能静默替换。

已核对本机 ZCode 3.14.4 的官方认证实现，并通过其随附客户端
`workspace/generateText` 进行独立调用验证：服务端拒绝请求，消息为
`request has been blocked due to unusual activity.`。
这不是 Coding Plan 的 429 额度错误，也不能据此认定 Start Plan 额度已用完。
目前 Start Plan 的模型调用尚未接通，`glm_code` 不会宣称使用了这份额度。
官方桌面客户端内的 Start Plan 是否可用，应在 ZCode 中验证；本桥接的查询成功不代表编码成功。

## 验证与当前限制

2026-10-04：Coding Plan 本机登录已成功认证，两个模型请求均返回 HTTP 429 / 1310（套餐周/月额度达到上限）。
平台提示重置时间为 2026-10-05 15:26:55，未标注时区；这里按平台原文记录。
MCP 工具额度不会抵扣模型调用额度。没有绕过限制或自动使用额度重置卡。
Coding Plan 的真实模型输出与编码端到端仍未验证；该套餐额度恢复后先调用 `glm_status(probe=true)`，
再用一条小型编码任务验证提案生成。失败时工具返回错误，不把连接状态或模拟测试当作编码完成。

离线验证命令：

```bash
node --test tests/test_glm_mcp.mjs
```

14 项检查覆盖凭据格式、路径/符号链接限制、模型选择、输出校验、输入变更、提案持久化、
套餐权益与剩余额度的区分、过期权益过滤、错误脱敏以及真实 stdio 进程的 MCP 握手；不消耗模型额度。
所有提案受 `/runs/` 忽略规则覆盖，不进入 Git。

需要取消本机接入时执行 `codex mcp remove glm_coder`，不会改变主模型或其他 MCP 服务。
