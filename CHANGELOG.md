# Changelog

## Unreleased

- 项目口径由"三层研究主线"调整为"以论文复现为基础、适当拓展"（纯文档变更）：
  README 与文档首页开头明确本项目复现 Ren & Shan（2026）与 Kim et al.（2025）两篇
  论文并在此基础上适当拓展；三层导航表的"核心问题"列改为组织定位（模型与接口＝
  仿真平台基线，规划与柔顺控制＝论文复现与控制实验，完整装配系统＝HexFrame 完整
  装配演示），HexFrame 标注为演示、petal_designer 标注为接口交互调形工具。
  《当前研究主线》（research_focus.md）与《研究范围与三层职责》（research_scope.md）
  合并为单页《研究组织与范围》（research_focus.md 删除，Git 可恢复），保留三层职责表、
  证据使用规则（结果优先级、分开归因、有限代表点诊断）与初始有效域；删除主线问题
  宣言、收束过程记录、停止条件等过程性内容，2 N·m 载荷阈值解释移入《接触阶段控制
  实验》判据节，被删结果引用均已有对应层文档与历史证据索引承接。《项目演化》将主线
  问题改为历史口径；AGENTS、CONTRIBUTING、MkDocs 导航（合并导航条目、"三层研究"
  改"三层结构"）及各层文档同步改口径。未改动任何代码、场景、资产、冻结结果或验收
  阈值。

- 将绕轴/横向释放实验入口由 RQ 编号命名改为描述性命名（按用户要求，去除
  AI 自拟的 Research Question 编号）：`experiments/control/rq1_yaw.py` →
  `yaw_release.py`、`rq2_lateral.py` → `lateral_release.py`，默认输出目录与
  文档命令示例同步更新（`runs/yaw_release_*`、`runs/lateral_release_*`）。
  README、AGENTS、CONTRIBUTING 与 docs 各页的 RQ1/RQ2 术语改为"绕轴释放实验/
  横向释放实验"；测试引用与 manifest 断言同步改名。纯重命名，协议、判据与
  冻结数据不变；源码指纹因此变化，旧运行目录按原口径解释，新运行使用新目录。

- 移除 SDF 对照实验线（SDF 仅有临时测试价值，按用户决定清理）：删除三份报告
  （`results/angle1_sdf_compare_20261005.md`、`crown_sdf_onset_20261005.md`、
  `crown_sdf_dynamic_20261005.md`）、三个实验入口（`experiments/models_interfaces/`
  下的 `angle1_sdf_compare.py`、`crown_sdf_onset_check.py`、`crown_sdf_dynamic_check.py`）、
  三项对应测试与三份 `docs/evidence/` JSON。`docs/models_interfaces.md` 的 SDF 对照
  小节、`docs/research_focus.md` 的"有限 SDF 对照"过程记录、`docs/historical_evidence.md`
  对应行及 AGENTS.md 的 SDF 条目同步移除；2 N·m 载荷阈值的解释仍保留在研究主线页。
  冠形 STL 资产与 `iiwa14_compliant_insertion.yaml` 场景被现有测试与场景引用，未删除。

- 清理 `results/` 中五个零引用且被取代的过程性报告（Git 历史可恢复）：
  `compliant_insertion_iiwa14.md`（RQ1 前身组合对接实验，被绕轴释放报告取代）、
  `contact_diagnostics_iiwa14.md`（冠形诊断过程记录，冠形线已关闭）、
  `petal_designer_validation.md`（调形工具检查，非实验证据）、
  `petal_dock_replacement_review.md`（PetalDock100 接入评审，决策已执行）、
  `petal_angle1_blend030_validation.md`（候选三轮摘要，被
  `petal_angle1_blend030_20261003/report.md` 完整取代）。
  全库（文档、代码、mkdocs、results 内部链接）确认无引用；保留文件不动。
  其中 `petal_angle1_blend030_validation.md` 曾被 `tests/test_generated_outputs.py`
  以硬编码路径防回归守卫，该断言随文件删除同步移除（工具本身已无写入路径）。

- 第三轮文档收尾：切断"RQ1/RQ2 结果自动导向 HexFrame 系统集成"的旧叙事。《接触阶段刚度实验结果与证据边界》（原《控制实验结果与系统集成选择》）删除"选择绕轴＋XY 释放作为系统集成候选策略"与"下一项是候选控制适配"表述，改为"结果说明特定方向恢复刚度与卡滞现象存在值得关注的关系，不自动决定 HexFrame 控制方案；RQ1/RQ2 当前阶段工作到此结束"；HexFrame 集成技术约束整节移入《系统验证》并通用化为"柔顺控制器接入 HexFrame 的通用约束"（面向任意接触控制器，不针对特定刚度策略），系统页同时明确"HexFrame 是完整装配系统验证平台，不是 RQ1/RQ2 的最终实验场景"；《研究主线》工作顺序改为 RQ1/RQ2 与 HexFrame 独立推进，注明系统控制器选择须结合控制点、携带负载、执行器接口、反馈链路与完整流程单独评估；《项目演化》三层名称统一为"模型与接口—规划与柔顺控制—完整装配系统"，HexFrame 定位改为"将单接口对接能力扩展到完整装配任务"；NRIC 不实现的理由改为主线表述，不再引用"接触柔顺分配"；卡滞机制解释统一为"观察事实＋恢复力可能参与"的假设口径。RQ1/RQ2 全部数据、失败记录、判据与阈值原样保留；未新增算法、实验或研究问题。

- 第二轮文档收缩：撤下上一轮过度提升的“选择性柔顺 / geometry-informed selective compliance”理论包装，项目主线回到“柔顺对接与完整装配”。删除理论页 `docs/theory/selective_compliance.md`（RCC / Salisbury / Mason / Raibert-Craig 文献链随之移除），MkDocs 导航与报告 hook 同步清理；《接触阶段控制实验》（原《柔顺控制算法研究协议》）重排为 目的 / 固定条件 / RQ1 / RQ2 / K=0 解释 / 判据 / 证据边界 / 已知缺口，RQ1/RQ2 定位为接触阶段的小型刚度机制实验（绕轴刚度与相位自对准、横向刚度与自定心），明确刚度是受控实验变量、不是新的刚度设计方法，K=0 是移除恢复力的极端实验点而非最优性结论，证据口径统一为“该方向恢复力可能参与观察到的卡滞机制”；三层第二层更名为“规划与柔顺控制”（覆盖方法库复现与局部控制实验），第三层为“完整装配系统”；README、文档首页、《研究主线》《研究范围》《项目演化》按新主线改写（研究主线核心问题改为“建立可复现柔顺对接仿真并连接规划、控制与完整装配”，工作分解为方法复现、接口整理、小型机制实验、HexFrame 集成四项）；论文对照扩展表改为“接触阶段绕轴/横向刚度机制实验”。几何研究与控制研究继续分开归因；未新增算法、实验、研究问题或结果，历史协议、参数、判据与 FAIL 状态原样保留。

- 文档按学术主线重构（纯文档变更，未改任何结果、阈值、场景或代码）：新增《项目演化》（真实历史：论文复现 → 接口扩展 → HexFrame → 收束，区分项目演化与研究组织两个维度）与理论文档《选择性柔顺》（RCC / 主动刚度 / 混合位置力控制 / SE(3) 阻抗四条来源、自对准与任务保持子空间表述、明确 K=0 是机制对照极端点而非理论最优，并注明"柔顺分配"表述来自收束期间的讨论与文档整理、非立项目标且晚于实验假设）；《研究主线》改为 2026-10-04 实证收束问题置顶、"接触后柔顺分配"降为其下的机制表述、RQ1/RQ2 定位为实证主线下的受控对照、2026-10-04/05 过程记录下沉为附录；三层表第二层（对接方法与接触柔顺）核心问题扩为两篇主要参考文献的问题（Ren & Shan 规划×柔顺统一、Kim et al. SE(3) 六自由度阻抗系统设计）加当前柔顺分配机制表述；《研究范围》明确三层为当前研究组织方式而非项目开发顺序；《控制研究协议》新增“为什么研究这些自由度”与 released/K=0 定位章节，原参数、场景、判据与 FAIL 状态原样保留；《论文对照》降级为历史方法来源记录，新增 Kim T-RO §III-A 已实现、§III-B NRIC 未实现的明确状态与取舍理由（NRIC 不实现、不新增 nric 代码或矩阵）；新接口 9/9 与 RQ2 7/9 明确为几何研究与控制研究分开归因；README、文档首页与 MkDocs 导航改为“项目与研究 / 理论基础 / 三层研究”阅读路径。

- 清理 `runs/` 中零引用的纯构建与迁移脚手架（17 项，约 100 MB）：旧 MkDocs 输出 `docs_site/`、迁移期 `generated_archive/`、`archive_workspaces/`、`archived_outputs/`（已删除 orbital_showcase 的旧输出）、`generated_policy_20261004_*`、`directory_cleanup_20261004_*`、基线 16a54d4 的 `restructuring_20261004_{model_baseline,precheck,checks}`、smoke 输出及其 source_check JSON，以及无 rollout 的 `angle1_nine_20261005/`、被 v3 取代的 `hexframe_interface_precheck_20261005/`、`narrow_default_check_20261004/`。冻结报告明文保留的首次/INCOMPLETE 目录与重构等价性对照核实后未动；`runs/` 不进入 Git，删除前整体备份于本机 `~/runs_cleanup_backup_20261005.tar.gz`，逐项记录见 `runs/README.md`。
- 文档输出恢复 MkDocs 默认 `site/` 目录：删除 `mkdocs.yml` 中的 `site_dir: runs/docs_site`，`runs/` 回归纯科研运行产物；Pages workflow 补 `actions/configure-pages@v5`、升级 `upload-pages-artifact@v4` 并增加 `site/index.html` 构建后检查。main 上此前发布失败的直接原因是旧版 `docs/_repository_reports.py` 未重写 `development_plan.md` 中的 `../results/` 链接，strict 构建中止；本分支 hook 已将该页纳入重写名单，本地 strict 构建通过。
- 整理贡献与协作外围：CONTRIBUTING.md 重组为环境安装、三层实验、验证与研究约定；AGENTS.md 的 CHANGELOG 规则放宽为仅记录影响使用方式、协议、默认行为或阶段性结果的改动；Issue/PR 模板改为面向中文用户的精简结构。README 主要二级标题加入少量 Emoji 并补在线文档链接；三层实验 README 改为短中文说明；统一“仿真研究仓库”表述（mkdocs site_description 与 docs 首页）。

- 将 GLM 编码委派内容清理出仓库：删除 `docs/glm_bridge.md` 与 mkdocs 导航条目，AGENTS.md 移除“GLM 编码委派”章节及外部编码代理条目；桥接脚本此前已迁至本机 `~/.codex/mcp/` 独立维护，仓库不再保存其使用规则。

- 减法重构：`archive/` 整体退出运行路径（pytest 只收集 `tests/`，目录删除，Git 历史可恢复）；删除 `experiments/orbital_showcase/`、`experiments/hexframe_validation/`、`experiments/_archive_compat.py` 及根目录 5 个转发 wrapper（`run_docking.py`、`petal_capture_grid.py` 等）、`check_env.py`、一次性冻结工具 `petal_guidance_freeze.py`。引用方改为直接使用分层正式入口；未新增任何替代目录或清单。

- 删除一次性过程文档（迁移计划/验证、目录清理、handoff、阶段收尾等 7 篇）与 `docs/evidence/` 中 4 个过程 JSON；入口表与生成数据位置并入《实验复现手册》。`results/` 移除 stage closeout / git 审计 JSON、历史 orbital 证据与重复 manifest（`results/petal_angle1_blend030_20261003/` 保留报告、图件、摘要与步长记录）；冻结报告原文未改写。

- AGENTS.md 压缩为 8 条工作规则加注释规范；README 重写为安装、三层实验、结果与文档导航。

- `assembly/simulation.py` 关键英文注释改为解释物理含义的中文（接触导纳、伺服阻尼、落座门限、weld 锚点）；AST 对比确认除 docstring 外零行为变化，未重跑大型矩阵。

- pytest 增加 `pythonpath = ["."]`：修复最后几个本地提交新增测试在裸 `uv run pytest` 下无法导入 `experiments` 包的问题（此前仅在仓库根已入 sys.path 的环境中通过）。

- 移除三层迁移的时点检查工具 `scripts/check_restructuring.py`：其冻结基线（`16a54d4`）已被后续功能提交合法越过，检查不再可运行；`scripts/` 目录随之取消，验证口径保留在带日期的记录文档中，工具本体存于 Git 历史。

- 将 GLM 编码桥接移出研究仓库：`glm_mcp.mjs` 及其 14 项边界测试迁至本机 `~/.codex/mcp/`，Codex 注册路径同步更新，`scripts/` 仅保留迁移检查工具；桥接行为与文档（`docs/glm_bridge.md`）中的本机接入说明不变。

- 将 PetalDock 交互加固移植到三层项目结构：保持 `runs/designs/petal_guidance` 输出与冻结候选来源，补齐预览版本、保存原子发布、输入和协议校验；默认半角1°/过渡0.3，移除普通界面的径向参数。

- 保存新对话交接记录：明确已完成结果、真实接口配合、冠形初始化/切换限制、视频和数据路径、GLM 实际连接边界及下一步有限控制任务。

- 完成 HexFrame 两种接口的有限移植及八工况回放：angle1_blend030 名义完整验收与独立审计通过；保留三点偏差失败及冠形存储预载/提前接触问题，附真实配合残差与来源核对。

- 增加 HexFrame 成功/未完成记录的原速视频入口，提供整体视角与接口特写供人工评判，并记录输入哈希；不重新仿真或修改验收。

- 补充 HexFrame 保存状态的真实接口配合残差：区分预设轨迹偏差与实际接口横向、轴向、倾斜及等效相位残差，未重跑或修改门槛。

- 完成 HexFrame 实际接收位姿四点验证：名义完整验收/独立审计通过，2 mm 横向、5° 绕轴及组合均就位超时；保存真实偏差注入、原门禁和独立轨迹复核。

- 澄清原冠形 2 N·m 载荷阈值缺乏已记录的物理依据：当前报告作为历史参考提示，分别解释任务完成与载荷观测；保留原始数据、旧评价输出和控制配置。

- 完成原冠形 SDF 三工况动态核对：末段平稳 3/3，落座候选 2/3；保留三组短暂绕轴净接触力矩超限，明确提前接触不等于动态失稳。

- 核对原冠形 STL 的安装角、挂载偏置及完整面片首触；四姿态静态比较区分正常导向接触与额外 SDF 偏差，保留原资产和默认控制。

- 将已验证的 angle1_blend030 1 ms 凸块任务设为默认；增加单进程分项计时与仅三点的花瓣 SDF／原止挡对照入口，不自动替换碰撞模型。完成后凸块 3/3 通过、SDF 0/3 通过，记录提前接触与穿透异常并保留凸块默认。

- 新增 angle1_blend030 九工况任务验证入口；自由空间提速，物理步长、控制周期和反馈延迟统一为 1 ms，保留原落座/误差/载荷门槛并记录运动限制与穿透量；九点全新运行及逐步审计均 9/9 通过。

- 单接口默认采用 narrow（窄平顶＋角向斜坡），保留 original 场景并固定历史研究入口；HexFrame 仍用旧模块接口，下一步保留导纳基线验证偏差。

- 核对现有 RQ1/RQ2 主结果与九组配对配置，保存轻量来源清单；保留两点组合卡滞与效率结论缺口，明确绕轴＋XY 释放候选的 HexFrame 集成边界。

- GLM 桥接增加独立 Start Plan 权益查询，明确区分套餐、发放量、剩余额度和模型连接；记录独立模型调用被平台拒绝的实际状态。
- 增加独立 GLM 编码 MCP：复用本机 ZCode 认证、限定文件范围并生成可审查提案；不替换主模型。

- 收束研究主线至对接完成、误差、效率与失败边界；精细步长和候选接口转为补充证据。
- 新增根目录 AGENTS.md 与统一注释规范，以中文用户理解为目标，解释首次出现的术语并说明职责、单位与副作用。
- 整理本地补充研究数据，保留原路径兼容与逐文件校验；清理可再生成构建和缓存。

- 按模型与接口、柔顺控制研究、完整对接/装配整理阅读路径与实验编排。
- RQ1/RQ2、误差范围和几何研究采用独立入口，共享单次试验与遥测实现。
- RQ2 主配对、数值复核、减速因素分别运行；历史命令协议保留。
- 明确 HexFrame 当前关节伺服/接触导纳配置及尚未集成 SE(3) 的范围。
- 增加贡献指南、问题/PR 模板和严格文档检查；保留历史资产、证据与用户绘图工作。

此次结构变更没有修改控制公式、物理参数或验收阈值，也未新增科研结论。
实际迁移和检查结果见 `docs/restructuring_validation.md`。
