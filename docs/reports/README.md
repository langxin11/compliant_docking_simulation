# 本地研究报告

`research_draft/` 由原顶层 `report/` 整体迁入，源文件、参考文献、图件和已有 PDF 均保留，内容未修改。
目录内原 README 是历史说明。当前在仓库根编译，输出统一写入忽略目录：

```bash
mkdir -p runs/reports/research_draft
typst compile docs/reports/research_draft/report.typ runs/reports/research_draft/report.pdf
```

报告源文件和整理过的图件可纳入版本管理；编译的 PDF 和页面预览写入 `runs/reports/`。
旧命令在草稿目录生成的 PDF 和 `page*.png` 也默认忽略。
`research_draft/` 不参与 MkDocs 发布；文档站点生成的历史证据页仍正常保留。
报告中的历史实验路径表示原始数据来源，不因目录迁移而改写实验结论。
