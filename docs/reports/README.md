# 本地研究报告

`research_draft/` 由原顶层 `report/` 整体迁入，源文件、参考文献、图件和已有 PDF 均保留，内容未修改。
目录内原 README 是历史说明，其中的 `cd report` 现在应替换为：

```bash
cd docs/reports/research_draft
typst compile report.typ
```

报告源文件和整理过的图件可纳入版本管理；编译出的 `report.pdf` 和 `page*.png` 默认忽略。
`research_draft/` 不参与 MkDocs 发布；文档站点生成的历史证据页仍正常保留。
报告中的历史实验路径表示原始数据来源，不因目录迁移而改写实验结论。
