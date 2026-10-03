# 平顶半角1°、斜坡过渡比例0.3

用户于2026-10-03选定的独立仿真候选，保存参数见 `selected_design.json`。
外径100 mm、导向内径64 mm、导向高度18 mm、边缘圆滑1 mm、轴向余量0.4 mm。
平顶总宽2°，每段角向斜坡两端各30%圆滑。未增加径向导面。

由 `experiments/prepare_petal_design.py` 委托原生成器生成，每侧500个导面凸单元；
安装件、止挡与46.4 mm名义法兰间距沿用原结构，整体质心和惯量按导面网格差值更新。
此候选没有新的制造STEP文件。保存参数与模型元数据中的 `UNVALIDATED_DESIGN`
说明参数选择本身不代表验证结论；实际动态结论独立保存在对应实验报告中。

场景：`scenes/iiwa14_petal_angle1_blend030.yaml`。
数据：`runs/petal_angle1_blend030_20261003`。
