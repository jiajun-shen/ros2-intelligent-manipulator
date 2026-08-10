# Version 1.0 Examples

这里包含三个由简到繁的可运行示例。首次运行前请先完成 [`docs/setup.md`](../docs/setup.md) 中的环境安装。

## 推荐：完整交互 Demo

文件：[`run_interactive_arm_2d.py`](run_interactive_arm_2d.py)

它会显示：

- 二维 2R 机械臂；
- `q1`、`q2` 角度滑块；
- 鼠标点击目标及其可达性；
- 末端执行器运动轨迹。

在 VS Code 中打开这个文件，确认解释器为项目 `.venv`，然后点击右上角的 `Run Python File`。

## 静态绘图 Demo

文件：[`run_arm_2d.py`](run_arm_2d.py)

该示例创建一个固定构型并绘制机械臂。它适合检查数据模型、正运动学和静态绘图之间的关系，不包含滑块。

## 可达性 Demo

文件：[`check_reachability.py`](check_reachability.py)

该示例不会打开图形窗口，而是输出机械臂理想工作空间的内外半径，并判断几个目标点是否可达。

## 命令行运行方式

安装并启用项目环境后，可以运行：

```bash
python examples/run_interactive_arm_2d.py
python examples/run_arm_2d.py
python examples/check_reachability.py
```

Version 1.0 的目标点击只进行可达性判断。机械臂不会自动移动到目标，因为逆运动学将在 Version 2.0 中学习和实现。
