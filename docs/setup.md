# Version 1.0 开发环境安装

Version 1.0 和后续 Version 2.0 使用普通 Python 环境，不需要安装 ROS 2、Gazebo 或 MoveIt 2。

## Python 版本

- 最低版本：Python 3.11
- 已验证版本：Python 3.12
- 推荐环境：项目根目录中的独立虚拟环境 `.venv`

## 当前电脑直接运行

当前项目已经有可用的 `.venv` 时，不需要重新安装，也不需要先激活虚拟环境：

1. 在 VS Code 中打开整个 `ros2-intelligent-manipulator` 文件夹。
2. 查看 VS Code 右下角显示的 Python 解释器。
3. 如果不是项目的 `.venv`，按 `Ctrl+Shift+P`。
4. 选择 `Python: Select Interpreter`。
5. 选择路径中包含 `.venv` 的 Python。
6. 打开 `examples/run_interactive_arm_2d.py`。
7. 点击右上角的 `Run Python File` 三角形按钮。

## Windows 首次安装

以下命令不要求激活虚拟环境，每一条都明确使用项目自己的 Python：

```text
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

如果电脑只有 Python 3.12，把第一条命令中的 `-3.11` 改为 `-3.12`。

安装后按照上一节在 VS Code 中选择 `.venv`，以后运行 Demo 只需点击 `Run Python File`。

## WSL2 或 Ubuntu 首次安装

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip python3-tk
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

`python3-tk` 用于确保 Matplotlib 可以打开桌面图形窗口。

## 依赖说明

| 依赖 | 用途 | 所属范围 |
| --- | --- | --- |
| Python `math` | 三角函数和距离计算 | 标准库 |
| Matplotlib | 绘图、滑块和鼠标事件 | 运行依赖 |
| pytest | 自动化测试 | 开发依赖 |
| Ruff | 代码检查和格式检查 | 开发依赖 |

本项目没有直接使用 NumPy 实现机器人运动学。Matplotlib 自身可能间接安装 NumPy，但 Version 1.0 的核心公式使用 Python 标准库完成。

## 为什么使用可编辑安装

```text
python -m pip install -e ".[dev]"
```

`-e` 表示可编辑安装。修改 `src/manipulator_2d/` 后不需要反复重新安装；`[dev]` 会额外安装 pytest 和 Ruff。

## 验证环境

```text
python -m pytest
ruff check .
ruff format --check .
```

Version 1.0 的正常结果是 36 项测试通过，Ruff 不报告错误。

## 常见错误

### `ModuleNotFoundError: No module named 'matplotlib'`

原因通常是 VS Code 使用了系统 Python，而不是项目的 `.venv`。重新执行 `Python: Select Interpreter` 并选择 `.venv`。

### `ModuleNotFoundError: No module named 'manipulator_2d'`

说明当前解释器中尚未执行可编辑安装，或 VS Code 选择了另一个解释器。先安装项目，再重新选择 `.venv`。

### 点击运行后没有图形窗口

确认运行的是 `examples/run_interactive_arm_2d.py`，并检查是否有旧的 Matplotlib 窗口隐藏在其他窗口后面。在 WSL2 或 Ubuntu 中还要确认已安装 `python3-tk` 并且桌面显示可用。

### 刚启动时看不到蓝色轨迹

初始轨迹只有一个点，与末端执行器标记重合。拖动任一角度滑块后，蓝色轨迹才会形成线段。
