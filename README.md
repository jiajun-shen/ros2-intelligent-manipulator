# ROS 2 Vision-Guided Intelligent Manipulator

中文名称：基于 ROS 2 的视觉引导智能机械臂抓取系统

这是一个面向机器人初学者的长期学习项目。项目将从纯 Python 的二维二连杆机械臂开始，逐步升级为 ROS 2、Gazebo、MoveIt 2 和虚拟相机协同工作的抓取与放置系统。

> 当前状态：Version 1.0 / Step 1 - 工程骨架。机械臂数学模型、绘图和交互功能尚未实现。

## 项目目标

在普通电脑上通过纯软件仿真完成机械臂建模、规划、控制、视觉定位、任务执行和失败恢复。项目不依赖真实机械臂，也不会用语言模型直接生成危险的关节控制命令。

## 五代路线

| 版本 | 主题 | 当前状态 |
| --- | --- | --- |
| 1.0 | Python 二维机械臂基础模型 | 进行中：基础骨架 |
| 2.0 | 逆运动学、轨迹、PID 和避障 | 未开始 |
| 3.0 | ROS 2、Gazebo、MoveIt 2 三维抓取 | 未开始 |
| 4.0 | 虚拟相机视觉引导抓取 | 未开始 |
| 5.0 | 任务规划、失败检测和自动恢复 | 未开始 |

每一代都在上一代代码和知识上升级。只有当前版本通过测试并完成文档后，才进入下一代。

## 当前步骤包含什么

- 使用 `src` 布局创建可安装的 Python 包 `manipulator_2d`。
- 使用 `pyproject.toml` 管理项目元数据和开发工具。
- 使用 `pytest` 验证安装后的包能够正常导入。
- 使用 Ruff 检查基础代码格式和常见问题。
- 准备 Windows、WSL2 和 Ubuntu 安装说明。

当前步骤**不包含**正运动学、逆运动学、绘图、ROS 2 或机械臂控制代码。

## 快速开始

Windows PowerShell：

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest
```

如果电脑安装的是 Python 3.12，可以把第一条命令中的 `-3.11` 改为 `-3.12`。

WSL2 或 Ubuntu：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest
```

更完整的环境说明见 [`docs/setup.md`](docs/setup.md)。

## 目录结构

```text
ros2-intelligent-manipulator/
├── docs/                       # 安装说明和后续设计文档
├── examples/                   # 可以直接运行的示例（后续步骤加入）
├── src/
│   └── manipulator_2d/         # Version 1.0/2.0 的 Python 包
├── tests/                      # 自动化测试
├── .gitignore                 # 不提交缓存、虚拟环境和 ROS 构建产物
├── LICENSE                    # MIT 开源许可证
├── pyproject.toml             # 安装、依赖和工具配置
└── README.md                  # 项目入口说明
```

## 验证

```bash
python -m pytest
ruff check .
ruff format --check .
```

正常情况下，最小测试应当全部通过，Ruff 不报告错误。

## 下一步

定义二维二连杆机械臂的数据模型，包括连杆长度、关节角和关节限制；暂时仍不实现正运动学。

## License

本项目使用 [MIT License](LICENSE)。
