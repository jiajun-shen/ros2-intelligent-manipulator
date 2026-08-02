# 开发环境安装

当前步骤只需要普通 Python 环境，不需要安装 ROS 2、Gazebo 或 MoveIt 2。

## Python 版本

- 最低版本：Python 3.11
- 已验证版本：Python 3.12
- 推荐做法：为项目创建独立虚拟环境 `.venv`

## Windows PowerShell

在项目根目录运行：

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest
```

如果 `py -3.11` 找不到解释器，先运行 `py --list` 查看已安装版本，再选择 3.11 或 3.12。

如果 PowerShell 阻止激活脚本，可以不激活环境，直接运行：

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest
```

## WSL2 或 Ubuntu

```bash
sudo apt update
sudo apt install python3 python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest
```

## 为什么使用可编辑安装

`python -m pip install -e ".[dev]"` 会把当前项目以可编辑方式安装。以后修改 `src/manipulator_2d/` 中的代码，不需要每次重新安装；`dev` 还会安装测试和代码检查工具。

## 本步骤不安装的工具

NumPy 和 Matplotlib 会在真正开始机械臂数学模型与绘图时加入。ROS 2、Gazebo 和 MoveIt 2 要到 Version 3.0 才安装，并会先确认 Ubuntu 与 ROS 2 的官方兼容关系。
