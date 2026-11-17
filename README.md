# Sci-Sim：双臂实验室 ManiSkill benchmark

面向实验室具身操作的 ManiSkill 工程原型：五张实验桌、每桌独立柜式双臂架台、基于本地 AtomBridge 资产重建的仪器与耗材、四路 RGB-D 和基于 MPlib 的示范采集。包含混匀转移、热循环仪、离心机、清洗玻片和旋涂五个操作流程；清洗与旋涂场景按用户 GIF 程序化建模。

2026-11-14：默认 UR7e + 2F-85 的三个任务在种子 42/43 下均已从 reset 完成实际规划与接触操作；24 项回归检查通过，并采集、回读了一条 424 帧的成功 LeRobot 示范。有限回合检查不代表大规模随机化成功率。见 [基本任务说明](docs/BASIC_TASKS.md) 和 [验证记录](docs/STATUS.md)。

2026-11-15：清洗玻片与旋涂均已通过 seed 42/43 完整操作；当时 42 项回归检查通过，五桌十臂二十路相机总览已验证。

2026-11-16：旋涂场景同桌接入闪蒸，正方形玻片依次完成旋涂与转移闪蒸。最终 seed 42/43 完整回合通过，50 项回归检查通过；视频与验证记录见 [旋涂与闪蒸说明](docs/SPIN_COATING.md)。

## 场景和机器人

| 任务 | 环境 ID | 仪器与成功条件 |
|---|---|---|
| 混匀并转移离心管 | `SciSim-MixTransfer-v0` | Vortex-Genie 2；夹持试管接触混匀头 2 秒，再放入目标架并释放 |
| 操作热循环仪 | `SciSim-ThermalCycler-v0` | Bio-Rad C1000；开盖、放 PCR 板、关盖、旋紧旋钮、接触屏幕启动按钮、完成抽象计时 |
| 操作离心机 | `SciSim-Centrifuge-v0` | Eppendorf 5430；开盖、放样品管和配平管到对置孔位、关盖、启动 |
| 旋涂与闪蒸 | `SciSim-SpinCoating-v0` | 开盖、放玻片、吸褐色液体、滴一滴、归还移液器、关盖、按键旋转；停转后开两盖、转移正方形玻片、关闪蒸盖、按键闪蒸 |
| 清洗玻片与入干燥箱 | `SciSim-SlideCleaning-v0` | 玻片槽入透明烧杯、烧杯入超声清洗槽、按键完成清洗、烧杯返回桌面、打开干燥箱、杯槽一起入箱、关门 |

默认 **UR7e + Robotiq 2F-85**，可切换 `ur5e`、`franka`（Panda）、`xarm6`；四种机械臂均装配同一 2F-85。UR7e 使用 Universal Robots 官方 `config/ur7e`，官方该配置复用 UR5e 外观网格，不是把 UR5e 类改名。

每张桌尺寸 1.4 × 0.9 m、台面高 0.76 m。架台为带脚轮的柜体、开槽顶板和 T 形圆柱支架：一根竖直圆柱加一根左右水平圆柱，二者直径均为 180 mm，略大于当前 UR7e 资产约 151 mm 的基座宽度。水平圆柱轴心高 1.43 m，两端为真实的 45° 斜切面，机械臂底座直接贴合切面；视觉和碰撞使用同一个封闭斜切网格。柜体与桌边间隔约 5 cm；两侧切面中心分别为 `(-0.245, -0.74, 1.43)` 和 `(0.245, -0.74, 1.43)` m，安装轴分别为 `(-1,0,-1)/√2` 和 `(1,0,-1)/√2`，即朝左右外侧、向下 45°。UR7e/UR5e 根坐标系直接位于切面中心；Franka/xArm6 原碰撞网格略低于其局部 Z=0 平面，分别沿安装轴补偿 0.0335/0.533 mm，避免基座网格嵌入架台，实际位姿写入采集清单。照片未提供尺寸，当前尺寸为建模假设；架台保持独立，脚轮仅为几何，采集时柜体固定于地面。

四种机械臂各自的左右初始关节角由 MPlib 在新安装姿态下离线求解，见 `resources/mount_homes.json`。frontcam 位于架台顶部前方、朝向桌面，避免立柱遮挡工作区。

`--layout lab` 是五桌十臂总览；`--layout single` 是单桌双臂任务。总览有每桌四路、共二十路相机，采集使用单桌四路：`frontcam`、`sidecam`、`left_wristcam`、`right_wristcam`。默认 640 × 480，20 Hz，理想渲染深度；不模拟 RealSense 的噪声、遮挡孔洞或主动红外物理。

## Linux 安装

推荐 Ubuntu + NVIDIA GPU + Vulkan，Python **3.12**。CPU PhysX 做接触仿真，GPU Vulkan 做渲染；v0 明确限制每进程一个环境，批量采集用多个独立进程。WSL2 的 CUDA 可用并不保证 Vulkan 渲染可用，需要分别检查。

```bash
# Ubuntu 系统依赖；WSL 内不要安装 Linux NVIDIA 内核驱动
sudo apt-get update
sudo apt-get install -y libvulkan1 vulkan-tools libgl1 libegl1 libx11-6 libxext6 libxrender1 ffmpeg

# 已安装 uv 后，在项目根目录
bash scripts/setup_linux.sh
source .venv/bin/activate
export SCI_SIM_DATA_PYTHON="$PWD/.venv-data/bin/python"
sci-sim prepare-assets
sci-sim import-instruments --source ../atombridge
```

固定接口版本：ManiSkill 3.0.1、SAPIEN 3.0.3、MPlib 0.2.1、LeRobot 0.7.0。后者要求 Python ≥3.12。其余依赖由安装器解析，尚未提供经过部署验证的完整锁文件。

仿真/规划和数据写入使用两个独立环境：MPlib 0.2.1 要求 NumPy <2，而 LeRobot 0.7.0 要求 NumPy ≥2。采集器通过本地子进程写数据，不要将两个 extras 混装。ManiSkill 3.0.1 自带示例策略依赖旧版 MPlib，本项目通过 `constraints/mplib.txt` 明确覆盖到 0.2.1，仅调用自己的专家接口。

本机已安装的 WSL 发行版名为 `SciSim-Ubuntu`，Linux 用户为 `scisim`。在 PowerShell 进入它，再激活项目配置：

```powershell
wsl -d SciSim-Ubuntu -u scisim
```

```bash
cd /mnt/c/Users/15261/Documents/GitHub/sci-sim
source scripts/activate_wsl.sh
sci-sim render --layout lab --output outputs/overview.png
python scripts/validate_scenes.py --output outputs/validation/wsl --steps 100
```

本机 WSL 使用 Mesa llvmpipe **CPU 软件 Vulkan**，不是 RTX GPU 渲染；激活脚本仅在当前 shell 指定其 ICD。无显示窗口的 RGB-D 与截图已实测可用，交互窗口尚未验证。原生 Windows 环境 `.venv-win` 也已通过场景渲染与步进。ManiSkill 官方不承诺 WSL 的 GPU 仿真/渲染支持，正式大规模采集仍应使用原生 Linux GPU，见 [安装说明](https://maniskill.readthedocs.io/en/latest/user_guide/getting_started/installation.html)。

资产会下载到 `assets/upstream/`，组装后的 URDF/SRDF 在 `assets/robots/`，下载地址、SHA256 与模型清单写入 `assets/manifest.json`。可用 `SCI_SIM_ASSET_DIR` 指定资产根目录。保留上游版权与许可文件；不要把仪器近似模型标注为厂家 CAD。

`import-instruments` 只读本地 AtomBridge 的 OBJ、STL、PNG 和 MJCF 层级/材质/关节数据，生成带纹理的仪器 body/lid/knob GLB、耗材 OBJ、碰撞参数和源文件 SHA256 清单。不会执行该项目的 MuJoCo 插件或修改源项目。默认要求这些资产存在；`--instruments procedural` 保留旧版预览模型，但它没有可操作的热循环仪旋钮，不支持完整热循环专家流程。生成文件不纳入版本控制，部署机器需重新导入或合法复制本地生成资产。

## 查看和采集

```bash
sci-sim catalog
sci-sim view --layout lab --robot ur7e
sci-sim render --layout lab --output outputs/overview.png
sci-sim view --task thermal_cycler --layout single --robot franka

# 执行实际规划/接触操作；添加 --render 显示 Linux 上的实时窗口
sci-sim demo --task mix_transfer --seed 42
sci-sim demo --task thermal_cycler --seed 42
sci-sim demo --task centrifuge --seed 42
sci-sim demo --task slide_cleaning --seed 42
sci-sim demo --task spin_coating --seed 42

sci-sim collect --task mix_transfer --robot ur7e \
  --episodes 10 --max-attempts 100 --seed 42 --split train \
  --root datasets/mix_ur7e_train --repo-id local/mix_ur7e_train

sci-sim collect --task thermal_cycler --robot ur7e \
  --episodes 10 --root datasets/thermal_ur7e_train
sci-sim collect --task centrifuge --robot ur7e \
  --episodes 10 --root datasets/centrifuge_ur7e_train
```

每次尝试按种子重新采样器械、试管架和烧杯的 XY 位置（±18 mm），耗材随其支架放置，拒绝重叠和越界布局。v0 不随机化姿态、仪器类型、材料或物体质量。训练/验证/测试使用不同种子区间；不同数据集仍需自行避免重复使用同一 split 和种子序列。

专家使用 RRTConnect 到达预抓取点，通过逐点逆运动学与密集碰撞检查完成笛卡尔接近、插入、铰链弧线及旋钮旋转；TOPP 进行时间参数化。UR 逆解优先保持避开立柱的肩肘分支。双臂分时规划，另一只臂和所有场景碰撞体进入 FCL；手指与被抓取物、携带物与指定支撑体允许接触。携带物保留实际复合碰撞形状，盖子上的旋钮随盖一起规划。持物约束仅用于规划，物理执行依靠夹爪接触。抓取失败、跟踪误差、掉落、双臂碰撞、超时会丢弃回合并记录。

数据默认保存在本地，不自动上传 Hugging Face。目标目录必须不存在；没有覆盖或隐式续写。成功回合不足时命令退出码为 2，并保留已成功的数据与全部尝试报告。

示范专家使用仿真提供的物体位姿、关节角和接触信息，包括插管前的姿态反馈；它不是仅根据相机图像决策的策略。RGB-D 和动作供后续学习使用。

## 数据契约

- `observation.state`：左右臂各自全部关节的 qpos、qvel，包含夹爪被动关节；名称在 feature schema 中。
- `action`：左臂关节目标 + 左夹爪目标 + 右臂关节目标 + 右夹爪目标，单位 rad。UR/xArm 是 14 维，Franka 是 16 维。不是末端 delta pose。
- `observation.images.*`：四路 uint8 RGB，LeRobot 原生 lossless image features。
- `observation.depth.*`：四路 uint16 毫米深度，0 表示无效值，LeRobot depth marker；不是 8-bit 伪彩色深度。
- `observation.camera.*`：每帧保存内参和 world-to-camera OpenCV 外参，包括运动腕部相机。
- 同一行严格为 `obs[t]`、`action[t]` 和该动作之后的 `next.reward/done/success/phase`；在第一个终止成功帧停止。
- `meta/scisim_manifest.json`：版本、任务配置、资产清单和单位约定。
- `meta/scisim_episodes.jsonl`：成功回合随机种子、初始位姿与任务结果。
- `meta/scisim_attempts.jsonl` / `scisim_summary.json`：含失败的全部尝试及总体成功率，避免只报告保留数据的成功率。

LeRobot 加载器将存储的 uint16 深度转换为 float32 张量，数值仍为毫米，未除以 255。`scripts/validate_dataset.py` 与 `scripts/check_dataset.py` 可验证往返一致性；其产物明确标为诊断数据，不属于成功示范。

## 模型边界与下一步

默认仪器保留 AtomBridge 的分部件外观、纹理、主盖铰链及 C1000 压力旋钮；尺寸来自网格测量，未经厂家图纸独立核对。SAPIEN 使用基础碰撞体和任务专用近似，包括开放的反应槽、插管导向孔和机器人抓盖把手。盖子操作限位为 1.65 rad（约 94.5°），旋钮以转角达到 1.4 rad 表示旋紧；该判定不代表真实拧紧扭矩或厂家规定圈数。屏幕按钮按表面法向接触，其他次级锁杆固定。

离心管和 PCR 板外观参考本地 AtomBridge 资产，记录在 `assets/labware/manifest.json`。PCR 板采用源模型的 114 × 76 mm 外廓，夹爪直接夹持短边；这是可操作的缩放模型，并非标准 SBS 尺寸。离心管外廓适配现有 48 mm 高碰撞代理，管盖固定。管架、PCR 板支座和烧杯为程序化模型；导向孔和板槽边框参与碰撞。

混匀是接触时长任务，不求解流体；热循环为抽象计时，不模拟温控；离心机为加载与互锁任务，5430 外观下使用 30 个简化垂直槽位、等质量管和固定转子，不模拟真实转子倾角、高速旋转或分离过程。尚不包含耗材开盖/拧盖、真实实验程序设置、跨桌移动和双臂交接策略。

当前完整任务验证限于默认 UR7e 和记录中的随机种子；其他机械臂已验证场景构造与初始保持，尚未验证这些完整操作策略。正式发布 benchmark 数据集前，需要在目标 Linux GPU 上扩大随机种子覆盖、统计全部尝试的成功率，并校准任务代理与实物的差异。

清洗玻片场景的烧杯和超声槽默认预装水，透明水面仅作视觉表达，清洗使用 `run_seconds` 抽象计时，不模拟流体、声场或去污效果。五片 25×75 mm 玻片固定在带提杆的玻片槽中；玻片槽和烧杯是两个独立动态刚体，嵌套搬运依靠重力及容器碰撞。烧杯增加外侧夹持耳以适配 2F-85。场景和相机采用支持透明材质的 `default` shader；透明物体的深度仍是理想渲染结果。详见 [清洗玻片说明](docs/SLIDE_CLEANING.md)。

```bash
pytest -m 'not integration'
SCI_SIM_INTEGRATION=1 pytest -m integration
```

旋涂场景包含透明翻盖、圆形内腔、电机驱动托盘、独立玻片与移液器、移液器架和褐色液体烧杯。吸液/单滴转移通过针尖位置和连续停留时间抽象实现，旋转采用演示速度，不求解液体铺展或膜厚。同桌新增带观察窗翻盖与多孔托盘外观的闪蒸仪，旋涂后继续转移同一玻片并启动闪蒸；闪蒸使用默认 2 s 计时模型。详见 [旋涂说明](docs/SPIN_COATING.md)。

## 来源

- 默认仪器：本地 `../atombridge/model/instrument/` 与 `../atombridge/assets/instrument/`；完整相对路径与哈希见生成的 `assets/instruments/manifest.json`。
- [ManiSkill 多机器人示例](https://github.com/mani-skill/ManiSkill/blob/main/mani_skill/envs/tasks/tabletop/two_robot_pick_cube.py)
- [UR 官方描述与 UR7e 参数](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/0eb1a861f7772fceb38510d90fa2f98ec668070c/config/ur7e)
- [ManiSkill XArm6 / Robotiq 资产](https://github.com/haosulab/ManiSkill-XArm6/tree/v0.1.1)
- [旧版程序化 Bio-Rad T100 规格](https://www.bio-rad.com/sites/default/files/2021-08/10000145577.pdf)
- [旧版程序化 Eppendorf 5425](https://www.eppendorf.com/ar-en/Products/Centrifugation/Centrifuges/Centrifuge-5425-5425R-p-PF-934144)
- [Vortex-Genie 2](https://www.scientificindustries.com/products/vortex-genie-2)
- [LeRobot Dataset API](https://huggingface.co/docs/lerobot/main/api/datasets)

详细几何与型号来源也见 `src/sci_sim/catalog.json`。
