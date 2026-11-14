# 安装和实际验证记录

更新于 2026-11-14，Asia/Shanghai。场景可运行与完整任务成功分别记录。

## 第五版：三个基本操作任务实际完成

默认双 UR7e + Robotiq 2F-85、AtomBridge 重建仪器，最终策略在两个随机布局种子上均从 `reset` 完整执行成功。使用 WSL CPU PhysX/MPlib；下表是六个有限回合的功能检查，不是大规模 benchmark 成功率。

| 任务 | seed 42 | seed 43 | 实际步骤 |
|---|---|---|---|
| 混匀并转移 | 成功，424 步 | 成功，425 步 | 左架取管、接触混匀头、右架放管并释放 |
| 热循环 | 成功，2754 步 | 成功，2815 步 | 开盖、直接抓放 PCR 板、关盖、夹持旋转压力旋钮、屏幕接触启动 |
| 离心 | 成功，2557 步 | 成功，2552 步 | 开盖、依次插入样品管/对置配平管、关盖、接触启动 |

- 可复现命令和具体判据见 [基本任务说明](BASIC_TASKS.md)。执行没有设置物体位姿或固定抓取约束；规划携带物仅作为 FCL 碰撞模型，实际抓取依赖接触力。示范专家使用仿真真实位姿和接触信息。
- 最终六份 JSON：`outputs/tasks/final_mix_transfer_42b.json`、`final_mix_transfer_43.json`、`verified_thermal_cycler_42.json`、`final_thermal_cycler_43.json`、`verified_centrifuge_42.json`、`verified_centrifuge_43.json`。同名 PKL 保存实际状态，MP4/PNG 由这些状态渲染。
- 调试中的失败记录保留。尤其 `final_centrifuge_43.json` 记录了配平管插入后倾倒；随后增加插入前测量抓持偏移与姿态反馈，在两个种子下重新完成全流程。旧失败、断点后缀验证和最终整回合结果不能混合计算成功率。
- 台面加深到 0.90 m，柜体与桌边留 5 cm；仪器向机械臂一侧布置以满足抓盖可达性。架台保持第四版等径圆柱 T 形和两侧真实 45° 切面。
- 重建 AtomBridge 离心管/PCR 板外观及 C1000 可转动旋钮；添加管架导向、反应槽、对置插管导向、抓盖把手和按钮接触代理。PCR 板采用 114×76 mm 缩放外廓；旋紧判据为 ≥1.4 rad，并非实际扭矩。流体、温控和高速转子物理未模拟。
- 规划修复了倾斜机器人基座坐标转换、复合持物碰撞体、UR 肩肘逆解选择及笛卡尔轨迹跟踪。轨迹由 RRTConnect、逐点 IK、密集碰撞检查和 TOPP 生成。
- WSL 最终回归 **24 passed**，包含三场景实际 RGB-D；日志 `outputs/tasks/pytest_final.log`。其他机器人仍只具备此前场景构造/初始保持检查，没有完整专家任务成功证据。
- **实际成功示范采集**：`datasets/basic_mix_demo_42`，seed 42，一次尝试一次成功，424 帧、20 Hz、四路 160×120 RGB-D。LeRobot 0.7.0 回读首/中/末帧，通过动作、深度毫米值、相机标定、末帧 done/success 检查；报告 `loader_validation.json`。它是一条成功示范，不同于早期三帧诊断数据。默认 640×480 的整回合采集尚未在本轮测量。
- 视频在 Windows 原生 Vulkan 上渲染同一组 WSL 物理快照；未把 Windows 回放当作另一轮独立任务验证。本机 WSL 仍是 Mesa 软件渲染。

## 第四版：横向圆柱两端真实 45° 斜切

- 第三版仍是平端圆柱叠加倾斜连接座，确实与机械臂基座相交；现已移除连接座，生成两端真实斜切、封闭且法向一致的圆柱网格。视觉与碰撞读取相同几何。
- 横梁中心高改为 1.43 m，与左右安装中心等高；竖柱与横梁直径均为 180 mm，安装轴保持左右朝外下倾 45°。
- UR7e/UR5e 根位姿不变。Franka 原始基座 STL 最低 Z=-0.032493 mm，xArm6 原凸分解碰撞网格最低 Z=-0.532 mm；分别向外补偿 0.0335/0.533 mm，避免源碰撞网格自身越过安装面。该补偿同步写入数据清单。
- 新增封闭网格/切面共面回归检查，以及 `scripts/validate_stand_clearance.py`：读取实际 SAPIEN 碰撞几何，检查四种模型全部连杆与整个架台，包含基座，不使用碰撞排除；检查初始化和保持 20 步后的状态，容差 10 µm。
- WSL 完整回归 **24 passed**（包含三场景 RGB-D）。四种模型共 8 次架台碰撞检查通过：UR7e/UR5e 最大接触深度约 0.0673 µm，Franka/xArm6 未检测到接触。记录为 `outputs/revision4/pytest.log` 和 `outputs/revision4/clearance.json`。
- 实际渲染近景 `outputs/revision4/mount_detail.png` 与全架台图 `outputs/revision4/stand.png` 已人工查看。此检查仅验证架台几何和初始保持状态，不代表完整任务轨迹成功。

## 第三版：等径圆柱 T 形支架（历史，端面相交已由第四版修复）

- 按用户最新纠正，用竖直圆柱与左右水平圆柱替换方柱和倒梯形顶部；两根圆柱直径统一为 180 mm。
- 当前 UR7e 碰撞网格测得基座 XY 外廓约 151 mm，因此采用略大的 180 mm 支撑直径。水平圆柱轴心高 1.49 m。
- 左右端短圆形连接座保留机械臂根位姿与朝外下倾 45° 的轴向；视觉几何和碰撞几何同步修改。
- 删除旧倒梯形网格文件；新版渲染为 `outputs/revision3/stand.png`，混匀场景已实际步进 20 步并检查无任务失败。
- WSL 回归测试 23 项通过，包含三场景 RGB-D 和实际基座安装轴检查；总览为 `outputs/revision3/overview.png`。

## 第二版：按照片修订架台与仪器（历史）

- 用户确认安装轴为左右分别朝外、向下 45°。已改为柜体、开槽顶板、单立柱、封闭倒梯形头部；机械臂实际根位姿随之修改。
- 基座中心为 x=±0.245、y=-0.74、z=1.43 m；这些尺寸为照片比例下的建模假设，并非实测尺寸。柜体与桌面独立，前缘距桌边约 10 cm。
- 四种机械臂的左右初始关节角重新通过碰撞感知 IK 求解，夹爪朝向桌面。四路相机仍为 RGB-D，frontcam 移到架台顶部前方，避开立柱。
- 从用户的 `../atombridge` 只读导入并重建 Vortex-Genie 2、Bio-Rad C1000、Eppendorf 5430 的分部件网格、纹理和主盖铰链。源项目未修改，其插件未执行。
- 生成 GLB、几何参数和源文件 SHA256 清单在 `assets/instruments/`；它们属于本地派生资产，未纳入版本控制。复现命令：`sci-sim import-instruments --source ../atombridge`。
- 默认已从 T100/5425 切换为 C1000/5430；原程序化仪器仅在明确选择 `--instruments procedural` 时使用。
- 新版 WSL 测试 **23 passed**；UR7e 三场景各完成 100 步静止保持和四路 RGB-D 检查，无掉落/双臂碰撞失败。
- 新版 UR5e、Franka、xArm6 在 Windows 各完成三场景 20 步与四路 RGB-D 检查；报告位于 `outputs/revision2/{ur5e,franka,xarm6}/report.json`。前相机最终视野已检查，能够看到仪器与两侧耗材架。
- 新版证据目录：`outputs/revision2/`。`overview.png` 是三桌总览，`*_stand.png` 是架台近景；`*_instrument_open.png` 为直接设置关节角生成的几何检查图，**不是专家成功开盖轨迹**。
- 导入的基础碰撞体和主盖关节参与物理；次级锁杆/旋钮固定，5430 转子孔位仍为简化的垂直槽。温控、流体与高速旋转未模拟。
- 以下旧版轨迹失败报告保留为历史记录，不能用于新架台、新仪器的成功率判断。新版完整任务成功率尚未建立。

## 已安装环境

- WSL 3.0.5.0，内核 6.18.58.1，发行版 **SciSim-Ubuntu**，Ubuntu 26.04.1。
- 用户提供的 `ubuntu-26.04.1-wsl-amd64.wsl` 已导入，虚拟磁盘在项目 `.wsl/Ubuntu/`。
- Linux 用户 `scisim`；未设置密码或免密 sudo。系统管理通过 `wsl -d SciSim-Ubuntu -u root`。
- 仿真环境 `/home/scisim/.venvs/sci-sim`：Python 3.12.16、ManiSkill 3.0.1、SAPIEN 3.0.3、MPlib 0.2.1、NumPy 1.26.4、CPU Torch 2.7.1。
- 写入环境 `/home/scisim/.venvs/sci-data`：LeRobot 0.7.0、NumPy 2.x、CPU Torch 2.9.1、Torchvision 0.24.1、TorchCodec 0.9.1；兼容 Ubuntu 26.04 的 FFmpeg 8。
- 完整安装版本清单：`outputs/setup/sim_versions.json`、`data_versions.json`。这是环境快照，不是跨平台依赖锁。
- MPlib 与 LeRobot 的 NumPy 版本要求冲突，使用两个环境和私有子进程通信，不在一个环境强行混装。

## 图形能力

WSL 可以识别 NVIDIA GPU 的 CUDA 设备，但该环境提供给 Vulkan 的设备是 llvmpipe 软件渲染器。显式设置 `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json` 后，SAPIEN 软件渲染实测成功。**不能把这些结果称为 WSL GPU 渲染验证。**

`scripts/activate_wsl.sh` 激活所需环境，仅影响当前 shell。未安装 Linux NVIDIA 内核驱动，未修改用户全局代理设置。安装时的临时 WSL 代理转发已停止。

Windows `.venv-win` 环境也完成原生 Vulkan 渲染验证，使用 CPU 张量读回避开 Windows 不支持的 CUDA 图像互操作。交互查看窗口尚未验证，截图及 RGB-D 观测已验证。

## 第一版通过的检查（历史）

- WSL 完整测试：**19 passed**，包括三个真实场景 reset/step/RGB-D 集成测试。
- 默认 UR7e：三个场景各 **100 步**，种子 42；无掉落或双臂碰撞失败，耗材漂移最大约 4.3 微米。
- 每个场景 front、side、左右 wrist 的 RGB 为 320×240×3，深度为 uint16 毫米。有效深度像素比例约 53%–100%，图像非空。
- 三桌六臂总览在 WSL 内实际渲染并检查：`outputs/validation/wsl/overview.png`。
- UR5e、Franka 与 xArm6：分别完成三个场景各 20 步和四路 RGB-D 检查。xArm 重复材质问题已修复并重新验证。
- 四种机器人资产均已生成，URDF 图结构、关节、相对网格路径与材质唯一性纳入检查。
- LeRobot 写入三个真实静止场景帧，使用官方加载器读回，**全部四路深度像素完全一致**。存储为 uint16，加载为 float32，数值单位仍为毫米。状态 48 维、动作 14 维、相机内外参形状通过检查。

主要证据：

- `outputs/validation/wsl/report.json` 及同目录 RGB PNG、原始 `*_depth.npz`。
- `outputs/validation/windows/report.json`：Windows 三场景各 100 步验证。
- `outputs/validation/{ur5e,franka,xarm6}/report.json`：可选机器人验证。
- `outputs/validation/diagnostic_dataset/roundtrip_report.json`：深度往返检查。
- `diagnostic_dataset/DIAGNOSTIC_ONLY.json`：明确标记诊断帧，**不是成功任务示范**。

## 第一版未通过的完整操作（历史）

实际运行种子 42 的 UR7e 专家，修复了 MPlib 网格缩放、不唯一的障碍物名、持物模型基坐标系与接触预压问题。当前结果：

| 任务 | 实际进展 | 失败位置 |
|---|---|---|
| 混匀转移 | 右臂让位、左臂抓管、抬升、真实接触混匀 2 秒、转移、释放 | 820 步后试管最终放置未满足位置/直立条件 |
| 热循环仪 | 抓取盖把手并开始抬盖 | 486 步后 screw 规划失败；实际盖角约 -0.365 rad，未达到开盖条件 |
| 离心机 | 抓取盖把手并开始抬盖 | 490 步后 screw 规划失败，未达到开盖条件 |

详见 `outputs/validation/wsl/expert_{mix,thermal,centrifuge}.json`。碰撞检查没有被关闭，也未在执行过程中设置物体位姿或粘接物体。尚无可报告的完整任务成功率，尚未产生成功 benchmark 示范数据。仍需调整试管架约束、精细放置和盖子操作的抓持/可达性，并进行多种子评估。

完整 `collect` 入口也已运行：种子 42、160×120、一个混匀回合，执行 820 步后正确将未成功回合丢弃，保留 0 个示范并写入失败报告。见 `outputs/validation/collection_smoke/meta/scisim_summary.json` 和 `scisim_attempts.jsonl`。这验证了失败过滤与报告链路，没有验证成功示范的端到端采集。

## 复现

PowerShell：

```powershell
wsl -d SciSim-Ubuntu -u scisim
```

Ubuntu shell：

```bash
cd /mnt/c/Users/15261/Documents/GitHub/sci-sim
source scripts/activate_wsl.sh
sci-sim render --layout lab --output outputs/overview.png
SCI_SIM_INTEGRATION=1 pytest -q
python scripts/validate_scenes.py --output outputs/validation/recheck --steps 100
python scripts/validate_expert.py --task mix_transfer --output outputs/validation/expert_recheck.json
```

专家验证失败会保留 JSON 并返回非零退出码。正式采集的成功条件保持不变。
