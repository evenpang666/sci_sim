# 三个基本操作任务

在项目根目录的 Linux 仿真环境中执行，默认双 UR7e + Robotiq 2F-85，使用本地 AtomBridge 重建资产：

```bash
source scripts/activate_wsl.sh  # 本机 WSL；其他 Linux 激活其仿真 venv
sci-sim demo --task mix_transfer --seed 42
sci-sim demo --task thermal_cycler --seed 42
sci-sim demo --task centrifuge --seed 42
```

`--render` 打开实时窗口，`--seed` 改变布局和规划随机种子。默认使用四路 RGB-D；WSL 软件渲染较慢，无图像的物理/规划验证可用下方验证脚本。

| 任务 | 实际动作 | 完成判据 |
|---|---|---|
| 混匀转移 | 左臂从左架抓取样品管，夹持在混匀头上接触 2 秒，移到右架释放 | 已混匀、管在目标位置、已释放且稳定 |
| 热循环 | 右臂开盖，左臂抓 PCR 板放入反应槽，右臂关盖、旋转压力旋钮、按屏幕启动点 | 板就位、盖关闭、旋钮转角 ≥1.4 rad、真实按钮接触并完成运行计时 |
| 离心 | 右臂开盖，左臂依次放样品管和等质量配平管到对置孔位，右臂关盖并按启动点 | 两管就位且直立、配平、盖关闭、真实按钮接触并完成运行计时 |

所有物体移动通过关节驱动和夹爪接触完成，运行中不设置物体位姿、不通过固定约束吸附物体。规划使用仿真真实位姿作为专家信息，包含持物碰撞模型和插管前重新测量抓持偏移。另一只臂作为规划障碍物；本版动作按双臂分工顺序执行。

热循环仪/离心机使用机器人抓盖把手、插入导向和按钮接触代理。旋紧采用转角阈值，不模拟真实扭矩；混匀不模拟液体，温控和离心运行采用计时状态机。PCR 板外廓为源模型的 114×76 mm 缩放尺寸，适配 2F-85；耗材管盖固定。管架为程序化带导向孔模型。

## 保存实际执行证据

```bash
python scripts/validate_expert.py --task thermal_cycler --seed 42 \
  --trace --output outputs/tasks/my_thermal.json
python scripts/render_task_trace.py outputs/tasks/my_thermal.pkl
```

验证脚本返回码 0 表示成功，2 表示任务失败；JSON 保存步骤数、阶段、最终事实与错误。PKL 为本地物理状态快照，仅加载可信的本项目产物。渲染脚本从这些快照导出 MP4、最终 PNG 和阶段拼图，不参与任务判定。当前检查范围和结果见 [STATUS.md](STATUS.md)。

## 采集 LeRobot 示范

```bash
sci-sim collect --task mix_transfer --seed 42 --episodes 1 --max-attempts 1 \
  --width 160 --height 120 --root datasets/my_mix_demo --repo-id local/my_mix_demo

# 在独立的数据 venv 中运行
python scripts/validate_demo_dataset.py --root datasets/my_mix_demo --repo-id local/my_mix_demo
```

每次采集使用新的输出目录。正式采集可使用默认 640×480，增加回合和尝试数。成功与失败尝试全部记入 meta 清单，失败回合不作为成功示范保存。数据包含四路 RGB、毫米深度、逐帧相机内外参、双臂关节状态和动作；不会自动上传。
