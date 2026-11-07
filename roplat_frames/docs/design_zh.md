# roplat_frames：坐标树设计

2026-10-27，yixing-windows｜yixing｜Claude Code（Claude Opus 5.5）｜session-claude-win-frames-01；roplat-agents 任务 `20261026-ecosystem`，缺口 E06。

状态：首版实现，未合并、未发布。只依赖 `roplat_msgs` 与 `roplat_description`；节点在可选 feature `roplat` 下。不改 roplat 核心，需要核心配合的地方列在[提议](#提议与待裁定)里。

## 范围

首批场景是机械臂操作：世界、基座、各 link、工具、相机（手眼外参）、被操作物体。不做导航、多机、跨进程分发。

包名用 `roplat_frames`：与 `FrameId` 同词，比 `roplat_tf` 这类 ROS 缩写更直白。

## 帧名与 `FrameId`

`frame_id(name)` 是 `const fn`，取名字 UTF-8 字节的 FNV-1a 32 位散列；散列为 0 时改为 1，0 仍表示未指定。

- 同一个名字在任何进程、任何树里都得到同一个 id。驱动和感知节点不需要拿到注册表，就能给 `Header.frame` 填值；`const TOOL: FrameId = frame_id("tool");` 在编译期确定。录制文件里的 id 离开名表也能用同样的名字复现。
- 注册表就是树本身：树保存 id 到名字的映射，供显示与诊断。建树时如果两个不同名字散列相同，报 `IdCollision`，所以树内 id 与名字一一对应。100 个帧碰撞的概率约为 1e-6。
- 不在树里的 id（例如手写的 `FrameId::new(7)`）查询时报 `UnknownFrame`。

另一种做法是由注册表顺序分配 1、2、3……，不会碰撞，但每个生产者都要持有注册表实例，不同程序里同名帧的 id 不同，录制必须附带名表。因此没有采用。

## 静态树 `FrameTree`

`FrameTree::builder()` 声明全部帧与边，`build()` 之后拓扑与静态位姿不再改变，运行中不增删帧。边有三种：

| 边 | 来源 | 位姿 |
|---|---|---|
| 固定 | URDF fixed 关节；标定外参 `fixed(parent, child, pose)` | 建树时确定。`Pose` 可以 const 构造，标定值可写成常量 |
| 关节 | URDF revolute、continuous、prismatic 关节 | `roplat_description::Joint::transform(q)`，q 来自所属关节组 |
| 动态 | 外部给出的 parent→child 位姿，如检测到的物体 | 按时间缓冲，见下节 |

相机的标定外参可以用 `calibration::calibrate_hand_eye` 求出：在若干工位同时记录法兰位姿（正运动学）与标定板在相机光学系中的位姿，按 `A X = X B` 求解（Park–Martin：先拟合旋转，再用线性最小二乘求平移）。眼在手上时结果是相机在法兰系中的位姿，眼在手外时是相机在基座系中的位姿，再用 `fixed` 写入树。结果附带各工位推算出的标定板位姿的离散程度（`rotation_rms`、`translation_rms`），用来判断标定是否可信。各工位须绕至少两根明显不同的轴转动，否则报 `Degenerate`。

建树时检查并报错：

- 恰好一个根，每帧一个父，无环。因此任意两帧连通，查询不会因为断开而失败。
- floating、planar、spherical 关节报 `UnsupportedJoint`。`roplat_description` 把这几种关节当作恒等变换，坐标树不能沿用这个默认。
- 关节组只能包含可动关节，一个关节最多属于一个组。没有分组的可动关节不是错误，但经过它的查询报 `Unpositioned`，例如只登记了手臂、没登记手指时的指尖帧。

`FrameTree::lookup_static(parent, child)` 只走固定边，路径上有关节边或动态边时报 `NotStatic`。所有查询都先求最近公共祖先，只沿两段路径相乘，不计算公共祖先以上的边。建树之后的查询与写入都不分配内存，测试用计数分配器验证。

## 时间缓冲 `FrameBuffer`

`FrameBuffer::new(tree, clock)` 拥有这棵树，并为每个关节组、每条动态边预分配一个环形缓冲，容量由建树时的 `History` 给出。

**时钟。** 一个缓冲只接受一个时钟。写入或查询的 `Stamp::clock` 与缓冲的时钟不同时报 `ClockMismatch`，与 `Stamp::nanos_since` 跨时钟返回 `None` 的语义一致。缓冲不接受 `clock::UNSPECIFIED`，必须指明时钟。

**写入。** `set_joints(group, stamp, &q)` 与 `set_transform(stamp, &transform)`：

- 时间必须单调。早于最新样本报 `OutOfOrder`；与最新样本同一时刻时覆盖它，因为仿真暂停时时钟不动。
- 非有限值或零四元数报 `InvalidValue`。
- 容量满时丢弃最旧的样本。

**查询。** `lookup(parent, child, at)` 返回 child 在 parent 中的位姿。路径上的每条非固定边都在同一时刻 `at` 取值：

| `at` 的位置 | 结果 |
|---|---|
| 两个样本之间 | 线性插值。关节先逐个插值位置再求变换；动态边的位置线性插值，姿态沿较短的弧 slerp（两样本恰好相差半圈时两侧等长，走哪一侧不作保证，采样应远比这密） |
| 两个样本相隔超过 `max_gap` | `Gap`，不跨越掉线区间插值 |
| 最新样本之后、最旧样本之前，且在 `hold` 以内 | 用该端的样本（零阶保持），不做线性外推 |
| 超出 `hold` | `AfterLatest`（数据尚未到达或已陈旧）、`BeforeHistory`（已被挤出或从未存在） |
| 该源还没有样本 | `NoData` |

**超时。** 查询从不阻塞。tf2 的 `lookupTransform(…, timeout)` 会等待数据到达；roplat 节点合作式执行，不应在 `process` 里阻塞，所以数据未到就返回错误，由图决定下一周期再试。新鲜度由 `hold` 限定：用 `clock.now()` 查询时，最新样本比 now 旧出 `hold` 就失败，起到超时的作用。

`hold`、`max_gap` 按源配置。两者包含等于上限的时距，覆盖 `Stamp::nanos` 的完整有符号范围。两个时刻相距最多 `u64::MAX` 纳秒，时距与上限用整数比较，插值比例按 `f64` 计算。直接查询样本时刻始终可以取该样本，无须跨间隙插值。1 kHz 的关节组可将 `hold` 设为一两个周期；放在桌上的物体，检测结果可以保持数秒。只写一次、始终有效的锁存变换，用容量 1、`hold = Duration::MAX` 的动态边表示。

不提供 tf2 的“最新公共时刻”（time 0）查询。roplat 图里时间戳随数据流动，查询方总能给出自己需要的时刻，例如关节状态或图像的 stamp。

## 由关节状态生成 link 位姿的节点

feature `roplat` 下的 `JointStateNode<N, P = JointState<N>>`：

- `Resources = (FrameBuffer,)`，`Input = (Stamp, P)`，`Output = Result<(), FrameError>`。业务错误是普通数据，图可以用 `?` 提升。
- `P` 可以是 `JointState<N>`（取 `meas.q`）、`JointSample<N>`（取 `q`）或 `[f64; N]`。位置缺失时报 `MissingPositions`，不补零。
- 关节组句柄 `JointGroup<N>` 由 `tree.group::<N>("arm")` 取得。组大小与 `N` 不符在取句柄时报错。节点先读取位置，再由缓冲依次检查时钟、句柄归属、数值有限性和时间顺序。

节点只把关节位置写入缓冲，link 位姿在查询时按需由 `Joint::transform` 计算。1 kHz 的写入只是拷贝 N 个数，三角函数只在查询时计算（相机、显示的频率）；插值关节位置也比插值各 link 位姿更贴近真实运动。

`ControlRhythm` 的 Yield 是 `(Obs, Duration)`，不带时间戳。目前需要在图里显式打戳，例如 `roplat::node::map(move |state| (clock.now(), state))`，得到的是接收时刻而不是测量时刻（见提议 2）。

## 所有权

没有全局单例。`FrameBuffer` 是普通值，由创建层拥有，节点通过 `&mut frames >> node.resources.0` 借用。读取方也借用 `&mut`，与写入方按数据或 `.trigger` 依赖排序，没有依赖路径时 System 编译报错。`FrameTree` 可以单独使用，只做静态查询。同一进程可以有多棵树。

关节组句柄只属于签发它的树及其克隆。`FrameBuffer::clone` 复制历史，两份缓冲独立接受写入。用相同声明再次 `build` 会得到一棵独立的树，向它使用原句柄会返回 `ForeignGroup` 并保留原样本。每次成功构造含关节组的树，在进程内用一个原子计数器划出一段不复用的编号；若剩余编号不足，建树返回 `TreeError::GroupIdsExhausted`。句柄本身只占一个机器字。写入时只需将编号换算为本树的组下标并判断范围。

## 提议与待裁定

1. **把 `frame_id` 移入 `roplat_msgs`**，例如 `FrameId::named(name)`，并在 `FrameId` 文档中写明散列规则。这样驱动给 `Header.frame` 填值时不必依赖 `roplat_frames`。这改变消息的公共契约，需要作者授权。
2. **控制节律的观测带测量时刻。** 让 `ControlRhythm` 产出 `Stamp`，或者把观测改为 `Stamped<…>`，与 E02 把 `Clock` 交给节律一起做。在此之前只能用接收时刻打戳。
3. **跨域读取。** 控制域写入，感知或显示域查询。候选做法：控制域把关节样本经 `Latest` 或 SPSC 旁路送到另一域，各域持有自己的 `FrameBuffer`，不共享锁。需要与旁路契约一起定。
4. **帧 id 是否需要保留区**，例如 world 固定为 1。目前不保留，只按名字散列。
5. **名字前缀。** 双臂使用同一份 URDF 时会重名，目前直接报错；需要时可以在 `model` 导入时加前缀。
6. **构建期生成。** 建树发生在启动时（解析 URDF 需要分配）。若要在 cargo 构建期由 URDF 生成 const 表（build.rs），以后可以加，不影响 API。

## 未覆盖

- mimic 关节：`roplat_description` 没有解析，夹爪联动需要手动分组。
- 速度、力的坐标变换（twist、wrench），以及点、向量的变换辅助函数。
- floating、planar、spherical 关节；线性外推；乱序插入。
- 真实设备与 Rerun 接入。`rerun_urdf` 改用本 crate 是后续工作。
