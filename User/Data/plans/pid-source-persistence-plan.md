# PID 参数来源固化方案（v2：单字段位域）

## 摘要

将 PID 三环 source 选择持久化到 Flash 的 **单个 uint32_t 字段** `pid_source_mask`（Index=127，ControlParam 段最后位）。每环占用 4 bit，低 12 bit 编码三环 source。新增 0x9C 命令让上位机读取真实 source 状态。

## 位域布局

```
pid_source_mask (uint32_t):
  bit[3:0]   = current ring source  (0=DEFAULT 1=FLASH 2=AUTOTUNE)
  bit[7:4]   = velocity ring source
  bit[11:8]  = position ring source
  bit[31:12] = reserved (0)
```

访问宏：
```c
#define PID_SRC_CUR_SHIFT  0
#define PID_SRC_VEL_SHIFT  4
#define PID_SRC_POS_SHIFT  8
#define PID_SRC_MASK       0xFu
```

## 变更清单

### 变更 1：motor_info.csv 新增 1 行

在 ControlParam 段末尾（Index=127）追加：

```
pid_source_mask,PID来源位掩码,ControlParam,RW,uint32_t,0,0,4294967295,,0x0140,127,bit[3:0]=电流环 bit[7:4]=速度环 bit[11:8]=位置环 0=默认 1=Flash 2=理论估计
```

运行 `python motor_info_generate.py` 重新生成 `motor_info.h/.c`。

### 变更 2：motor_pid_load.c 接入 Flash 持久化

#### 2a. 新增位域访问宏（motor_pid_load.h）

```c
#define PID_SRC_CUR_SHIFT  0
#define PID_SRC_VEL_SHIFT  4
#define PID_SRC_POS_SHIFT  8
#define PID_SRC_MASK       0xFu

/* 从 mask 提取指定环 source */
static inline pid_source_e pid_source_from_mask(uint32_t mask, pid_ring_e ring)
{
    uint8_t s;
    switch (ring) {
        case PID_RING_CURRENT:  s = (mask >> PID_SRC_CUR_SHIFT) & PID_SRC_MASK; break;
        case PID_RING_VELOCITY: s = (mask >> PID_SRC_VEL_SHIFT) & PID_SRC_MASK; break;
        case PID_RING_POSITION: s = (mask >> PID_SRC_POS_SHIFT) & PID_SRC_MASK; break;
        default: return PID_SOURCE_DEFAULT;
    }
    return (s <= (uint8_t)PID_SOURCE_AUTOTUNE) ? (pid_source_e)s : PID_SOURCE_DEFAULT;
}

/* 把指定环 source 写入 mask 对应位段，返回新 mask */
static inline uint32_t pid_source_to_mask(uint32_t mask, pid_ring_e ring, pid_source_e src)
{
    uint32_t shift;
    switch (ring) {
        case PID_RING_CURRENT:  shift = PID_SRC_CUR_SHIFT; break;
        case PID_RING_VELOCITY: shift = PID_SRC_VEL_SHIFT; break;
        case PID_RING_POSITION: shift = PID_SRC_POS_SHIFT; break;
        default: return mask;
    }
    return (mask & ~(PID_SRC_MASK << shift)) | ((uint32_t)src << shift);
}
```

#### 2b. 新增 `motor_pid_load_source_from_flash`（motor_pid_load.c）

```c
void motor_pid_load_source_from_flash(const motor_info_t *info)
{
    if (info == NULL) return;
    uint32_t mask = info->blocks.control.pid_source_mask;
    /* 非 0 值表示用户曾显式选择，覆盖 load_boot 的自动回退结果 */
    pid_source_e cur = pid_source_from_mask(mask, PID_RING_CURRENT);
    pid_source_e vel = pid_source_from_mask(mask, PID_RING_VELOCITY);
    pid_source_e pos = pid_source_from_mask(mask, PID_RING_POSITION);
    if (cur != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_CURRENT]  = cur;
    if (vel != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_VELOCITY] = vel;
    if (pos != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_POSITION] = pos;
}
```

#### 2c. motor_loop.c 接入

```c
motor_pid_load_boot(param, motor_info_storage_get());
motor_pid_load_source_from_flash(motor_info_storage_get());  /* 新增 */
motor_pid_load(param, motor_info_storage_get());             /* 按 source 重新加载 */
```

### 变更 3：0x9B 写入 mask 字段

jm_proto_ops.c `app_pid_source_set` 中，切 source 时同步更新 `pid_source_mask`：

```c
motor_info_t *info = motor_info_storage_get();
uint32_t mask = info->blocks.control.pid_source_mask;

if (ring_select & 0x01) {
    motor_pid_set_source(PID_RING_CURRENT, src);
    mask = pid_source_to_mask(mask, PID_RING_CURRENT, src);
}
if (ring_select & 0x02) {
    motor_pid_set_source(PID_RING_VELOCITY, src);
    mask = pid_source_to_mask(mask, PID_RING_VELOCITY, src);
}
if (ring_select & 0x04) {
    motor_pid_set_source(PID_RING_POSITION, src);
    mask = pid_source_to_mask(mask, PID_RING_POSITION, src);
}
info->blocks.control.pid_source_mask = mask;  /* 写 RAM，由 0xEA 固化 */
```

### 变更 4：新增 0x9C 读 source 命令

- CSV 命令表追加 0x9C 条目
- jm_cmd_def.h 加 `JM_CMD_PID_SOURCE_GET = 0x9C`
- jm_proto.h ops 加 `pid_source_get` 函数指针
- jm_proto_ops.c 实现：调 `motor_pid_get_source` 返回 3 字节
- jm_proto.c dispatch 加 0x9C 分支

### 变更 5：上位机实现（已完成）

- cmd_def.py 加 `PID_SOURCE_GET = 0x9C`
- motor_client.py 加 `pid_source_get()` 方法 + `pid_source_received` 信号 + `_on_frame` 0x9C 分发
- pid_panel.py 加 `on_source_received()` 槽函数（回填徽章 + 同步单选按钮）
- main_window.py 连接 `pid_source_received` 信号 + 连接建立时主动查询
- 上位机 CSV 资源同步 0x9C 行

## 状态：全部完成

| 变更 | 内容 | 状态 |
|------|------|------|
| 1 | motor_info.csv 新增 pid_source_mask (Index=127) | ✅ |
| 2 | motor_pid_load.h/c 位域宏 + source_from_flash | ✅ |
| 3 | motor_loop.c 上电加载 source_from_flash | ✅ |
| 4 | 下位机 0x9C 协议命令 (CSV/enum/ops/dispatch) | ✅ |
| 4b | 0x9B 写入 pid_source_mask 到 RAM | ✅ |
| 5 | 上位机 0x9C (cmd_def/motor_client/pid_panel/main_window/CSV) | ✅ |

## 验证

1. CSV 重新生成后 `ControlParam_t` 新增 `pid_source_mask` 字段
2. 0x9B 切 source 后 0xEA 保存，断电重启 source 保持
3. 0x9C 读取返回 3 字节与 `motor_pid_get_source()` 一致
4. 上位机重连后徽章显示真实 source
5. Python 语法检查通过（py_compile）

## 待用户操作

- Keil 工程中全编译验证下位机
- 实机测试：连接 → 徽章回填 → 0x9B 切换 → 0xEA 保存 → 断电重启 → 0x9C 读取确认
