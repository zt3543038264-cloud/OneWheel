# Yaw 反馈符号修正、绝对航向保持与共模动量回收

2026-11-11。基于 `port/asc-control-r10h`（= upstream/master）修改。
已于同日烧录并实测：符号修正与绝对航向保持**已验证通过**，共模动量回收尚未观察到触发。
本文的取值来自当天在车上用 WiFi cfg 逐项实测。

## 1. 问题

`config.h` 的 `Y_RATE_BASE = -1` 假设"正占空比共模 → 负 Yaw 角加速度"。本车实测相反：
正共模占空比让车身朝正 `gyro_z` 方向转。于是速度环变成正反馈，航向环无法收敛，表现为
**转到某个角度后摆幅越来越大**，且航向误差无界累积（实测一度达到 840°）。

实测依据（WiFi cfg，Balance 模式）：

| 实验 | 结果 |
|---|---|
| `y_angle_kp=0`、`y_rate_kp=+10` | 车身自行越转越快（200~350 °/s） |
| `y_angle_kp=0`、`y_rate_kp=-3` | 转速 129 → 7 °/s 自行衰减 |
| `y_rate_kp=-1` | `u / gyro_z = -1.00`，换算关系精确吻合 |
| 由 `u` 反推的误差与积分航向对比 | 斜率 1.010，`att.yaw` 与 `gyro_z` 同号 |

注意：`motor_dir_a/b` 同时作用于占空比下发和转速回读，翻转它不会改变环路符号，
所以只能在控制器侧修正。

## 2. 符号修正

- `config.h`：`Y_RATE_BASE` 由 `-1.0f` 改为 `+1.0f`。
- `control.c`：Yaw 航向误差改为 `g_yaw_target - imu_get_angle_yaw()`，覆盖三处
  （正式串级、Path 分支、Yaw 单轴测试）。
- 结果：两个 Yaw 增益都是**正值**用户值，与其余轴一致。
- `param.c` 新增 `param_normalize_yaw_sign()`：加载 Flash 后把负的 Yaw 增益取反，
  旧固件存下的 `-60 / -24` 会自动变成 `+60 / +24`，不必 Reset 全部参数。
  新约定下负 Yaw 增益本身就是不稳定配置，取反永远安全。

## 3. 绝对航向保持（`yaw_hold_deg`）

Balance 原来只在启动瞬间锁存航向，没有输入目标角的入口。新增参数 `yaw_hold_deg`
（Yaw 页 `Yaw Hold`，−180～180，步长 1，默认 0）：

```c
// motion_target_update()，仅普通 Balance
want = s_balance_yaw_ref + YAW_HOLD_DEG + s_yaw_desat_offset;
g_yaw_target 按 YAW_HOLD_SLEW_DPS(30°/s) 逼近 want;
```

`s_balance_yaw_ref` 在 `control_balance_start()` 取当前航向，`YAW_HOLD_DEG=0` 时
行为与改造前完全一致。目标走斜率是避免大角度阶跃——阶跃会把角度环打进饱和，
这正是原来超调的来源。

实测参考：`y_rate_kp=+60`、`y_angle_kp=+24` 时，给 +20° 目标可稳定在
**18.2～19.4°**（残差约 1.6°，已是接触静摩擦死区）。

## 4. 共模动量回收

反作用轮只有**转速变化**时才给车身力矩。P-only 航向环为了顶住静态误差会持续输出占空比，
于是共模转速单向累积（实测保持 20° 时 1050 → 1330 RPM），最终撞 `FLY_SPEED_LIMIT=8000` 停机。
差模有回收环（`RPM_A - RPM_B`），共模没有。

新增（`config.h`，`YAW_DESAT_ENABLE` 可整体关闭）：

```c
YAW_DESAT_RPM        (1200.0f)   // 超过该 |共模| 开始回收
YAW_DESAT_DPS        (3.0f)      // 让角速率 °/s
YAW_DESAT_LIMIT_DEG  (90.0f)     // 累计让角上限
```

20 ms 拍在 `roll_recovery_offset()` 里按超出门限的量累加 `s_yaw_desat_offset`，
累加进第 3 节的 `want`。让角本身产生反向转速变化把飞轮带回 0。

**它必须加在航向目标上，不能加在速率环设定值上**：加在环内会被航向环抵消（零和），
加在目标上则是"允许航向缓慢走一点"。

代价：保持航向时航向会缓慢漂移。这是反作用轮动量管理的固有代价，无法用增益规避。

## 5. 参数与验证

参数表 95 → 96 项（新增 `yaw_hold_deg`）。仓库内上位机 profile 未同步，需要时另行生成。

主机回归（本机 gcc，直接调用测试函数，未用 pytest）：

- `test_direction_baseline`：符号断言更新为新约定（`Y_RATE_KP==56`、存 13 → 读 13）。
  旧 Flash 的负值经 `param_normalize_yaw_sign()` 转为正值，其余参数不受影响。
- `test_angle_timing`、`test_roll_pi_direction`、`test_lean_limits`、
  `test_motor_encoder_menu`、`test_direction_limits`、`test_imu_forward`、
  `test_roll_photo_defaults`、`test_beacon_restore`、根目录菜单/驱动测试：全部通过。
- `test_linked_menu::test_same_draw_functions_font_and_image_renderer` **在改动前的
  HEAD 上就失败**（`render_group` / `render_group_live` 的渲染哈希在上游合并后未同步），
  与本次改动无关，未修改。

## 6. 上板顺序

1. ADS 全量编译，确认 0 错误 0 警告；核对 `.ads/build/latest.json` 与 HEX 哈希。
2. 上电静止，读回 `y_angle_kp` / `y_rate_kp`，确认已被归一化为正值。
3. 先进 Balance、`Yaw Hold=0`，手扭 20° 松手：应回正且不摆，共模转速不应持续爬升。
4. 再设 `Yaw Hold=20`，确认转到约 20° 并停住。
5. 若共模回收方向相反（越回收越涨），先把 `YAW_DESAT_ENABLE` 置 0 重新编译，
   说明本车的共模转速符号与本文假设不同，需要单独核对 `enc_dir_a/b`。

## 7. 实测结果（2026-11-11，烧录后）

烧录后 `y_angle_kp=+24`、`y_rate_kp=+60`（旧 Flash 的负值经归一化得到）。

| 项目 | 结果 |
|---|---|
| 静止 25 s | 航向漂移 <1°，yaw rate std 1.5，共模 −732 → −933 RPM |
| 进入 Balance 即刻 | 无自转，`yaw out` 稳态约 −900，不再发散 |
| `Yaw Hold=10` | 0.6 s 到 7.8°，10 s 内稳定在 8.5~8.7°（峰值 9.0°） |
| 共模转速（保持 10° 时） | 64 ~ 1089 RPM，未越过 1200 门限 |
| 残差 | 约 1.3°，为接触静摩擦死区，不是增益不足 |

共模动量回收（第 4 节）**尚未观察到实际触发**：保持 10° 时共模最高 1089 RPM，
低于 `YAW_DESAT_RPM`。需要更长时间保持或更大角度才能验证让角方向是否正确。

另：已烧录的固件里 `cfg_group()` 尚未识别 `yaw_` 前缀（本次已补），所以
`yaw_hold_deg` 在上位机 Schema 中显示为 `Other`，`cfg:save,Yaw` 也不包含它。
再次编译烧录后自动归入 Yaw 组；在此之前用 `cfg:save,all` 或机身 Yaw 页 Save 保存。

## 8. 首次上板发现的两个缺陷（同日修正）

烧录后实车反馈：**偏一个较大的角度后会沿偏差方向一直转过去，而不是往回转；
并且常伴随双飞轮占空比打满后倒车。** 两处都是本次改造引入的。

### 8.1 角度环输出没有限幅

`y_angle_kp=24` 在 40° 偏差下命令 `24×40=960 °/s`，内环 60 duty/(°/s) 需要 57600，
远超 10000 → 两个飞轮共模直接打满。饱和期间内环失去阻尼，飞轮冲向高速；
高速下力矩衰减，Roll 也随之失去权限，于是倒车。

修正：`yaw_cascade_ctrl()` 与 `yaw_test_ctrl()` 把角度环输出钳到
`YAW_RATE_CMD_LIMIT = 120 °/s`（与 `DIRECTION_YAW_RATE_LIMIT` 同量级）后再送内环。
钳的是前馈值，`y_angle_pid.out` 自身状态不变。

### 8.2 动量回收缺少门控，与大角度回正相互冲突

回收把让角写进航向目标。回正时车身朝负方向转、共模转速为负，
回收按设计把目标往正方向挪——**目标跟着偏差走，车便顺着偏差方向一直转，直到停在目标上**。
回收只在"已经稳住"时才有意义。

修正：增加两道门控，`YAW_DESAT_MAX_ERR_DEG = 3°`、`YAW_DESAT_MAX_OUTPUT = 6000`
（未打满）。航向误差大或输出已接近饱和时完全不让角。

### 8.3 参数层面的临时缓解（未重烧前）

现在板上仍是无门控、无限幅的固件，Flash 存的是 `y_angle_kp=24`。
在大角度下这组值会打满飞轮。重烧前可先用
`cfg:set` 把 `y_angle_kp` 降到 6~8（需 STOP），减小大误差下的指令幅值；
但这只是缓解，饱和与让角冲突仍在。
