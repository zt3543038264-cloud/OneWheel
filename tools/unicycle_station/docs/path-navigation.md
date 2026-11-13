# 0.6.0 路线设计与惯性导航

更新时间：2026-09-16。需要配套包含 `path1` 接口的车端固件。普通 Remote 的操作方式保持原样。

## 使用

1. 打开 **路线与惯导**。选择长度，点 **生成直线路线**；也可勾选地图添加航点，或编辑右侧 X/Y 表格，再点 **按航点生成路线**。
2. 坐标单位为米。第一个点固定 `(0,0)`，`+Y` 是车端锁定的路线起始方向（首次未锁定时取当前车头方向），`+X` 向右。青色线是编辑路线。路线按所选间距采样，最多 256 点，超限会提示。
3. 可保存/打开 JSON 路线文件。文件含坐标、坐标约定、版本、速度上限；可以在 PC 保存多条路线，车辆仍只保存一条当前路线。
4. 连接车辆，等待握手与 STOP 确认，点 **上传到车内 RAM**。每个点收到应答才发送下一个点，最后核验 CRC 并提交。成功后黄色线显示车内路线。上传不自动发车。
5. 需要掉电保留时，点 **保存车内路线到 Flash**。车端要求车轮稳定停止约 500 ms、接收空闲至少 100 ms，并回读验证。失败应答不会显示保存成功。
6. 选择 **0.05–1.50 m/s** 的执行速度上限，默认 0.20。把车放到起点、朝向图中 +Y；在车端 Path → Lock Start Heading 锁定，然后刷新车内路线，点 **执行已核对的车内路线**，核对长度和速度后确认。
7. 绿色轨迹与红色车位来自车端编码器/IMU 估计；状态显示路径进度、目标速度、故障原因、RAM 是否未保存。到达终点后保持平衡；使用顶部立即停车、空格或车身返回键完全停机。

上传、下载、Flash 操作均要求停车。下载完成后可点“车内路线复制到编辑区”继续编辑。加载 Flash、车身修改或停止造成路线版本变化后，先下载核对，再启动。

路线自主运行不要求 PC 连续发送速度命令；断开串口不会自动结束车端导航。现场应能使用车身返回键停机。这里是轮式里程计与惯性航向融合的相对导航，存在航向漂移、轮胎打滑和里程比例误差，没有绝对定位或避障。

折线路线保留航点形状，急拐角会触发提前减速。设计缓弯时请沿圆弧布置多个航点；不支持瞬间拐角或倒车段。设定 1.50 m/s 是速度上限，实际目标还受曲率、终点制动、航向误差与飞轮动量权限约束。首次实车使用仍先验证 0.20 m/s 短直线和缓弯。

示例文件：[3 米直线](../examples/routes/straight_3m.json)、[半径约 2 米右转缓弯](../examples/routes/right_turn_r2m.json)，均默认 0.20 m/s。

### 航向基准（2026-10-08）

同一上电会话、同一 RAM 路线的重复发车保留锁定方向，Yaw 角闭环在向前行驶中纠正小偏角。首次未锁定时自动记住当前方向，无法判断首次是否摆正。相对基准超过 30° 会返回 `START_BLOCKED`，需要摆正；锁定方向、停车或换路线导致版本变化后重新刷新/下载路线。上位机现有 nav 帧和启动命令保持兼容，基准值在车端 Path 页面查看和重设。新建/上传/加载路线、清空或重启清除基准；Flash 不保存绝对 Yaw。

### Mock

点击 **Mock 演示**，再点立即停车使模拟车保持 STOP，即可上传、下载、保存、加载、启动并查看模拟轨迹。Mock 按理想折线移动，只验证软件交互，不模拟独轮车动力学或证明实车性能。回放可显示记录中的 `nav` 位姿，设备操作保持只读。

## path1 协议

沿用 cfg v1 和请求序号，命令不含 CR/LF 最多 63 字节。成功为 `rsp:<seq>,ok,<operation>,...`，错误为 `rsp:<seq>,err,<code>,<detail>`。旧 `stat/run/task` 字段数保持不变。hello 当前标识 `tc264-cfg1-task1-path1-sign2`，上位机另查询 `path_status` 确认能力。

| 请求（`cfg:` 后） | 成功应答字段（`ok,` 后） | 说明 |
|---|---|---|
| `path_status,seq` | `path_status,1,256,1.50` | 订阅约 5 Hz 的 nav，并立即发送一次 |
| `path_begin,seq,count` | `path_begin,token` | 2–256 点；token 等于本次 begin 的 seq |
| `path_point,seq,token,index,x_mm,y_mm` | `path_point,index` | 从 0 顺序发送；坐标为 ±1000000 内的整数毫米 |
| `path_commit,seq,token,crc32` | `path_commit,revision` | CRC、点数、几何均通过后原子替换 RAM |
| `path_cancel,seq` | `path_cancel` | 取消未提交传输/启动令牌/待处理路径存储，不停车 |
| `path_get,seq,revision,index` | `path_get,index,x_m,y_m` | 读出米坐标；版本不一致拒绝，避免拼接两条路线 |
| `path_arm,seq,revision` | `path_arm,token` | 对已核对版本签发 3 秒内有效的一次性启动令牌 |
| `path_start,seq,token,speed_mps` | `path_start` | 0.05–1.50；令牌立即失效，仍须通过车端 Balance 门槛 |
| `path_save,seq` | `path_save,VERIFIED` | 延迟到接收空闲后保存，回读验证成功才确认 |
| `path_load,seq` | `path_load,VERIFIED` | 延迟到接收空闲后读取有效 Flash 路线 |

CRC 为标准 CRC32/IEEE：依次把每点 `x_mm,y_mm` 编成小端有符号 int32，按字节计算（Python `zlib.crc32`）。`(0,0),(0,200),(0,400)` 的 CRC 为十进制 `1240199005`。车端自行从坐标计算累计折线长度，不信任 PC 传入的里程。

上传缓存独立于当前路线。重复/乱序点、5 秒无点更新、路线版本变化、进入运行、错误 CRC 或非法几何会阻止提交。未提交前当前路线保留；提交/保存应答丢失时结果不确定，PC 不自动重发，应下载核对。断线不恢复旧上传会话，也不自动续发启动。立即停车走原有优先 `stop` 通道，并撤销未完成传输和启动令牌。启动令牌也绑定车内路线版本；过期和重复使用均拒绝。

`nav` 共 15 个字段：

```text
nav:uptime_ms,state,reason,count,segment,x_m,y_m,yaw_deg,progress_m,length_m,speed_target_mps,dirty,valid,active,revision
```

- `state`：0 空闲，1 录制，2 就绪，3 运行，4 制动，5 完成，6 故障。
- `reason`：0 正常，1 容量满，2 回退过多，3 输入异常，4 离轨，5 停滞/制动超时，6 取消。
- `active` 包含完成后的平衡保持；不能把 state=5 当作电机已停止。
- `revision` 为 RAM 路线/任务代次，和参数 revision、Flash 保存代数无关。
- `yaw_deg` 相对本次起始方向。坐标与速度为车端估计/目标，不是外部测量。

## 源码与验证

车端：[vofa_path.inc](../../../code/Protocols/vofa_path.inc)、[path_nav.c](../../../code/Controller/Planner/path_nav.c)。PC：[路线服务](../app/services/routes.py)、[路线页面](../app/ui/route_page.py)。

验证入口：`python tools/tests/test_path_nav.py`、上位机 `pytest`、`python tools/ads/build.py`。GUI `--smoke` 会包含新增页面。详见 [车端路径指南](../../../docs/guides/路径导航指南.md)。构建和 Mock 测试不代表完成烧录或实车验收。
