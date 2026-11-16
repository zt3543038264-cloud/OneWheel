# Embedded Station · 独轮车上位机

整理日期：2026-11-16。本包为上传源码的整理副本，不是 Windows EXE 发行版。

## 启动

在原工程 `tools/unicycle_station` 中运行 **start_windows.bat**。
首次启动需要 Python 3.11+，脚本会按 pyproject.toml 在 `.venv` 安装所需依赖。
`start_app.bat` 只用于已经生成 `dist/EmbeddedStation/EmbeddedStation.exe` 的情况；本包没有 dist，不要因该入口提示而反复运行构建。
`build_windows.bat` 用于在 Windows 测试并打包，源码启动不要求先打包。
源码更新不自动更新旧 EXE。

## 当前用途

- 连接设置、通信诊断、参数读取与回读、姿态与波形、记录/回放。
- **调参工作台 → Pitch** 的角速度/角度计算预览，见 [预览说明](docs/pitch-preview-gui.txt)。预览不是电机动力测试。
- 当前主轮测试暂停。整车尚未验收脱手站稳，视觉到点是后续目标。
- 参数以当前固件 Schema 和实际回读为准；Mock/Profile/示例不代表实车状态或已调好参数。
- 当前固件手动参数 Save 已启用但待实测；开机参数读取关闭、路径 Flash 关闭。界面有按钮不意味着相应固件功能已启用或验证。

## 连接

选择 WiFi 并填写实际车端地址、源端口和电脑监听端口；历史观测为 192.168.0.113:5001、电脑 8081，不保证当前相同。
同一时间不要让其他脚本占用同一监听端口。TX/RX 灯闪烁不等于协议握手完成。
连接故障见 [握手排查](docs/handshake-diagnosis.md)、[WiFi-SPI](docs/wifi-spi.md)。

## 文档与测试

[交互](docs/interaction-workflow.md) · [参数与在线调试](docs/live-tuning.md) · [记录/自定义](docs/personalization-workflow.md) · [协议](docs/protocol.md) · [验证范围](docs/validation.md)

保留全部源码、Profile、固件测试夹具及 examples；测试可能引用这些数据，不应当作缓存删除。
历史测量 CSV/PNG、参数快照仍保留原路径，仅供历史分析。旧实车参数不能直接用作新车默认值。
`connection.json`、`parameters.preset.json` 保留不变，不主动发送、自动应用或修改它们。

本次只清理文档及可再生成缓存，不连接车辆、不改电机控制、不写 Flash。
