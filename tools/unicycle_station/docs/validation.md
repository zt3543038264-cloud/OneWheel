# 本次整理验证范围

2026-11-16：整理上传源码，不是一次新固件或新 Windows EXE 发布。

- 保留 app、Profile、启动和打包脚本、tests、examples、connection.json、parameters.preset.json 的原始内容。
- 删除 Python/pytest 缓存、可重建 egg-info 和旧发布/清理记录。
- 修正 README 将缺失 dist 称为唯一发行入口的问题，实际源码启动使用 start_windows.bat。
- 历史测试通过次数、旧发布 hash 不能作为当前包测试结果，已从本页移除。
- 参数保存、开机读取、独立平衡、视觉导航没有因本轮整理得到硬件验证。

本包不含完整固件源码；依赖 firmware/code/libraries 的测试需要放回完整工程后运行。Windows BAT、PyInstaller 打包与真实桌面显示不能由 Linux 静态检查替代。

## 本轮实际检查

- 82 个 Python 文件语法检查通过；JSON 可解析。
- 102 个运行源码、配置、示例、测试及启动相关文件与上传原件字节一致。
- 模型、协议、重同步、握手诊断、WiFi、Pitch 预览、优先级/回放 7 个专项共 78 项通过（仅离线/本机回环）。
- Linux offscreen Mock 启动、11 页捕获与退出完成，退出码 0；OpenGL context 无法创建，3D 画面不算验证通过。
- 较广泛 GUI 测试在 10 项后未继续推进，已中断；不宣称完整 GUI 测试通过。
- 未运行 Windows BAT、Windows 打包或实车连接，未执行依赖缺失固件源码的测试。
