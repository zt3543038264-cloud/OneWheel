> 历史旧驱动覆盖补丁。当前驱动已另行更新，禁止把本目录当作最新驱动安装；保留用于源码/许可证参考。

# CYT2BL3 双驱修复源码

来自用户提供的逐飞 `CYT2BL3_Brushless_Driver_Project-master/Seekfree_CYT2BL3_Double_Foc_Project/project/code`，源文件保留逐飞 GPL-3.0-or-later 版权声明；许可证见 [LICENSE.md](LICENSE.md)。仓库内存为UTF-8，安装脚本转回原工程的GBK/CRLF。

这里提供可覆盖到原IAR工程的修订文件，不是完整SDK或可直接下载的驱动固件；TC264 ADS不会编译此目录。

实际更新两个文件：

- `motor_driver_uart_control.c`：按帧头区分二进制与文本；二进制负载的0x0A不再被当作换行；每帧重新计算校验；排队指令逐条应用；文本PWM严格检查数值和长度，超长文本有界丢弃。
- `motor_control.c`：左右电机占空比降到过载判断门限以下后，清除过载累计时间；持续过载/堵转仍会锁存保护，零指令不会自动清故障。

`driver_config.h`、`motor_driver_uart_control.h` 为原版配置/接口参考及主机测试依赖，安装脚本不覆盖它们。未更改驱动模式、KV、引脚、保护阈值或失联保护开关。

在原驱动工程上安装：

```powershell
python tools/cyt2bl3_driver/apply_overlay.py --project "C:\Users\Lenovo\Desktop\CYT2BL3_Brushless_Driver_Project-master\Seekfree_CYT2BL3_Double_Foc_Project\project"
```

脚本先核对两个目标文件，确认是已审查的原版或本修订版后才写入；遇到其他修改会拒绝覆盖。可追加 `--check` 仅核对。已经同步修改了本机上述驱动工程中的这两个C文件。

安装后用原工程指定的IAR环境编译，再通过与板卡匹配的DAPLink/SWD烧录流程更新驱动板。本次只完成保护逻辑和解析器的主机回归，未完成IAR整板构建、驱动烧录或实车验证；不要把TC264生成的HEX烧给CYT2BL3。

主控默认 `QCAR_DRIVER_TEXT_DUTY=1`，使用原驱动已有的文本命令避开二进制解析问题；可先更新主控。仅当驱动侧修订已经烧入并验证，才切换为0使用二进制占空比。

回归：`python tests/run_all.py`，也可加 `--compiler <zig.exe>`。`check_driver_link.py` 覆盖真实Mode3/5/6/8→真实串口发送→原版/修订驱动解析器，扫描8001组独立左右PWM；`check_driver_protection.py` 分别覆盖两路保护和间断过载计数。
