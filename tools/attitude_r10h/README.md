> 历史核心测试依赖，不是当前 IMU660RA 解算入口。旧 pre_migration 源码备份已从整理副本移除，原上传 zip 仍保留。

# A+ r10h 离线接入检查

运行 `python tools/attitude_r10h/run_tests.py --gcc C:/mingw64/bin/gcc.exe`。可执行程序只在系统临时目录生成并自动清理；不使用串口、不烧录、不驱动电机。结果保存在 host_results.json。

integration.c 编译真实 imu.c、attitude.c 与 r10h 核心，测试时以 R10H_HOST_TEST 替代 QSPI 寄存器传输，并模拟 STM。api_contract、core_regression、strong_vibration 来自原移植包；后两项输入已适配本车采用的单位矩阵，保留原阈值。bounded_spi.c 另行编译实际目标 QSPI 等待实现，使用寄存器模拟验证截止时间、释放片选及计时器回绕。真实 QSPI 电气行为尚未在硬件验证。
