# 技术资料索引

本目录保存电机协议、标定测量和历史审计资料。安装与运行见 [根 README](../README.md)，网页操作见 [前端 README](../web/README.md)，模块职责见 [包索引](../src/README.md)。

| 资料 | 用途 |
| --- | --- |
| [CAN 总线通讯协议 V2.6](CAN总线通讯协议V2.6.pdf) | 电机原始协议；文件内封面版本为 V2.36，引用时按命令字/章节核对 |
| [标定历史](calibration-history/README.md) | 原始测量、ROM 写入审计、历史配置与接受记录 |
| [绳索模型](../src/pi_arm_transmission/README.md) | 当前生产公式、通道关系、非线性与速度映射 |
| [机械模型](../src/pi_arm_description/README.md) | 主动/随动关节、坐标系、限位与模型来源 |
| [硬件配置](../src/pi_arm_hardware/config/README.md) | 当前 R10、协议/输出/关节角、标零与配置变更 |
| [MoveIt 启动与退出](../src/pi_arm_moveit_config/README.md) | 规划配置与系统 MoveIt 2.12.4 的退出兼容处理 |

旧开发计划、审查快照、阶段验收和示例截图可在恢复标签 `repeatability-v1.0`（`95b65be`）查看。历史审计文件中的旧文档路径按该 Git 版本追溯。

## 原始资料维护

协议 PDF、已接受标定、对应模型快照、采样和总线审计配套保存，用于复现实机参考。新增测量注明采样条件、单位、配置和时间；操作步骤更新到对应目录 README。
