# 程序执行与控制会话

## 用途与运行条件

网页、步骤程序和 Python SDK 共用 manager 的运动准入与到位核验。`pi_arm_execution` 管理程序、独占控制会话、暂停停止和运行记录；正常 `bringup.launch.py` 自动启动该节点。机械臂须已使能、反馈新鲜、模型与硬件控制条件有效。

连接客户端即可查询状态。程序启动或 SDK 进入 `control()` 后取得控制权；运动、等待和暂停期间，其他客户端不能插入运动。每 500 ms 续租，manager 默认在 3 s 未收到续租后撤销令牌并发起停止。SDK 另有连接心跳；程序的续租由树莓派负责。

| 操作 | 结果 |
| --- | --- |
| 暂停运动 | 当前步骤完成到位核验后暂停 |
| 暂停等待 | 暂停计时，保留剩余时间 |
| 继续 | 核对控制器、反馈和令牌后继续 |
| 停止 | 先禁止下一步，取消当前运动并请求保持当前位置 |
| 停止失败 | 保存原因和未确认状态，结束本次运行，不再派发 |

停止后不能继续原运行。SDK 异常退出、断线或心跳超时会停止并结束会话。网页关闭、刷新或断线不影响已启动的步骤程序。执行服务或 manager 重启后，旧运行不能自动续跑；重连后重新创建控制会话。

## 程序参数

程序采用 JSON schema 1，包含 `name` 和 `steps`。关节角与姿态使用度，末端位置使用毫米，时间使用秒。支持 `move`、`movej`、`movel`、`wait` 和单层 `repeat`；展开后最多 10000 步。

```json
{
  "schema_version": 1,
  "name": "点位往返",
  "steps": [
    {"type": "repeat", "count": 3, "steps": [
      {"type": "movej", "point_id": "点位A的ID", "params": {"vel": 2, "acc": 5}},
      {"type": "wait", "seconds": 1},
      {"type": "movej", "point_id": "点位B的ID", "params": {"vel": 2, "acc": 5}}
    ]}
  ]
}
```

运动步骤可引用 `point_id`，或在 `params` 中直接给目标：

| 运动 | 参数 |
| --- | --- |
| `move` | `targets` 六关节角；`max_speed` 度/秒 |
| `movej` | `target` 六关节角；`vel` 度/秒；`acc` 度/秒² |
| `movel` | `target=[x,y,z,rx,ry,rz]`；`linear_vel` 毫米/秒；`linear_acc` 毫米/秒²；`angular_vel` 度/秒；`angular_acc` 度/秒² |

`output_speed_limit` 为输出轴限速，范围 1/6–10 度/秒。关节和角速度不超过 10 度/秒。MoveL 旋转加速度由后端规划配置决定，`angular_acc` 不构成独立约束。引用关节点位时支持 `move` / `movej`，笛卡尔点位使用 `movel`。

启动会冻结完整步骤、点位值和参数。点位引用在启动时检查运行模式、模型、单位、关节顺序及参考信息；运行中编辑原点位或原程序只影响下次启动。失败、取消、停止未确认均不会进入下一步。

## WebSocket 调用

关节步进只提交选中关节的目标角。控制器从当前反馈取得其余关节角，每个轨迹周期用六个关节角统一换算各电机的位置，实现绳索联动补偿。单关节定位使用同一流程。反馈有效时，未选中关节越过指令限位最多 0.5°（含）可取该边界作为保持目标，超过 0.5° 则拒绝本次运动。选中关节的目标须在限位内。

地址为 `ws://树莓派IP:8765`。每次请求使用不同 `msg_id`；会产生运行或运动的请求另带稳定的 `request_id`，重查同一请求时使用原编号。

```json
{"type":"req","msg_id":1,"cmd":"session_open","params":{"name":"Python 控制","request_id":"客户端生成的UUID"}}
```

响应为 `{"type":"rsp","msg_id":1,"code":0,"ms":"ok","data":{...}}`，后台还会推送 `type=state`。`code=0` 表示请求被处理；运动成功以步骤的 `SUCCEEDED` 终态为准。

| 命令 | 参数 / 返回 |
| --- | --- |
| `get_info`、`get_state` | 控制器版本 `version`、协议版本 `protocol_version`、模式、能力、模型元数据 / 新鲜反馈和当前执行状态 |
| `session_open` | `name`、`request_id`；返回 `session_id`、`run_id`、控制器实例 |
| `session_heartbeat`、`session_close` | `session_id`；必须由创建会话的连接调用 |
| `move`、`movej`、`movel`、`wait` | 运动参数或 `seconds`，加 `session_id`、`request_id`；返回步骤编号 |
| `step_move` | `space=joint/position/orientation`、从零开始的 `axis`、`delta`、`request_id`；可附 `session_id` |
| `move_to_point` | `point` 为名称或 ID，`motion=move/movej/movel` 及速度参数、`request_id` |
| `task_get` | `step_id`；查询持久化步骤状态和 `task_id` |
| `task_cancel` | 运行编号 `run_id` 和步骤编号 `step_id`。取消当前步骤会停止所属会话，已结束的步骤返回原结果。 |
| `request_get` | `request_id`；返回 `accepted`，尚无受理记录时为 null |
| `execution_state` | 查询当前/最近一次运行 |
| `execution_pause/resume/stop` | 可附 `session_id` 或 `run_id` 核对目标运行；网页可控制当前运行 |
| `program_list/get/save/delete` | 列表、按 `id` 读取、`program`+`revision` 保存、按 `id`+`revision` 删除 |
| `program_validate` | `program`；返回展开步数和冻结目标 |
| `program_start` | `id` 或 `program`，以及 `request_id` |
| `run_list/get/export` | `limit` / `run_id` / `run_id`+`format=json/csv` |

旧式单次运动调用仍可使用。带 `request_id` 的单次运动进入执行服务，生成运行记录并去重；不带编号的兼容调用直接转给 manager，仍受最终控制权检查。相同编号、相同内容只受理一次；编号被用于不同内容时拒绝。停止和心跳独立处理，不等待普通请求完成。

## 返回结果与异常

`run_id` 标识一次程序或控制会话，`step_id` 标识其中一次调用，`task_id` 是 manager 的运动任务号。状态同时携带 `controller_instance_id`，不能只凭重启前的任务号判断新任务。运动受理先返回步骤编号，随后可查询到任务号。

运行状态包括 `STARTING`、`RUNNING`、`PAUSING`、`PAUSED`、`STOPPING`、`SUCCEEDED`、`CANCELED`、`FAILED`、`INTERRUPTED`。SDK 的 `MotionResult` 仅用于成功到位；其他终态抛出带编号、错误码和原因的 `ArmError`。

| 错误码 | 含义 |
| --- | --- |
| 1004 | 参数、版本或请求编号冲突 |
| 2001 | 状态/服务不可用 |
| 2003 | 控制权或运动被占用 |
| 3006 | 停止或位置保持未确认 |
| 3101 | 会话无效、已结束或令牌失效 |
| 3102 | SDK 连接或心跳丢失 |
| 3103 | 已暂停，暂不受理新步骤 |
| 3104 | 控制器/执行服务重启或任务反馈无法核对 |
| 3105 | 关键运行记录无法写入 |
| 3106 | 运动未成功结束 |
| 3107 | 客户端等待超时，已请求停止 |

manager 的具体规划、硬件和到位错误码会原样保留。请求超时先查 `request_get`，不要自动重发运动。运动等待超时应请求停止并检查 `stop_confirmed`；失败时保留运行记录和现场反馈以便排查。

## 数据与部署

默认记录为 `~/.local/share/pi_arm/execution-real.sqlite3` 和 `execution-mock.sqlite3`，通过 `execution_file` 参数改路径。SQLite 使用事务、WAL 和完整同步；运行快照、步骤终态、停止结果在树莓派持久保存。关键记录写入失败会禁止后续派发并请求停止。耗时用单调时钟，记录日期用 UTC。

点库继续使用 schema 1。首次修改既有点库前保留 `.v0.5.bak`；导出使用原始精度。名称/备注修改保留点位 ID 和来源，数值修改记录为手动来源；导入保留 ID 及原来源信息，并记录导入来源。先预检，再以同一库 revision 整批提交，冲突可跳过或更新相同 ID。

本次 ROS Action 增加 `control_token`，RobotState 增加控制器实例标识，并新增 `ControlSession`、`ExecutionCommand`、`ExecutionState`。升级时停止旧服务、统一重建相关包，再启动同一套 install；不要混用新旧生成接口。

ROS 客户端可通过 `/pi_arm/execution/command` 调用以上命令：`command`、JSON 字符串 `params_json`、客户端自选的稳定 `connection_id`；SDK 会话需续发心跳。`/pi_arm/execution/state` 发布当前运行。直接使用 manager 时，通过 `/pi_arm/control_session` acquire/renew/release 获得令牌，并把令牌放入运动 Goal。`StopMotion` 会撤销当前令牌。

Python 安装与完整调用示例见 [SDK](../sdk/python/README.md)。实现验证和实机待验项见 [版本记录](versioning.md)。
