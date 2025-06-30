/**
 * @page page_MotorControlInterface 电机控制接口
 *  电机控制接口提供了步进电机移动的相关操作。
 *
 *  具体命令见： @ref module_MotorControlInterface
 *
 * @section sec_MCI_ChangeLog 版本变更历史
 *  接口历史变更记录：
 *  - 1.0.0 基本版本 (2018.1.17)
 *
 */

/**
 * @addtogroup module_MotorControlInterface 电机控制接口
 * @{
 */

/**
 * @file
 * @brief 电机控制接口。
 * @details 定义了一序列步进电机控制相关的操作。
 * @version 1.0.0
 * @author yongxi
 * @date 2018.1.17
 */

#ifndef DSCP_MOTOR_CONTROL_INTERFACE_H_
#define DSCP_MOTOR_CONTROL_INTERFACE_H_

#define DSCP_MCI_CBASE                  0x0000 + 0x0600     ///< 命令基值
#define DSCP_MCI_EBASE                  0x8000 + 0x0600     ///< 事件基值
#define DSCP_MCI_SBASE                  0x0000 + 0x0600     ///< 状态基值


// *******************************************************************
// 命令和回应
/**
 * @brief 电机移动模式。
 */
//enum MotorMoveMode
//{
//     MOTOR_MOVE_ABSOLUTE_MODE = 0,       ///< 绝对模式。
//     MOTOR_MOVE_RELATIVE_MODE = 1,         ///< 相对模式。
//     MOTOR_MOVE_SAFE_MODE = 2         ///< 安全模式。
//};


/**
 * @brief 查询系统操作的步进电机数目。
 * @return 总电机数目， Uint16。
 */
#define DSCP_CMD_MCI_GET_TOTAL_MOTORS           (DSCP_MCI_CBASE + 0x00)

/**
 * @brief 查询指定电机的运动参数。
 * @details 系统将根据设定的运动参数进行运动控制和规划。
 * @param index Uint8，要查询的电机索引。
 * @return 运动参数， 数据格式为：
 *  - acceleration Float32，加速度，单位:步/平方秒。
 *  - speed Float32，电机最大运转速度，单位: 步/秒。
 * @see DSCP_CMD_MCI_SET_MOTION_PARAM
 */
#define DSCP_CMD_MCI_GET_MOTION_PARAM           (DSCP_MCI_CBASE + 0x01)

/**
 * @brief 设置指定电机的运动参数。
 * @details 系统将根据设定的运动参数进行运动控制和规划。运动参数将永久保存。
 * @param index Uint8，要设置的电机索引。
 * @param acceleration Float32，加速度，单位:步/平方秒。
 * @param speed Float32，电机最大运转速度，单位: 步/秒。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @see DSCP_CMD_MCI_GET_MOTION_PARAM
 */
#define DSCP_CMD_MCI_SET_MOTION_PARAM           (DSCP_MCI_CBASE + 0x02)

/**
 * @brief 查询指定电机的工作状态。
 * @param index Uint8，要查询的电机索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_IDLE 空闲；
 *  - @ref DSCP_BUSY 忙碌，需要停止后才能做下一个动作；
 */
#define DSCP_CMD_MCI_GET_MOTOR_STATUS           (DSCP_MCI_CBASE + 0x03)

/**
 * @brief 查询最大坐标值。
 * @param void。
 * @return  坐标 Motor3DPos, 包含3个Int32类型, 分别对应x,y,z的极限位置。
 */
#define DSCP_CMD_MCI_GET_MOTOR_MAX_POS        (DSCP_MCI_CBASE + 0x04)

/**
 * @brief 查询指定电机的初始位置步数。
 * @param index Uint8，要查询的电机索引。
 * @return 步数， Uint16。
 */
#define DSCP_CMD_MCI_GET_MOTOR_INIT_STEPS       (DSCP_MCI_CBASE + 0x05)

/**
 * @brief 查询当前坐标。
 * @param void。
 * @return 坐标 Motor3DPos, 包含3个Int32类型, 分别对应x,y,z的当前位置。
 */
#define DSCP_CMD_MCI_GET_MOTOR_CURRENT_POS    (DSCP_MCI_CBASE + 0x06)

/**
 * @brief 电机移动。
 * @details 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
 * @param index Uint8，要操作的电机索引。
 * @param steps short16，电机移动步数，正数时为正向转动，负数时为反向移动。
 * @param mode Uint8, 电机移动模式,（ @ref MotorMoveMode ）, 定义如下：
 *  - @ref MOTOR_MOVE_ABSOLUTE_MODE, 绝对模式,此时steps参数指电机相对初始位置转动的步数；
 *  - @ref MOTOR_MOVE_RELATIVE_MODE, 相对模式,此时steps参数指电机相对当前位置转动的步数；
 *  - @ref MOTOR_MOVE_SAFE_MODE, 安全模式，此时steps参数指电机相对当前位置转动的步数，如Z轴电机不在初始位置，移动X轴电机会先复位Z轴电机。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败，如电机正在运转，无法启动电机，需要先停止；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @note 该命令将立即返回，电机移动完成将以事件的形式上报。
 */
#define DSCP_CMD_MCI_MOTOR_MOVE                 (DSCP_MCI_CBASE + 0x07)

/**
 * @brief 停止电机。
 * @param index Uint8，要操作的电机索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 */
#define DSCP_CMD_MCI_MOTOR_STOP                 (DSCP_MCI_CBASE + 0x08)

/**
 * @brief 电机复位。
 * @details 电机回到0位置，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
 * @param index Uint8，要操作的电机索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 */
#define DSCP_CMD_MCI_MOTOR_RESET                (DSCP_MCI_CBASE + 0x09)

/**
 * @brief 查询指定位置传感器的状态。
 * @param index Uint8，要查询的位置传感器索引。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_IDLE 空闲, 传感器未被遮挡；
 *  - @ref DSCP_BUSY 忙碌，传感器被遮挡。
 */
#define DSCP_CMD_MCI_GET_SENSOR_STATUS           (DSCP_MCI_CBASE + 0x0A)

/**
 * @brief 真空泵及匀质器控制接口。
 * @param index Uint8，要查询的位置电机驱动索引。
 * @return gstat Uint8，驱动全局寄存器的值
 */
#define DSCP_CMD_MCI_EXTRADEVICE_CONTROL           (DSCP_MCI_CBASE + 0x0B)

/**
 * @brief 电机移动。
 * @details 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
 * * @param mode Uint8, 电机移动模式,（ @ref MotorMoveMode ）, 定义如下：
 *  - @ref MOTOR_MOVE_ABSOLUTE_MODE, 绝对模式,此时steps参数指电机相对初始位置转动的步数；
 *  - @ref MOTOR_MOVE_RELATIVE_MODE, 相对模式,此时steps参数指电机相对当前位置转动的步数；
 *  - @ref MOTOR_MOVE_SAFE_MODE, 安全模式，此时steps参数指电机相对当前位置转动的步数，如Z轴电机不在初始位置，移动X轴电机会先复位Z轴电机。
 * @param Motor3DPos xyzPos，包含XYZ三个电机的移动步数，正数时为正向转动，负数时为反向移动。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败，如电机正在运转，无法启动电机，需要先停止；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @note 该命令将立即返回，电机移动完成将以事件的形式上报。
 */
#define DSCP_CMD_MCI_MOTOR_MOVE_XYZ                 (DSCP_MCI_CBASE + 0x0C)

/**
 * @brief SY11注射器控制
 * @details 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
* @param index Uint8, 开启的阀号, 支持1-4
 * @param mode Uint8, 注射器模式, 定义如下：
 *  - @ref SY11_PUMP, 抽液模式,此时float参数指抽液体积，单位uL；
 *  - @ref SY11_DRAIN, 排液模式,此时float参数指排液体积，单位uL；
 *  - @ref SY11_ABS, 绝对位置模式，比如满行程对应1000uL,设置500uL，不管当前所处体积是何处直接走500uL体积的位置
 * * * @param float 体积,单位：uL,最大不超过1000uL
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败，如电机正在运转，无法启动电机，需要先停止；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @note 该命令将立即返回，电机移动完成将以事件的形式上报。
 */
#define DSCP_CMD_MCI_SY11_STRAT                     (DSCP_MCI_CBASE + 0x0D)

/**
 * @brief SY11注射器停止
 * @details 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
* @param void
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败，如电机正在运转，无法启动电机，需要先停止；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @note 该命令将立即返回，电机移动完成将以事件的形式上报。
 */
#define DSCP_CMD_MCI_SY11_STOP                 (DSCP_MCI_CBASE + 0x0E)

/**
 * @brief SY11注射器复位
 * @details 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。关联的事件有：
 *   - @ref DSCP_EVENT_MCI_MOTOR_RESULT
* @param void
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败，如电机正在运转，无法启动电机，需要先停止；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 * @note 该命令将立即返回，电机移动完成将以事件的形式上报。
 */
#define DSCP_CMD_MCI_SY11_RESET                 (DSCP_MCI_CBASE + 0x0F)

/**
 * @brief 查询注射器当前体积。
 * @param void。
 * @return  体积 float 单位uL 。
 */
#define DSCP_CMD_MCI_GET_SY11_CURRENT_VOL        (DSCP_MCI_CBASE + 0x10)

/**
 * @brief 设置机械臂运行速度。
 * @param Uint8 speed;
 * @details 速度为档位值，可调档位0-15:20~3000rpm，默认转速7档:800rpm
 * | A   | 对应参数  | 默认值(rpm) |
	|-----|-----------|-------------|
	| 0   | P5.060    | 20          |
	| 1   | P5.061    | 50          |
	| 2   | P5.062    | 100         |
	| 3   | P5.063    | 200         |
	| 4   | P5.064    | 300         |
	| 5   | P5.065    | 500         |
	| 6   | P5.066    | 600         |
	| 7   | P5.067    | 800         |
	| 8   | P5.068    | 1000        |
	| 9   | P5.069    | 1300        |
	| 10  | P5.070    | 1500        |
	| 11  | P5.071    | 1800        |
	| 12  | P5.072    | 2000        |
	| 13  | P5.073    | 2300        |
	| 14  | P5.074    | 2500        |
	| 15  | P5.075    | 3000        |
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 *  - @ref DSCP_ERROR_PARAM 参数错误，传入的参数有问题
 */
#define DSCP_CMD_MCI_SET_MOTOR_RUN_SPEED        (DSCP_MCI_CBASE + 0x11)

/**
 * @brief 获取机械臂运行速度。
 * @return Uint8 speed;
 * @details 速度为档位值，可调档位0-15:20~3000rpm，默认转速7档:800rpm
 */
#define DSCP_CMD_MCI_GET_MOTOR_RUN_SPEED        (DSCP_MCI_CBASE + 0x12)

// *******************************************************************
// 事件

/**
 * @brief 电机操作结果。
 */
//enum MotorResult
//{
//    MOTOR_RESULT_FINISHED = 0,       ///< 电机操作正常完成。
//    MOTOR_RESULT_FAILED = 1,         ///< 电机操作中途出现故障，未能完成。
//    MOTOR_RESULT_STOPPED = 2         ///< 电机操作被停止。
//};

/**
 * @brief 电机操作结果事件。
 * @details 电机移动操作结束时将产生该事件。
 * @param index Uint8，产生事件的泵索引，0号泵为光学定量泵。
 * @param result Uint8，泵操作结果码（ @ref MotorResult ），定义如下：
 *  - @ref MOTOR_RESULT_FINISHED  电机操作正常完成；
 *  - @ref MOTOR_RESULT_FAILED  电机操作中途出现故障，未能完成。
 *  - @ref MOTOR_RESULT_STOPPED  电机操作被停止。
 */
#define DSCP_EVENT_MCI_MOTOR_RESULT             (DSCP_MCI_EBASE + 0x00)
// *******************************************************************
// 状态返回



#endif // DSCP_MOTOR_CONTROL_INTERFACE_H_

/** @} */
