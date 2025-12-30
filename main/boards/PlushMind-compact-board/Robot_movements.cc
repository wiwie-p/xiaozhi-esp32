#include "Robot_movements.h"

#include <algorithm>

#include "freertos/idf_additions.h"
#include "oscillator.h"

static const char* TAG = "RobotMovements";

// 左手初始角度值
#define LEFT_HAND_HOME_POSITION 45
// 右手初始角度值
#define RIGHT_HAND_HOME_POSITION 45

Robot::Robot() {
    is_Robot_resting_ = false;
    has_hands_ = false;
    // 初始化所有舵机管脚为-1（未连接）
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Robot::~Robot() {
    DetachServos();
}

unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief 初始化机器人对象
 * 
 * @param head 头部舵机的引脚编号
 * @param right_hand 右手舵机的引脚编号，-1表示没有右手舵机
 * @param left_hand 左手舵机的引脚编号，-1表示没有左手舵机
 */
void Robot::Init(int head, int left_hand, int right_hand) {
    servo_pins_[HEAD] = head;
    servo_pins_[LH] = left_hand;
    servo_pins_[RH] = right_hand;

    // 检查是否有手部舵机
    has_hands_ = (left_hand != -1 && right_hand != -1);

    AttachServos();
    is_Robot_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
/**
 * @brief 将舵机连接到指定的引脚
 * 
 * 该函数遍历所有舵机，根据预设的引脚配置将舵机连接到对应的GPIO引脚
 * 只有当引脚配置不为-1时才会执行连接操作
 * 
 * @param 无参数
 * @return 无返回值
 */
void Robot::AttachServos() {
    // 遍历所有舵机并连接到对应的引脚
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Attach(servo_pins_[i]);
        }
    }
}

/**
 * @brief 断开机器人所有舵机的连接
 * 
 * 该函数遍历所有舵机引脚，对于已分配引脚的舵机执行断开操作，
 * 释放舵机控制资源，使舵机进入自由转动状态
 * 
 * @param 无参数
 * @return 无返回值
 */
void Robot::DetachServos() {
    // 遍历所有舵机并断开已分配的舵机连接
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].Detach();
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
/**
 * @brief 设置机器人各舵机的微调值
 * 
 * 该函数用于设置机器人头部和手部舵机的微调值，如果机器人配备手部，
 * 则同时设置左右手的微调值，并将这些微调值应用到对应的舵机上。
 * 
 * @param head 头部舵机的微调值
 * @param right_hand 右手舵机的微调值
 * @param left_hand 左手舵机的微调值
 * @return 无返回值
 */
void Robot::SetTrims(int head, int right_hand, int left_hand) {
    XXX :微调值先全设为0
    servo_trim_[HEAD] = head;
    servo_trim_[LEFT_HAND] = left_hand;
    servo_trim_[RIGHT_HAND] = right_hand;
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
/**
 * @brief 控制机器人舵机移动到目标位置
 * 
 * 该函数将机器人的所有舵机在指定时间内平滑移动到目标位置，
 * 如果时间较短则直接设置目标位置，如果时间较长则分步移动以实现平滑过渡
 * 
 * @param time 移动所需的时间（毫秒），如果时间大于10ms则采用平滑移动，否则直接设置位置
 * @param servo_target[] 目标舵机位置数组，包含每个舵机的目标角度值
 * @return 无返回值
 */
void Robot::MoveServos(int time, int servo_target[]) {
    // 如果机器人处于休息状态，则设置为非休息状态
    if (GetRestState() == true) {
        SetRestState(false);
    }

    final_time_ = millis() + time;
    if (time > 10) {
        // 计算每个舵机每次移动的增量值
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                increment_[i] = (servo_target[i] - servo_[i].GetPosition()) / (time / 10.0);
            }
        }

        // 分步执行舵机移动，每次移动10ms
        for (int iteration = 1; millis() < final_time_; iteration++) {
            partial_time_ = millis() + 10;
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_[i].GetPosition() + increment_[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        // 时间较短时直接设置舵机到目标位置
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].SetPosition(servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
    }

    // 最终调整以确保所有舵机都精确到达目标位置
    bool f = true;
    int adjustment_count = 0;
    while (f && adjustment_count < 10) {
        f = false;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1 && servo_target[i] != servo_[i].GetPosition()) {
                f = true;
                break;
            }
        }
        if (f) {
            for (int i = 0; i < SERVO_COUNT; i++) {
                if (servo_pins_[i] != -1) {
                    servo_[i].SetPosition(servo_target[i]);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            adjustment_count++;
        }
    };
}

/**
 * @brief 控制单个舵机移动到指定位置
 * 
 * @param position 目标位置角度值，范围应为0-180度
 * @param servo_number 舵机编号，用于指定要控制的舵机
 * @return void
 */
void Robot::MoveSingle(int position, int servo_number) {
    // 限制位置角度在0-180度范围内，超出范围则设置为90度
    if (position > 180)
        position = 90;
    if (position < 0)
        position = 90;

    // 检查并更新机器人状态，如果处于休息状态则切换到非休息状态
    if (GetRestState() == true) {
        SetRestState(false);
    }

    // 验证舵机编号有效性并执行舵机位置设置
    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1) {
        servo_[servo_number].SetPosition(position);
    }
}

/**
 * @brief 控制机器人舵机进行振荡运动
 * 
 * 该函数通过设置振荡参数使机器人多个舵机按照指定的振幅、偏移、周期和相位差进行振荡运动
 * 
 * @param amplitude 舵机振荡幅度数组，每个元素对应一个舵机的振荡幅度
 * @param offset 舵机中位偏移数组，每个元素对应一个舵机的中位偏移值
 * @param period 振荡周期，所有舵机共享相同的振荡周期
 * @param phase_diff 舵机相位差数组，每个元素对应一个舵机相对于参考相位的相位差
 * @param cycle 振荡循环次数，默认为1次
 * @return 无返回值
 */
void Robot::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                           double phase_diff[SERVO_COUNT], float cycle = 1) {
    // 配置每个有效舵机的振荡参数
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    // 计算振荡运动的总执行时间
    double ref = millis();
    double end_time = period * cycle + ref;

    // 在指定时间内持续更新舵机位置以实现振荡效果
    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
            }
        }
        vTaskDelay(5);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * @brief 执行机器人伺服电机振荡运动
 * 
 * 该函数控制机器人伺服电机按照指定的振幅、偏移、周期和相位差执行振荡运动
 * 可以执行完整周期和部分周期的运动
 * 
 * @param amplitude 伺服电机振幅数组，每个伺服电机的振荡幅度
 * @param offset 伺服电机偏移数组，每个伺服电机的基准位置偏移
 * @param period 振荡周期，控制运动的速度
 * @param phase_diff 伺服电机相位差数组，每个伺服电机之间的相位差
 * @param steps 要执行的步数/周期数，默认为1.0
 * @return 无返回值
 */
void Robot::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                   double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    //-- Execute the final not complete cycle
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

//---------------------------------------------------------
//-- Execute2: 使用绝对角度作为振荡中心
//--  Parameters:
//--    amplitude: 振幅数组（每个舵机的振荡幅度）
//--    center_angle: 绝对角度数组（0-180度），作为振荡中心位置
//--    period: 周期（毫秒）
//--    phase_diff: 相位差数组（弧度）
//--    steps: 步数/周期数（可为小数）
//---------------------------------------------------------
void Robot::Execute2(int amplitude[SERVO_COUNT], int center_angle[SERVO_COUNT], int period,
                    double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    // 将绝对角度转换为offset（offset = center_angle - 90）
    int offset[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        offset[i] = center_angle[i] - 90;
    }

    int cycles = (int)steps;

    //-- Execute complete cycles
    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    //-- Execute the final not complete cycle
    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Robot at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Robot::Home(bool hands_down) {
    if (is_Robot_resting_ == false) {  // Go to rest position only if necessary
        int homes[SERVO_COUNT];
        homes[HEAD] = 90;
        homes[LH] = LEFT_HAND_HOME_POSITION;     // 左手位置
        homes[RH] = RIGHT_HAND_HOME_POSITION;    // 右手位置
        
        MoveServos(700, homes);
        is_Robot_resting_ = true;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
}

/**
 * @brief 获取机器人当前的休息状态
 * 
 * @return bool 返回机器人是否处于休息状态
 *         true: 机器人正在休息
 *         false: 机器人未在休息
 */
bool Robot::GetRestState() {
    return is_Robot_resting_;
}

void Robot::SetRestState(bool state) {
    is_Robot_resting_ = state;
}

///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////
//-- Robot movement: Jump
//--  Parameters:
//--    steps: Number of steps
//--    T: Period
//---------------------------------------------------------
void Robot::Calp(void) {
    if (!has_hands_) {
        return;
    }
    TODO #500 测试时修改初始值与偏差值
    int target[SERVO_COUNT] = {90, 90, 90};
    MoveServos(400, target);
    int C[SERVO_COUNT] = {90, 90, 90};
    int A[SERVO_COUNT] = {0, 45, 45};
    double phase_diff[SERVO_COUNT] = {0, 0, 0};
    Execute2(A, C, 300, phase_diff, 1);
}

void Robot::EnableServoLimit(int diff_limit) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetLimiter(diff_limit);
        }
    }
}

void Robot::DisableServoLimit() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].DisableLimiter();
        }
    }
}
