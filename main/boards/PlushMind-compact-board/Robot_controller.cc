/*
    Robot机器人控制器 - MCP协议版本
*/

#include <cJSON.h>
#include <esp_log.h>

#include <cstdlib> 
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "Robot_movements.h"
#include "power_manager.h"
#include "sdkconfig.h"
#include "settings.h"
#include <wifi_manager.h>

#define TAG "RobotController"

class RobotController {
private:
    Robot Robot_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool has_hands_ = false;
    bool is_action_in_progress_ = false;

    struct RobotActionParams {
        int action_type;
        int steps;
        int speed;
        int direction;
        int amount;
        char servo_sequence_json[512];  // 用于存储舵机序列的JSON字符串
    };

    enum ActionType {
        ACTION_CLAP = 1            // 拍手
    };

    /**
     * @brief 机器人动作任务函数
     * @param arg 指向RobotController对象的指针，用于控制机器人执行各种动作
     * @return 无返回值
     */
    static void ActionTask(void* arg) {
        RobotController* controller = static_cast<RobotController*>(arg);
        RobotActionParams params;
        controller->Robot_.AttachServos();
    
        // 循环处理动作队列中的任务
        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                // HACK #1 没有电池
                // PowerManager::PauseBatteryUpdate();  // 动作开始时暂停电量更新
                controller->is_action_in_progress_ = true;
                switch (params.action_type) {
                    case ACTION_WALK: 
                        controller->Robot_.Calp();
                        break;
                    default:
                        break;
                }
                controller->is_action_in_progress_ = false;
                // HACK #1 没有电池
                // PowerManager::ResumeBatteryUpdate();  // 动作结束时恢复电量更新
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    /**
     * @brief 如果动作任务尚未创建，则启动一个新的动作任务
     * 
     * 该函数检查当前是否已存在动作任务句柄，如果不存在则创建一个新的FreeRTOS任务
     * 用于处理机器人的动作执行，该任务具有最高优先级以确保及时响应
     * 
     * @return 无返回值
     */
    void StartActionTaskIfNeeded() {
        // 检查是否需要创建新的动作任务
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "Robot_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    /**
     * @brief 队列化执行机器人动作
     * 
     * 将指定的动作参数加入队列，由动作任务异步执行。该函数会检查手部动作的硬件配置要求，
     * 并将动作参数发送到动作队列中。
     * 
     * @param action_type 动作类型，包括手部动作、风车动作、起飞动作等
     * @param steps 动作执行的步数
     * @param speed 动作执行的速度
     * @param direction 动作执行的方向
     * @param amount 动作执行的幅度
     * @return 无返回值
     */
    void QueueAction(int action_type, int steps, int speed, int direction, int amount) {
        // 检查手部动作是否需要手部舵机支持
        if (action_type == ACTION_CLAP) {
            if (!has_hands_) {
                ESP_LOGW(TAG, "尝试执行手部动作，但机器人没有配置手部舵机");
                return;
            }
        }
    
        ESP_LOGI(TAG, "动作控制: 类型=%d, 步数=%d, 速度=%d, 方向=%d, 幅度=%d", action_type, steps,
                 speed, direction, amount);
    
        RobotActionParams params = {action_type, steps, speed, direction, amount, ""};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    /**
     * @brief 将舵机序列JSON添加到执行队列中
     * 
     * 该函数接收一个包含舵机序列配置的JSON字符串，验证其有效性后，
     * 将其封装到参数结构体中并发送到动作执行队列，然后启动动作任务。
     * 
     * @param servo_sequence_json 包含舵机序列配置的JSON字符串，格式应为有效的JSON
     * @return 无返回值，但会通过日志输出操作结果和错误信息
     */
    void QueueServoSequence(const char* servo_sequence_json) {
        if (servo_sequence_json == nullptr) {
            ESP_LOGE(TAG, "序列JSON为空");
            return;
        }
        
        int input_len = strlen(servo_sequence_json);
        const int buffer_size = 512;  // servo_sequence_json数组大小
        ESP_LOGI(TAG, "队列舵机序列，输入长度=%d，缓冲区大小=%d", input_len, buffer_size);
        
        if (input_len >= buffer_size) {
            ESP_LOGE(TAG, "JSON字符串太长！输入长度=%d，最大允许=%d", input_len, buffer_size - 1);
            return;
        }
        
        if (input_len == 0) {
            ESP_LOGW(TAG, "序列JSON为空字符串");
            return;
        }
        
        RobotActionParams params = {ACTION_SERVO_SEQUENCE, 0, 0, 0, 0, ""};
        // 复制JSON字符串到结构体中（限制长度）
        strncpy(params.servo_sequence_json, servo_sequence_json, sizeof(params.servo_sequence_json) - 1);
        params.servo_sequence_json[sizeof(params.servo_sequence_json) - 1] = '\0';
        
        ESP_LOGD(TAG, "序列已加入队列: %s", params.servo_sequence_json);
        
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    /**
     * 从NVS（非易失性存储）加载机器人的微调设置
     * 该函数读取存储在NVS中的各个关节的微调值，并应用到机器人上
     * 
     * 参数: 无
     * 返回值: void
     */
    void LoadTrimsFromNVS() {
        Settings settings("Robot_trims", false);
    
        int head = settings.GetInt("head", 0);
        int right_hand = settings.GetInt("right_hand", 0);
        int left_hand = settings.GetInt("left_hand", 0);
    
        // 记录从NVS加载的微调设置信息
        ESP_LOGI(TAG, "从NVS加载微调设置: 头=%d, 右手=%d, 左手=%d",head, right_hand, left_hand);
    
        Robot_.SetTrims(head, right_hand, left_hand);
    }

public:
    RobotController(const HardwareConfig& hw_config) {
        Robot_.Init(
            HEAD_GPIO_NUM, 
            RH_GPIO_NUM, 
            LH_GPIO_NUM
        );

        has_hands_ = (hw_config.left_hand_pin != GPIO_NUM_NC && hw_config.right_hand_pin != GPIO_NUM_NC);
        ESP_LOGI(TAG, "Robot机器人初始化%s手部舵机", has_hands_ ? "带" : "不带");
        ESP_LOGI(TAG, "舵机引脚配置: Head=%d, rightHand=%d, leftHand=%d",
                 hw_config.left_leg_pin, hw_config.right_leg_pin,hw_config.left_foot_pin);

        LoadTrimsFromNVS();

        action_queue_ = xQueueCreate(10, sizeof(RobotActionParams));

        QueueAction(ACTION_HOME, 1, 1000, 1, 0);  // direction=1表示复位手部

        RegisterMcpTools();
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        ESP_LOGI(TAG, "开始注册MCP工具...");

        mcp_server.AddTool("self.Robot.calp","拍手",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                            QueueAction(ACTION_CALP, 1, 1000, 1, 0);
                            }
        /*
        TODO :#2 MCP添加其他动作函数
        // 统一动作工具（除了舵机序列外的所有动作）
        mcp_server.AddTool("self.Robot.action",
                           "执行机器人动作。action: 动作名称；根据动作类型提供相应参数：direction: 方向，1=前进/左转，-1=后退/右转；0=左右同时"
                           "steps: 动作步数，1-100；speed: 动作速度，100-3000，数值越小越快；amount: 动作幅度，0-170；arm_swing: 手臂摆动幅度，0-170；"
                           "基础动作：walk(行走，需steps/speed/direction/arm_swing)、turn(转身，需steps/speed/direction/arm_swing)、jump(跳跃，需steps/speed)、"
                           "swing(摇摆，需steps/speed/amount)、moonwalk(太空步，需steps/speed/direction/amount)、bend(弯曲，需steps/speed/direction)、"
                           "shake_leg(摇腿，需steps/speed/direction)、updown(上下运动，需steps/speed/amount)、whirlwind_leg(旋风腿，需steps/speed/amount)；"
                           "固定动作：sit(坐下)、showcase(展示动作)、home(复位)；"
                           "手部动作(需手部舵机)：hands_up(举手，需speed/direction)、hands_down(放手，需speed/direction)、hand_wave(挥手，需direction)、"
                           "windmill(大风车，需steps/speed/amount)、takeoff(起飞，需steps/speed/amount)、fitness(健身，需steps/speed/amount)、"
                           "greeting(打招呼，需direction/steps)、shy(害羞，需direction/steps)、radio_calisthenics(广播体操)、magic_circle(爱的魔力转圈圈)",
                           PropertyList({
                               Property("action", kPropertyTypeString, "sit"),
                               Property("steps", kPropertyTypeInteger, 3, 1, 100),
                               Property("speed", kPropertyTypeInteger, 700, 100, 3000),
                               Property("direction", kPropertyTypeInteger, 1, -1, 1),
                               Property("amount", kPropertyTypeInteger, 30, 0, 170),
                               Property("arm_swing", kPropertyTypeInteger, 50, 0, 170)
                           }),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string action = properties["action"].value<std::string>();
                               // 所有参数都有默认值，直接访问即可
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int direction = properties["direction"].value<int>();
                               int amount = properties["amount"].value<int>();
                               int arm_swing = properties["arm_swing"].value<int>();

                               // 基础移动动作
                               if (action == "walk") {
                                   QueueAction(ACTION_WALK, steps, speed, direction, arm_swing);
                                   return true;
                               } else if (action == "turn") {
                                   QueueAction(ACTION_TURN, steps, speed, direction, arm_swing);
                                   return true;
                               } else if (action == "jump") {
                                   QueueAction(ACTION_JUMP, steps, speed, 0, 0);
                                   return true;
                               } else if (action == "swing") {
                                   QueueAction(ACTION_SWING, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "moonwalk") {
                                   QueueAction(ACTION_MOONWALK, steps, speed, direction, amount);
                                   return true;
                               } else if (action == "bend") {
                                   QueueAction(ACTION_BEND, steps, speed, direction, 0);
                                   return true;
                               } else if (action == "shake_leg") {
                                   QueueAction(ACTION_SHAKE_LEG, steps, speed, direction, 0);
                                   return true;
                               } else if (action == "updown") {
                                   QueueAction(ACTION_UPDOWN, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "whirlwind_leg") {
                                   QueueAction(ACTION_WHIRLWIND_LEG, steps, speed, 0, amount);
                                   return true;
                               }
                               // 固定动作
                               else if (action == "sit") {
                                   QueueAction(ACTION_SIT, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "showcase") {
                                   QueueAction(ACTION_SHOWCASE, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "home") {
                                   QueueAction(ACTION_HOME, 1, 1000, 1, 0);
                                   return true;
                               }
                               // 手部动作
                               else if (action == "hands_up") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_HANDS_UP, 1, speed, direction, 0);
                                   return true;
                               } else if (action == "hands_down") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_HANDS_DOWN, 1, speed, direction, 0);
                                   return true;
                               } else if (action == "hand_wave") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_HAND_WAVE, 1, 0, 0, direction);
                                   return true;
                               } else if (action == "windmill") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_WINDMILL, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "takeoff") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_TAKEOFF, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "fitness") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_FITNESS, steps, speed, 0, amount);
                                   return true;
                               } else if (action == "greeting") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_GREETING, steps, 0, direction, 0);
                                   return true;
                               } else if (action == "shy") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_SHY, steps, 0, direction, 0);
                                   return true;
                               } else if (action == "radio_calisthenics") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_RADIO_CALISTHENICS, 1, 0, 0, 0);
                                   return true;
                               } else if (action == "magic_circle") {
                                   if (!has_hands_) {
                                       return "错误：此动作需要手部舵机支持";
                                   }
                                   QueueAction(ACTION_MAGIC_CIRCLE, 1, 0, 0, 0);
                                   return true;
                               } else {
                                   return "错误：无效的动作名称。可用动作：walk, turn, jump, swing, moonwalk, bend, shake_leg, updown, whirlwind_leg, sit, showcase, home, hands_up, hands_down, hand_wave, windmill, takeoff, fitness, greeting, shy, radio_calisthenics, magic_circle";
                               }
                           });
        */

        /*
        // 舵机序列工具（支持分段发送，每次发送一个序列，自动排队执行）
        mcp_server.AddTool(
            "self.Robot.servo_sequences",
            "AI自定义动作编程（即兴动作）。支持分段发送序列：超过5个序列建议AI可以连续多次调用此工具，每次发送一个短序列，系统会自动排队按顺序执行。支持普通移动和振荡器两种模式。"
            "机器人结构：双手可上下摆动，双腿可内收外展，双脚可上下翻转。"
            "舵机说明："
            "ll(左腿)：内收外展，0度=完全外展，90度=中立，180度=完全内收；"
            "rl(右腿)：内收外展，0度=完全内收，90度=中立，180度=完全外展；"
            "lf(左脚)：上下翻转，0度=完全向上，90度=水平，180度=完全向下；"
            "rf(右脚)：上下翻转，0度=完全向下，90度=水平，180度=完全向上；"
            "lh(左手)：上下摆动，0度=完全向下，90度=水平，180度=完全向上；"
            "rh(右手)：上下摆动，0度=完全向上，90度=水平，180度=完全向下；"
            "sequence: 单个序列对象，包含'a'动作数组，顶层可选参数："
            "'d'(序列执行完成后延迟毫秒数，用于序列之间的停顿)。"
            "每个动作对象包含："
            "普通模式：'s'舵机位置对象(键名：ll/rl/lf/rf/lh/rh，值：0-180度)，'v'移动速度100-3000毫秒(默认1000)，'d'动作后延迟毫秒数(默认0)；"
            "振荡模式：'osc'振荡器对象，包含'a'振幅对象(各舵机振幅10-90度，默认20度)，'o'中心角度对象(各舵机振荡中心绝对角度0-180度，默认90度)，'ph'相位差对象(各舵机相位差，度，0-360度，默认0度)，'p'周期100-3000毫秒(默认500)，'c'周期数0.1-20.0(默认5.0)；"
            "使用方式：AI可以连续多次调用此工具，每次发送一个序列，系统会自动排队按顺序执行。"
            "重要说明：左右腿脚震荡的时候，有一只脚必须在90度，否则会损坏机器人，如果发送多个序列（序列数>1），完成所有序列后需要复位时，AI应该最后单独调用self.Robot.home工具进行复位，不要在序列中设置复位参数。"
            "普通模式示例：发送3个序列，最后调用复位："
            "第1次调用{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":100},\\\"v\\\":1000}],\\\"d\\\":500}\"}，"
            "第2次调用{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":90},\\\"v\\\":800}],\\\"d\\\":500}\"}，"
            "第3次调用{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":80},\\\"v\\\":800}]}\"}，"
            "最后调用self.Robot.home工具进行复位。"
            "振荡器模式示例："
            "示例1-双臂同步摆动：{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"lh\\\":30,\\\"rh\\\":30},\\\"o\\\":{\\\"lh\\\":90,\\\"rh\\\":-90},\\\"p\\\":500,\\\"c\\\":5.0}}],\\\"d\\\":0}\"}；"
            "示例2-双腿交替振荡（波浪效果）：{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":20,\\\"rl\\\":20},\\\"o\\\":{\\\"ll\\\":90,\\\"rl\\\":-90},\\\"ph\\\":{\\\"rl\\\":180},\\\"p\\\":600,\\\"c\\\":3.0}}],\\\"d\\\":0}\"}；"
            "示例3-单腿振荡配合固定脚（安全）：{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":45},\\\"o\\\":{\\\"ll\\\":90,\\\"lf\\\":90},\\\"p\\\":400,\\\"c\\\":4.0}}],\\\"d\\\":0}\"}；"
            "示例4-复杂多舵机振荡（手和腿）：{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"lh\\\":25,\\\"rh\\\":25,\\\"ll\\\":15},\\\"o\\\":{\\\"lh\\\":90,\\\"rh\\\":90,\\\"ll\\\":90,\\\"lf\\\":90},\\\"ph\\\":{\\\"rh\\\":180},\\\"p\\\":800,\\\"c\\\":6.0}}],\\\"d\\\":500}\"}；"
            "示例5-快速摇摆：{\"sequence\":\"{\\\"a\\\":[{\\\"osc\\\":{\\\"a\\\":{\\\"ll\\\":30,\\\"rl\\\":30},\\\"o\\\":{\\\"ll\\\":90,\\\"rl\\\":90},\\\"ph\\\":{\\\"rl\\\":180},\\\"p\\\":300,\\\"c\\\":10.0}}],\\\"d\\\":0}\"}。",
            PropertyList({Property("sequence", kPropertyTypeString,
                                   "{\"a\":[{\"s\":{\"ll\":90,\"rl\":90},\"v\":1000}]}")}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string sequence = properties["sequence"].value<std::string>();
                // 检查是否是JSON对象（可能是字符串格式或已解析的对象）
                // 如果sequence是JSON字符串，直接使用；如果是对象字符串，也需要使用
                QueueServoSequence(sequence.c_str());
                return true;
            });
            */

        mcp_server.AddTool("self.Robot.stop", "立即停止所有动作并复位", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (action_task_handle_ != nullptr) {
                                   vTaskDelete(action_task_handle_);
                                   action_task_handle_ = nullptr;
                               }
                               is_action_in_progress_ = false;
                               PowerManager::ResumeBatteryUpdate();  // 停止动作时恢复电量更新
                               xQueueReset(action_queue_);

                               QueueAction(ACTION_HOME, 1, 1000, 1, 0);
                               return true;
                           });
                           
        mcp_server.AddTool(
            "self.Robot.set_trim",
            "校准单个舵机位置。设置指定舵机的微调参数以调整机器人的初始站立姿态，设置将永久保存。"
            "servo_type: 舵机类型(head/right_hand/left_hand); "
            "trim_value: 微调值(-50到50度)",
            PropertyList({Property("servo_type", kPropertyTypeString, "head"),
                          Property("trim_value", kPropertyTypeInteger, 0, -50, 50)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();

                ESP_LOGI(TAG, "设置舵机微调: %s = %d度", servo_type.c_str(), trim_value);

                // 获取当前所有微调值
                Settings settings("Robot_trims", true);
                int head = settings.GetInt("head", 0);
                int right_hand = settings.GetInt("right_hand", 0);
                int left_hand = settings.GetInt("left_hand", 0);

                // 更新指定舵机的微调值
                if (servo_type == "head") {
                    head = trim_value;
                    settings.SetInt("head", head);
                } else if (servo_type == "right_hand") {
                    right_hand = trim_value;
                    settings.SetInt("right_hand", right_hand);
                } else if (servo_type == "left_hand") {
                    left_hand = trim_value;
                    settings.SetInt("left_hand", left_hand);
                } else {
                    return "错误：无效的舵机类型，请使用: head, right_hand, left_hand";
                }

                Robot_.SetTrims(head, right_hand, left_hand, right_foot, left_hand, right_hand);

                QueueAction(ACTION_CLAP, 1, 500, 0, 0);

                return "舵机 " + servo_type + " 微调设置为 " + std::to_string(trim_value) +
                       " 度，已永久保存";
            });

        mcp_server.AddTool("self.Robot.get_trims", "获取当前的舵机微调设置", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("Robot_trims", false);

                               int head = settings.GetInt("head", 0);
                               int right_hand = settings.GetInt("right_hand", 0);
                               int left_hand = settings.GetInt("left_hand", 0);

                               std::string result =
                                   "{\"head\":" + std::to_string(head) +
                                   ",\"right_hand\":" + std::to_string(right_hand) +
                                   ",\"left_hand\":" + std::to_string(left_hand) +

                               ESP_LOGI(TAG, "获取微调设置: %s", result.c_str());
                               return result;
                           });

        mcp_server.AddTool("self.Robot.get_status", "获取机器人状态，返回 moving 或 idle",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        mcp_server.AddTool("self.battery.get_level", "获取机器人电池电量和充电状态", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& board = Board::GetInstance();
                               int level = 0;
                               bool charging = false;
                               bool discharging = false;
                               board.GetBatteryLevel(level, charging, discharging);

                               std::string status =
                                   "{\"level\":" + std::to_string(level) +
                                   ",\"charging\":" + (charging ? "true" : "false") + "}";
                               return status;
                           });
                           
        mcp_server.AddTool("self.Robot.get_ip", "获取机器人WiFi IP地址", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& wifi = WifiManager::GetInstance();
                               std::string ip = wifi.GetIpAddress();
                               if (ip.empty()) {
                                   return "{\"ip\":\"\",\"connected\":false}";
                               }
                               std::string status = "{\"ip\":\"" + ip + "\",\"connected\":true}";
                               return status;
                           });                           

        ESP_LOGI(TAG, "MCP工具注册完成");
    }

    ~RobotController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static RobotController* g_Robot_controller = nullptr;

void InitializeRobotController(const HardwareConfig& hw_config) {
    if (g_Robot_controller == nullptr) {
        g_Robot_controller = new RobotController(hw_config);
        ESP_LOGI(TAG, "Robot控制器已初始化并注册MCP工具");
    }
}
