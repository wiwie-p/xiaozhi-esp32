#ifndef __Robot_MOVEMENTS_H__
#define __Robot_MOVEMENTS_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oscillator.h"

//-- Constants
#define FORWARD 1
#define BACKWARD -1
#define LEFT 1
#define RIGHT -1
#define BOTH 0
#define SMALL 5
#define MEDIUM 15
#define BIG 30

// -- Servo delta limit default. degree / sec
#define SERVO_LIMIT_DEFAULT 240

// -- Servo indexes for easy access
#define HEAD    0
#define LH      1
#define RH      2

#define SERVO_COUNT 3

class Robot {
public:
    Robot();
    ~Robot();

    //-- Robot initialization
    void Init(int head=-1, int right_hand=-1, int left_hand=-1);
    //-- Attach & detach functions
    void AttachServos();
    void DetachServos();

    //-- Oscillator Trims
    void SetTrims(int head=0, int right_hand=0, int left_hand=0);

    //-- Predetermined Motion Functions
    void MoveServos(int time, int servo_target[]);
    void MoveSingle(int position, int servo_number);
    void OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                         double phase_diff[SERVO_COUNT], float cycle);
    void Execute2(int amplitude[SERVO_COUNT], int center_angle[SERVO_COUNT], int period,
                  double phase_diff[SERVO_COUNT], float steps);

    //-- HOME = Robot at rest position
    void Home(bool hands_down = true);
    bool GetRestState();
    void SetRestState(bool state);

    void Calp(void);      // 拍手

    // -- Servo limiter
    void EnableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void DisableServoLimit();

private:
    Oscillator servo_[SERVO_COUNT];

    int servo_pins_[SERVO_COUNT];
    int servo_trim_[SERVO_COUNT];

    unsigned long final_time_;
    unsigned long partial_time_;
    float increment_[SERVO_COUNT];

    bool is_Robot_resting_;
    bool has_hands_;  // 是否有手部舵机

    void Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                 double phase_diff[SERVO_COUNT], float steps);

};

#endif  // __Robot_MOVEMENTS_H__