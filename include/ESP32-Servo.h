#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
// #include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "Arduino.h"
#include <cmath>




#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define SERVO_MAX_DEG        90      // maximum angle in degrees
#define SERVO_MIN_DEG        -90        // minimum angle in degrees

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000 // 20000us, 20ms period

/*
    This servo class is used to control a servo motor using the LEDC peripheral of the ESP32.
    The servo motor is controlled by a PWM signal with a frequency of 50Hz and a duty cycle of 5% to 10%.
    The servo motor is modified to have feedback using the internal potentiometer. 
    The feedback is used to calibrate maximum range of motion of the servo.
*/

/*
    TODO:
    - do a proper calibration
    - add a function to set the range of motion of the servo
    - add get_angle from feedback value (using scaler)
    - add set_angle_scaler function
*/
namespace ESP32Servo{
    class Servo
    {
        public:

        Servo();  
        Servo(bool use_feedback);  
        Servo(int min, int max);  
        Servo(int min, int max, bool use_feedback);

        ~Servo();
        
        void setName(char *name);

        void setPwmPin(int pin);

        void setFeedbackPin(int pin);
        
        void begin();
        void begin(SemaphoreHandle_t *mutex);
        void syncTimerTEZ(mcpwm_timer_handle_t timers[]);
        void get_timer(mcpwm_timer_handle_t *timer);

        void setFrequency(uint32_t frequency);

        bool calibrate();
        bool calibrate(int);
        void setCalibration(int analog_min, int analog_max, int pwm_min, int pwm_max);
        void setAngleDegreesLimit(float min, float max);
        void setGroup(int group);
        void writeMicroseconds(int usec);
        int get_pwm();
        int read();
        float readDeg();

    private:

        //MARK: MCPWM 
        // Timer
        mcpwm_timer_handle_t _timer= NULL;
        mcpwm_timer_config_t _timer_conf;
        // operator
        mcpwm_oper_handle_t _oper= NULL;
        mcpwm_operator_config_t _oper_conf;
        // comparator
        mcpwm_cmpr_handle_t _comparator= NULL;
        mcpwm_comparator_config_t _comparator_conf;
        // generator
        mcpwm_gen_handle_t _generator= NULL;
        mcpwm_generator_config_t _generator_conf;

        int _pwm_pin;
        int _pwm_val; // in microseconds
        int _feedback_pin;
        uint32_t _cur_ticks;
        // not yet used
        // int _min;
        // int _max;
        // float _angle;
        // int _prev_feedback;

        uint32_t _frequency;

        int _duty;
        int _feedback;

        int _feedback_min;
        int _feedback_max;

        int _pwm_min;
        int _pwm_max;
        float _deg_max;
        float _deg_min;
        bool _use_feedback;
        char *_name;

        double _map(double val, double in_min, double in_max, double out_min, double out_max);
        float _map(int val, int in_min, int in_max, float out_min, float out_max);
        float _map(float val, float in_min, float in_max, float out_min, float out_max);
    };
}