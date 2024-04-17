#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "Arduino.h"
#include <cmath>



#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
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

        void setFrequency(uint32_t frequency);

        void setTimer(ledc_timer_t timer);
        void setTimer(int timer);

        void setResolution(ledc_timer_bit_t resolution);
        void setResolution(int resolution);

        void setChannel(ledc_channel_t channel);
        void setChannel(int channel);

        bool calibrate();
        void setCalibration(int min, int max);

        void writeTicks(int ticks);
        void writeMicroseconds(int usec);

        int read();
        float readDeg();


        int usToTicks(int usec);
        int ticksToUs(int ticks);

    private:
        ledc_channel_t _channel;
        int _pwm_pin;
        int _feedback_pin;
        int _min;
        int _max;
        ledc_timer_bit_t _resolution;
        ledc_timer_t _timer;
        ledc_mode_t _mode;
        uint32_t _frequency;
        int _angle;
        int _duty;
        int _feedback;
        // int _angle_min;
        // int _angle_max;
        int _feedback_min;
        int _feedback_max;
        float _deg_max;
        float _deg_min;
        bool _use_feedback;
        char *_name;

        double _map(double val, double in_min, double in_max, double out_min, double out_max);
        double _map(int val, int in_min, int in_max, float out_min, float out_max);
    };
}