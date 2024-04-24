#include "ESP32-Servo.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO);
static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - 0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / (90 - 0) + MIN_PULSE_WIDTH;
}

namespace ESP32Servo{

Servo::Servo() : Servo(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, false){}

Servo::Servo(int min, int max): Servo(min, max, false){}

Servo::Servo(bool use_feedback): Servo(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, use_feedback){}

Servo::Servo(int min, int max, bool use_feedback){

    // MCPWM TIMER
    _timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    // MCPWM OPERATOR
    _oper_conf = {
        .group_id = 0, // operator must be in the same group to the timer
    };

    // MCPWM COMPARATOR
    _comparator_conf = {
        .flags.update_cmp_on_tez = true, // update the comparator value on the next timer zero event
    };

    // MCPWM GENERATOR
    _generator_conf = {
        .gen_gpio_num = _pwm_pin,
    };

    // for constraint the movement of the servo
    _feedback = -1;
    _feedback_min = -1; //min analog value set with calibration
    _feedback_max = -1; //max analog value set with calibration
    _feedback_pin = -1; 
    _use_feedback = use_feedback; 

    _pwm_min = min;
    _pwm_max = max;
    _pwm_pin = -1;
    _deg_max = 90.0;
    _deg_min = 0.0;
    // Constructor
    _name = (char*)malloc(20);
    strcpy(_name, "servo name not set");
}


Servo::~Servo(){
    // Destructor
    free(_name);
    mcpwm_del_timer(_timer);
    mcpwm_del_operator(_oper);
    mcpwm_del_comparator(_comparator);
    mcpwm_del_generator(_generator);
}

void Servo::setPwmPin(int pin){
    // Set the PWM pin
    _pwm_pin = pin;
}

void Servo::setFeedbackPin(int pin){
    // ANALOG PIN ONLY (recommend ADC1)
    // ESP32 ANALOG PIN : 25,32-36,39
    // Set the feedback pin
    if ((pin < 32 || pin > 36)&& pin != 25 && pin != 39) {
        ESP_LOGE(_name, "Invalid feedback pin: %d. Valid pins are 25,32-36,39", pin);
        return;
    }
    _feedback_pin = pin;
    pinMode(_feedback_pin, INPUT);
}

void Servo::begin(SemaphoreHandle_t *mutex){
    xSemaphoreTakeRecursive(*mutex, 10/portTICK_PERIOD_MS);
    this->begin();
    xSemaphoreGive(*mutex);
}

void Servo::begin(){
    // check if the pwm pin is set
    if (_pwm_pin < 0) {
        ESP_LOGE(_name, "PWM pin not set");
        return;
    }

    // MCPWM TIMER
    ESP_LOGD(_name, "Create timer and operator");
    ESP_ERROR_CHECK(mcpwm_new_timer(&_timer_conf, &_timer));
    // MCPWM OPERATOR
    ESP_ERROR_CHECK(mcpwm_new_operator(&_oper_conf, &_oper));
    ESP_LOGD(_name, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper, _timer));

    // MCPWM COMPARATOR
    ESP_LOGD(_name, "Create comparator and generator from the operator");
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper, &_comparator_conf, &_comparator));

    // MCPWM GENERATOR
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper, &_generator_conf, &_generator));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator, 1500));
    ESP_LOGD(_name, "set generator action on timer and compare event");

    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(_generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(_generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_LOGD(_name, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP));

    if (_use_feedback) {
        if (_feedback_pin < 0) {
            ESP_LOGE(_name, "Feedback pin not set");
            return;
        }
        if (_feedback_min < 0 || _feedback_max < 0) {
            ESP_LOGI(_name, "Feedback calibration not set, calibrating...");

            if(!this->calibrate()){
                ESP_LOGE(_name, "Calibration failed");
                return;
            }
            else{
                ESP_LOGI(_name, "Calibration successful");
            }
        }
    }
}

void Servo::syncTimerTEZ(mcpwm_timer_handle_t timers[]){
    // sync the timer with the other timers
    //          +->timer1
    // (TEZ)    |
    // timer0---+
    //          |
    //          +->timer2
    ESP_LOGD(_name, "Create TEZ sync source from timer0");
    mcpwm_sync_handle_t timer_sync_source = NULL;
    mcpwm_timer_sync_src_config_t timer_sync_config = {
        .timer_event = MCPWM_TIMER_EVENT_EMPTY, // generate sync event on timer empty
    };

    ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(_timer, &timer_sync_config, &timer_sync_source));

    ESP_LOGD(_name, "Set other timers sync to the first timer");
    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = timer_sync_source,
    };
    for (int i = 1; i < sizeof(timers)/sizeof(mcpwm_timer_handle_t); i++) {
        ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timers[i], &sync_phase_config));
    }

    ESP_LOGD(_name, "Wait some time for the timer TEZ event");
    vTaskDelay(pdMS_TO_TICKS(10));
}


void Servo::setName(char *name){
    strcpy(_name, name);
}

void Servo::setGroup(int group){
    // group must be set before begin
    _timer_conf.group_id = group;
    _oper_conf.group_id = group;
}



bool Servo::calibrate(){
    return Servo::calibrate(10000);
}

bool Servo::calibrate(int ms){
    // ms -> time calibration in miliseconds
    if (_feedback_pin < 0) {
            ESP_LOGE(_name, "Feedback pin not set");
            return false;
    }
        // to calibrate the range of motion of the servo
    
    int min = 4096;
    int max = 0;
    ESP_LOGI(_name, "Calibration is running for %d seconds", ms/1000);
    ESP_LOGI(_name, "Please move/turn the servo");
    TickType_t tick_start = xTaskGetTickCount();
    ESP_LOGD(_name, "Calibration started at %d", (int)tick_start);
    while (xTaskGetTickCount() - tick_start < ms / portTICK_PERIOD_MS){
        _feedback = analogRead(_feedback_pin);
        if (_feedback < min) {
            min = _feedback;
        }
        if (_feedback > max) {
            max = _feedback;
        }
        
        ESP_LOGD(_name, "Feedback: %d, Min: %d, Max: %d", _feedback, min, max);
        ESP_LOGV(_name, "Calibration is running for %d seconds", (int)(xTaskGetTickCount() - tick_start) / 1000);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(_name, "Reading complete");
    ESP_LOGI(_name, "Min: %d, Max: %d", min, max);
    ESP_LOGW(_name, "MOVING SERVO");
    this->begin();
    int pwm_min = 2500;
    int pwm_max = 500;
    int cur_pwm = 1500;
    int8_t flag = 1;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    for (;;)
    {
        _feedback = analogRead(_feedback_pin);
        if(_feedback >= max-10){
            pwm_max = cur_pwm;
            flag = -1;
        }
        if(_feedback<=min+10){
            pwm_min = cur_pwm;
            break;
        }
        if(cur_pwm>=_pwm_max || cur_pwm<=_pwm_min){
            if(cur_pwm>=_pwm_max)
                max = _feedback;
            else
                min = _feedback;
            ESP_LOGE(_name, "Servo reached the end of its range of motion");
        }
        ESP_LOGD(_name, "Feedback: %d, Min: %d, Max: %d, PWM: %d", _feedback, min, max, cur_pwm);
        this->writeMicroseconds(cur_pwm);
        cur_pwm += flag;
        vTaskDelay(50/portTICK_PERIOD_MS); //20Hz
    }

    ESP_LOGI(_name, "Calibration completed");
    ESP_LOGI(_name, "Min analog: %d, Max analog: %d", min, max);
    ESP_LOGI(_name, "Min PWM: %d Max PWM: %d", pwm_min, pwm_max);
    this->setCalibration(min, max,pwm_min,pwm_max);
    return true;
}

int Servo::read(){
    // read the feedback pin
    if (_feedback_pin < 0) {
        ESP_LOGE(_name, "Feedback pin not set");
        return 0;
    }
    _feedback= analogRead(_feedback_pin);
    
    return _feedback;
}

float Servo::readDeg(){
    // read the feedback pin and convert to degrees
    return _map(read(), _feedback_min, _feedback_max, _deg_min, _deg_max);
}

void Servo::setCalibration(int analog_min, int analog_max, int pwm_min, int pwm_max){
    // set the calibration range
    _feedback_min = analog_min;
    _feedback_max = analog_max;
    _pwm_max = pwm_max;
    _pwm_min = pwm_min;
}

void Servo::setAngleDegreesLimit(float min, float max){
    _deg_min = min;
    _deg_max = max;
}

double Servo::_map(double val, double in_min, double in_max, double out_min, double out_max){
    // map the value from one range to another
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Servo::_map(int val, int in_min, int in_max, float out_min, float out_max){
    // map the value from one range to another
    return _map((float)val, (float)in_min, (float)in_max, (float)out_min, (float)out_max);
}

float Servo::_map(float val, float in_min, float in_max, float out_min, float out_max){
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Servo::writeMicroseconds(int usec){
    
    // write the microseconds to the servo
    if (usec < _pwm_min) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be greater than %d", usec, _pwm_min);
        return;
    }
    if (usec > _pwm_max) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be less than %d", usec, _pwm_max);
        return;
    }
    _pwm_val = usec;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_comparator, usec));
}

int Servo::get_pwm(){
    /*
    return the current pwm value in microseconds (int)
    */
    return _pwm_val;
}

void Servo::get_timer(mcpwm_timer_handle_t *timer){
    *timer = _timer;
    return;
}
