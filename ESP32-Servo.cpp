#include "ESP32-Servo.h"

namespace ESP32Servo{

Servo::Servo() : Servo(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, false){}

Servo::Servo(int min, int max): Servo(min, max, false){}

Servo::Servo(bool use_feedback): Servo(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, use_feedback){}

Servo::Servo(int min, int max, bool use_feedback){
    _timer = LEDC_TIMER_MAX; // MAX for unset
    _resolution = LEDC_TIMER_16_BIT;
    _channel = LEDC_CHANNEL_MAX; // MAX for unset
    _mode = LEDC_LOW_SPEED_MODE;
    _frequency = 50;
    _angle = 0;
    _duty = -1;

    _feedback = -1;
    _feedback_min = -1;
    _feedback_max = -1;
    _feedback_pin = -1;
    _use_feedback = use_feedback; 

    _min = min;
    _max = max;
    _pwm_pin = -1;
    _deg_max = 90.0;
    _deg_min = 0.0;
    // Constructor
    _name = (char*)malloc(20);
    strcpy(_name, "servo name not set");
}

Servo::~Servo(){
    // Destructor
}


void Servo::setPwmPin(int pin){
    // Set the PWM pin
    _pwm_pin = pin;
}

void Servo::setTimer(ledc_timer_t timer){
    // Set the timer
    if (timer < 0 || timer > LEDC_TIMER_MAX - 1) {
        ESP_LOGE(_name, "Invalid timer: %d. Valid timers are 0-%d", (int)timer, (int)(LEDC_TIMER_MAX - 1));
        return;
    }
    _timer = timer;
}

void Servo::setTimer(int timer){
    this->setTimer((ledc_timer_t)timer);
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

void Servo::setFrequency(uint32_t frequency){
    // Set the frequency
    if(frequency != 50){
        ESP_LOGW(_name, "The frequency is set to: %d. Common Hobby servo frequency is 50Hz.", (int)frequency);
    }
    _frequency = frequency;
}

void Servo::setChannel(ledc_channel_t channel){
    if (channel < 0 || channel > LEDC_CHANNEL_MAX - 1) {
        ESP_LOGE(_name, "Invalid channel: %d. Valid channels are 0-%d", (int)channel, (int)(LEDC_CHANNEL_MAX-1));
        return;
    }
    _channel = channel;
}

void Servo::setChannel(int channel){
    this->setChannel((ledc_channel_t)channel);
}

void Servo::setResolution(ledc_timer_bit_t resolution){
    if (resolution < 0 || resolution > LEDC_TIMER_BIT_MAX - 1) {
        ESP_LOGE(_name, "Invalid resolution: %d. Valid resolutions are 0-%d", (int)resolution, (int)(LEDC_TIMER_BIT_MAX-1));
        return;
    }

    _resolution = resolution;
}

void Servo::setResolution(int resolution){
    this->setResolution((ledc_timer_bit_t)resolution);
}


void Servo::begin(){

    // check if the pwm pin is set
    if (_pwm_pin < 0) {
        ESP_LOGE(_name, "PWM pin not set");
        return;
    }
    // check timer is set
    if (_timer == LEDC_TIMER_MAX) {
        ESP_LOGE(_name, "Timer not set");
        return;
    }
    // check resolution is set
    if (_resolution == LEDC_TIMER_BIT_MAX) {
        ESP_LOGE(_name, "Resolution not set");
        return;
    }

    // initialize timer
    ledc_timer_config_t timer_conf{
        .speed_mode = _mode,
        .duty_resolution = _resolution,
        .timer_num = _timer,
        .freq_hz = _frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t res = ledc_timer_config(&timer_conf);
    if (res != ESP_OK) {
        ESP_LOGE(_name, "Error configuring timer: %s", esp_err_to_name(res));
    }
    else{
        ESP_LOGV(_name, "Timer configured successfully");
    }

    ledc_channel_config_t ledc_conf{
        .gpio_num = _pwm_pin,
        .speed_mode = _mode,
        .channel = _channel,
        .intr_type = LEDC_INTR_FADE_END, // need to change this
        .timer_sel = _timer,
        .duty = 0,
        .hpoint = 0,
        .flags = 0
    };

    res = ledc_channel_config(&ledc_conf);

    if (res != ESP_OK) {
        ESP_LOGE(_name, "Error configuring channel: %s", esp_err_to_name(res));
    }
    else{
        ESP_LOGV(_name, "Channel configured successfully");
    }

    delay(500);
    ledc_set_duty(_mode, _channel, 0);
    ledc_update_duty(_mode, _channel);

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

void Servo::setName(char *name){
    strcpy(_name, name);
}

int Servo::usToTicks(int usec){
    /*
    value = (pulse_width     /(  1   /  pwm_frequency    * 2**timer_resolution))
    */
    return (int)((double)usec / (((double)1000000/(double)_frequency) / (double)pow(2, (double)_resolution) ));
}

int Servo::ticksToUs(int ticks){
    return (int)((double)ticks * (((double)1000000/(double)_frequency) / (double)pow(2,(double)_resolution)));

}

bool Servo::calibrate(){
    if (_feedback_pin < 0) {
            ESP_LOGE(_name, "Feedback pin not set");
            return false;
    }
        // to calibrate the range of motion of the servo
    int min = 4096;
    int max = 0;
    ESP_LOGI(_name, "Calibration is running for 15 seconds");
    TickType_t tick_start = xTaskGetTickCount();
    ESP_LOGV(_name, "Calibration started at %d", (int)tick_start);
    while (xTaskGetTickCount() - tick_start < 15000 / portTICK_PERIOD_MS){
        _feedback = analogRead(_feedback_pin);
        if (_feedback < min) {
            min = _feedback;
        }
        if (_feedback > max) {
            max = _feedback;
        }
        ESP_LOGV(_name, "Feedback: %d, Min: %d, Max: %d", _feedback, min, max);
        ESP_LOGV(_name, "Calibration is running for %d seconds", (int)(xTaskGetTickCount() - tick_start) / 1000);
    }
    ESP_LOGI(_name, "Calibration completed");
    ESP_LOGI(_name, "Min: %d, Max: %d", min, max);
    this->setCalibration(min,max);
    return true;
}

int Servo::read(){
    // read the feedback pin
    if (_feedback_pin < 0) {
        ESP_LOGE(_name, "Feedback pin not set");
        return 0;
    }
    analogRead(_feedback_pin);
    
    return _feedback;
}

float Servo::readDeg(){
    // read the feedback pin and convert to degrees
    if (_feedback_pin < 0) {
        ESP_LOGE(_name, "Feedback pin not set");
        return 0;
    }
    analogRead(_feedback_pin);
    return _map(_feedback, _feedback_min, _feedback_max, _deg_min, _deg_max);
}

void Servo::setCalibration(int min, int max){
    // set the calibration range
    // _angle_min minimum value of the analog feedback
    // _angle_max maximum value of the analog feedback
    _feedback_min = min;
    _feedback_max = max;
}

double Servo::_map(double val, double in_min, double in_max, double out_min, double out_max){
    // map the value from one range to another
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double Servo::_map(int val, int in_min, int in_max, float out_min, float out_max){
    // map the value from one range to another
    return _map((double)val, (double)in_min, (double)in_max, (double)out_min, (double)out_max);
}

void Servo::writeTicks(int ticks){
    // write the ticks to the servo
    if (ticks < 0) {
        ESP_LOGE(_name, "Invalid ticks: %d. Ticks must be greater than 0", ticks);
        return;
    }
    if (ticks > (int)pow(2, (int)_resolution)) {
        ESP_LOGE(_name, "Invalid ticks: %d. Ticks must be less than %d", ticks, (int)pow(2,(int)_resolution));
        return;
    }
    ledc_set_duty(_mode, _channel, ticks);
    ledc_update_duty(_mode, _channel);
}

void Servo::writeMicroseconds(int usec){
    // write the microseconds to the servo
    if (usec < _min) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be greater than %d", usec, _min);
        return;
    }
    if (usec > _max) {
        ESP_LOGE(_name, "Invalid microseconds: %d. Microseconds must be less than %d", usec, _max);
        return;
    }
    this->writeTicks(this->usToTicks(usec));
}
}