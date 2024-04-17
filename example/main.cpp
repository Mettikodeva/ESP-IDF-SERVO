#include "Arduino.h"
#include "ESP32-Servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


using namespace ESP32Servo;

void sweepTask(void *pvParameters){
    Servo *servo = (Servo*)pvParameters;
    int counter = 0;
    for(;;) {
        Serial.print("Counter: ");
        Serial.println(counter++);
        for(int i = 1000; i <= 2000; i++){
            Serial.println(i);
            servo->writeMicroseconds(i);
            Serial.println(servo->read());
            delay(10);
        }
        for(int i = 2000; i >= 100; i--){
            Serial.println(i);
            servo->writeMicroseconds(i);
            Serial.println(servo->read());
            delay(10);
        }
        
    }
}

void knobTask(void *pvParameters){
    Servo *servo = (Servo*)pvParameters;
    pinMode(35, INPUT);
    for(;;){
        int val = analogRead(35);
        Serial.println(val);
        Serial.println(map(val, 0, 4095, 1000, 2000));
        servo->writeMicroseconds((int)map(val, 0, 4095, 1000, 2000));
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main()
{
    initArduino();
    Servo *myservo = new Servo(true);
    myservo->setPwmPin(14);
    myservo->setChannel(0);
    myservo->setTimer(0);
    myservo->setFeedbackPin(35);
    myservo->setResolution(16);
    myservo->setName("Servo 1");
    myservo->calibrate();
    myservo->begin();
    Serial.begin(115200);
    xTaskCreate(sweepTask, "sweepTask", 2048, myservo, 5, NULL);

    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // xTaskCreate(knobTask, "knobTask", 2048, myservo, 5, NULL);
}
