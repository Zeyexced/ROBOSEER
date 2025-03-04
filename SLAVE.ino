// ESP32U DEV BOARD 38P - SLAVE (Robot)
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Stepper.h>

// Motor driver pins for BTS7960
#define M1_L_PWM 14
#define M1_R_PWM 27
#define M1_EN_L  26
#define M1_EN_R  25

#define M2_L_PWM 32
#define M2_R_PWM 33
#define M2_EN_L  12
#define M2_EN_R  13

// Stepper motor pins
#define STEPPER_PIN1 19
#define STEPPER_PIN2 18
#define STEPPER_PIN3 5
#define STEPPER_PIN4 17

const int stepsPerRevolution = 2048;
Stepper panStepper(stepsPerRevolution, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

enum Command { STOP, FORWARD, BACKWARD, LEFT, RIGHT, PAN_LEFT, PAN_RIGHT };

// FreeRTOS components
QueueHandle_t commandQueue;
TaskHandle_t motorTaskHandle;
unsigned long lastReceivedTime = 0;
volatile int panDirection = 0; // 0 = stop, 1 = right, -1 = left

// Motor control function for BTS7960
void controlMotor(int speed, uint8_t l_pwm, uint8_t r_pwm, uint8_t en_l, uint8_t en_r) {
    digitalWrite(en_l, HIGH);
    digitalWrite(en_r, HIGH);
    
    if(speed > 0) {
        analogWrite(l_pwm, speed);
        analogWrite(r_pwm, 0);
    } else if(speed < 0) {
        analogWrite(l_pwm, 0);
        analogWrite(r_pwm, -speed);
    } else {
        analogWrite(l_pwm, 0);
        analogWrite(r_pwm, 0);
    }
}

// Motor control task with queue integration
void motorTask(void *pvParams) {
    Command receivedCmd;
    while(1) {
        if(xQueueReceive(commandQueue, &receivedCmd, pdMS_TO_TICKS(50)) == pdPASS) {
            lastReceivedTime = millis();
            
            switch(receivedCmd) {
                case FORWARD:
                    controlMotor(100, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(100, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    panDirection = 0;
                    break;
                case BACKWARD:
                    controlMotor(-100, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(-100, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    panDirection = 0;
                    break;
                case LEFT:
                    controlMotor(-100, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(100, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    panDirection = 0;
                    break;
                case RIGHT:
                    controlMotor(100, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(-100, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    panDirection = 0;
                    break;
                case PAN_LEFT:
                    panDirection = -1;
                    controlMotor(0, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(0, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    break;
                case PAN_RIGHT:
                    panDirection = 1;
                    controlMotor(0, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(0, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    break;
                default: // STOP
                    controlMotor(0, M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R);
                    controlMotor(0, M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R);
                    panDirection = 0;
                    break;
            }
        }
        
        
    }
}

// Pan control task
void panTask(void *pvParams) {
    panStepper.setSpeed(10); // Set speed in RPM
    while(1) {
        if (panDirection != 0) {
            panStepper.step(panDirection); // Move one step in the desired direction
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ESP-NOW callback
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    Command cmd = static_cast<Command>(*data);
    xQueueSendToFront(commandQueue, &cmd, 0);
}

void setup() {
    Serial.begin(115200);
    
    // Motor pin initialization
    const uint8_t motorPins[] = {M1_L_PWM, M1_R_PWM, M1_EN_L, M1_EN_R,
                                M2_L_PWM, M2_R_PWM, M2_EN_L, M2_EN_R};
    for(uint8_t pin : motorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Stepper motor pin initialization
    pinMode(STEPPER_PIN1, OUTPUT);
    pinMode(STEPPER_PIN2, OUTPUT);
    pinMode(STEPPER_PIN3, OUTPUT);
    pinMode(STEPPER_PIN4, OUTPUT);

    // WiFi configuration
    WiFi.mode(WIFI_STA);
    WiFi.enableLongRange(true);
    esp_wifi_set_max_tx_power(82);
    gpio_set_level(GPIO_NUM_21, 1);

    // FreeRTOS initialization
    commandQueue = xQueueCreate(5, sizeof(Command));
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorControl",
        4096,
        nullptr,
        2,  // Higher priority
        &motorTaskHandle,
        0   // Core 0
    );
    xTaskCreatePinnedToCore(
        panTask,
        "PanControl",
        2048,
        nullptr,
        1,  // Lower priority
        nullptr,
        0   // Core 0
    );

    // ESP-NOW initialization
    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
}

void loop() { vTaskDelete(nullptr); }