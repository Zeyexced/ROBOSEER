// ESP32U DEV BOARD 38P - MASTER (Controller)
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define BUTTON_UP        15
#define BUTTON_DOWN      23
#define BUTTON_LEFT      12
#define BUTTON_RIGHT     13
#define BUTTON_PAN_LEFT  14
#define BUTTON_PAN_RIGHT 27

uint8_t slaveMAC[] = {0x20, 0x43, 0xA8, 0x65, 0x6F, 0xE8};
enum Command { STOP, FORWARD, BACKWARD, LEFT, RIGHT, PAN_LEFT, PAN_RIGHT };
QueueHandle_t buttonQueue;

// Debounced button task
void buttonTask(void *pvParams) {
    uint8_t lastStates = 0;
    while(1) {
        uint8_t currentStates = (!digitalRead(BUTTON_UP) << 5) |
                                (!digitalRead(BUTTON_DOWN) << 4) |
                                (!digitalRead(BUTTON_LEFT) << 3) |
                                (!digitalRead(BUTTON_RIGHT) << 2) |
                                (!digitalRead(BUTTON_PAN_LEFT) << 1) |
                                (!digitalRead(BUTTON_PAN_RIGHT));
        
        if(currentStates != lastStates) {
            xQueueSend(buttonQueue, &currentStates, 0);
            lastStates = currentStates;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // Debounce delay
    }
}

// Transmission task
void transmitTask(void *pvParams) {
    uint8_t buttonStates;
    while(1) {
        if(xQueueReceive(buttonQueue, &buttonStates, portMAX_DELAY)) {
            Command cmd = STOP;
            if (buttonStates & 0b100000) {
                cmd = FORWARD;
            } else if (buttonStates & 0b010000) {
                cmd = BACKWARD;
            } else if (buttonStates & 0b001000) {
                cmd = LEFT;
            } else if (buttonStates & 0b000100) {
                cmd = RIGHT;
            } else if (buttonStates & 0b000010) {
                cmd = PAN_LEFT;
            } else if (buttonStates & 0b000001) {
                cmd = PAN_RIGHT;
            }
            
            esp_now_send(slaveMAC, (uint8_t *)&cmd, sizeof(cmd));
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // Button initialization
    pinMode(BUTTON_UP, INPUT_PULLUP);
    pinMode(BUTTON_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(BUTTON_PAN_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_PAN_RIGHT, INPUT_PULLUP);

    // WiFi configuration
    WiFi.mode(WIFI_STA);
    WiFi.enableLongRange(true);
    esp_wifi_set_max_tx_power(82);
    gpio_set_level(GPIO_NUM_21, 1);

    // FreeRTOS initialization
    buttonQueue = xQueueCreate(5, sizeof(uint8_t));
    xTaskCreate(buttonTask, "ButtonTask", 2048, nullptr, 2, nullptr);
    xTaskCreate(transmitTask, "TransmitTask", 2048, nullptr, 3, nullptr);

    // ESP-NOW initialization
    if(esp_now_init() != ESP_OK) return;
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, slaveMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void loop() { vTaskDelete(nullptr); }