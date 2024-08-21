#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "cJSON.h" // Include the cJSON library

#define RS485_CONTROL_PIN GPIO_NUM_8
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define EX_UART_NUM UART_NUM_0

static QueueHandle_t uart1_queue;
static bool isExtendedQuery = false;  // Flag to indicate an extended query


// CRC8 Calculation Function
uint8_t calculateCRC8(uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t extract = data[i];
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}
// Transmission Control Functions
void preTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 1);
}
void postTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 0);
}
// Send Command Function
void sendCommand(uint8_t *command, size_t length) {
    preTransmission();
    uart_write_bytes(UART_PORT_NUM, (const char*)command, length);
    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
    postTransmission();
}
// Conversion Functions
int16_t convertCurrentToData(float current) {
    return (int16_t)(current * 4095.875); // 32767 / 8 = 4095.875
}

float convertDataToCurrent(int16_t data) {
    return (float)data / 4095.875;
}

int16_t convertSpeedToData(float speed) {
    return (int16_t)speed;
}

float convertDataToSpeed(int16_t data) {
    return (float)data;
}

uint16_t convertPositionToData(float position) {
    return (uint16_t)(position * 91.0194); // 32767 / 360 = 91.0194
}

float convertDataToPosition(uint16_t data) {
    return (float)data / 91.0194;
}
// Command Functions
void switchToVelocityLoop(uint8_t mode) {
    uint8_t command[] = {0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode};
    uint8_t command1[] = {0x02, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode};
    sendCommand(command, sizeof(command)); // No CRC calculation needed
    vTaskDelay(pdMS_TO_TICKS(10));
    sendCommand(command1, sizeof(command1)); 
    vTaskDelay(pdMS_TO_TICKS(10));
}
void queryMotorMode() {
    uint8_t command[] = {0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
    uint8_t command1[] = {0x02, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1};
    sendCommand(command, sizeof(command));
    vTaskDelay(pdMS_TO_TICKS(10));
    sendCommand(command1, sizeof(command1));
    vTaskDelay(pdMS_TO_TICKS(10));
        isExtendedQuery = true;    // Set the flag to indicate that the extended query was made
}
void brakeCommand() {
    uint8_t command[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1};
    uint8_t command1[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x24};
    sendCommand(command, sizeof(command));
    vTaskDelay(pdMS_TO_TICKS(10));
    sendCommand(command1,sizeof(command1));
    vTaskDelay(pdMS_TO_TICKS(10));
}
void setMotorID(uint8_t newID) {
    printf("Setting motor ID to: %d\n", newID);
    // Prepare command
    uint8_t command[] = {0xAA, 0x55, 0x53, newID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // Send command
    for(int i = 0;i < 5;i++){
    sendCommand(command, sizeof(command));
    // Log command sent
    printf("Sent setMotorID command: ");
    for (int i = 0; i < sizeof(command); i++) {
        printf("%02X ", command[i]);
    }
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void queryID() {
    uint8_t command[] = {0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE};
    sendCommand(command, sizeof(command));
}
// Command Functions for Dual Motor Control
void currentLoopCommand(uint8_t motorID1, float current1, uint8_t motorID2, float current2) {
    int16_t currentData1 = convertCurrentToData(current1);
    int16_t currentData2 = convertCurrentToData(current2);

    // Command for Motor 1
    uint8_t command1[10] = {motorID1, 0x64, (uint8_t)(currentData1 >> 8), (uint8_t)currentData1, 0x00, 0x00, 0x00, 0x00, 0x00};
    command1[9] = calculateCRC8(command1, 9);
    sendCommand(command1, sizeof(command1));
vTaskDelay(pdMS_TO_TICKS(10));
    // Command for Motor 2
    uint8_t command2[10] = {motorID2, 0x64, (uint8_t)(currentData2 >> 8), (uint8_t)currentData2, 0x00, 0x00, 0x00, 0x00, 0x00};
    command2[9] = calculateCRC8(command2, 9);
    sendCommand(command2, sizeof(command2));
vTaskDelay(pdMS_TO_TICKS(10));
}
void velocityLoopCommand(uint8_t motorID1, float speed1, uint8_t motorID2, float speed2) {
    int16_t speedData1 = convertSpeedToData(speed1);
    int16_t speedData2 = convertSpeedToData(speed2);

    // Command for Motor 1
    uint8_t command1[10] = {motorID1, 0x64, (uint8_t)(speedData1 >> 8), (uint8_t)speedData1, 0x00, 0x00, 0x00, 0x00, 0x00};
    command1[9] = calculateCRC8(command1, 9);
    sendCommand(command1, sizeof(command1));
vTaskDelay(pdMS_TO_TICKS(10));
    // Command for Motor 2
    uint8_t command2[10] = {motorID2, 0x64, (uint8_t)(speedData2 >> 8), (uint8_t)speedData2, 0x00, 0x00, 0x00, 0x00, 0x00};
    command2[9] = calculateCRC8(command2, 9);
    sendCommand(command2, sizeof(command2));
vTaskDelay(pdMS_TO_TICKS(10));
}
void positionLoopCommand(uint8_t motorID1, float position1, uint8_t motorID2, float position2) {
    uint16_t positionData1 = convertPositionToData(position1);
    uint16_t positionData2 = convertPositionToData(position2);

    // Command for Motor 1
    uint8_t command1[10] = {motorID1, 0x64, (uint8_t)(positionData1 >> 8), (uint8_t)positionData1, 0x00, 0x00, 0x00, 0x00, 0x00};
    command1[9] = calculateCRC8(command1, 9);
    sendCommand(command1, sizeof(command1));
vTaskDelay(pdMS_TO_TICKS(10));
    // Command for Motor 2
    uint8_t command2[10] = {motorID2, 0x64, (uint8_t)(positionData2 >> 8), (uint8_t)positionData2, 0x00, 0x00, 0x00, 0x00, 0x00};
    command2[9] = calculateCRC8(command2, 9);
    sendCommand(command2, sizeof(command2));
vTaskDelay(pdMS_TO_TICKS(10));
}
// JSON COMMAND FUNCTION
void handleJSONCommand(const char *jsonString) {
    cJSON *json = cJSON_Parse(jsonString);
    if (json == NULL) {
        printf("Invalid JSON string\n");
        return;
    }
    // Check for "BRAKE" command
    cJSON *brakeItem = cJSON_GetObjectItem(json, "BRAKE");
    if (brakeItem != NULL) {
        printf("BRAKE command received\n");
        brakeCommand();
    }
    // Check for "QUERY MOTOR" command
    cJSON *queryMotorItem = cJSON_GetObjectItem(json, "QUERY MOTOR");
    if (queryMotorItem != NULL) {
        printf("QUERY MOTOR command received\n");
        queryMotorMode();
        // Assume you'll handle the response in the UART event task
    }
    // Check for "QUERY ID" command
    cJSON *queryIDItem = cJSON_GetObjectItem(json, "QUERY ID");
    if (queryIDItem != NULL) {
        printf("QUERY ID command received\n");
        queryID();
    }
    // Check for "mode" command
    cJSON *modeItem = cJSON_GetObjectItem(json, "mode");
    if (modeItem != NULL) {
        uint8_t mode = (uint8_t)modeItem->valueint;
        printf("Mode command received: %d\n", mode);
        // Set the motor to the specified mode
        switchToVelocityLoop(mode);
        // Motor IDs
        uint8_t motorID1 = 0x01;
        uint8_t motorID2 = 0x02;
        // Execute the appropriate command based on the mode
        switch (mode) {
            case 0x01: { // Current mode
                cJSON *current1Item = cJSON_GetObjectItem(json, "current1");
                cJSON *current2Item = cJSON_GetObjectItem(json, "current2");
                if (current1Item != NULL && current2Item != NULL) {
                    float current1 = (float)current1Item->valuedouble;
                    float current2 = (float)current2Item->valuedouble;
                    currentLoopCommand(motorID1, current1, motorID2, current2);
                } else {
                    printf("Error: Missing current parameters for Current mode.\n");
                }
                break;
            }
            case 0x02: { // Velocity mode
                cJSON *velocity1Item = cJSON_GetObjectItem(json, "velocity1");
                cJSON *velocity2Item = cJSON_GetObjectItem(json, "velocity2");
                if (velocity1Item != NULL && velocity2Item != NULL) {
                    float velocity1 = (float)velocity1Item->valuedouble;
                    float velocity2 = (float)velocity2Item->valuedouble;
                    velocityLoopCommand(motorID1, velocity1, motorID2, velocity2);
                } else {
                    printf("Error: Missing velocity parameters for Velocity mode.\n");
                }
                break;
            }
            case 0x03: { // Position mode
                cJSON *position1Item = cJSON_GetObjectItem(json, "position1");
                cJSON *position2Item = cJSON_GetObjectItem(json, "position2");
                if (position1Item != NULL && position2Item != NULL) {
                    float position1 = (float)position1Item->valuedouble;
                    float position2 = (float)position2Item->valuedouble;
                    positionLoopCommand(motorID1, position1, motorID2, position2);
                } else {
                    printf("Error: Missing position parameters for Position mode.\n");
                }
                break;
            }
            default:
                printf("Error: Unknown mode received.\n");
                break;
        }
    } else {
        printf("Error: No mode command found in JSON input.\n");
    }
    // Check for "newID" command
    cJSON *newIDItem = cJSON_GetObjectItem(json, "newID");
    if (newIDItem != NULL) {
        uint8_t newID = (uint8_t)newIDItem->valueint;
        printf("Setting new motor ID to: %d\n", newID);
        setMotorID(newID);
    }
    cJSON_Delete(json);
}

void decodeStandardMotorResponse(uint8_t *data) {
     uint8_t motorID = data[0];
     uint8_t mode = data[1];

    int16_t torqueCurrent = (data[2] << 8) | data[3];
    float torqueCurrentValue = (torqueCurrent / 4095.0) * 8.0; // Convert to Amperes

    int16_t velocity = (data[4] << 8) | data[5];
    float velocityValue = velocity; // Already in rpm

    uint16_t position = (data[6] << 8) | data[7];
    float positionValue = (position / 32767.0) * 360.0; // Convert to degrees

    uint8_t errorCode = data[8];
    uint8_t crc8 = data[9];

    // Validate CRC8
    uint8_t calculatedCRC8 = calculateCRC8(data, 9);
    if (calculatedCRC8 != crc8) {
        printf("Warning: CRC8 mismatch: calculated %02X, received %02X\n", calculatedCRC8, crc8);
        return;
    }

    printf("Decoded QUERY MOTOR Response:\n");
    printf("{\"ID\": \"%d\", \"MODE\": \"%d\", \"VELOCITY\": \"%f rpm\", \"POSITION\": \"%f degrees\", \"CURRENT\": \"%f A\", \"ERROR\": \"%d\"}\n", 
           data[0], mode, velocityValue, positionValue, torqueCurrentValue, errorCode);
}

void decodeExtendedMotorResponse(uint8_t *data) {
    uint8_t motorID = data[0];
    uint8_t mode = data[1];

    int16_t torqueCurrent = (data[2] << 8) | data[3];
    float torqueCurrentValue = (torqueCurrent / 4095.0) * 8.0; // Convert to Amperes

    int16_t velocity = (data[4] << 8) | data[5];
    float velocityValue = velocity; // Already in rpm

    uint8_t windingTemp = data[6]; // Winding temperature

    uint8_t position = data[7]; // U8 position value

    uint8_t errorCode = data[8];
    uint8_t crc8 = data[9];

    // Validate CRC8
    uint8_t calculatedCRC8 = calculateCRC8(data, 9);
    if (calculatedCRC8 != crc8) {
        printf("Warning: CRC8 mismatch: calculated %02X, received %02X\n", calculatedCRC8, crc8);
        return;
    }

    printf("Decoded Extended Motor Response:\n");
    printf("{\"ID\": \"%d\", \"MODE\": \"%d\", \"CURRENT\": \"%f A\", \"VELOCITY\": \"%f rpm\", \"WINDING_TEMP\": \"%d C\", \"POSITION\": \"%d\", \"ERROR\": \"%d\"}\n", 
           motorID, mode, torqueCurrentValue, velocityValue, windingTemp, position, errorCode);
}
// UART Event Task
void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[BUF_SIZE];

    while (1) {
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size) {
                        int len = uart_read_bytes(UART_PORT_NUM, data, event.size, 100 / portTICK_PERIOD_MS);
                        printf("Received data event:\n");
                        for (int i = 0; i < len; i++) {
                            printf("%02X ", data[i]);
                        }
                        printf("\n");

                        // Decode response based on the flag
                        if (isExtendedQuery ) {
                            decodeExtendedMotorResponse(data);
                            // Reset the flag after decoding
                            isExtendedQuery = false;
                        } else {
                            decodeStandardMotorResponse(data);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}
// Main Application
void app_main(void) {
    // Configure UART1 (RS485)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(UART_PORT_NUM, &uart_config) != ESP_OK) {
        printf("Failed to configure UART\n");
        return;
    }
    if (uart_set_pin(UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        printf("Failed to set UART pins\n");
        return;
    }
    if (uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0) != ESP_OK) {
        printf("Failed to install UART driver\n");
        return;
    }

    // Configure RS485 control pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    postTransmission(); // Set initial state to listening

    // Configure UART0 (serial monitor)
    if (uart_driver_install(EX_UART_NUM, BUF_SIZE * 4, BUF_SIZE * 4, 0, NULL, 0) != ESP_OK) {
        printf("Failed to install UART0 driver\n");
        return;
    }
    if (uart_param_config(EX_UART_NUM, &uart_config) != ESP_OK) {
        printf("Failed to configure UART0\n");
        return;
    }
    if (uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        printf("Failed to set UART0 pins\n");
        return;
    }

    // Create UART event task
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);

    // Main loop to read from serial monitor and send over RS485
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    if (data == NULL) {
        printf("Failed to allocate memory\n");
        return;
    }
    while (1) {
        // Read user input from the serial monitor
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the input string
            printf("Read from UART0: %s\n", data);

            // Handle JSON command
            handleJSONCommand((const char*)data);

            // Log after handling command
            printf("Handled command\n");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay before next iteration
    }

    free(data);
}