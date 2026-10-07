#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// --- AX12 PIN DEFINITIONS ---
#define AX12_TX_PIN      4
#define AX12_RX_PIN      25
#define AX12_DIR_PIN     18
#define AX12_UART        UART_NUM_2

const int BAUDRATES[] = {1000000, 115200, 57600, 9600};
const int NUM_BAUDRATES = sizeof(BAUDRATES) / sizeof(BAUDRATES[0]);

uint8_t calculate_checksum(uint8_t id, uint8_t length, uint8_t instruction, uint8_t *parameters, uint8_t param_len) {
    uint32_t checksum_accum = id + length + instruction;
    for (int i = 0; i < param_len; i++) {
        checksum_accum += parameters[i];
    }
    return (uint8_t)(~checksum_accum);
}

void init_ax12_uart(int baud_rate) {
    if (uart_is_driver_installed(AX12_UART)) {
        uart_driver_delete(AX12_UART);
    }

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(AX12_UART, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(AX12_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(AX12_UART, AX12_TX_PIN, AX12_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Invert ONLY the TX pin in software to cancel out the inverting buffer!
    ESP_ERROR_CHECK(uart_set_line_inverse(AX12_UART, UART_SIGNAL_TXD_INV));

    // Configure DIR pin as output
    gpio_reset_pin(AX12_DIR_PIN);
    gpio_set_direction(AX12_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(AX12_DIR_PIN, 1); // Default to RX mode (HIGH disables active-low buffer)
}

bool ping_ax12(uint8_t id) {
    uint8_t tx_packet[6];
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = id;
    tx_packet[3] = 0x02; // Length
    tx_packet[4] = 0x01; // Ping
    tx_packet[5] = calculate_checksum(id, 0x02, 0x01, NULL, 0);

    // Clear RX FIFO
    uart_flush_input(AX12_UART);

    // Enable buffer for TX (LOW for active-low 1G_bar)
    gpio_set_level(AX12_DIR_PIN, 0);
    
    // Write packet
    uart_write_bytes(AX12_UART, (const char*)tx_packet, sizeof(tx_packet));
    
    // Wait until physical transmission ends
    uart_wait_tx_done(AX12_UART, pdMS_TO_TICKS(10));
    
    // Read echo while buffer is still enabled (protects against timing glitches)
    uint8_t echo_buf[16];
    int echo_bytes = uart_read_bytes(AX12_UART, echo_buf, sizeof(tx_packet), pdMS_TO_TICKS(20));
    
    // Disable buffer for RX (HIGH for active-low 1G_bar)
    gpio_set_level(AX12_DIR_PIN, 1);

    // Read response
    uint8_t rx_buf[16];
    int rx_bytes = uart_read_bytes(AX12_UART, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(50));

    if (rx_bytes >= 6) {
        if (rx_buf[0] == 0xFF && rx_buf[1] == 0xFF) {
            uint8_t motor_id = rx_buf[2];
            printf("\n[SUCCESS !!!] AX12 responded! Motor ID: %d (Pinged ID: %d) Packet: ", motor_id, id);
            for (int i = 0; i < rx_bytes; i++) {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n\n");
            return true;
        }
    } else {
        // Log echo status for debugging
        if (echo_bytes > 0) {
            bool echo_match = true;
            if (echo_bytes == sizeof(tx_packet)) {
                for (int i = 0; i < echo_bytes; i++) {
                    if (echo_buf[i] != tx_packet[i]) echo_match = false;
                }
            } else {
                echo_match = false;
            }
            
            if (echo_match) {
                if (id == 254) {
                    printf("  AX12 Broadcast (254): Echo OK (Match), but No Response from the single motor.\n");
                } else {
                    printf("  AX12 ID %d: Echo OK (Match), but No Response.\n", id);
                }
            } else {
                printf("  AX12 ID %d: Echo Corrupted (read %d bytes: ", id, echo_bytes);
                for (int i = 0; i < echo_bytes; i++) printf("%02X ", echo_buf[i]);
                printf(").\n");
            }
        } else {
            printf("  AX12 ID %d: NO ECHO (Wiring/IC issue).\n", id);
        }
    }
    return false;
}

void app_main(void) {
    printf("\n=========================================\n");
    printf("  AX-12A BROADCAST + SINGLE ID SCANNER\n");
    printf("=========================================\n");

    while (1) {
        for (int b = 0; b < NUM_BAUDRATES; b++) {
            int baud = BAUDRATES[b];
            printf("\n--- Scanning AX12 at Baudrate: %d ---\n", baud);

            init_ax12_uart(baud);
            vTaskDelay(pdMS_TO_TICKS(100));

            // 1. First send Broadcast Ping (ID 254) - if single motor is connected, it MUST answer regardless of its ID!
            printf("Sending Broadcast Ping (ID 254)...\n");
            ping_ax12(254);
            vTaskDelay(pdMS_TO_TICKS(50));

            // 2. Then scan individual IDs 1 to 20
            for (uint8_t id = 1; id <= 20; id++) {
                ping_ax12(id);
                vTaskDelay(pdMS_TO_TICKS(30));
            }
        }
        
        printf("\nScan complete. Waiting 10 seconds before next run...\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
