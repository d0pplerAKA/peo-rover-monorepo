#pragma once
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include <vector>
#include <cstring>
struct uart_config_t { int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk; };
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
extern std::vector<uint8_t> mock_rx;
extern size_t mock_rx_pos;
extern std::vector<std::vector<uint8_t>> mock_tx;
extern unsigned mock_tx_waits;
extern unsigned mock_fail_drains;
extern void (*mock_on_drain)();
extern std::vector<char> mock_tx_order;
inline int uart_driver_install(int,int,int,int,void*,int) { return ESP_OK; }
inline int uart_param_config(int,const uart_config_t*) { return ESP_OK; }
inline int uart_set_pin(int,int,int,int,int) { return ESP_OK; }
inline int uart_read_bytes(int,void* out,unsigned n,TickType_t) {
    if(mock_rx_pos==mock_rx.size()) throw StopTask{};
    unsigned count=static_cast<unsigned>(mock_rx.size()-mock_rx_pos); if(count>n) count=n;
    memcpy(out,mock_rx.data()+mock_rx_pos,count); mock_rx_pos+=count; return count;
}
inline int uart_write_bytes(int,const void* data,unsigned len) {
    mock_tx_order.push_back('W');
    auto p=static_cast<const uint8_t*>(data); mock_tx.emplace_back(p,p+len); return len;
}
inline int uart_wait_tx_done(int,TickType_t) {
    ++mock_tx_waits; mock_tx_order.push_back('D');
    if(mock_on_drain) { auto hook=mock_on_drain; mock_on_drain=nullptr; hook(); }
    if(mock_fail_drains) { --mock_fail_drains; return ESP_FAIL; }
    return ESP_OK;
}
