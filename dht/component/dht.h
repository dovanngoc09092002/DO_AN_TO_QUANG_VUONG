// #ifndef DHT11_H
// #define DHT11_H

// #include "driver/gpio.h"

// #define DHT11_OK 0
// #define DHT11_ERROR_TIMEOUT -1
// #define DHT11_ERROR_CRC -2

// void dht11_init(gpio_num_t dht_pin);
// int dht11_read_data(gpio_num_t dht_pin, float *temperature, float *humidity);

// #endif

#ifndef DHT11_H_
#define DHT11_H_

#include "driver/gpio.h"

enum dht11_status {
    DHT11_CRC_ERROR = -2,
    DHT11_TIMEOUT_ERROR,
    DHT11_OK
};

struct dht11_reading {
    int status;
    float temperature;
    float humidity;
};

void DHT11_init(gpio_num_t);

int dht11_read_data(gpio_num_t dht_pin, float *temperature, float *humidity);

#endif