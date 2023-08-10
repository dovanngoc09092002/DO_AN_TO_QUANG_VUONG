// #include "dht.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// void dht11_init(gpio_num_t dht_pin) {
//     gpio_pad_select_gpio(dht_pin);
//     gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
//     gpio_set_level(dht_pin, 1);
// }

// int dht11_read_data(gpio_num_t dht_pin, float *temperature, float *humidity) {
//     uint8_t data[5] = {0};
//     uint8_t bit_shift = 7;
//     uint8_t byte_index = 0;

//     gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
//     gpio_set_level(dht_pin, 0);
//     vTaskDelay(pdMS_TO_TICKS(20));

//     gpio_set_level(dht_pin, 1);
//     vTaskDelay(30);

//     gpio_set_direction(dht_pin, GPIO_MODE_INPUT);

//     uint32_t timeout = 1000;
//     while (gpio_get_level(dht_pin) == 0) {
//         if (--timeout == 0) {
//             return DHT11_ERROR_TIMEOUT;
//         }
//         vTaskDelay(1);
//     }

//     timeout = 1000;
//     while (gpio_get_level(dht_pin) == 1) {
//         if (--timeout == 0) {
//             return DHT11_ERROR_TIMEOUT;
//         }
//         vTaskDelay(1);
//     }

//     for (uint8_t i = 0; i < 40; i++) {
//         timeout = 1000;
//         while (gpio_get_level(dht_pin) == 0) {
//             if (--timeout == 0) {
//                 return DHT11_ERROR_TIMEOUT;
//             }
//             ets_delay_us(1);
//         }

//         uint32_t duration = 0;
//         timeout = 1000;
//         while (gpio_get_level(dht_pin) == 1) {
//             if (--timeout == 0) {
//                 return DHT11_ERROR_TIMEOUT;
//             }
//             ets_delay_us(1);
//             duration++;
//         }

//         if (i % 8 == 0) {
//             data[byte_index] = 0;
//         }

//         if (duration > 50) {
//             data[byte_index] |= (1 << bit_shift);
//         }

//         if (bit_shift == 0) {
//             bit_shift = 7;
//             byte_index++;
//         } else {
//             bit_shift--;
//         }
//     }

//     if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
//         *humidity = data[0];
//         *temperature = data[2];
//         return DHT11_OK;
//     } else {
//         return DHT11_ERROR_CRC;
//     }
// }

#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dht.h"

static gpio_num_t dht_gpio;
static int64_t last_read_time = -2000000;
static struct dht11_reading last_read;

static int _waitOrTimeout(uint16_t microSeconds, int level) {
    int micros_ticks = 0;
    while (gpio_get_level(dht_gpio) == level) { 
        if (micros_ticks++ > microSeconds) 
            return DHT11_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}

static int _checkCRC(uint8_t data[]) {
    if (data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DHT11_OK;
    else
        return DHT11_CRC_ERROR;
}

static void _sendStartSignal() {
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

static int _checkResponse() {
    /* Wait for next step ~80us */
    if (_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;

    /* Wait for next step ~80us */
    if (_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR) 
        return DHT11_TIMEOUT_ERROR;

    return DHT11_OK;
}

static struct dht11_reading _timeoutError() {
    struct dht11_reading timeoutError = {DHT11_TIMEOUT_ERROR, -1, -1};
    return timeoutError;
}

static struct dht11_reading _crcError() {
    struct dht11_reading crcError = {DHT11_CRC_ERROR, -1, -1};
    return crcError;
}

void DHT11_init(gpio_num_t gpio_num) {
    /* Wait 1 second to make the device pass its initial unstable status */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    dht_gpio = gpio_num;
}

int dht11_read_data(gpio_num_t dht_pin, float *temperature, float *humidity) {
    /* Tried to sense too soon since last read (DHT11 needs ~2 seconds to make a new read) */
    if (esp_timer_get_time() - 2000000 < last_read_time) {
        *temperature = last_read.temperature;
        *humidity = last_read.humidity;
        return last_read.status;
    }

    last_read_time = esp_timer_get_time();

    uint8_t data[5] = {0, 0, 0, 0, 0};

    _sendStartSignal();

    if (_checkResponse() == DHT11_TIMEOUT_ERROR) {
        last_read = _timeoutError();
        *temperature = last_read.temperature;
        *humidity = last_read.humidity;
        return last_read.status;
    }
    
    /* Read response */
    for (int i = 0; i < 40; i++) {
        /* Initial data */
        if (_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR) {
            last_read = _timeoutError();
            *temperature = last_read.temperature;
            *humidity = last_read.humidity;
            return last_read.status;
        }
                
        if (_waitOrTimeout(70, 1) > 28) {
            /* Bit received was a 1 */
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    if (_checkCRC(data) != DHT11_CRC_ERROR) {
        last_read.status = DHT11_OK;
        last_read.temperature = data[2];
        last_read.humidity = data[0];
        *temperature = last_read.temperature;
        *humidity = last_read.humidity;
        return last_read.status;
    } else {
        last_read = _crcError();
        *temperature = last_read.temperature;
        *humidity = last_read.humidity;
        return last_read.status;
    }
}