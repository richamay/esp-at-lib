/**
 * \file            esp_ll_arduino.cpp
 * \brief           Generic arduino driver, included in various Arduino driver variants
 */

/*
 * Copyright (c) 2019 Tilen MAJERLE
 * Copyright (C) 2019 Seeed Technology Co.,Ltd.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ESP-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         $_version_$
 *
 * Author:          Peter Yang <turmary@126.com>
 * Version:         1.01
 *                  Arduino porting
 *
 */

/*
 * How it works
 *
 * On first call to \ref esp_ll_init, new thread is created and processed in usart_ll_thread function.
 * USART is configured in RX DMA mode and any incoming bytes are processed inside thread function.
 * DMA and USART implement interrupt handlers to notify main thread about new data ready to send to upper layer.
 *
 * \ref ESP_CFG_INPUT_USE_PROCESS must be enabled in `esp_config.h` to use this driver.
 */
#include "Arduino.h"
#include "esp/esp.h"
#include "esp/esp_mem.h"
#include "esp/esp_input.h"
#include "system/esp_ll.h"
#include "esp_ll_spi.h"

#if !__DOXYGEN__

#if !ESP_CFG_INPUT_USE_PROCESS
#error "ESP_CFG_INPUT_USE_PROCESS must be enabled in `esp_config.h` to use this driver."
#endif /* ESP_CFG_INPUT_USE_PROCESS */

#if !defined(ESP_USART_DMA_RX_BUFF_SIZE)
#define ESP_USART_DMA_RX_BUFF_SIZE      0x1000
#endif /* !defined(ESP_USART_DMA_RX_BUFF_SIZE) */

#if !defined(ESP_MEM_SIZE)
#define ESP_MEM_SIZE                    0x2000
#endif /* !defined(ESP_MEM_SIZE) */

#if !defined(ESP_USART_RDR_NAME)
#define ESP_USART_RDR_NAME              RDR
#endif /* !defined(ESP_USART_RDR_NAME) */


#if ESP_CFG_USE_SPI
auto& at_uart = ARDUINO_SPI_TO_ESPAT;
static esp_sys_mutex_t spi_ll_mutex_id;
#else
auto& at_uart = ARDUINO_SERIAL_TO_ESP32AT;
#endif

/* USART memory */
static uint8_t      usart_mem[ESP_USART_DMA_RX_BUFF_SIZE];
static uint8_t      is_running, initialized;
static size_t       old_pos;

/* USART thread */
static void usart_ll_thread(void* arg);
static esp_sys_thread_t usart_ll_thread_id;

/* Message queue */
static esp_sys_mbox_t usart_ll_mbox_id;

extern "C"
size_t get_free_heap(void) {
    size_t i;

    /* Get max heap size */
    for (i = 1; i < 4096; i++) {
        char* p;

        p = (char*)malloc(1024 * i);
        if (p) {
            free(p);
        } else {
            break;
        }
    }
    return i - 1;
}

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void* arg) {
    size_t pos;

    ESP_UNUSED(arg);

    while (1) {
        int sz;

        #if ESP_CFG_USE_SPI
        esp_sys_mutex_lock(&spi_ll_mutex_id);
        sz = at_spi_read(&usart_mem[0], ESP_USART_DMA_RX_BUFF_SIZE);
        esp_sys_mutex_unlock(&spi_ll_mutex_id);
        if (sz <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        esp_input_process(&usart_mem[0], sz);

        #else

        sz = at_uart.available();
        if (sz <= 0) {
            // Delay required here, 
            // if this task has highest priority.
            vTaskDelay(pdMS_TO_TICKS(1));
            // esp_sys_thread_yield();
            continue;
        }
        if (sz > ESP_USART_DMA_RX_BUFF_SIZE) {
            sz = ESP_USART_DMA_RX_BUFF_SIZE;
        }
        at_uart.readBytes(&usart_mem[0], sz);
        esp_input_process(&usart_mem[0], sz);
        #endif
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void
configure_uart(uint32_t baudrate) {
    if (!initialized) {
        #if ESP_CFG_USE_SPI
        at_spi_begin();
        #else
        at_uart.begin(baudrate);
        #endif

        old_pos = 0;
        is_running = 1;
    } else {
        vTaskDelay(10);
        // at_uart.begin(baudrate);
    }

    /* Create mbox and start thread */
    if (usart_ll_mbox_id == NULL) {
        esp_sys_mbox_create(&usart_ll_mbox_id, 10);
    }
    if (usart_ll_thread_id == NULL) {
        esp_sys_thread_create(&usart_ll_thread_id, "usart_ll_thread",
                              usart_ll_thread, usart_ll_mbox_id,
                              2048,
                              ESP_SYS_THREAD_PRIO - 1
                              );
    }
}

#if defined(ESP_RESET_PIN)
/**
 * \brief           Hardware reset callback
 */
static uint8_t
reset_device(uint8_t state) {
    if (state) {                                /* Activate reset line */
        at_uart.end();
    } else {
        #if ESP_CFG_USE_SPI
        at_spi_begin();
        #else
        at_uart.begin(ll->uart.baudrate);
        #endif
    }
    return 1;
}
#endif /* defined(ESP_RESET_PIN) */

/**
 * \brief           Send data to GSM device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void* data, size_t len) {
    const uint8_t* d = (const uint8_t*)data;

    #if ESP_CFG_USE_SPI
    if (data) {
        esp_sys_mutex_lock(&spi_ll_mutex_id);
        len = at_spi_write(d, len);
        esp_sys_mutex_unlock(&spi_ll_mutex_id);
    } else {
        // no need flush
    }
    #else
    at_uart.write(d, len);
    at_uart.flush();
    #endif
    return len;
}

/**
 * \brief           Callback function called from initialization process
 * \note            This function may be called multiple times if AT baudrate is changed from application
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \param[in]       baudrate: Baudrate to use on AT port
 * \return          Member of \ref espr_t enumeration
 */
espr_t
esp_ll_init(esp_ll_t* ll) {
#if !ESP_CFG_MEM_CUSTOM
    static uint8_t memory[ESP_MEM_SIZE];
    esp_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };

    if (!initialized) {
        esp_mem_assignmemory(mem_regions, ESP_ARRAYSIZE(mem_regions));  /* Assign memory for allocations */
    }
#endif /* !ESP_CFG_MEM_CUSTOM */

    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
#if defined(ESP_RESET_PIN)
        ll->reset_fn = reset_device;            /* Set callback for hardware reset */
#endif /* defined(ESP_RESET_PIN) */
    }

    #if ESP_CFG_USE_SPI
    if (spi_ll_mutex_id == NULL) {
        esp_sys_mutex_create(&spi_ll_mutex_id);
    }
    #endif

    configure_uart(ll->uart.baudrate);          /* Initialize UART for communication */
    initialized = 1;
    return espOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \return          \ref espOK on success, member of \ref espr_t enumeration otherwise
 */
espr_t
esp_ll_deinit(esp_ll_t* ll) {
    ll = ll;

    if (usart_ll_mbox_id != NULL) {
        esp_sys_mbox_t tmp = usart_ll_mbox_id;
        usart_ll_mbox_id = NULL;
        esp_sys_mbox_delete(&tmp);
    }
    if (usart_ll_thread_id != NULL) {
        esp_sys_thread_t tmp = usart_ll_thread_id;
        usart_ll_thread_id = NULL;
        esp_sys_thread_terminate(&tmp);
    }

    #if ESP_CFG_USE_SPI
    if (spi_ll_mutex_id == NULL) {
        esp_sys_mutex_create(&spi_ll_mutex_id);
    }
    if (spi_ll_mutex_id != NULL) {
        esp_sys_mutex_delete(&spi_ll_mutex_id);
        spi_ll_mutex_id = NULL;
    }
    #endif

    initialized = 0;
    return espOK;
}

// setup and loop code block
extern void _real_body();
static void init_thread(void* arg);

void _wrap_body(){
    esp_sys_thread_t id_init;

    Serial.begin(115200);
    /*
    while (!Serial) {
        // wait for serial port to connect.
        // Needed for native USB port only
        ;
    }
    */

    /* Initialize, create first thread and start kernel */
    // osKernelInitialize();
    esp_sys_thread_create(&id_init, "init_thread",
                          init_thread, NULL,
                          2048,
                          ESP_SYS_THREAD_PRIO - 1);
    
    vTaskStartScheduler();
    return;
}

static espr_t esp_callback_func(esp_evt_t* evt);

/**
 * \brief           Initialization thread
 * \param[in]       arg: Thread argument
 */
static void
init_thread(void* arg) {
    int i;

    arg = arg;

    i = (int)get_free_heap();
    xprintf("\r\n*** malloc MAX = %dKiB ***\r\n", i);
    
    /* Get max C++ heap size */
    for (i = 1; i < 4096; i++) {
        char* p;

        p = new char[1024 * i];
        if (p) {
            delete[] p;
        } else {
            xprintf("\r\n*** new    MAX = %dKiB ***\r\n", i - 1);
            break;
        }
    }

    /* Initialize ESP with default callback function */
    xprintf("Initializing ESP-AT Lib\r\n");
    if (esp_init(esp_callback_func, 1) != espOK) {
        xprintf("Cannot initialize ESP-AT Lib!\r\n");
    } else {
        xprintf("ESP-AT Lib initialized!\r\n");
    }

    _real_body();
}

/**
 * \brief           Event callback function for ESP stack
 * \param[in]       evt: Event information with data
 * \return          espOK on success, member of \ref espr_t otherwise
 */
static espr_t
esp_callback_func(esp_evt_t* evt) {
    switch (esp_evt_get_type(evt)) {
        case ESP_EVT_AT_VERSION_NOT_SUPPORTED: {
            esp_sw_version_t v_min, v_curr;

            esp_get_min_at_fw_version(&v_min);
            esp_get_current_at_fw_version(&v_curr);

            xprintf("Current ESP8266 AT version is not supported by library!\r\n");
            xprintf("Minimum required AT version is: %d.%d.%d\r\n", (int)v_min.major, (int)v_min.minor, (int)v_min.patch);
            xprintf("Current AT version is: %d.%d.%d\r\n", (int)v_curr.major, (int)v_curr.minor, (int)v_curr.patch);
            break;
        }
        case ESP_EVT_INIT_FINISH: {
            xprintf("Library initialized!\r\n");
            break;
        }
        case ESP_EVT_RESET: {
            xprintf("Device reset sequence finished!\r\n");
            break;
        }
        case ESP_EVT_RESET_DETECTED: {
            xprintf("Device reset detected!\r\n");
            break;
        }
        default: break;
    }
    return espOK;
}

extern "C" {

#undef printf
#include <stdio.h>

static char xprt_buf[0x180];
int xprintf(const char *fmt, ...)
{
    va_list argptr;
    int rv;

    va_start(argptr, fmt);
    rv = vsnprintf(xprt_buf, sizeof xprt_buf - 4, fmt, argptr);
    va_end(argptr);

    if (!Serial) {
        return rv;
    }

    Serial.write(xprt_buf);
    Serial.flush();
    return rv;
}

}// extern "C"

/*
// arm-none-eabi-gcc/7-2017q4
// already defined __throw_bad_alloc
// lib/thumb/v7e-m/fpv4-sp/hard\libstdc++_nano.a(functexcept.o): In function `std::__throw_bad_alloc()':

namespace std{
    void __throw_bad_function_call(){
        xprintf("__throw_bad_function_call:ERROR\n");
    }
    void __throw_bad_alloc() {
        xprintf("__throw_bad_alloc:ERROR\n");
    }
    void __throw_length_error(char const* s){
        xprintf("__throw_length_error:ERROR %s\n", s);
    }
}
//*/

#endif /* !__DOXYGEN__ */
