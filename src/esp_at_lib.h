/*
 *  The MIT License (MIT)
 *  Copyright (C) 2019  Seeed Technology Co.,Ltd.
 */
#ifndef _ESP_AT_LIB_H_
#define _ESP_AT_LIB_H_

#include <Arduino.h>

#include <esp_config.h>
#include <esp/esp_includes.h>

extern
#ifdef __cplusplus
"C" 
#endif
size_t get_free_heap(void);

#endif//_ESP_AT_LIB_H_
