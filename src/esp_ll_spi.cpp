/*
 * Copyright (C) 2019 Seeed Technology Co.,Ltd.
 *
 * created 27 March 2020
 * by Peter Yang <turmary@126.com>
 *
 * MIT License.
 *
 */

// the device communicates using SPI, so include the library:
#include "esp_config.h"
#include "esp_ll_spi.h"

#define RESPONSE_READY  "\r\nready\r\n"

enum SPI_DIR
{
    SPI_DIR_MISO = LOW,
    SPI_DIR_MOSI = HIGH,
};

enum
{
	SPT_TAG_PRE	= 0x55, /* Master initiate a TRANSFER */
	SPT_TAG_WR	= 0x80, /* Master WRITE  to Slave */
	SPT_TAG_RD	= 0x00, /* Master READ from Slave */
	SPT_TAG_ACK	= 0xBE, /* Slave  Acknowledgement */
	SPT_TAG_DMY	= 0xFF, /* dummy */

	SPT_ERR_OK	= 0x00,
};

static bool spi_wait_dir(SPI_DIR dir, int loop_wait)
{
    const auto loop_wait_tick = loop_wait * 100;

    for (int i = 0; digitalRead(ARDUINO_SPI_PIN_DIR) != dir; ++i)
    {
        delayMicroseconds(10);
        if (i > loop_wait_tick) return false;
    }

    return true;
}

static inline bool spi_exist_data()
{
    return digitalRead(ARDUINO_SPI_PIN_EXIST_DATA) == HIGH;
}

static inline void spi_cs(bool active)
{
    digitalWrite(ARDUINO_SPI_PIN_CS, active ? LOW : HIGH);
}

static inline int spi_transfer(uint8_t v)
{
    return ARDUINO_SPI_TO_ESPAT.transfer(v);
}

static inline int spi_transfer8_8(uint8_t v, uint8_t v2)
{
    uint16_t r;

    r  = spi_transfer(v) << 8;
    r |= spi_transfer(v2);

    return r;
}

static inline int spi_transfer16(uint16_t v)
{
    uint16_t r;

    r  = spi_transfer(v >> 8) << 8;
    r |= spi_transfer(v & 0xFF);

    return r;
}

int at_spi_write(const uint8_t* buf, uint16_t len, int loop_wait)
{
    uint8_t v;

    if (!spi_wait_dir(SPI_DIR_MOSI, loop_wait)) return -2000;

    spi_cs(true);
    spi_transfer8_8(SPT_TAG_PRE, SPT_TAG_WR);
    spi_transfer16(len);
    spi_cs(false);

    if (!spi_wait_dir(SPI_DIR_MISO, loop_wait)) return -2001;

    spi_cs(true);
    v = spi_transfer(SPT_TAG_DMY);
    if (v != SPT_TAG_ACK)
    {
        Serial.printf("No ACK, %#04X in WR\r\n", v);    // TODO
        spi_cs(false);
        return -1;
    }
    v = spi_transfer(SPT_TAG_DMY);
    if (v != SPT_ERR_OK)
    {
        spi_cs(false);
        return -1000 - v; /* device not ready */
    }
    len = spi_transfer8_8(SPT_TAG_DMY, SPT_TAG_DMY);
    spi_cs(false);

    if (!spi_wait_dir(SPI_DIR_MOSI, loop_wait)) return -2002;

    spi_cs(true);
    if (len) for (int i = 0; i < len; ++i) spi_transfer(buf[i]);
    spi_cs(false);

    return len; /* success transfer len bytes */
}

int at_spi_read(uint8_t* buf, uint16_t len, int loop_wait)
{
    if (!spi_exist_data()) return 0;

    uint8_t v;

    if (!spi_wait_dir(SPI_DIR_MOSI, loop_wait)) return -2000;

    spi_cs(true);
    spi_transfer8_8(SPT_TAG_PRE, SPT_TAG_RD);
    spi_transfer16(len);
    spi_cs(false);

    if (!spi_wait_dir(SPI_DIR_MISO, loop_wait)) return -2001;

    spi_cs(true);
    v = spi_transfer(SPT_TAG_DMY);
    if (v != SPT_TAG_ACK)
    {
        Serial.printf("No ACK, %#04X in RD\r\n", v);    // TODO
        spi_cs(false);
        return -1;
    }
    v = spi_transfer(SPT_TAG_DMY);
    if (v != SPT_ERR_OK)
    {
        spi_cs(false);
        return -1000 - v; /* device not ready */
    }
    len = spi_transfer8_8(SPT_TAG_DMY, SPT_TAG_DMY);
    if (len) for (int i = 0; i < len; ++i) buf[i] = spi_transfer(SPT_TAG_DMY);
    spi_cs(false);

    return len; /* success transfer len bytes */
}

int at_spi_begin(void)
{
    // Reset SPI slave device(RTL8720D)
    pinMode(RTL8720D_CHIP_PU, OUTPUT);
    digitalWrite(RTL8720D_CHIP_PU, LOW);

    // start the SPI library:
    ARDUINO_SPI_TO_ESPAT.begin();
    // Start SPI transaction at a quarter of the MAX frequency
    ARDUINO_SPI_TO_ESPAT.beginTransaction(SPISettings(MAX_SPI / 4, MSBFIRST, SPI_MODE0));

    // initalize the  data ready and chip select pins:
    pinMode(ARDUINO_SPI_PIN_DIR, INPUT);
    pinMode(ARDUINO_SPI_PIN_EXIST_DATA, INPUT);
    pinMode(ARDUINO_SPI_PIN_CS, OUTPUT);
    digitalWrite(ARDUINO_SPI_PIN_CS, HIGH);

    // When RTL8720D startup, set pin UART_LOG_TXD to lowlevel
    // will force the device enter UARTBURN mode.
    // Explicit high level will prevent above things.
    pinMode(PIN_SERIAL2_RX, OUTPUT);
    digitalWrite(PIN_SERIAL2_RX, HIGH);

    // reset duration
    delay(20);

    // Release RTL8720D reset, start bootup.
    digitalWrite(RTL8720D_CHIP_PU, HIGH);

    // give the slave time to set up
    delay(500);

    pinMode(PIN_SERIAL2_RX, INPUT);

    // Receive "\r\nready\r\n"
    uint8_t buf[strlen(RESPONSE_READY) + 10];
    int bufLen = at_spi_read(buf, sizeof(buf) - 1);
    if (bufLen != strlen(RESPONSE_READY)) return 1;
    if (memcmp(buf, RESPONSE_READY, bufLen) != 0) return 2;

    return 0;
}
