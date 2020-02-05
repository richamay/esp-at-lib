#include "station_manager.h"

#ifdef printf
  #undef printf
#endif
static espr_t conn_callback_func(esp_evt_t* evt);

/**
 * \brief           Program entry point
 */
void
setup(void) {
    espr_t res;

    Serial.printf("Application running on SAMD51!\r\n");

    /*
     * Connect to access point.
     *
     * Try unlimited time until access point accepts up.
     * Check for station_manager.c to define preferred access points ESP should connect to
     */
    connect_to_preferred_access_point(1);

    /* Start a new connection as client in non-blocking mode */
    if ((res = esp_conn_start(NULL, ESP_CONN_TYPE_TCP, "mirrors.tuna.tsinghua.edu.cn", 80, NULL, conn_callback_func, 0)) == espOK) {
        Serial.printf("Connection to tsinghua.edu.cn started...\r\n");
    } else {
        Serial.printf("Cannot start connection to example.com!\r\n");
    }
}

void loop(void) {
    return;
}

/**
 * \brief           Request data for connection
 */
static const
uint8_t req_data[] = ""
"GET /debian HTTP/1.1\r\n"
"Host: mirrors.tuna.tsinghua.edu.cn\r\n"
"Connection: close\r\n"
"\r\n";

/**
 * \brief           Event callback function for connection-only
 * \param[in]       evt: Event information with data
 * \return          espOK on success, member of \ref espr_t otherwise
 */
static espr_t
conn_callback_func(esp_evt_t* evt) {
    esp_conn_p conn;
    espr_t res;

    conn = esp_conn_get_from_evt(evt);          /* Get connection handle from event */
    if (conn == NULL) {
        return espERR;
    }
    switch (esp_evt_get_type(evt)) {
        case ESP_EVT_CONN_ACTIVE: {             /* Connection just active */
            Serial.printf("Connection active!\r\n");
            res = esp_conn_send(conn, req_data, sizeof(req_data) - 1, NULL, 0); /* Start sending data in non-blocking mode */
            if (res == espOK) {
                Serial.printf("Sending request data to server...\r\n");
            } else {
                Serial.printf("Cannot send request data to server. Closing connection manually...\r\n");
                esp_conn_close(conn, 0);        /* Close the connection */
            }
            break;
        }
        case ESP_EVT_CONN_CLOSE: {              /* Connection closed */
            if (esp_evt_conn_close_is_forced(evt)) {
                Serial.printf("Connection closed by client!\r\n");
            } else {
                Serial.printf("Connection closed by remote side!\r\n");
            }
            break;
        }
        case ESP_EVT_CONN_SEND: {               /* Data send event */
            espr_t res = esp_evt_conn_send_get_result(evt);
            if (res == espOK) {
                Serial.printf("Data sent successfully...waiting to receive data from remote side...\r\n");
            } else {
                Serial.printf("Error while sending data!\r\n");
            }
            break;
        }
        case ESP_EVT_CONN_RECV: {               /* Data received from remote side */
            esp_pbuf_p pbuf = esp_evt_conn_recv_get_buff(evt);
            esp_conn_recved(conn, pbuf);        /* Notify stack about received pbuf */
            Serial.printf("Received %d bytes on connection..\r\n", (int)esp_pbuf_length(pbuf, 1));
            {
            char* d = (char*)esp_pbuf_data(pbuf);
            char* s = strchr(d, '\n');
            if (s) {
              *s = '\0';
              Serial.printf("The first line: %s\n", d);
            }
            }
            break;
        }
        default: break;
    }
    return espOK;
}
