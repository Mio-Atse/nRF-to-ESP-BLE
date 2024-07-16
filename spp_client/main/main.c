/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_spp_client.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define PEER_ADDR_VAL_SIZE      6
#define BLE_SVC_UUID                    0x0001
#define BLE_CHR_UUID                    0x0002
#define BLE_GATT_MTU_SIZE 23  // Default BLE MTU size

static const char* TAG = "NVS_Manager";

static const char *tag = "NimBLE_SPP_BLE_CENT";
static int ble_spp_client_gap_event(struct ble_gap_event *event, void *arg);
QueueHandle_t spp_common_uart_queue = NULL;
void ble_store_config_init(void);
uint16_t attribute_handle[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];
static void ble_spp_client_scan(void);
static ble_addr_t connected_addr[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

static void
ble_spp_client_set_handle(const struct peer *peer)
{
    const struct peer_chr *chr;
    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(GATT_SPP_SVC_UUID),
                             BLE_UUID16_DECLARE(GATT_SPP_CHR_UUID));
    attribute_handle[peer->conn_handle] = chr->chr.val_handle;
}

/**
 * Called when service discovery of the specified peer has completed.
 */


esp_err_t save_sensor_data(float temperature, float humidity) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(my_handle, "temperature", &temperature, sizeof(float));
    if (err == ESP_OK) {
        err = nvs_set_blob(my_handle, "humidity", &humidity, sizeof(float));
    }

    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }

    nvs_close(my_handle);
    return err;
}

esp_err_t load_sensor_data(float *temperature, float *humidity) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(float);
    err = nvs_get_blob(my_handle, "temperature", temperature, &required_size);
    if (err == ESP_OK) {
        err = nvs_get_blob(my_handle, "humidity", humidity, &required_size);
    }

    nvs_close(my_handle);
    return err;
}

esp_err_t save_accel_data(int16_t x, int16_t y, int16_t z) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } accel_data = {x, y, z};

    err = nvs_set_blob(my_handle, "accel_data", &accel_data, sizeof(accel_data));
    
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }

    nvs_close(my_handle);
    return err;
}

esp_err_t load_accel_data(int16_t *x, int16_t *y, int16_t *z) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } accel_data;

    size_t required_size = sizeof(accel_data);
    err = nvs_get_blob(my_handle, "accel_data", &accel_data, &required_size);

    if (err == ESP_OK) {
        *x = accel_data.x;
        *y = accel_data.y;
        *z = accel_data.z;
    }

    nvs_close(my_handle);
    return err;
}

static void
ble_spp_client_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    if (status != 0) {
        MODLOG_DFLT(ERROR, "Error: Service discovery failed; status=%d "
                    "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    MODLOG_DFLT(INFO, "Service discovery complete; status=%d "
                "conn_handle=%d\n", status, peer->conn_handle);

    const struct peer_svc *svc;
    const struct peer_chr *chr;

    const ble_uuid128_t nus_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

    svc = peer_svc_find_uuid(peer, &nus_uuid.u);
    if (svc == NULL) {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the Nordic UART Service\n");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    // Nordic UART Service RX characteristic UUID
    const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

    chr = peer_chr_find_uuid(peer, &nus_uuid.u, &nus_rx_uuid.u);
    if (chr == NULL) {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the Nordic UART RX Characteristic\n");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    attribute_handle[peer->conn_handle] = chr->chr.val_handle;

    // Nordic UART Service TX characteristic UUID
    const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

    chr = peer_chr_find_uuid(peer, &nus_uuid.u, &nus_tx_uuid.u);
    if (chr == NULL) {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the Nordic UART TX Characteristic\n");
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    // Subscribe to notifications for the TX characteristic
    uint8_t value[2] = {0x01, 0x00};
    ble_gattc_write_flat(peer->conn_handle, chr->chr.val_handle + 1, value, sizeof(value), NULL, NULL);

    MODLOG_DFLT(INFO, "Successfully subscribed to notifications\n");
}
    

/**
 * Initiates the GAP general discovery procedure.
 */
static void
ble_spp_client_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      ble_spp_client_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
static int
ble_spp_client_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return 0;
    }

    /* Check if the device name matches */
    if (fields.name != NULL && fields.name_len == strlen("NRF52_SPP_Server") &&
        memcmp(fields.name, "NRF52_SPP_Server", fields.name_len) == 0) {
        return 1;
    }

    // If we reach here, it means we didn't find a matching device
    return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
ble_spp_client_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    uint8_t own_addr_type;
    int rc;

    /* Don't do anything if we don't care about this advertiser. */
    if (!ble_spp_client_should_connect(disc)) {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */

    rc = ble_gap_connect(own_addr_type, &disc->addr, 30000, NULL,
                         ble_spp_client_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "Error: Failed to connect to device; addr_type=%d "
                    "addr=%s; rc=%d\n",
                    disc->addr.type, addr_str(disc->addr.val), rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  ble_spp_client uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_client.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
ble_spp_client_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        /* An advertisment report was received during GAP discovery. */
        print_adv_fields(&fields);

        /* Try to connect to the advertiser if it looks interesting. */
        ble_spp_client_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            /* Connection successfully established. */
            MODLOG_DFLT(INFO, "Connection established ");
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            memcpy(&connected_addr[event->connect.conn_handle].val, desc.peer_id_addr.val,
                   PEER_ADDR_VAL_SIZE);
            print_conn_desc(&desc);
            MODLOG_DFLT(INFO, "\n");

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               ble_spp_client_on_disc_complete, NULL);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
                return 0;
            }
        } else {
            /* Connection attempt failed; resume scanning. */
            MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
            ble_spp_client_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        /* Forget about peer. */
        memset(&connected_addr[event->disconnect.conn.conn_handle].val, 0, PEER_ADDR_VAL_SIZE);
        attribute_handle[event->disconnect.conn.conn_handle] = 0;
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        ble_spp_client_scan();
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
                    event->disc_complete.reason);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
    MODLOG_DFLT(INFO, "Received data from server\n");
    
    // Convert hex data to ASCII
    char *received_data = calloc(event->notify_rx.om->om_len + 1, sizeof(char));
    for (int i = 0; i < event->notify_rx.om->om_len; i += 2) {
        char hex[3] = {0};
        memcpy(hex, &event->notify_rx.om->om_data[i], 2);
        received_data[i/2] = (char)strtol(hex, NULL, 16);
    }
    
    MODLOG_DFLT(INFO, "Data (hex): %.*s", event->notify_rx.om->om_len, event->notify_rx.om->om_data);
    MODLOG_DFLT(INFO, "Data (ASCII): %s", received_data);
    
    // Parse and display the data
    if (strncmp(received_data, "T:", 2) == 0) {
        float temp, hum;
        sscanf(received_data, "T:%f,H:%f", &temp, &hum);
        MODLOG_DFLT(INFO, "Temperature: %.1f, Humidity: %.1f\n", temp, hum);
        save_sensor_data(temp, hum);
    } else if (strncmp(received_data, "X:", 2) == 0) {
        int x, y, z;
        sscanf(received_data, "X:%d,Y:%d,Z:%d", &x, &y, &z);
        MODLOG_DFLT(INFO, "Accelerometer - X: %d, Y: %d, Z: %d\n", x, y, z);
        save_accel_data(x, y, z);
    }
    
    // Write hex data to UART
    uart_write_bytes(UART_NUM_0, event->notify_rx.om->om_data, event->notify_rx.om->om_len);
    uart_write_bytes(UART_NUM_0, "\n", 1);
    
    free(received_data);
    break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    default:
        return 0;
    }
    return 0;
}

static void
ble_spp_client_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_spp_client_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin scanning for a peripheral to connect to. */
    ble_spp_client_scan();
}

void ble_spp_client_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
static void send_data_to_server(uint8_t *data, uint16_t length)
{
    int rc;
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        if (attribute_handle[i] != 0) {
            rc = ble_gattc_write_flat(i, attribute_handle[i], data, length, NULL, NULL);
            if (rc == 0) {
                ESP_LOGI(tag, "Command sent to server successfully!");
            } else {
                ESP_LOGI(tag, "Error in sending command to server rc=%d", rc);
            }
        }
    }
}



void send_test_data(void)
{
    static uint8_t test_data[] = "Hello from ESP32!";
    send_data_to_server(test_data, sizeof(test_data) - 1);  // -1 to exclude null terminator
}

void ble_client_uart_task(void *pvParameters)
{
    ESP_LOGI(tag, "BLE client UART task started");
    uart_event_t event;
    uint8_t command[2] = {0};  // Buffer for the command

    for (;;) {
        if (xQueueReceive(spp_common_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA:
                if (event.size) {
                    uart_read_bytes(UART_NUM_0, command, event.size, portMAX_DELAY);
                    
                    // Check if the command is valid (31 or 32 in hex)
                    if (command[0] == '3' && (command[1] == '1' || command[1] == '2')) {
                        ESP_LOGI(tag, "Sending command: %c%c", command[0], command[1]);
                        send_data_to_server(command, 2);
                    } else {
                        ESP_LOGI(tag, "Invalid command. Please enter 31 or 32 in hex.");
                    }
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}
static void ble_spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_common_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(ble_client_uart_task, "uTask", 4096, (void *)UART_NUM_0, 8, NULL);
}


void
app_main(void)
{   
    
    int rc;
    /* Initialize NVS — it is used to store PHY calibration data */
   // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Print memory info
    
    // Load and print sensor data
    float temp, hum;
    esp_err_t err = load_sensor_data(&temp, &hum);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded sensor data - Temperature: %.2f, Humidity: %.2f", temp, hum);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No sensor data found in NVS");
    } else {
        ESP_LOGE(TAG, "Error loading sensor data: %s", esp_err_to_name(err));
    }

    // Load and print accelerometer data
    int16_t x, y, z;
    err = load_accel_data(&x, &y, &z);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded accelerometer data - X: %d, Y: %d, Z: %d", x, y, z);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No accelerometer data found in NVS");
    } else {
        ESP_LOGE(TAG, "Error loading accelerometer data: %s", esp_err_to_name(err));
    }

    // Print memory info again

    // Delay BLE initialization
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Initialize BLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return;
    }
    

    /* Initialize UART driver and start uart task */
    ble_spp_uart_init();

    /* Configure the host. */
    ble_hs_cfg.reset_cb = ble_spp_client_on_reset;
    ble_hs_cfg.sync_cb = ble_spp_client_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-ble-spp-client");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(ble_spp_client_host_task);
    
}
