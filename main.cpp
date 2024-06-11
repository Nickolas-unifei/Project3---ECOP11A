#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <modbus.h>
#include <microhttpd.h>

#define PORT 8888
#define MAX_DATA_POINTS 100

typedef struct {
    float temperature[MAX_DATA_POINTS];
    float humidity[MAX_DATA_POINTS];
    float pressure[MAX_DATA_POINTS];
    int count;
} SensorData;

SensorData data;
pthread_mutex_t data_mutex;

int configure_serial_port(const char *portname);
int read_modbus_data(modbus_t *ctx, uint16_t address, float *value, float scale);
void update_sensor_data();
int answer_to_connection(void *cls, struct MHD_Connection *connection,
                         const char *url, const char *method, const char *version,
                         const char *upload_data, size_t *upload_data_size, void **con_cls);

int main() {
    pthread_t sensor_thread;
    pthread_mutex_init(&data_mutex, NULL);
    data.count = 0;

    // Start sensor data update thread
    pthread_create(&sensor_thread, NULL, (void *)update_sensor_data, NULL);

    // Start web server
    struct MHD_Daemon *daemon;
    daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, PORT, NULL, NULL,
                              &answer_to_connection, NULL, MHD_OPTION_END);
    if (NULL == daemon) {
        return 1;
    }

    printf("Web server running on port %d\n", PORT);
    getchar();  // Keep the server running

    MHD_stop_daemon(daemon);
    pthread_mutex_destroy(&data_mutex);
    return 0;
}

void update_sensor_data() {
    const char *portname = "/dev/ttyUSB0";
    modbus_t *ctx = modbus_new_rtu(portname, 9600, 'N', 8, 1);
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return;
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return;
    }

    const uint16_t temp_address = 0x0000;
    const uint16_t umid_address = 0x0001;
    const uint16_t pres_address = 0x0002;

    while (1) {
        float temp, umid, pres;

        if (read_modbus_data(ctx, temp_address, &temp, 10.0f) == 0 &&
            read_modbus_data(ctx, umid_address, &umid, 1000.0f) == 0 &&
            read_modbus_data(ctx, pres_address, &pres, 1000.0f) == 0) {

            pthread_mutex_lock(&data_mutex);
            if (data.count < MAX_DATA_POINTS) {
                data.temperature[data.count] = temp;
                data.humidity[data.count] = umid;
                data.pressure[data.count] = pres;
                data.count++;
            } else {
                memmove(data.temperature, data.temperature + 1, (MAX_DATA_POINTS - 1) * sizeof(float));
                memmove(data.humidity, data.humidity + 1, (MAX_DATA_POINTS - 1) * sizeof(float));
                memmove(data.pressure, data.pressure + 1, (MAX_DATA_POINTS - 1) * sizeof(float));
                data.temperature[MAX_DATA_POINTS - 1] = temp;
                data.humidity[MAX_DATA_POINTS - 1] = umid;
                data.pressure[MAX_DATA_POINTS - 1] = pres;
            }
            pthread_mutex_unlock(&data_mutex);
        }

        sleep(1);
    }

    modbus_close(ctx);
    modbus_free(ctx);
}

int read_modbus_data(modbus_t *ctx, uint16_t address, float *value, float scale) {
    uint16_t reg;
    if (modbus_read_registers(ctx, address, 1, &reg) == -1) {
        fprintf(stderr, "Modbus read failed: %s\n", modbus_strerror(errno));
        return -1;
    }
    *value = reg / scale;
    return 0;
}

int answer_to_connection(void *cls, struct MHD_Connection *connection,
                         const char *url, const char *method, const char *version,
                         const char *upload_data, size_t *upload_data_size, void **con_cls) {
    static const char *page = "<html><body><h1>Sensor Data</h1>"
                              "<p>Temperature: %s</p>"
                              "<p>Humidity: %s</p>"
                              "<p>Pressure: %s</p>"
                              "</body></html>";

    char temp_str[512];
    char umid_str[512];
    char pres_str[512];

    pthread_mutex_lock(&data_mutex);
    int count = data.count;
    snprintf(temp_str, sizeof(temp_str), "[");
    snprintf(umid_str, sizeof(umid_str), "[");
    snprintf(pres_str, sizeof(pres_str), "[");

    for (int i = 0; i < count; i++) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%.2f", data.temperature[i]);
        strncat(temp_str, buffer, sizeof(temp_str) - strlen(temp_str) - 1);
        if (i < count - 1) {
            strncat(temp_str, ", ", sizeof(temp_str) - strlen(temp_str) - 1);
        }

        snprintf(buffer, sizeof(buffer), "%.3f", data.humidity[i]);
        strncat(umid_str, buffer, sizeof(umid_str) - strlen(umid_str) - 1);
        if (i < count - 1) {
            strncat(umid_str, ", ", sizeof(umid_str) - strlen(umid_str) - 1);
        }

        snprintf(buffer, sizeof(buffer), "%.3f", data.pressure[i]);
        strncat(pres_str, buffer, sizeof(pres_str) - strlen(pres_str) - 1);
        if (i < count - 1) {
            strncat(pres_str, ", ", sizeof(pres_str) - strlen(pres_str) - 1);
        }
    }

    strncat(temp_str, "]", sizeof(temp_str) - strlen(temp_str) - 1);
    strncat(umid_str, "]", sizeof(umid_str) - strlen(umid_str) - 1);
    strncat(pres_str, "]", sizeof(pres_str) - strlen(pres_str) - 1);
    pthread_mutex_unlock(&data_mutex);

    char buffer[2048];
    snprintf(buffer, sizeof(buffer), page, temp_str, umid_str, pres_str);

    struct MHD_Response *response;
    int ret;

    response = MHD_create_response_from_buffer(strlen(buffer), (void *)buffer, MHD_RESPMEM_PERSISTENT);
    ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
                         }
    return 0;
}
