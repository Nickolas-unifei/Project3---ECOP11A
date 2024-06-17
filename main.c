#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>  /* Standard library definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/socket.h> /* Socket definitions */
#include <netinet/in.h> /* Internet address definitions */
#include <arpa/inet.h> /* ARPA Internet address definitions */
#include <sys/select.h> /* POSIX select definitions */
#include <time.h>

#define PORT 8086
#define BUFFER_SIZE 2048
#define NUM_MEASUREMENTS 10

typedef struct {
    float tempVal;
    float umidVal;
    float presVal;
} SensorData;

SensorData sensorData[NUM_MEASUREMENTS];
int currentIndex = 0;

// Função para configurar a porta serial
int configure_serial_port(char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erro ao abrir a porta serial");
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("Erro ao obter os atributos da porta serial");
        return -1;
    }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Erro ao configurar a porta serial");
        return -1;
    }
    return fd;
}

// Função para ler os dados da porta serial
void read_serial_data(int fd) {
    unsigned char temp[8] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
    unsigned char umid[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
    unsigned char pres[8] = {0x01, 0x03, 0x00, 0x62, 0x00, 0x01, 0x25, 0xD4};
    unsigned char response[10];

    // Requisita a temperatura
    write(fd, temp, sizeof(temp));
    usleep(200 * 1000);
    int bytes_read = read(fd, response, 8);
    if (bytes_read > 4) {
        sensorData[currentIndex].tempVal = (int)((response[3] << 8) + response[4]) / 10.0f;
    }

    // Requisita umidade
    write(fd, umid, sizeof(umid));
    usleep(200 * 1000);
    bytes_read = read(fd, response, 8);
    if (bytes_read > 4) {
        sensorData[currentIndex].umidVal = (int)((response[3] << 8) + response[4]) / 100.0f;
    }

    // Requisita pressão
    write(fd, pres, sizeof(pres));
    usleep(200 * 1000);
    bytes_read = read(fd, response, 8);
    if (bytes_read > 4) {
        sensorData[currentIndex].presVal = (int)((response[3] << 8) + response[4]) / 100.0f;
    }

    currentIndex = (currentIndex + 1) % NUM_MEASUREMENTS;
}

// Função para lidar com requisições HTTP
void handle_request(int client_socket) {

    time_t currentTime;
    time(&currentTime);

    struct tm *localTime;
    localTime = localtime(&currentTime);

     int hour = localTime->tm_hour;     
     int min = localTime->tm_min;     
     int sec = localTime->tm_sec;     
     int timee = hour * 10000 + min * 100 + sec;

    char buffer[BUFFER_SIZE];
    recv(client_socket, buffer, BUFFER_SIZE, 0);

    char response[BUFFER_SIZE];
    int length = snprintf(response, sizeof(response),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n"
             "Connection: close\r\n"
             "\r\n"
             "<html><body><h1>Dados do Sensor</h1>");

    length += snprintf(response + length, sizeof(response) - length,
             "<table border='1'><tr><th>Índice</th><th>Temperatura (&deg;C)</th><th>Umidade (%)</th><th>Pressao (hPa)</th></tr>");

    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        int index = (currentIndex + i) % NUM_MEASUREMENTS;
        length += snprintf(response + length, sizeof(response) - length,
                           "<tr><td>%d</td><td>%.1f</td><td>%.2f</td><td>%.2f</td></tr>",
                           timee, sensorData[index].tempVal, sensorData[index].umidVal, sensorData[index].presVal);
    }

    length += snprintf(response + length, sizeof(response) - length,
             "</table>"
             "<script>"
             "setTimeout(function() { location.reload(); }, 1000);"
             "</script>"
             "</body></html>");

    send(client_socket, response, length, 0);
    close(client_socket);
}

// Função para iniciar o servidor web
void start_server(int serial_fd) {
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        perror("Erro ao criar o socket do servidor");
        exit(1);
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Erro ao fazer o bind do socket");
        exit(1);
    }

    if (listen(server_socket, 3) < 0) {
        perror("Erro ao ouvir no socket");
        exit(1);
    }

    printf("Servidor iniciado na porta %d...\n", PORT);

    while (1) {
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(server_socket, &readfds);
        int max_fd = server_socket;

        int activity = select(max_fd + 1, &readfds, NULL, NULL, &timeout);

        if (activity < 0 && errno != EINTR) {
            perror("Erro na função select");
        }

        if (activity > 0) {
            if (FD_ISSET(server_socket, &readfds)) {
                int client_socket = accept(server_socket, NULL, NULL);
                if (client_socket < 0) {
                    perror("Erro ao aceitar conexão");
                    continue;
                }
                handle_request(client_socket);
            }
        }

        read_serial_data(serial_fd);
    }

    close(server_socket);
}

int main() {
    printf("Configurando porta Serial:\n");
    char *portname = "/dev/ttyUSB0";
    int serial_fd = configure_serial_port(portname);
    if (serial_fd < 0) {
        return 1;
    }

    start_server(serial_fd);

    close(serial_fd);
    printf("Terminado\n");
    return 0;
}