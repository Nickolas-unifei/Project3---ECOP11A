#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

float modbus_read(int serial, char *cmd, float div) // Subrotina
{
    static unsigned char resposta[10];
    int raw = 0;
    float saida = 0.0f;
    size_t bytes_escritos = write(serial, cmd, sizeof(cmd));

    usleep(100 * 1000); // 100 ms

    size_t bytes_lidos = read(serial, resposta, 8);
    if (bytes_lidos > 4)
    {
        raw = (resposta[3] << 8) + resposta[4];
        return raw / div;
    }
    else
        return -1;
}

int main()
{
    // Abrindo porta serial do ESP32
    printf("Configurando porta Serial:\n");
    char *portname = "/dev/ttyUSB0"; 
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erro ao abrir a porta serial");
        return 1;
    }
    // Configurando comunicação da porta serial do ESP32
    struct termios tty;
    memset(&tty, 0, sizeof(tty)); // função "memset"
    if (tcgetattr(fd, &tty) != 0) {
        perror("Erro ao obter os atributos da porta serial");
        return 1;
    }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    //tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Erro ao configurar a porta serial");
        return 1;
    }

    // transmissao da porta serial em MODBUS
    int bytes_escritos = 0;
    int bytes_lidos = 0;
    char resposta[10];

    // comandos modbus para cada medida
    // ID_SLAVE, CMD_Leitura (0x03), Med_H, Med_L, SE_H, SE_L, CRC_L, CRC_H}
    char temp[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
    char umid[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
    char pres[] = {0x01, 0x03, 0x00, 0x62, 0x00, 0x01, 0x25, 0xD4};

    float valor = 0.0f;

    while (1) // repete infinitamente
    {   

        valor = modbus_read(fd,temp,10.0f);
        if (valor >= 0.0f)
            printf("Temp: %.1f\n", valor);
        else
            printf("Temp: Erro\n");

        usleep(100 * 1000); //100 ms

        valor = modbus_read(fd,umid,100.0f);
        if (valor >= 0.0f)
            printf("Umid: %.2f\n", valor);
        else
            printf("Umid: Erro\n");
        
        usleep(100 * 1000); //100 ms

        valor = modbus_read(fd,pres,100.0f);
        if (valor >= 0.0f)
            printf("Pres: %.2f\n", valor);
        else
            printf("Pres: Erro\n");

        usleep(500 * 1000); //500 ms
    }

    close(fd);

    printf("Terminado\n");
    return 0;
}