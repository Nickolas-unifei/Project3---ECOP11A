#include <Arduino.h>
#include <ModbusRTU.h>

ModbusRTU mb;

// Definição dos endereços MODBUS para os registros
const uint16_t TEMP_REG = 0x01;
const uint16_t HUM_REG = 0x00;
const uint16_t PRESS_REG = 0x62;

void setup() {
  Serial.begin(9600, SERIAL_8N1); // Inicializa a comunicação serial a 9600 bps
  mb.begin(&Serial); // Inicializa a comunicação MODBUS a 9600 bps
  mb.slave(1); // Define o dispositivo MODBUS como escravo 1
  mb.addHreg(TEMP_REG); // Adiciona registro de temperatura
  mb.addHreg(HUM_REG); // Adiciona registro de umidade
  mb.addHreg(PRESS_REG); // Adiciona registro de pressão
}

long int tempo = 0;

void loop() {
  if (millis() - tempo > 1000) // atual - ultimaAtualizacao > 1000 ms
  {
    int temperatura = random(0, 401);
    int umidade = random(0, 101);
    int pressao = random(0, 201);

    // Envia os valores via MODBUS
    mb.Hreg(TEMP_REG, temperatura);
    mb.Hreg(HUM_REG, umidade);
    mb.Hreg(PRESS_REG, pressao);

    tempo = millis(); // salva o instante atual
  }

  mb.task();
  yield();
}

