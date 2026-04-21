/*
    Firmware de teste de hardware - PolArStatLite

    Este firmware executa quatro testes principais:
    1) Inicio de escala (MCP = 0)
    2) Meio de escala (MCP = 2047)
    3) Fim de escala (MCP = 4095)
    4) Sweep completo do MCP

    Em todos os testes, o firmware escreve no MCP4725 e faz leituras nos canais
    ADS1 e ADS3 do ADS1115.

    Como rodar pela Arduino IDE:
    - Fazer upload deste firmware
    - Abrir o Serial Monitor em 115200 baud
    - Enviar os comandos abaixo:
        0 -> teste com MCP = 0
        1 -> teste com MCP = 2047
        2 -> teste com MCP = 4095
        3 -> sweep do MCP de 0 ate 4095, em passos de 10

    Observacao:
    - Os comandos acima sao caracteres ASCII enviados no Serial Monitor.
        Ex.: 0 corresponde ao byte 0x30.
*/

#include "Wire.h"
#include "ADS1X15.h"
#include "MCP4725.h"

MCP4725 MCP(0x60);
ADS1115 ADS(0x48);

void executeMCPSweep() {
    digitalWrite(2, HIGH);
    digitalWrite(5, HIGH);

    int16_t ads1, ads3;

    int dacCode = 0;
    int step = 50;

    while (1) {
        MCP.setValue(dacCode);
        delay(100);

        ads1 = ADS.readADC(1);
        ads3 = ADS.readADC(3);
        if (ads3 < 0) ads3 = 0;

        Serial.print("MCP=");
        Serial.print(dacCode);
        Serial.print(" ADS3=");
        Serial.print(ads3);

        Serial.print(" ADS1=");
        Serial.println(ads1);

        if (dacCode == 4095) break;

        if (dacCode + step > 4095) {
            dacCode = 4095;
        } else {
            dacCode += step;
        }
    }

    digitalWrite(2, LOW);
    digitalWrite(5, LOW);
}

void executeStep(int dacCode) {
    MCP.setValue(dacCode);
    digitalWrite(2, HIGH);
    digitalWrite(5, HIGH);
    // delay stabilization
    delay(100);

    for (unsigned int i=0;i<10;i++){
      int16_t ads1 = ADS.readADC(1);
      int16_t ads3 = ADS.readADC(3);
      if (ads3 < 0) ads3 = 0;

      Serial.print("MCP=");
      Serial.print(dacCode);
      Serial.print(" ADS3=");
      Serial.print(ads3);

      Serial.print(" ADS1=");
      Serial.println(ads1);
      delay(200);
    }
    digitalWrite(2, LOW);
    digitalWrite(5, LOW);
}

void setup() {
    // Keep the same base setup used in the original firmware.
    Serial.begin(115200);
    TCCR1B = TCCR1B & B11111000 | B00000001;
    Wire.begin();
    Wire.setClock(800000);
    MCP.begin();
    MCP.setValue(0);
    ADS.begin();
    ADS.setGain(1);
    ADS.setDataRate(7);

    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);

    Serial.println("HardwareTestReady");
}

void loop() {
    while (!Serial.available()) {
    }

    int incoming = Serial.read();
    if (incoming < 0) {
        return;
    }

    uint8_t command = (uint8_t)incoming;

    // Drain any stale bytes to keep command framing clean.
    while (Serial.available() > 0) {
        Serial.read();
    }

    switch (command) {
        case 0x30:
            // Start of scale: write 0 and read ADS1/ADS3.
            executeStep(0);
            break;
        case 0x31:
            // Mid-scale: write 2047 and read ADS1/ADS3.
            executeStep(2047);
            break;
        case 0x32:
            // End of scale: write 4095 and read ADS1/ADS3.
            executeStep(4095);
            break;
        case 0x33:
            // Local sweep on Arduino: MCP from 0 to 4095.
            executeMCPSweep();
            break;
        default:
            Serial.print("UNKNOWN_CMD=0x");
            Serial.println(command, HEX);
            break;
    }
}
