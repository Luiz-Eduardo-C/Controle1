#define I2C_SDA 21
#define I2C_SCL 22
/*Olá celeste*/
#include <SPI.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Configuração do divisor de tensão
const int pinADC = 34;  // Pino conectado ao divisor
float fatorDivisor = 3.5;  // Fator do divisor de tensão

unsigned long startTime;

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(1); }

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  // Inicializa o tempo inicial para medir o tempo decorrido
  startTime = millis();

  // Serial.println("Iniciando leitura...");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);


    // Variável para tensão medida pelo divisor
  float tensaoEntrada = 0.0;

  if (measure.RangeStatus != 4) {
    // Lê a altura em mm
    int altura = measure.RangeMilliMeter-30;
    
    // Calcula o tempo decorrido
    unsigned long currentTime = (millis() - startTime)/1000;

      // Leitura do ADC e cálculo da tensão no divisor
    int leituraADC = analogRead(pinADC);
    tensaoEntrada = (leituraADC / 4095.0) * 3.3 * fatorDivisor;

    // Mapeia a altura para o valor do DAC (inverte o mapeamento)
    int valor_dac = map(altura, 250, 0, 0, 255); // Note que os limites de entrada foram invertidos

    // Garante que o valor do DAC seja pelo menos zero
    if (valor_dac < 0) {
      valor_dac = 0;
    }

    float tensao = (valor_dac / 255.0) * 3.3;
    
    
    // Printa o tempo, tensão e altura em colunas
    Serial.print(currentTime);
    Serial.print(" ");
    Serial.print(tensaoEntrada, 2);  // Limita a tensão a duas casas decimais
    Serial.print(" ");
    Serial.print(tensao); // Limita a tensão a duas casas decimais
    Serial.print(" ");
    Serial.println(altura);


    // Código para gerar saída analógica (comentado)
    
    // Mapear a altura (0-2000 mm) para o valor do DAC (0-255)
    //int valor_dac = map(altura, 0, 200, 0, 255);
    
    // Limitar o valor do DAC entre 0 e 255
    valor_dac = constrain(valor_dac, 0, 255);

    // Enviar o valor mapeado para o DAC (GPIO25)
    dacWrite(25, valor_dac);

    // Imprimir a tensão correspondente no DAC (comentado)
    // float tensao = (valor_dac / 255.0) * 3.3;
    // Serial.print("Tensão DAC (V): ");
    // Serial.println(tensao);
  
  } else {
    Serial.println("Fora de alcance");
  }

  delay(500);  // Aguarde 1 segundo para a próxima leitura
}
