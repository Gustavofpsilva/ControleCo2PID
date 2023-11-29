#include <SoftwareSerial.h>
#include <PID_v1.h>

#define RX_PIN 2   // Pino RX do sensor MH-Z19
#define TX_PIN 3   // Pino TX do sensor MH-Z19
#define CO2_PIN A0 // Pino analógico para leitura do nível de CO2

SoftwareSerial co2Serial(RX_PIN, TX_PIN);

// Configuração do PID
double setpoint = 1000.0; // Nível de CO2 desejado em ppm
double input, output;
double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

// Definindo os limites de saída do PID
double outMin = 0;
double outMax = 100;

// Pinagem do dispositivo de controle de CO2 (um ventilador, por exemplo)
// Substitua pelo pino que controla o dispositivo em seu hardware específico
const int CONTROL_PIN = 13;

void setup() {
  Serial.begin(9600);
  co2Serial.begin(9600);
  
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);
}

void loop() {
  double currentCO2 = readCO2();

  // Chama a função do PID
  PID_Controller(currentCO2);

  // Exibe informações no Serial Monitor
  Serial.print("Nível de CO2 Atual: ");
  Serial.print(currentCO2);
  Serial.print(" ppm | Saída do PID: ");
  Serial.println(output);

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

double readCO2() {
  co2Serial.write(0xFF);
  co2Serial.write(0x01);
  co2Serial.write(0x86);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);

  delay(10);

  while (co2Serial.available() > 0) {
    if (co2Serial.read() == 0xFF) {
      if (co2Serial.read() == 0x86) {
        int co2High = co2Serial.read();
        int co2Low = co2Serial.read();
        return (co2High << 8) + co2Low;
      }
    }
  }

  return -1; // Retorno de erro se não conseguir ler corretamente
}

void PID_Controller(double currentCO2) {
  // Calcula o erro
  error = setpoint - currentCO2;

  // Calcula o tempo decorrido
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  output = P + I + D;

  // Limita a saída do PID dentro dos limites especificados
  output = constrain(output, outMin, outMax);

  // Aplica a saída do PID para controlar algum dispositivo (por exemplo, um ventilador)
  // Neste exemplo, apenas exibimos a saída no Serial Monitor
  // Você deve adaptar esta parte para controlar seu sistema específico
  // (por exemplo, controle de um ventilador para regular a ventilação)
  digitalWrite(CONTROL_PIN, output > 50); // Liga o dispositivo se a saída for maior que 50

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;
}
