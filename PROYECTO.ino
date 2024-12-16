#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <BluetoothSerial.h>

#define REPORTING_PERIOD_HR_MS 3000  // Reporte cada 3 segundo
#define INT_PIN 25
#define SENSOR_PIN 26
#define SAMPLING_TIME 5000  // Tiempo de muestreo para respiraciones

PulseOximeter pox;
BluetoothSerial SerialBT;

uint32_t tsLastReportHR = 0;
float heartRateSum = 0;
int readingCount = 0;
unsigned long lastBreathTime = 0;

unsigned long startTime;
int breathCount = 0;

bool sensor1Active = false;
bool sensor2Active = false;

void onBeatDetected() {
  Serial.println("Latido detectado!");
}

void initializePulseOximeter() {
  if (!pox.begin()) {
    Serial.println("Fallo en inicialización del PulseOximeter. Reiniciando...");
    delay(1000);
    ESP.restart();  // Reinicia en caso de fallo crítico
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_PulseSensor");  // Configuración del Bluetooth
  Serial.println("Bluetooth iniciado, esperando comandos...");
  Serial.println("Inicializando...");

  // Inicialización del bus I2C
  Wire.begin();

  // Inicializar MAX30100
  initializePulseOximeter();

  // Configuración inicial
  startTime = millis();

  Serial.println("Inicialización completada.");
}

void loop() {
  if (SerialBT.available()) {
    char comando = SerialBT.read();
    Serial.print("Comando recibido: ");
    Serial.println(comando);

    if (comando == 'A') {
      sensor1Active = !sensor1Active;
      sensor2Active = false;  // Asegurar que el otro sensor se desactiva
    } else if (comando == 'B') {
      sensor2Active = !sensor2Active;
      sensor1Active = false;  // Asegurar que el otro sensor se desactiva
    } else {
      Serial.println("Comando no válido. Usa 'A' o 'B'.");
    }
  }

  unsigned long currentTime = millis();
  pox.update(); // Actualiza el sensor MAX30100
    

  if (sensor1Active) {
    // Procesar datos del MAX30100
    float currentHeartRate = pox.getHeartRate();
    if (currentHeartRate > 0) {
      heartRateSum += currentHeartRate;
      readingCount++;
    }

    // Reportar datos cada 3 segundos
    if (currentTime - tsLastReportHR > REPORTING_PERIOD_HR_MS) {
      if (readingCount > 0) {
        float averageHeartRate = heartRateSum / readingCount;
        String data = "HR: " + String(averageHeartRate) + " bpm, SpO2: " + String(pox.getSpO2()) + "%";
        SerialBT.println(data);  // Enviar datos por Bluetooth
        Serial.println(data);    // Imprimir en Serial Monitor

        heartRateSum = 0;
        readingCount = 0;
      } else {
        Serial.println("Sin lecturas válidas de frecuencia cardíaca");
      }
      tsLastReportHR = currentTime;
    }
  } else if (sensor2Active) {
    int sensorValue = analogRead(SENSOR_PIN);

    // Detectar respiraciones
    if (sensorValue > 2500 && currentTime - lastBreathTime > 200) {  // Umbral con debounce
      breathCount++;
      lastBreathTime = currentTime;
    }

    // Calcular frecuencia respiratoria
    if (currentTime - tsLastReportHR >= REPORTING_PERIOD_HR_MS) {
      float breathsPerMinute = (breathCount * 60.0) / (SAMPLING_TIME / 1000.0);
      String data = "Resp: " + String(breathsPerMinute-30) + " RPM";
      SerialBT.println(data);  // Enviar datos por Bluetooth
      Serial.println(breathsPerMinute);    // Imprimir en Serial Monitor

      breathCount = 0;
      startTime = currentTime;
      tsLastReportHR = currentTime;
    }
  } else {
    Serial.println("Selecciona la variable a mostrar.");
    //delay(1000);  // Reducir frecuencia de impresión
  }
}
