#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Configuraciones del LCD
#define LCD_ADDRESS 0x27  // Dirección del LCD (puede variar según el módulo)
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Pines de conexión del LCD al ESP32
#define SDA_PIN 5  // SDA conectado al pin D5
#define SCL_PIN 18 // SCL conectado al pin D18

// Configuración del sensor DHT
#define DHTPIN 4       // Pin al que está conectado el sensor LHT11
#define DHTTYPE DHT11  // Tipo de sensor DHT11
DHT dht(DHTPIN, DHTTYPE);

// Pines del LED RGB
#define RED_PIN 21   // Pin del color rojo
#define GREEN_PIN 22 // Pin del color verde
#define BLUE_PIN 23  // Pin del color azul

// Pin del relé para el ventilador
#define RELAY_PIN 27  // Pin al que está conectado el relé

// Variables para los pulsadores
#define boton1Pin 33
#define boton2Pin 35
#define boton3Pin 34

// Variables para el control de la temperatura máxima
volatile int temperatura_maxima = 25;  // Valor inicial de la temperatura máxima
volatile bool mostrarTemperaturaMaxima = false;
volatile bool cambioTemperatura = false;

void IRAM_ATTR ISR_boton1() { //ISR=Interrupt service routine, la funcion se ejecuta cuando hay una interrupcion por el boton 1
  mostrarTemperaturaMaxima = !mostrarTemperaturaMaxima; // Cambiar entre mostrar temperatura/humedad y temperatura máxima
}

void IRAM_ATTR ISR_boton2() { //IRAM_ATTR=Almacenarse en la RAM interna del ESP32
  temperatura_maxima--; // Disminuir temperatura máxima
  cambioTemperatura = true;
}

void IRAM_ATTR ISR_boton3() {
  temperatura_maxima++; // Aumentar temperatura máxima
  cambioTemperatura = true;
}

void setup() {
  // Inicializar la comunicación I2C con los pines SDA y SCL
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializar el LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.backlight(); // Encender la retroiluminación del LCD
  lcd.setCursor(0, 0);
  lcd.print("Inicializando...");

  // Inicializar el sensor DHT
  dht.begin();

  // Configurar los pines del LED RGB como salidas
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Configurar el pin del relé como salida
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Ventilador apagado inicialmente

  // Configurar los pines de los pulsadores
  pinMode(boton1Pin, INPUT_PULLUP);
  pinMode(boton2Pin, INPUT_PULLUP);
  pinMode(boton3Pin, INPUT_PULLUP);

  // Configurar interrupciones
  attachInterrupt(digitalPinToInterrupt(boton1Pin), ISR_boton1, FALLING);
  attachInterrupt(digitalPinToInterrupt(boton2Pin), ISR_boton2, FALLING);
  attachInterrupt(digitalPinToInterrupt(boton3Pin), ISR_boton3, FALLING);

  delay(2000);  // Esperar a que el sensor esté listo
}

void loop() {
  // Leer temperatura y humedad del sensor DHT11
  float humedad = dht.readHumidity();
  float temperatura = dht.readTemperature();

  // Verificar si las lecturas son válidas
  if (isnan(humedad) || isnan(temperatura)) { //Is not a number
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error sensor");
    return;
  }

  // Mostrar temperatura/humedad o temperatura máxima según el estado de 'mostrarTemperaturaMaxima'
  if (mostrarTemperaturaMaxima) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp Maxima:");
    lcd.setCursor(0, 1);
    lcd.print(temperatura_maxima);
    lcd.print(" C");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperatura);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Humedad: ");
    lcd.print(humedad);
    lcd.print(" %");
  }

  // Actualizar si se cambió la temperatura máxima
  if (cambioTemperatura) {
    if (mostrarTemperaturaMaxima) {
      lcd.setCursor(0, 1);
      lcd.print("                "); // Limpiar la fila
      lcd.setCursor(0, 1);
      lcd.print(temperatura_maxima);
      lcd.print(" C");
    }
    cambioTemperatura = false;
  }

  // Controlar el LED RGB y el ventilador según la temperatura actual y temperatura máxima
  if (temperatura < temperatura_maxima - 1) {
    // Mostrar azul y apagar el ventilador
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    digitalWrite(RELAY_PIN, LOW);  // Ventilador apagado
  } else if (temperatura >= temperatura_maxima - 1 && temperatura < temperatura_maxima) {
    // Mostrar verde y apagar el ventilador
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);  // Ventilador apagado
  } else {
    // Mostrar rojo y encender el ventilador
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(RELAY_PIN, HIGH);  // Ventilador encendido
  }

  // Esperar 2 segundos antes de la siguiente lectura
  delay(2000);
}




