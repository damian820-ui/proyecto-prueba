// Módulo 1: Configuración Wi-Fi
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

const char* ssid = "Motomoto1";
const char* password = "motomoto";
bool wifiConnected = false;
bool prevWifiConnected = false;  // Variable para el estado anterior
bool isConnecting = false;

// Módulo 2: Dependencias: SPIFFS, RTC, NTP
#include <SPIFFS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>
RTC_DS3231 rtc;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -10800, 60000);

// Módulo 3: Pines y variables relacionadas con presión, bomba y relé
const int pressurePin = 34;
const int pumpPin = 26;      // Pin lógico de la bomba (no relé)
const int relayPin = 25;     // Pin para controlar el relé de la bomba
const int ledPin = 2;        // LED indicador
float maxVoltage = 3.3;
float maxPressure = 5.0;
float currentPressure = 0.0;
float pressureSum = 0.0;
int pressureCount = 0;
bool pumpOn = false;
bool prevPumpOn = false;     // Variable para el estado anterior de la bomba
float offset = -0.6;         // Offset ajustable

// Módulo 4: Variables para calcular horas de servicio de la bomba
DateTime pumpStartTime = DateTime((uint32_t)0);  // Hora de inicio del ciclo de la bomba
unsigned long totalPumpTime = 0;                 // Tiempo total acumulado en segundos

// Variables nuevas para el tiempo parcial
unsigned long partialPumpTime = 0;               // Tiempo parcial acumulado en segundos
DateTime lastPartialSaveTime = DateTime((uint32_t)0);  // Última vez que se guardó el tiempo parcial

// Definiciones para la versión del firmware y la fecha de compilación
#define FIRMWARE_VERSION "V1.0.0"
#define BUILD_DATE "17/09/2024"

// Módulo 5: Archivos para registro de eventos y presión
const char* eventFile = "/eventos.txt";
const char* pressureFile = "/presion_promedio.csv";
const char* totalPumpTimeFile = "/total_pump_time.txt";
const char* partialPumpTimeFile = "/partial_pump_time.txt";  // Archivo para tiempo parcial
const char* versionFile = "/version.txt";  // Nuevo archivo para la versión
String eventBuffer = "";
String pressureBuffer = "";

// Mutex para proteger variables compartidas
SemaphoreHandle_t xMutex;

// Módulo 6: Sincronización de RTC con NTP
void syncRTCWithNTP() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  DateTime now = DateTime(epochTime);
  rtc.adjust(now);
  Serial.println("RTC ajustado con la hora NTP.");
}

// Módulo 7: Obtener timestamp del RTC
String getTimestamp() {
  DateTime now = rtc.now();
  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "%02d/%02d/%04d %02d:%02d:%02d", 
           now.day(), now.month(), now.year(), 
           now.hour(), now.minute(), now.second());
  return String(timestamp);
}

// Módulo 8: Configuración de la atenuación del ADC
void setupADC() {
  // Configuración de atenuación de 11dB para rango de 0 a 3.3V
  analogSetPinAttenuation(pressurePin, ADC_11db);
}

// Módulo 9: Leer la presión con corrección de offset
float readPressure() {
  int adcValue = analogRead(pressurePin);
  float voltage = ((float)adcValue / 4095.0) * maxVoltage;
  float pressure = (voltage / maxVoltage) * maxPressure;
  return pressure + offset;
}

// Módulo 10: Promedio de 10 lecturas
float getAveragePressure() {
  float sum = 0.0;
  int sampleCount = 10;
  for (int i = 0; i < sampleCount; i++) {
    sum += readPressure();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return sum / sampleCount;
}

// Módulo 11: Registrar eventos en archivo de eventos
void logEvent(String eventType, String eventValue) {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    String timestamp = getTimestamp();
    String logEntry = timestamp + ", " + eventType + ", " + eventValue + "\n";
    eventBuffer += logEntry;
    Serial.println("Evento registrado: " + eventType + " - " + eventValue);

    // Escribir en el archivo inmediatamente
    File file = SPIFFS.open(eventFile, FILE_APPEND);
    if (!file) {
      Serial.println("Error al abrir archivo de eventos para escritura.");
    } else {
      file.print(eventBuffer);
      file.close();
      eventBuffer = "";
      Serial.println("Evento guardado en el archivo de eventos.");
    }
    xSemaphoreGive(xMutex);
  }
}

// Módulo 12: Registrar presión promedio en archivo de presión
void logPressure(float avgPressure) {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    String timestamp = getTimestamp();
    String logEntry = timestamp + ", " + String(avgPressure, 2) + " bar\n";
    pressureBuffer += logEntry;
    Serial.println("Presión promedio registrada: " + String(avgPressure, 2) + " bar");

    // Escribir en el archivo inmediatamente
    File file = SPIFFS.open(pressureFile, FILE_APPEND);
    if (!file) {
      Serial.println("Error al abrir archivo de presión para escritura.");
    } else {
      file.print(pressureBuffer);
      file.close();
      pressureBuffer = "";
      Serial.println("Dato de presión guardado en el archivo de presión.");
    }
    xSemaphoreGive(xMutex);
  }
}

// Nueva función para escribir la versión y fecha de compilación en un archivo separado
void writeVersionFile() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    File file = SPIFFS.open(versionFile, FILE_WRITE);
    if (file) {
      String content = "**************OHM INSTRUMENTAL Div. I+D+I***********************\n";
      content += "Version de firmware: " + String(FIRMWARE_VERSION) + "\n";
      content += "Fecha de compilacion: " + String(BUILD_DATE) + "\n";
      file.print(content);
      file.close();
      Serial.println("Archivo de versión actualizado.");
    } else {
      Serial.println("Error al crear o abrir el archivo de versión.");
    }
    xSemaphoreGive(xMutex);
  }
}

// Módulo 13: Control de la bomba con registro de eventos booleanos y cálculo de horas de servicio
void controlPump(float pressure) {
  if (pressure < 1.5 && !pumpOn) {
    digitalWrite(pumpPin, HIGH);  // Encender la bomba lógicamente
    digitalWrite(relayPin, LOW);  // Activar el relé (generalmente en LOW)
    pumpOn = true;
    if (pumpOn != prevPumpOn) {
      logEvent("Bomba", "Encendida");
      prevPumpOn = pumpOn;

      // Registrar hora de inicio del ciclo
      pumpStartTime = rtc.now();
      lastPartialSaveTime = pumpStartTime;  // Inicializar lastPartialSaveTime
      Serial.print("pumpStartTime establecido a: ");
      Serial.println(pumpStartTime.timestamp());
      Serial.print("lastPartialSaveTime establecido a: ");
      Serial.println(lastPartialSaveTime.timestamp());
    }
  } else if (pressure >= 2.8 && pumpOn) {
    digitalWrite(pumpPin, LOW);   // Apagar la bomba lógicamente
    digitalWrite(relayPin, HIGH); // Desactivar el relé
    pumpOn = false;
    if (pumpOn != prevPumpOn) {
      logEvent("Bomba", "Apagada");
      prevPumpOn = pumpOn;

      // Verificar si pumpStartTime es válido
      if (pumpStartTime.unixtime() > 0) {
        // Calcular tiempo de servicio del ciclo
        DateTime pumpEndTime = rtc.now();
        TimeSpan cycleTime = pumpEndTime - pumpStartTime;
        int cycleMinutes = cycleTime.totalseconds() / 60;

        Serial.print("pumpStartTime: ");
        Serial.println(pumpStartTime.timestamp());
        Serial.print("pumpEndTime: ");
        Serial.println(pumpEndTime.timestamp());
        Serial.print("Tiempo de servicio del ciclo (segundos): ");
        Serial.println(cycleTime.totalseconds());

        // Agregar tiempo desde el último guardado parcial hasta ahora
        TimeSpan partialCycleTime = pumpEndTime - lastPartialSaveTime;
        partialPumpTime += partialCycleTime.totalseconds();

        // Actualizar totalPumpTime con partialPumpTime
        totalPumpTime += partialPumpTime;

        // Eliminar archivo de tiempo parcial si existe
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          SPIFFS.remove(partialPumpTimeFile);
          xSemaphoreGive(xMutex);
        }
        partialPumpTime = 0;  // Reiniciar tiempo parcial

        if (cycleMinutes >= 1) {
          // Registrar tiempo de servicio del ciclo
          int hours = cycleTime.totalseconds() / 3600;
          int minutes = (cycleTime.totalseconds() % 3600) / 60;

          String serviceTimeEntry = "Tiempo de bomba en servicio: " + String(hours) + " horas " + String(minutes) + " minutos";
          logEvent("", serviceTimeEntry);

          // Calcular horas y minutos totales
          unsigned long totalHours = totalPumpTime / 3600;
          unsigned long totalMinutes = (totalPumpTime % 3600) / 60;

          String totalTimeEntry = "HORAS BOMBA: " + String(totalHours) + " horas " + String(totalMinutes) + " minutos";
          logEvent("", totalTimeEntry);

          // Guardar tiempo total en memoria persistente
          if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            File file = SPIFFS.open(totalPumpTimeFile, FILE_WRITE);
            if (file) {
              file.println(totalPumpTime);
              file.close();
              Serial.println("Tiempo total de la bomba guardado correctamente.");
            } else {
              Serial.println("Error al guardar tiempo total de la bomba.");
            }
            xSemaphoreGive(xMutex);
          }
        }
        // Reiniciar pumpStartTime y lastPartialSaveTime
        pumpStartTime = DateTime((uint32_t)0);
        lastPartialSaveTime = DateTime((uint32_t)0);
      } else {
        Serial.println("pumpStartTime no es válido. No se puede calcular el tiempo de servicio.");
      }
    }
  }
}

// Módulo 14: Leer y mostrar el archivo de eventos
void readEvents() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    File file = SPIFFS.open(eventFile, FILE_READ);
    if (!file) {
      Serial.println("Error al abrir archivo de eventos.");
      xSemaphoreGive(xMutex);
      return;
    }
    Serial.println("Contenido del archivo de eventos:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    xSemaphoreGive(xMutex);
  }
}

// Módulo 15: Leer y mostrar el archivo de presión
void readPressureData() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    File file = SPIFFS.open(pressureFile, FILE_READ);
    if (!file) {
      Serial.println("Error al abrir archivo de presión.");
      xSemaphoreGive(xMutex);
      return;
    }
    Serial.println("Contenido del archivo de presión:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    xSemaphoreGive(xMutex);
  }
}

// Nueva función para leer y mostrar el archivo de versión
void readVersionFile() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    File file = SPIFFS.open(versionFile, FILE_READ);
    if (!file) {
      Serial.println("Error al abrir archivo de versión.");
      xSemaphoreGive(xMutex);
      return;
    }
    Serial.println("Contenido del archivo de versión:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    xSemaphoreGive(xMutex);
  }
}

// Módulo 16: Eliminar los archivos
void deleteFiles() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    if (SPIFFS.remove(eventFile)) {
      Serial.println("Archivo de eventos eliminado.");
    } else {
      Serial.println("Error al eliminar archivo de eventos.");
    }
    if (SPIFFS.remove(pressureFile)) {
      Serial.println("Archivo de presión eliminado.");
    } else {
      Serial.println("Error al eliminar archivo de presión.");
    }
    // Eliminar el archivo de tiempo total de la bomba
    if (SPIFFS.remove(totalPumpTimeFile)) {
      Serial.println("Archivo de tiempo total de la bomba eliminado.");
      totalPumpTime = 0;  // Reiniciar el totalizador
    } else {
      Serial.println("Error al eliminar archivo de tiempo total de la bomba.");
    }
    // Eliminar el archivo de tiempo parcial de la bomba
    if (SPIFFS.remove(partialPumpTimeFile)) {
      Serial.println("Archivo de tiempo parcial de la bomba eliminado.");
      partialPumpTime = 0;  // Reiniciar el tiempo parcial
    } else {
      Serial.println("Error al eliminar archivo de tiempo parcial de la bomba.");
    }
    // Eliminar el archivo de versión
    if (SPIFFS.remove(versionFile)) {
      Serial.println("Archivo de versión eliminado.");
    } else {
      Serial.println("Error al eliminar archivo de versión.");
    }
    xSemaphoreGive(xMutex);
  }
}

// Módulo 17: Tarea de lectura de presión y control de la bomba
void TaskReadPressure(void *pvParameters) {
  unsigned long lowPressureStartTime = 0;
  bool lowPressureAlarm = false;
  bool waitingToRetry = false;
  unsigned long retryStartTime = 0;
  const TickType_t xDelay500ms = pdMS_TO_TICKS(500);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  float avgPressure = 0.0;

  for(;;) {
    float pressure = getAveragePressure();

    Serial.print("Presión promedio (10 lecturas): ");
    Serial.println(pressure);

    // Controlar la bomba según la presión solo si no está en espera
    if (!waitingToRetry) {
      controlPump(pressure);
    }

    // Si la bomba está encendida, verificar si ha pasado 2 minutos para guardar el tiempo parcial
    if (pumpOn && pumpStartTime.unixtime() > 0) {
      DateTime now = rtc.now();
      TimeSpan elapsedSinceLastSave = now - lastPartialSaveTime;

      Serial.print("Tiempo transcurrido desde lastPartialSaveTime: ");
      Serial.println(elapsedSinceLastSave.totalseconds());

      if (elapsedSinceLastSave.totalseconds() >= 120) {  // Han pasado 2 minutos
        // Calcular tiempo desde lastPartialSaveTime hasta ahora
        TimeSpan partialTimeSpan = now - lastPartialSaveTime;
        partialPumpTime += partialTimeSpan.totalseconds();

        // Guardar partialPumpTime en el archivo de buffer
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          File file = SPIFFS.open(partialPumpTimeFile, FILE_WRITE);
          if (file) {
            file.println(partialPumpTime);
            file.flush();  // Asegurar que los datos se escriben físicamente
            file.close();
            Serial.println("Tiempo parcial de la bomba guardado en memoria persistente.");
            Serial.print("partialPumpTime guardado: ");
            Serial.println(partialPumpTime);
          } else {
            Serial.println("Error al guardar tiempo parcial de la bomba.");
          }
          xSemaphoreGive(xMutex);
        }

        lastPartialSaveTime = now;  // Actualizar lastPartialSaveTime
        Serial.print("lastPartialSaveTime actualizado a: ");
        Serial.println(lastPartialSaveTime.timestamp());
      }
    }

    // Verificar si la bomba está encendida y la presión es menor a 0.5 bar
    if (pumpOn && pressure < 0.5 && !waitingToRetry) {
      if (lowPressureStartTime == 0) {
        lowPressureStartTime = xTaskGetTickCount();  // Marcar el tiempo cuando la presión baja
      } else if ((xTaskGetTickCount() - lowPressureStartTime) > pdMS_TO_TICKS(10000)) {  // 10 segundos en baja presión
        lowPressureAlarm = true;
        logEvent("Alarma", "Baja presión prolongada, bomba apagada");
        digitalWrite(pumpPin, LOW);    // Apagar la bomba lógicamente
        digitalWrite(relayPin, HIGH);  // Desactivar el relé (apagado)
        pumpOn = false;
        prevPumpOn = pumpOn;
        waitingToRetry = true;
        retryStartTime = xTaskGetTickCount();  // Iniciar el tiempo de espera de 5 minutos
        Serial.println("Bomba apagada por baja presión prolongada.");

        // Proteger variables al resetear
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          pressureSum = 0.0;
          pressureCount = 0;
          xSemaphoreGive(xMutex);
        }

        // Verificar si pumpStartTime es válido
        if (pumpStartTime.unixtime() > 0) {
          // Calcular tiempo de servicio del ciclo
          DateTime pumpEndTime = rtc.now();
          TimeSpan cycleTime = pumpEndTime - pumpStartTime;
          int cycleMinutes = cycleTime.totalseconds() / 60;

          Serial.print("pumpStartTime: ");
          Serial.println(pumpStartTime.timestamp());
          Serial.print("pumpEndTime: ");
          Serial.println(pumpEndTime.timestamp());
          Serial.print("Tiempo de servicio del ciclo (segundos): ");
          Serial.println(cycleTime.totalseconds());

          // Agregar tiempo desde el último guardado parcial hasta ahora
          TimeSpan partialCycleTime = pumpEndTime - lastPartialSaveTime;
          partialPumpTime += partialCycleTime.totalseconds();

          // Actualizar totalPumpTime con partialPumpTime
          totalPumpTime += partialPumpTime;

          // Eliminar archivo de tiempo parcial si existe
          if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            SPIFFS.remove(partialPumpTimeFile);
            xSemaphoreGive(xMutex);
          }
          partialPumpTime = 0;  // Reiniciar tiempo parcial

          if (cycleMinutes >= 1) {
            // Registrar tiempo de servicio del ciclo
            int hours = cycleTime.totalseconds() / 3600;
            int minutes = (cycleTime.totalseconds() % 3600) / 60;

            String serviceTimeEntry = "Tiempo de bomba en servicio: " + String(hours) + " horas " + String(minutes) + " minutos";
            logEvent("", serviceTimeEntry);

            // Calcular horas y minutos totales
            unsigned long totalHours = totalPumpTime / 3600;
            unsigned long totalMinutes = (totalPumpTime % 3600) / 60;

            String totalTimeEntry = "HORAS BOMBA: " + String(totalHours) + " horas " + String(totalMinutes) + " minutos";
            logEvent("", totalTimeEntry);

            // Guardar tiempo total en memoria persistente
            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
              File file = SPIFFS.open(totalPumpTimeFile, FILE_WRITE);
              if (file) {
                file.println(totalPumpTime);
                file.close();
                Serial.println("Tiempo total de la bomba guardado correctamente.");
              } else {
                Serial.println("Error al guardar tiempo total de la bomba.");
              }
              xSemaphoreGive(xMutex);
            }
          }
          // Reiniciar pumpStartTime y lastPartialSaveTime
          pumpStartTime = DateTime((uint32_t)0);
          lastPartialSaveTime = DateTime((uint32_t)0);
        } else {
          Serial.println("pumpStartTime no es válido. No se puede calcular el tiempo de servicio.");
        }
      }
    } else {
      // Reiniciar el contador si la presión se recupera o si la bomba no está encendida
      lowPressureStartTime = 0;
      lowPressureAlarm = false;
    }

    // Si estamos esperando para reintentar, verificar si han pasado 5 minutos
    if (waitingToRetry && (xTaskGetTickCount() - retryStartTime) > pdMS_TO_TICKS(300000)) {  // 5 minutos
      waitingToRetry = false;  // Termina el tiempo de espera, volver a intentar
      Serial.println("Reintentando encender la bomba después de 5 minutos.");
      logEvent("Bomba", "Reintentando encender después de 5 minutos");

      // Intentar encender la bomba de nuevo si la presión es menor a 1.5 bar
      if (pressure < 1.5) {
        digitalWrite(pumpPin, HIGH);   // Encender la bomba
        digitalWrite(relayPin, LOW);   // Activar el relé
        pumpOn = true;
        if (pumpOn != prevPumpOn) {
          logEvent("Bomba", "Encendida después de 5 minutos de espera");
          prevPumpOn = pumpOn;

          // Registrar hora de inicio del ciclo
          pumpStartTime = rtc.now();
          lastPartialSaveTime = pumpStartTime;  // Inicializar lastPartialSaveTime
          Serial.print("pumpStartTime establecido a: ");
          Serial.println(pumpStartTime.timestamp());
          Serial.print("lastPartialSaveTime establecido a: ");
          Serial.println(lastPartialSaveTime.timestamp());
        }
        Serial.println("Bomba encendida después de 5 minutos de espera.");

        // Proteger variables al resetear
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          pressureSum = 0.0;
          pressureCount = 0;
          xSemaphoreGive(xMutex);
        }
      } else {
        logEvent("Bomba", "No se pudo encender, presión mayor a 1.5 bar");
        Serial.println("No se pudo encender la bomba, presión mayor a 1.5 bar.");
      }
    }

    // Actualizar y verificar el contador de presión dentro del mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      if (pumpOn) {
        pressureSum += pressure;
        pressureCount++;
        if (pressureCount >= 240) {  // Cada 2 minutos (240 * 500ms)
          avgPressure = pressureSum / pressureCount;
          pressureSum = 0.0;
          pressureCount = 0;
          xSemaphoreGive(xMutex);

          // Registrar el promedio de presión
          logPressure(avgPressure);
        } else {
          xSemaphoreGive(xMutex);
        }
      } else {
        // Si la bomba no está encendida, reiniciar variables
        pressureSum = 0.0;
        pressureCount = 0;
        xSemaphoreGive(xMutex);
      }
    }

    // Intervalo de 500 milisegundos para mostrar los valores promedio
    vTaskDelayUntil(&xLastWakeTime, xDelay500ms);
  }
}

// Módulo 18: Tarea de conexión Wi-Fi mejorada
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Conectado al punto de acceso.");
      isConnecting = false;  // Restablecer isConnecting
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("Dirección IP: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      if (!prevWifiConnected) {
        digitalWrite(ledPin, HIGH);
        logEvent("Wi-Fi", "Conectado a la red Wi-Fi");
        prevWifiConnected = wifiConnected;
      }
      syncRTCWithNTP();  // Sincronizar RTC al conectarse
      isConnecting = false;  // Restablecer isConnecting
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      wifiConnected = false;
      if (prevWifiConnected) {
        Serial.println("Desconectado del punto de acceso.");
        digitalWrite(ledPin, LOW);
        logEvent("Wi-Fi", "No hay conexión Wi-Fi");
        prevWifiConnected = wifiConnected;
      }
      isConnecting = false;  // Restablecer isConnecting
      break;
    default:
      isConnecting = false;  // Restablecer isConnecting en otros eventos también
      break;
  }
}

void TaskWiFiConnection(void *pvParameters) {
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);

  const int maxRetries = 5;
  int retryCount = 0;

  for(;;) {
    if (!wifiConnected && !isConnecting) {
      if (retryCount < maxRetries) {
        isConnecting = true;
        Serial.println("Intentando conectar a Wi-Fi...");
        WiFi.begin(ssid, password);
        retryCount++;
      } else {
        Serial.println("Número máximo de reintentos alcanzado. Esperando antes de volver a intentar.");
        vTaskDelay(pdMS_TO_TICKS(60000));  // Esperar 1 minuto antes de reintentar
        retryCount = 0;
      }
    }

    if (wifiConnected) {
      retryCount = 0;  // Restablecer contador al conectarse
      // Sincronizar RTC cada hora
      static unsigned long lastSyncTime = 0;
      if (millis() - lastSyncTime > 3600000) {  // Cada 1 hora
        syncRTCWithNTP();
        lastSyncTime = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// Módulo 19: Tarea de comandos del monitor serie
void TaskSerialMonitor(void *pvParameters) {
  for(;;) {
    if (Serial.available()) {
      char command = Serial.read();
      if (command == 'e') {
        readEvents();
      } else if (command == 'p') {
        readPressureData();
      } else if (command == 'd') {
        deleteFiles();
      } else if (command == 'v') {
        readVersionFile();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Módulo 20: setup() y loop()
void setup() {
  Serial.begin(115200);
  // Configurar nivel de log para Wi-Fi
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);  // Nivel de log detallado

  pinMode(pressurePin, INPUT);
  pinMode(pumpPin, OUTPUT);       // Configurar el pin lógico de la bomba
  pinMode(relayPin, OUTPUT);      // Configurar el pin del relé
  pinMode(ledPin, OUTPUT);        // LED indicador
  digitalWrite(pumpPin, LOW);     // Asegurarse de que la bomba esté apagada
  digitalWrite(relayPin, HIGH);   // Asegurarse de que el relé esté desactivado
  digitalWrite(ledPin, LOW);      // Asegurarse de que el LED esté apagado

  if (!SPIFFS.begin()) {
    Serial.println("Error al inicializar SPIFFS");
    return;
  }

  if (!rtc.begin()) {
    Serial.println("Error al inicializar RTC.");
    return;
  }

  // Crear mutex
  xMutex = xSemaphoreCreateMutex();

  // Escribir el archivo de versión
  writeVersionFile();

  // Detección de razón de reinicio y registro de evento
  esp_reset_reason_t reset_reason = esp_reset_reason();

  switch (reset_reason) {
    case ESP_RST_POWERON:
      logEvent("Reinicio", "Tras corte de energía");
      break;
    case ESP_RST_SW:
      logEvent("Reinicio", "Reinicio manual (software)");
      break;
    case ESP_RST_WDT:
      logEvent("Reinicio", "Reinicio por watchdog timer");
      break;
    case ESP_RST_BROWNOUT:
      logEvent("Reinicio", "Reinicio por caída de voltaje (brownout)");
      break;
    case ESP_RST_PANIC:
      logEvent("Reinicio", "Reinicio por error de software (panic)");
      break;
    default:
      logEvent("Reinicio", "Otra razón");
      break;
  }

  setupADC();

  // Leer el tiempo total de servicio de la bomba desde memoria persistente
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    File file = SPIFFS.open(totalPumpTimeFile, FILE_READ);
    if (file) {
      String content = file.readStringUntil('\n');
      totalPumpTime = content.toInt();
      file.close();
      Serial.print("Tiempo total de la bomba leído: ");
      Serial.println(totalPumpTime);
    } else {
      totalPumpTime = 0;
      Serial.println("No se encontró archivo de tiempo total de la bomba. Se inicia en cero.");
    }
    xSemaphoreGive(xMutex);
  }

  // Leer el tiempo parcial de servicio de la bomba desde memoria persistente
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    Serial.println("Intentando leer el tiempo parcial de la bomba...");
    File file = SPIFFS.open(partialPumpTimeFile, FILE_READ);
    if (file) {
      String content = file.readStringUntil('\n');
      partialPumpTime = content.toInt();
      totalPumpTime += partialPumpTime;  // Sumar el tiempo parcial al totalizador general
      file.close();

      // Eliminar el archivo de tiempo parcial ya que se ha sumado al total
      SPIFFS.remove(partialPumpTimeFile);
      partialPumpTime = 0;
      Serial.println("Tiempo parcial de la bomba recuperado y sumado al total.");
      Serial.print("partialPumpTime leído: ");
      Serial.println(partialPumpTime);
      Serial.print("totalPumpTime actualizado: ");
      Serial.println(totalPumpTime);

      // Guardar el nuevo totalPumpTime en memoria persistente
      File totalFile = SPIFFS.open(totalPumpTimeFile, FILE_WRITE);
      if (totalFile) {
        totalFile.println(totalPumpTime);
        totalFile.close();
        Serial.println("Tiempo total de la bomba actualizado después de sumar tiempo parcial.");
      } else {
        Serial.println("Error al actualizar tiempo total de la bomba después de sumar tiempo parcial.");
      }
    } else {
      partialPumpTime = 0;
      Serial.println("No se encontró archivo de tiempo parcial de la bomba.");
    }
    xSemaphoreGive(xMutex);
  }

  // Asignar tareas a los núcleos de manera equilibrada
  xTaskCreatePinnedToCore(TaskReadPressure, "ReadPressure", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskWiFiConnection, "WiFiConnection", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSerialMonitor, "SerialMonitor", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // El bucle principal queda vacío ya que todas las tareas están siendo manejadas por FreeRTOS
}
