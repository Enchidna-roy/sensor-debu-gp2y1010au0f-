
#include <Arduino.h>
#include <algorithm>
#include <math.h>
#include <ESP32Servo.h>

// ==================================================
// PIN CONFIGURATION
// ==================================================

#define DUST_LED       18
#define DUST_ANALOG    34
#define SERVO_PIN      13
#define RELAY_PIN      27
#define ACS712_PIN     32

// ==================================================
// SENSOR PARAMETERS
// ==================================================

const int SAMPLE_COUNT        = 100;
const int CALIBRATION_SECONDS = 10;
const int ADC_INVALID_BELOW   = 20;

const int   MIN_OFFSET       = 150;
const float SIGMA_MULT       = 6.0;

// Batas deteksi debu
const float DUST_LIMIT_PERCENT = 5.0;

const float BASELINE_ALPHA = 0.0;

// ==================================================
// SERVO PARAMETERS
// ==================================================

Servo wiper;

const int SERVO_MIN      = 0;
const int SERVO_MAX      = 90;
const int SERVO_STEP     = 3;
const int SERVO_INTERVAL = 15;

int servoPos = SERVO_MIN;
int servoDir = 1;

// ==================================================
// SHARED VARIABLES
// ==================================================

float baseline = 0;
float sigma = 0;
int threshold = 0;

volatile bool dustDetected = false;

// ==================================================
// FREERTOS OBJECTS
// ==================================================

QueueHandle_t sensorQueue;
QueueHandle_t outputQueue;

EventGroupHandle_t systemEvent;

#define DUST_DETECTED_BIT (1 << 0)

// ==================================================
// DATA STRUCTURES
// ==================================================

struct SensorData {
  int samples[SAMPLE_COUNT];
  int valid;
};

struct ProcessedData {
  float average;
  int median;
  int adcMin;
  int adcMax;
  int highCount;

  float highPercent;
  float baselineValue;
  int thresholdValue;

  bool dustStatus;
};

// ==================================================
// SENSOR READING
// ==================================================

int readDustSensor() {

  uint32_t t0 = micros();

  digitalWrite(DUST_LED, LOW);

  while (micros() - t0 < 280);

  int adc = analogRead(DUST_ANALOG);

  while (micros() - t0 < 320);

  digitalWrite(DUST_LED, HIGH);

  return adc;
}

// ==================================================
// THRESHOLD
// ==================================================

void updateThreshold() {

  float offset = max(
    (float)MIN_OFFSET,
    SIGMA_MULT * sigma
  );

  threshold = (int)(baseline + offset);
}

// ==================================================
// CALIBRATION
// ==================================================

void calibrateSensor() {

  Serial.println("\n=== KALIBRASI SENSOR ===");
  Serial.println("Pastikan udara dalam kondisi normal.");

  float mean = 0;
  float m2 = 0;

  int n = 0;

  int total = CALIBRATION_SECONDS * SAMPLE_COUNT;

  for (int i = 0; i < total; i++) {

    int adc = readDustSensor();

    if (adc >= ADC_INVALID_BELOW) {

      n++;

      float d = adc - mean;

      mean += d / n;

      m2 += d * (adc - mean);
    }

    if (i % SAMPLE_COUNT == 0) {

      Serial.printf(
        "Kalibrasi: %d detik\n",
        i / SAMPLE_COUNT
      );
    }

    delayMicroseconds(9680);
  }

  baseline = mean;

  sigma = (n > 1)
    ? sqrtf(m2 / (n - 1))
    : 0;

  updateThreshold();

  Serial.println("\n=== KALIBRASI SELESAI ===");

  Serial.printf("Baseline : %.1f\n", baseline);
  Serial.printf("Sigma    : %.1f\n", sigma);
  Serial.printf("Threshold: %d\n", threshold);
}

// ==================================================
// TASK 1: SENSOR TASK
// Core 1 | Priority 3
// ==================================================

void sensorTask(void *parameter) {

  SensorData data;

  TickType_t lastWakeTime = xTaskGetTickCount();

  const TickType_t samplingPeriod = pdMS_TO_TICKS(10);

  while (true) {

    data.valid = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {

      int adc = readDustSensor();

      if (adc >= ADC_INVALID_BELOW) {
        data.samples[data.valid++] = adc;
      }

      vTaskDelayUntil(
        &lastWakeTime,
        samplingPeriod
      );
    }

    xQueueOverwrite(sensorQueue, &data);
  }
}

// ==================================================
// TASK 2: PROCESSING TASK
// Core 1 | Priority 2
// ==================================================

void processingTask(void *parameter) {

  SensorData data;
  ProcessedData result;

  while (true) {

    if (xQueueReceive(
      sensorQueue,
      &data,
      portMAX_DELAY
    ) == pdTRUE) {

      int valid = data.valid;

      if (valid < SAMPLE_COUNT / 2) {

        Serial.println(
          "PERINGATAN: Sampel invalid terlalu banyak!"
        );

        continue;
      }

      int highCount = 0;
      long total = 0;

      for (int i = 0; i < valid; i++) {

        total += data.samples[i];

        if (data.samples[i] > threshold) {
          highCount++;
        }
      }

      std::sort(
        data.samples,
        data.samples + valid
      );

      int adcMin = data.samples[0];
      int adcMax = data.samples[valid - 1];
      int adcMedian = data.samples[valid / 2];

      float average = (float)total / valid;

      float highPercent =
        (float)highCount / valid * 100.0;

      // ------------------------------------------
      // DETEKSI DEBU: BATAS 5%
      // ------------------------------------------

      if (highPercent >= DUST_LIMIT_PERCENT) {

        dustDetected = true;

        xEventGroupSetBits(
          systemEvent,
          DUST_DETECTED_BIT
        );

      } else {

        dustDetected = false;

        xEventGroupClearBits(
          systemEvent,
          DUST_DETECTED_BIT
        );
      }

      // ------------------------------------------
      // BASELINE UPDATE
      // ------------------------------------------

      if (!dustDetected && highCount == 0) {

        baseline =
          (1.0 - BASELINE_ALPHA) * baseline
          + BASELINE_ALPHA * adcMedian;

        updateThreshold();
      }

      // ------------------------------------------
      // SAVE RESULTS
      // ------------------------------------------

      result.average = average;
      result.median = adcMedian;

      result.adcMin = adcMin;
      result.adcMax = adcMax;

      result.highCount = highCount;
      result.highPercent = highPercent;

      result.baselineValue = baseline;
      result.thresholdValue = threshold;

      result.dustStatus = dustDetected;

      xQueueOverwrite(outputQueue, &result);
    }
  }
}

// ==================================================
// TASK 3: SERVO + RELAY TASK
// Core 0 | Priority 2
// ==================================================

void servoTask(void *parameter) {

  while (true) {

    EventBits_t bits = xEventGroupGetBits(systemEvent);

    bool dust = bits & DUST_DETECTED_BIT;

    if (dust) {

      // RELAY ON
      digitalWrite(RELAY_PIN, HIGH);

      // Wiper bergerak bolak-balik
      servoPos += servoDir * SERVO_STEP;

      if (servoPos >= SERVO_MAX) {

        servoPos = SERVO_MAX;
        servoDir = -1;

      } else if (servoPos <= SERVO_MIN) {

        servoPos = SERVO_MIN;
        servoDir = 1;
      }

      wiper.write(servoPos);

    } else {

      // RELAY LANGSUNG OFF
      digitalWrite(RELAY_PIN, LOW);

      // Servo kembali ke posisi awal
      if (servoPos != SERVO_MIN) {

        servoPos -= SERVO_STEP;

        if (servoPos < SERVO_MIN) {
          servoPos = SERVO_MIN;
        }

        servoDir = 1;

        wiper.write(servoPos);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SERVO_INTERVAL));
  }
}

// ==================================================
// TASK 4: SERIAL MONITOR TASK
// Core 0 | Priority 1
// ==================================================

void serialTask(void *parameter) {

  ProcessedData result;

  while (true) {

    if (xQueueReceive(
      outputQueue,
      &result,
      portMAX_DELAY
    ) == pdTRUE) {

      // BACA ADC RAW ACS712
      int acsRaw = analogRead(ACS712_PIN);

      // BACA STATUS GPIO RELAY
      int relayRaw = digitalRead(RELAY_PIN);

      Serial.println("\n-------------------------");

      Serial.printf(
        "ADC Avg/Med : %.1f / %d\n",
        result.average,
        result.median
      );

      Serial.printf(
        "ADC Min/Max : %d / %d\n",
        result.adcMin,
        result.adcMax
      );

      Serial.printf(
        "Baseline    : %.1f | Threshold: %d\n",
        result.baselineValue,
        result.thresholdValue
      );

      Serial.printf(
        "High Samples: %.1f%% (%d sampel)\n",
        result.highPercent,
        result.highCount
      );

      Serial.printf(
        "Max>Thr     : %s\n",
        result.adcMax > result.thresholdValue
          ? "YA"
          : "tidak"
      );

      Serial.printf(
        "STATUS      : %s\n",
        result.dustStatus
          ? "DEBU TERDETEKSI!"
          : "DEBU TIDAK TERDETEKSI"
      );

      Serial.printf(
        "SERVO       : %s\n",
        result.dustStatus
          ? "WIPER AKTIF"
          : "KEMBALI / DIAM"
      );

      Serial.printf(
        "RELAY GPIO27: %s\n",
        relayRaw == HIGH ? "HIGH (ON)" : "LOW (OFF)"
      );

      Serial.printf(
        "ACS712 RAW  : %d\n",
        acsRaw
      );
    }
  }
}

// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(115200);

  // Dust sensor
  pinMode(DUST_LED, OUTPUT);
  digitalWrite(DUST_LED, HIGH);

  // ADC configuration
  analogReadResolution(12);

  analogSetPinAttenuation(
    DUST_ANALOG,
    ADC_11db
  );

  analogSetPinAttenuation(
    ACS712_PIN,
    ADC_11db
  );

  pinMode(ACS712_PIN, INPUT);

  // Relay initialization
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Servo initialization
  wiper.setPeriodHertz(50);

  wiper.attach(
    SERVO_PIN,
    500,
    2400
  );

  wiper.write(SERVO_MIN);

  // Calibration
  calibrateSensor();

  // Create FreeRTOS Queue
  sensorQueue = xQueueCreate(
    1,
    sizeof(SensorData)
  );

  outputQueue = xQueueCreate(
    1,
    sizeof(ProcessedData)
  );

  // Create Event Group
  systemEvent = xEventGroupCreate();

  if (
    sensorQueue == NULL ||
    outputQueue == NULL ||
    systemEvent == NULL
  ) {

    Serial.println("ERROR: Gagal membuat RTOS objects!");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("\n=== FREERTOS STARTED ===");

  // ------------------------------------------
  // CREATE TASKS
  // ------------------------------------------

  xTaskCreatePinnedToCore(
    sensorTask,
    "SensorTask",
    4096,
    NULL,
    3,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    processingTask,
    "ProcessingTask",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    servoTask,
    "ServoTask",
    2048,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    serialTask,
    "SerialTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("Semua task berhasil dibuat.");
}

// ==================================================
// LOOP
// ==================================================

void loop() {

  vTaskDelay(pdMS_TO_TICKS(1000));
}
