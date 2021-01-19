#include <Arduino.h>
#include <pins_arduino.h>
#include <ESP8266WiFi.h>
#include "ArduinoOSC/ArduinoOSC.h"
#include "network.h"
#define UP 14
#define DOWN 15
#define LEFT 13
#define RIGHT 12
#define ENTER 4
#define LED 16
#define BZ 5

#define OPR_UP (1 << 0)
#define OPR_DOWN (1 << 1)
#define OPR_LEFT (1 << 2)
#define OPR_RIGHT (1 << 3)
#define OPR_ENTER (1 << 4)
#define OPR_ENTER_LONG (1 << 5)

#define BUFFER_LEN 50
#define UPDATE_INTERVAL 50
#define MAX_ALLOWED_GAP_TIME 1000

volatile int8_t opr = 0;
unsigned long lastOpr = 0;

unsigned long lastPulse = 0;
unsigned long lastTick = 0;
unsigned long lastEnterPushed = 0;

int8_t mode = 0;
int8_t prevMode = 0;

int16_t p = 0;
int vals[BUFFER_LEN];
unsigned long ts[BUFFER_LEN];

unsigned long tmpT, prevT, pulse;
long pulseTs[10];

void ICACHE_RAM_ATTR enter_push() {
  lastEnterPushed = millis();
}
void ICACHE_RAM_ATTR enter() {
  opr |= lastEnterPushed > millis() - 500 ? OPR_ENTER : OPR_ENTER_LONG;
}
void ICACHE_RAM_ATTR left() {
  opr |= OPR_LEFT;
}
void ICACHE_RAM_ATTR right() {
  opr |= OPR_RIGHT;
}
void ICACHE_RAM_ATTR up() {
  opr |= OPR_UP;
}
void ICACHE_RAM_ATTR down() {
  opr |= OPR_DOWN;
}

void beep(unsigned long t, uint16_t freq, uint16_t length, uint16_t cycle, unsigned long origin = 0) {
  if (floor((t - origin) / cycle) != floor((lastTick - origin) / cycle)) {
    tone(BZ, freq, length);
  }
}

void blink(unsigned long t, uint16_t length, uint16_t cycle, unsigned long origin = 0) {
  if ((t - origin) % cycle < length) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}

void onOscReceived(const OscMessage& m)
{
    mode = m.arg<int>(0);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BZ, OUTPUT);

  attachInterrupt(ENTER, enter_push, RISING);
  attachInterrupt(ENTER, enter, FALLING);
  attachInterrupt(UP, up, FALLING);
  attachInterrupt(DOWN, down, FALLING);
  attachInterrupt(LEFT, left, FALLING);
  attachInterrupt(RIGHT, right, FALLING);

  WiFi.begin(SSID, NET_PW);
  while (WiFi.status() != WL_CONNECTED)
    {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);
    }

  Serial.begin(115200);
  Serial.println(WiFi.localIP());
  OscWiFi.send(HOST, PORT, "/setup", 0);
  OscWiFi.subscribe(PORT, "/mode", onOscReceived);

  tone(BZ, 880, 500);
  digitalWrite(2, HIGH);
  digitalWrite(LED, HIGH);
}

void loop() {
  unsigned long now = millis();
  bool updated = false;

  if (floor(lastTick / UPDATE_INTERVAL) != floor(now / UPDATE_INTERVAL)) {
    p++;
    if (p >= BUFFER_LEN) { p = 0; }

    vals[p] = analogRead(A0);
    ts[p] = now;
    updated = true;
  }
    
  uint16_t min = 1024;
  uint16_t max = 0;
  uint16_t min_short = 1024;
  uint16_t max_short = 0;

  for (uint16_t i = 0; i < BUFFER_LEN; i++) {
    uint16_t j = p - i >= 0 ? (p - i) : (p - i + BUFFER_LEN);
    if (vals[j] < min) {
      min = vals[j];
    }
    if (max < vals[j]) {
      max = vals[j];
    }

    if (i < MAX_ALLOWED_GAP_TIME / UPDATE_INTERVAL) {
      if (vals[j] < min_short) {
        min_short = vals[j];
      }
      if (max_short < vals[j]) {
        max_short = vals[j];
      }
    }
  }

  const uint16_t range = max - min;
  const uint16_t range_short = max_short - min_short;
  const uint16_t thr = max - range / 4;
  uint16_t prev = vals[p];
  uint8_t pulseTsSize = 0;

  for (uint16_t i = 1; i < BUFFER_LEN; i++) {
    uint16_t j = p - i >= 0 ? (p - i) : (p - i + BUFFER_LEN);
    if (vals[j] < thr && prev >= thr) {
      pulseTs[pulseTsSize] = ts[j];
      pulseTsSize++;
      if (pulseTsSize >= 10) break;
    }
    prev = vals[j];
  }

  if (pulseTsSize >= 2){
    tmpT = 0;
    prevT = pulseTs[0];
    uint8_t cnt = 0;
    for (uint16_t i = 1; i < pulseTsSize; i++) {
      tmpT += (prevT - pulseTs[i]);
      prevT = pulseTs[i];
      cnt++;
    }
    if (cnt != 0) {
      pulse = tmpT / cnt;
    } else {
      pulse = 0;
    }
  }

  if (
    60 * 1000 / 250 < pulse && pulse <= 60 * 1000 / 50
  ) {
    lastPulse = now;
  }

  if (mode == 0) {
    digitalWrite(LED, HIGH);
  } else if (mode == 1) {
    if (vals[p - 1 >= 0 ? (p - 1) : (p - 1 + BUFFER_LEN)] >= thr) {
      if(vals[p] < thr) {
        tone(BZ, 1760 * 2, 50);
      }
      digitalWrite(LED, HIGH);
    } else {
      digitalWrite(LED, LOW);
    }

    if (floor(lastTick / 1000) != floor(now / 1000)) {
      OscWiFi.send(HOST, PORT, "/hr", (int)(pulse != 0 ? 60 * 1000 / pulse : -1));
    }
  } else if (mode == 2) {
    if (lastPulse < now - 5000 || range_short < 300) {
      blink(now, 200, 400);
      beep(now, 880 * 4, 100, 2000, 0);
      beep(now, 880 * 1, 100, 2000, 300);

      if (floor(lastTick / 1000) != floor(now / 1000)) {
        OscWiFi.send(HOST, PORT, "/hr_invalid");
      }
    } else {
      if (floor(lastTick / 1000) != floor(now / 1000)) {
        OscWiFi.send(HOST, PORT, "/hr", (int)(pulse != 0 ? 60 * 1000 / pulse : -1));
      }
    }
  }

  if (prevMode != mode) {
    tone(BZ, 1000, 500);
    prevMode = mode;
  }

  if (opr > 0 && now - lastOpr > 150) {
    if (opr & OPR_ENTER_LONG) {
      tone(BZ, 3520, 500);
    } else {
      tone(BZ, 3520, 50);
    }
    OscWiFi.send(HOST, PORT, "/opr", (int32_t)opr);
    lastOpr = now;
  }

  opr = 0;

  if (updated) {
    Serial.printf("%d %d %d\n", vals[p], range, range_short);
  }

  if (floor(lastTick / 3000) != floor(now / 3000)) {
    OscWiFi.send(HOST, PORT, "/setup", 0);
  }

  lastTick = now;
  OscWiFi.update();
}
