// ==========================================
// Autonomous Evacuation Rover (AER) - ESP32 Firmware
// Firmware Pengendali Motor Diferensial dengan Watchdog Pengaman
// ==========================================

// KONFIGURASI PIN MOTOR
// Motor A (Kanan / Motor A)
#define IN1 25
#define IN2 14  
#define ENCA_A 32
#define ENCB_A 33

// Motor B (Kiri / Motor B)
#define IN3 4   
#define IN4 16  
#define ENCA_B 19  
#define ENCB_B 23  

// SPESIFIKASI MOTOR 
const float PPR = 11.0;            
const float GEAR_RATIO = 34.0;     
const float TICK_PER_REV = PPR * GEAR_RATIO; // Total tick per 1 rotasi

// VARIABEL ENCODER & WAKTU
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

long prevCountA = 0;
long prevCountB = 0;
unsigned long prevTime = 0;
const int intervalPID = 50; // Update PID setiap 50 milidetik (20 Hz)

// ==========================================
// WATCHDOG PENGAMAN SERIAL (FAIL-SAFE)
// ==========================================
// Jika tidak ada perintah dari Raspberry Pi selama 600 ms,
// motor WAJIB berhenti seketika secara otomatis!
const unsigned long CMD_TIMEOUT_MS = 600;
unsigned long lastCmdTime = 0;

// TARGET DARI RASPBERRY PI
float targetRpmA = 0.0;
float targetRpmB = 0.0;

// PARAMETER TUNING PID
float Kp = 1.5;  // Proportional
float Ki = 0.05; // Integral
float Kd = 0.1;  // Derivative

// VARIABEL PENYIMPANAN ERROR PID
float eA_prev = 0, eB_prev = 0;
float integralA = 0, integralB = 0;

// INTERRUPT ENCODER
void IRAM_ATTR readEncoderA() {
  if (digitalRead(ENCB_A) == HIGH) { encoderCountA++; } else { encoderCountA--; }
}
void IRAM_ATTR readEncoderB() {
  if (digitalRead(ENCB_B) == HIGH) { encoderCountB++; } else { encoderCountB--; }
}

// FUNGSI PENGGERAK MOTOR
void gerakMotorA(int pwm) {
  if (pwm > 0) { analogWrite(IN1, pwm); analogWrite(IN2, 0); } 
  else if (pwm < 0) { analogWrite(IN1, 0); analogWrite(IN2, -pwm); } 
  else { analogWrite(IN1, 0); analogWrite(IN2, 0); }
}

void gerakMotorB(int pwm) {
  if (pwm > 0) { analogWrite(IN3, pwm); analogWrite(IN4, 0); } 
  else if (pwm < 0) { analogWrite(IN3, 0); analogWrite(IN4, -pwm); } 
  else { analogWrite(IN3, 0); analogWrite(IN4, 0); }
}

void setup() {
  // 1. PENGAMAN BOOT PERTAMA: Langsung kunci semua pin motor ke LOW (0 PWM)
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  gerakMotorA(0);
  gerakMotorB(0);

  Serial.begin(115200);

  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCB_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), readEncoderB, RISING);

  delay(500); 
  // Pastikan motor tetap mati setelah inisialisasi
  gerakMotorA(0);
  gerakMotorB(0);
  lastCmdTime = millis();

  Serial.println("Sistem PID ESP32 Aktif!");
}

// PERHITUNGAN PID & RPM
void hitungDanEksekusiPID() {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - prevTime;

  if (dt >= intervalPID) {
    long deltaTickA = encoderCountA - prevCountA;
    long deltaTickB = encoderCountB - prevCountB;

    float currentRpmA = ((float)deltaTickA / TICK_PER_REV) * (60000.0 / dt);
    float currentRpmB = ((float)deltaTickB / TICK_PER_REV) * (60000.0 / dt);

    float eA = targetRpmA - currentRpmA;
    float eB = targetRpmB - currentRpmB;

    integralA += (eA * dt);
    integralB += (eB * dt);
    integralA = constrain(integralA, -5000, 5000); 
    integralB = constrain(integralB, -5000, 5000);

    float derivativeA = (eA - eA_prev) / dt;
    float derivativeB = (eB - eB_prev) / dt;

    float outputPwmA = (Kp * eA) + (Ki * integralA) + (Kd * derivativeA);
    float outputPwmB = (Kp * eB) + (Ki * integralB) + (Kd * derivativeB);

    // Pengaman jika target 0 atau mendekati 0
    if (abs(targetRpmA) < 0.1) { outputPwmA = 0; integralA = 0; }
    if (abs(targetRpmB) < 0.1) { outputPwmB = 0; integralB = 0; }

    int finalPwmA = constrain((int)outputPwmA, -255, 255);
    int finalPwmB = constrain((int)outputPwmB, -255, 255);

    gerakMotorA(finalPwmA);
    gerakMotorB(finalPwmB);

    // Format Telemetri: DATA:TickA,TickB,RpmA,RpmB
    Serial.print("DATA:");
    Serial.print(encoderCountA); Serial.print(",");
    Serial.print(encoderCountB); Serial.print(",");
    Serial.print(currentRpmA); Serial.print(",");
    Serial.println(currentRpmB);

    eA_prev = eA;
    eB_prev = eB;
    prevCountA = encoderCountA;
    prevCountB = encoderCountB;
    prevTime = currentTime;
  }
}

void loop() {
  // WATCHDOG PENGAMAN: Jika Raspberry Pi putus / tidak kirim perintah > 600ms, MATIKAN MOTOR!
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    targetRpmA = 0.0;
    targetRpmB = 0.0;
  }

  // BACA PERINTAH DARI RASPBERRY PI
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data == "berhenti") {
      targetRpmA = 0;
      targetRpmB = 0;
      lastCmdTime = millis();
    } 
    else if (data.startsWith("speed(")) {
      float tA = 0, tB = 0;
      if (sscanf(data.c_str(), "speed(%f,%f)", &tA, &tB) == 2) {
        targetRpmA = tA;
        targetRpmB = tB;
        lastCmdTime = millis();
      }
    }
  }

  // JALANKAN RUTIN PID DI BACKGROUND
  hitungDanEksekusiPID();
}
