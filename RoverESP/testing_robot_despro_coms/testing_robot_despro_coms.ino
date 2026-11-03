// ==========================================
// KONFIGURASI PIN MOTOR (Sesuai Desain Final)
// ==========================================
// Motor A (Kiri)
#define IN1 25
#define IN2 14  
#define ENCA_A 32
#define ENCB_A 33

// Motor B (Kanan)
#define IN3 4   
#define IN4 16  
#define ENCA_B 19  
#define ENCB_B 23  

// ==========================================
// SPESIFIKASI MOTOR 
// ==========================================
const float PPR = 11.0;            
const float GEAR_RATIO = 34.0;     
const float TICK_PER_REV = PPR * GEAR_RATIO; // Total tick per 1 rotasi

// ==========================================
// VARIABEL ENCODER & WAKTU
// ==========================================
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

long prevCountA = 0;
long prevCountB = 0;
unsigned long prevTime = 0;
const int intervalPID = 50; // Update PID setiap 50 milidetik (20 Hz)

// ==========================================
// VARIABEL PID CONTROLLER
// ==========================================
// TARGET DARI RASPBERRY PI
float targetRpmA = 0.0;
float targetRpmB = 0.0;

// PARAMETER TUNING PID (Wajib disesuaikan dengan beban fisik robot)
float Kp = 1.5;  // Proportional (Respon instan)
float Ki = 0.05; // Integral (Koreksi akumulasi error)
float Kd = 0.1;  // Derivative (Peredam getaran/overshoot)

// VARIABEL PENYIMPANAN ERROR PID
float eA_prev = 0, eB_prev = 0;
float integralA = 0, integralB = 0;

// ==========================================
// INTERRUPT ENCODER
// ==========================================
void IRAM_ATTR readEncoderA() {
  if (digitalRead(ENCB_A) == HIGH) { encoderCountA++; } else { encoderCountA--; }
}
void IRAM_ATTR readEncoderB() {
  if (digitalRead(ENCB_B) == HIGH) { encoderCountB++; } else { encoderCountB--; }
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(ENCA_A, INPUT_PULLUP); pinMode(ENCB_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP); pinMode(ENCB_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), readEncoderB, RISING);

  // Tunggu koneksi serial stabil
  delay(1000); 
  Serial.println("Sistem PID ESP32 Aktif!");
}

// ==========================================
// FUNGSI PENGGERAK MOTOR (Aman dari Bug)
// ==========================================
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

// ==========================================
// FUNGSI UTAMA: PERHITUNGAN PID & RPM
// ==========================================
void hitungDanEksekusiPID() {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - prevTime;

  // Jalankan rutin PID hanya pada interval yang ditentukan
  if (dt >= intervalPID) {
    // 1. Baca selisih tick dan hitung RPM Aktual
    long deltaTickA = encoderCountA - prevCountA;
    long deltaTickB = encoderCountB - prevCountB;

    float currentRpmA = ((float)deltaTickA / TICK_PER_REV) * (60000.0 / dt);
    float currentRpmB = ((float)deltaTickB / TICK_PER_REV) * (60000.0 / dt);

    // 2. Hitung Error (Target - Aktual)
    float eA = targetRpmA - currentRpmA;
    float eB = targetRpmB - currentRpmB;

    // 3. Hitung Integral & Anti-Windup (Batas Maksimal Akumulasi)
    integralA += (eA * dt);
    integralB += (eB * dt);
    integralA = constrain(integralA, -5000, 5000); 
    integralB = constrain(integralB, -5000, 5000);

    // 4. Hitung Derivative
    float derivativeA = (eA - eA_prev) / dt;
    float derivativeB = (eB - eB_prev) / dt;

    // 5. Kalkulasi Rumus PID 
    float outputPwmA = (Kp * eA) + (Ki * integralA) + (Kd * derivativeA);
    float outputPwmB = (Kp * eB) + (Ki * integralB) + (Kd * derivativeB);

    // 6. Pengaman jika target 0 (Reset Integral agar motor tidak mendengung)
    if (targetRpmA == 0) { outputPwmA = 0; integralA = 0; }
    if (targetRpmB == 0) { outputPwmB = 0; integralB = 0; }

    // 7. Batasi nilai Output PID menjadi rentang PWM (-255 sampai 255)
    int finalPwmA = constrain((int)outputPwmA, -255, 255);
    int finalPwmB = constrain((int)outputPwmB, -255, 255);

    // 8. Eksekusi penggerak motor
    gerakMotorA(finalPwmA);
    gerakMotorB(finalPwmB);

    // 9. Kirim Data Telemetri ke Raspberry Pi via Serial
    // Format: DATA:TickA,TickB,RpmA,RpmB
    Serial.print("DATA:");
    Serial.print(encoderCountA); Serial.print(",");
    Serial.print(encoderCountB); Serial.print(",");
    Serial.print(currentRpmA); Serial.print(",");
    Serial.println(currentRpmB);

    // 10. Update riwayat untuk loop berikutnya
    eA_prev = eA;
    eB_prev = eB;
    prevCountA = encoderCountA;
    prevCountB = encoderCountB;
    prevTime = currentTime;
  }
}

void loop() {
  // BACA PERINTAH DARI RASPBERRY PI
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    if (data == "berhenti") {
      targetRpmA = 0;
      targetRpmB = 0;
    } 
    else if (data.startsWith("speed(")) {
      float tA = 0, tB = 0;
      if (sscanf(data.c_str(), "speed(%f,%f)", &tA, &tB) == 2) {
        targetRpmA = tA;
        targetRpmB = tB;
      }
    }
  }

  // JALANKAN RUTIN PID TERUS MENERUS DI BACKGROUND
  hitungDanEksekusiPID();
}