/*
 * Copyright (C) 2026 Robson Filgueiras.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <FS.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Controle de Placa
#define USE_CYD_BOARD 1


  #include <Adafruit_FT6206.h>


#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

/* ================================================================
   PROJETO:     GeoTank LaserScan v3.0
   PLATAFORMA:  ESP32 DevKit C V4
   DESCRIÇÃO:   Perfilometria de fundo de tanque por laser/ultrassônico.
                Varredura centrípeta com cota topográfica adaptativa.
                Referência angular: θ a partir do eixo horizontal.
                Cota = H - L × sin(θ). Negativo = recalque.
   ================================================================ */

// ── Flag de Placa ────────────────────────────────────────────────
// Mude para 0 para usar o hardware original (ESP32 DevKit + FT6206)
// Mude para 1 para testar a UI na placa ESP32-2432S028R (CYD - Cheap Yellow Display)
// (Definido no topo do arquivo)

// ── Pinagem ──────────────────────────────────────────────────────

  #define TFT_CS    15
  #define TFT_DC     2
  #define TFT_RST    4
  #define PIN_STEP  25
  #define PIN_DIR   26
  #define PIN_ENA   27
  #define PIN_SD_CS  5


// Simulador vs Hardware Real
#define SIMULACAO_WOKWI true  // Mude para false no ESP32 físico

// Pinos Ultrassônico (Wokwi)
#define PIN_TRIG  13
#define PIN_ECHO  12

// Pinos Laser UART RS232/RS485 (Hardware Real)
#define PIN_LASER_RX 17
#define PIN_LASER_TX 16

// Buzzer de feedback
#define PIN_BUZZER 32

// ── Periféricos ──────────────────────────────────────────────────

  Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
  Adafruit_FT6206  ts;


Adafruit_MPU6050 mpu;

// ── Paleta de Cores ──────────────────────────────────────────────
#define C_BG       0x0000
#define C_HEADER   0x18E3
#define C_FOOTER   0x18E3
#define C_BTN      0x2945
#define C_BTN_HI   0x3186
#define C_BORDER   0x4A69
#define C_ACCENT   0xFD20
#define C_START    0x14E2
#define C_WHITE    0xFFFF
#define C_DIM      0x7BEF
#define C_GREEN    0x07E0
#define C_RED      0xF800
#define C_YELLOW   0xFFE0
#define C_CYAN     0x07FF

// ── Máquina de Estados ──────────────────────────────────────────
enum Screen {
    SCR_MAIN_MENU,
    SCR_BOTTOM_MENU,
    SCR_SETUP,       // Captura manual da quina (motor livre)
    SCR_CHECKPOINT,
    SCR_SCANNING,
    SCR_HORIZON      // Nivelamento no horizonte
};

// ── Zonas de Toque ──────────────────────────────────────────────
struct Zone {
    int16_t x, y, w, h;
};

// ── Estrutura de Dados por Ponto ────────────────────────────────
struct MeasPoint {
    int      mp;
    float    x_m;
    float    theta_deg;
    float    l_m;
    float    cota_mm;
};

// ── Constantes de Motor ─────────────────────────────────────────
// A4988: 1/16 microstepping com motor 200 passos/rev = 3200 pulsos/rev
#define STEPS_PER_REV    3200.0f
#define STEPS_PER_DEG    (STEPS_PER_REV / 360.0f)  // ~8.89 passos/grau
#define MOTOR_PULSE_US   1500  // Largura do pulso STEP (aumentado para visibilidade no Wokwi)

// ── Resolução de Passo Radial (metros) ──────────────────────────
// Índice 0=Baixa, 1=Média, 2=Alta
const float RES_STEP[] = {0.25f, 0.10f, 0.05f};
const char* RES_LABEL[] = {"250 mm (Fast)", "100 mm (Normal)", "50 mm (Fine)"};

// ── Tolerância de Correção (metros) ─────────────────────────────
#define R_TOLERANCE  0.003f   // ±3mm (Apertado para o laser novo)

// ── Variáveis Globais ───────────────────────────────────────────
Screen   activeScreen = SCR_MAIN_MENU;
bool     screenDirty  = true;

bool     imuOnline    = false;
bool     scanOnline   = false;
bool     sdOnline     = false;

// Menu
bool     isCalibrated = false;
int      resLevel     = 1;       // 0=Baixa, 1=Média, 2=Alta
int      radialCount  = 8;       // Mínimo normativo API 653

// Calibração do Zero (Nadir)
bool     zeroCalibrated = false; // H foi medido no nadir?
float    setupH       = 0.0f;   // Altura real laser→piso medida no nadir (m)
float    setupL_zero  = 0.0f;   // Distância L medida no nadir (m)

// Captura da Quina
float    setupTheta   = 0.0f;   // Ângulo capturado na quina (graus, a partir da horizontal)
float    setupL       = 0.0f;   // Distância capturada na quina (m)
float    setupRmax    = 0.0f;   // Raio horizontal máximo neste azimute (m)
float    setupCotaQ   = 0.0f;   // Cota na quina (mm) — já calculada com H calibrado
bool     quinaCaptured = false;

// Controle de Fluxo Radial
int      currentRadial = 1;

// Estado da Varredura
int      scanIndex    = 0;
int      scanTotalPts = 0;
float    lastZ        = 1.0f;   // Última profundidade Z medida (inicia com estimativa)
float    currentMotorAngle = 0.0f;  // Posição angular atual do motor (graus)
bool     scanRunning  = false;
bool     confirmingStop = false; // Flag para popup de confirmação
unsigned long lastScanStepMs = 0;
int      reshots      = 0;      // Contagem de re-disparos na Survey Line atual

// Último ponto coletado (para telemetria)
MeasPoint lastPoint = {0, 0.0f, 0.0f, 0.0f, 0.0f};

// Arquivo SD
File     dataFile;
char     fileName[32];

// ── Zonas de Interação ──────────────────────────────────────────
// Main Menu
const int MAIN_MENU_ZONE_COUNT = 3;
Zone mainMenuZones[MAIN_MENU_ZONE_COUNT] = {
    { 10,  35, 300, 50 },   // BOTTOM SCAN
    { 10,  95, 300, 50 },   // SHELL SCAN
    { 10, 155, 300, 50 }    // HORIZON LEVEL
};

// Bottom Menu
const int BOTTOM_MENU_ZONE_COUNT = 4;
Zone bottomMenuZones[BOTTOM_MENU_ZONE_COUNT] = {
    { 10,  35, 300, 50 },   // SURVEY LINES
    { 10,  95, 300, 50 },   // RESOLUTION
    { 10, 155,  50, 50 },   // BACK (index 2)
    { 70, 155, 240, 50 }    // START PROJECT (index 3)
};

// Setup
Zone setupCaptureZone = {10, 120, 300, 45};   // CAPTURAR QUINA
Zone setupAbortZone   = {10, 175, 300, 40};    // ABORTAR

// Checkpoint
Zone checkScanZone      = {10, 100, 300, 45};
Zone checkUndoZone      = {10, 160, 145, 45};
Zone checkAbortHalfZone = {165, 160, 145, 45};
Zone checkAbortFullZone = {10, 160, 300, 45};
Zone checkFinishZone    = {10, 130, 300, 50};

// Scanning
Zone scanStopZone = {10, 185, 300, 40};
Zone confirmNoZone  = {10, 140, 145, 50};
Zone confirmYesZone = {165, 140, 145, 50};

// ══════════════════════════════════════════════════════════════════
//  FUNÇÕES UTILITÁRIAS DE DISPLAY
// ══════════════════════════════════════════════════════════════════

void drawCentered(const char* str, int16_t cx, int16_t cy) {
    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds(str, 0, 0, &x1, &y1, &tw, &th);
    tft.setCursor(cx - (tw / 2) - x1, cy - (th / 2) - y1);
    tft.print(str);
}

void drawRightAligned(const char* str, int16_t rx, int16_t cy) {
    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds(str, 0, 0, &x1, &y1, &tw, &th);
    tft.setCursor(rx - tw - x1, cy - (th / 2) - y1);
    tft.print(str);
}

void drawButton(int16_t x, int16_t y, int16_t w, int16_t h,
                uint16_t bg, uint16_t border,
                const char* line1, const char* line2) {
    tft.fillRoundRect(x, y, w, h, 4, bg);
    tft.drawRoundRect(x, y, w, h, 4, border);

    if (line2 != NULL) {
        tft.setFont(&FreeSansBold9pt7b);
        tft.setTextColor(C_WHITE);
        drawCentered(line1, x + w / 2, y + h / 2 - 8);
        
        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_DIM);
        drawCentered(line2, x + w / 2, y + h / 2 + 10);
    } else {
        tft.setFont(&FreeSansBold9pt7b);
        tft.setTextColor(C_WHITE);
        drawCentered(line1, x + w / 2, y + h / 2);
    }
}

void drawHeader() {
    tft.fillRect(0, 0, 320, 24, C_HEADER);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(4, 17);
    tft.print("GeoTank v3.0");

    tft.setFont(NULL);
    tft.setTextSize(1);
    int16_t xr = 316;
    
    tft.setTextColor(C_GREEN);
    drawRightAligned("85%", xr, 12);
    xr -= 30;

    tft.setTextColor(imuOnline ? C_GREEN : C_RED);
    drawRightAligned("IMU", xr, 12);
    xr -= 28;

    tft.setTextColor(sdOnline ? C_GREEN : C_RED);
    drawRightAligned("SD", xr, 12);
    xr -= 24;

    tft.setTextColor(scanOnline ? C_GREEN : C_YELLOW);
    drawRightAligned("SCAN", xr, 12);
}

void drawFooter(const char* msg) {
    tft.fillRect(0, 218, 320, 22, C_FOOTER);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    tft.setCursor(4, 226);
    tft.print(msg);
}

void beep(int ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(ms);
    digitalWrite(PIN_BUZZER, LOW);
}

void beepSuccess() {
    beep(100); delay(100);
    beep(150);
}

void blinkError(const char* msg) {
    for (int i = 0; i < 6; i++) {
        tft.fillRect(0, 218, 320, 22, C_FOOTER);
        if (i % 2 == 0) {
            tft.setFont(NULL);
            tft.setTextSize(1);
            tft.setTextColor(C_RED);
            tft.setCursor(4, 226);
            tft.print(msg);
            digitalWrite(PIN_BUZZER, HIGH);
            delay(200);
            digitalWrite(PIN_BUZZER, LOW);
            delay(300);
        } else {
            delay(500);
        }
    }
    screenDirty = true;
}

// ══════════════════════════════════════════════════════════════════
//  CONTROLE DO MOTOR DE PASSO
// ══════════════════════════════════════════════════════════════════

void enableMotor() {
    digitalWrite(PIN_ENA, LOW);    // A4988: LOW = ativo
}

void disableMotor() {
    digitalWrite(PIN_ENA, HIGH);   // A4988: HIGH = livre (sem torque)
}

void moveMotorToAngle(float targetDeg) {
    float delta = targetDeg - currentMotorAngle;
    int steps = abs((int)(delta * STEPS_PER_DEG));
    
    // DIR: HIGH = sentido horário (aumenta θ, desce), LOW = anti-horário (sobe)
    digitalWrite(PIN_DIR, delta > 0 ? HIGH : LOW);
    
    for (int i = 0; i < steps; i++) {
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(MOTOR_PULSE_US);
        digitalWrite(PIN_STEP, LOW);
        delayMicroseconds(MOTOR_PULSE_US);
    }
    
    currentMotorAngle = targetDeg;
}

// ══════════════════════════════════════════════════════════════════
//  SENSOR DE DISTÂNCIA
// ══════════════════════════════════════════════════════════════════

float readDistanceMM() {
#if SIMULACAO_WOKWI
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long duration = pulseIn(PIN_ECHO, HIGH, 50000);
    if (duration == 0) return -1.0f;
    return (duration * 0.343f) / 2.0f;  // Retorna em mm
#else
    // Limpa buffer de entrada da UART2
    while (Serial2.available()) Serial2.read();
    
    // Envia comando genérico de leitura única (ASCII)
    // O comando exato dependerá do datasheet do laser (ex: 'D', 'O', ou frame HEX)
    Serial2.print("D\r\n");
    
    long timeout = millis();
    String response = "";
    
    while (millis() - timeout < 500) {
        if (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\n') break;
            if (isDigit(c) || c == '.' || c == '-') {
                response += c;
            }
        }
    }
    
    if (response.length() > 0) {
        float dist_m = response.toFloat(); 
        if (dist_m > 0) return dist_m * 1000.0f; // Retorna em mm
    }
    return -1.0f; // Falha de leitura
#endif
}

// ══════════════════════════════════════════════════════════════════
//  LEITURA DO IMU (Ângulo de Inclinação)
// ══════════════════════════════════════════════════════════════════

float readIMUAngleDeg() {
#if SIMULACAO_WOKWI
    // Na simulação, não podemos "girar o sensor na mão" tão facilmente em tempo de execução.
    // Assim, se o H já estiver calibrado, calculamos a inclinação (theta) que faria sentido
    // para a distância hipotética que o usuário ajustou no slider do ultrassônico.
    if (zeroCalibrated && setupH > 0.1f) {
        float L_m = readDistanceMM() / 1000.0f;
        if (L_m >= setupH) {
            // Theta é o ângulo em relação à vertical (0° = para baixo)
            return acosf(setupH / L_m) * (180.0f / M_PI);
        }
    }
    return 0.0f; // 0 graus = Vertical (Nadir)
#else
    if (!imuOnline) return 0.0f;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calcula o ângulo em relação à vertical (Nadir = 0°)
    // Se ax é a aceleração no eixo do feixe, quando apontado para baixo ax = 1g, az = 0
    // Logo, o ângulo a partir do nadir é calculado baseando-se no desvio da gravidade.
    // atan2f(z, x) te dá o ângulo do vetor gravidade em relação ao sensor.
    float angleDeg = atan2f(a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
    return fabsf(angleDeg); // O valor exato depende da montagem física do IMU, mas ajustado para que Nadir seja ~0
#endif
}

// ══════════════════════════════════════════════════════════════════
//  GRAVAÇÃO NO SD CARD
// ══════════════════════════════════════════════════════════════════

void openSDFile() {
    int ptIndex = currentRadial - 1;
    sprintf(fileName, "/PT%02d.csv", ptIndex);
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        char ptLabel[48];
        getPointLabel(currentRadial, radialCount, ptLabel);
        dataFile.print("# ");
        dataFile.println(ptLabel); // Inclui o azimute no cabeçalho (ex: # PONTO 0 (N))
        dataFile.println("MP,X_m,Theta_deg,L_m,Z_mm");
        dataFile.flush();
    }
}

void writePointToSD(MeasPoint &pt) {
    if (dataFile) {
        char line[80];
        sprintf(line, "%d,%.3f,%.4f,%.4f,%.1f",
                pt.mp, pt.x_m, pt.theta_deg, pt.l_m, pt.cota_mm);
        dataFile.println(line);
        dataFile.flush();
    }
    // Eco na Serial para debug
    Serial.printf("MP%d | X=%.3fm | θ=%.2f° | L=%.3fm | Z=%.1fmm\n",
                  pt.mp, pt.x_m, pt.theta_deg, pt.l_m, pt.cota_mm);
}

void closeSDFile() {
    if (dataFile) {
        dataFile.close();
    }
}

// ══════════════════════════════════════════════════════════════════
//  TELAS: MENUS
// ══════════════════════════════════════════════════════════════════

void drawMainMenu() {
    tft.fillScreen(C_BG);
    drawHeader();

    drawButton(10, 35, 300, 50, C_BTN, C_BORDER,
               "BOTTOM SCAN", "Tank floor mapping");
    drawButton(10, 95, 300, 50, C_DIM, C_BORDER,
               "SHELL SCAN", "Under development");
    drawButton(10, 155, 300, 50, C_START, C_BORDER,
               "HORIZON LEVEL", "Align laser to 90 deg");
    drawFooter("Select a mode.");
}

void drawBottomMenu() {
    tft.fillScreen(C_BG);
    drawHeader();

    char strRad[32];
    sprintf(strRad, "%d lines (min 8)", radialCount);

    drawButton(10, 35, 300, 50, C_BTN, C_BORDER,
               "SURVEY LINES", strRad);
    drawButton(10, 95, 300, 50, C_BTN, C_BORDER,
               "RESOLUTION", RES_LABEL[resLevel]);
               
    // BACK (50px, ~1/6) na esquerda, START PROJECT (240px, ~5/6) na direita
    drawButton(10, 155, 50, 50, C_RED, C_BORDER,
               "BACK", NULL);
    drawButton(70, 155, 240, 50, C_START, C_BORDER,
               "START PROJECT", "Lock and proceed");
               
    drawFooter("Tap panels to configure.");
}

void drawHorizonScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    tft.setTextColor(C_CYAN);
    tft.setTextSize(2);
    tft.setCursor(60, 80);
    tft.print("HORIZON LEVEL");

    tft.setTextColor(C_WHITE);
    tft.setTextSize(1);
    tft.setCursor(40, 110);
    tft.print("Laser is locked at 90 deg.");

    drawButton(10, 175, 300, 40, C_DIM, C_BORDER, "BACK TO MENU", NULL);
    drawFooter("Motor energized.");
}

// ══════════════════════════════════════════════════════════════════
//  TELA: SETUP — CALIBRAÇÃO DO ZERO + CAPTURA DA QUINA
// ══════════════════════════════════════════════════════════════════

void getPointLabel(int radialIndex, int totalRadials, char* buffer) {
    int pt = radialIndex - 1; // 0-based index
    if (totalRadials == 8) {
        const char* card[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        sprintf(buffer, "POINT %d (%s)", pt, card[pt % 8]);
    } else {
        float ang = pt * (360.0f / totalRadials);
        sprintf(buffer, "POINT %d (%.0f deg)", pt, ang);
    }
}

void drawSetupScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    char strLine[32];
    getPointLabel(currentRadial, radialCount, strLine);

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT);
    drawCentered(strLine, 160, 42);

    // ── ETAPA 1: Calibrar Zero (vertical) ──
    if (!zeroCalibrated) {
        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_DIM);
        drawCentered("Motor is at NADIR (pointing down).", 160, 62);
        drawCentered("The laser will measure tripod height (H).", 160, 74);

        drawButton(setupCaptureZone.x, setupCaptureZone.y,
                   setupCaptureZone.w, setupCaptureZone.h,
                   C_ACCENT, C_BORDER,
                   "CALIBRATE HEIGHT", "Fire laser at nadir");
    }
    // ── ETAPA 2: Capturar Junção (motor livre) ──
    else if (!quinaCaptured) {
        tft.setFont(NULL);
        tft.setTextSize(1);

        // Mostra H calibrado
        tft.setTextColor(C_GREEN);
        char strH[32];
        sprintf(strH, "H = %.3f m (calibrated)", setupH);
        drawCentered(strH, 160, 62);

        // Destaque para Motor Livre
        tft.setTextSize(2);
        tft.setTextColor(C_YELLOW);
        drawCentered("Motor FREE", 160, 80);

        tft.setTextSize(1);
        tft.setTextColor(C_WHITE);
        drawCentered("Point the laser to the FLOOR-WALL JOINT", 160, 102);

        drawButton(setupCaptureZone.x, setupCaptureZone.y,
                   setupCaptureZone.w, setupCaptureZone.h,
                   C_ACCENT, C_BORDER,
                   "CAPTURE JOINT", "Register scanning boundaries");
    }
    // ── ETAPA 3: Confirmar e Iniciar ──
    else {
        tft.setFont(NULL);
        tft.setTextSize(1);

        // Mostra H calibrado
        tft.setTextColor(C_GREEN);
        char info1[48];
        sprintf(info1, "H = %.3f m (calibrated)", setupH);
        drawCentered(info1, 160, 60);

        // Mostra dados da junção
        tft.setTextColor(C_CYAN);
        char info2[48], info3[48];
        sprintf(info2, "Rmax = %.2f m   Joint Z = %.0f mm", setupRmax, setupCotaQ);
        sprintf(info3, "Theta = %.1f    L = %.3f m", setupTheta, setupL);
        drawCentered(info2, 160, 78);
        drawCentered(info3, 160, 93);

        int totalPts = (int)(setupRmax / RES_STEP[resLevel]) + 1;
        char info4[48];
        sprintf(info4, "Expected points: %d (%.0fmm step)", totalPts, RES_STEP[resLevel]*1000);
        tft.setTextColor(C_DIM);
        drawCentered(info4, 160, 108);

        drawButton(setupCaptureZone.x, setupCaptureZone.y,
                   setupCaptureZone.w, setupCaptureZone.h,
                   C_START, C_BORDER,
                   "START SCANNING", "From joint to center");
    }

    drawButton(setupAbortZone.x, setupAbortZone.y,
               setupAbortZone.w, setupAbortZone.h,
               C_RED, C_WHITE, "ABORT", NULL);

    const char* footerMsg = !zeroCalibrated ? "Step 1/3: Calibrate tripod height." :
                            !quinaCaptured  ? "Step 2/3: Aim laser manually." :
                                              "Step 3/3: Ready to scan.";
    drawFooter(footerMsg);
}

// ══════════════════════════════════════════════════════════════════
//  TELA: CHECKPOINT (entre Survey Lines)
// ══════════════════════════════════════════════════════════════════

void drawCheckpointScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    if (currentRadial <= radialCount) {
        char strTitle[48];
        getPointLabel(currentRadial, radialCount, strTitle);
        
        tft.setFont(&FreeSansBold9pt7b);
        tft.setTextColor(C_ACCENT);
        drawCentered(strTitle, 160, 45);

        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_DIM);
        char strInst[48];
        sprintf(strInst, "Rotate tripod to %s", strTitle);
        drawCentered(strInst, 160, 65);
        drawCentered("and tap to capture joint", 160, 77);

        // Resumo da última captura
        if (setupRmax > 0) {
            tft.setTextColor(C_CYAN);
            char strResumo[48];
            sprintf(strResumo, "Last: Rmax=%.2fm  %d pts  %d corr.",
                    setupRmax, scanIndex, reshots);
            drawCentered(strResumo, 160, 85);
        }

        drawButton(checkScanZone.x, checkScanZone.y,
                   checkScanZone.w, checkScanZone.h,
                   C_START, C_BORDER,
                   "POSITION JOINT", "Free motor and capture");

        if (currentRadial > 1) {
            drawButton(checkUndoZone.x, checkUndoZone.y,
                       checkUndoZone.w, checkUndoZone.h,
                       C_BTN, C_BORDER, "REDO", "Erase last line");
            drawButton(checkAbortHalfZone.x, checkAbortHalfZone.y,
                       checkAbortHalfZone.w, checkAbortHalfZone.h,
                       C_RED, C_WHITE, "ABORT", "Destroy project");
        } else {
            drawButton(checkAbortFullZone.x, checkAbortFullZone.y,
                       checkAbortFullZone.w, checkAbortFullZone.h,
                       C_RED, C_WHITE, "ABORT ALL", "Destroy project");
        }
        
        drawFooter("Rotate equipment to next azimuth.");
    } else {
        // Todas as Survey Lines concluídas
        tft.setFont(&FreeSansBold9pt7b);
        tft.setTextColor(C_GREEN);
        drawCentered("ACQUISITION COMPLETE", 160, 70);

        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_DIM);
        char strFinal[48];
        sprintf(strFinal, "%d Survey Lines processed.", radialCount);
        drawCentered(strFinal, 160, 95);

        drawButton(checkFinishZone.x, checkFinishZone.y,
                   checkFinishZone.w, checkFinishZone.h,
                   C_START, C_BORDER,
                   "FINISH AND SAVE", "Close files and return");
                   
        drawFooter("All lines successfully processed.");
    }
}

// ══════════════════════════════════════════════════════════════════
//  TELA: VARREDURA EM PROGRESSO
// ══════════════════════════════════════════════════════════════════

void updateScanningTelemetry() {
    // Limpa área de telemetria
    tft.fillRect(10, 65, 300, 110, C_BG);

    // Linha 1: MP e posição
    char tel1[48];
    sprintf(tel1, "MP %d / %d", lastPoint.mp, scanTotalPts);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered(tel1, 160, 80);

    // Linha 2: X e Cota
    char tel2[48];
    sprintf(tel2, "X=%.2fm   Z=%.0fmm", lastPoint.x_m, lastPoint.cota_mm);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(lastPoint.cota_mm < -50 ? C_RED :
                     lastPoint.cota_mm < -20 ? C_YELLOW : C_GREEN);
    drawCentered(tel2, 160, 105);

    // Linha 3: θ e L
    char tel3[48];
    sprintf(tel3, "Theta=%.2f   L=%.3fm", lastPoint.theta_deg, lastPoint.l_m);
    tft.setTextColor(C_DIM);
    drawCentered(tel3, 160, 120);

    // Linha 4: Re-disparos
    char tel4[32];
    sprintf(tel4, "Corrections: %d", reshots);
    tft.setTextColor(reshots > 0 ? C_YELLOW : C_DIM);
    drawCentered(tel4, 160, 135);

    // Barra de progresso
    float progress = (scanTotalPts > 0) ?
                     (float)lastPoint.mp / (float)scanTotalPts : 0.0f;
    int barW = (int)(280.0f * progress);
    tft.fillRect(20, 152, 280, 8, C_BTN);
    tft.fillRect(20, 152, barW, 8, C_GREEN);
    tft.drawRect(20, 152, 280, 8, C_BORDER);
}

void drawScanningScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    char strPasso[48];
    sprintf(strPasso, "SURVEY LINE %d - BOTTOM", currentRadial);

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT);
    drawCentered(strPasso, 160, 50);

    updateScanningTelemetry();

    if (confirmingStop) {
        // Desenha popup sobrepondo os controles
        tft.fillRect(0, 100, 320, 120, C_BG);
        tft.drawRect(0, 100, 320, 120, C_RED);
        
        tft.setFont(&FreeSansBold9pt7b);
        tft.setTextColor(C_WHITE);
        drawCentered("Do you want to interrupt", 160, 120);
        drawCentered("the scan of this line?", 160, 135);
        
        drawButton(confirmNoZone.x, confirmNoZone.y, confirmNoZone.w, confirmNoZone.h,
                   C_BTN, C_WHITE, "NO", "Resume");
        drawButton(confirmYesZone.x, confirmYesZone.y, confirmYesZone.w, confirmYesZone.h,
                   C_RED, C_WHITE, "YES", "Abort");
                   
        drawFooter("Motor paused awaiting confirmation.");
    } else {
        drawButton(scanStopZone.x, scanStopZone.y,
                   scanStopZone.w, scanStopZone.h,
                   C_RED, C_WHITE, "EMERGENCY STOP", "Pause scanning");

        drawFooter("Motor ACTIVE. Firing laser...");
    }
}

// ══════════════════════════════════════════════════════════════════
//  ALGORITMO DE VARREDURA CENTRÍPETA
// ══════════════════════════════════════════════════════════════════

void startScan() {
    scanIndex = 0;
    reshots = 0;
    lastZ = setupH;
    scanTotalPts = (int)(setupRmax / RES_STEP[resLevel]) + 1;
    scanRunning = true;
    lastScanStepMs = millis();

    enableMotor();

    // O motor já está na posição da quina (setupTheta)
    // A varredura inicia daqui, decrementando X em direção ao centro
    currentMotorAngle = setupTheta;

    openSDFile();
    
    // Feedback sonoro longo para início
    beep(300);
}

void executeScanStep() {
    float step = RES_STEP[resLevel];
    
    // Raio-alvo decrementa da borda para o centro
    float rTarget = setupRmax - (scanIndex * step);
    
    if (rTarget < 0.0f) {
        // Varredura do fundo concluída para esta Survey Line
        finishSurveyLine();
        return;
    }
    
    float thetaEst, L, thetaRad, rReal, zReal, cota;
    bool corrected = false;

    // ── Primeiro disparo ──
    if (rTarget < 0.001f) {
        // Ponto central (nadir): apontado para baixo
        thetaEst = 0.0f;
    } else {
        // Angulo a partir da vertical: tan(theta) = R / Z
        thetaEst = atan2f(rTarget, lastZ) * (180.0f / M_PI);
    }

    moveMotorToAngle(thetaEst);
    delay(50);  // Estabilização mecânica

    L = readDistanceMM() / 1000.0f;  // mm → m
    
    thetaRad = thetaEst * (M_PI / 180.0f);
    rReal = L * sinf(thetaRad); // sin para a horizontal
    zReal = L * cosf(thetaRad); // cos para a vertical
    cota  = (setupH - zReal) * 1000.0f;  // metros → mm

    // ── Malha fechada: verifica erro radial ──
    if (rTarget > 0.001f && fabsf(rReal - rTarget) > R_TOLERANCE) {
        // Recalcula θ com Z real recém-medido
        lastZ = zReal;
        thetaEst = atan2f(rTarget, zReal) * (180.0f / M_PI);
        
        moveMotorToAngle(thetaEst);
        delay(50);
        
        L = readDistanceMM() / 1000.0f;
        
        thetaRad = thetaEst * (M_PI / 180.0f);
        rReal = L * sinf(thetaRad);
        zReal = L * cosf(thetaRad);
        cota  = (setupH - zReal) * 1000.0f;

        reshots++;
        corrected = true;
    }

    // ── Grava ponto ──
    MeasPoint pt;
    pt.mp        = scanIndex;
    pt.x_m       = rReal;
    pt.theta_deg = thetaEst;
    pt.l_m       = L;
    pt.cota_mm   = cota;

    writePointToSD(pt);
    lastPoint = pt;

    // Atualiza Z para próxima iteração
    lastZ = zReal;
    scanIndex++;
}

void finishSurveyLine() {
    scanRunning = false;
    closeSDFile();

    // Retorna motor a vertical (0°) e TRAVA para facilitar a proxima leitura
    moveMotorToAngle(0.0f);
    // disableMotor(); -> Removido a pedido para manter torque


    currentRadial++;
    quinaCaptured = false;
    activeScreen = SCR_CHECKPOINT;
    screenDirty = true;
    
    beepSuccess();
}

// ══════════════════════════════════════════════════════════════════
//  CALIBRAÇÃO DO ZERO (Nadir)
// ══════════════════════════════════════════════════════════════════

void calibrateZero() {
    // Motor energizado no nadir (0°) — apontando direto para baixo
    enableMotor();
    moveMotorToAngle(0.0f);
    delay(200);  // Estabilização

    // Múltiplas leituras para média (reduz ruído do sensor)
    float soma = 0.0f;
    int nLeituras = 5;
    for (int i = 0; i < nLeituras; i++) {
        soma += readDistanceMM();
        delay(50);
    }
    float L_mm = soma / nLeituras;
    float L_m  = L_mm / 1000.0f;

    if (L_mm < 100.0f || L_mm > 15000.0f) {
        blinkError("ERROR: Invalid vertical reading!");
        return;
    }

    // No nadir (θ=0°), L = H diretamente
    setupH      = L_m;
    setupL_zero = L_m;
    zeroCalibrated = true;

    Serial.printf("ZERO: H = %.3f m (average of %d readings)\n", setupH, nLeituras);

    // Calcula o ângulo limite correspondente ao alcance máximo do laser no piso (60m)
    // Se 0° é nadir, a inclinação é theta = acos(H / L_max)
    float thetaLimite = acosf(setupH / 60.0f) * (180.0f / M_PI);

    // Move o laser para apontar para o horizonte do piso (perto dos 89 graus)
    moveMotorToAngle(thetaLimite);
    delay(200);

    // Libera motor para o operador fazer o ajuste fino manualmente até a quina
    disableMotor();
    
    beepSuccess();

    screenDirty = true;
}

// ══════════════════════════════════════════════════════════════════
//  CAPTURA DA QUINA (usa H calibrado)
// ══════════════════════════════════════════════════════════════════

void captureQuina() {
    // Lê ângulo do IMU e distância do sensor
    float theta = readIMUAngleDeg();
    float L_mm  = readDistanceMM();
    float L_m   = L_mm / 1000.0f;

    if (L_mm < 10.0f || L_mm > 60000.0f) {
        blinkError("ERROR: Invalid sensor reading!");
        return;
    }

    float thetaRad = theta * (M_PI / 180.0f);

    setupTheta = theta;
    setupL     = L_m;

    // Decompõe usando H calibrado no nadir (0°)
    setupRmax  = L_m * sinf(thetaRad);                // Distância horizontal (sin para 0=nadir)
    float zQuina = L_m * cosf(thetaRad);               // Profundidade vertical (cos para 0=nadir)
    setupCotaQ = (setupH - zQuina) * 1000.0f;          // Cota em mm (negativo = recalque)

    // Validação de sanidade (Raio do tanque max 60m)
    if (setupRmax < 0.5f || setupRmax > 60.0f) {
        blinkError("ERROR: Radius out of bounds (0.5~60m)!");
        return;
    }

    quinaCaptured = true;
    
    // CRÍTICO: Como o motor estava livre e o operador moveu com a mão, 
    // a variável de controle de passos perdeu a referência.
    // Sincronizamos a posição atual do motor com a realidade lida pelo IMU!
    currentMotorAngle = theta;

    Serial.printf("JOINT: θ=%.2f° L=%.3fm Rmax=%.2fm Z=%.0fmm\n",
                  setupTheta, setupL, setupRmax, setupCotaQ);
    
    beepSuccess();

    screenDirty = true;
}

// ══════════════════════════════════════════════════════════════════
//  TOQUE E NAVEGAÇÃO
// ══════════════════════════════════════════════════════════════════

bool getTouch(int16_t &tx, int16_t &ty) {

    if (!ts.touched()) return false;
    TS_Point p = ts.getPoint();
    tx = p.y;
    ty = 239 - p.x;


    // Feedback sonoro para o toque
    beep(30);
    
    return true;
}

bool insideZone(int16_t tx, int16_t ty, Zone &z) {
    return (tx >= z.x && tx < z.x + z.w && ty >= z.y && ty < z.y + z.h);
}

void handleTouch() {
    int16_t tx, ty;
    if (!getTouch(tx, ty)) return;

    switch (activeScreen) {

    case SCR_MAIN_MENU:
        if (insideZone(tx, ty, mainMenuZones[0])) {
            activeScreen = SCR_BOTTOM_MENU;
            screenDirty = true;
        }
        else if (insideZone(tx, ty, mainMenuZones[1])) {
            blinkError("Not implemented yet!");
        }
        else if (insideZone(tx, ty, mainMenuZones[2])) {
            activeScreen = SCR_HORIZON;
            enableMotor();
            moveMotorToAngle(90.0f); // 90° = Horizontal
            screenDirty = true;
        }
        break;
        
    case SCR_HORIZON:
        // Any click returns to main menu
        moveMotorToAngle(0.0f); // Volta pro nadir
        disableMotor();
        activeScreen = SCR_MAIN_MENU;
        screenDirty = true;
        break;

    case SCR_BOTTOM_MENU:
        if (insideZone(tx, ty, bottomMenuZones[0])) {
            radialCount += 4;
            if (radialCount > 36) radialCount = 8;
            screenDirty = true;
        }
        else if (insideZone(tx, ty, bottomMenuZones[1])) {
            resLevel = (resLevel + 1) % 3;
            screenDirty = true;
        }
        else if (insideZone(tx, ty, bottomMenuZones[2])) {
            activeScreen = SCR_MAIN_MENU;
            screenDirty = true;
        }
        else if (insideZone(tx, ty, bottomMenuZones[3])) {
            currentRadial = 1;
            zeroCalibrated = false;
            quinaCaptured = false;
            activeScreen = SCR_SETUP;
            enableMotor();
            moveMotorToAngle(0.0f);
            screenDirty = true;
        }
        break;

    case SCR_SETUP:
        if (insideZone(tx, ty, setupCaptureZone)) {
            if (!zeroCalibrated) {
                calibrateZero();
            }
            else if (!quinaCaptured) {
                captureQuina();
            }
            else {
                activeScreen = SCR_SCANNING;
                screenDirty = true;
                startScan();
            }
        }
        else if (insideZone(tx, ty, setupAbortZone)) {
            enableMotor();
            moveMotorToAngle(0.0f); // Volta pro nadir (0)
            disableMotor();
            zeroCalibrated = false;
            quinaCaptured = false;
            activeScreen = SCR_BOTTOM_MENU;
            screenDirty = true;
        }
        break;

    case SCR_CHECKPOINT:
        if (currentRadial <= radialCount) {
            if (insideZone(tx, ty, checkScanZone)) {
                // Ir para SETUP da próxima Survey Line
                zeroCalibrated = false; // Exige nova captura de altura (H)
                quinaCaptured = false;
                
                // Gira pro nadir para iniciar a calibração
                enableMotor();
                moveMotorToAngle(90.0f);
                
                activeScreen = SCR_SETUP;
                screenDirty = true;
            }
            else if (currentRadial > 1 && insideZone(tx, ty, checkUndoZone)) {
                // Apagar última Survey Line e refazer
                currentRadial--;
                // Remove arquivo da Survey Line anterior (que agora virou currentRadial após o decremento)
                sprintf(fileName, "/PT%02d.csv", currentRadial - 1);
                SD.remove(fileName);
                screenDirty = true;
            }
            else if (currentRadial > 1 && insideZone(tx, ty, checkAbortHalfZone)) {
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
            else if (currentRadial == 1 && insideZone(tx, ty, checkAbortFullZone)) {
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
        } else {
            if (insideZone(tx, ty, checkFinishZone)) {
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
        }
        break;

    case SCR_SCANNING:
        if (!confirmingStop) {
            if (insideZone(tx, ty, scanStopZone)) {
                // Pausar varredura para confirmação
                scanRunning = false;
                confirmingStop = true;
                screenDirty = true;
            }
        } else {
            if (insideZone(tx, ty, confirmYesZone)) {
                // Abortar de fato
                confirmingStop = false;
                closeSDFile();
                // Volta motor para a vertical e trava
                moveMotorToAngle(0.0f);
                activeScreen = SCR_CHECKPOINT;
                screenDirty = true;
            } else if (insideZone(tx, ty, confirmNoZone)) {
                // Retomar varredura
                confirmingStop = false;
                scanRunning = true;
                screenDirty = true;
            }
        }
        break;
    }

    delay(200);  // Debounce tátil
}

// ══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println("GeoTank LaserScan v3.0 - Inicializando...");

    // Pinos de controle do motor e buzzer
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    
    disableMotor();  // Inicia com motor livre

    // Inicialização do Sensor de Distância
#if SIMULACAO_WOKWI
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
#else
    // UART2 para comunicação com o Laser Industrial
    Serial2.begin(115200, SERIAL_8N1, PIN_LASER_RX, PIN_LASER_TX);
    // Se o driver RS485 for half-duplex, adicione aqui o pinMode(PIN_DE_RE, OUTPUT);
#endif

    // Barramento I2C
    Wire.begin(21, 22);

    // Display TFT e Touch

    tft.begin();
    tft.setRotation(1);
    ts.begin(40); // FT6206


    // IMU
    if (mpu.begin()) {
        imuOnline = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("IMU: Online");
    } else {
        Serial.println("IMU: FAILED");
    }

    // SD Card
    if (SD.begin(PIN_SD_CS)) {
        sdOnline = true;
        Serial.println("SD: Online");
    } else {
        Serial.println("SD: FAILED");
    }

    // Verificação do sensor de distância
    float testDist = readDistanceMM();
    scanOnline = (testDist > 0);
    Serial.printf("SCAN: %s (%.0fmm)\n", scanOnline ? "Online" : "FAILED", testDist);

    Serial.println("Initialization complete.");
}

bool blinkState = false;
unsigned long lastBlinkMs = 0;

void loop() {
    // Redesenha tela se necessário
    if (screenDirty) {
        switch (activeScreen) {
            case SCR_MAIN_MENU:  drawMainMenu();        break;
            case SCR_BOTTOM_MENU:drawBottomMenu();      break;
            case SCR_HORIZON:    drawHorizonScreen();   break;
            case SCR_SETUP:      drawSetupScreen();     break;
            case SCR_CHECKPOINT: drawCheckpointScreen(); break;
            case SCR_SCANNING:   drawScanningScreen();  break;
        }
        screenDirty = false;
    }

    // Animação de piscar para a Etapa 2 do Setup (Motor Livre)
    if (activeScreen == SCR_SETUP && zeroCalibrated && !quinaCaptured && !screenDirty) {
        if (millis() - lastBlinkMs > 500) {
            lastBlinkMs = millis();
            blinkState = !blinkState;
            tft.setFont(NULL);
            tft.setTextSize(1);
            tft.fillRect(0, 102, 320, 10, C_BG); // Apaga a área do texto
            tft.setTextColor(blinkState ? C_WHITE : C_BG);
            drawCentered("Point the laser to the FLOOR-WALL JOINT", 160, 102);
        }
    }

    // Processa toque
    handleTouch();

    // Executa passos de varredura quando ativo
    if (activeScreen == SCR_SCANNING && scanRunning) {
        if (millis() - lastScanStepMs > 500) {
            lastScanStepMs = millis();
            executeScanStep();
            updateScanningTelemetry();
        }
    }
}
