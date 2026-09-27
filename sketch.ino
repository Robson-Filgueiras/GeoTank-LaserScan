/*
 * Copyright (C) 2026 Robson Filgueiras.
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
 *
 * ----------------------------------------------------------------------
 *
 * This project is based on and includes modifications to the U8x_Laser_Distance
 * project by Chandra Wijaya Sentosa, which is licensed under the MIT License:
 *
 * MIT License
 * 
 * Copyright (c) 2023 Chandra Wijaya Sentosa
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
#include <AccelStepper.h>

// ── Pinagem (ANTES de qualquer instância que use os pinos) ───────
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4
#define PIN_STEP  25
#define PIN_DIR   26
#define PIN_ENA   27
#define PIN_SD_CS  5

// Simulador vs Hardware Real
#define SIMULACAO_WOKWI false  // Hardware real — laser via Serial2, IMU via MPU6050

// Pinos Ultrassônico (Wokwi apenas)
#define PIN_TRIG  13
#define PIN_ECHO  12

// Pinos Laser UART RS232/RS485 (Hardware Real)
#define PIN_LASER_RX 17
#define PIN_LASER_TX 16

// Buzzer de feedback
#define PIN_BUZZER 32

// ── Motor e Laser Async ───────────────────────────────────────────
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
bool novaLeituraLaser = false;
float ultimaDistanciaLaser = 0.0f;
char bufferLaser[32];
int bufferIndex = 0;

// Controle de Placa
#define USE_CYD_BOARD 1

#include <XPT2046_Touchscreen.h>
#define TOUCH_CS 14

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "priner_logo.h"

/* ================================================================
   PROJETO:     GeoTank LaserScan v3.0
   PLATAFORMA:  ESP32 DevKit C V4
   DESCRIÇÃO:   Perfilometria de fundo de tanque por laser/ultrassônico.
                Varredura centrípeta com cota topográfica adaptativa.
                Referência angular: θ a partir do eixo horizontal.
                Cota = H - L × sin(θ). Negativo = recalque.
   ================================================================ */

// ── Periféricos ──────────────────────────────────────────────────
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);

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
    tft.setTextSize(1); // Trava de segurança (impede vazamento da fonte tamanho 3 do Horizon)
    tft.fillRect(0, 0, 320, 24, C_HEADER);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT); // Cor do título restaurada
    tft.setCursor(2, 17); // Puxado 2 pixels para a esquerda
    tft.print("GeoTank v3.0");

    // Desenha apenas o símbolo verde da Priner (cortado rigorosamente)
    int16_t logo_x = 124; // Mais perto do GeoTank
    int16_t logo_y = 4;   // Centralizado no header (agora tem 16px de altura -> (24-16)/2 = 4)
    for (int y = 0; y < priner_logo_h; y++) {
        for (int x = 0; x < priner_logo_w; x++) {
            uint16_t color = pgm_read_word(&priner_logo_data[y * priner_logo_w + x]);
			  if (color != 0xFFFF) {
                tft.drawPixel(logo_x + x, logo_y + y, color);
            }
        }
    }

    // Escreve a palavra "Priner" (P maiúsculo e fonte normal para economizar espaço horizontal)
    tft.setFont(&FreeSans9pt7b); // Fonte mais fina que a Bold
    tft.setTextColor(C_DIM);
    tft.setCursor(logo_x + priner_logo_w + 3, 17);
    tft.print("Priner");

    tft.setFont(NULL);
    tft.setTextSize(1);
    int16_t xr = 316;
    
    // Silhueta de bateria vazada contendo o percentual no interior
    tft.drawRect(xr - 36, 4, 34, 16, C_BORDER); // Corpo da bateria
    tft.fillRect(xr - 38, 8, 2, 8, C_BORDER);   // Terminal positivo
    
    tft.setTextColor(C_GREEN);
    drawRightAligned("85%", xr - 10, 12);
    
    xr -= 46;

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
    // Trem de pulsos a ~2 kHz para excitar tanto buzzers passivos quanto ativos
    unsigned long start = millis();
    while (millis() - start < ms) {
        digitalWrite(PIN_BUZZER, HIGH);
        delayMicroseconds(250);
        digitalWrite(PIN_BUZZER, LOW);
        delayMicroseconds(250);
    }
}

void beepSuccess() {
    beep(100); delay(100);
    beep(150);
}

unsigned long errorMessageUntil = 0;
bool isErrorShowing = false;

void blinkError(const char* msg) {
    tft.fillRect(0, 218, 320, 22, C_FOOTER);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_RED);
    tft.setCursor(4, 226);
    tft.print(msg);
    beep(150);
    isErrorShowing = true;
    errorMessageUntil = millis() + 2000;
}

// ══════════════════════════════════════════════════════════════════
//  CONTROLE DO MOTOR DE PASSO
// ══════════════════════════════════════════════════════════════════

void enableMotor() { stepper.enableOutputs(); }

void disableMotor() { stepper.disableOutputs(); }

// Motor gira buscando o prumo gravitacional verdadeiro (Nadir absoluto) descendo pelo mesmo caminho para nao tracionar cabos
void seekNadirBlocking() {
    enableMotor();
    
    tft.fillScreen(C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_YELLOW);
    drawCentered("SEEKING NADIR...", 160, 80);
    
    while (true) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Pega o ângulo RAW original da trigonometria (sem fabs) para ter consciência de se passou do Zero ou não
        float currentThetaRaw = atan2f(a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
        float error = 0.0f - currentThetaRaw;
        
        // --- Exibição na Tela em Tempo Real ---
        static unsigned long lastDraw = 0;
        if (millis() - lastDraw > 200) {
            lastDraw = millis();
            tft.fillRect(60, 120, 200, 40, C_BG);
            tft.setFont(NULL);
            tft.setTextSize(3);
            tft.setTextColor(fabs(error) <= 0.15f ? C_GREEN : C_YELLOW);
            char angStr[16];
            // Exibimos em módulo pro usuário enxergar sempre os 0.0 limpos
            sprintf(angStr, "%.1f deg", fabs(currentThetaRaw));
            drawCentered(angStr, 160, 140);
        }

        // --- Homing Mecânico ---
        if (fabs(error) <= 0.15f) {
            stepper.setSpeed(0);
            stepper.runSpeed();
            delay(150); 
            mpu.getEvent(&a, &g, &temp);
            currentThetaRaw = atan2f(a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
            if (fabs(0.0f - currentThetaRaw) <= 0.25f) break; // Cravou perfeitamente
        }
        
        float spd;
        if (fabs(error) > 5.0f) {
            // No V013: error= +90, a gente mandava -1000 e ele SUUBIA esmagando cabo.
            // Para DESCER, invertemos: se erro for positivo, spd DEVE SER POSITIVO.
            spd = (error > 0) ? 1000.0f : -1000.0f;
        } else {
            spd = (error > 0) ? 30.0f : -30.0f; // Acoplamento firme. Se passar do 0, o sinal inverte e ele Volta.
        }
        
        stepper.setSpeed(spd);
        stepper.runSpeed();
        
        pollLaser();
        yield();
    }
    
    tft.setTextSize(1); // Destrava tamanho de fonte
    
    // Sincroniza a posição mecânica física atual a zero passos virtuais
    stepper.setCurrentPosition(0);
}

// Motor viaja para o alvo sem trigar o cão de guarda (WDT) do FreeRTOS e sem atolar a porta Serial do Laser
void moveMotorBlocking(long targetPosition) {
    stepper.moveTo(targetPosition);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        pollLaser(); // Drena o buffer da Serial assíncrona p/ n estourar
        yield();     // Refresca o Watchdog Timer do ESP32 liberando a Thread Idle
    }
}

// Motor gira buscando um prumo gravitacional absoluto com malha fechada
void seekAngleBlocking(float targetAngle) {
    enableMotor();
    
    // Mensagem de Feedback para a UI
    tft.fillScreen(C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_YELLOW);
    drawCentered(targetAngle == 0.0f ? "SEEKING NADIR..." : "SEEKING ANGLE...", 160, 120);
    
    while (true) {
        float currentTheta = readIMUAngleDeg();
        float error = targetAngle - currentTheta;
        
        if (fabs(error) <= 0.15f) {
            stepper.setSpeed(0);
            stepper.runSpeed();
            delay(150); // Assenta inércia mecânica
            currentTheta = readIMUAngleDeg();
            if (fabs(targetAngle - currentTheta) <= 0.25f) break; // Cravado
        }
        
        float spd;
        if (fabs(error) > 5.0f) {
            spd = (error > 0) ? -1000.0f : 1000.0f;
        } else {
            spd = (error > 0) ? -50.0f : 50.0f; // Acoplamento firme e seguro
        }
        
        stepper.setSpeed(spd);
        stepper.runSpeed();
        
        pollLaser();
        yield();
    }
    
    // Assinatura mecânica do Zero: Sincroniza passos virtuais com gravidade!
    if (targetAngle == 0.0f) {
        stepper.setCurrentPosition(0);
    }
}



// ══════════════════════════════════════════════════════════════════
//  SENSOR DE DISTÂNCIA
// ══════════════════════════════════════════════════════════════════

void pollLaser() {
#if SIMULACAO_WOKWI
    static unsigned long lastSim = 0;
    if (millis() - lastSim > 20) {
        lastSim = millis();
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);
        long duration = pulseIn(PIN_ECHO, HIGH, 5000); 
        if (duration > 0) {
            ultimaDistanciaLaser = ((duration * 0.343f) / 2.0f) / 1000.0f;
            novaLeituraLaser = true;
        }
    }
#else
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                bufferLaser[bufferIndex] = '\0'; // Termina C-String
                float dist = atof(bufferLaser);
                if (dist > 0.0f) {
                    ultimaDistanciaLaser = dist; // Sensor retorna em metros
                    novaLeituraLaser = true;
                }
                bufferIndex = 0; // Reseta buffer
            }
        } else if (bufferIndex < 31 && (isDigit(c) || c == '.' || c == '-')) {
            bufferLaser[bufferIndex++] = c;
        }
    }
#endif
}

float readDistanceStaticMM() {
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
    if (zeroCalibrated && setupH > 0.1f) {
        float L_m = readDistanceStaticMM() / 1000.0f;
        if (L_m >= setupH) {
            return acosf(setupH / L_m) * (180.0f / M_PI);
        }
    }
    return 0.0f;
#else
    if (!imuOnline) return 0.0f;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calcula o ângulo bruto em relação à vertical (Nadir = 0°)
    float angleDeg = atan2f(a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
    float thetaBruto = fabsf(angleDeg); // Mantém sempre positivo para estabilidade no horizonte
    
	// Filtro passa-baixa EMA — retém 60% da memória, absorve 40% da nova leitura
    static float thetaFiltrado = -1.0f;
    
    if (thetaFiltrado < 0.0f) {
        thetaFiltrado = thetaBruto;
    } else {
	   thetaFiltrado = (0.60f * thetaFiltrado) + (0.40f * thetaBruto);
	 }
    
    return thetaFiltrado;
#endif
}

// ══════════════════════════════════════════════════════════════════
//  GRAVAÇÃO NO SD CARD
// ══════════════════════════════════════════════════════════════════

bool openSDFile() {
    int ptIndex = currentRadial - 1;
    sprintf(fileName, "/PT%02d.csv", ptIndex);
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        char ptLabel[48];
        getPointLabel(currentRadial, radialCount, ptLabel);
        dataFile.print("# ");
        dataFile.println(ptLabel);
        dataFile.println("MP,X_m,Theta_deg,L_m,Z_mm");
        dataFile.flush();
        return true;
    }
    return false;
}

void writePointToSD(MeasPoint &pt) {
    if (dataFile) {
        char line[80];
        sprintf(line, "%d,%.3f,%.4f,%.4f,%.1f",
                pt.mp, pt.x_m, pt.theta_deg, pt.l_m, pt.cota_mm);
        dataFile.println(line);
        dataFile.flush();
    }
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
               
    drawButton(10, 155, 50, 50, C_RED, C_BORDER,
               "BACK", NULL);
    drawButton(70, 155, 240, 50, C_START, C_BORDER,
               "START PROJECT", "Lock and proceed");
               
    drawFooter("Tap panels to configure.");
}

void drawHorizonScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    tft.setTextSize(1);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_CYAN);
    drawCentered("HORIZON LEVEL", 160, 50);

    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_WHITE);
    drawCentered("Laser is seeking 90.0 deg.", 160, 85);

    drawButton(10, 175, 300, 40, C_DIM, C_BORDER, "BACK TO MENU", NULL);
    drawFooter("Motor energized.");
}

// ══════════════════════════════════════════════════════════════════
//  TELA: SETUP — CALIBRAÇÃO DO ZERO + CAPTURA DA QUINA
// ══════════════════════════════════════════════════════════════════

void getPointLabel(int radialIndex, int totalRadials, char* buffer) {
    int pt = radialIndex - 1;
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

        tft.setTextColor(C_GREEN);
        char strH[32];
        sprintf(strH, "H = %.3f m (calibrated)", setupH);
        drawCentered(strH, 160, 62);

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

        tft.setTextColor(C_GREEN);
        char info1[48];
        sprintf(info1, "H = %.3f m (calibrated)", setupH);
        drawCentered(info1, 160, 60);

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
    tft.fillRect(10, 65, 300, 110, C_BG);

    char tel1[48];
    sprintf(tel1, "MP %d / %d", lastPoint.mp, scanTotalPts);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered(tel1, 160, 80);

    char tel2[48];
    sprintf(tel2, "X=%.2fm   Z=%.0fmm", lastPoint.x_m, lastPoint.cota_mm);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(lastPoint.cota_mm < -50 ? C_RED :
                     lastPoint.cota_mm < -20 ? C_YELLOW : C_GREEN);
    drawCentered(tel2, 160, 105);

    char tel3[48];
    sprintf(tel3, "Theta=%.2f   L=%.3fm", lastPoint.theta_deg, lastPoint.l_m);
    tft.setTextColor(C_DIM);
    drawCentered(tel3, 160, 120);

    char tel4[32];
    sprintf(tel4, "Corrections: %d", reshots);
    tft.setTextColor(reshots > 0 ? C_YELLOW : C_DIM);
    drawCentered(tel4, 160, 135);

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

    currentMotorAngle = setupTheta;

    openSDFile();
    
    beep(300);
}

void executeScanStep() { /* Destruido */ }

void finishSurveyLine() {
    scanRunning = false;
    closeSDFile();

    moveMotorBlocking(0);

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
    enableMotor();
    seekNadirBlocking();

    float soma = 0.0f;
    int nLeituras = 5;
    for (int i = 0; i < nLeituras; i++) {
        soma += readDistanceStaticMM();
        delay(50);
    }
    float L_mm = soma / nLeituras;
    float L_m  = L_mm / 1000.0f;

    if (L_mm < 100.0f || L_mm > 15000.0f) {
        blinkError("ERROR: Invalid vertical reading!");
        return;
    }

    setupH      = L_m;
    setupL_zero = L_m;
    zeroCalibrated = true;

    Serial.printf("ZERO: H = %.3f m (average of %d readings)\n", setupH, nLeituras);

    float thetaLimite = acosf(setupH / 60.0f) * (180.0f / M_PI);

    long stepsLimite = (long)(thetaLimite * STEPS_PER_DEG);
    moveMotorBlocking(stepsLimite);

    disableMotor();
    
    beepSuccess();

    screenDirty = true;
}

// ══════════════════════════════════════════════════════════════════
//  CAPTURA DA QUINA (usa H calibrado)
// ══════════════════════════════════════════════════════════════════

void captureQuina() {
    float theta = readIMUAngleDeg();
    float L_mm  = readDistanceStaticMM();
    float L_m   = L_mm / 1000.0f;

    if (L_mm < 10.0f || L_mm > 60000.0f) {
        blinkError("ERROR: Invalid sensor reading!");
        return;
    }

    float thetaRad = theta * (M_PI / 180.0f);

    setupTheta = theta;
    setupL     = L_m;

    setupRmax  = L_m * sinf(thetaRad);
    float zQuina = L_m * cosf(thetaRad);
    setupCotaQ = (setupH - zQuina) * 1000.0f;

    if (setupRmax < 0.5f || setupRmax > 60.0f) {
        blinkError("ERROR: Radius out of bounds (0.5~60m)!");
        return;
    }

    quinaCaptured = true;
    
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
    static unsigned long lastTouchTime = 0;
    if (millis() - lastTouchTime < 250) return false;

    if (!ts.touched()) return false;
    
    TS_Point p = ts.getPoint();
    
    // Mapeamento deduzido (equivalente ao Modo 6 do teste):
    // Eixos trocados (p.y controla X, p.x controla Y) com p.x invertido
    tx = map(p.y, 250, 3800, 0, 320); 
    ty = map(p.x, 250, 3800, 240, 0);

    // Feedback sonoro para o toque
    beep(30);
    lastTouchTime = millis();
    return true;
}

bool insideZone(int16_t tx, int16_t ty, Zone &z) {
    return (tx >= z.x && tx < z.x + z.w && ty >= z.y && ty < z.y + z.h);
}

// Função para desenhar um cursor visual onde o toque foi registrado
void drawTouchCursor(int16_t x, int16_t y) {
    tft.drawLine(x - 8, y, x + 8, y, C_RED);
    tft.drawLine(x, y - 8, x, y + 8, C_RED);
    tft.drawCircle(x, y, 4, C_RED);
}

void handleTouch() {
    int16_t tx, ty;
    if (!getTouch(tx, ty)) return;

    // Mostra o cursor visual no local do toque
    // drawTouchCursor(tx, ty); // <-- Desabilitado conforme solicitado

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
            stepper.enableOutputs();
            screenDirty = true;
        }
        break;
        
    case SCR_HORIZON:
        stepper.stop();
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
            // Motor não é mais zerado aqui de forma invisível. Agora só quando apertar CALIBRATE.
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
            stepper.enableOutputs();
            stepper.moveTo(0);
            zeroCalibrated = false;
            quinaCaptured = false;
            activeScreen = SCR_BOTTOM_MENU;
            screenDirty = true;
        }
        break;

    case SCR_CHECKPOINT:
        if (currentRadial <= radialCount) {
            if (insideZone(tx, ty, checkScanZone)) {
                zeroCalibrated = false;
                quinaCaptured = false;
                
                enableMotor();
                moveMotorBlocking((long)(90.0f * STEPS_PER_DEG));
                
                activeScreen = SCR_SETUP;
                screenDirty = true;
            }
            else if (currentRadial > 1 && insideZone(tx, ty, checkUndoZone)) {
                currentRadial--;
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
                stepper.stop();
                scanRunning = false;
                confirmingStop = true;
                screenDirty = true;
            }
        } else {
            if (insideZone(tx, ty, confirmYesZone)) {
                confirmingStop = false;
                closeSDFile();
                stepper.moveTo(0);
                activeScreen = SCR_CHECKPOINT;
                screenDirty = true;
            } else if (insideZone(tx, ty, confirmNoZone)) {
                confirmingStop = false;
                scanRunning = true;
                screenDirty = true;
            }
        }
        break;
    }
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
    
    stepper.setEnablePin(PIN_ENA);
    stepper.setPinsInverted(false, false, true); 
    stepper.setMaxSpeed(2000.0);
    stepper.setAcceleration(800.0);
    stepper.disableOutputs(); // Inicia motor livre

    // Inicialização do Sensor de Distância
#if SIMULACAO_WOKWI
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
#else
    // UART2 para comunicação com o Laser Industrial
    Serial2.begin(115200, SERIAL_8N1, PIN_LASER_RX, PIN_LASER_TX);
#endif

    // Barramento I2C
    Wire.begin(21, 22);

    // Display TFT e Touch
    tft.begin();
    tft.setRotation(1);
    ts.begin();
    ts.setRotation(1);

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
    float testDist = readDistanceStaticMM();
    scanOnline = (testDist > 0);
    Serial.printf("SCAN: %s (%.0fmm)\n", scanOnline ? "Online" : "FAILED", testDist);

    Serial.println("Initialization complete.");
}

bool blinkState = false;
unsigned long lastBlinkMs = 0;

void loop() {
    // Limpa mensagem de erro após 2 segundos (não-bloqueante)
    if (isErrorShowing && millis() > errorMessageUntil) {
        isErrorShowing = false;
        screenDirty = true;
    }

    // Redesenha tela se necessário
    if (screenDirty) {
        isErrorShowing = false; // Se a tela for redesenhada por outro motivo, reseta a flag
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
            tft.fillRect(0, 102, 320, 10, C_BG);
            tft.setTextColor(blinkState ? C_WHITE : C_BG);
            drawCentered("Point the laser to the FLOOR-WALL JOINT", 160, 102);
        }
    }

    // Processa toque
    handleTouch();

    pollLaser();

    // Controle de Nível Absoluto (Horizonte = 90.0 deg) via Dual-Speed Constante
    if (activeScreen == SCR_HORIZON) {
        float currentTheta = readIMUAngleDeg();
        float error = 90.0f - currentTheta;
        
        // Zona morta de 0.15° (filtra o jitter basal do acelerômetro)
        if (fabs(error) > 0.15f) {
            float spd;
            if (fabs(error) > 5.0f) {
                // Viagem veloz de cruzeiro, taxa fixa (Bang-Bang)
                spd = (error > 0) ? -1000.0f : 1000.0f;
            } else {
                // Acoplamento final, velocidade constante de manobra (Lenta e calculada para evitar overshoot)
                spd = (error > 0) ? -10.0f : 10.0f;
            }
            
            stepper.setSpeed(spd);
            stepper.runSpeed();
        } else {
            stepper.setSpeed(0);
            stepper.runSpeed();
        }

        // Atualização do ângulo em tempo real (centro da tela)
        static unsigned long lastAngMs = 0;
        if (millis() - lastAngMs > 200) {
            lastAngMs = millis();
            tft.fillRect(60, 110, 200, 40, C_BG); // Apaga o bloco central sem tocar no botão (Y=175)
            tft.setFont(NULL);
            tft.setTextSize(3); // Fonte maior e mais chamativa
            tft.setTextColor(fabs(error) <= 0.15f ? C_GREEN : C_YELLOW);
            char angStr[16];
            sprintf(angStr, "%.1f deg", currentTheta);
            drawCentered(angStr, 160, 130);
            
            tft.setTextSize(1); // RESTAURAÇÃO OBRIGATÓRIA DA TIPOGRAFIA BASE
        }
    } else {
        stepper.run();
    }

    // Executa passos de varredura quando ativo
    if (activeScreen == SCR_SCANNING && scanRunning && !confirmingStop) {
        if (novaLeituraLaser) {
            novaLeituraLaser = false;
            
            float currentTheta = stepper.currentPosition() / STEPS_PER_DEG;
            float L_m = ultimaDistanciaLaser;
            float thetaRad = currentTheta * (M_PI / 180.0f);
            
            float rReal = L_m * sinf(thetaRad);
            float zReal = L_m * cosf(thetaRad);
            float cota  = (setupH - zReal) * 1000.0f;
            
            MeasPoint pt;
            pt.mp        = scanIndex++;
            pt.x_m       = rReal;
            pt.theta_deg = currentTheta;
            pt.l_m       = L_m;
            pt.cota_mm   = cota;
            
            if (dataFile) {
                char line[80];
                sprintf(line, "%d,%.3f,%.4f,%.4f,%.1f", pt.mp, pt.x_m, pt.theta_deg, pt.l_m, pt.cota_mm);
                dataFile.println(line);
                
                if (pt.mp % 50 == 0) {
                    dataFile.flush();
                }
            }
            lastPoint = pt;
            
            if (stepper.distanceToGo() == 0) {
                finishSurveyLine();
            }
        }
        
        static unsigned long lastTelMs = 0;
        if (millis() - lastTelMs > 500) {
            lastTelMs = millis();
            updateScanningTelemetry();
        }
    }

    // Telemetria Global do HUD de Ângulo (Canto inferior direito)
    static unsigned long lastFooterAngMs = 0;
    if (millis() - lastFooterAngMs > 250) {
        lastFooterAngMs = millis();
        float thetaReal = readIMUAngleDeg();
        // Atualiza apenas a caixa à direita para não flikcar o texto de status
        tft.fillRect(250, 218, 70, 22, C_FOOTER);
        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_WHITE);
        char globalAngStr[16];
        sprintf(globalAngStr, "%.1f deg", thetaReal);
        drawRightAligned(globalAngStr, 316, 226);
    }
}
