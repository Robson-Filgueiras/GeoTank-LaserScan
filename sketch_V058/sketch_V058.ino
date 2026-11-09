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
#include <time.h>
#include <sys/time.h>
#include <esp_timer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AccelStepper.h>
#include <Preferences.h>

Preferences calibPrefs;
// Referências angulares brutas do MPU6050 calibradas no prumo/nível a laser externo (V058):
//   - Nadir   (  0,00° real) -> atan2(-Az, Ax) =  +6,96° (H = 784 mm cravado no risco do chão)
//   - Nível   ( 90,00° real) -> atan2(-Az, Ax) = +88,30° (cravado no nível horizontal a L=2150 mm, ajuste -0,65° / -25 mm)
//   - Zênite  (180,00° real) -> atan2(-Az, Ax) = +176,76° (cravado no prumo do teto)
float imuRawNadir0Deg    = 6.96f;
float imuRawHorizon90Deg = 88.30f;
float imuRawZenith180Deg = 176.76f;
float imuPitchOffsetDeg  = 0.00f;  // Trim fino adicional sobre a curva de 3 pontos
float imuRawUnwrappedDeg = 0.0f;   // Ângulo bruto desdobrado do MPU6050 sem calibração

// ── Pinagem (ANTES de qualquer instância que use os pinos) ───────
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4
#define PIN_STEP  25
#define PIN_DIR   26
#define PIN_ENA   27

// Barramento HSPI Dedicado ao Cartão MicroSD (Isolado dos pinos da Tela/Touch)
#define PIN_SD_CS    5
#define PIN_SD_SCK  33
#define PIN_SD_MOSI 13
#define PIN_SD_MISO 34

// Simulador vs Hardware Real
#define SIMULACAO_WOKWI false  // Hardware real — laser via Serial2, IMU via MPU6050

// Pinos Ultrassônico (Wokwi apenas)
#if SIMULACAO_WOKWI
#define PIN_TRIG  13
#define PIN_ECHO  12
#endif

// Pinos Laser UART TTL 3,3V M01-V0.4 (Hardware Real - 9600 bps)
// ESP32 GPIO16 (RX2) <--- Laser Pad 3 (TXD / Fio Verde)
// ESP32 GPIO17 (TX2) ---> Laser Pad 2 (RXD / Fio Azul-Escuro)
#define PIN_LASER_RX 16
#define PIN_LASER_TX 17

// Buzzer de feedback
#define PIN_BUZZER 32

// Instância SPI de Hardware Independente (HSPI) para o Cartão MicroSD
SPIClass spiSD(HSPI);

// ── Motor e Laser Async (Protocolo Binário M01 - 9600 bps) ──────
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
bool novaLeituraLaser = false;
float ultimaDistanciaLaser = 0.0f;
uint32_t ultimaDistanciaMM = 0;
uint32_t ultimaBrutaMM     = 0;
unsigned long lastValidLaserMs = 0;
unsigned long lastAnyByteLaserMs = 0;
unsigned long lastContCmdMs = 0;
bool laserContActive   = false;
bool keepLaserAlive    = false;
bool useQuickLaserMode = false; // V058: Modo Contínuo Padrão 0x21 (3.2 Hz contínuos reais) ativo por padrão; 0x22 interrompe após 1 amostra

// ── Gerador de Pulsos por Acumulador Inteiro de 64 bits com Spinlock Multi-Core ──
// motorBgMode:
//   0 = stepper.run()               (rampa suave p/ retorno Centro -> Quina)
//   1 = Acumulador contínuo (64-bit)(velocidade contínua s/ reset de fase p/ Nadir / Horizonte)
//   2 = Acumulador p/ alvo (64-bit) (posicionamento ponto a ponto da varredura V046)
portMUX_TYPE       motorTimerMux     = portMUX_INITIALIZER_UNLOCKED;
volatile bool      motorBgActive     = false;
volatile uint8_t   motorBgMode       = 0;
volatile int32_t   bgCurrentPosSteps = 0;
volatile int32_t   bgTargetPosSteps  = 0;
volatile uint32_t  bgStepIntervalUs  = 0;
volatile int8_t    bgStepDir         = 1;
volatile int64_t   bgNextStepUs      = 0;
esp_timer_handle_t motorStepTimer    = NULL;

// ── Proteção Térmica do Motor (Corta Corrente em Repouso) ───────
bool          motorCurrentlyEnergized = false;
unsigned long motorEnergizedSinceMs   = 0;
#define MOTOR_IDLE_THERMAL_TIMEOUT_MS  45000UL  // Desenergiza após 45s parado em telas comuns
#define MOTOR_HOLD_THERMAL_TIMEOUT_MS 180000UL  // Limite térmico de 3 min p/ retenção estática em Setup/Checkpoint

// ── Parâmetros da Varredura Discreta Ponto a Ponto com Média em 0,5 s (V050) ──
// Regra de Zonas Angulares (Partindo de qualquer Quina até 0°):
//   - Acima de 63° (θ > 63°): avança no menor passo (1 micropasso = 0,1125°), para >= 0,5s e tira a média.
//   - De 63° até 45° (63° >= θ > 45°): 50 pontos exatos no trecho de 18° (Δθ = 0,36°/ponto), para >= 0,5s e tira a média.
//   - Abaixo de 45° (45° >= θ >= 0°): 1 leitura a cada 2,0° (23 pontos até 0°), para >= 0,5s e tira a média.
#define SCAN_DWELL_MS               650UL  // V053: Parada de 0,65s por ponto p/ permitir amostragem múltipla anti-outlier no dwell
#define SCAN_MAX_DWELL_MS          1800UL  // Tempo máximo de espera por amostra óptica inédita em alvos oblíquos (> 63°)
#define SCAN_FALLBACK_MAX_AGE_MS   2400UL  // Idade máxima aceitável p/ fallback sinalizado (jamais usa ponto antigo distante)
#define SCAN_ZONE1_LIMIT_DEG        63.0f  // Acima de 63°: menor passo (0,1125°)
#define SCAN_ZONE2_LIMIT_DEG        45.0f  // Entre 63° e 45°: 50 pontos (0,36°/ponto)
#define SCAN_ZONE2_STEP_DEG         0.36f  // 18° / 50 pontos = 0,36° por ponto
#define SCAN_ZONE3_STEP_DEG         2.00f  // Abaixo de 45°: 1 leitura a cada 2,0°
#define SCAN_PIVOT_OFFSET_MM        55.5f  // V056: Compensacao Cinematica do Pivo Fisico d=55.5mm (Laser na Vertical / Foto 2)
#define SCAN_KZ_CORRECTION_MM_PER_M 0.00f  // V056: Constante linear radial residual (zerada pois d_pivo absorve 100% da deflexao)
#define SCAN_VERTEX_GRAZING_H_MM     9.5f  // V058: Offset fino de spot rasante na quina c/ Horizonte calibrado em 88.30° (55.5 + 9.5 = 65.0 mm)

#define MOTOR_SCAN_SPEED_FAST     40.0f  // Velocidade de transição entre pontos
#define MOTOR_SCAN_SPEED_NORMAL   36.0f  // Velocidade limpa de avanço entre pontos discretos
#define MOTOR_SCAN_SPEED_FINE     28.0f

#define MOTOR_BELOW_45_MULT        1.25f
#define MOTOR_MIN_SMOOTH_SPEED    14.0f  // Velocidade mínima anti-cogging no DRV8825 (~1,6 graus/s)
#define MOTOR_RETURN_SPEED        80.0f  // ~9,0 graus/s no retorno
#define MOTOR_RETURN_ACCEL        40.0f  // Rampa macia e firme
#define MOTOR_SEEK_MAX_SPEED      55.0f  // ~6,2 graus/s padrão de busca lenta
#define MOTOR_FAST_SEEK_SPEED    120.0f  // ~13,5 graus/s Slew Rápido (Nadir <-> Horizonte <-> Vértice API 650)

// Velocidade customizada via COM5 (0 = usa a velocidade da resolução escolhida)
float customSweepSpeedSteps = 0.0f;
bool  below45BoostApplied   = false;

// Máquina de Estados Ponto a Ponto da Varredura (V050: Move -> Para >= 0,5s c/ Ângulo IMU Estático + Correção Kz no TFT/CSV -> Próximo Ponto)
enum ScanPointPhase {
    SCAN_PHASE_DWELL = 0,
    SCAN_PHASE_MOVING = 1
};
ScanPointPhase scanPhase         = SCAN_PHASE_DWELL;
unsigned long  scanDwellStartMs  = 0;
unsigned long  scanStartTimeMs   = 0;
float          scanTargetTheta   = 0.0f;
float          scanIdealTheta    = 0.0f;  // Acumulador angular contínuo sem truncagem cumulativa na Zona 2/3
#define PT_MAX_SAMPLES            6
uint32_t       ptSamples[PT_MAX_SAMPLES]; // V053: Buffer de amostras individuais no dwell p/ filtro de mediana local
uint32_t       ptSampleSumMM     = 0;
int            ptSampleCount     = 0;
bool           ptRetrySent       = false;
float          ptImuSumDeg       = 0.0f;
int            ptImuCount        = 0;

// Controle de trava amortecida no Nível Horizonte (evita pingue-pongue 89.5° <-> 93.1°)
bool          horizonLocked       = false;
unsigned long lastHorizonStepMs   = 0;

// Compensação do tempo médio de exposição óptica do M01-V0.4 (~180 ms)
#define LASER_EXPOSURE_LAG_SEC   0.18f

// ── Filtro de Média Aparada por Janela de Tempo (Para Medições Estáticas: Nadir e Quina) ──
#define MAX_WINDOW_SAMPLES 32

struct TimeSample {
    uint32_t distMM;
    unsigned long tMs;
};

TimeSample winBuf[MAX_WINDOW_SAMPLES];
int winCount = 0;
unsigned long windowDurationMs = 1500;
uint32_t avgDistMM      = 0;
float    avgDistM       = 0.0f;
uint32_t winMinMM       = 0;
uint32_t winMaxMM       = 0;
uint32_t winSpreadMM    = 0;
float    measuredRateHz = 0.0f;
int      stepJumpCount  = 0;

// Filtro curto de mediana sobre a Cota Z (mm) durante a varredura
int32_t zHistMM[3] = {0, 0, 0};
int     zHistCount = 0;

const uint8_t M01_CMD_LASER_ON[]   = {0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0xC1};
const uint8_t M01_CMD_LASER_OFF[]  = {0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x00, 0xC0};
const uint8_t M01_CMD_SINGLE[]     = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x21};
const uint8_t M01_CMD_CONT[]       = {0xAA, 0x00, 0x00, 0x21, 0x00, 0x01, 0x00, 0x00, 0x22};
const uint8_t M01_CMD_QUICK[]      = {0xAA, 0x00, 0x00, 0x22, 0x00, 0x01, 0x00, 0x00, 0x23};
const uint8_t M01_CMD_READ_LAST[]  = {0xAA, 0x80, 0x00, 0x22, 0xA2};
const uint8_t M01_CMD_VOLTAGE[]    = {0xAA, 0x80, 0x00, 0x06, 0x86};

// ── Telemetria Triaxial IMU (MPU-6050) ──────────────────────────
float imuAccelX   = 0.0f;
float imuAccelY   = 0.0f;
float imuAccelZ   = 0.0f;
float imuPitchDeg = 0.0f;
float imuRollDeg  = 0.0f;
float imuNormG    = 10.5f;
bool  forceImuFilterSync = false;


// Controle de Placa
#define USE_CYD_BOARD 1

#include <XPT2046_Touchscreen.h>
#define TOUCH_CS 14

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "priner_logo.h"

/* ================================================================
   PROJETO:     GeoTank LaserScan v3.0 (Firmware V058 - Auto-Busca Exata na Quina + Compensacao Diedro/Rasante Z)
   PLATAFORMA:  ESP32 DevKit C V4
   DESCRIÇÃO:   Perfilometria de fundo de tanque por laser M01-V0.4.
                - Mantém arquivadas V044 a V057 intactas.
                - Auto-Busca do Vértice (Quina) Costado-Fundo:
                  1) Busca o Nadir (0,00°) e mede a cota H do instrumento ao piso central por 1,5s com motor travado.
                  2) Slew Rápido para o Horizonte (90,00°) e mede a distância horizontal R ao costado por 1,5s.
                  3) Aplica a inclinação de projeto API Standard 650 (Item 5.4.4 e Anexo B.3.3):
                     Mínimo 1:120 (0,83% ou 8,33 mm/m):
                     * Cone-Up: quina mais baixa em DeltaZ = -R/120 (h_quina = H + R/120).
                     * Cone-Down: quina mais alta em DeltaZ = +R/120 (h_quina = H - R/120).
                     * Plano: DeltaZ = 0 (h_quina = H).
                     Calcula o ângulo exato do vértice: theta = atan2(R, h_quina) e a diagonal (hipotenusa) L = sqrt(R^2 + h_quina^2).
                  4) Slew Rápido direto para o vértice calculado, posicionando o feixe laser exatamente na quina.
                  5) Transição para SCR_JOINT com telemetria ativa e pronto para iniciar a varredura radial.
                - Slew Rápido de Posicionamento (MOTOR_FAST_SEEK_SPEED = 120.0 passos/s ~13,5°/s) com rampa de desaceleração suave,
                  reduzindo o tempo de movimentação angular entre Nadir, Horizonte e Vértice de ~15s para ~4s a 6s.
                - Menu interativo TFT (SCR_BOTTOM_MENU) com botão para selecionar o perfil API 650 (CONE-UP, CONE-DOWN, PLANO),
                  também acessível via atalho serial COM5 [B].
                - Preserva constante linear radial Kz = +6,63 mm/m e registro rastreável do perfil no cabeçalho do arquivo CSV.
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
#define C_MAGENTA  0xF81F  // V054: Alerta visual para Obstruções / Tubulações no TFT

// ── Máquina de Estados ──────────────────────────────────────────
enum Screen {
    SCR_MAIN_MENU,
    SCR_BOTTOM_MENU,
    SCR_SETUP,
    SCR_JOINT,
    SCR_CHECKPOINT,
    SCR_SCANNING,
    SCR_HORIZON
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
    int32_t  cota_mm;
    uint8_t  fallback; // 0 = amostra óptica estática inédita no ponto; 1 = fallback validado por janela temporal
};

// ── Constantes de Motor ─────────────────────────────────────────
#define STEPS_PER_REV    3200.0f
#define STEPS_PER_DEG    (STEPS_PER_REV / 360.0f)  // ~8.89 passos/grau
#define MOTOR_PULSE_US   25

// ── Resolução de Passo Radial (metros) ──────────────────────────
const float RES_STEP[] = {0.25f, 0.10f, 0.05f};
const char* RES_LABEL[] = {"250 mm (Fast)", "100 mm (Normal)", "50 mm (Fine)"};

// ── Variáveis Globais ───────────────────────────────────────────
Screen   activeScreen = SCR_MAIN_MENU;
bool     screenDirty  = true;
bool     motorUnlocked= false;
unsigned long motorUnlockedAtMs = 0;

bool     imuOnline    = false;
bool     scanOnline   = false;
bool     sdOnline     = false;

// Menu
bool     isCalibrated = false;
int      resLevel     = 1;
int      radialCount  = 8;

// ── Perfil de Caimento do Fundo conforme API Standard 650 (Item 5.4.4 e Anexo B.3.3) ──
enum BottomSlopeType {
    SLOPE_CONE_UP_1_120   = 0, // Cone-Up: 1:120 (-8,33 mm/m, fundo desce do centro para a quina)
    SLOPE_CONE_DOWN_1_120 = 1, // Cone-Down: 1:120 (+8,33 mm/m, fundo sobe do centro para a quina)
    SLOPE_FLAT            = 2  // Plano / Horizontal nominal (0,00 mm/m)
};
const char* SLOPE_LABEL[] = {
    "CONE-UP (1:120 / -8,3 mm/m)",
    "CONE-DOWN (1:120 / +8,3 mm/m)",
    "PLANO (0:1 / 0,0 mm/m)"
};
const char* SLOPE_SHORT[] = {
    "CONE-UP 1:120",
    "CONE-DOWN 1:120",
    "PLANO"
};
BottomSlopeType bottomSlopeType = SLOPE_CONE_UP_1_120; // Padrão de projeto API 650

// Calibração do Zero (Nadir)
bool     zeroCalibrated = false;
float    setupH       = 0.0f;
uint32_t setupH_mm    = 0;
float    setupL_zero  = 0.0f;

// Captura da Quina
float    setupTheta   = 0.0f;
float    setupL       = 0.0f;
uint32_t setupL_mm    = 0;
float    setupRmax    = 0.0f;
int32_t  setupCotaQ   = 0;
bool     quinaCaptured = false;

// Controle de Fluxo Radial
int      currentRadial = 1;

// Estado da Varredura
int      scanIndex    = 0;
int      scanTotalPts = 0;
float    lastZ        = 1.0f;
float    currentMotorAngle = 0.0f;
float    activeSweepSpeedSteps = MOTOR_SCAN_SPEED_NORMAL;
bool     scanRunning  = false;
bool     confirmingStop = false;
unsigned long lastScanStepMs = 0;
int      reshots      = 0;

// Último ponto coletado (para telemetria)
MeasPoint lastPoint = {0, 0.0f, 0.0f, 0.0f, 0, 0};

// Arquivo SD
File     dataFile;
char     fileName[32];

void getPointLabel(int radialIndex, int totalRadials, char* buffer);
float readIMUAngleDeg();
void printSerialRemoteStatus();
void writePointToSD(MeasPoint &pt);
bool insideZone(int16_t tx, int16_t ty, Zone &z);

static inline float computeCalibratedSignedPitchDeg(float rawUnwrappedDeg) {
    float span0to90 = imuRawHorizon90Deg - imuRawNadir0Deg;
    if (span0to90 < 10.0f) span0to90 = 81.34f;
    float span90to180 = imuRawZenith180Deg - imuRawHorizon90Deg;
    if (span90to180 < 10.0f) span90to180 = 88.46f;

    float calibratedSigned = 0.0f;
    if (rawUnwrappedDeg <= imuRawHorizon90Deg) {
        calibratedSigned = (rawUnwrappedDeg - imuRawNadir0Deg) * (90.0f / span0to90);
    } else {
        calibratedSigned = 90.0f + (rawUnwrappedDeg - imuRawHorizon90Deg) * (90.0f / span90to180);
    }
    return calibratedSigned + imuPitchOffsetDeg;
}

// ══════════════════════════════════════════════════════════════════
//  MODELO CINEMÁTICO E ÓPTICO V058 (Pivô 55,5 mm + Horizonte 88,30° + Supressão de Multicaminho Diedro na Quina)
// ══════════════════════════════════════════════════════════════════

static inline float getActiveSlopeRatio() {
    if (bottomSlopeType == SLOPE_CONE_DOWN_1_120) return 1.0f / 120.0f;   // +8,33 mm/m
    if (bottomSlopeType == SLOPE_CONE_UP_1_120)   return -1.0f / 120.0f;  // -8,33 mm/m
    return 0.0f;                                                          // Plano (0,0 mm/m)
}

static inline int32_t computeV058FloorKinematics(float L_m, float thetaDeg, float &outR_m) {
    if (L_m < 0.04f) {
        outR_m = 0.0f;
        return 0;
    }
    float thetaRad = thetaDeg * (M_PI / 180.0f);
    float cosT     = cosf(thetaRad);
    float sinT     = sinf(thetaRad);
    float dPivM    = SCAN_PIVOT_OFFSET_MM / 1000.0f;

    // 1. Raio horizontal cinemático com braço de alavanca do pivô (55,5 mm)
    float rReal = (L_m + dPivM) * sinT;
    if (setupRmax > 0.10f && rReal > setupRmax) {
        rReal = setupRmax;
    }
    outR_m = rReal;

    // 2. Cota Z cinemática base (exata de 0° a 58° e em retornos difusos diretos do piso)
    float zBaseMM = ((setupH - (L_m * cosT)) * 1000.0f)
                  + (SCAN_PIVOT_OFFSET_MM * (1.0f - cosT))
                  + (SCAN_KZ_CORRECTION_MM_PER_M * rReal);

    // 3. Compensação de Multicaminho Diedro de Canto (Quina Parede-Piso, θ > 68,5°)
    //    e Deriva de Centro de Fase em Incidência Rasante (58,0° < θ <= 68,5°):
    float zCorrMM = zBaseMM;
    if (thetaDeg > 58.0f) {
        float slopeRatio       = getActiveSlopeRatio();
        float zNominalFloorMM  = (slopeRatio * rReal) * 1000.0f;
        float opticalDeficitMM = zNominalFloorMM - zBaseMM;

        float maxOpticalEnvelopeMM = 0.0f;
        float gain = 0.0f;
        if (thetaDeg <= 68.5f) {
            float u = (thetaDeg - 58.0f) / 10.5f;
            maxOpticalEnvelopeMM = 8.0f + (16.0f * u);
            gain = 0.88f;
        } else {
            float uCorner = (thetaDeg - 68.5f) / 4.5f;
            if (uCorner > 1.0f) uCorner = 1.0f;
            maxOpticalEnvelopeMM = 24.0f + (18.0f * uCorner);
            gain = 0.92f;
        }

        if (opticalDeficitMM > 0.0f && opticalDeficitMM <= (maxOpticalEnvelopeMM + 6.0f)) {
            zCorrMM = zBaseMM + (gain * opticalDeficitMM);
        } else if (opticalDeficitMM > (maxOpticalEnvelopeMM + 6.0f) && thetaDeg >= 68.5f && opticalDeficitMM < 46.0f) {
            // Na zona de diedro da quina (> 68,5°), limita picos intensos de reflexão mantendo detecção de degraus > 46 mm
            zCorrMM = zBaseMM + (gain * maxOpticalEnvelopeMM);
        }
    }

    return (int32_t)lroundf(zCorrMM);
}

// ══════════════════════════════════════════════════════════════════
//  RELÓGIO EM TEMPO REAL INTERNO (RTC ESP32 — DATA E HORA POR PONTO)
// ══════════════════════════════════════════════════════════════════

void initInternalRTCFromCompileTime() {
    const char* compDate = __DATE__;
    const char* compTime = __TIME__;

    char mStr[4] = {0};
    int day = 1, year = 2026, hh = 0, mm = 0, ss = 0;
    sscanf(compDate, "%3s %d %d", mStr, &day, &year);
    sscanf(compTime, "%d:%d:%d", &hh, &mm, &ss);

    const char* months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    int mon = 0;
    for (int i = 0; i < 12; i++) {
        if (strncmp(mStr, months[i], 3) == 0) {
            mon = i;
            break;
        }
    }

    struct tm t = {0};
    t.tm_year = year - 1900;
    t.tm_mon  = mon;
    t.tm_mday = day;
    t.tm_hour = hh;
    t.tm_min  = mm;
    t.tm_sec  = ss;

    time_t compileEpoch = mktime(&t);
    time_t nowEpoch = time(NULL);

    if (nowEpoch < compileEpoch) {
        struct timeval tv = { .tv_sec = compileEpoch, .tv_usec = 0 };
        settimeofday(&tv, NULL);
    }
}

void getCurrentDateTimeStr(char* dateBuf, size_t dateLen, char* timeBuf, size_t timeLen) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    snprintf(dateBuf, dateLen, "%02d/%02d/%04d",
             timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    snprintf(timeBuf, timeLen, "%02d:%02d:%02d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

bool setInternalRTCFromSerial(const char* input) {
    int d = 0, m = 0, y = 0, hh = 0, mm = 0, ss = 0;
    if (sscanf(input, "%d/%d/%d %d:%d:%d", &d, &m, &y, &hh, &mm, &ss) == 6) {
        // DD/MM/YYYY
    } else if (sscanf(input, "%d-%d-%d %d:%d:%d", &y, &m, &d, &hh, &mm, &ss) == 6) {
        // YYYY-MM-DD
    } else {
        return false;
    }

    if (y < 2024 || m < 1 || m > 12 || d < 1 || d > 31) return false;

    struct tm t = {0};
    t.tm_year = y - 1900;
    t.tm_mon  = m - 1;
    t.tm_mday = d;
    t.tm_hour = hh;
    t.tm_min  = mm;
    t.tm_sec  = ss;

    time_t newEpoch = mktime(&t);
    struct timeval tv = { .tv_sec = newEpoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    return true;
}

// ══════════════════════════════════════════════════════════════════
//  GERADOR DE PULSOS POR ACUMULADOR DE FASE 64-BIT COM SPINLOCK MULTI-CORE
// ══════════════════════════════════════════════════════════════════
void setSmoothBgSpeed(float speedStepsPerSec) {
    if (fabsf(speedStepsPerSec) < 0.5f) {
        portENTER_CRITICAL(&motorTimerMux);
        bgStepIntervalUs = 0;
        portEXIT_CRITICAL(&motorTimerMux);
        return;
    }
    int8_t newDir = (speedStepsPerSec >= 0.0f) ? 1 : -1;
    uint32_t newIntervalUs = (uint32_t)lroundf(1000000.0f / fabsf(speedStepsPerSec));
    if (newIntervalUs < 2000UL) newIntervalUs = 2000UL; // limite seguro 500 passos/s

    portENTER_CRITICAL(&motorTimerMux);
    bool wasStopped = (bgStepIntervalUs == 0) || (newDir != bgStepDir);
    bgStepDir = newDir;
    digitalWrite(PIN_DIR, (bgStepDir > 0) ? LOW : HIGH); // V052: Invertido para TMC2209
    bgStepIntervalUs = newIntervalUs;

    // Se estava parado ou inverteu sentido, agenda o 1º pulso após 1,5 ms de setup de DIR;
    // os pulsos seguintes usam newIntervalUs preservando a fase de 64 bits!
    if (wasStopped) {
        bgNextStepUs = esp_timer_get_time() + 1500LL;
    }
    portEXIT_CRITICAL(&motorTimerMux);
}

void onMotorStepTimer(void* arg) {
    if (!motorBgActive) return;
    uint8_t mode = motorBgMode;
    if (mode == 0) {
        stepper.run();
        return;
    }
    if (bgStepIntervalUs == 0) return;

    int8_t dir = 0;
    bool doStep = false;

    // ESP_TIMER_TASK executa no contexto da task esp_timer (não em ISR): usa portENTER_CRITICAL padrão
    portENTER_CRITICAL(&motorTimerMux);
    uint32_t intervalUs = bgStepIntervalUs;
    if (intervalUs > 0) {
        if (mode == 2 && bgCurrentPosSteps == bgTargetPosSteps) {
            bgStepIntervalUs = 0;
        } else {
            int64_t nowUs = esp_timer_get_time();
            if (nowUs >= bgNextStepUs) {
                bgNextStepUs += (int64_t)intervalUs;
                if (nowUs - bgNextStepUs > (int64_t)intervalUs) {
                    bgNextStepUs = nowUs + (int64_t)intervalUs;
                }
                dir = bgStepDir;
                if (mode == 2) {
                    dir = (bgTargetPosSteps > bgCurrentPosSteps) ? 1 : -1;
                }
                bgCurrentPosSteps += dir;
                if (mode == 2 && bgCurrentPosSteps == bgTargetPosSteps) {
                    bgStepIntervalUs = 0;
                }
                doStep = true;
            }
        }
    }
    portEXIT_CRITICAL(&motorTimerMux);

    if (doStep) {
        digitalWrite(PIN_DIR, (dir > 0) ? LOW : HIGH); // V052: Invertido para TMC2209
        digitalWrite(PIN_STEP, HIGH);
        ets_delay_us(MOTOR_PULSE_US);
        digitalWrite(PIN_STEP, LOW);
    }
}

// ── Zonas de Interação ──────────────────────────────────────────
const int MAIN_MENU_ZONE_COUNT = 3;
Zone mainMenuZones[MAIN_MENU_ZONE_COUNT] = {
    { 10,  35, 300, 50 },   // BOTTOM SCAN
    { 10,  95, 300, 50 },   // SHELL SCAN
    { 10, 155, 300, 50 }    // HORIZON LEVEL
};

const int BOTTOM_MENU_ZONE_COUNT = 5;
Zone bottomMenuZones[BOTTOM_MENU_ZONE_COUNT] = {
    {  10,  35, 145, 50 },   // SURVEY LINES (index 0)
    { 165,  35, 145, 50 },   // RESOLUTION (index 1)
    {  10,  95, 300, 50 },   // PERFIL API 650 (index 2)
    {  10, 155,  50, 50 },   // BACK (index 3)
    {  70, 155, 240, 50 }    // START PROJECT (index 4)
};

Zone setupCaptureZone   = {10, 110, 300, 48};
Zone setupAbortZone     = {10, 168, 300, 42};
Zone setupReaimZone     = {10, 175, 145, 40};
Zone setupAbortHalfZone = {165, 175, 145, 40};
Zone horizonExitZone    = { 10, 175, 300, 40}; // V052: Botao Voltar da tela Horizonte

// ── Zonas da Tela de Captura do Joint ───────────────────────────
Zone jointDownZone   = {  4,  98,  96, 54};
Zone jointStepZone   = {104,  98, 112, 54};
Zone jointUpZone     = {220,  98,  96, 54};
Zone jointNadirZone  = {  4, 158,  84, 54};
Zone jointStartZone  = { 94, 158, 222, 54};

const float JOINT_STEPS[] = {0.1f, 0.2f, 0.5f, 1.0f};
const uint8_t NUM_JOINT_STEPS = 4;
uint8_t jointStepIdx = 1; // Padrão: 0.2°

Zone checkScanZone      = {10, 100, 300, 45};
Zone checkUndoZone      = {10, 160, 145, 45};
Zone checkAbortHalfZone = {165, 160, 145, 45};
Zone checkAbortFullZone = {10, 160, 300, 45};
Zone checkFinishZone    = {10, 130, 300, 50};

Zone scanStopZone  = {10, 185, 300, 40};
Zone confirmNoZone = {10, 140, 145, 50};
Zone confirmYesZone= {165, 140, 145, 50};

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
    tft.setTextSize(1);
    tft.fillRect(0, 0, 320, 24, C_HEADER);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(2, 17);
    tft.print("GeoTank v3.0");

    int16_t logo_x = 124;
    int16_t logo_y = 4;
    for (int y = 0; y < priner_logo_h; y++) {
        for (int x = 0; x < priner_logo_w; x++) {
            uint16_t color = pgm_read_word(&priner_logo_data[y * priner_logo_w + x]);
            if (color != 0xFFFF) {
                tft.drawPixel(logo_x + x, logo_y + y, color);
            }
        }
    }

    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(C_DIM);
    tft.setCursor(logo_x + priner_logo_w + 3, 17);
    tft.print("Priner");

    tft.setFont(NULL);
    tft.setTextSize(1);
    int16_t xr = 316;
    
    tft.drawRect(xr - 36, 4, 34, 16, C_BORDER);
    tft.fillRect(xr - 38, 8, 2, 8, C_BORDER);
    
    tft.setTextColor(C_GREEN);
    drawRightAligned("85%", xr - 10, 12);
    
    xr -= 46;

    tft.setTextColor(imuOnline ? C_GREEN : C_RED);
    drawRightAligned("IMU", xr, 12);
    xr -= 28;

    tft.setTextColor(sdOnline ? C_GREEN : C_RED);
    drawRightAligned("SD", xr, 12);
    xr -= 24;

    tft.setTextColor(scanOnline ? C_GREEN : C_RED);
    drawRightAligned("SCAN", xr, 12);
}

void drawFooterTelemetryLine() {
    tft.fillRect(0, 230, 320, 10, C_FOOTER);
    tft.setFont(NULL);
    tft.setTextSize(1);
    
    tft.setCursor(4, 231);
    tft.setTextColor(C_WHITE);
    char pStr[24];
    sprintf(pStr, "PITCH:%5.1f deg  ", imuPitchDeg);
    tft.print(pStr);

    uint16_t rollColor = C_GREEN;
    if (fabsf(imuRollDeg) > 1.5f) {
        rollColor = C_RED;
    } else if (fabsf(imuRollDeg) > 0.5f) {
        rollColor = C_YELLOW;
    }
    tft.setTextColor(rollColor);
    char rStr[24];
    sprintf(rStr, "ROLL:%+5.1f deg", imuRollDeg);
    tft.print(rStr);

    char azStr[24];
    if (activeScreen == SCR_SETUP || activeScreen == SCR_JOINT || activeScreen == SCR_CHECKPOINT || activeScreen == SCR_SCANNING) {
        int pt = currentRadial - 1;
        if (pt < 0) pt = 0;
        if (radialCount == 8) {
            const char* card[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
            sprintf(azStr, "AZ: %s (%.0f deg)", card[pt % 8], (pt % 8) * 45.0f);
        } else {
            float ang = pt * (360.0f / radialCount);
            sprintf(azStr, "AZ: P%d (%.0f deg)", pt, ang);
        }
        tft.setTextColor(C_CYAN);
        drawRightAligned(azStr, 316, 235);
    } else if (activeScreen == SCR_HORIZON) {
        tft.setTextColor(horizonLocked ? C_GREEN : C_YELLOW);
        drawRightAligned(horizonLocked ? "LOCKED 90.0" : "ALVO: 90.0 deg", 316, 235);
    } else {
        tft.setTextColor(C_DIM);
        drawRightAligned("AZ: --", 316, 235);
    }
}

void drawFooter(const char* msg) {
    tft.fillRect(0, 218, 320, 22, C_FOOTER);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    tft.setCursor(4, 220);
    tft.print(msg);

    drawFooterTelemetryLine();
}

void beep(int ms) {
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
    tft.setCursor(4, 220);
    tft.print(msg);
    drawFooterTelemetryLine();
    beep(150);
    isErrorShowing = true;
    errorMessageUntil = millis() + 2500;
}

// ══════════════════════════════════════════════════════════════════
//  FILTROS: JANELA TEMPORAL (ESTÁTICO) E MEDIANA DE COTA Z (DINÂMICO)
// ══════════════════════════════════════════════════════════════════

void resetTimeWindow() {
    winCount = 0;
    winSpreadMM = 0;
    measuredRateHz = 0.0f;
    stepJumpCount = 0;
    zHistCount = 0;
}

int32_t filterDynamicCotaMM(int32_t rawCotaMM) {
    // V056: Filtro de mediana movel tripla anti-jitter restaurado.
    // Degraus fisicos reais de tubulacao ou coluna (> 30 mm) sao preservados instantaneamente.
    if (zHistCount == 0) {
        zHistMM[0] = rawCotaMM;
        zHistCount = 1;
        return rawCotaMM;
    } else if (zHistCount == 1) {
        zHistMM[1] = rawCotaMM;
        zHistCount = 2;
        return (zHistMM[0] + zHistMM[1]) / 2;
    } else {
        zHistMM[0] = zHistMM[1];
        zHistMM[1] = zHistMM[2];
        zHistMM[2] = rawCotaMM;
        zHistCount = 3;

        int32_t a = zHistMM[0], b = zHistMM[1], c = zHistMM[2];
        if (a > b) { int32_t t = a; a = b; b = t; }
        if (b > c) { int32_t t = b; b = c; c = t; }
        if (a > b) { int32_t t = a; a = b; b = t; }
        int32_t median = b;

        return median; // V057: Mediana pura de 3 pontos sem vazamento de picos opticos de reflexao
    }
}

uint32_t pushSampleToTimeWindow(uint32_t newMM, unsigned long activeWindowMs) {
    unsigned long now = millis();

    if (winCount >= 2 && abs((long)newMM - (long)avgDistMM) > 80) {
        stepJumpCount++;
        if (stepJumpCount >= 2) {
            resetTimeWindow();
        }
    } else {
        stepJumpCount = 0;
    }

    int writeIdx = 0;
    for (int i = 0; i < winCount; i++) {
        if (now - winBuf[i].tMs <= activeWindowMs) {
            winBuf[writeIdx++] = winBuf[i];
        }
    }
    winCount = writeIdx;

    if (winCount >= MAX_WINDOW_SAMPLES) {
        for (int i = 1; i < MAX_WINDOW_SAMPLES; i++) {
            winBuf[i - 1] = winBuf[i];
        }
        winCount = MAX_WINDOW_SAMPLES - 1;
    }

    winBuf[winCount].distMM = newMM;
    winBuf[winCount].tMs    = now;
    winCount++;

    if (winCount >= 2) {
        unsigned long dt = winBuf[winCount - 1].tMs - winBuf[0].tMs;
        if (dt > 0) {
            measuredRateHz = ((float)(winCount - 1) * 1000.0f) / (float)dt;
        }
    } else {
        measuredRateHz = 0.0f;
    }

    uint32_t sorted[MAX_WINDOW_SAMPLES];
    for (int i = 0; i < winCount; i++) {
        sorted[i] = winBuf[i].distMM;
    }
    for (int i = 0; i < winCount - 1; i++) {
        for (int j = i + 1; j < winCount; j++) {
            if (sorted[j] < sorted[i]) {
                uint32_t tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    winMinMM    = sorted[0];
    winMaxMM    = sorted[winCount - 1];
    winSpreadMM = winMaxMM - winMinMM;

    int startIdx = 0;
    int endIdx   = winCount - 1;
    if (winCount >= 6) {
        startIdx = 2;
        endIdx   = winCount - 3;
    } else if (winCount >= 3) {
        startIdx = 1;
        endIdx   = winCount - 2;
    }

    uint32_t sum = 0;
    uint32_t usedCount = 0;
    for (int i = startIdx; i <= endIdx; i++) {
        sum += sorted[i];
        usedCount++;
    }

    if (usedCount > 0) {
        avgDistMM = (sum + (usedCount / 2)) / usedCount;
    } else {
        avgDistMM = newMM;
    }
    avgDistM = (float)avgDistMM / 1000.0f;
    return avgDistMM;
}

// ══════════════════════════════════════════════════════════════════
//  SENSOR DE DISTÂNCIA — DRIVER OFICIAL M01-V0.4 (UART 9600 bps)
// ══════════════════════════════════════════════════════════════════

#if !SIMULACAO_WOKWI
static inline bool checkSumM01OK(const uint8_t* f, int n) {
    if (n < 3) return false;
    uint32_t sum = 0;
    for (int i = 1; i < n - 1; i++) sum += f[i];
    return ((uint8_t)sum) == f[n - 1];
}

static inline int expectedM01FrameLen(const uint8_t* f, int pos) {
    if (pos < 4) return 13;
    if (f[1] == 0xEE || f[3] == 0xEE) return 9;
    if (pos < 6) return 13;
    if (f[2] == 0x01 && f[3] == 0xBE) return 9;
    if (f[1] == 0x80 && f[3] == 0x06) return 9;
    uint16_t payloadLen = ((uint16_t)f[4] << 8) | f[5];
    if (payloadLen == 1) return 9;
    return 13;
}

bool decodeM01Frame(const uint8_t* f, int len, float &dist_m, uint32_t &dist_mm) {
    if (len < 13 || f[0] != 0xAA) return false;
    if (!checkSumM01OK(f, 13)) return false;
    if (f[1] == 0xEE || f[3] == 0xEE) return false;

    uint32_t raw_bcd = 0;
    bool is_bcd = true;
    for (int i = 0; i < 4; i++) {
        uint8_t b = f[6 + i];
        uint8_t hi = (b >> 4) & 0x0F;
        uint8_t lo = b & 0x0F;
        if (hi > 9 || lo > 9) {
            is_bcd = false;
            break;
        }
        raw_bcd = raw_bcd * 100 + (hi * 10 + lo);
    }

    // V058: Protocolo M01-V0.4 é estritamente BCD nos bytes 6..9.
    // Rejeita pacotes de status/erro óptico com nibbles hexadecimais A..F (evita falsos 11,9m / 21,3m / 44,6m na quina)
    if (!is_bcd || raw_bcd < 40 || raw_bcd > 50000) {
        return false;
    }
    dist_mm = raw_bcd;

    dist_m = (float)dist_mm / 1000.0f;
    return true;
}

void initLaserUART() {
    Serial2.end();
    pinMode(PIN_LASER_TX, OUTPUT);
    digitalWrite(PIN_LASER_TX, HIGH);
    pinMode(PIN_LASER_RX, INPUT_PULLUP);
    delay(50);

    Serial2.setRxBufferSize(512);
    Serial2.begin(9600, SERIAL_8N1, PIN_LASER_RX, PIN_LASER_TX);
    delay(350);
    while (Serial2.available()) Serial2.read();
    laserContActive = false;
    keepLaserAlive  = false;
}

static inline void sendActiveContinuousCmd() {
    if (useQuickLaserMode) {
        Serial2.write(M01_CMD_QUICK, sizeof(M01_CMD_QUICK));
    } else {
        Serial2.write(M01_CMD_CONT, sizeof(M01_CMD_CONT));
    }
    Serial2.flush();
}

void laserPointerOn() {
    keepLaserAlive = true;
    if (laserContActive && (millis() - lastValidLaserMs < 1500UL)) return;
    while (Serial2.available()) Serial2.read();
    sendActiveContinuousCmd();
    laserContActive = true;
    lastContCmdMs = millis();
    lastAnyByteLaserMs = millis();
}

void laserStartContinuous(bool forceRestart = false) {
    keepLaserAlive = true;
    if (laserContActive && !forceRestart && (millis() - lastValidLaserMs < 1500UL)) {
        return;
    }
    while (Serial2.available()) Serial2.read();
    sendActiveContinuousCmd();
    laserContActive = true;
    lastContCmdMs = millis();
    lastAnyByteLaserMs = millis();
}

void laserStop() {
    keepLaserAlive  = false;
    laserContActive = false;
    delay(25);
    while (Serial2.available()) Serial2.read();
    Serial2.write(M01_CMD_LASER_OFF, sizeof(M01_CMD_LASER_OFF));
    Serial2.flush();
    delay(25);
    while (Serial2.available()) Serial2.read();
    Serial.println("[LASER] Emissao optica desativada.");
}
#else
void initLaserUART() {}
void laserPointerOn() {}
void laserStartContinuous(bool forceRestart = false) {}
void laserStop() {}
#endif

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
            uint32_t d_mm = (uint32_t)(((duration * 0.343f) / 2.0f) + 0.5f);
            ultimaBrutaMM = d_mm;
            ultimaDistanciaMM = pushSampleToTimeWindow(d_mm, 1200);
            ultimaDistanciaLaser = (float)ultimaDistanciaMM / 1000.0f;
            novaLeituraLaser = true;
        }
    }
#else
    static uint8_t fBuf[24];
    static int fPos = 0;
    static unsigned long lastByteMs = 0;
    static int consecutiveLaserErrors = 0; // V053 Anti-flicker: compartilhado entre blocos

    if (fPos > 0 && millis() - lastByteMs > 55) {
        fPos = 0;
    }

    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        lastAnyByteLaserMs = millis();

        if (fPos == 0 && b != 0xAA) continue;
        fBuf[fPos++] = b;
        lastByteMs = lastAnyByteLaserMs;

        int targetLen = expectedM01FrameLen(fBuf, fPos);

        if (fPos >= targetLen) {
            // Valida Checksum M01 antes de aceitar o frame; se falhar, desliza janela ate proximo 0xAA
            if (!checkSumM01OK(fBuf, targetLen)) {
                int nextAA = -1;
                for (int k = 1; k < fPos; k++) {
                    if (fBuf[k] == 0xAA) {
                        nextAA = k;
                        break;
                    }
                }
                if (nextAA > 0) {
                    int rem = fPos - nextAA;
                    memmove(fBuf, fBuf + nextAA, rem);
                    fPos = rem;
                } else {
                    fPos = 0;
                }
                continue;
            }

            scanOnline = true;

            // V053 Anti-flicker: pacote de 9 bytes (0xEE ou status payloadLen==1 de campo proximo/borda)
            // Debounce de erros consecutivos: não reseta o feixe óptico a cada transiente isolado!
            if (targetLen == 9) {
                fPos = 0;
                consecutiveLaserErrors++;
                if ((keepLaserAlive || scanRunning) && (consecutiveLaserErrors >= 3 || millis() - lastContCmdMs > 1200UL)) {
                    if (scanRunning && !confirmingStop) reshots++;
                    sendActiveContinuousCmd();
                    laserContActive = true;
                    lastContCmdMs = millis();
                    consecutiveLaserErrors = 0;
                }
                continue;
            }

            if (targetLen == 13) {
                float dist_m = 0.0f;
                uint32_t dist_mm = 0;
                if (decodeM01Frame(fBuf, 13, dist_m, dist_mm)) {
                    ultimaBrutaMM = dist_mm;
                    uint32_t filt_mm = pushSampleToTimeWindow(dist_mm, windowDurationMs);

                    if (activeScreen == SCR_SCANNING && scanRunning) {
                        ultimaDistanciaMM    = dist_mm;
                        ultimaDistanciaLaser = (float)dist_mm / 1000.0f;
                    } else {
                        ultimaDistanciaMM    = filt_mm;
                        ultimaDistanciaLaser = (float)filt_mm / 1000.0f;
                    }
                    novaLeituraLaser = true;
                    lastValidLaserMs = millis();
                    laserContActive  = true;
                    consecutiveLaserErrors = 0; // V053: zera contador de erros ao receber leitura válida
                }
            }
            fPos = 0;
        }
    }

    // Watchdog óptico baseado em lastValidLaserMs (1500 ms): se o sensor abortar sem enviar 9 bytes, rearma!
    if ((keepLaserAlive || scanRunning) && !confirmingStop) {
        unsigned long silenceLimitMs = 1500UL;
        if (millis() - lastValidLaserMs > silenceLimitMs && millis() - lastContCmdMs > silenceLimitMs) {
            if (scanRunning) reshots++;
            sendActiveContinuousCmd();
            laserContActive = true;
            lastContCmdMs = millis();
        }
    }
#endif
}

int32_t readDistanceTimedAverageMM(unsigned long durationMs = 1500, int minSamples = 4, bool keepContinuousAlive = true) {
#if SIMULACAO_WOKWI
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long duration = pulseIn(PIN_ECHO, HIGH, 50000);
    if (duration == 0) return -1;
    return (int32_t)(((duration * 0.343f) / 2.0f) + 0.5f);
#else
    resetTimeWindow();
    keepLaserAlive = true;

    // V058: Rearma imediatamente o trem contínuo ao iniciar medição estática após Slew Rápido (0° -> 90° -> Quina)
    if (!laserContActive || (millis() - lastValidLaserMs > 350UL)) {
        while (Serial2.available()) Serial2.read();
        sendActiveContinuousCmd();
        laserContActive = true;
        lastContCmdMs = millis();
        lastAnyByteLaserMs = millis();
    }

    unsigned long tStart = millis();
    unsigned long maxWaitMs = durationMs + 1200;

    while (millis() - tStart < maxWaitMs) {
        pollLaser();
        static unsigned long lastImuSample = 0;
        if (millis() - lastImuSample >= 25) {
            lastImuSample = millis();
            readIMUAngleDeg();
        }
        if (millis() - tStart >= durationMs && winCount >= minSamples) {
            break;
        }
        yield();
    }

    if (!keepContinuousAlive) {
        laserStop();
    }

    if (winCount > 0) {
        ultimaDistanciaMM    = avgDistMM;
        ultimaDistanciaLaser = avgDistM;
        novaLeituraLaser     = true;
        lastValidLaserMs     = millis();
        Serial.printf("[MEDIA TEMPORAL CRAVADA] >>> %u mm (%.3f m) <<< | N=%d amostras (%.1f Hz) | Min:%u Max:%u (Oscil:%u mm)\n",
                      (unsigned int)avgDistMM, avgDistM, winCount, measuredRateHz,
                      (unsigned int)winMinMM, (unsigned int)winMaxMM, (unsigned int)winSpreadMM);
        return (int32_t)avgDistMM;
    }

    Serial.println("[LASER] ERRO: Janela temporal encerrada sem amostras validas.");
    return -1;
#endif
}

bool probeLaserHardware() {
#if SIMULACAO_WOKWI
    return true;
#else
    while (Serial2.available()) Serial2.read();
    Serial2.write(M01_CMD_VOLTAGE, sizeof(M01_CMD_VOLTAGE));
    Serial2.flush();

    uint8_t buf[16];
    int pos = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 400) {
        while (Serial2.available()) {
            uint8_t b = Serial2.read();
            if (pos == 0 && b != 0xAA) continue;
            buf[pos++] = b;
            if (pos >= 9) {
                uint8_t vHi = buf[6];
                uint8_t vLo = buf[7];
                float vSupply = ((vHi >> 4) * 10 + (vHi & 0x0F)) +
                                (((vLo >> 4) * 10 + (vLo & 0x0F)) / 100.0f);
                Serial.printf("[LASER] M01-V0.4 detectado na UART2 (Tensao interna: %.2fV)\n", vSupply);
                scanOnline = true;
                return true;
            }
        }
        yield();
    }
    return scanOnline;
#endif
}

// ══════════════════════════════════════════════════════════════════
//  CONTROLE LINEAR DO MOTOR ACIMA DA RESSONÂNCIA DO DRV8825 (CORE 0)
// ══════════════════════════════════════════════════════════════════

void enableMotor() {
    stepper.enableOutputs();
    motorCurrentlyEnergized = true;
    motorEnergizedSinceMs   = millis();
}

void disableMotor() {
    motorBgActive    = false;
    bgStepIntervalUs = 0;
    stepper.setSpeed(0.0f);
    stepper.disableOutputs();
    motorCurrentlyEnergized = false;
}

// Aproximação suave mantendo SEMPRE velocidade >= 14 passos/s (> 1,5°/s) enquanto em movimento
// para nunca cair na zona de stick-slip (< 1°/s) do DRV8825!
static inline float computeDampedSeekSpeed(float error, float signDir, float prevSpeed, float maxSpeed = MOTOR_SEEK_MAX_SPEED) {
    float absErr = fabsf(error);
    float targetMag = 0.0f;

    if (absErr > 10.0f) {
        targetMag = maxSpeed;
    } else if (absErr > 1.4f) {
        // Desacelera suavemente de maxSpeed até MOTOR_MIN_SMOOTH_SPEED (14 passos/s)
        targetMag = MOTOR_MIN_SMOOTH_SPEED + ((absErr - 1.4f) / 8.6f) * (maxSpeed - MOTOR_MIN_SMOOTH_SPEED);
    } else if (absErr > 0.28f) {
        targetMag = MOTOR_MIN_SMOOTH_SPEED;
    } else {
        return 0.0f;
    }

    float targetSpd = (error > 0.0f) ? (signDir * targetMag) : (-signDir * targetMag);
    if ((prevSpeed * targetSpd) < 0.0f) {
        return targetSpd;
    }
    return (0.70f * prevSpeed) + (0.30f * targetSpd);
}

// Retorna a velocidade linear base (acima de 45°) em passos/s (sempre >= 22 passos/s = 2,47°/s!)
static inline float getConstantSweepSpeedSteps() {
    if (customSweepSpeedSteps >= 1.5f) return customSweepSpeedSteps;
    if (resLevel == 0) return MOTOR_SCAN_SPEED_FAST;   // 36,0 passos/s (~4,05°/s)
    if (resLevel == 2) return MOTOR_SCAN_SPEED_FINE;   // 22,0 passos/s (~2,47°/s)
    return MOTOR_SCAN_SPEED_NORMAL;                    // 28,0 passos/s (~3,15°/s)
}

// Leitura calma do IMU após parar o motor (descarta vibração mecânica de passo e sincroniza o filtro IIR na hora)
float readSettledIMUAngleDeg(int samples = 8, int delayMs = 20) {
    setSmoothBgSpeed(0.0f);
    delay(120);
    forceImuFilterSync = true;
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
        sum += readIMUAngleDeg();
        delay(delayMs);
    }
    return sum / (float)samples;
}

void seekNadirBlocking(bool fast = false) {
    motorBgActive = false;
    enableMotor();
    
    tft.fillScreen(C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_YELLOW);
    drawCentered(fast ? "FAST SEEKING NADIR..." : "SEEKING NADIR...", 160, 80);
    
    float maxSpd = fast ? MOTOR_FAST_SEEK_SPEED : MOTOR_SEEK_MAX_SPEED;

    // Leitura estática inicial para definir sentido único de aproximação contínua (jamais inverte em motorBgMode=1)
    readSettledIMUAngleDeg(4, 15);
    float initialSignedTheta = computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
    float initialError       = 0.0f - initialSignedTheta;

    if (fabsf(initialError) > 1.80f) {
        float smoothSpd = 0.0f;
        setSmoothBgSpeed(0.0f);
        motorBgMode   = 1;
        motorBgActive = true;

        int lastDisplayedTenths = -9999;
        unsigned long lastDraw = 0;
        unsigned long tStart = millis();

        while (millis() - tStart < 15000) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            
            imuAccelX = a.acceleration.x;
            imuAccelY = a.acceleration.y;
            imuAccelZ = a.acceleration.z;
            
            float unwrappedDeg = atan2f(-a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
            if (unwrappedDeg < -90.0f) unwrappedDeg += 360.0f;
            imuRawUnwrappedDeg = unwrappedDeg;

            float currentThetaCal = computeCalibratedSignedPitchDeg(unwrappedDeg);
            float error = 0.0f - currentThetaCal;
            imuPitchDeg = fabsf(currentThetaCal);
            imuRollDeg  = atan2f(a.acceleration.y, sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * (180.0f / M_PI);

            int tenths = (int)lroundf(fabsf(currentThetaCal) * 10.0f);
            if (millis() - lastDraw > 300 && tenths != lastDisplayedTenths) {
                lastDraw = millis();
                lastDisplayedTenths = tenths;
                tft.fillRect(75, 125, 170, 30, C_BG);
                tft.setFont(NULL);
                tft.setTextSize(3);
                tft.setTextColor(fabsf(error) <= 1.80f ? C_GREEN : C_YELLOW);
                char angStr[16];
                sprintf(angStr, "%.1f deg", fabsf(currentThetaCal));
                drawCentered(angStr, 160, 140);
            }

            // Desarma o modo contínuo aos 1,80° antes do atraso de fase de ~200 ms do DLPF 5 Hz gerar overshoot/hunting
            if (fabsf(error) <= 1.80f || (error * initialError) <= 0.0f) {
                setSmoothBgSpeed(0.0f);
                break;
            }

            smoothSpd = computeDampedSeekSpeed(error, -1.0f, smoothSpd, maxSpd);
            setSmoothBgSpeed(smoothSpd);

            pollLaser();
            delay(20);
        }
    }
    
    motorBgActive = false;
    setSmoothBgSpeed(0.0f);

    // Ajuste fino determinístico por contagem exata de micropassos (até 2 passes após estabilização real do DLPF 5 Hz)
    for (int pass = 0; pass < 2; pass++) {
        readSettledIMUAngleDeg(8, 20);
        float residualErr0 = 0.0f - computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
        if (fabsf(residualErr0) < 0.14f || fabsf(residualErr0) > 8.0f) break;
        int32_t trimSteps = (int32_t)lroundf(fabsf(residualErr0) * STEPS_PER_DEG);
        if (trimSteps < 1) break;
        int8_t stepDir = (residualErr0 >= 0.0f) ? -1 : 1;
        portENTER_CRITICAL(&motorTimerMux);
        bgCurrentPosSteps = 0;
        bgTargetPosSteps  = stepDir * trimSteps;
        motorBgMode       = 2;
        motorBgActive     = true;
        portEXIT_CRITICAL(&motorTimerMux);
        setSmoothBgSpeed(stepDir * 18.0f);
        unsigned long tTrim = millis();
        while (bgCurrentPosSteps != bgTargetPosSteps && (millis() - tTrim < 3000)) {
            pollLaser();
            delay(5);
        }
        setSmoothBgSpeed(0.0f);
        motorBgActive = false;
    }

    float finalNadirDeg = readSettledIMUAngleDeg(6, 20);
    tft.fillRect(75, 125, 170, 30, C_BG);
    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.setTextColor(C_GREEN);
    char finalAngStr[16];
    sprintf(finalAngStr, "%.1f deg", finalNadirDeg);
    drawCentered(finalAngStr, 160, 140);

    tft.setTextSize(1);
    bgCurrentPosSteps = 0;
    stepper.setCurrentPosition(0);
    motorEnergizedSinceMs = millis();
}

void moveMotorBlocking(long targetPosition) {
    motorBgActive = false;
    enableMotor();
    stepper.setMaxSpeed(MOTOR_RETURN_SPEED);
    stepper.setAcceleration(MOTOR_RETURN_ACCEL);
    stepper.moveTo(targetPosition);

    motorBgMode   = 0; // run() com rampa suave única
    motorBgActive = true;

    while (stepper.distanceToGo() != 0) {
        pollLaser();
        delay(5);
    }

    motorBgActive = false;
}

// Avanço fino direto por micropassos (+grau gira no sentido positivo 0°->90°->180°; -grau gira de volta)
void nudgeAngleBySteps(float deltaDeg) {
    int32_t steps = (int32_t)lroundf(fabsf(deltaDeg) * STEPS_PER_DEG);
    if (steps < 1) steps = 1;
    enableMotor();
    keepLaserAlive = true;
    laserStartContinuous(false);

    // No referencial mecânico do equipamento: dir = -1 aumenta o ângulo (0° -> 90° -> 180°), dir = +1 diminui
    int8_t stepDir = (deltaDeg >= 0.0f) ? -1 : 1;

    portENTER_CRITICAL(&motorTimerMux);
    bgCurrentPosSteps = 0;
    bgTargetPosSteps  = stepDir * steps;
    motorBgMode       = 2;
    motorBgActive     = true;
    portEXIT_CRITICAL(&motorTimerMux);

    setSmoothBgSpeed(stepDir * 24.0f);

    unsigned long t0 = millis();
    while (bgCurrentPosSteps != bgTargetPosSteps && (millis() - t0 < 8000)) {
        pollLaser();
        delay(5);
    }
    setSmoothBgSpeed(0.0f);
    motorBgActive = false;
    motorEnergizedSinceMs = millis();

    float settled = readSettledIMUAngleDeg(6, 20);
    Serial.printf("[AJUSTE FINO CONCLUIDO] Moveu %+.2f° (%d micropassos) | Pitch Calibrado=%.2f° | IMU Bruto=%.2f° (Trim=%+.2f°)\n",
                  deltaDeg, (int)steps, settled, imuRawUnwrappedDeg, imuPitchOffsetDeg);
    Serial.println("  -> Quando o laser coincidir com seu prumo/nivel (0°, 90° ou 180°), envie [O] para gravar na Flash!");
}

void seekAngleBlocking(float targetAngle, bool fast = false) {
    targetAngle = constrain(targetAngle, 0.0f, 185.0f);
    if (targetAngle == 0.0f) {
        seekNadirBlocking(fast);
        return;
    }
    motorBgActive = false;
    enableMotor();
    
    tft.fillScreen(C_BG);
    tft.setTextSize(2);
    tft.setTextColor(C_YELLOW);
    if (targetAngle >= 89.0f && targetAngle <= 91.0f) {
        drawCentered(fast ? "FAST SEEKING HORIZON (90°)..." : "SEEKING HORIZON (90°)...", 160, 100);
    } else if (targetAngle >= 175.0f) {
        drawCentered("SEEKING 180 ZENITH...", 160, 100);
    } else {
        char buf[32];
        sprintf(buf, fast ? "FAST AIMING %.1f DEG..." : "AIMING %.1f DEG...", targetAngle);
        drawCentered(buf, 160, 100);
    }
    
    float maxSpd = fast ? MOTOR_FAST_SEEK_SPEED : MOTOR_SEEK_MAX_SPEED;

    readSettledIMUAngleDeg(4, 15);
    float initialSignedTheta = computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
    float initialError       = targetAngle - initialSignedTheta;

    if (fabsf(initialError) > 1.80f) {
        float smoothSpd = 0.0f;
        setSmoothBgSpeed(0.0f);
        motorBgMode   = 1;
        motorBgActive = true;

        unsigned long t0 = millis();
        while (millis() - t0 < 25000) {
            readIMUAngleDeg();
            float currentSignedTheta = computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
            float error = targetAngle - currentSignedTheta;
            
            // Desarma o modo contínuo aos 1,80° sem jamais inverter direção em motorBgMode=1
            if (fabsf(error) <= 1.80f || (error * initialError) <= 0.0f) {
                setSmoothBgSpeed(0.0f);
                break;
            }
            
            smoothSpd = computeDampedSeekSpeed(error, -1.0f, smoothSpd, maxSpd);
            setSmoothBgSpeed(smoothSpd);
            
            pollLaser();
            delay(20);
        }
    }
    
    motorBgActive = false;
    setSmoothBgSpeed(0.0f);

    // Ajuste fino determinístico por contagem exata de micropassos (até 2 passes após estabilização do DLPF 5 Hz)
    for (int pass = 0; pass < 2; pass++) {
        readSettledIMUAngleDeg(8, 20);
        float residualErr = targetAngle - computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
        if (fabsf(residualErr) < 0.14f || fabsf(residualErr) > 8.0f) break;
        int32_t trimSteps = (int32_t)lroundf(fabsf(residualErr) * STEPS_PER_DEG);
        if (trimSteps < 1) break;
        int8_t stepDir = (residualErr >= 0.0f) ? -1 : 1;
        portENTER_CRITICAL(&motorTimerMux);
        bgCurrentPosSteps = 0;
        bgTargetPosSteps  = stepDir * trimSteps;
        motorBgMode       = 2;
        motorBgActive     = true;
        portEXIT_CRITICAL(&motorTimerMux);
        setSmoothBgSpeed(stepDir * 18.0f);
        unsigned long tTrim = millis();
        while (bgCurrentPosSteps != bgTargetPosSteps && (millis() - tTrim < 3000)) {
            pollLaser();
            delay(5);
        }
        setSmoothBgSpeed(0.0f);
        motorBgActive = false;
    }
    readSettledIMUAngleDeg(6, 20);

    tft.setTextSize(1);
    motorEnergizedSinceMs = millis();
}

// ══════════════════════════════════════════════════════════════════
//  LEITURA DO IMU (Com Filtro de Rejeição de Choque Mecânico e Desdobramento 0°..180°+)
// ══════════════════════════════════════════════════════════════════

float readIMUAngleDeg() {
#if SIMULACAO_WOKWI
    if (zeroCalibrated && setupH > 0.1f) {
        float L_m = (float)readDistanceTimedAverageMM(500, 2, true) / 1000.0f;
        if (L_m >= setupH) {
            imuPitchDeg = acosf(setupH / L_m) * (180.0f / M_PI);
            imuRollDeg  = 0.0f;
            return imuPitchDeg;
        }
    }
    imuPitchDeg = 0.0f;
    imuRollDeg  = 0.0f;
    return 0.0f;
#else
    if (!imuOnline) {
        imuPitchDeg = 0.0f;
        imuRollDeg  = 0.0f;
        return 0.0f;
    }
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    imuAccelX = a.acceleration.x;
    imuAccelY = a.acceleration.y;
    imuAccelZ = a.acceleration.z;
    
    // Norma do vetor gravidade: atualiza continuamente sem deadlock de limiar entre 0° e 90°
    float normNow = sqrtf(imuAccelX * imuAccelX + imuAccelY * imuAccelY + imuAccelZ * imuAccelZ);
    static float normBaseline = -1.0f;
    if (normBaseline < 0.0f || forceImuFilterSync) {
        normBaseline = normNow;
    } else {
        normBaseline = (0.92f * normBaseline) + (0.08f * normNow);
    }
    imuNormG = normNow;

    // Desdobramento contínuo no sentido positivo (0° = Nadir, +90° = Horizonte, +180° = Teto/Zênite, até +270°)
    float unwrappedDeg = atan2f(-a.acceleration.z, a.acceleration.x) * (180.0f / M_PI);
    if (unwrappedDeg < -90.0f) unwrappedDeg += 360.0f;
    imuRawUnwrappedDeg = unwrappedDeg;

    float calibratedDeg = computeCalibratedSignedPitchDeg(unwrappedDeg);
    float pitchBruto = (calibratedDeg < 0.0f) ? fabsf(calibratedDeg) : calibratedDeg;
    
    float alpha = (fabsf(normNow - normBaseline) > 0.45f) ? 0.12f : 0.38f;

    static float pitchFiltrado = -1.0f;
    if (pitchFiltrado < 0.0f || forceImuFilterSync) {
        pitchFiltrado = pitchBruto;
        forceImuFilterSync = false;
    } else {
        pitchFiltrado = ((1.0f - alpha) * pitchFiltrado) + (alpha * pitchBruto);
    }
    imuPitchDeg = pitchFiltrado;

    float rollBruto = atan2f(a.acceleration.y, sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * (180.0f / M_PI);
    static float rollFiltrado = 0.0f;
    static bool rollInit = false;
    if (!rollInit) {
        rollFiltrado = rollBruto;
        rollInit = true;
    } else {
        rollFiltrado = (0.70f * rollFiltrado) + (0.30f * rollBruto);
    }
    imuRollDeg = rollFiltrado;
    
    return pitchFiltrado;
#endif
}

// ══════════════════════════════════════════════════════════════════
//  GRAVAÇÃO NO SD CARD (DATA + HORA + COLUNAS SEPARADAS POR ';')
// ══════════════════════════════════════════════════════════════════

bool initSDCard() {
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    spiSD.end();
    spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

    if (SD.begin(PIN_SD_CS, spiSD, 4000000)) {
        uint8_t cardType = SD.cardType();
        if (cardType != CARD_NONE) {
            sdOnline = true;
            uint64_t cardSizeMB = SD.cardSize() / (1024ULL * 1024ULL);
            Serial.printf("[SD HSPI] Online (CS=%d, SCK=%d, MOSI=%d, MISO=%d) | Capacidade: %llu MB\n",
                          PIN_SD_CS, PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO, cardSizeMB);
            return true;
        }
    }
    sdOnline = false;
    Serial.printf("[SD HSPI] FALHA ou Cartao Ausente (CS=%d, SCK=%d, MOSI=%d, MISO=%d)\n",
                  PIN_SD_CS, PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO);
    return false;
}

bool openSDFile() {
    if (!sdOnline) {
        initSDCard();
    }
    int ptIndex = currentRadial - 1;
    sprintf(fileName, "/PT%02d.csv", ptIndex);

    if (SD.exists(fileName)) {
        SD.remove(fileName);
    }

    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
        sdOnline = true;
        char ptLabel[48], dateStr[16], timeStr[16];
        getPointLabel(currentRadial, radialCount, ptLabel);
        getCurrentDateTimeStr(dateStr, sizeof(dateStr), timeStr, sizeof(timeStr));

        dataFile.printf("# %s - Inicio: %s %s - H=%u mm - d_pivo=%.1f mm - API 650: %s (V058)\n",
                        ptLabel, dateStr, timeStr, (unsigned int)setupH_mm, SCAN_PIVOT_OFFSET_MM, SLOPE_SHORT[bottomSlopeType]);
        dataFile.println("MP;Data;Hora;X_m;Theta_deg;L_m;Z_mm;Fallback");
        dataFile.flush();
        return true;
    }
    return false;
}

void writePointToSD(MeasPoint &pt) {
    char dateStr[16], timeStr[16];
    getCurrentDateTimeStr(dateStr, sizeof(dateStr), timeStr, sizeof(timeStr));

    if (dataFile) {
        char line[128];
        snprintf(line, sizeof(line), "%d;%s;%s;%.3f;%.2f;%.3f;%d;%u",
                 pt.mp, dateStr, timeStr, pt.x_m, pt.theta_deg, pt.l_m, (int)pt.cota_mm, (unsigned int)pt.fallback);
        dataFile.println(line);
        // Flush periódico a cada 10 pontos (executado sempre com o motor 100% parado no dwell)
        if (pt.mp % 10 == 0) {
            dataFile.flush();
        }
    }
    Serial.printf("[%s %s] MP%d | X=%.3fm | θ=%.2f° | L=%.3fm (%d mm) | Z=%d mm | FB=%u%s\n",
                  dateStr, timeStr, pt.mp, pt.x_m, pt.theta_deg, pt.l_m,
                  (int)lroundf(pt.l_m * 1000.0f), (int)pt.cota_mm, (unsigned int)pt.fallback,
                  pt.fallback == 2 ? " [OBSTRUCAO]" : (pt.fallback == 1 ? " [FALLBACK]" : ""));
}

void closeSDFile() {
    if (dataFile) {
        dataFile.flush();
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
    drawFooter("COM5: [N/1] Start  [H] Horizon  [?] Help");
}

void drawBottomMenu() {
    tft.fillScreen(C_BG);
    drawHeader();

    char strRad[32];
    sprintf(strRad, "%d linhas", radialCount);

    drawButton(bottomMenuZones[0].x, bottomMenuZones[0].y,
               bottomMenuZones[0].w, bottomMenuZones[0].h,
               C_BTN, C_BORDER, "RADIAIS", strRad);
               
    drawButton(bottomMenuZones[1].x, bottomMenuZones[1].y,
               bottomMenuZones[1].w, bottomMenuZones[1].h,
               C_BTN, C_BORDER, "PASSO", RES_LABEL[resLevel]);

    drawButton(bottomMenuZones[2].x, bottomMenuZones[2].y,
               bottomMenuZones[2].w, bottomMenuZones[2].h,
               C_BTN, C_BORDER, "PERFIL API 650", SLOPE_LABEL[bottomSlopeType]);

    drawButton(bottomMenuZones[3].x, bottomMenuZones[3].y,
               bottomMenuZones[3].w, bottomMenuZones[3].h,
               C_RED, C_BORDER, "VOLTAR", NULL);

    drawButton(bottomMenuZones[4].x, bottomMenuZones[4].y,
               bottomMenuZones[4].w, bottomMenuZones[4].h,
               C_START, C_BORDER, "INICIAR PROJETO", "Auto-busca Vertice e Scan");

    drawFooter("COM5: [N/1] Iniciar  [B] Perfil  [R] Res  [0] Voltar");
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
    drawCentered("Damped 90.0 deg lock (Anti-Bounce).", 160, 85);

    drawButton(horizonExitZone.x, horizonExitZone.y, horizonExitZone.w, horizonExitZone.h, C_DIM, C_BORDER, "BACK TO MENU [0]", NULL);
    drawFooter("COM5: [0/X] Back & Cool Motor");
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

    tft.setFont(NULL);
    tft.setTextSize(2);
    tft.setTextColor(C_YELLOW);
    drawCentered("AUTO-BUSCA DO VERTICE", 160, 68);

    tft.setTextSize(1);
    tft.setTextColor(C_WHITE);
    drawCentered("API 650: Nadir (0 deg) -> Horizonte (90 deg) -> Quina", 160, 88);
    tft.setTextColor(C_CYAN);
    char profBuf[64];
    sprintf(profBuf, "Perfil: %s | Slew Rapido Ativo", SLOPE_SHORT[bottomSlopeType]);
    drawCentered(profBuf, 160, 102);

    drawButton(setupCaptureZone.x, setupCaptureZone.y,
               setupCaptureZone.w, setupCaptureZone.h,
               C_ACCENT, C_BORDER,
               "BUSCAR VERTICE & CALIBRAR", "Mede H (0 deg) e R (90 deg) -> Mira Quina");

    drawButton(setupAbortZone.x, setupAbortZone.y,
               setupAbortZone.w, setupAbortZone.h,
               C_RED, C_WHITE, "VOLTAR AO MENU [0]", NULL);

    drawFooter("COM5: [1/N] Auto-Busca  [B] Perfil  [0] Voltar");
}

// ══════════════════════════════════════════════════════════════════
//  TELA: CAPTURA E REGULAGEM DO JOINT (Com +/- no visor TFT)
// ══════════════════════════════════════════════════════════════════

void drawJointStepButton() {
    tft.fillRoundRect(jointStepZone.x, jointStepZone.y, jointStepZone.w, jointStepZone.h, 5, C_HEADER);
    tft.drawRoundRect(jointStepZone.x, jointStepZone.y, jointStepZone.w, jointStepZone.h, 5, C_ACCENT);
    
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    drawCentered("PASSO ANG.", jointStepZone.x + jointStepZone.w / 2, jointStepZone.y + 14);

    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_ACCENT);
    char stepStr[16];
    sprintf(stepStr, "+/- %.1f deg", JOINT_STEPS[jointStepIdx]);
    drawCentered(stepStr, jointStepZone.x + jointStepZone.w / 2, jointStepZone.y + 38);
}

void updateJointTelemetry() {
    float curTheta = readIMUAngleDeg();
    float distL    = ultimaDistanciaLaser;
    float radiusR  = 0.0f;
    int32_t cotaZ  = (distL >= 0.04f) ? computeV058FloorKinematics(distL, curTheta, radiusR) : 0;

    tft.setFont(&FreeSansBold9pt7b);

    // 1. TILT (θ)
    tft.fillRect(44, 28, 110, 29, C_BG);
    tft.setTextColor(C_CYAN);
    char buf1[24];
    sprintf(buf1, "%.2f deg", curTheta);
    drawRightAligned(buf1, 154, 44);

    // 2. DIST (L)
    tft.fillRect(212, 28, 100, 29, C_BG);
    tft.setTextColor(C_WHITE);
    char buf2[24];
    if (distL >= 0.04f && distL <= 65.0f) {
        sprintf(buf2, "%.3f m", distL);
    } else {
        sprintf(buf2, "---");
    }
    drawRightAligned(buf2, 312, 44);

    // 3. RAIO (R)
    tft.fillRect(44, 63, 110, 29, C_BG);
    tft.setTextColor(C_GREEN);
    char buf3[24];
    if (distL >= 0.04f && radiusR > 0.01f) {
        sprintf(buf3, "%.3f m", radiusR);
    } else {
        sprintf(buf3, "---");
    }
    drawRightAligned(buf3, 154, 79);

    // 4. COTA (Z)
    tft.fillRect(212, 63, 100, 29, C_BG);
    uint16_t zColor = (abs(cotaZ) > 50) ? C_RED : (abs(cotaZ) > 20) ? C_YELLOW : C_GREEN;
    tft.setTextColor(zColor);
    char buf4[24];
    if (distL >= 0.04f) {
        sprintf(buf4, "%+d mm", (int)cotaZ);
    } else {
        sprintf(buf4, "---");
    }
    drawRightAligned(buf4, 312, 79);
}

void drawJointScreen() {
    tft.fillScreen(C_BG);
    drawHeader();

    // 1. Molduras de Telemetria (Cards 2x2 no topo: y = 26 a 94)
    tft.drawRoundRect(4, 26, 154, 33, 4, C_BORDER);
    tft.drawRoundRect(162, 26, 154, 33, 4, C_BORDER);
    tft.drawRoundRect(4, 61, 154, 33, 4, C_BORDER);
    tft.drawRoundRect(162, 61, 154, 33, 4, C_BORDER);

    // Labels estáticos de Telemetria
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    tft.setCursor(8, 30);   tft.print("TILT:");
    tft.setCursor(166, 30); tft.print("DIST (L):");
    tft.setCursor(8, 65);   tft.print("RAIO (R):");
    tft.setCursor(166, 65); tft.print("COTA (Z):");

    // 2. Botões de Regulagem (+ / - e Seletor de Passo) (y = 98 a 152)
    // Botão - DESCER
    tft.fillRoundRect(jointDownZone.x, jointDownZone.y, jointDownZone.w, jointDownZone.h, 5, C_BTN);
    tft.drawRoundRect(jointDownZone.x, jointDownZone.y, jointDownZone.w, jointDownZone.h, 5, C_BORDER);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered("- DESCER", jointDownZone.x + jointDownZone.w / 2, jointDownZone.y + jointDownZone.h / 2 - 8);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    drawCentered("Tilt Down", jointDownZone.x + jointDownZone.w / 2, jointDownZone.y + jointDownZone.h / 2 + 12);

    // Botão PASSO
    drawJointStepButton();

    // Botão + SUBIR
    tft.fillRoundRect(jointUpZone.x, jointUpZone.y, jointUpZone.w, jointUpZone.h, 5, C_BTN);
    tft.drawRoundRect(jointUpZone.x, jointUpZone.y, jointUpZone.w, jointUpZone.h, 5, C_BORDER);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered("+ SUBIR", jointUpZone.x + jointUpZone.w / 2, jointUpZone.y + jointUpZone.h / 2 - 8);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    drawCentered("Tilt Up", jointUpZone.x + jointUpZone.w / 2, jointUpZone.y + jointUpZone.h / 2 + 12);

    // 3. Botões Inferiores (y = 158 a 212)
    // Botão < NADIR
    tft.fillRoundRect(jointNadirZone.x, jointNadirZone.y, jointNadirZone.w, jointNadirZone.h, 5, C_HEADER);
    tft.drawRoundRect(jointNadirZone.x, jointNadirZone.y, jointNadirZone.w, jointNadirZone.h, 5, C_BORDER);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(C_DIM);
    drawCentered("< NADIR", jointNadirZone.x + jointNadirZone.w / 2, jointNadirZone.y + jointNadirZone.h / 2 - 6);
    tft.setFont(NULL);
    tft.setTextSize(1);
    drawCentered("Re-Zero", jointNadirZone.x + jointNadirZone.w / 2, jointNadirZone.y + jointNadirZone.h / 2 + 12);

    // Botão INICIAR A VARREDURA
    tft.fillRoundRect(jointStartZone.x, jointStartZone.y, jointStartZone.w, jointStartZone.h, 5, C_START);
    tft.drawRoundRect(jointStartZone.x, jointStartZone.y, jointStartZone.w, jointStartZone.h, 5, C_GREEN);
    tft.drawRoundRect(jointStartZone.x + 1, jointStartZone.y + 1, jointStartZone.w - 2, jointStartZone.h - 2, 4, C_GREEN);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered("> INICIAR VARREDURA", jointStartZone.x + jointStartZone.w / 2, jointStartZone.y + jointStartZone.h / 2 - 8);
    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(C_GREEN);
    drawCentered("Travar Joint e Varrer", jointStartZone.x + jointStartZone.w / 2, jointStartZone.y + jointStartZone.h / 2 + 12);

    drawFooter("COM5: [+] Subir [-] Descer [P] Passo [4/N] Iniciar Scan");
    updateJointTelemetry();
}

// ══════════════════════════════════════════════════════════════════
//  TELA: CHECKPOINT (entre Survey Lines — Motor Desenergizado/Frio!)
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
        tft.setTextColor(C_GREEN);
        drawCentered("Motor Coils OFF (Cool Standby at Nadir)", 160, 65);
        tft.setTextColor(C_DIM);
        drawCentered("Send [N] or [2] on COM5 for next line", 160, 77);

        if (setupRmax > 0) {
            tft.setTextColor(C_CYAN);
            char strResumo[48];
            sprintf(strResumo, "Saved: Rmax=%.2fm  %d pts  %d corr.",
                    setupRmax, scanIndex, reshots);
            drawCentered(strResumo, 160, 88);
        }

        drawButton(checkScanZone.x, checkScanZone.y,
                   checkScanZone.w, checkScanZone.h,
                   C_START, C_BORDER,
                   "NEXT SCAN POINT", "COM5: [N] or [2] Next Line");

        if (currentRadial > 1) {
            drawButton(checkUndoZone.x, checkUndoZone.y,
                       checkUndoZone.w, checkUndoZone.h,
                       C_BTN, C_BORDER, "REDO [U]", "Erase last line");
            drawButton(checkAbortHalfZone.x, checkAbortHalfZone.y,
                       checkAbortHalfZone.w, checkAbortHalfZone.h,
                       C_RED, C_WHITE, "ABORT [0]", "Exit to menu");
        } else {
            drawButton(checkAbortFullZone.x, checkAbortFullZone.y,
                       checkAbortFullZone.w, checkAbortFullZone.h,
                       C_RED, C_WHITE, "ABORT ALL [0]", "Exit to menu");
        }
        
        drawFooter("COM5: [N/2] Next Line  [U] Redo  [0] Exit");
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
                   "FINISH AND SAVE", "COM5: [N] or [0] Return");
                   
        drawFooter("All lines saved. Motor Coils OFF.");
    }
}

// ══════════════════════════════════════════════════════════════════
//  TELA: VARREDURA EM PROGRESSO
// ══════════════════════════════════════════════════════════════════

void updateScanningTelemetry() {
    tft.fillRect(40, 68, 240, 22, C_BG);
    char tel1[48];
    sprintf(tel1, "MP %d  (X = %.2f m)", lastPoint.mp, lastPoint.x_m);
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextColor(C_WHITE);
    drawCentered(tel1, 160, 80);

    tft.fillRect(30, 98, 260, 46, C_BG);
    char tel2[48];
    if (lastPoint.fallback == 2) {
        sprintf(tel2, "Z = %d mm [OBST] Th = %.2f deg", (int)lastPoint.cota_mm, lastPoint.theta_deg);
        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(C_MAGENTA); // V054: Alerta Magenta de Obstrução / Tubulação
    } else {
        sprintf(tel2, "Z = %d mm   Theta = %.2f deg", (int)lastPoint.cota_mm, lastPoint.theta_deg);
        tft.setFont(NULL);
        tft.setTextSize(1);
        tft.setTextColor(abs((int)lastPoint.cota_mm) > 50 ? C_RED :
                         abs((int)lastPoint.cota_mm) > 20 ? C_YELLOW : C_GREEN);
    }
    drawCentered(tel2, 160, 105);

    char tel3[48];
    sprintf(tel3, "L = %d mm (%.3fm) | Spd: %.1f stp/s",
            (int)lroundf(lastPoint.l_m * 1000.0f), lastPoint.l_m, activeSweepSpeedSteps);
    tft.setTextColor(C_DIM);
    drawCentered(tel3, 160, 120);

    char tel4[48];
    sprintf(tel4, "Rate: %.1f Hz | Corr: %d", measuredRateHz, reshots);
    tft.setTextColor(reshots > 0 ? C_YELLOW : C_DIM);
    drawCentered(tel4, 160, 135);

    float progress = (setupTheta > 0.5f) ?
                     (setupTheta - lastPoint.theta_deg) / setupTheta : 0.0f;
    progress = constrain(progress, 0.0f, 1.0f);
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
                   C_BTN, C_WHITE, "NO [N]", "Resume");
        drawButton(confirmYesZone.x, confirmYesZone.y, confirmYesZone.w, confirmYesZone.h,
                   C_RED, C_WHITE, "YES [0]", "Abort");
                   
        drawFooter("COM5: [N] Resume  [0/X] Confirm Abort");
    } else {
        drawButton(scanStopZone.x, scanStopZone.y,
                   scanStopZone.w, scanStopZone.h,
                   C_RED, C_WHITE, "EMERGENCY STOP [0]", "COM5: Send [0] or [X]");

        drawFooter("Scanning... COM5: [0/X] Stop");
    }
}

// ══════════════════════════════════════════════════════════════════
//  ALGORITMO DE VARREDURA V049: PONTO A PONTO COM PARADA >= 0,5s, ÂNGULO IMU ESTÁTICO E COMPENSAÇÃO ATÉ O NADIR
//  - Zona 1 (θ > 63°): menor passo (1 micropasso = 0,1125°), para >= 0,5s c/ amostra garantida
//  - Zona 2 (63° >= θ > 45°): 50 pontos exatos (acumulador contínuo de 0,36°/ponto), para >= 0,5s
//  - Zona 3 (45° >= θ >= 0°): a cada 2,0° (23 pontos) + extensão até Nadir físico real (<= 0,45°)
// ══════════════════════════════════════════════════════════════════

int computeTotalZonedScanPoints(float startTheta) {
    int count = 1; // MP0 na Quina
    int32_t totalSteps    = (int32_t)lroundf(startTheta * STEPS_PER_DEG);
    int32_t simSteps      = 0;
    float   simTheta      = startTheta;
    float   simIdealTheta = startTheta;

    while (simSteps < totalSteps && simIdealTheta > 0.001f && simTheta > 0.05f) {
        int32_t nextSteps = simSteps + 1;
        if (simTheta > SCAN_ZONE1_LIMIT_DEG) {
            nextSteps     = simSteps + 1;
            simIdealTheta = startTheta - ((float)nextSteps / STEPS_PER_DEG);
        } else if (simIdealTheta > SCAN_ZONE2_LIMIT_DEG + 0.001f) {
            simIdealTheta -= SCAN_ZONE2_STEP_DEG;
            if (simIdealTheta < SCAN_ZONE2_LIMIT_DEG) simIdealTheta = SCAN_ZONE2_LIMIT_DEG;
            nextSteps = (int32_t)lroundf((startTheta - simIdealTheta) * STEPS_PER_DEG);
            if (nextSteps <= simSteps) nextSteps = simSteps + 1;
        } else {
            simIdealTheta -= SCAN_ZONE3_STEP_DEG;
            if (simIdealTheta < 0.0f) simIdealTheta = 0.0f;
            nextSteps = (int32_t)lroundf((startTheta - simIdealTheta) * STEPS_PER_DEG);
            if (nextSteps <= simSteps) nextSteps = simSteps + 1;
        }
        if (nextSteps > totalSteps) {
            nextSteps     = totalSteps;
            simIdealTheta = 0.0f;
        }
        simSteps = nextSteps;
        simTheta = startTheta - ((float)simSteps / STEPS_PER_DEG);
        count++;
        if (count > 600) break;
    }
    return count;
}

bool scheduleNextScanPoint() {
    int32_t totalSteps     = (int32_t)lroundf(setupTheta * STEPS_PER_DEG);
    int32_t curSteps       = bgCurrentPosSteps;
    float   curTheta       = setupTheta - ((float)curSteps / STEPS_PER_DEG);
    float   signedImuTheta = computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);

    // Parada imediata caso o ângulo físico assinalado já tenha atingido o Nadir (<= 0,55°), impedindo cruzamento negativo
    if (imuOnline && curSteps > 10 && signedImuTheta <= 0.55f) {
        finishSurveyLine();
        return false;
    }

    if (curSteps >= totalSteps || scanIdealTheta <= 0.001f || curTheta <= 0.05f) {
        // Caso o stick-slip de 1 micropasso na Zona 1 tenha deixado o braço físico positivo (> +0,55°),
        // avança pontos adicionais de Zona 3 (2,0°) usando o ângulo ASSINALADO (jamais confunde negativo com positivo!)
        int32_t maxExtraSteps = totalSteps + (int32_t)lroundf(12.0f * STEPS_PER_DEG);
        if (imuOnline && signedImuTheta > 0.55f && curSteps < maxExtraSteps) {
            float stepDeg = (signedImuTheta > SCAN_ZONE3_STEP_DEG) ? SCAN_ZONE3_STEP_DEG : signedImuTheta;
            int32_t extraSteps = (int32_t)lroundf(stepDeg * STEPS_PER_DEG);
            if (extraSteps < 2) extraSteps = 2;
            int32_t nextExtra = curSteps + extraSteps;
            if (nextExtra > maxExtraSteps) nextExtra = maxExtraSteps;

            scanTargetTheta   = (signedImuTheta > stepDeg) ? (signedImuTheta - stepDeg) : 0.0f;
            currentMotorAngle = scanTargetTheta;

            portENTER_CRITICAL(&motorTimerMux);
            bgTargetPosSteps = nextExtra;
            motorBgMode      = 2;
            motorBgActive    = true;
            portEXIT_CRITICAL(&motorTimerMux);

            scanPhase = SCAN_PHASE_MOVING;
            setSmoothBgSpeed(activeSweepSpeedSteps);
            return true;
        }

        finishSurveyLine();
        return false;
    }

    int32_t nextSteps = curSteps + 1;

    if (curTheta > SCAN_ZONE1_LIMIT_DEG) {
        // Zona 1 (> 63°): avança no menor passo físico do motor (1 micropasso = 0,1125°)
        nextSteps      = curSteps + 1;
        scanIdealTheta = setupTheta - ((float)nextSteps / STEPS_PER_DEG);
    } else if (scanIdealTheta > SCAN_ZONE2_LIMIT_DEG + 0.001f) {
        // Zona 2 (63° até 45°): 50 pontos exatos no trecho de 18° (acumulador contínuo de 0,36°/ponto)
        scanIdealTheta -= SCAN_ZONE2_STEP_DEG;
        if (scanIdealTheta < SCAN_ZONE2_LIMIT_DEG) {
            scanIdealTheta = SCAN_ZONE2_LIMIT_DEG;
        }
        nextSteps = (int32_t)lroundf((setupTheta - scanIdealTheta) * STEPS_PER_DEG);
        if (nextSteps <= curSteps) nextSteps = curSteps + 1;
    } else {
        // Zona 3 (<= 45° até 0°): 1 leitura a cada 2,0° (23 pontos até o Nadir)
        scanIdealTheta -= SCAN_ZONE3_STEP_DEG;
        if (scanIdealTheta < 0.0f) scanIdealTheta = 0.0f;
        nextSteps = (int32_t)lroundf((setupTheta - scanIdealTheta) * STEPS_PER_DEG);
        if (nextSteps <= curSteps) nextSteps = curSteps + 1;
    }

    if (nextSteps > totalSteps) {
        nextSteps      = totalSteps;
        scanIdealTheta = 0.0f;
    }

    scanTargetTheta   = setupTheta - ((float)nextSteps / STEPS_PER_DEG);
    if (scanTargetTheta < 0.0f) scanTargetTheta = 0.0f;
    currentMotorAngle = scanTargetTheta;

    portENTER_CRITICAL(&motorTimerMux);
    bgTargetPosSteps = nextSteps;
    motorBgMode      = 2;
    motorBgActive    = true;
    portEXIT_CRITICAL(&motorTimerMux);

    scanPhase = SCAN_PHASE_MOVING;
    setSmoothBgSpeed((nextSteps >= curSteps) ? activeSweepSpeedSteps : -activeSweepSpeedSteps);
    return true;
}

void startScan() {
    motorBgActive    = false;
    bgStepIntervalUs = 0;
    scanIndex        = 0;
    reshots          = 0;
    zHistCount       = 0;
    lastZ            = setupH;

    bool wasAlreadyLocked = motorCurrentlyEnergized;
    enableMotor();
    delay(80); // Assentamento magnético das bobinas antes de iniciar a varredura

    // Se o motor estava desenergizado e alguém mexeu no braço após CAPTURE JOINT (> 2.5°),
    // ressincroniza TANTO o ângulo (IMU) QUANTO a distância óptica L (M01-V0.4) para jamais combinar ângulo novo com L antigo!
    float liveImuTheta = readSettledIMUAngleDeg(6, 15);
    if (!wasAlreadyLocked && fabsf(liveImuTheta - setupTheta) > 2.5f && liveImuTheta > 5.0f) {
        Serial.printf("[SCAN V054] Braço movido apos captura (%.2f° -> %.2f°). Reamostrando distancia L com motor travado...\n",
                      setupTheta, liveImuTheta);
        keepLaserAlive = true;
        laserStartContinuous(true);
        int32_t resyncL_mm = readDistanceTimedAverageMM(1000, 3, true);
        if (resyncL_mm < 40 || resyncL_mm > 60000) {
            blinkError("Joint moved! Recapture [3]");
            Serial.println("[SCAN ABORTADO] Braço movido e sem leitura optica valida na nova posicao. Capture a quina [3] novamente.");
            return;
        }
        setupTheta     = liveImuTheta;
        setupL_mm      = (uint32_t)resyncL_mm;
        setupL         = (float)setupL_mm / 1000.0f;
        setupRmax      = 0.0f;
        setupCotaQ     = computeV058FloorKinematics(setupL, setupTheta, setupRmax);
        Serial.printf("[SCAN V058] Quina ressincronizada c/ d=%.1fmm: θ=%.2f° | L=%u mm | Rmax=%.3fm | Z=%d mm\n",
                      SCAN_PIVOT_OFFSET_MM, setupTheta, (unsigned int)setupL_mm, setupRmax, (int)setupCotaQ);
    }

    scanTotalPts = computeTotalZonedScanPoints(setupTheta);

    portENTER_CRITICAL(&motorTimerMux);
    stepper.setCurrentPosition(0);
    bgCurrentPosSteps = 0;
    bgTargetPosSteps  = 0;
    bgStepIntervalUs  = 0;
    portEXIT_CRITICAL(&motorTimerMux);

    activeSweepSpeedSteps = getConstantSweepSpeedSteps();
    below45BoostApplied   = false;

    scanRunning       = true;
    keepLaserAlive    = true;
    lastScanStepMs    = millis();
    scanStartTimeMs   = millis();
    currentMotorAngle = setupTheta;
    scanTargetTheta   = setupTheta;
    scanIdealTheta    = setupTheta;
    novaLeituraLaser  = false;

    openSDFile();

    // MP0 (Quina): gravado direto no CSV e no visor TFT já com o fator de correção Kz aplicado
    zHistCount = 0;
    MeasPoint pt0;
    pt0.mp        = scanIndex++;
    pt0.x_m       = setupRmax;
    pt0.theta_deg = setupTheta;
    pt0.l_m       = setupL;
    pt0.cota_mm   = setupCotaQ;
    pt0.fallback  = 0;
    writePointToSD(pt0);
    lastPoint = pt0;

    activeScreen = SCR_SCANNING;
    drawScanningScreen();
    screenDirty = false;
    printSerialRemoteStatus();

    long totalSteps = (long)lroundf(setupTheta * STEPS_PER_DEG);
    Serial.printf("[SCAN V058] Ponto-a-Ponto + Compensacao Cinematica d_pivo=%.1f mm (TFT e CSV) iniciado: θ_ini=%.2f° (%ld passos, %d pontos previstos)\n",
                  SCAN_PIVOT_OFFSET_MM, setupTheta, totalSteps, scanTotalPts);
    Serial.println("  -> Zona 1 (> 63°): menor passo (0,1125°) + parada >= 0,5s c/ Kz aplicado");
    Serial.println("  -> Zona 2 (63° a 45°): 50 pontos exatos (0,36°/pt) + parada >= 0,5s c/ Kz aplicado");
    Serial.println("  -> Zona 3 (< 45° a 0°): a cada 2,0° + parada exata no Nadir assinalado (0,00°)");

    beep(120);

    // Rearma o stream óptico contínuo APÓS inicialização do SD, TFT e beep, logo antes do 1º passo
    laserStartContinuous(true);
    lastValidLaserMs = millis();

    // Dispara o deslocamento para o primeiro ponto discreto após MP0
    scheduleNextScanPoint();
}

void finishSurveyLine() {
    portENTER_CRITICAL(&motorTimerMux);
    motorBgActive    = false;
    bgStepIntervalUs = 0;
    portEXIT_CRITICAL(&motorTimerMux);
    scanRunning      = false;

    closeSDFile();
    laserStop();

    enableMotor();

    // Verifica o ângulo estático no fim da linha e executa ajuste fino determinístico se houver resíduo > 0,20°
    readSettledIMUAngleDeg(6, 15);
    float residualEndErr = 0.0f - computeCalibratedSignedPitchDeg(imuRawUnwrappedDeg);
    if (imuOnline && fabsf(residualEndErr) >= 0.20f && fabsf(residualEndErr) <= 8.0f) {
        int32_t trimSteps = (int32_t)lroundf(fabsf(residualEndErr) * STEPS_PER_DEG);
        if (trimSteps >= 1) {
            int8_t stepDir = (residualEndErr >= 0.0f) ? -1 : 1;
            portENTER_CRITICAL(&motorTimerMux);
            bgCurrentPosSteps = 0;
            bgTargetPosSteps  = stepDir * trimSteps;
            motorBgMode       = 2;
            motorBgActive     = true;
            portEXIT_CRITICAL(&motorTimerMux);
            setSmoothBgSpeed(stepDir * 18.0f);
            unsigned long tTrim = millis();
            while (bgCurrentPosSteps != bgTargetPosSteps && (millis() - tTrim < 3000)) {
                delay(5);
            }
            setSmoothBgSpeed(0.0f);
            motorBgActive = false;
        }
    }

    // No fim da descida o braço está no Centro (Nadir = 0,00°): mantém o motor TRAVADO/ENERGIZADO!
    bgCurrentPosSteps = 0;
    stepper.setCurrentPosition(0);
    enableMotor();
    motorEnergizedSinceMs = millis();

    // Sincroniza imediatamente o filtro do IMU no Nadir após parar
    float finalNadirDeg = readSettledIMUAngleDeg(6, 15);
    float elapsedSec    = (millis() - scanStartTimeMs) / 1000.0f;

    Serial.printf("\n[SCAN CONCLUIDO] Arquivo %s salvo (%d/%d pontos | Fallbacks=%d | Tempo=%.1f s). Motor TRAVADO no Nadir (Pitch=%.2f deg).\n",
                  fileName, scanIndex, scanTotalPts, reshots, elapsedSec, finalNadirDeg);

    currentRadial++;
    quinaCaptured = false;
    motorUnlocked = false;
    activeScreen  = SCR_CHECKPOINT;
    screenDirty   = true;

    beepSuccess();
}

// ══════════════════════════════════════════════════════════════════
//  CALIBRAÇÃO DO ZERO (Nadir — Média Temporal Inteira de 1,5 s)
// ══════════════════════════════════════════════════════════════════

void calibrateZero() {
    enableMotor();
    keepLaserAlive = true;
    laserStartContinuous(false);

    // ── PASSO 1: Busca e amostragem do Nadir (0,00°) em Slew Rápido ──
    drawFooter("Buscando Nadir 0,00 deg (Slew Rapido)...");
    seekNadirBlocking(true); // Slew rápido até o Nadir
    enableMotor();

    drawFooter("Medindo Altura H no Nadir (1.5s)...");
    Serial.println("[CALIBRACAO] Medindo altura H no Nadir por 1.5s (motor TRAVADO em 0,00 deg)...");

    int32_t L_mm = readDistanceTimedAverageMM(windowDurationMs, 4, true);
    if (L_mm < 40 || L_mm > 60000) {
        blinkError("ERRO: Sem leitura optica no Nadir!");
        return;
    }

    setupH_mm      = (uint32_t)L_mm;
    setupH         = (float)setupH_mm / 1000.0f;
    setupL_zero    = setupH;
    zeroCalibrated = true;
    scanOnline     = true;
    enableMotor();

    Serial.printf("[NADIR REAL] H = %u mm (%.3f m) | Motor travado em 0,00 deg.\n",
                  (unsigned int)setupH_mm, setupH);
    beepSuccess();

    // ── PASSO 2: Slew Rápido para o Horizonte (90,00°) e medição do Raio R no Costado ──
    drawFooter("Slew Rapido para Horizonte 90,00 deg (Costado)...");
    Serial.println("[API 650] Movendo braco rapidamente para 90,00 deg (Horizonte/Costado)...");
    seekAngleBlocking(90.0f, true); // Slew rápido a 120 passos/s

    drawFooter("Medindo Raio R no Costado (1.5s)...");
    Serial.println("[API 650] Medindo distancia horizontal R ate o costado por 1.5s...");
    int32_t R_mm = readDistanceTimedAverageMM(windowDurationMs, 4, true);
    if (R_mm < 100 || R_mm > 65000) {
        blinkError("ERRO: Sem leitura laser no Costado (90 deg)!");
        return;
    }
    float R_m = (float)R_mm / 1000.0f;
    Serial.printf("[COSTADO REAL] R medido = %d mm (%.3f m).\n", (int)R_mm, R_m);
    beepSuccess();

    // ── PASSO 3: Cálculo Trigonométrico do Vértice Costado-Fundo com Caimento API Standard 650 (V058) ──
    float slopeRatio = getActiveSlopeRatio();
    float dPivM      = SCAN_PIVOT_OFFSET_MM / 1000.0f;
    float dCornerM   = (SCAN_PIVOT_OFFSET_MM + SCAN_VERTEX_GRAZING_H_MM) / 1000.0f; // 65,0 mm (pivô 55,5 mm + ajuste fino de spot 9,5 mm c/ Horizonte 88,30°)
    float deltaZ_m   = slopeRatio * R_m;

    // V058: Projeção horizontal (R_m + d_pivo) e vertical (H + 65,0 mm - deltaZ) na junção física piso-parede
    // Com o Horizonte calibrado exatamente no nível (88,30° bruto, -25 mm a 2,15 m), mira direto na quina (~73,55°)!
    float R_axis_m   = R_m + dPivM;
    float H_axis_m   = (setupH + dCornerM) - deltaZ_m;
    if (H_axis_m < 0.05f) H_axis_m = 0.05f;

    float thetaQuinaRad = atan2f(R_axis_m, H_axis_m);
    float thetaQuinaDeg = thetaQuinaRad * (180.0f / M_PI);
    thetaQuinaDeg = constrain(thetaQuinaDeg, 15.0f, 88.5f);

    float D_axis_m  = sqrtf((R_axis_m * R_axis_m) + (H_axis_m * H_axis_m));
    float L_quina_m = D_axis_m - dPivM; // Distância óptica esperada da lente até a quina física
    if (L_quina_m < 0.05f) L_quina_m = 0.05f;
    uint32_t L_quina_mm = (uint32_t)lroundf(L_quina_m * 1000.0f);

    Serial.printf("[API 650 VERTICE V058] Perfil: %s | H=%.3fm | R=%.3fm | DeltaZ=%+dmm | ThetaQuina=%.2f deg | L_hipotenusa=%.3fm\n",
                  SLOPE_LABEL[bottomSlopeType], setupH, R_m, (int)lroundf(deltaZ_m * 1000.0f), thetaQuinaDeg, L_quina_m);

    // ── PASSO 4: Slew Rápido direto para o Vértice Físico + Amostragem Real na Quina ──
    char bufMsg[48];
    sprintf(bufMsg, "Slew Rapido para Vertice (%.2f deg)...", thetaQuinaDeg);
    drawFooter(bufMsg);
    seekAngleBlocking(thetaQuinaDeg, true); // Slew rápido direto para a quina física (~73,55°)

    // Amostra o ângulo estático real e a distância laser real na quina logo após cravar o motor
    float settledVertexTheta = readSettledIMUAngleDeg(6, 15);
    int32_t measuredVertexL  = readDistanceTimedAverageMM(900, 2, true);
    if (measuredVertexL >= 100 && measuredVertexL <= 65000) {
        L_quina_mm = (uint32_t)measuredVertexL;
        L_quina_m  = (float)L_quina_mm / 1000.0f;
    }

    setupL_mm     = L_quina_mm;
    setupL        = L_quina_m;
    setupTheta    = (imuOnline && settledVertexTheta > 5.0f) ? settledVertexTheta : thetaQuinaDeg;
    setupRmax     = R_m;
    float rCalcQ  = R_m;
    setupCotaQ    = computeV058FloorKinematics(setupL, setupTheta, rCalcQ);
    setupRmax     = rCalcQ;
    quinaCaptured = true;

    beepSuccess();

    // ── PASSO 5: Transição para a Tela de Ajuste Fino / Início do Joint ──
    activeScreen = SCR_JOINT;
    screenDirty  = true;
    keepLaserAlive = true;
    laserStartContinuous(false);
}

// ══════════════════════════════════════════════════════════════════
//  CAPTURA DA QUINA (Média Síncrona de Ângulo + Distância + Trava Braço)
// ══════════════════════════════════════════════════════════════════

void captureQuina() {
    drawFooter("Hold steady! Sampling Joint for 1.5s...");
    Serial.println("[QUINA] Coletando media de 1.5s da Juncao Piso-Parede...");

    keepLaserAlive = true;

    for (int i = 0; i < 10; i++) {
        readIMUAngleDeg();
        delay(10);
    }
    int32_t L_mm = readDistanceTimedAverageMM(windowDurationMs, 4, true);
    float theta  = readIMUAngleDeg();

    if (L_mm < 40 || L_mm > 60000) {
        laserStartContinuous(true);
        blinkError("ERROR: Sem leitura laser na quina!");
        return;
    }

    setupL_mm      = (uint32_t)L_mm;
    float L_m      = (float)setupL_mm / 1000.0f;
    float thetaRad = theta * (M_PI / 180.0f);

    stepper.setCurrentPosition(0);
    enableMotor();
    setupTheta = theta;
    setupL     = L_m;

    setupRmax    = 0.0f;
    setupCotaQ   = computeV058FloorKinematics(L_m, theta, setupRmax);

    if (setupRmax < 0.02f || setupRmax > 60.0f) {
        blinkError("ERROR: Radius out of bounds (0.02~60m)!");
        return;
    }

    quinaCaptured     = true;
    scanOnline        = true;
    currentMotorAngle = theta;

    Serial.printf("[QUINA REAL FILTRADA c/ Kz] θ=%.2f° | L=%u mm (%.3fm) | Rmax=%.3fm | Z=%d mm (Envie 4 ou N p/ escanear)\n",
                  setupTheta, (unsigned int)setupL_mm, setupL, setupRmax, (int)setupCotaQ);

    beepSuccess();
    screenDirty = true;
}

// ══════════════════════════════════════════════════════════════════
//  ATALHOS E NAVEGAÇÃO REMOTA PELA PORTA COM5 (SEM TOCAR NO APARELHO)
// ══════════════════════════════════════════════════════════════════

void printSerialRemoteStatus() {
    readIMUAngleDeg();
    Serial.println("------------------------------------------------------------------");
    switch (activeScreen) {
        case SCR_MAIN_MENU:
            Serial.printf("[TELA: MENU PRINCIPAL] Pitch=%.1f° | Motor:%s\n",
                          imuPitchDeg, motorCurrentlyEnergized ? "ENERGIZADO" : "FRIO/OFF");
            Serial.println("  Atalhos COM5 -> [N] ou [1]: Iniciar Fundo | [H]: Nivel 90° | [?]: Ajuda");
            break;
        case SCR_BOTTOM_MENU:
            Serial.printf("[TELA: CONFIG FUNDO] Linhas=%d | Res=%s | API 650: %s\n",
                          radialCount, RES_LABEL[resLevel], SLOPE_SHORT[bottomSlopeType]);
            Serial.println("  Atalhos COM5 -> [N] ou [1]: Avancar | [B]: Mudar Perfil API 650 | [R]: Mudar Res | [0]: Voltar");
            break;
        case SCR_SETUP:
            Serial.printf("[TELA: AUTO-BUSCA VERTICE API 650] Pitch=%.1f° | Perfil: %s | Motor:%s\n",
                          imuPitchDeg, SLOPE_SHORT[bottomSlopeType], motorCurrentlyEnergized ? "ENERGIZADO" : "FRIO/OFF");
            Serial.println("  Atalhos COM5 -> [1] ou [N]: Auto-Busca Nadir + Horizonte + Vertice | [B]: Perfil | [0]: Voltar");
            break;
        case SCR_JOINT: {
            float rJoint = 0.0f;
            int32_t zJoint = computeV058FloorKinematics(ultimaDistanciaLaser, imuPitchDeg, rJoint);
            Serial.printf("[TELA: AJUSTE DO JOINT] Pitch=%.2f° | Passo=%.1f° | Dist=%.3fm | Raio=%.3fm | Cota=%dmm\n",
                          imuPitchDeg, JOINT_STEPS[jointStepIdx], ultimaDistanciaLaser, rJoint, (int)zJoint);
            Serial.println("  Atalhos COM5 -> [+] Subir | [-] Descer | [P] Passo | [4/N] Iniciar Varredura | [0] Voltar Nadir");
            break;
        }
        case SCR_SCANNING:
            Serial.printf("[VARREDURA EM CURSO] Radial %d/%d | Vel=%.1f passos/s | Envie [0] ou [X] p/ Parar\n",
                          currentRadial, radialCount, activeSweepSpeedSteps);
            break;
        case SCR_CHECKPOINT:
            Serial.printf("[CHECKPOINT RADIAL %d/%d] Pitch=%.2f° | Motor:%s no Nadir.\n",
                          currentRadial, radialCount, imuPitchDeg, motorCurrentlyEnergized ? "ENERGIZADO" : "FRIO/OFF");
            Serial.println("  Atalhos COM5 -> [N] ou [2]: Proxima Radial | [U]: Refazer Anterior | [F]: Esfriar Motor | [0]: Sair");
            break;
        case SCR_HORIZON:
            Serial.printf("[NIVEL HORIZONTE 90°] Pitch=%.2f° (%s) | Envie [0] ou [X] p/ Sair e Desenergizar\n",
                          imuPitchDeg, horizonLocked ? "TRAVADO SUAVE" : "AJUSTANDO");
            break;
    }
    Serial.println("------------------------------------------------------------------");
}

void triggerNextStepFromSerial() {
    switch (activeScreen) {
        case SCR_MAIN_MENU:
            currentRadial  = 1;
            zeroCalibrated = false;
            motorUnlocked  = false;
            quinaCaptured  = false;
            enableMotor();
            activeScreen   = SCR_SETUP;
            screenDirty    = true;
            break;

        case SCR_BOTTOM_MENU:
            currentRadial  = 1;
            zeroCalibrated = false;
            motorUnlocked  = false;
            quinaCaptured  = false;
            enableMotor();
            activeScreen   = SCR_SETUP;
            screenDirty    = true;
            break;

        case SCR_SETUP:
            calibrateZero();
            break;

        case SCR_JOINT:
            captureQuina();
            if (quinaCaptured) {
                startScan();
            }
            break;

        case SCR_CHECKPOINT:
            if (currentRadial <= radialCount) {
                quinaCaptured = false;
                keepLaserAlive = true;
                laserStartContinuous(false);
                activeScreen = SCR_JOINT;
                screenDirty  = true;
                float aimAngle = (setupTheta >= 20.0f && setupTheta <= 85.0f) ? setupTheta : 70.0f;
                seekAngleBlocking(aimAngle, true); // Slew Rápido direto para o vértice da próxima radial
                keepLaserAlive = true;
                laserStartContinuous(false);
            } else {
                motorBgActive = false;
                laserStop();
                disableMotor();
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty  = true;
            }
            break;

        case SCR_SCANNING:
            if (confirmingStop) {
                confirmingStop = false;
                scanRunning = true;
                keepLaserAlive = true;
                laserStartContinuous(false);
                drawScanningScreen();
                screenDirty = false;
                enableMotor();
                scheduleNextScanPoint();
            }
            break;

        case SCR_HORIZON:
            motorBgActive = false;
            disableMotor();
            laserStop();
            activeScreen = SCR_MAIN_MENU;
            screenDirty  = true;
            break;
    }
}

// ══════════════════════════════════════════════════════════════════
//  TOQUE E NAVEGAÇÃO
// ══════════════════════════════════════════════════════════════════

bool getTouch(int16_t &tx, int16_t &ty) {
    static unsigned long lastTouchTime = 0;
    if (millis() - lastTouchTime < 300) return false;

    if (!ts.touched()) return false;
    
    TS_Point p = ts.getPoint();
    if (p.z < 100) return false;
    
    tx = constrain(map(p.y, 250, 3800, 0, 320), 0, 319);
    ty = constrain(map(p.x, 250, 3800, 240, 0), 0, 239);

    beep(30);
    lastTouchTime = millis();
    return true;
}

bool insideZone(int16_t tx, int16_t ty, Zone &z) {
    return (tx >= z.x && tx < z.x + z.w && ty >= z.y && ty < z.y + z.h);
}

void drawTouchCursor(int16_t x, int16_t y) {
    tft.drawLine(x - 8, y, x + 8, y, C_RED);
    tft.drawLine(x, y - 8, x, y + 8, C_RED);
    tft.drawCircle(x, y, 4, C_RED);
}

void handleTouch() {
    int16_t tx, ty;
    bool isTouched = getTouch(tx, ty);
    static bool lastTouchedState = false;
    if (!isTouched) {
        lastTouchedState = false;
        return;
    }
    if (lastTouchedState) return;
    lastTouchedState = true;

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
            activeScreen  = SCR_HORIZON;
            horizonLocked = false;
            lastHorizonStepMs = millis();
            enableMotor();
            setSmoothBgSpeed(0.0f);
            motorBgMode   = 1;
            motorBgActive = true;
            laserPointerOn();
            screenDirty = true;
        }
        break;
        
    case SCR_HORIZON:
        // V052: So sai se o toque for explicitamente no botao BACK e apos 600 ms da entrada na tela
        if ((millis() - lastHorizonStepMs > 600) && insideZone(tx, ty, horizonExitZone)) {
            motorBgActive = false;
            setSmoothBgSpeed(0.0f);
            disableMotor();
            laserStop();
            activeScreen = SCR_MAIN_MENU;
            screenDirty = true;
        }
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
            bottomSlopeType = (BottomSlopeType)((bottomSlopeType + 1) % 3);
            screenDirty = true;
        }
        else if (insideZone(tx, ty, bottomMenuZones[3])) {
            activeScreen = SCR_MAIN_MENU;
            screenDirty = true;
        }
        else if (insideZone(tx, ty, bottomMenuZones[4])) {
            currentRadial  = 1;
            zeroCalibrated = false;
            motorUnlocked  = false;
            quinaCaptured  = false;
            enableMotor();
            activeScreen   = SCR_SETUP;
            screenDirty    = true;
        }
        break;

    case SCR_SETUP:
        if (insideZone(tx, ty, setupCaptureZone)) {
            calibrateZero();
        }
        else if (insideZone(tx, ty, setupAbortZone)) {
            motorBgActive = false;
            laserStop();
            disableMotor();
            zeroCalibrated = false;
            motorUnlocked  = false;
            quinaCaptured  = false;
            activeScreen = SCR_BOTTOM_MENU;
            screenDirty = true;
        }
        break;

    case SCR_JOINT:
        if (insideZone(tx, ty, jointDownZone)) {
            nudgeAngleBySteps(-JOINT_STEPS[jointStepIdx]);
            updateJointTelemetry();
        }
        else if (insideZone(tx, ty, jointStepZone)) {
            jointStepIdx = (jointStepIdx + 1) % NUM_JOINT_STEPS;
            drawJointStepButton();
        }
        else if (insideZone(tx, ty, jointUpZone)) {
            nudgeAngleBySteps(+JOINT_STEPS[jointStepIdx]);
            updateJointTelemetry();
        }
        else if (insideZone(tx, ty, jointNadirZone)) {
            zeroCalibrated = false;
            activeScreen   = SCR_SETUP;
            screenDirty    = true;
            seekNadirBlocking();
        }
        else if (insideZone(tx, ty, jointStartZone)) {
            captureQuina();
            if (quinaCaptured) {
                startScan();
            }
        }
        break;

    case SCR_CHECKPOINT:
        if (currentRadial <= radialCount) {
            if (insideZone(tx, ty, checkScanZone)) {
                quinaCaptured = false;
                keepLaserAlive = true;
                laserStartContinuous(false);
                
                activeScreen = SCR_JOINT;
                screenDirty = true;

                float aimAngle = (setupTheta >= 20.0f && setupTheta <= 85.0f) ? setupTheta : 70.0f;
                seekAngleBlocking(aimAngle, true); // Slew Rápido direto para o vértice da próxima radial
                keepLaserAlive = true;
                laserStartContinuous(false);
            }
            else if (currentRadial > 1 && insideZone(tx, ty, checkUndoZone)) {
                currentRadial--;
                sprintf(fileName, "/PT%02d.csv", currentRadial - 1);
                SD.remove(fileName);
                screenDirty = true;
            }
            else if (currentRadial > 1 && insideZone(tx, ty, checkAbortHalfZone)) {
                motorBgActive = false;
                laserStop();
                disableMotor();
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
            else if (currentRadial == 1 && insideZone(tx, ty, checkAbortFullZone)) {
                motorBgActive = false;
                laserStop();
                disableMotor();
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
        } else {
            if (insideZone(tx, ty, checkFinishZone)) {
                motorBgActive = false;
                laserStop();
                disableMotor();
                isCalibrated = false;
                activeScreen = SCR_MAIN_MENU;
                screenDirty = true;
            }
        }
        break;

    case SCR_SCANNING:
        if (!confirmingStop) {
            if (insideZone(tx, ty, scanStopZone)) {
                motorBgActive = false;
                setSmoothBgSpeed(0.0f);
                disableMotor();
                scanRunning = false;
                laserStop();
                confirmingStop = true;
                screenDirty = true;
            }
        } else {
            if (insideZone(tx, ty, confirmYesZone)) {
                confirmingStop = false;
                closeSDFile();
                laserStop();
                disableMotor();
                activeScreen = SCR_CHECKPOINT;
                screenDirty = true;
            } else if (insideZone(tx, ty, confirmNoZone)) {
                confirmingStop = false;
                scanRunning = true;
                keepLaserAlive = true;
                laserStartContinuous(false);
                drawScanningScreen();
                screenDirty = false;
                enableMotor();
                scheduleNextScanPoint();
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
    Serial.println("GeoTank LaserScan v3.0 (Firmware V058 - Auto-Busca Exata na Quina + Compensacao Diedro/Rasante Z) - Inicializando...");

    initInternalRTCFromCompileTime();
    char dStr[16], tStr[16];
    getCurrentDateTimeStr(dStr, sizeof(dStr), tStr, sizeof(tStr));
    Serial.printf("[RTC INTERNO] Data/Hora ativa: %s %s\n", dStr, tStr);

    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_ENA, HIGH);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    
    stepper.setEnablePin(PIN_ENA);
    stepper.setPinsInverted(true, false, true); // V052: Invertido DIR para TMC2209
    stepper.setMinPulseWidth(MOTOR_PULSE_US);
    stepper.setMaxSpeed(MOTOR_RETURN_SPEED);
    stepper.setAcceleration(MOTOR_RETURN_ACCEL);
    disableMotor();

    const esp_timer_create_args_t timerArgs = {
        .callback = &onMotorStepTimer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MotorStep100us"
    };
    esp_timer_create(&timerArgs, &motorStepTimer);
    esp_timer_start_periodic(motorStepTimer, 100); // 100 us (10 kHz) reduz pela metade o overhead de task switch mantendo < 0,36% de erro de fase!

#if SIMULACAO_WOKWI
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
#else
    initLaserUART();
#endif

    Wire.begin(21, 22);

    tft.begin();
    tft.setRotation(1);
    ts.begin();
    ts.setRotation(1);

    calibPrefs.begin("geotank", false);
    imuRawNadir0Deg    = calibPrefs.getFloat("rawNadir0",  6.96f);
    imuRawHorizon90Deg = calibPrefs.getFloat("rawHoriz90", 88.30f);
    if (fabsf(imuRawHorizon90Deg - 88.96f) < 0.05f) {
        imuRawHorizon90Deg = 88.30f;
        calibPrefs.putFloat("rawHoriz90", 88.30f);
    }
    imuRawZenith180Deg = calibPrefs.getFloat("rawZen180",  176.76f);
    imuPitchOffsetDeg  = calibPrefs.getFloat("pitchTrim",  0.00f);
    calibPrefs.end();

    if (mpu.begin()) {
        imuOnline = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
        Serial.printf("IMU: Online (DLPF = 5 Hz | Curva 3-Pts: 0°=%.2f° 90°=%.2f° 180°=%.2f° | Trim=%+.2f°)\n",
                      imuRawNadir0Deg, imuRawHorizon90Deg, imuRawZenith180Deg, imuPitchOffsetDeg);
    } else {
        Serial.println("IMU: FAILED");
    }

    initSDCard();

    scanOnline = probeLaserHardware();
    laserStop();
    Serial.printf("SCAN M01: %s\n", scanOnline ? "Online (Hardware Real)" : "Aguardando Alvo");

    Serial.println("Initialization complete. Envie '?' na COM5 para ver todos os atalhos remotos.");
}

bool blinkState = false;
unsigned long lastBlinkMs = 0;

void loop() {
    if (isErrorShowing && millis() > errorMessageUntil) {
        isErrorShowing = false;
        screenDirty = true;
    }

    // Proteção Térmica Automática: limite de 3 min (180s) em retenção estática no Setup/Joint/Checkpoint para proteger bobinas e DRV8825
    if (motorCurrentlyEnergized && !motorBgActive && !scanRunning && activeScreen != SCR_HORIZON) {
        bool keepLockedInSetup = (activeScreen == SCR_SETUP) || (activeScreen == SCR_JOINT) || (activeScreen == SCR_CHECKPOINT);
        unsigned long activeTimeoutMs = (keepLockedInSetup || imuPitchDeg > 135.0f) ? MOTOR_HOLD_THERMAL_TIMEOUT_MS : MOTOR_IDLE_THERMAL_TIMEOUT_MS;
        if (millis() - motorEnergizedSinceMs > activeTimeoutMs) {
            disableMotor();
            Serial.println("[PROTECAO TERMICA] Motor desenergizado automaticamente para evitar aquecimento.");
            if (activeScreen == SCR_SETUP || activeScreen == SCR_JOINT || activeScreen == SCR_CHECKPOINT) screenDirty = true;
        }
    }

    if (screenDirty) {
        isErrorShowing = false;
        switch (activeScreen) {
            case SCR_MAIN_MENU:   drawMainMenu();         break;
            case SCR_BOTTOM_MENU: drawBottomMenu();       break;
            case SCR_HORIZON:     drawHorizonScreen();    break;
            case SCR_SETUP:       drawSetupScreen();      break;
            case SCR_JOINT:       drawJointScreen();      break;
            case SCR_CHECKPOINT:  drawCheckpointScreen(); break;
            case SCR_SCANNING:    drawScanningScreen();   break;
        }
        screenDirty = false;
        printSerialRemoteStatus();
    }

    if (activeScreen == SCR_JOINT && !screenDirty) {
        static unsigned long lastJointTelemMs = 0;
        if (millis() - lastJointTelemMs >= 160) {
            lastJointTelemMs = millis();
            updateJointTelemetry();
        }
    }

    // Nunca consulta o SPI do Touch enquanto o motor está em movimento na varredura!
    if (!(activeScreen == SCR_SCANNING && scanRunning && scanPhase == SCAN_PHASE_MOVING)) {
        static unsigned long lastScanTouchPollMs = 0;
        if (!(activeScreen == SCR_SCANNING && scanRunning) || (millis() - lastScanTouchPollMs >= 250)) {
            lastScanTouchPollMs = millis();
            handleTouch();
        }
    }

    pollLaser();

    if (Serial.available()) {
        char rawCh = (char)Serial.read();
        if (rawCh == '\r' || rawCh == '\n' || rawCh == ' ') {
            // Ignora quebras de linha e espaços soltos
        } else {
            char cmd = toupper((unsigned char)rawCh);

            // ── ATALHOS DE OPERAÇÃO SEM TOCAR NO APARELHO (COM5) ──
            if (cmd == 'N') {
                Serial.println("[COM5 REMOTO] Avancando etapa (NEXT)...");
                triggerNextStepFromSerial();
            }
            else if (cmd == '1') {
                Serial.println("[COM5 REMOTO] Calibrar Altura no Nadir (H)...");
                activeScreen   = SCR_SETUP;
                zeroCalibrated = false;
                calibrateZero();
            }
            else if (cmd == '2') {
                Serial.println("[COM5 REMOTO] Avancar p/ Tela de Joint...");
                activeScreen   = SCR_JOINT;
                screenDirty    = true;
                float aimAngle = (setupTheta >= 20.0f && setupTheta <= 85.0f) ? setupTheta : 70.0f;
                seekAngleBlocking(aimAngle, true); // Slew Rápido direto para o vértice da próxima radial
                keepLaserAlive = true;
                laserStartContinuous(false);
            }
            else if (cmd == '3' || cmd == 'J') {
                if (activeScreen == SCR_JOINT) {
                    captureQuina();
                }
            }
            else if (cmd == '4' || cmd == 'S') {
                if (activeScreen == SCR_JOINT) {
                    captureQuina();
                    if (quinaCaptured) startScan();
                } else if (activeScreen == SCR_SETUP) {
                    calibrateZero();
                }
            }
            else if (cmd == 'P') {
                jointStepIdx = (jointStepIdx + 1) % NUM_JOINT_STEPS;
                Serial.printf("[PASSO ANGULAR] Passo fixado em +/- %.1f deg\n", JOINT_STEPS[jointStepIdx]);
                if (activeScreen == SCR_JOINT) drawJointStepButton();
            }
            else if (cmd == 'B') {
                bottomSlopeType = (BottomSlopeType)((bottomSlopeType + 1) % 3);
                Serial.printf("[PERFIL API 650] Perfil de Caimento: %s\n", SLOPE_LABEL[bottomSlopeType]);
                if (activeScreen == SCR_BOTTOM_MENU || activeScreen == SCR_SETUP || activeScreen == SCR_JOINT) {
                    screenDirty = true;
                }
            }
            else if (cmd == 'G') {
                String arg = Serial.readStringUntil('\n');
                arg.trim();
                if (arg.length() > 0) {
                    float targetDeg = arg.toFloat();
                    if (targetDeg >= 0.0f && targetDeg <= 185.0f) {
                        Serial.printf("[COM5 AUTO-MIRA] Movendo braco pelo motor no sentido positivo para %.2f graus via IMU...\n", targetDeg);
                        activeScreen  = SCR_SETUP;
                        motorUnlocked = true;
                        quinaCaptured = false;
                        keepLaserAlive = true;
                        laserStartContinuous(false);
                        seekAngleBlocking(targetDeg);
                        float curDeg = readIMUAngleDeg();
                        Serial.printf("[COM5 AUTO-MIRA] Posicionado! IMU Calibrado = %.2f deg (Bruto = %.2f deg | Curva 0°=%.2f° 90°=%.2f° 180°=%.2f°)\n",
                                      curDeg, imuRawUnwrappedDeg, imuRawNadir0Deg, imuRawHorizon90Deg, imuRawZenith180Deg);
                        Serial.println("[PRUMO LASER] Use '+ <graus>' ou '- <graus>' para ajuste fino e 'O' para gravar na Flash!");
                        screenDirty = true;
                    }
                } else if (zeroCalibrated && quinaCaptured) {
                    startScan();
                }
            }
            else if (cmd == '+' || cmd == '=') {
                String arg = Serial.readStringUntil('\n');
                arg.trim();
                float stepDeg = (arg.length() > 0) ? fabsf(arg.toFloat()) : JOINT_STEPS[jointStepIdx];
                if (stepDeg < 0.05f) stepDeg = 0.05f;
                if (stepDeg > 45.0f) stepDeg = 45.0f;
                keepLaserAlive = true;
                laserStartContinuous(false);
                nudgeAngleBySteps(+stepDeg);
                if (activeScreen == SCR_JOINT) updateJointTelemetry();
                else screenDirty = true;
            }
            else if (cmd == '-') {
                String arg = Serial.readStringUntil('\n');
                arg.trim();
                float stepDeg = (arg.length() > 0) ? fabsf(arg.toFloat()) : JOINT_STEPS[jointStepIdx];
                if (stepDeg < 0.05f) stepDeg = 0.05f;
                if (stepDeg > 45.0f) stepDeg = 45.0f;
                keepLaserAlive = true;
                laserStartContinuous(false);
                nudgeAngleBySteps(-stepDeg);
                if (activeScreen == SCR_JOINT) updateJointTelemetry();
                else screenDirty = true;
            }
            else if (cmd == 'O') {
                String arg = Serial.readStringUntil('\n');
                arg.trim();
                readIMUAngleDeg();
                calibPrefs.begin("geotank", false);
                if (arg.length() > 0) {
                    imuPitchOffsetDeg = arg.toFloat();
                    calibPrefs.putFloat("pitchTrim", imuPitchOffsetDeg);
                } else if (imuRawUnwrappedDeg > 135.0f) {
                    imuRawZenith180Deg = imuRawUnwrappedDeg;
                    imuPitchOffsetDeg  = 0.00f;
                    calibPrefs.putFloat("rawZen180", imuRawZenith180Deg);
                    calibPrefs.putFloat("pitchTrim", 0.00f);
                } else if (imuRawUnwrappedDeg >= 45.0f) {
                    imuRawHorizon90Deg = imuRawUnwrappedDeg;
                    imuPitchOffsetDeg  = 0.00f;
                    calibPrefs.putFloat("rawHoriz90", imuRawHorizon90Deg);
                    calibPrefs.putFloat("pitchTrim",  0.00f);
                } else {
                    imuRawNadir0Deg   = imuRawUnwrappedDeg;
                    imuPitchOffsetDeg = 0.00f;
                    calibPrefs.putFloat("rawNadir0", imuRawNadir0Deg);
                    calibPrefs.putFloat("pitchTrim", 0.00f);
                }
                calibPrefs.end();
                float calibratedNow = readSettledIMUAngleDeg(6, 15);
                Serial.printf("[CALIBRACAO LASER/MPU SALVA] Curva: 0°=%.2f° | 90°=%.2f° | 180°=%.2f° (Trim=%+.2f°) -> Angulo atual = %.2f deg (Bruto = %.2f deg)\n",
                              imuRawNadir0Deg, imuRawHorizon90Deg, imuRawZenith180Deg, imuPitchOffsetDeg, calibratedNow, imuRawUnwrappedDeg);
                screenDirty = true;
            }
            else if (cmd == '0' || cmd == 'X') {
                Serial.println("[COM5 EMERGENCIA/ABORTAR] Parando tudo e DESENERGIZANDO motor (0 A)...");
                motorBgActive  = false;
                scanRunning    = false;
                confirmingStop = false;
                horizonLocked  = false;
                closeSDFile();
                laserStop();
                disableMotor();
                activeScreen = SCR_MAIN_MENU;
                screenDirty  = true;
            }
            else if (cmd == 'F') {
                disableMotor();
                Serial.println("[COM5 MOTOR FRIO] Bobinas do motor 100% DESENERGIZADAS (PIN_ENA = HIGH).");
                if (activeScreen == SCR_SETUP || activeScreen == SCR_CHECKPOINT) screenDirty = true;
            }
            else if (cmd == 'P') {
                String arg = Serial.readStringUntil('\n');
                arg.trim();
                float spd = arg.toFloat();
                if (spd >= 1.5f && spd <= 120.0f) {
                    customSweepSpeedSteps = spd;
                    activeSweepSpeedSteps = spd;
                    if (scanRunning && scanPhase == SCAN_PHASE_MOVING) {
                        setSmoothBgSpeed((bgTargetPosSteps >= bgCurrentPosSteps) ? activeSweepSpeedSteps : -activeSweepSpeedSteps);
                    }
                    Serial.printf("[VELOCIDADE VARREDURA] Fixada em %.1f passos/s (%.2f graus/s)\n",
                                  customSweepSpeedSteps, customSweepSpeedSteps / STEPS_PER_DEG);
                    if (activeScreen == SCR_SETUP) screenDirty = true;
                } else {
                    customSweepSpeedSteps = 0.0f;
                    Serial.printf("[VELOCIDADE VARREDURA] Modo padrao da resolucao: %.1f passos/s | Para fixar envie Ex: P 35\n",
                                  getConstantSweepSpeedSteps());
                }
            }
            else if (cmd == 'R') {
                resLevel = (resLevel + 1) % 3;
                customSweepSpeedSteps = 0.0f;
                Serial.printf("[RESOLUCAO] Alterada para: %s (%.1f passos/s)\n",
                              RES_LABEL[resLevel], getConstantSweepSpeedSteps());
                screenDirty = true;
            }
            else if (cmd == 'Y') {
                useQuickLaserMode = !useQuickLaserMode;
                Serial.printf("[MODO LASER M01] Alterado para: %s\n",
                              useQuickLaserMode ? "CONTINUO RAPIDO (0x22 Quick)" : "CONTINUO PADRAO (0x21 Cont)");
                if (keepLaserAlive || scanRunning) {
                    laserStartContinuous(true);
                }
            }
            else if (cmd == 'U') {
                if (activeScreen == SCR_CHECKPOINT && currentRadial > 1) {
                    currentRadial--;
                    sprintf(fileName, "/PT%02d.csv", currentRadial - 1);
                    SD.remove(fileName);
                    Serial.printf("[REDO] Radial %d apagada. Pronta para repetir.\n", currentRadial);
                    screenDirty = true;
                }
            }
            else if (cmd == 'H') {
                Serial.println("[COM5 REMOTO] Modo Nivel Horizonte (90 deg) com Trava Amortecida...");
                activeScreen      = SCR_HORIZON;
                horizonLocked     = false;
                lastHorizonStepMs = millis();
                enableMotor();
                setSmoothBgSpeed(0.0f);
                motorBgMode   = 1;
                motorBgActive = true;
                laserPointerOn();
                screenDirty = true;
            }
            // ── COMANDOS DE DIAGNÓSTICO E SENSORES ──
            else if (cmd == 'L') {
                laserPointerOn();
            } else if (cmd == 'K' || cmd == 'S') {
                laserStop();
            } else if (cmd == 'M' || cmd == 'Q') {
                Serial.printf("[TESTE] Coletando media temporal inteira (%.1fs)...\n", windowDurationMs / 1000.0f);
                int32_t mm = readDistanceTimedAverageMM(windowDurationMs, 4, false);
                Serial.printf("[TESTE] Resultado Filtrado: %d mm (%.3f m)\n", (int)mm, mm > 0 ? mm / 1000.0f : 0.0f);
            } else if (cmd == 'C') {
                laserStartContinuous(true);
            } else if (cmd == 'W') {
                if (windowDurationMs == 1000)      windowDurationMs = 1500;
                else if (windowDurationMs == 1500) windowDurationMs = 2000;
                else if (windowDurationMs == 2000) windowDurationMs = 3000;
                else                               windowDurationMs = 1000;
                resetTimeWindow();
                Serial.printf("[JANELA TEMPORAL] Ajustada para %.1f s (%lu ms)\n", windowDurationMs / 1000.0f, windowDurationMs);
            } else if (cmd == 'V') {
                probeLaserHardware();
            } else if (cmd == 'D') {
                initSDCard();
                drawHeader();
            } else if (cmd == 'T') {
                String rest = Serial.readStringUntil('\n');
                rest.trim();
                if (rest.length() >= 16 && setInternalRTCFromSerial(rest.c_str())) {
                    char dNow[16], tNow[16];
                    getCurrentDateTimeStr(dNow, sizeof(dNow), tNow, sizeof(tNow));
                    Serial.printf("[RTC INTERNO] Relogio atualizado com sucesso: %s %s\n", dNow, tNow);
                } else {
                    char dNow[16], tNow[16];
                    getCurrentDateTimeStr(dNow, sizeof(dNow), tNow, sizeof(tNow));
                    Serial.printf("[RTC INTERNO] Atual: %s %s | Para ajustar envie: T DD/MM/YYYY HH:MM:SS\n", dNow, tNow);
                }
            } else if (cmd == 'I') {
                readIMUAngleDeg();
                char azName[32];
                getPointLabel(currentRadial, radialCount, azName);
                Serial.printf("[IMU] PITCH: %.2f deg | ROLL: %+.2f deg | Accel(m/s2): X:%+.2f Y:%+.2f Z:%+.2f | ||g||:%.2f | Motor:%s\n",
                              imuPitchDeg, imuRollDeg, imuAccelX, imuAccelY, imuAccelZ, imuNormG,
                              motorCurrentlyEnergized ? "ENERGIZADO" : "FRIO/OFF");
            } else if (cmd == '?') {
                Serial.println("\n================ CONTROLE REMOTO COM5 — GEOTANK V052 ================");
                Serial.println("  [N]        -> AVANCAR ETAPA (clica no botao principal da tela atual)");
                Serial.println("  [1]        -> Etapa 1: Ir ao Nadir (0°) sem hunting e CALIBRAR ALTURA (H)");
                Serial.println("  [2]        -> Etapa 2: SOLTAR MOTOR + Ligar Laser p/ mirar c/ a mao");
                Serial.println("  [G <grau>] -> AUTO-MIRA PELO MOTOR sem tocar! (Ex: G 70 vai p/ 70°)");
                Serial.println("  [+] / [-]  -> Ajuste fino do angulo pelo motor (+2° / -2°)");
                Serial.println("  [3] ou [J] -> Etapa 3: CAPTURAR QUINA (1.5s filtrado c/ Kz)");
                Serial.println("  [4]        -> Etapa 4: INICIAR VARREDURA (Kz=+6.63mm/m no Visor TFT e CSV)");
                Serial.println("  [0] ou [X] -> PARAR TUDO / ABORTAR e desenergizar motor");
                Serial.println("  [F]        -> ESFRIAR MOTOR (desliga corrente das bobinas na hora)");
                Serial.println("  [P <vel>]  -> Ajustar velocidade de deslocamento em passos/s");
                Serial.println("  [R]        -> Alternar resolucao (250mm / 100mm / 50mm)");
                Serial.println("  [Y]        -> Alternar Modo Laser M01: Continuo Padrao (0x21) / Rapido (0x22)");
                Serial.println("  [H]        -> Nivel Horizonte (90°) com trava amortecida");
                Serial.println("  [M] / [C] / [S] / [L] / [I] / [D] / [T DD/MM/YYYY HH:MM:SS]");
                Serial.println("=====================================================================\n");
                printSerialRemoteStatus();
            }
        }
    }

    // ── NÍVEL HORIZONTE 90° COM APROXIMAÇÃO ACIMA DE 14 PASSOS/S + TRAVA ANTI-PULO ──
    if (activeScreen == SCR_HORIZON) {
        static unsigned long lastHorImuMs = 0;
        static float currentTheta = 0.0f;
        static float error = 0.0f;
        static float smoothHorSpd = 0.0f;

        if (millis() - lastHorImuMs >= 25) {
            lastHorImuMs = millis();
            currentTheta = readIMUAngleDeg();
            error = 90.0f - currentTheta;
            float absErr = fabsf(error);

            if (horizonLocked) {
                setSmoothBgSpeed(0.0f);
                smoothHorSpd = 0.0f;
                if (absErr > 0.85f) {
                    horizonLocked = false;
                }
            } else {
                if (absErr <= 0.30f) {
                    setSmoothBgSpeed(0.0f);
                    smoothHorSpd = 0.0f;
                    if (millis() - lastHorizonStepMs >= 260) {
                        horizonLocked = true;
                        drawFooter("LOCKED at 90.0 deg | COM5: [0] Exit");
                        Serial.printf("[HORIZONTE TRAVADO] Estabilizado em %.2f deg sem oscilacao.\n", currentTheta);
                    }
                } else {
                    smoothHorSpd = computeDampedSeekSpeed(error, -1.0f, smoothHorSpd);
                    setSmoothBgSpeed(smoothHorSpd);
                    lastHorizonStepMs = millis();
                }
            }
        }

        static unsigned long lastAngMs = 0;
        if (millis() - lastAngMs > 350) {
            lastAngMs = millis();
            tft.fillRect(75, 115, 170, 30, C_BG);
            tft.setFont(NULL);
            tft.setTextSize(3);
            tft.setTextColor(horizonLocked || fabsf(error) <= 0.32f ? C_GREEN : C_YELLOW);
            char angStr[16];
            sprintf(angStr, "%.1f deg", currentTheta);
            drawCentered(angStr, 160, 130);
            tft.setTextSize(1);
        }
    }

    // ── MÁQUINA DE ESTADOS DE VARREDURA PONTO A PONTO COM ÂNGULO IMU ESTÁTICO + CONSTANTE Kz NO TFT E CSV (V050) ──
    if (activeScreen == SCR_SCANNING && scanRunning && !confirmingStop) {
        if (scanPhase == SCAN_PHASE_MOVING) {
            // Fase 1: Aguarda o motor atingir exatamente o passo do ponto-alvo
            if (bgCurrentPosSteps == bgTargetPosSteps) {
                setSmoothBgSpeed(0.0f);
                ptSampleSumMM    = 0;
                ptSampleCount    = 0;
                for (int i = 0; i < PT_MAX_SAMPLES; i++) ptSamples[i] = 0; // V053: limpa buffer local
                ptRetrySent      = false;
                ptImuSumDeg      = 0.0f;
                ptImuCount       = 0;
                novaLeituraLaser = false; // Descarta pacote integrado durante o deslocamento
                scanDwellStartMs = millis();
                scanPhase        = SCAN_PHASE_DWELL;
            }
        } else {
            // Fase 2: Motor 100% PARADO no ponto por no mínimo 0,5 s (500 ms) coletando amostras ópticas e ângulo IMU estático
            unsigned long dwellElapsedMs = millis() - scanDwellStartMs;
            static unsigned long lastDwellImuMs = 0;

            if (dwellElapsedMs < 160UL) {
                // Nos primeiros 160 ms após parar, sincroniza o filtro IIR enquanto o DLPF de 5 Hz assenta
                if (millis() - lastDwellImuMs >= 75) {
                    lastDwellImuMs     = millis();
                    forceImuFilterSync = true;
                    readIMUAngleDeg();
                }
            } else if (millis() - lastDwellImuMs >= 55) {
                // Após 160 ms de parada estática, acumula a média do ângulo físico real medido pelo IMU calibrado
                lastDwellImuMs = millis();
                ptImuSumDeg += readIMUAngleDeg();
                ptImuCount++;
            }

            if (novaLeituraLaser) {
                novaLeituraLaser = false;
                float L_m = ultimaDistanciaLaser;
                float maxPlausibleL = (setupL > 0.35f) ? (setupL * 1.45f + 0.30f) : 60.0f;
                if (L_m >= 0.04f && L_m <= maxPlausibleL) {
                    if (ptSampleCount < PT_MAX_SAMPLES) {
                        ptSamples[ptSampleCount] = ultimaBrutaMM; // V053: armazena amostras individuais
                    }
                    ptSampleSumMM += ultimaBrutaMM;
                    ptSampleCount++;
                }
            }

            // V053: Critério de amostragem inteligente anti-outlier no Dwell:
            // - Se já tem >= 2 amostras congruentes (|s0 - s1| <= 6 mm) e passou SCAN_DWELL_MS (650 ms), conclui rápido.
            // - Se as primeiras divergirem (> 6 mm), aguarda 3 amostras para aplicar filtro de MEDIANA pura.
            // - Se atingir SCAN_MAX_DWELL_MS (1800 ms), conclui com as amostras disponíveis.
            bool dwellReady = false;
            if (ptSampleCount >= 3) {
                dwellReady = true;
            } else if (ptSampleCount >= 2 && dwellElapsedMs >= SCAN_DWELL_MS) {
                if (abs((long)ptSamples[0] - (long)ptSamples[1]) <= 6) {
                    dwellReady = true;
                }
            } else if (dwellElapsedMs >= SCAN_MAX_DWELL_MS && ptSampleCount >= 1) {
                dwellReady = true;
            }

            if (dwellReady) {
                float settledImuTheta = (ptImuCount > 0) ? (ptImuSumDeg / (float)ptImuCount) : readIMUAngleDeg();
                float thetaDeg        = (imuOnline && ptImuCount > 0) ? settledImuTheta : scanTargetTheta;
                currentMotorAngle     = thetaDeg;

                float maxPlausibleL  = (setupL > 0.35f) ? (setupL * 1.45f + 0.30f) : 60.0f;
                uint32_t avgMM       = 0;
                uint8_t usedFallback = 0;
                unsigned long sampleAgeMs = millis() - lastValidLaserMs;

                if (ptSampleCount >= 3) {
                    // V053: Mediana pura entre as amostras coletadas no dwell (elimina picos espúrios de +22 mm ou -6 mm)
                    int n = min(ptSampleCount, (int)PT_MAX_SAMPLES);
                    uint32_t s[PT_MAX_SAMPLES];
                    for (int i = 0; i < n; i++) s[i] = ptSamples[i];
                    for (int i = 0; i < n - 1; i++) {
                        for (int j = i + 1; j < n; j++) {
                            if (s[j] < s[i]) { uint32_t t = s[i]; s[i] = s[j]; s[j] = t; }
                        }
                    }
                    avgMM        = s[n / 2];
                    usedFallback = 0;
                } else if (ptSampleCount == 2) {
                    avgMM        = (ptSamples[0] + ptSamples[1] + 1) / 2;
                    usedFallback = 0;
                } else if (ptSampleCount == 1) {
                    avgMM        = ptSamples[0];
                    usedFallback = 0;
                } else if (ultimaBrutaMM >= 40 && sampleAgeMs <= SCAN_FALLBACK_MAX_AGE_MS) {
                    // Fallback auditado: só aceita se a última leitura tiver idade <= 2400 ms e marca Fallback=1 na planilha CSV
                    float lastL = (float)ultimaBrutaMM / 1000.0f;
                    if (lastL <= maxPlausibleL) {
                        avgMM        = ultimaBrutaMM;
                        usedFallback = 1;
                        reshots++;
                    }
                } else {
                    Serial.printf("[SCAN ALERTA] Sem leitura optica valida em θ=%.2f° (idade=%lu ms > %lu ms). Ponto omitido p/ nao falsificar CSV.\n",
                                  thetaDeg, sampleAgeMs, SCAN_FALLBACK_MAX_AGE_MS);
                }

                if (avgMM > 0) {
                    float L_m      = (float)avgMM / 1000.0f;
                    float rReal    = 0.0f;
                    // V058: Modelo cinemático d_pivo=55.5mm + supressão de multicaminho diedro na quina e deriva rasante (> 58°)
                    int32_t rawCotaMM  = computeV058FloorKinematics(L_m, thetaDeg, rReal);
                    int32_t filtCotaMM = filterDynamicCotaMM(rawCotaMM);

                    // V054 (API 653): Detecção em tempo real de Obstruções Físicas (Tubulações, Serpentinas, Colunas)
                    // Se o salto vertical for brusco (|dZ| >= 35 mm em dX < 0.25 m) ou elevação sustentada sobre tubulação:
                    if (scanIndex > 1 && usedFallback == 0) {
                        float dX_m  = fabsf(rReal - lastPoint.x_m);
                        float dZ_mm = fabsf((float)rawCotaMM - (float)lastPoint.cota_mm);
                        if ((dX_m < 0.25f && dZ_mm >= 35.0f) || (lastPoint.fallback == 2 && abs(rawCotaMM) >= 40)) {
                            usedFallback = 2; // Flag 2: Ponto de Obstrução / Tubulação (excluído do recalque no Analyser)
                        }
                    }

                    MeasPoint pt;
                    pt.mp        = scanIndex++;
                    pt.x_m       = rReal;
                    pt.theta_deg = thetaDeg;
                    pt.l_m       = L_m;
                    pt.cota_mm   = filtCotaMM;
                    pt.fallback  = usedFallback;

                    // Grava no SD (c/ coluna Z_mm já corrigida por Kz) e atualiza o visor TFT em cada ponto com o motor parado
                    writePointToSD(pt);
                    lastPoint = pt;
                    updateScanningTelemetry();
                }

                // Segue para o próximo ponto desejado
                scheduleNextScanPoint();
            }
        }
    }

    // Durante SCR_SCANNING, não bloqueia o barramento I2C/SPI com rodapé
    if (!(activeScreen == SCR_SCANNING && scanRunning)) {
        static unsigned long lastFooterAngMs = 0;
        if (millis() - lastFooterAngMs > 200) {
            lastFooterAngMs = millis();
            readIMUAngleDeg();
            drawFooterTelemetryLine();
        }
    }
}
