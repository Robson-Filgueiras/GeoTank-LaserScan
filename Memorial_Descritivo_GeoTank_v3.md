# Memorial Descritivo — GeoTank LaserScan v3.0

## 1. Objeto

O GeoTank LaserScan v3.0 é um sistema robótico automatizado de perfilometria a laser para tanques industriais de armazenamento. O equipamento foi desenvolvido para atender aos requisitos de integridade estrutural da norma API 653 (*Tank Inspection, Repair, Alteration, and Reconstruction*), substituindo o método tradicional de levantamento topográfico manual — que exige dois inspetores operando com nível a laser rotativo e trena — por um processo motorizado autônomo controlado por um único operador.

O sistema realiza varreduras radiais (centrípetas ou centrífugas) de forma automatizada, medindo continuamente a distância e o ângulo de incidência de um feixe laser de precisão. Os dados brutos são gravados em armazenamento local (cartão SD) e geram uma malha de pontos tridimensional que permite modelar superfícies internas do tanque e identificar anomalias estruturais com resolução espacial configurável.

---

## 2. Aplicações pela Norma API 653

O princípio mecânico do GeoTank — um distanciômetro a laser montado em eixo motorizado rotativo com leitura angular por IMU — atende diretamente a seis requisitos de inspeção normativa e, com adaptação de procedimento, a dois requisitos adicionais.

### 2.1 Levantamento Topográfico do Fundo (Settlement Measurements)

> **Referência:** API 653, Seção B.2.1

A aplicação primária do equipamento. O GeoTank é posicionado no centro do tanque (ou em estações de referência conhecidas) e executa varreduras radiais do piso. Cada survey line coleta centenas de pontos de elevação entre o centro e a junção chapa–costado, gerando o perfil de recalque do fundo com adensamento de pontos muito superior ao método manual.

A cota topográfica de cada ponto é calculada pelo firmware através da decomposição trigonométrica:

$$Z = H - L \cdot \cos(\theta)$$

Onde $H$ é a altura calibrada do tripé ao piso (medida no nadir), $L$ é a distância laser e $\theta$ é o ângulo em relação à vertical. Valores negativos indicam recalque (afundamento).

O número mínimo de survey lines é 8 (conforme recomendação da Figura B.1 da API 653), configurável na IHM até 36 posições.

### 2.2 Recalque de Borda (Edge Settlement)

> **Referência:** API 653, Seção B.2.3 e B.3.4

O recalque de borda ocorre quando o costado deforma a chapa de fundo na junção piso–parede. O GeoTank detecta essa anomalia naturalmente: os últimos pontos de cada survey line (próximos à quina) registram variações abruptas de cota que caracterizam o *breakover area* descrito na Figura B.6.

O pós-processamento dos dados permite calcular o gradiente de elevação nos 300 mm finais de cada radial e comparar com os limites $B_{ew}$ da Figura B.11 da norma. Quando o recalque medido ultrapassa 75% de $B_{ew}$, o relatório sinaliza a necessidade de exame visual e ensaio por partículas magnéticas ou líquido penetrante nas soldas da região.

### 2.3 Recalque do Costado (Shell Settlement)

> **Referência:** API 653, Seção B.2.2

O levantamento de recalque do costado exige a medição de elevações ao redor da circunferência do tanque. O GeoTank executa essa medição posicionando o feixe laser na horizontal (modo HORIZON LEVEL, $\theta = 90°$) e registrando a elevação em pontos equidistantes ao longo do perímetro.

A norma classifica três componentes de recalque do costado:

- **Recalque uniforme** (B.2.2.1) — detectado pela média geral das elevações.
- **Inclinação de corpo rígido** (B.2.2.2) — representada por onda cossenoidal/senoidal na plotagem polar das elevações. O *tilt* causa aumento da tensão circunferencial (*hoop stress*) e pode travar selos periféricos de tetos flutuantes.
- **Recalque fora-de-plano** (B.2.2.4, B.3.2) — deformações localizadas que não seguem padrão planar. A magnitude absoluta do máximo recalque é comparada aos limites permissíveis de B.3.2.1 ou B.3.2.2.

### 2.4 Verticalidade (Plumbness)

> **Referência:** API 653, Seção 10.5.2

O desvio de verticalidade do topo do costado em relação à base não pode exceder 1/100 da altura total do tanque, com máximo de 125 mm. O mesmo critério se aplica a colunas de teto fixo.

O GeoTank mede a verticalidade comparando as distâncias horizontais laser–costado em duas alturas distintas: na base (próximo à solda chapa–fundo) e no topo do costado. A diferença entre os raios medidos nas duas elevações corresponde diretamente ao desvio de prumo.

### 2.5 Circularidade (Roundness)

> **Referência:** API 653, Seção 10.5.3 e Tabela 10.2

Os raios medidos a 300 mm acima da solda chapa–fundo não podem exceder as tolerâncias da Tabela 10.2. Acima dessa cota, as tolerâncias admissíveis são triplicadas.

O GeoTank realiza essa medição posicionando o eixo do laser na horizontal e executando leituras de distância radial em múltiplos azimutes. Com o equipamento posicionado no centro geométrico do tanque, cada leitura de distância horizontal corresponde a um raio. A comparação entre os raios medidos e o raio nominal do tanque gera o perfil de circularidade e identifica seções ovalizadas.

### 2.6 Recalque do Fundo Próximo ao Costado (Bottom Settlement Near Shell)

> **Referência:** API 653, Seção B.2.4

O equipamento identifica essa condição ao analisar o gradiente de cotas nos pontos finais das survey lines (região de transição entre fundo e costado). A equação de B.3.3 pode ser aplicada sobre os dados coletados para avaliar a severidade do recalque nessa zona de transição.

---

## 3. Aplicações com Adequação de Procedimento

Os dois requisitos abaixo utilizam o mesmo hardware do GeoTank, porém exigem adaptação do procedimento de campo e/ou do ferramental de apoio. A interpretação das seções 10.5.4 e 10.5.5 da API 653, combinada com a API 650, fornece os critérios de aceitação.

### 3.1 Mossamento Vertical — Peaking

> **Referência:** API 653, Seção 10.5.4

A norma define que, utilizando um gabarito horizontal (*sweep board*) de 900 mm conformado ao raio externo verdadeiro do tanque, o mossamento (peaking) não deve exceder 13 mm.

O GeoTank pode executar a varredura vertical do costado em alta resolução angular, gerando um perfil de distância que, comparado ao arco teórico do raio nominal, quantifica o desvio localizado em cada junta soldada vertical. A resolução do sensor a laser (tipicamente ±1 mm) permite detectar variações dentro da faixa normativa.

### 3.2 Abaulamento Circunferencial — Banding

> **Referência:** API 653, Seção 10.5.5

Com um gabarito vertical de 900 mm, o abaulamento (banding) não pode exceder 25 mm.

O mesmo princípio de varredura vertical do peaking se aplica. O perfil de distância ao longo da altura do costado, comparado à reta teórica vertical, identifica deformações circunferenciais em juntas horizontais entre virolas.

---

## 4. Arquitetura de Hardware

### 4.1 Unidade Central de Processamento

Microcontrolador ESP32 (dual-core Xtensa LX6, 240 MHz), responsável pelo firmware de aquisição, controle do motor de passo, leitura dos sensores e interface com o operador.

### 4.2 Subsistema de Medição

| Componente | Função | Interface |
| :--- | :--- | :--- |
| Distanciômetro a Laser | Medição de distância (L) | UART (Serial2) |
| IMU MPU6050 | Medição do ângulo de incidência (θ) | I2C (SDA/SCL) |

### 4.3 Subsistema de Acionamento

| Componente | Função | Interface |
| :--- | :--- | :--- |
| Motor de Passo NEMA 17 | Rotação do eixo do laser (elevação) | Trem de pulsos (STEP/DIR) |
| Driver A4988 | Acionamento com microstepping 1/16 | GPIO digital |

### 4.4 Subsistema de IHM e Armazenamento

| Componente | Função | Interface |
| :--- | :--- | :--- |
| Display TFT ILI9341 (2.4") | Interface gráfica com o operador | SPI |
| Touchscreen Capacitivo FT6206 | Entrada tátil de comandos | I2C |
| Módulo Micro SD | Gravação dos arquivos CSV de campo | SPI (CS dedicado) |

### 4.5 Topologia de Alimentação

Fonte primária: Bateria recarregável de 24 V DC.

| Barramento | Tensão | Componentes Alimentados |
| :--- | :--- | :--- |
| Potência (direto) | 24 V | Driver A4988 (pino VMOT) |
| Step-Down 1 | 5 V | ESP32 (pino 5V), Sensor HC-SR04 (simulação) |
| Step-Down 2 | 3,3 V | Display TFT, SD Card, MPU6050, A4988 (pino VDD) |

O pino 3V3 do ESP32 permanece isolado. Os barramentos de GND convergem em um único ponto de aterramento em estrela (*Star Grounding*).

---

## 5. Firmware — Fluxo Operacional

### 5.1 Modo BOTTOM SCAN (Implementado)

```
MAIN MENU → BOTTOM MENU → SETUP (Calibrar H + Capturar Quina) → SCANNING → CHECKPOINT → [próxima survey line] → FINISH
```

O operador configura o número de survey lines (mínimo 8) e a resolução de passo (50 mm, 100 mm ou 250 mm). Para cada survey line:

1. O motor posiciona o laser no nadir (θ = 0°) e mede a altura H do tripé ao piso.
2. O motor é liberado para que o operador direcione manualmente o feixe até a junção chapa–costado (quina). O ângulo e a distância são registrados.
3. A varredura automática inicia da quina em direção ao centro (centrípeta), decrementando o raio-alvo pelo passo configurado.
4. Cada ponto passa por malha fechada de correção angular (tolerância ±3 mm) para garantir que o feixe incida na posição radial correta.
5. Os dados são gravados em arquivo CSV no cartão SD.

### 5.2 Modo SHELL SCAN (Em Desenvolvimento)

Destinado às medições de verticalidade, circularidade e recalque do costado. O eixo de varredura será reconfigurado para executar perfis verticais do costado e leituras horizontais em múltiplos azimutes.

### 5.3 Modo HORIZON LEVEL (Implementado)

Trava o motor na posição horizontal (θ = 90°) para alinhamento de referência e verificação visual do nível do equipamento.

---

## 6. Vantagens Operacionais

O método tradicional de levantamento topográfico em tanques de armazenamento exige dois inspetores operando simultaneamente: um posiciona a mira (trena ou régua graduada) nos pontos de medição enquanto o outro registra a leitura do nível a laser rotativo. Esse processo demanda horas de trabalho contínuo dentro de espaço confinado, produz um número reduzido de pontos de amostragem e está sujeito a erros de leitura visual e anotação manual.

O GeoTank automatiza integralmente o ciclo de aquisição. Um único inspetor posiciona o tripé, executa a calibração pela IHM e aciona a varredura. O motor realiza o giro autônomo enquanto o sensor coleta centenas de pontos por survey line sem intervenção humana. A exposição do operador ao ambiente confinado é reduzida drasticamente.

O custo de fabricação do protótipo utiliza componentes de prototipagem eletrônica acessíveis, representando uma fração do investimento necessário para aquisição de estações totais ou scanners topográficos comerciais.

---

## 7. Referências Normativas

- API 653 — *Tank Inspection, Repair, Alteration, and Reconstruction* (Seções 10.5.2, 10.5.3, 10.5.4, 10.5.5, Apêndice B)
- API 650 — *Welded Tanks for Oil Storage* (Seção 7.5.2, Seção H.4.1.1)
