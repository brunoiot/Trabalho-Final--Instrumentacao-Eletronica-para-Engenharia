#include <Wire.h>
#include <Preferences.h>

// Pinos
const uint8_t MPU_ADDR = 0x68;      // AD0 -> GND
const int PINO_SDA = 21;
const int PINO_SCL = 22;

// Registradores do MPU-6050
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

// Fundo de escala ±2 g -> sensibilidade de 16384 LSB/g.
// Para inclinação só atua a gravidade (|a| <= 1 g), então ±2 g dá a
// melhor resolução (1/16384 ≈ 61 µg por LSB).
const float LSB_POR_G = 16384.0f;

// Aquisição
const uint32_t PERIODO_US       = 10000;  // 100 Hz
const int      AMOSTRAS_PADRAO  = 200;    // roteiro pede no mínimo 100
const uint32_t TEMPO_ACOMODACAO = 1500;   // ms de espera antes de gravar

// Calibração
float ganho[3]  = {1.0f, 1.0f, 1.0f};
float offset[3] = {0.0f, 0.0f, 0.0f};
int   modoComp  = 2;                      // 0 nenhum, 1 offset, 2 offset+ganho
bool  streaming = false;

Preferences prefs;
const char* NOMES_EIXO = "XYZ";

//  Funções de baixo nível do MPU-6050
void escreveRegistro(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission(true);
}

uint8_t leRegistro(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Lê os 3 eixos em g (sem compensação). Retorna false se a leitura falhar.
bool leAcelerometro(float a[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;

  for (int i = 0; i < 3; i++) {
    int16_t bruto = (int16_t)((Wire.read() << 8) | Wire.read());  // complemento de 2
    a[i] = bruto / LSB_POR_G;
  }
  return true;
}

// Aplica a compensação conforme o modo selecionado.
void compensa(const float a[3], float c[3]) {
  for (int i = 0; i < 3; i++) {
    if (modoComp == 0)      c[i] = a[i];
    else if (modoComp == 1) c[i] = a[i] - offset[i];
    else                    c[i] = (a[i] - offset[i]) / ganho[i];
  }
}

void configuraMPU() {
  escreveRegistro(REG_PWR_MGMT_1, 0x00);   // acorda o sensor (limpa SLEEP)
  delay(100);
  escreveRegistro(REG_PWR_MGMT_1, 0x01);   // clock pelo PLL do giroscópio X (mais estável)
  escreveRegistro(REG_CONFIG, 0x03);       // DLPF ≈ 44 Hz: reduz ruído de alta frequência
  escreveRegistro(REG_SMPLRT_DIV, 9);      // 1 kHz / (1 + 9) = 100 Hz
  escreveRegistro(REG_ACCEL_CONFIG, 0x00); // AFS_SEL = 0 -> ±2 g (explícito)
  delay(100);
}

//  Parâmetros na memória não volátil (NVS)
void carregaParametros() {
  prefs.begin("calib_mpu", true);
  for (int i = 0; i < 3; i++) {
    ganho[i]  = prefs.getFloat((String("g") + i).c_str(), 1.0f);
    offset[i] = prefs.getFloat((String("o") + i).c_str(), 0.0f);
  }
  modoComp = prefs.getInt("modo", 2);
  prefs.end();
}

void salvaParametros() {
  prefs.begin("calib_mpu", false);
  for (int i = 0; i < 3; i++) {
    prefs.putFloat((String("g") + i).c_str(), ganho[i]);
    prefs.putFloat((String("o") + i).c_str(), offset[i]);
  }
  prefs.putInt("modo", modoComp);
  prefs.end();
}

void mostraParametros() {
  Serial.println(F("# ---------------- Parâmetros de calibração ----------------"));
  Serial.println(F("# Eixo |    Ganho    |  Offset [g]"));
  for (int i = 0; i < 3; i++) {
    Serial.printf("#  %c   |  %9.6f  |  %+9.6f\n", NOMES_EIXO[i], ganho[i], offset[i]);
  }
  Serial.printf("# Modo de compensação: %d (%s)\n", modoComp,
                modoComp == 0 ? "nenhum" : modoComp == 1 ? "offset" : "offset + ganho");
}

//  Aquisição de n amostras a taxa fixa (100 Hz)
//  Se 'imprime' = true, envia cada amostra (formato para o Python).
//  Em 'soma' acumula as leituras sem compensação (para as médias).
int adquire(int n, bool imprime, double soma[3]) {
  if (soma) soma[0] = soma[1] = soma[2] = 0.0;
  int validas = 0;
  uint32_t t = micros();

  for (int k = 0; k < n; k++) {
    while ((uint32_t)(micros() - t) < PERIODO_US) { /* espera o próximo instante */ }
    t += PERIODO_US;

    float a[3], c[3];
    if (!leAcelerometro(a)) continue;   // descarta leitura com falha no I2C
    compensa(a, c);
    validas++;

    if (soma) for (int i = 0; i < 3; i++) soma[i] += a[i];
    if (imprime) {
      Serial.printf("D,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", a[0], a[1], a[2], c[0], c[1], c[2]);
    }
  }
  return validas;
}

//  Rotina de calibração (6 posições)
void esperaEnter() {
  while (Serial.available()) Serial.read();          // limpa o que sobrou
  while (!Serial.available()) delay(10);             // espera a confirmação
  delay(50);
  while (Serial.available()) Serial.read();
}

void rotinaCalibracao(int n) {
  // Posições: eixo e sinal da gravidade lido pelo eixo.
  // "+X para cima" significa o eixo X apontando para cima (leitura ≈ +1 g).
  const int   eixoPos[6]  = {0, 0, 1, 1, 2, 2};
  const int   sinalPos[6] = {+1, -1, +1, -1, +1, -1};
  double media[6][3];

  Serial.println(F("\n# ===== CALIBRAÇÃO ESTÁTICA DO ACELERÔMETRO (6 posições) ====="));
  Serial.printf("# %d amostras por posição a 100 Hz. Use superfície nivelada.\n", n);

  for (int p = 0; p < 6; p++) {
    bool ok = false;
    while (!ok) {
      Serial.printf("\n# Posição %d/6: eixo %c%c PARA CIMA (leitura esperada %+d g).\n",
                    p + 1, sinalPos[p] > 0 ? '+' : '-', NOMES_EIXO[eixoPos[p]], sinalPos[p]);
      Serial.println(F("# Posicione, deixe parado e envie ENTER..."));
      esperaEnter();

      delay(TEMPO_ACOMODACAO);                       // deixa a montagem parar de vibrar
      double soma[3];
      int validas = adquire(n, false, soma);
      if (validas < n / 2) { Serial.println(F("# ERRO: falhas de leitura I2C. Repetindo.")); continue; }

      for (int i = 0; i < 3; i++) media[p][i] = soma[i] / validas;
      Serial.printf("# Médias: X=%+.5f  Y=%+.5f  Z=%+.5f g  (%d amostras)\n",
                    media[p][0], media[p][1], media[p][2], validas);

      // Verificação simples de orientação: o eixo esperado deve ler perto de ±1 g
      double v = media[p][eixoPos[p]] * sinalPos[p];
      if (v < 0.8) {
        Serial.println(F("# AVISO: o eixo esperado não está alinhado à gravidade. Repetindo esta posição."));
      } else {
        ok = true;
      }
    }
  }

  // Sistema de 2 equações por eixo
  for (int i = 0; i < 3; i++) {
    double Amais  = media[2 * i][i];       // eixo i para cima   -> +1 g
    double Amenos = media[2 * i + 1][i];   // eixo i para baixo  -> -1 g
    ganho[i]  = (Amais - Amenos) / 2.0;
    offset[i] = (Amais + Amenos) / 2.0;
    Serial.printf("# Eixo %c: Ā(+1g)=%+.5f  Ā(-1g)=%+.5f  ->  Ganho=%.6f  Offset=%+.6f g\n",
                  NOMES_EIXO[i], Amais, Amenos, ganho[i], offset[i]);
  }
  salvaParametros();
  Serial.println(F("# Parâmetros gravados na memória (NVS)."));
  mostraParametros();
}

//  Interpretação de comandos
void trataComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd.charAt(0));
  String arg = cmd.substring(1);
  arg.trim();

  switch (c) {
    case 'C': {
      int n = arg.length() ? arg.toInt() : AMOSTRAS_PADRAO;
      rotinaCalibracao(max(n, 100));
      break;
    }
    case 'A': {
      int n = arg.length() ? arg.toInt() : AMOSTRAS_PADRAO;
      if (n < 1) n = AMOSTRAS_PADRAO;
      adquire(n, true, nullptr);
      Serial.println("END");
      break;
    }
    case 'P': {
      float v[6];
      char buf[160];
      arg.toCharArray(buf, sizeof(buf));
      char* p = buf;
      int lidos = 0;
      for (; lidos < 6; lidos++) {
        char* fim;
        v[lidos] = strtof(p, &fim);
        if (fim == p) break;
        p = fim;
      }
      if (lidos == 6 && v[0] > 0.5f && v[1] > 0.5f && v[2] > 0.5f) {
        for (int i = 0; i < 3; i++) { ganho[i] = v[i]; offset[i] = v[i + 3]; }
        salvaParametros();
        Serial.println("OK P");
        mostraParametros();
      } else {
        Serial.println("ERRO P: use 'P gx gy gz ox oy oz' (ganhos > 0.5)");
      }
      break;
    }
    case 'M': {
      int m = arg.toInt();
      if (m >= 0 && m <= 2) { modoComp = m; salvaParametros(); Serial.printf("OK M %d\n", m); }
      else Serial.println("ERRO M: use M 0, M 1 ou M 2");
      break;
    }
    case 'S':
      streaming = !streaming;
      Serial.printf("# Envio contínuo %s\n", streaming ? "LIGADO" : "DESLIGADO");
      break;
    case 'R':
      for (int i = 0; i < 3; i++) { ganho[i] = 1.0f; offset[i] = 0.0f; }
      salvaParametros();
      Serial.println("OK R");
      mostraParametros();
      break;
    case '?':
      mostraParametros();
      break;
    default:
      Serial.println("# Comandos: C, A <n>, P gx gy gz ox oy oz, M <0|1|2>, S, R, ?");
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PINO_SDA, PINO_SCL);
  Wire.setClock(400000);              

  uint8_t who = leRegistro(REG_WHO_AM_I);
  Serial.printf("\n# MPU-6050 WHO_AM_I = 0x%02X %s\n", who,
                who == 0x68 ? "(OK)" : "(inesperado: confira as ligações ou se o módulo é clone)");

  configuraMPU();
  carregaParametros();
  mostraParametros();
  Serial.println(F("# Pronto. Envie 'C' para calibrar pelo Monitor Serial, ou use o script Python."));
}

void loop() {
  if (Serial.available()) {
    trataComando(Serial.readStringUntil('\n'));
  }

  if (streaming) {
    static uint32_t ultimo = 0;
    if (millis() - ultimo >= 50) {          // 20 Hz para visualização
      ultimo = millis();
      float a[3], c[3];
      if (leAcelerometro(a)) {
        compensa(a, c);
        Serial.printf("Lido: X=%+.4f Y=%+.4f Z=%+.4f g | Comp: X=%+.4f Y=%+.4f Z=%+.4f g\n",
                      a[0], a[1], a[2], c[0], c[1], c[2]);
      }
    }
  }
}
