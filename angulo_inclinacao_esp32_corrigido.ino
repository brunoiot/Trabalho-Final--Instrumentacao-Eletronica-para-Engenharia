#include <Wire.h>
#include <Preferences.h>
#include <math.h>

//  Pinos e endereços
const uint8_t MPU_ADDR = 0x68;
const int PINO_SDA = 21;
const int PINO_SCL = 22;

const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const float LSB_POR_G = 16384.0f;          // fundo de escala ±2 g

// Parâmetros da calibração estática X, Y, Z
const float GANHO[3]  = {0.9991594f, 0.996337075f, 1.008936375f};
const float OFFSET[3] = {0.020235850f, 0.004450975f, 0.078392575f};   // [g]

// Excentricidade do transferidor
const float EXCENTRICIDADE_MM = 19.0f;     // deslocamento do centro da escala
const float RAIO_ESCALA_MM    = 60.0f;     // raio do transferidor (Maped 12 cm)

// Parâmetros
const uint32_t PERIODO_US        = 10000;  // 100 Hz
const int      N_MEDIA_MOVEL     = 20;     // janela da exibição: 20 amostras = 0,2 s
const uint32_t INTERVALO_TELA_MS = 100;    // atualiza a tela 10 vezes por segundo
const int      N_PONTO           = 200;    // amostras por ponto do ensaio (2 s)
const uint32_t ACOMODACAO_MS     = 1000;   // espera antes de medir o ponto
const int      MAX_PONTOS        = 80;

//  Configuração
int  eixoBraco = 1;        // 0 = X, 1 = Y
int  sinal     = -1;       // +1 ou -1
bool modoPlotter = false;
bool pausado     = false;
bool descendo    = false;  // sentido da varredura registrado em cada ponto

Preferences prefs;

//  Média móvel
float bufComp[N_MEDIA_MOVEL][3];
float bufBruto[N_MEDIA_MOVEL][3];
int   idxBuf = 0, cheioBuf = 0;

// Tabela do ensaio
struct Ponto {
  float leituraT;      // leitura do transferidor [°]
  float ref;           // theta_ref corrigido pela excentricidade
  float medido;        // média do ângulo compensado
  float desvio;        // desvio padrão do ângulo compensado
  float semComp;       // média do ângulo sem compensação
  float foraPlano;     // inclinação fora do plano de rotação (diagnóstico)
  bool  descida;
};
Ponto pontos[MAX_PONTOS];
int nPontos = 0;

//  MPU-6050
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

bool leAcelerometro(float a[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 3; i++) {
    int16_t bruto = (int16_t)((Wire.read() << 8) | Wire.read());
    a[i] = bruto / LSB_POR_G;
  }
  return true;
}

void configuraMPU() {
  escreveRegistro(REG_PWR_MGMT_1, 0x00);   // acorda
  delay(100);
  escreveRegistro(REG_PWR_MGMT_1, 0x01);   // clock pelo PLL do giroscópio X
  escreveRegistro(REG_CONFIG, 0x03);       // filtro digital ≈ 44 Hz
  escreveRegistro(REG_SMPLRT_DIV, 9);      // 100 Hz
  escreveRegistro(REG_ACCEL_CONFIG, 0x00); // ±2 g (mesma escala da calibração)
  delay(100);
}

//  Cálculo do ângulo
void compensa(const float a[3], float c[3]) {
  for (int i = 0; i < 3; i++) c[i] = (a[i] - OFFSET[i]) / GANHO[i];
}

// Ângulo de inclinação no plano de rotação [graus]
float anguloInclinacao(const float a[3]) {
  return atan2f(sinal * a[eixoBraco], -a[2]) * RAD_TO_DEG;
}

// Inclinação do vetor gravidade para fora do plano de rotação [graus].
// Idealmente 0: indica desalinhamento do sensor em relação ao eixo de giro.
float anguloForaPlano(const float a[3]) {
  int outro = (eixoBraco == 0) ? 1 : 0;
  float noPlano = sqrtf(a[eixoBraco] * a[eixoBraco] + a[2] * a[2]);
  return atan2f(a[outro], noPlano) * RAD_TO_DEG;
}

//  Excentricidade do transferidor
// Ângulo real do braço [graus] a partir da leitura do ponteiro
float referenciaCorrigida(float leituraT) {
  float phi = (leituraT - 90.0f) * DEG_TO_RAD;
  float R = RAIO_ESCALA_MM, e = EXCENTRICIDADE_MM;
  return atan2f(R * sinf(phi), e + R * cosf(phi)) * RAD_TO_DEG;
}

// Leitura prevista do ponteiro [graus] para o ângulo real theta
float leituraPrevista(float thetaGraus) {
  float th = thetaGraus * DEG_TO_RAD;
  float R = RAIO_ESCALA_MM, e = EXCENTRICIDADE_MM;
  float s = sinf(th), c = cosf(th);
  float rad = R * R - e * e * s * s;
  if (rad < 0) rad = 0;
  float t = e * c + sqrtf(rad);
  return atan2f(t * s, t * c - e) * RAD_TO_DEG + 90.0f;
}

//  Configuração salva (NVS)
void carregaConfig() {
  prefs.begin("angulo", true);
  eixoBraco = prefs.getInt("eixo", 0);
  sinal     = prefs.getInt("sinal", -1);
  prefs.end();
}

void salvaConfig() {
  prefs.begin("angulo", false);
  prefs.putInt("eixo", eixoBraco);
  prefs.putInt("sinal", sinal);
  prefs.end();
}

void mostraAjuda() {
  Serial.println(F("\n# =============== MEDIÇÃO DO ÂNGULO DE INCLINAÇÃO ==============="));
  Serial.printf("# Eixo ao longo do braço: %c | sinal: %+d\n", eixoBraco == 0 ? 'X' : 'Y', sinal);
  Serial.printf("# Transferidor: excentricidade %.1f mm, raio %.0f mm\n", EXCENTRICIDADE_MM, RAIO_ESCALA_MM);
  Serial.println(F("# Ganho / Offset da calibração:"));
  for (int i = 0; i < 3; i++)
    Serial.printf("#   %c: Ganho = %.6f  Offset = %+.6f g\n", "XYZ"[i], GANHO[i], OFFSET[i]);
  Serial.println(F("# Para registrar um ponto: digite a leitura do transferidor (ex.: 120) + ENTER"));
  Serial.println(F("# T tabela CSV | C limpa | S sentido | G plotter | P pausa | X/Y eixo | I inverte | ? ajuda"));
  Serial.printf("# Pontos registrados: %d | sentido atual: %s\n\n", nPontos, descendo ? "DESCIDA" : "SUBIDA");
}

//  Registro de um ponto do ensaio
void registraPonto(float leituraT) {
  if (nPontos >= MAX_PONTOS) { Serial.println(F("# Tabela cheia. Envie T para copiar e C para limpar.")); return; }
  if (leituraT < 0 || leituraT > 180) { Serial.println(F("# Leitura inválida: use 0 a 180.")); return; }

  Serial.printf("# Medindo ponto: transferidor = %.1f° (ref corrigida = %+.2f°). Não toque no braço...\n",
                leituraT, referenciaCorrigida(leituraT));
  delay(ACOMODACAO_MS);

  double soma = 0, soma2 = 0, somaBruto = 0, somaFora = 0;
  int validas = 0;
  uint32_t t = micros();
  for (int k = 0; k < N_PONTO; k++) {
    while ((uint32_t)(micros() - t) < PERIODO_US) {}
    t += PERIODO_US;
    float a[3], c[3];
    if (!leAcelerometro(a)) continue;
    compensa(a, c);
    float th = anguloInclinacao(c);
    soma += th;  soma2 += (double)th * th;
    somaBruto += anguloInclinacao(a);
    somaFora  += anguloForaPlano(c);
    validas++;
  }
  if (validas < N_PONTO / 2) { Serial.println(F("# ERRO: falhas de leitura no I2C. Repita o ponto.")); return; }

  Ponto &p = pontos[nPontos];
  p.leituraT  = leituraT;
  p.ref       = referenciaCorrigida(leituraT);
  p.medido    = soma / validas;
  p.desvio    = sqrt(fmax(0.0, (soma2 - soma * soma / validas) / (validas - 1)));
  p.semComp   = somaBruto / validas;
  p.foraPlano = somaFora / validas;
  p.descida   = descendo;
  nPontos++;

  Serial.printf("# PONTO %d (%s): ref = %+.2f° | medido = %+.2f° (desvio %.2f°) | erro = %+.2f° | sem compensação: erro = %+.2f°\n",
                nPontos, p.descida ? "descida" : "subida", p.ref, p.medido, p.desvio,
                p.ref - p.medido, p.ref - p.semComp);
  if (fabs(p.foraPlano) > 3.0f)
    Serial.printf("# AVISO: gravidade %.1f° fora do plano de rotação. Confira o alinhamento do sensor.\n", p.foraPlano);
}

void imprimeTabela() {
  Serial.println(F("\n# ----- copie as linhas abaixo (incluindo o cabeçalho) -----"));
  Serial.println(F("ponto,sentido,leitura_transferidor,theta_ref,theta_medido,desvio,erro,theta_sem_comp,erro_sem_comp,fora_do_plano"));
  for (int i = 0; i < nPontos; i++) {
    Ponto &p = pontos[i];
    Serial.printf("%d,%s,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
                  i + 1, p.descida ? "descida" : "subida", p.leituraT, p.ref, p.medido, p.desvio,
                  p.ref - p.medido, p.semComp, p.ref - p.semComp, p.foraPlano);
  }
  Serial.println(F("# ----- fim da tabela -----\n"));
}

//  Comandos
void trataComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  // Número = leitura do transferidor -> registra um ponto
  if (isDigit(c) || c == '.' ) {
    cmd.replace(',', '.');                 // aceita "112,5"
    registraPonto(cmd.toFloat());
    return;
  }

  switch (toupper(c)) {
    case 'T': imprimeTabela(); break;
    case 'C': nPontos = 0; Serial.println(F("# Tabela apagada.")); break;
    case 'S':
      descendo = !descendo;
      Serial.printf("# Sentido da varredura: %s\n", descendo ? "DESCIDA" : "SUBIDA");
      break;
    case 'G':
      modoPlotter = !modoPlotter;
      Serial.printf("# Formato %s\n", modoPlotter ? "Serial Plotter" : "texto");
      break;
    case 'P':
      pausado = !pausado;
      Serial.printf("# Exibição %s\n", pausado ? "pausada" : "retomada");
      break;
    case 'X': eixoBraco = 0; salvaConfig(); Serial.println(F("# Eixo do braço: X")); break;
    case 'Y': eixoBraco = 1; salvaConfig(); Serial.println(F("# Eixo do braço: Y")); break;
    case 'I': sinal = -sinal; salvaConfig(); Serial.printf("# Sinal do ângulo: %+d\n", sinal); break;
    default:  mostraAjuda();
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PINO_SDA, PINO_SCL);
  Wire.setClock(400000);

  uint8_t who = leRegistro(REG_WHO_AM_I);
  Serial.printf("\n# MPU-6050 WHO_AM_I = 0x%02X %s\n", who, who == 0x68 ? "(OK)" : "(confira as ligações)");

  configuraMPU();
  carregaConfig();
  mostraAjuda();
}

void loop() {
  if (Serial.available()) trataComando(Serial.readStringUntil('\n'));

  // ---- aquisição a 100 Hz com média móvel ----
  static uint32_t tAmostra = micros();
  if ((uint32_t)(micros() - tAmostra) > 5 * PERIODO_US) tAmostra = micros();  // após medir um ponto
  if ((uint32_t)(micros() - tAmostra) >= PERIODO_US) {
    tAmostra += PERIODO_US;
    float a[3], c[3];
    if (leAcelerometro(a)) {
      compensa(a, c);
      for (int i = 0; i < 3; i++) { bufComp[idxBuf][i] = c[i]; bufBruto[idxBuf][i] = a[i]; }
      idxBuf = (idxBuf + 1) % N_MEDIA_MOVEL;
      if (cheioBuf < N_MEDIA_MOVEL) cheioBuf++;
    }
  }

  // ---- exibição em tempo real ----
  static uint32_t tTela = 0;
  if (!pausado && cheioBuf > 0 && millis() - tTela >= INTERVALO_TELA_MS) {
    tTela = millis();
    float mc[3] = {0, 0, 0}, mb[3] = {0, 0, 0};
    for (int k = 0; k < cheioBuf; k++)
      for (int i = 0; i < 3; i++) { mc[i] += bufComp[k][i]; mb[i] += bufBruto[k][i]; }
    for (int i = 0; i < 3; i++) { mc[i] /= cheioBuf; mb[i] /= cheioBuf; }

    float th     = anguloInclinacao(mc);
    float thBrut = anguloInclinacao(mb);
    float fora   = anguloForaPlano(mc);
    float modulo = sqrtf(mc[0] * mc[0] + mc[1] * mc[1] + mc[2] * mc[2]);

    if (modoPlotter) {
      Serial.printf("angulo:%.2f,sem_compensacao:%.2f\n", th, thBrut);
    } else {
      Serial.printf("Inclinação = %+7.2f°  (transferidor ≈ %6.1f°) | sem compensação: %+7.2f° | fora do plano: %+5.1f° | |g| = %.3f\n",
                    th, leituraPrevista(th), thBrut, fora, modulo);
    }
  }
}
