#include <Wire.h>
#include <LCD_1602_RUS.h>
#include <EEPROM.h>
#include <PID_v1_bc.h>
#include "max6675.h"

/* пины */
constexpr uint8_t PIN_SSR   = 8;   // релюшка
constexpr uint8_t PIN_BTN_L = 6;   // кнопка -10 мин
constexpr uint8_t PIN_BTN_R = 5;   // кнопка +10 мин
constexpr uint8_t LCD_ADDR  = 0x27;
LCD_1602_RUS lcd(LCD_ADDR, 16, 2);
MAX6675 thermocouple(4, 3, 2);     // SCK, CS, SO (DO)

/* ПИД Регулятр */
double Input  = 25.0;      // температура на всякий
double Setpoint = 20.0;    // уставка
double Output = 0.0;       // 0…WINDOW_SIZE мс
const  double Kp = 3;
double Ki = 0.2;
const  double Kd = 0;
constexpr uint32_t WINDOW_SIZE = 2000UL;   // 2 s
uint32_t windowStartMs = 0;

/* процент на экран */
#define SM_BUF 4
uint8_t dutyBuf[SM_BUF] = {0};
uint8_t dutyIdx = 0;

/* Температурный профиль Prestige для обычных */
struct Phase { uint16_t startMin, endMin; uint16_t tStart, tEnd; char code; };
const Phase phases[] PROGMEM = {
  /*      время, мин    t°С   код */
  {   0,  60,  20, 150, 'R'},   // Ramp 20→150 за 1 ч
  {  60, 240, 150, 150, 'S'},   // Soak 150 °С 3 ч
  { 240, 300, 150, 370, 'R'},   // Ramp 150→370 за 1 ч
  { 300, 420, 370, 370, 'S'},   // Soak 370 °С 2 ч
  { 420, 540, 370, 750, 'R'},   // Ramp 370→750 за 2 ч
  { 540, 780, 750, 750, 'S'},   // Soak 750 °С 4 ч
  { 780, 0xFFFF, 640, 640, 'T'} // Standby 640 °С (бессрочно)
};
constexpr uint8_t LAST_PHASE_IDX = sizeof(phases)/sizeof(Phase)-1;
uint8_t phaseIdx = 0;

/* EEPROM журнал */
constexpr uint8_t LOG_START = 3;
constexpr uint8_t PAIRS     = 60;                 // 60*10 мин = 10 ч
constexpr uint8_t LOG_END   = LOG_START + PAIRS*2 - 1;
uint8_t  eadr;                         // указатель «карандаш»
constexpr uint32_t SAVE_MS = 6000UL; // 10 мин 600000UL;
bool standbyLogged = false;

/* временые челики */
uint32_t progStartMs = 0;  // millis() соответствующий времени 0:00, ну если из епрома ниче не прочитаем
uint32_t lastSaveMs  = 0;

/* ПИД */
PID kilnPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* Вспомогательные функции */
inline uint32_t elapsedMs() { return millis() - progStartMs; }

void updatePhase(uint32_t elapsedMin) {
  for (uint8_t i=0;i<sizeof(phases)/sizeof(Phase);++i) {
    Phase ph; memcpy_P(&ph, &phases[i], sizeof(Phase));
    if (elapsedMin >= ph.startMin && elapsedMin < ph.endMin) {
      phaseIdx = i;
      float f = float(elapsedMin - ph.startMin) /
                float(ph.endMin - ph.startMin);
      Setpoint = ph.tStart + (ph.tEnd - ph.tStart) * f;
      return;
    }
  }
  phaseIdx = LAST_PHASE_IDX;
  Setpoint = 640;
}

void fmtTime(char *buf, uint32_t sec) {
  sprintf(buf, "%02lu:%02lu:%02lu",
          sec/3600UL, (sec/60UL)%60, sec%60);
}


/* рассчет crc */
uint8_t crc8(uint8_t v) {
  for (uint8_t i=0;i<8;++i)
    v = (v & 0x80) ? (v<<1) ^ 0x31 : (v<<1);
  return v;
}
/* запись в епром */
void saveSnapshot(uint16_t val10) {          // val - кол-во минут/10
  uint8_t crc = crc8(val10);
  EEPROM.update(eadr,     val10);
  EEPROM.update(eadr + 1, crc);
  eadr += 2; if (eadr > LOG_END) eadr = LOG_START;
  EEPROM.update(0, eadr);                   // обновление указателя
}
/* загрузка снимка */
bool loadSnapshot(uint32_t &elapsedSec) {
  eadr = EEPROM.read(0);
  if (eadr < LOG_START || eadr > LOG_END) eadr = LOG_START;
  for (uint8_t p=eadr; p!=uint8_t(eadr-2); p=(p<=LOG_START?LOG_END:p-2)) {
    uint8_t v = EEPROM.read(p);
    uint8_t c = EEPROM.read(p+1);
    if (crc8(v) == c) { elapsedSec = uint32_t(v)*600UL; return true; }
  }
  return false;
}

/* setup() */
void setup(){
  Serial.begin(9600);
  pinMode(PIN_SSR, OUTPUT); digitalWrite(PIN_SSR, LOW);
  pinMode(PIN_BTN_L, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);
#ifdef PIN_BUZZ
  pinMode(PIN_BUZZ, OUTPUT);
#endif

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0,1); lcd.print(F("Init…"));

  kilnPID.SetOutputLimits(0, WINDOW_SIZE);
  kilnPID.SetMode(AUTOMATIC);
  windowStartMs = millis();

  /* восстановление времени */
  uint32_t elapsedSec = 0;
  if (!loadSnapshot(elapsedSec)) elapsedSec = 0;
  progStartMs = millis() - elapsedSec*1000UL;

  standbyLogged = false;
  lcd.clear();
}
double lastTCms = 0;

/* ────── loop() ──────────────────────────────────────────────────── */
void loop() {
  static uint32_t lastBtn=0, lastTemp=0, lastLCD=0;
  uint32_t now = millis();

  /* кнопки */
  if (now - lastBtn > 300) {
    if (!digitalRead(PIN_BTN_L)) { progStartMs += 600000UL; lastBtn=now; }
    if (!digitalRead(PIN_BTN_R)) { progStartMs -= 600000UL; lastBtn=now; }
  }

  /* --- время + фаза --- */
  uint32_t eMs  = elapsedMs();
  uint32_t eMin = eMs / 60000UL;
  updatePhase(eMin);

  /* Standby-reset: единожды пишем 0 */
  if (phaseIdx == LAST_PHASE_IDX) {
    if (!standbyLogged) { saveSnapshot(0); standbyLogged = true; }
  } else standbyLogged = false;

  /* --- обнуление интегральной составляющей, для исключения перерегулирования --- */
  double lastSP = Setpoint;
  if (Setpoint < lastSP - 1.0) {
    kilnPID.SetMode(MANUAL); Output = 0; kilnPID.Initialize();
    kilnPID.SetMode(AUTOMATIC);
  }
  lastSP = Setpoint;

  /* боремся с интегральным перерегулированием*/

  if (Setpoint - Input < 10) {Ki = 0.001;}
  else {Ki = 0.2;}

   /*измеряем температуру*/
if (now - lastTCms >= 500) {
    thermocouple.readCelsius();            // пробный
    Input = thermocouple.readCelsius();    // рабочий
    lastTCms = now;
}
  /* --- оконный ШИМ 2 с --- */
  if ((now - windowStartMs) >= WINDOW_SIZE) windowStartMs = now;
  kilnPID.Compute();
  if (Input == Setpoint+2) Output = 0;   // защита перегрева
  digitalWrite(PIN_SSR, (now - windowStartMs) < Output ? HIGH : LOW);

  /* обновляем буфер duty% для красивого вывода */
  dutyBuf[dutyIdx] = uint8_t((Output*100)/WINDOW_SIZE);
  dutyIdx = (dutyIdx + 1) % SM_BUF;

  /* --- сохранение снимка каждые 10 мин --- */
  if ((now - lastSaveMs >= SAVE_MS)& (not(standbyLogged))) {
    saveSnapshot(eMin/10);
    lastSaveMs = now;
  }

  /* --- вывод на дисплей --- */
  if (now - lastLCD >= 500) {
    char line[17];
    // верхняя строка
    sprintf(line, "T %3d%c SET %3d%c",
            int(Input+0.5), 223,              // символ °
            int(Setpoint+0.5), 223);
    lcd.setCursor(0,0); lcd.print(line);

    // нижняя строка
    uint32_t sec = eMs / 1000UL;
    char tbuf[9]; fmtTime(tbuf, sec);
    uint32_t sum=0; for(uint8_t i=0;i<SM_BUF;++i) sum+= dutyBuf[i];
    uint8_t pct = sum / SM_BUF;

    Phase ph; memcpy_P(&ph, &phases[phaseIdx], sizeof(Phase));
    sprintf(line, "%s %c %3u%%", tbuf, ph.code, pct);
    lcd.setCursor(0,1); lcd.print(line);
    lastLCD = now;
  }
}

