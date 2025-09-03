// UNO (maître) : envoie pos/temp/hum à l'ESP32 et affiche sur LCD 16x2
#include <DHT11.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define ESP32_ADDR  0x08
#define LCD_ADDR    0x27
#define I2C_SPEED   100000UL

// === Définition GPIO ===
#define LED1 3
#define LED2 4
#define LED3 5


int GPIObutton1 = 6;
int GPIObutton2 = 7;
int GPIObutton3 = 8;
int GPIObutton4 = 9;

int GPIOHCSR04Trig = 10;   // Trig HC-SR04
int GPIOHCSR04Echo = 11;   // Echo HC-SR04
int GPIOBUZZER     = 12;   // Buzzer
int GPIODHT11      = 13;   // DHT11



// === Flags d'événements (appuis détectés) + anti-rebond ===
volatile bool btn1Pressed = false;
volatile bool btn2Pressed = false;
volatile bool btn3Pressed = false;
volatile bool btn4Pressed = false;

volatile uint8_t prevPINB;  // état précédent du port B (D8..D13)
volatile uint8_t prevPIND;  // état précédent du port D (D0..D7)

unsigned long lastPress1 = 0;
unsigned long lastPress2 = 0;
unsigned long lastPress3 = 0;
unsigned long lastPress4 = 0;


unsigned long lastPress = 0;


// === Horloge (simple) ===
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 sec
int secondes = 0, minutes = 0, heures = 0;
int jour = 21, mois = 8, annee = 2025;
int calandarMap = 0; // 0:sec 1:min 2:hr 3:jour 4:mois 5:annee

uint8_t screen ;   // état ON/OFF de l'ESP32
uint8_t  state ;   // écran demandé par l'ESP32 (0..N)

// === Affichage séquentiel ===
unsigned long lastSwitch = 0;
int currentScreen = 0;              // (utilisé si tu veux cycler localement)
const unsigned long screenDelay = 3000;

// === Proximité / Backlight ===
const int  PROX_NEAR_CM = 20;
const int  PROX_FAR_CM  = 24;
const unsigned long INACTIVITY_MS = 30000;
bool presence = false;
unsigned long lastPresenceMillis=0;
bool backlightOn = true;




// === Anti-rebond boutons ===
const unsigned long DEBOUNCE_MS = 30;
struct Button {
  uint8_t pin;
  bool lastRaw;
  bool stable;
  unsigned long lastChange;
};
Button btns[4] = {
  { (uint8_t)GPIObutton1, HIGH, HIGH, 0 },
  { (uint8_t)GPIObutton2, HIGH, HIGH, 0 },
  { (uint8_t)GPIObutton3, HIGH, HIGH, 0 },
  { (uint8_t)GPIObutton4, HIGH, HIGH, 0 },
};

// === Prototypes ===
void LCD_DHT11(int temperature, int humidity);
void LCD_D_H();
void LCD_HC_SR04();
long lireDistanceMediane();
void setAlert(bool on);
void updateBacklight();
void wakeBacklight();
void printLine(uint8_t col, uint8_t row, const String &txt);
bool buttonPressed(Button &b);
void addToCalendarField(int delta);

// === Helpers I2C (casts AVR) ===
uint8_t readReg8(uint8_t reg) {
  Wire.beginTransmission((uint8_t)ESP32_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF; // repeated START
  if (Wire.requestFrom((uint8_t)ESP32_ADDR, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}
bool writeReg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission((uint8_t)ESP32_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

// === Implémentations ===
bool buttonPressed(Button &b) {
  bool raw = digitalRead(b.pin);
  if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChange = millis(); }
  if ((millis() - b.lastChange) > DEBOUNCE_MS) {
    if (b.stable != b.lastRaw) {
      b.stable = b.lastRaw;
      if (b.stable == LOW) return true; // INPUT_PULLUP: appui = LOW
    }
  }
  return false;
}

void addToCalendarField(int delta) {
  int* fields[] = { &secondes, &minutes, &heures, &jour, &mois, &annee };
  if (*fields[calandarMap] && calandarMap >= 0 && calandarMap < 6) *fields[calandarMap] += delta;
  Serial.print(" | Added="); Serial.println(delta);
}




// === Utilitaires ===
inline void processButton1() {
  switch (screen) {
    case 0: break;
    case 1: break;
    default:
      calandarMap = (calandarMap + 1) % 6; // suivant
      break;
  }
}

inline void processButton2() {
  switch (screen) {
    case 0: break;
    case 1: break;
    default:
      calandarMap = (calandarMap + 5) % 6; // équiv. (calandarMap - 1) % 6
      break;
  }
}

inline void processButton3() {
  switch (screen) {
    case 0: break;
    case 1: break;
    default:
      addToCalendarField(1);
      break;
  }
}

inline void processButton4() {
  switch (screen) {
    case 0: break;
    case 1: break;
    default:
      addToCalendarField(-1);
      break;
  }
}

// === Objets ===
Ultrasonic ultrasonic(GPIOHCSR04Trig, GPIOHCSR04Echo);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
DHT11 dht11(GPIODHT11);


void setup() {
  Wire.begin();                    // UNO: SDA=A4, SCL=A5
  Wire.setClock(I2C_SPEED);
  Serial.begin(115200);

  pinMode(GPIObutton1, INPUT_PULLUP);
  pinMode(GPIObutton2, INPUT_PULLUP);
  pinMode(GPIObutton3, INPUT_PULLUP);
  pinMode(GPIObutton4, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(GPIOBUZZER, OUTPUT);

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(GPIOBUZZER, LOW);

  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print("MaxiBox");
  delay(800); lcd.clear();

  // États initiaux des ports
  prevPINB = PINB; // D8..D13
  prevPIND = PIND; // D0..D7

  // === Active les interruptions "Pin Change" pour PB0/PB1 (D8,D9) ===
  PCICR  |= (1 << PCIE0);           // autorise PCINT[7:0] (PORTB)
  PCMSK0 |= (1 << PCINT0);          // PB0 (D8)
  PCMSK0 |= (1 << PCINT1);          // PB1 (D9)

  // === Active les interruptions "Pin Change" pour PD6/PD7 (D6,D7) ===
  PCICR  |= (1 << PCIE2);           // autorise PCINT[23:16] (PORTD)
  PCMSK2 |= (1 << PCINT22);         // PD6 (D6)
  PCMSK2 |= (1 << PCINT23);         // PD7 (D7)

  // Pas besoin de Serial ici, mais utile pour debug
  // Serial.begin(115200);
}

// == ISR pour PORTB (D8..D13) ==
ISR(PCINT0_vect) {
  uint8_t curr = PINB;
  uint8_t changed = curr ^ prevPINB;

  // D8 (PB0)
  if (changed & (1 << PB0)) {
    // front descendant ? (HIGH -> LOW) sachant INPUT_PULLUP
    if ( (prevPINB & (1 << PB0)) && !(curr & (1 << PB0)) ) {
      btn3Pressed = true; // bouton3 sur D8
    }
  }
  // D9 (PB1)
  if (changed & (1 << PB1)) {
    if ( (prevPINB & (1 << PB1)) && !(curr & (1 << PB1)) ) {
      btn4Pressed = true; // bouton4 sur D9
    }
  }
  prevPINB = curr; // mets à jour l'état précédent
}

// == ISR pour PORTD (D0..D7) ==
ISR(PCINT2_vect) {
  uint8_t curr = PIND;
  uint8_t changed = curr ^ prevPIND;

  // D6 (PD6)
  if (changed & (1 << PD6)) {
    if ( (prevPIND & (1 << PD6)) && !(curr & (1 << PD6)) ) {
      btn1Pressed = true; // bouton1 sur D6
    }
  }
  // D7 (PD7)
  if (changed & (1 << PD7)) {
    if ( (prevPIND & (1 << PD7)) && !(curr & (1 << PD7)) ) {
      btn2Pressed = true; // bouton2 sur D7
    }
  }
  prevPIND = curr;

}




void loop() {

  digitalWrite(LED1,HIGH) ;

  unsigned long now = millis();

  if (millis() - lastPress >= 20) {
    digitalWrite(LED2,  LOW);
  }

  
  if (state==0xFF) {
    digitalWrite(LED3,  LOW);
  }else{digitalWrite(LED3,  HIGH);}
  
  // On traite les événements hors ISR + anti-rebond
  if (btn1Pressed && (now - lastPress1 >= DEBOUNCE_MS)) {
    noInterrupts();
    btn1Pressed = false;
    interrupts();
    lastPress1 = now;
    processButton1();
  }
  if (btn2Pressed && (now - lastPress2 >= DEBOUNCE_MS)) {
    noInterrupts();
    btn2Pressed = false;
    interrupts();
    lastPress2 = now;
    processButton2();
  }
  if (btn3Pressed && (now - lastPress3 >= DEBOUNCE_MS)) {
    noInterrupts();
    btn3Pressed = false;
    interrupts();
    lastPress3 = now;
    processButton3();
  }
  if (btn4Pressed && (now - lastPress4 >= DEBOUNCE_MS)) {
    noInterrupts();
    btn4Pressed = false;
    interrupts();
    lastPress4 = now;
    processButton4();
  }

  // --- Capteurs ---
  int ti = dht11.readTemperature();     // int (lib DHT11)
  int hi = dht11.readHumidity();        // int
  long pd = lireDistanceMediane();      // cm

  // Clamp / transformation vers uint8_t
  uint8_t t = (ti >= 0 && ti <= 255) ? (uint8_t)ti : 0xFF;
  uint8_t h = (hi >= 0 && hi <= 100) ? (uint8_t)hi : 0xFF;
  uint8_t p = (pd >= 0 && pd <= 255) ? (uint8_t)pd : 0xFF;

  // --- Envois I2C -> ESP32 ---
  writeReg8(0x29, p);   // position / distance
  writeReg8(0x30, t);   // temperature
  writeReg8(0x31, h);   // humidite

  // --- Lectures I2C <- ESP32 ---
  screen  = readReg8(0x01);   // état ON/OFF de l'ESP32
  state   = readReg8(0x02);   // écran demandé par l'ESP32 (0..N)

  // --- Affichage selon écran ---
  switch (screen) {
    case 0:
      LCD_D_H();
      break;

    case 1:
      LCD_HC_SR04();
      break;

    case 2:
      LCD_DHT11(t, h);
      break;

    default:
      LCD_D_H();
      break;
  }

  // --- Avance horloge simple ---
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    secondes++;
    if (secondes >= 60) { secondes = 0; minutes++; }
    if (minutes >= 60) { minutes = 0; heures++; }
    if (heures >= 24)   { heures = 0; jour++; }
    if (jour > 30)      { jour = 1; mois++; } // simplifié
    if (mois > 12)      { mois = 1; annee++; }
  }

  // Debug série compact
  Serial.print("[UNO] T="); Serial.print((int)t);
  Serial.print(" H="); Serial.print((int)h);
  Serial.print(" | state="); Serial.print((state==0xFF)?-1:state);
  Serial.print(" screen="); Serial.print((int)screen);
  Serial.print(" | calMap="); Serial.println(calandarMap);

  delay(500);
}





// === Outils d'affichage propre (remplir ligne) ===
void printLine(uint8_t col, uint8_t row, const String &txt) {
  lcd.setCursor(col, row);
  String s = txt;
  while ((int)s.length() < (16 - col)) s += ' ';  // LCD 16 colonnes
  lcd.print(s);
}

// === Température & humidité ===
void LCD_DHT11(int temperature, int humidity) {
  wakeBacklight();
  String tStr = (temperature == 0xFF) ? "--" : String(temperature);
  String hStr = (humidity    == 0xFF) ? "--" : String(humidity) + "%";
  printLine(0, 0, "Temp: " + tStr + (char)223 + "C");
  printLine(0, 1, "Hum:  " + hStr);
}

// === Horloge ===
void LCD_D_H() {
  wakeBacklight();
  char buf1[17];
  char buf2[17];
  snprintf(buf1, sizeof(buf1), "%02d:%02d:%02d", heures, minutes, secondes);
  snprintf(buf2, sizeof(buf2), "%02d/%02d/%04d", jour, mois, annee);
  printLine(0, 0, String(buf1));
  printLine(0, 1, String(buf2));
}

// === Distance ===
void LCD_HC_SR04() {
  wakeBacklight();
  long distance = lireDistanceMediane(); // cm (médiane de 3 mesures)
  printLine(0, 0, "Distance:");
  if (distance == 0) {
    printLine(0, 1, "Aucun echo");
    setAlert(false);
  } else if (distance < 2 || distance > 400) {
    printLine(0, 1, "Hors portee");
    setAlert(false);

  } else {
    printLine(0, 1, String(distance) + " cm");
    // Hystérésis de présence
    if (!presence && distance <= PROX_NEAR_CM) {
      presence = true; lastPresenceMillis = millis(); setAlert(true);
    } else if (presence && distance >= PROX_FAR_CM) {
      presence = false; 
      setAlert(false);
    } else if (presence) {
      lastPresenceMillis = millis();
      setAlert(false);
    }
  }
}

// --- 3 lectures + médiane pour stabilité ---
long lireDistanceMediane() {
  long a = ultrasonic.Ranging(CM); delay(40);
  long b = ultrasonic.Ranging(CM); delay(40);
  long c = ultrasonic.Ranging(CM);
  long m;
  if ((a <= b && b <= c) || (c <= b && b <= a)) m = b;
  else if ((b <= a && a <= c) || (c <= a && a <= b)) m = a;
  else m = c;
  return m;
}

// --- Pilotage Buzzer/LEDs ---
void setAlert(bool on) {
  digitalWrite(LED2, HIGH);
  digitalWrite(GPIOBUZZER, on ? HIGH : LOW);
}

// --- Gestion automatique du rétro-éclairage ---
void updateBacklight() {
  unsigned long now = millis();
  if (backlightOn && (now - lastPresenceMillis >= INACTIVITY_MS)) {
    lcd.noBacklight();
    backlightOn = false;
  }
}
void wakeBacklight() {
  if (!backlightOn) { lcd.backlight(); backlightOn = true; }
}
