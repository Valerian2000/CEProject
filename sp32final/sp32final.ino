// ESP32 (esclave I2C @0x08) + serveur Web (HTML/CSS + API JSON)
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#define I2C_SLAVE_ADDR 0x08

// ---------- Wi-Fi ----------
const char* WIFI_SSID = "MaxiBox";     // Nom du wifi
const char* WIFI_PASS = "maxi123";      // mot de passe du wifi
const char* AP_SSID   = "MaxiBox";
const char* AP_PASS   = "12345678";

WebServer server(80);

// ---------- État/Mesures échangés via I2C ----------
volatile bool    stateLunched = false;   // renvoyé au maître (reg 0x01)
volatile uint8_t screenPC = 0xFF;           // renvoyé au maître (reg 0x02)

volatile uint8_t tempC = 0xFF;           // reçu du maître (reg 0x30)
volatile uint8_t humPC = 0xFF;           // reçu du maître (reg 0x31)
volatile uint8_t posPC = 0xFF;           // reçu du maître (reg 0x29)

volatile uint8_t lastRegPtr = 0x00;      // registre à lire lors d'une requête

// SEGMENTS a, b, c, d, e, f, g, dp
const int segPins[8] = {13, 12, 14, 27, 26, 25, 23, 2};  
// ajoute GPIO32 pour le point décimal


// DIGITS (cathodes communes)
const int digitPins[4] = {16, 17, 18, 19};  // digits 1 à 4

      
unsigned long lastInc = 0; 
// Table des segments pour les chiffres 0–9
// Format : a,b,c,d,e,f,g,dp
const byte digits[10][8] = {
  {0,0,0,0,0,0,1,1}, // 0
  {1,0,0,1,1,1,1,1}, // 1
  {0,0,1,0,0,1,0,1}, // 2
  {0,0,0,0,1,1,0,1}, // 3
  {1,0,0,1,1,0,0,1}, // 4
  {0,1,0,0,1,0,0,1}, // 5
  {0,1,0,0,0,0,0,1}, // 6
  {0,0,0,1,1,1,1,1}, // 7
  {0,0,0,0,0,0,0,1}, // 8
  {0,0,0,0,1,0,0,1}  // 9
};


// Définition des 4 broches tactiles
const int touchPins[4] = {T9, T8};  
// T9=GPI32, T8=GPIO33, 

int seuil = 30; // seuil de détection (à calibrer)
int ctr = 0;

// Active ou non le point par digit
bool pointOff[4] = {false, false, false, false};  
// Ici seul le digit 2 a son point activé


// ---------- I2C Callbacks ----------
void onReceiveI2C(int len) {
  if (len <= 0) return;
  uint8_t reg = Wire.read();
  int remaining = len - 1;

  if (remaining <= 0) {             // pointeur de registre pour une future lecture
    lastRegPtr = reg;
    return;
  }

  // écriture d'1 octet (payload)
  uint8_t val = Wire.read();
  switch (reg) {
    case 0x29: posPC = val; break;  // postion  reçue******
    case 0x30: tempC = val; break;  // température reçue
    case 0x31: humPC = val; break;  // humidité reçue
    default: break;
  }
  while (Wire.available()) Wire.read(); // purge si plus d'1 octet
}

void onRequestI2C() {
  uint8_t out = 0xFF;
  switch (lastRegPtr) {
    case 0x01: out = stateLunched ? 1 : 0; break;   // état
    case 0x02: out = screenPC;            break;   // écran (tu le modifies via touches)
    default:   out = 0xFF;                break;   // registre inconnu
  }
  Wire.write(&out, 1);  // UNE SEULE écriture pour la requête
}


// ---------- HTML (template minimal, CSS + JS) ----------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="fr"><head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>MaxiBox Dashboard</title>
<style>
  :root { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }
  body { margin:0; background:#0f172a; color:#e2e8f0; display:flex; min-height:100vh; }
  .wrap { margin:auto; width:min(720px, 92vw); }
  .card { background:#111827; border:1px solid #1f2937; border-radius:16px; padding:20px; box-shadow:0 6px 20px rgba(0,0,0,.35); }
  h1 { margin:0 0 10px; font-size:22px; }
  .grid { display:grid; grid-template-columns:1fr 1fr 1fr; gap:12px; margin-top:10px; }
  .tile { background:#0b1220; border:1px solid #1f2937; border-radius:12px; padding:16px; text-align:center; }
  .lbl { opacity:.7; font-size:12px; letter-spacing:.04em; text-transform:uppercase; margin-bottom:6px; }
  .val { font-size:28px; font-weight:700; }
  .btn { display:inline-block; margin-top:14px; background:#2563eb; color:white; border:none; padding:10px 14px; border-radius:10px; cursor:pointer; text-decoration:none; }
  .btn:active { transform: translateY(1px); }
  .foot { margin-top:12px; font-size:12px; opacity:.65; }
</style>
</head><body>
  <div class="wrap">
    <div class="card">
      <h1>MaxiBox</h1>
      <div class="grid">
        <div class="tile"><div class="lbl">État</div><div id="st" class="val">--</div></div>
        <div class="tile"><div class="lbl">Température</div><div id="t" class="val">--</div></div>
        <div class="tile"><div class="lbl">Humidité</div><div id="h" class="val">--</div></div>
        <div class="tile"><div class="lbl">Distance</div><div id="d" class="val">--</div></div>
      </div>
      <a class="btn" href="/toggle">Basculer l'état</a>
      <div class="foot">Mise à jour auto toutes les 1 s</div>
    </div>
  </div>
<script>
async function refresh(){
  try{
    const r = await fetch('/api/status'); if(!r.ok) throw 0;
    const dr = await r.json();
    document.getElementById('st').textContent = dr.launched ? 'ON' : 'OFF';
    document.getElementById('t').textContent  = (dr.tempC===255 ? '--' : (dr.tempC + '°C'));
    document.getElementById('h').textContent  = (dr.humPC===255 ? '--' : (dr.humPC + '%'));
    document.getElementById('d').textContent  = (dr.posPC===255 ? '--' : dr.posPC);
  }catch(e){}
}
refresh(); setInterval(refresh, 1000);
</script>

</body></html>
)HTML";

// ---------- Handlers HTTP ----------
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleAPI() {
  noInterrupts();
  bool launched = stateLunched;
  uint8_t s = screenPC, t = tempC, h = humPC, p = posPC;
  interrupts();

  String json = "{";
  json += "\"launched\":"; json += (launched ? "true":"false"); json += ",";
  json += "\"tempC\":";    json += String(t); json += ",";
  json += "\"humPC\":";    json += String(h); json += ",";
  json += "\"posPC\":";    json += String(p);
  json += "}";

  server.send(200, "application/json", json);
}


void handleToggle() {
  // bascule l'état et redirige à la page d'accueil
  noInterrupts();
  stateLunched = !stateLunched;
  interrupts();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "OK");
}

// ---------- Wi-Fi helpers ----------
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0 < 7000) delay(100);

  if (WiFi.status()==WL_CONNECTED) {
    Serial.print("WiFi STA connecté: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("STA échec -> SoftAP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP: "); Serial.print(AP_SSID);
    Serial.print("  IP: "); Serial.println(WiFi.softAPIP());
  }
}

void startHTTP() {
  server.on("/", handleRoot);
  server.on("/api/status", handleAPI);
  server.on("/toggle", handleToggle);
  server.begin();
  Serial.println("HTTP server démarré sur port 80");
}
// Affiche un nombre sur 4 digits
void displayNumber(int number) {
  int n[4] = {
    (number / 1000) % 10,
    (number / 100) % 10,
    (number / 10) % 10,
    number % 10
  };

  for (int i = 0; i < 4; i++) {
    lightDigit(i, n[i], pointOff[i]);
    delay(3);  
    clearDisplay();
  }
}

// Active un chiffre (digit) et ses segments + point
void lightDigit(int digitIndex, int num, bool showPoint) {
  for (int i = 0; i < 7; i++) {
    digitalWrite(segPins[i], digits[num][i] ? HIGH : LOW);
  }
  digitalWrite(segPins[7], showPoint ? HIGH : LOW); // DP
  digitalWrite(digitPins[digitIndex], HIGH); // cathode commune → HIGH active le digit
}

// Éteint tous les digits
void clearDisplay() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(digitPins[i], LOW);
  }
}


// en haut du fichier
unsigned long lastT9 = 0, lastT8 = 0;
const unsigned long debounceMs = 120;

void readtouches() {
  const int touchPinsFiltered[2] = { T9, T8 };
  int v9 = touchRead(touchPinsFiltered[0]);
  int v8 = touchRead(touchPinsFiltered[1]);
  unsigned long now = millis();

  // rappel: sur ESP32, touchRead() baisse quand on touche -> seuil "val < seuil"
  if (v9 < seuil && now - lastT9 > debounceMs) {
    lastT9 = now;
    stateLunched = !stateLunched;
    ctr = 0 ;
    screenPC = 0;
    Serial.print("GPIO32 touch -> stateLunched=");
    Serial.println(stateLunched);
  }

  if (v8 < seuil && now - lastT8 > debounceMs) {
    lastT8 = now;
    if (stateLunched) screenPC = (screenPC == 2) ? 0 : screenPC + 1;
    Serial.print("GPIO33 touch -> screenPC=");
    Serial.println(screenPC);
  }
}


// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  //Serial.println("\nESP32 I2C Slave + Web");

  // I2C esclave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceiveI2C);
  Wire.onRequest(onRequestI2C);

  for (int i = 0; i < 8; i++) {
    pinMode(segPins[i], OUTPUT);
    digitalWrite(segPins[i], LOW);
    }
  for (int i = 0; i < 4; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], LOW);
    }


  // Wi-Fi + HTTP
  startWiFi();
  startHTTP();
}

void loop() {
  server.handleClient();
  // petit témoin visuel
  //digitalWrite(LED_BUILTIN, stateLunched ? HIGH : LOW);
  readtouches();
  // Incrémenter le compteur toutes les secondes
  if (millis() - lastInc >= 1000) {
    ctr++;
    if (ctr > 9999) ctr = 0;
    lastInc = millis();
    }

  if (stateLunched){

    switch (screenPC) {

      case 0:
        displayNumber(ctr);
        break;            // sort du switch

      case 1:
        displayNumber(1);
            break;
      
      case 2:
        displayNumber(2);
            break;
      

      default:
    // si aucune valeur ne correspond
        break;
        }
    }

  delay(5);
}




