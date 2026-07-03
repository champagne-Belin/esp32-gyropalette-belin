/*
 * ============================================================
 * CHAMPAGNE O.BELIN — Gyropalette connectée
 * ESP32 WROOM + MPU-6050
 * ============================================================
 *
 * Fonctionnement résumé :
 * - Détection automatique du départ de cycle (angle de référence
 *   mesuré au démarrage + seuil de déviation)
 * - Progression du cycle basée sur le TEMPS écoulé (84h / 3,5 jours
 *   par défaut, configurable dans le Google Sheet), PAS sur l'angle
 * - Angle enregistré en continu à titre indicatif / diagnostic
 * - État du cycle (t0, angle_repos, cycle_actif) persistant dans
 *   le Google Sheet pour survivre à une coupure réseau/électrique
 * - Pas de reboot quotidien forcé (contrairement aux modules
 *   température) — le cycle dure 3,5 jours en continu
 * - OTA via GitHub (pas d'ArduinoOTA WiFi) : vérification
 *   périodique de version.txt, flash si nouvelle version
 * - Reboot à distance via un flag dans le Sheet
 * - Alertes email à 25/50/75/100% de progression
 * - Fallback automatique entre 2 réseaux WiFi (domicile / Belin)
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <time.h>

#include "secrets.h"

// ============================================================
// CONSTANTES DU MODULE
// ============================================================
#define MODULE_ID           "gyropalette"
#define FIRMWARE_VERSION    "0.1.0"

// GitHub — OTA
#define GITHUB_VERSION_URL  "https://raw.githubusercontent.com/champagne-Belin/esp32-gyropalette-belin/main/version.txt"
#define GITHUB_BIN_URL      "https://raw.githubusercontent.com/champagne-Belin/esp32-gyropalette-belin/main/firmware/latest/firmware.bin"

// Pin LED de statut (cohérent avec les autres modules ESP32 Belin)
#define LED_STATUT           2

// I2C — MPU-6050 (pins par défaut ESP32 : SDA=21, SCL=22)
#define SDA_PIN              21
#define SCL_PIN              22

// Seuil de déviation pour détecter le départ d'un cycle (degrés)
#define SEUIL_DEPART_DEG      8.0

// Intervalles
#define INTERVALLE_LOG_MS        (5UL * 60UL * 1000UL)   // 5 min — log angle/temps
#define INTERVALLE_CHECK_MS      (10UL * 60UL * 1000UL)  // 10 min — check reboot/OTA
#define INTERVALLE_LECTURE_MS    (2000UL)                 // 2 s — lecture angle en continu

// ============================================================
// OBJETS GLOBAUX
// ============================================================
Adafruit_MPU6050 mpu;

// État du cycle (chargé depuis / sauvegardé vers le Sheet)
float    angle_repos       = 0.0;
bool     cycle_actif        = false;
time_t   t0                 = 0;
int      duree_cycle_h      = 84;      // valeur par défaut, écrasée par le Sheet
String   cycle_id           = "";

// Suivi des seuils déjà notifiés pour le cycle en cours (évite les doublons)
bool seuil_25_envoye  = false;
bool seuil_50_envoye  = false;
bool seuil_75_envoye  = false;
bool seuil_100_envoye = false;

unsigned long dernier_log      = 0;
unsigned long dernier_check    = 0;
unsigned long derniere_lecture = 0;

// --- Gestion LED de statut WiFi (non-bloquant) ---
unsigned long dernier_blink_led = 0;
bool          etat_led          = false;
#define BLINK_LENT_MS   1000   // WiFi connecté = clignotement lent
#define BLINK_RAPIDE_MS 150    // WiFi déconnecté/erreur = clignotement rapide

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Gyropalette Belin — démarrage ===");
  Serial.println("Firmware version: " FIRMWARE_VERSION);

  pinMode(LED_STATUT, OUTPUT);
  digitalWrite(LED_STATUT, LOW);

  // --- MPU-6050 ---
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("ERREUR: MPU-6050 non détecté !");
    while (1) {
      digitalWrite(LED_STATUT, !digitalRead(LED_STATUT));
      delay(150); // clignotement rapide = erreur capteur
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU-6050 OK");

  // --- WiFi (fallback 2 réseaux) ---
  if (!connecterWiFi()) {
    Serial.println("Aucun WiFi disponible — redémarrage dans 30s");
    delay(30000);
    ESP.restart();
  }

  // --- NTP (nécessaire pour t0 en timestamp Unix) ---
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov"); // Europe/Paris (UTC+1, +DST auto approx)
  Serial.print("Synchronisation NTP");
  time_t maintenant = time(nullptr);
  int tentatives = 0;
  while (maintenant < 100000 && tentatives < 20) {
    delay(500);
    Serial.print(".");
    maintenant = time(nullptr);
    tentatives++;
  }
  Serial.println();

  // --- Récupération de l'état depuis le Sheet (survie à une coupure) ---
  recupererEtatDepuisSheet();

  // --- Si aucun cycle actif, on calibre l'angle de repos ---
  if (!cycle_actif) {
    calibrerAngleRepos();
  } else {
    Serial.println("Cycle déjà en cours (repris depuis le Sheet) — t0=" + String((unsigned long)t0));
  }

  // --- Vérification OTA au démarrage ---
  verifierMiseAJourOTA();

  Serial.println("=== Setup terminé ===\n");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long maintenant_ms = millis();

  // --- LED de statut WiFi (clignotement non-bloquant) ---
  gererLedStatut(maintenant_ms);

  // Vérifier le WiFi, reconnecter si besoin
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdu, reconnexion...");
    connecterWiFi();
  }

  // --- Lecture périodique de l'angle ---
  if (maintenant_ms - derniere_lecture >= INTERVALLE_LECTURE_MS) {
    derniere_lecture = maintenant_ms;
    float angle_actuel = lireAngle();

    if (!cycle_actif) {
      // On surveille l'écart pour détecter un départ de cycle
      float ecart = abs(angle_actuel - angle_repos);
      if (ecart >= SEUIL_DEPART_DEG) {
        demarrerNouveauCycle();
      }
    } else {
      // Cycle en cours : calcul de la progression basée sur le temps
      time_t maintenant_epoch = time(nullptr);
      float temps_ecoule_h = (float)(maintenant_epoch - t0) / 3600.0;
      float progression_pct = (temps_ecoule_h / (float)duree_cycle_h) * 100.0;
      if (progression_pct > 100.0) progression_pct = 100.0;

      // Log périodique (toutes les 5 min) dans l'historique
      if (maintenant_ms - dernier_log >= INTERVALLE_LOG_MS || dernier_log == 0) {
        dernier_log = maintenant_ms;
        envoyerLog(temps_ecoule_h, angle_actuel, progression_pct);
        verifierSeuilsAlerte(temps_ecoule_h, angle_actuel, progression_pct);
      }

      // Fin de cycle : temps écoulé atteint
      if (progression_pct >= 100.0) {
        Serial.println("Cycle terminé (84h atteintes)");
        terminerCycle();
      }
    }
  }

  // --- Vérification périodique : reboot à distance + OTA ---
  if (maintenant_ms - dernier_check >= INTERVALLE_CHECK_MS || dernier_check == 0) {
    dernier_check = maintenant_ms;
    verifierRebootDistance();
    verifierMiseAJourOTA();
  }
}

// ============================================================
// WIFI — FALLBACK ENTRE 2 RÉSEAUX
// ============================================================
bool connecterWiFi() {
  WiFi.mode(WIFI_STA);

  // Tentative réseau 1 (domicile / test)
  WiFi.begin(WIFI_SSID_1, WIFI_PASSWORD_1);
  Serial.println("Tentative réseau 1: " WIFI_SSID_1);
  unsigned long debut = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - debut < 10000) {
    digitalWrite(LED_STATUT, !digitalRead(LED_STATUT)); // clignote pendant la tentative
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnecté sur réseau 1: " WIFI_SSID_1 " — IP: " + WiFi.localIP().toString());
    return true;
  }

  // Échec réseau 1 → tentative réseau 2 (Belin)
  Serial.println("\nRéseau 1 indisponible, tentative réseau 2: " WIFI_SSID_2);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID_2, WIFI_PASSWORD_2);
  debut = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - debut < 10000) {
    digitalWrite(LED_STATUT, !digitalRead(LED_STATUT)); // clignote pendant la tentative
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnecté sur réseau 2: " WIFI_SSID_2 " — IP: " + WiFi.localIP().toString());
    return true;
  }

  Serial.println("\nAucun des deux réseaux disponible !");
  return false;
}

// ============================================================
// LED DE STATUT — clignotement non-bloquant
// Lent (1s)   = WiFi connecté, tout va bien
// Rapide (150ms) = WiFi déconnecté / en reconnexion
// ============================================================
void gererLedStatut(unsigned long maintenant_ms) {
  unsigned long intervalle = (WiFi.status() == WL_CONNECTED) ? BLINK_LENT_MS : BLINK_RAPIDE_MS;

  if (maintenant_ms - dernier_blink_led >= intervalle) {
    dernier_blink_led = maintenant_ms;
    etat_led = !etat_led;
    digitalWrite(LED_STATUT, etat_led);
  }
}

// ============================================================
// LECTURE DE L'ANGLE (MPU-6050)
// ============================================================
float lireAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Angle d'inclinaison calculé à partir de l'accéléromètre (axe X)
  float angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  return angleX;
}

// ============================================================
// CALIBRATION DE L'ANGLE DE REPOS (moyenne de 10 lectures)
// ============================================================
void calibrerAngleRepos() {
  Serial.println("Calibration de l'angle de repos...");
  float somme = 0.0;
  int n = 10;
  for (int i = 0; i < n; i++) {
    somme += lireAngle();
    delay(200);
  }
  angle_repos = somme / n;
  Serial.println("Angle de repos détecté: " + String(angle_repos) + "°");

  // On sauvegarde immédiatement dans le Sheet
  envoyerMiseAJourEtat(0, angle_repos, false, false);
}

// ============================================================
// DÉMARRAGE D'UN NOUVEAU CYCLE
// ============================================================
void demarrerNouveauCycle() {
  t0 = time(nullptr);
  cycle_actif = true;
  cycle_id = String((unsigned long)t0); // identifiant simple basé sur le timestamp

  seuil_25_envoye  = false;
  seuil_50_envoye  = false;
  seuil_75_envoye  = false;
  seuil_100_envoye = false;

  Serial.println("Nouveau cycle détecté ! t0=" + String((unsigned long)t0));
  envoyerMiseAJourEtat((unsigned long)t0, angle_repos, true, false);
}

// ============================================================
// FIN DE CYCLE
// ============================================================
void terminerCycle() {
  cycle_actif = false;
  envoyerMiseAJourEtat(0, angle_repos, false, false);

  // On recalibre l'angle de repos pour le prochain cycle
  calibrerAngleRepos();
}

// ============================================================
// VÉRIFICATION DES SEUILS DE PROGRESSION (25/50/75/100%)
// ============================================================
void verifierSeuilsAlerte(float temps_ecoule_h, float angle, float progression_pct) {
  if (progression_pct >= 25.0 && !seuil_25_envoye) {
    seuil_25_envoye = true;
    envoyerAlerte(temps_ecoule_h, angle, 25);
  }
  if (progression_pct >= 50.0 && !seuil_50_envoye) {
    seuil_50_envoye = true;
    envoyerAlerte(temps_ecoule_h, angle, 50);
  }
  if (progression_pct >= 75.0 && !seuil_75_envoye) {
    seuil_75_envoye = true;
    envoyerAlerte(temps_ecoule_h, angle, 75);
  }
  if (progression_pct >= 100.0 && !seuil_100_envoye) {
    seuil_100_envoye = true;
    envoyerAlerte(temps_ecoule_h, angle, 100);
  }
}

// ============================================================
// COMMUNICATION AVEC LE SHEET (Apps Script)
// ============================================================

// --- GET: récupère l'état courant du cycle ---
void recupererEtatDepuisSheet() {
  HTTPClient http;
  String url = String(GYRO_SCRIPT_URL) + "?type=gyro_status";

  http.begin(url);
  http.setTimeout(30000); // TLS handshake Apps Script peut prendre 15-20s
  int code = http.GET();

  if (code == 200) {
    String payload = http.getString();
    Serial.println("État reçu: " + payload);

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      long t0_recu = doc["t0"] | 0;
      float angle_recu = doc["angle_repos"] | 0.0;
      String actif_str = doc["cycle_actif"] | "NON";
      int duree_recue = doc["duree_cycle_h"] | 84;

      t0 = (time_t)t0_recu;
      angle_repos = angle_recu;
      cycle_actif = (actif_str == "OUI");
      duree_cycle_h = duree_recue;

      Serial.println("État restauré — cycle_actif=" + actif_str + " duree_cycle_h=" + String(duree_cycle_h));
    } else {
      Serial.println("Erreur parsing JSON état: " + String(err.c_str()));
    }
  } else {
    Serial.println("Erreur récupération état, code HTTP: " + String(code));
  }
  http.end();
}

// --- POST: met à jour l'état du cycle dans le Sheet ---
void envoyerMiseAJourEtat(unsigned long t0_val, float angle_repos_val, bool actif, bool reboot_demande) {
  HTTPClient http;
  http.begin(GYRO_SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(30000);

  StaticJsonDocument<256> doc;
  doc["type"] = "gyro_update_state";
  doc["t0"] = t0_val;
  doc["angle_repos"] = angle_repos_val;
  doc["cycle_actif"] = actif ? "OUI" : "NON";
  doc["reboot_demande"] = reboot_demande ? "OUI" : "NON";

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.println("Mise à jour état envoyée, code: " + String(code));
  http.end();
}

// --- POST: log d'un point angle/temps dans l'historique ---
void envoyerLog(float temps_ecoule_h, float angle, float progression_pct) {
  HTTPClient http;
  http.begin(GYRO_SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(30000);

  StaticJsonDocument<256> doc;
  doc["type"] = "gyro_log";
  doc["cycle_id"] = cycle_id;
  doc["temps_ecoule_h"] = temps_ecoule_h;
  doc["angle"] = angle;
  doc["progression_pct"] = progression_pct;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.println("Log envoyé (" + String(progression_pct) + "%), code: " + String(code));
  http.end();
}

// --- POST: envoi d'une alerte email à un seuil ---
void envoyerAlerte(float temps_ecoule_h, float angle, int seuil) {
  HTTPClient http;
  http.begin(GYRO_SCRIPT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(30000);

  StaticJsonDocument<256> doc;
  doc["type"] = "gyro_alert";
  doc["cycle_id"] = cycle_id;
  doc["temps_ecoule_h"] = temps_ecoule_h;
  doc["angle"] = angle;
  doc["progression_pct"] = seuil;

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  Serial.println("Alerte " + String(seuil) + "% envoyée, code: " + String(code));
  http.end();
}

// ============================================================
// REBOOT À DISTANCE
// ============================================================
void verifierRebootDistance() {
  HTTPClient http;
  String url = String(GYRO_SCRIPT_URL) + "?type=gyro_status";
  http.begin(url);
  http.setTimeout(30000);
  int code = http.GET();

  if (code == 200) {
    String payload = http.getString();
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, payload)) {
      String reboot_str = doc["reboot_demande"] | "NON";
      if (reboot_str == "OUI") {
        Serial.println("Reboot à distance demandé — redémarrage...");
        http.end();
        // On réinitialise le flag avant de redémarrer
        envoyerMiseAJourEtat((unsigned long)t0, angle_repos, cycle_actif, false);
        delay(1000);
        ESP.restart();
      }
    }
  }
  http.end();
}

// ============================================================
// OTA — VÉRIFICATION ET MISE À JOUR DEPUIS GITHUB
// ============================================================
void verifierMiseAJourOTA() {
  WiFiClientSecure client;
  client.setInsecure(); // simplification — pas de vérification de certificat

  HTTPClient http;
  http.begin(client, GITHUB_VERSION_URL);
  int code = http.GET();

  if (code == 200) {
    String version_distante = http.getString();
    version_distante.trim();
    Serial.println("Version distante: " + version_distante + " / locale: " FIRMWARE_VERSION);

    if (version_distante != FIRMWARE_VERSION) {
      Serial.println("Nouvelle version détectée, téléchargement du firmware...");
      http.end();
      lancerOTA();
      return;
    }
  } else {
    Serial.println("Erreur vérification version OTA, code: " + String(code));
  }
  http.end();
}

void lancerOTA() {
  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.setLedPin(LED_STATUT, LOW);
  t_httpUpdate_return resultat = httpUpdate.update(client, GITHUB_BIN_URL);

  switch (resultat) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("Échec OTA: (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("Pas de mise à jour disponible");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("Mise à jour OK, redémarrage...");
      break;
  }
}
