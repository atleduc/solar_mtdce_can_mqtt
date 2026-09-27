// ============================================================================
// Passerelle CAN -> MQTT pour SOREL MTDC-E
//
// Perimetre : lecture passive du bus CAN du MTDC-E via un module MCP2515,
// publication des trames (brutes et/ou decodees) vers un broker MQTT.
// Le programme n'emet aucune trame : il n'y a aucun appel d'envoi dans le
// code. Le MCP2515 tourne en mode normal pour fournir au MTDC-E le bit
// d'acquittement dont il a besoin - sur un bus a deux noeuds, l'ecoute seule
// laisse le regulateur sans acquittement et le fait passer en bus-off.
// Voir CAN_ECOUTE_SEULE dans config.h.
//
// Le MTDC-E est un regulateur differentiel SOREL. Son bus CAN utilise le
// protocole SCBI (SOREL CAN Bus Interface) a 250 kbit/s, decrit dans
// decodage.cpp a partir des sources listees dans le README.
//
// CABLAGE (ESP32 <-> module MCP2515)
//   GPIO18 : SCK
//   GPIO19 : SO   (MISO du module)
//   GPIO23 : SI   (MOSI du module)
//   GPIO5  : CS
//   GPIO4  : INT
//   VCC    : 5 V (VIN) - le TJA1050 exige 4,75 V minimum. Les deux lignes
//            qui remontent vers l'ESP32, SO et INT, passent alors par un
//            pont diviseur 1k/2k. Voir README.
//   GND    : masse commune avec le MTDC-E
//
//   Cote bus : CANH et CANL du module vers CANH / CANL du MTDC-E.
//   Le MTDC-E a deux ports CAN : placer un bouchon 120 ohms dans le port
//   reste libre, sinon rien n'est decode sur le bus.
//
// FONCTIONNEMENT
//   - Detection automatique du debit CAN au demarrage (125/250/500/1000 k).
//   - Une tache dediee vide en continu les tampons du MCP2515 vers une file.
//   - La boucle principale publie : trames brutes, signaux decodes, inventaire
//     des identifiants observes, statistiques.
//   - Topic <base>/cmd pour piloter la passerelle a chaud.
//
// Reglages : voir config.h    Mapping des trames : voir decodage.cpp
// ============================================================================

#include <SPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_system.h>
#include <mcp_can.h>

#include "config.h"
#include "decodage.h"

// ---------------------------------------------------------------------------
// Objets globaux
// ---------------------------------------------------------------------------
MCP_CAN       CAN(PIN_CAN_CS);
WiFiClient    reseau;
PubSubClient  mqtt(reseau);

// Une trame telle que recue du MCP2515.
struct TrameCan {
  uint32_t id;
  uint32_t horodatage;
  uint8_t  data[8];
  uint8_t  dlc;
  bool     etendu;
  bool     rtr;
};

// Suivi d'une "voie" du bus : sert a l'anti-flood et a l'inventaire.
//
// La cle n'est pas seulement l'identifiant CAN mais le couple
// (identifiant, premier octet de donnees). Le protocole SCBI est fortement
// multiplexe : un meme identifiant transporte plusieurs grandeurs, designees
// par un index en tete de trame. Sans le premier octet dans la cle, les voies
// s'ecrasent mutuellement sur le meme topic et on ne voit que la derniere -
// ce qui rendait le groupe 0x07 impossible a cartographier.
struct SuiviId {
  bool     utilise;
  uint32_t id;
  bool     etendu;
  uint8_t  mux[2];     // octets de tete servant d'index
  uint8_t  muxLen;     // 0 si la trame n'a pas de donnees
  uint32_t compte;
  uint32_t dernierVu;
  uint32_t periodeMs;
  uint8_t  data[8];
  uint8_t  dlc;
  uint32_t dernierePub;
  uint8_t  dataPub[8];
  uint8_t  dlcPub;
  bool     dejaPub;
  uint32_t comptePub;  // valeur de compte lors du dernier inventaire publie
};

static SuiviId  suivis[MAX_IDS_SUIVIS];
static uint16_t nbSuivis = 0;
static uint32_t nbVoiesPerdues = 0;   // voies vues mais non suivies, table pleine

static QueueHandle_t fileTrames = nullptr;
static TaskHandle_t  tacheCanHandle = nullptr;

static volatile uint32_t nbRecues = 0;
static volatile uint32_t nbPerdues = 0;

// Registres d'erreur du MCP2515, relus par la tache CAN. Ils ne peuvent pas
// etre lus depuis loop() : la bibliotheque mcp_can n'est pas reentrante et
// deux taches emettant des transactions SPI concurrentes se corrompraient.
static volatile uint8_t canEflg = 0;   // registre EFLG
static volatile uint8_t canRec  = 0;   // compteur d'erreurs en reception
static volatile uint8_t canTec  = 0;   // compteur d'erreurs en emission

// Resultat rendu par setMode(), qui relit CANSTAT pour confirmer que la puce a
// reellement pris le mode demande. MCP2515_OK = confirme par le materiel.
static volatile uint8_t canModeRes = MCP2515_FAIL;
static uint32_t nbPubliees = 0;
static uint32_t recuesPrecedent = 0;

// Liste des reseaux WiFi, essayes dans l'ordre. Le repli garantit qu'un
// changement de SSID ne rende jamais la carte injoignable en OTA.
// Declaree ici, avec les globales : les prototypes que l'IDE Arduino genere
// automatiquement en tete de fichier referencent ce type, il doit donc etre
// connu avant toute fonction qui l'utilise.
struct Reseau { const char* ssid; const char* pass; };

static const Reseau RESEAUX[] = {
  { WIFI_SSID,   WIFI_PASS   },
  { WIFI_SSID_2, WIFI_PASS_2 },
};
static const uint8_t NB_RESEAUX = sizeof(RESEAUX) / sizeof(RESEAUX[0]);

// Supervision d'une installation laissee seule : sans ces compteurs, une nuit
// de coupures ne laisse aucune trace. uptime_s dit seulement qu'il n'y a pas
// eu de redemarrage, pas combien de fois la liaison est tombee.
static uint32_t nbConnexionsWifi = 0;  // liaisons WiFi etablies depuis le boot
static uint32_t nbConnexionsMqtt = 0;  // sessions MQTT etablies depuis le boot
static uint32_t nbDemarrages     = 0;  // compteur persistant, survit aux coupures

static const char* ssidActif  = "";   // reseau WiFi effectivement joint
static const char* debitLabel = "?";
static uint8_t     debitCode  = CAN_DEBIT_FIXE;
static bool        canPret    = false;

static bool publierBrut  = PUBLIER_BRUT;
static bool demandeScan  = false;
static bool demandeReset = false;

static uint32_t tStats = 0;
static uint32_t tDecouverte = 0;
static uint32_t tReconnexionMqtt = 0;

// Topics construits une fois pour toutes.
static char topicEtat[96];
static char topicCmd[96];

// ---------------------------------------------------------------------------
// Outils
// ---------------------------------------------------------------------------

// Formate un flottant sans zeros inutiles ("12.300" -> "12.3", "5.000" -> "5").
static void formaterNombre(float v, char* dest, size_t taille) {
  snprintf(dest, taille, "%.3f", (double)v);
  char* point = strchr(dest, '.');
  if (!point) return;
  char* fin = dest + strlen(dest) - 1;
  while (fin > point && *fin == '0') *fin-- = '\0';
  if (fin == point) *fin = '\0';
}

// Cause du dernier redemarrage, telle que la retient le silicium. C'est elle
// qui distingue une coupure secteur d'un plantage logiciel ou d'un reset
// volontaire - trois evenements qui se ressemblent vus de loin.
static const char* raisonReset() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "mise sous tension";
    case ESP_RST_EXT:       return "reset externe";
    case ESP_RST_SW:        return "reset logiciel";     // OTA, commande reboot
    case ESP_RST_PANIC:     return "panique logicielle";
    case ESP_RST_INT_WDT:   return "chien de garde interruptions";
    case ESP_RST_TASK_WDT:  return "chien de garde tache";
    case ESP_RST_WDT:       return "chien de garde";
    case ESP_RST_DEEPSLEEP: return "sortie de veille";
    case ESP_RST_BROWNOUT:  return "brownout";           // alimentation faible
    case ESP_RST_SDIO:      return "reset SDIO";
    default:                return "inconnue";
  }
}

static void identifiantTexte(uint32_t id, bool etendu, char* dest, size_t taille) {
  if (etendu) snprintf(dest, taille, "0x%08lX", (unsigned long)id);
  else        snprintf(dest, taille, "0x%03lX", (unsigned long)id);
}

static void donneesTexte(const uint8_t* data, uint8_t dlc, char* dest, size_t taille) {
  dest[0] = '\0';
  for (uint8_t i = 0; i < dlc; i++) {
    size_t n = strlen(dest);
    if (n + 4 >= taille) break;
    snprintf(dest + n, taille - n, i ? " %02X" : "%02X", data[i]);
  }
}

// Traduit le registre EFLG du MCP2515 en liste de drapeaux lisibles, restreinte
// aux bits de "masque".
//   EWARN/RXWAR/TXWAR : compteurs d'erreurs au-dessus de 96
//   RXEP/TXEP         : passif en erreur (au-dessus de 127)
//   BUSOFF            : le controleur s'est retire du bus
//   RXxOVR            : tampon de reception deborde
//
// Les deux familles sont publiees separement car elles ne se comportent pas
// pareil : les bits d'erreur suivent les compteurs et redescendent d'eux-memes,
// alors que les bits de debordement restent colles jusqu'a une remise a zero du
// composant. Seul un redemarrage les efface durablement : la commande "scan"
// reinitialise bien le MCP2515, mais suspend la tache CAN pendant la detection
// de debit, ce qui provoque aussitot un nouveau debordement. En fonctionnement
// normal, la reception par interruption ne doit en produire aucun.
#define EFLG_ERREURS      0x3F
#define EFLG_DEBORDEMENTS 0xC0

static void decrireEflg(uint8_t e, uint8_t masque, char* dest, size_t taille) {
  e &= masque;
  static const struct { uint8_t bit; const char* nom; } DRAPEAUX[] = {
    { 0x01, "EWARN"  }, { 0x02, "RXWAR"  }, { 0x04, "TXWAR"  },
    { 0x08, "RXEP"   }, { 0x10, "TXEP"   }, { 0x20, "BUSOFF" },
    { 0x40, "RX0OVR" }, { 0x80, "RX1OVR" },
  };
  dest[0] = '\0';
  for (uint8_t i = 0; i < 8; i++) {
    if (!(e & DRAPEAUX[i].bit)) continue;
    size_t n = strlen(dest);
    if (n + strlen(DRAPEAUX[i].nom) + 2 >= taille) break;
    snprintf(dest + n, taille - n, n ? " %s" : "%s", DRAPEAUX[i].nom);
  }
  if (!dest[0]) snprintf(dest, taille, "aucune");
}

// ---------------------------------------------------------------------------
// Bus CAN
// ---------------------------------------------------------------------------

struct Debit { uint8_t code; const char* label; };

// Ordre d'essai : du plus probable au moins probable sur ce type d'appareil.
static const Debit DEBITS[] = {
  { CAN_250KBPS,  "250k" },
  { CAN_500KBPS,  "500k" },
  { CAN_125KBPS,  "125k" },
  { CAN_1000KBPS, "1M"   },
};
static const uint8_t NB_DEBITS = sizeof(DEBITS) / sizeof(DEBITS[0]);

// Mode d'exploitation voulu pour le MCP2515.
static inline uint8_t modeVoulu() {
  return CAN_ECOUTE_SEULE ? MCP_LISTENONLY : MCP_NORMAL;
}

// Ouvre le controleur a un debit donne.
//
// Piege de la bibliotheque mcp_can : begin() laisse la puce en LOOPBACK, ou
// elle ne recoit que ses propres emissions - donc zero trame du bus ET zero
// erreur. Il faut imposer le mode voulu ensuite, et verifier qu'il a bien ete
// pris : setMode() relit CANSTAT et ne rend MCP2515_OK que si les bits de mode
// correspondent effectivement.
//
// La valeur de retour ne reflete que l'acces SPI (begin), pour que la
// passerelle continue a publier son diagnostic meme si le mode a echoue.
static bool ouvrirCan(uint8_t code) {
  canModeRes = MCP2515_FAIL;
  if (CAN.begin(MCP_ANY, code, CAN_QUARTZ) != CAN_OK) return false;

  uint8_t res = CAN.setMode(modeVoulu());
  if (res != MCP2515_OK) {
    delay(20);
    res = CAN.setMode(modeVoulu());
  }
  canModeRes = res;

  if (res != MCP2515_OK) {
    Serial.println(F("[CAN] Mode refuse par le MCP2515 (reste en loopback ?)"));
  }
  delay(10);
  return true;
}

// Compte les trames valides recues pendant une fenetre donnee.
static uint16_t compterTrames(uint32_t dureeMs) {
  uint16_t n = 0;
  uint32_t fin = millis() + dureeMs;
  unsigned long id;
  byte ext, len, buf[8];
  while ((int32_t)(fin - millis()) > 0) {
    if (digitalRead(PIN_CAN_INT) == LOW) {
      if (CAN.readMsgBuf(&id, &ext, &len, buf) == CAN_OK) n++;
    } else {
      delay(1);
    }
  }
  return n;
}

// Initialise le bus, avec detection automatique du debit si activee.
static bool initialiserCan() {
  canPret = false;

  if (CAN_AUTO_DEBIT) {
    Serial.println(F("[CAN] Detection du debit..."));
    for (uint8_t i = 0; i < NB_DEBITS; i++) {
      if (!ouvrirCan(DEBITS[i].code)) {
        Serial.println(F("[CAN] MCP2515 injoignable (cablage / quartz ?)"));
        return false;
      }
      uint16_t n = compterTrames(1200);
      Serial.printf("[CAN]   %-5s : %u trame(s)\n", DEBITS[i].label, n);
      if (n >= 2) {
        debitCode  = DEBITS[i].code;
        debitLabel = DEBITS[i].label;
        canPret    = true;
        Serial.printf("[CAN] Debit retenu : %s\n", debitLabel);
        return true;
      }
    }
    Serial.println(F("[CAN] Aucun trafic detecte, repli sur le debit fixe."));
  }

  debitCode = CAN_DEBIT_FIXE;
  for (uint8_t i = 0; i < NB_DEBITS; i++) {
    if (DEBITS[i].code == debitCode) debitLabel = DEBITS[i].label;
  }
  canPret = ouvrirCan(debitCode);
  if (!canPret) Serial.println(F("[CAN] Echec d'initialisation du MCP2515."));
  return canPret;
}

// Reveille la tache CAN des que le MCP2515 signale une trame recue.
//
// Un simple vTaskDelay entre deux relevés ne suffit pas : le MCP2515 n'a que
// deux tampons de reception, et le SCBI emet par rafales de trames
// consecutives. A 250 kbit/s elles se suivent a environ 0,5 ms, donc une
// rafale de trois trames deborde les tampons avant le reveil de la tache -
// c'est ce que signalait le drapeau RX0OVR. Avec l'interruption, la latence
// tombe a quelques microsecondes.
static void IRAM_ATTR interruptionCan() {
  if (tacheCanHandle) vTaskNotifyGiveFromISR(tacheCanHandle, NULL);
}

// Tache dediee : vide les tampons du MCP2515 le plus vite possible.
// Sans elle, une publication MQTT un peu lente ferait deborder les deux
// tampons de reception du controleur et perdre des trames.
static void tacheCan(void* arg) {
  (void)arg;
  unsigned long id;
  byte ext, len, buf[8];
  uint32_t dernierEtat = 0;

  for (;;) {
    // Releve periodique des registres d'erreur, dans cette tache car c'est la
    // seule qui a le droit de parler au MCP2515 une fois le systeme demarre.
    uint32_t maintenant = millis();
    if (maintenant - dernierEtat >= 1000) {
      dernierEtat = maintenant;
      canEflg = CAN.getError();
      canRec  = CAN.errorCountRX();
      canTec  = CAN.errorCountTX();

      // Si le mode n'a jamais ete confirme par le materiel, on reessaie : sans
      // cela la puce resterait muette indefiniment, sans aucune erreur visible.
      if (canModeRes != MCP2515_OK) {
        canModeRes = CAN.setMode(modeVoulu());
      }
    }

    while (digitalRead(PIN_CAN_INT) == LOW) {
      if (CAN.readMsgBuf(&id, &ext, &len, buf) != CAN_OK) break;

      TrameCan t;
      t.id         = (uint32_t)(id & 0x1FFFFFFFUL);
      t.etendu     = (ext != 0) || ((id & 0x80000000UL) != 0);
      t.rtr        = (id & 0x40000000UL) != 0;
      t.dlc        = len > 8 ? 8 : len;
      t.horodatage = millis();
      memcpy(t.data, buf, t.dlc);
      if (t.dlc < 8) memset(t.data + t.dlc, 0, 8 - t.dlc);

      nbRecues++;
      if (xQueueSend(fileTrames, &t, 0) != pdTRUE) nbPerdues++;
    }

    // Attente de la prochaine interruption. Le delai d'attente garantit que le
    // releve des registres continue meme si INT reste haut, et sert de filet
    // si INT se bloque bas sur une interruption qui n'est pas une reception.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
  }
}

// ---------------------------------------------------------------------------
// Suivi des identifiants
// ---------------------------------------------------------------------------

static SuiviId* suivre(const TrameCan& t) {
  uint8_t muxLen = longueurMux(t.id, t.etendu);
  if (muxLen > t.dlc) muxLen = t.dlc;   // trame trop courte, voire vide

  for (uint16_t i = 0; i < nbSuivis; i++) {
    SuiviId& s = suivis[i];
    if (s.id != t.id || s.etendu != t.etendu || s.muxLen != muxLen) continue;
    if (memcmp(s.mux, t.data, muxLen) != 0) continue;
    return &s;
  }
  if (nbSuivis >= MAX_IDS_SUIVIS) {
    nbVoiesPerdues++;
    return nullptr;
  }

  SuiviId* s = &suivis[nbSuivis++];
  memset(s, 0, sizeof(SuiviId));
  s->utilise = true;
  s->id      = t.id;
  s->etendu  = t.etendu;
  s->muxLen  = muxLen;
  memcpy(s->mux, t.data, muxLen);
  return s;
}

// Construit le suffixe de topic d'une voie : "<id>/<index en hexa>", ou "<id>"
// seul pour une trame sans donnees. L'index concatene les octets de tete :
// "0x10011A80/00" pour un capteur, "0x10071A80/2101" pour le groupe 0x07.
static void voieTexte(const SuiviId& s, char* dest, size_t taille) {
  char idTxt[16];
  identifiantTexte(s.id, s.etendu, idTxt, sizeof(idTxt));
  if (s.muxLen == 0) {
    snprintf(dest, taille, "%s", idTxt);
    return;
  }
  size_t n = snprintf(dest, taille, "%s/", idTxt);
  for (uint8_t i = 0; i < s.muxLen && n + 3 < taille; i++) {
    n += snprintf(dest + n, taille - n, "%02X", s.mux[i]);
  }
}

// Index de multiplexage seul, pour la charge utile JSON : "0x2101", ou "" si
// la trame n'a pas de donnees.
static void muxTexte(const SuiviId& s, char* dest, size_t taille) {
  if (s.muxLen == 0) { dest[0] = '\0'; return; }
  size_t n = snprintf(dest, taille, "0x");
  for (uint8_t i = 0; i < s.muxLen && n + 3 < taille; i++) {
    n += snprintf(dest + n, taille - n, "%02X", s.mux[i]);
  }
}

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------

static void publierTexte(const char* topic, const char* charge, bool retain) {
  if (mqtt.publish(topic, charge, retain)) nbPubliees++;
}

static void publierDecouverteHA();

static bool connecterMqtt() {
  char clientId[48];
  snprintf(clientId, sizeof(clientId), "%s-%06lX",
           NOM_HOTE, (unsigned long)(ESP.getEfuseMac() & 0xFFFFFFULL));

  bool ok;
  if (strlen(MQTT_UTILISATEUR) > 0) {
    ok = mqtt.connect(clientId, MQTT_UTILISATEUR, MQTT_MOTDEPASSE,
                      topicEtat, 0, true, "offline");
  } else {
    ok = mqtt.connect(clientId, NULL, NULL,
                      topicEtat, 0, true, "offline");
  }
  if (!ok) {
    Serial.printf("[MQTT] Echec de connexion (etat %d)\n", mqtt.state());
    return false;
  }

  Serial.println(F("[MQTT] Connecte."));
  nbConnexionsMqtt++;
  publierTexte(topicEtat, "online", true);
  mqtt.subscribe(topicCmd);
  publierDecouverteHA();
  return true;
}

// Publie une trame brute, avec anti-flood : republication seulement si la
// donnee a change, ou au bout de INTERVALLE_MAX_MS.
static void publierTrameBrute(const TrameCan& t, SuiviId* s) {
  if (!publierBrut || !s) return;

  uint32_t maintenant = millis();
  bool changee = (s->dlcPub != t.dlc) || memcmp(s->dataPub, t.data, t.dlc) != 0;
  bool tropTot = s->dejaPub && (maintenant - s->dernierePub) < INTERVALLE_MIN_MS;
  bool perime  = !s->dejaPub || (maintenant - s->dernierePub) >= INTERVALLE_MAX_MS;

  if (tropTot) return;
  if (!changee && !perime) return;

  char idTxt[16], voie[24], muxTxt[12], dataTxt[32], topic[128], charge[256];
  identifiantTexte(t.id, t.etendu, idTxt, sizeof(idTxt));
  voieTexte(*s, voie, sizeof(voie));
  muxTexte(*s, muxTxt, sizeof(muxTxt));
  donneesTexte(t.data, t.dlc, dataTxt, sizeof(dataTxt));

  snprintf(topic, sizeof(topic), MQTT_BASE "/brut/%s", voie);
  snprintf(charge, sizeof(charge),
           "{\"id\":\"%s\",\"mux\":\"%s\",\"etendu\":%s,\"rtr\":%s,"
           "\"dlc\":%u,\"data\":\"%s\",\"periode_ms\":%lu,\"compte\":%lu}",
           idTxt, muxTxt, t.etendu ? "true" : "false", t.rtr ? "true" : "false",
           t.dlc, dataTxt, (unsigned long)s->periodeMs, (unsigned long)s->compte);

  publierTexte(topic, charge, true);

  memcpy(s->dataPub, t.data, 8);
  s->dlcPub      = t.dlc;
  s->dernierePub = maintenant;
  s->dejaPub     = true;
}

// Publie les signaux decodes de la trame. Plusieurs entrees de la table
// peuvent partager le meme identifiant CAN et se distinguer par leur prefixe
// (multiplexage), d'ou le parcours complet de la table.
static void publierSignaux(const TrameCan& t) {
#if DECODAGE_ACTIF
  for (uint8_t k = 0; k < NB_TRAMES; k++) {
    const Trame& d = TRAMES[k];
    if (!trameCorrespond(d, t.id, t.etendu, t.data, t.dlc)) continue;

    for (uint8_t i = 0; i < d.nbSignaux; i++) {
      const Signal& s = d.signaux[i];
      float valeur;
      if (!extraireSignal(s, t.data, t.dlc, valeur)) continue;

      char topic[160], charge[24];
      snprintf(topic, sizeof(topic), MQTT_BASE "/valeur/%s/%s", d.nom, s.nom);
      formaterNombre(valeur, charge, sizeof(charge));
      publierTexte(topic, charge, true);
    }
  }
#else
  (void)t;
#endif
}

// Inventaire des identifiants observes : un topic retenu par identifiant.
// C'est le point de depart pour identifier le protocole du MTDC-E.
// tout = true republie toutes les voies (commande "decouverte"). Sinon seules
// les voies ayant recu de nouvelles trames sont republiees : avec plusieurs
// centaines de voies, republier l'inventaire complet toutes les 15 s
// saturerait inutilement le broker, et les topics sont deja retenus.
static void publierDecouverte(bool tout) {
  char idTxt[16], voie[24], muxTxt[12], dataTxt[32], topic[128], charge[256];
  uint16_t publiees = 0;

  for (uint16_t i = 0; i < nbSuivis; i++) {
    SuiviId& s = suivis[i];
    if (!tout && s.compte == s.comptePub) continue;

    identifiantTexte(s.id, s.etendu, idTxt, sizeof(idTxt));
    voieTexte(s, voie, sizeof(voie));
    muxTexte(s, muxTxt, sizeof(muxTxt));
    donneesTexte(s.data, s.dlc, dataTxt, sizeof(dataTxt));

    snprintf(topic, sizeof(topic), MQTT_BASE "/decouverte/%s", voie);
    snprintf(charge, sizeof(charge),
             "{\"id\":\"%s\",\"mux\":\"%s\",\"etendu\":%s,\"dlc\":%u,"
             "\"compte\":%lu,\"periode_ms\":%lu,\"dernier\":\"%s\","
             "\"connu\":%s}",
             idTxt, muxTxt, s.etendu ? "true" : "false", s.dlc,
             (unsigned long)s.compte, (unsigned long)s.periodeMs, dataTxt,
             identifiantConnu(s.id, s.etendu) ? "true" : "false");

    publierTexte(topic, charge, true);
    s.comptePub = s.compte;
    publiees++;
  }

  snprintf(charge, sizeof(charge),
           "{\"voies\":%u,\"max\":%u,\"voies_perdues\":%lu,\"publiees\":%u,"
           "\"debit\":\"%s\"}",
           nbSuivis, (unsigned)MAX_IDS_SUIVIS, (unsigned long)nbVoiesPerdues,
           publiees, debitLabel);
  publierTexte(MQTT_BASE "/decouverte", charge, true);
}

static void publierStats() {
  uint32_t recues = nbRecues;
  uint32_t parSeconde = (recues - recuesPrecedent) * 1000UL / INTERVALLE_STATS_MS;
  recuesPrecedent = recues;

  char errTxt[64], ovrTxt[32];
  uint8_t eflg = canEflg;
  decrireEflg(eflg, EFLG_ERREURS, errTxt, sizeof(errTxt));
  decrireEflg(eflg, EFLG_DEBORDEMENTS, ovrTxt, sizeof(ovrTxt));

  char charge[672];
  snprintf(charge, sizeof(charge),
           "{\"uptime_s\":%lu,\"rssi\":%d,\"ssid\":\"%s\",\"ip\":\"%s\","
           "\"debit\":\"%s\","
           "\"mode\":\"%s\",\"mode_ok\":%s,\"can_ok\":%s,"
           "\"trames\":%lu,\"trames_s\":%lu,"
           "\"perdues\":%lu,\"voies\":%u,\"publications\":%lu,"
           "\"brut\":%s,\"ram\":%lu,"
           "\"eflg\":\"0x%02X\",\"erreurs\":\"%s\",\"debordements\":\"%s\","
           "\"rec\":%u,\"tec\":%u,"
           "\"connexions_wifi\":%lu,\"connexions_mqtt\":%lu,"
           "\"demarrages\":%lu,\"raison_reset\":\"%s\"}",
           (unsigned long)(millis() / 1000UL), (int)WiFi.RSSI(),
           ssidActif, WiFi.localIP().toString().c_str(), debitLabel,
           CAN_ECOUTE_SEULE ? "ecoute seule" : "normal",
           (canModeRes == MCP2515_OK) ? "true" : "false",
           canPret ? "true" : "false",
           (unsigned long)recues, (unsigned long)parSeconde,
           (unsigned long)nbPerdues, nbSuivis, (unsigned long)nbPubliees,
           publierBrut ? "true" : "false",
           (unsigned long)ESP.getFreeHeap(),
           eflg, errTxt, ovrTxt, (unsigned)canRec, (unsigned)canTec,
           (unsigned long)nbConnexionsWifi, (unsigned long)nbConnexionsMqtt,
           (unsigned long)nbDemarrages, raisonReset());

  publierTexte(MQTT_BASE "/etat/stats", charge, false);
}

// Declaration des capteurs aupres de Home Assistant (optionnel).
static void publierDecouverteHA() {
#if DECODAGE_ACTIF && HA_DISCOVERY
  char topic[192], charge[700], extra[128];
  for (uint8_t i = 0; i < NB_TRAMES; i++) {
    const Trame& d = TRAMES[i];
    for (uint8_t j = 0; j < d.nbSignaux; j++) {
      const Signal& s = d.signaux[j];

      // Unite et classe ne sont declarees que si elles sont connues : Home
      // Assistant refuse une device_class vide.
      extra[0] = '\0';
      if (s.unite[0]) {
        snprintf(extra, sizeof(extra), "\"unit_of_meas\":\"%s\",", s.unite);
      }
      if (s.classeHA[0]) {
        size_t n = strlen(extra);
        snprintf(extra + n, sizeof(extra) - n,
                 "\"dev_cla\":\"%s\",\"stat_cla\":\"measurement\",", s.classeHA);
      }

      snprintf(topic, sizeof(topic), HA_PREFIXE "/sensor/%s_%s_%s/config",
               NOM_HOTE, d.nom, s.nom);
      snprintf(charge, sizeof(charge),
               "{\"name\":\"%s %s\",\"uniq_id\":\"%s_%s_%s\","
               "\"stat_t\":\"" MQTT_BASE "/valeur/%s/%s\",%s"
               "\"avty_t\":\"%s\",\"pl_avail\":\"online\","
               "\"pl_not_avail\":\"offline\","
               "\"dev\":{\"ids\":[\"%s\"],\"name\":\"MTDC-E\","
               "\"mf\":\"SOREL\",\"mdl\":\"MTDC-E\"}}",
               d.nom, s.nom, NOM_HOTE, d.nom, s.nom, d.nom, s.nom,
               extra, topicEtat, NOM_HOTE);
      publierTexte(topic, charge, true);
    }
  }
#endif
}

// ---------------------------------------------------------------------------
// Commandes recues sur <base>/cmd
//   brut on | brut off | decouverte | reset | scan | reboot
// ---------------------------------------------------------------------------
static void surMessage(char* topic, byte* charge, unsigned int longueur) {
  (void)topic;
  char cmd[32];
  unsigned int n = longueur < sizeof(cmd) - 1 ? longueur : sizeof(cmd) - 1;
  memcpy(cmd, charge, n);
  cmd[n] = '\0';
  for (char* p = cmd; *p; p++) *p = tolower((unsigned char)*p);

  Serial.printf("[CMD] %s\n", cmd);

  if (!strcmp(cmd, "brut on") || !strcmp(cmd, "brut 1")) {
    publierBrut = true;
  } else if (!strcmp(cmd, "brut off") || !strcmp(cmd, "brut 0")) {
    publierBrut = false;
  } else if (!strcmp(cmd, "decouverte")) {
    publierDecouverte(true);
  } else if (!strcmp(cmd, "reset")) {
    demandeReset = true;
  } else if (!strcmp(cmd, "scan")) {
    demandeScan = true;
  } else if (!strcmp(cmd, "reboot")) {
    publierTexte(topicEtat, "offline", true);
    delay(200);
    ESP.restart();
  } else {
    publierTexte(MQTT_BASE "/etat/erreur", "commande inconnue", false);
  }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------


static bool essayerReseau(const Reseau& r) {
  if (!r.ssid || !r.ssid[0]) return false;

  Serial.printf("[WiFi] Essai de %s", r.ssid);
  WiFi.begin(r.ssid, r.pass);
  uint32_t debut = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - debut < WIFI_ESSAI_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    return false;
  }
  ssidActif = r.ssid;
  Serial.printf("[WiFi] Connecte a %s, IP %s, RSSI %d dBm\n",
                r.ssid, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  return true;
}

static void connecterWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NOM_HOTE);
  WiFi.setSleep(false);              // evite les latences sur le trafic MQTT
  WiFi.setAutoReconnect(true);

  for (uint8_t i = 0; i < NB_RESEAUX; i++) {
    if (essayerReseau(RESEAUX[i])) return;
  }
  ssidActif = "";
  Serial.println(F("[WiFi] Aucun reseau joignable, nouvel essai plus tard."));
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Passerelle CAN -> MQTT MTDC-E ==="));

  // Compteur de demarrages en memoire non volatile : c'est le seul temoin qui
  // survit a une coupure secteur, la ou uptime_s repart de zero.
  {
    Preferences prefs;
    if (prefs.begin("mtdce", false)) {
      nbDemarrages = prefs.getUInt("demarrages", 0) + 1;
      prefs.putUInt("demarrages", nbDemarrages);
      prefs.end();
    }
  }
  Serial.printf("[BOOT] Demarrage n%lu, cause : %s\n",
                (unsigned long)nbDemarrages, raisonReset());

  snprintf(topicEtat, sizeof(topicEtat), "%s", MQTT_BASE "/etat");
  snprintf(topicCmd,  sizeof(topicCmd),  "%s", MQTT_BASE "/cmd");

  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);

  if (!initialiserCan()) {
    Serial.println(F("[CAN] Demarrage degrade : verifier CS/INT et le quartz."));
  }
  Serial.printf("[CAN] Mode %s, debit %s\n",
                CAN_ECOUTE_SEULE ? "ecoute seule" : "normal", debitLabel);

  fileTrames = xQueueCreate(TAILLE_FILE, sizeof(TrameCan));
  xTaskCreatePinnedToCore(tacheCan, "can", 4096, NULL, 3, &tacheCanHandle, 0);
  // Apres la creation de la tache : l'ISR a besoin de son handle.
  attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), interruptionCan, FALLING);

  connecterWifi();

  ArduinoOTA.setHostname(NOM_HOTE);
  // La tache CAN monopolise le SPI et tourne en priorite 3 : on la met en
  // pause pendant l'ecriture de la flash. Il faut imperativement la relancer
  // si la mise a jour echoue, sinon la passerelle reste en vie mais aveugle
  // au bus jusqu'au prochain redemarrage.
  ArduinoOTA.onStart([]() { vTaskSuspend(tacheCanHandle); });
  ArduinoOTA.onError([](ota_error_t) { vTaskResume(tacheCanHandle); });
  ArduinoOTA.onEnd([]() { vTaskResume(tacheCanHandle); });
  ArduinoOTA.begin();

  mqtt.setServer(MQTT_HOTE, MQTT_PORT);
  mqtt.setCallback(surMessage);
  mqtt.setKeepAlive(30);
  mqtt.setBufferSize(1024);

  tStats = tDecouverte = millis();
}

void loop() {
  ArduinoOTA.handle();

  // La file est vidée quel que soit l'etat du reseau. Sortir de loop() tant
  // que le WiFi est absent - ce que faisait la version precedente - laissait
  // la file se remplir en deux minutes et perdre des trames en silence : c'est
  // ce qui expliquait un compteur "perdues" a plusieurs dizaines de milliers
  // apres une nuit de liaison WiFi instable. Seule la publication est
  // conditionnee au reseau ; le suivi des voies, lui, continue.
  const bool reseauOk = (WiFi.status() == WL_CONNECTED);

  // Comptage des liaisons WiFi etablies. Le comptage se fait sur la transition
  // et non dans connecterWifi() : la reconnexion automatique de l'ESP32 remet
  // souvent le lien sans passer par notre code, et ces retours-la comptent
  // autant que les autres.
  static bool wifiPrecedent = false;
  if (reseauOk && !wifiPrecedent) nbConnexionsWifi++;
  wifiPrecedent = reseauOk;

  // Reprise complete de la recherche apres une perte durable : la reconnexion
  // automatique de l'ESP32 ne retente que le dernier SSID connu, donc elle ne
  // suffit pas si c'est ce reseau-la qui a disparu. L'appel bloque au pire
  // quelques dizaines de secondes, pendant lesquelles la file - 256 places
  // pour deux trames par seconde - a largement de quoi tenir.
  static uint32_t tPerteWifi = 0;
  if (!reseauOk) {
    if (tPerteWifi == 0) {
      tPerteWifi = millis();
    } else if (millis() - tPerteWifi > WIFI_RELANCE_MS) {
      tPerteWifi = 0;
      connecterWifi();
    }
  } else {
    tPerteWifi = 0;
  }

  if (reseauOk) {
    if (!mqtt.connected()) {
      if (millis() - tReconnexionMqtt > 5000) {
        tReconnexionMqtt = millis();
        connecterMqtt();
      }
    }
    mqtt.loop();
  }
  const bool publiable = reseauOk && mqtt.connected();

  // Relance d'une detection de debit demandee par MQTT.
  if (demandeScan) {
    demandeScan = false;
    vTaskSuspend(tacheCanHandle);
    initialiserCan();
    vTaskResume(tacheCanHandle);
    if (publiable) publierTexte(MQTT_BASE "/etat/erreur", debitLabel, false);
  }

  if (demandeReset) {
    demandeReset = false;
    nbSuivis = 0; nbVoiesPerdues = 0;
    memset(suivis, 0, sizeof(suivis));
  }

  // Vidage de la file : borne a 64 trames par tour pour laisser respirer
  // la pile MQTT et la supervision WiFi.
  TrameCan t;
  uint8_t traitees = 0;
  while (traitees < 64 && xQueueReceive(fileTrames, &t, 0) == pdTRUE) {
    traitees++;

    SuiviId* s = suivre(t);
    if (s) {
      if (s->compte > 0 && t.horodatage > s->dernierVu) {
        uint32_t delta = t.horodatage - s->dernierVu;
        // Moyenne glissante simple sur la periode d'emission.
        s->periodeMs = s->periodeMs ? (s->periodeMs * 3 + delta) / 4 : delta;
      }
      s->compte++;
      s->dernierVu = t.horodatage;
      s->dlc = t.dlc;
      memcpy(s->data, t.data, 8);
    }

    if (publiable) {
      publierTrameBrute(t, s);
      publierSignaux(t);
    }
  }

  uint32_t maintenant = millis();

  if (PUBLIER_DECOUVERTE && publiable &&
      maintenant - tDecouverte >= INTERVALLE_DECOUVERTE_MS) {
    tDecouverte = maintenant;
    publierDecouverte(false);
  }

  if (publiable && maintenant - tStats >= INTERVALLE_STATS_MS) {
    tStats = maintenant;
    publierStats();
  }

  // Sans reseau, on ne tourne pas a vide : la file reste videe mais la boucle
  // rend la main. 20 ms laissent la place a 64 trames par tour, tres au-dela
  // des deux trames par seconde du bus.
  if (!reseauOk) delay(20);
}
