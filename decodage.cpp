// ============================================================================
// decodage.cpp - Table de decodage du protocole SCBI (SOREL) et extraction
//
// ETAT DES CONNAISSANCES (voir README, section "Sources")
//
//   L'APPAREIL : un SOREL MTDC version 4, et non un MTDC-E - l'ecran affiche
//   "MTDC v4". La distinction compte : le MTDC-E n'a que 3 sondes et pas de
//   sortie 0-10 V, alors que le MTDC version 3 et 4 en possede une (V1). Aucun
//   des manuels publies par SOREL ne correspond exactement a la numerotation
//   des menus de ce firmware (4.19.x), mais le manuel MTDC anglais 16005 decrit
//   bien le materiel, y compris la mention "MTDC Version 3+4" pour V1.
//
//   CONFIRME sur cette installation
//     - bus a 250 kbit/s, identifiants 29 bits
//       0x1000_0000 | groupe<<16 | adresse<<8 | 0x80
//     - adresse 0x1A, lue dans le menu du regulateur ("CAN ID 26" en decimal)
//     - variante B : data = [index][valeur LSB][valeur MSB], DLC 5
//     - groupe 0x01 = capteurs, temperatures 16 bits little endian signees /10
//       (releve 10011A80 : 00 BE 00 00 00 -> capteur 0, 190 -> 19,0 degC)
//     - une sonde absente remonte la sentinelle 0x8001, ecartee par la plage
//       de vraisemblance des signaux
//
//     - groupe 0x02 = relais : [index][mode][etat]. L'etat vaut 0x00 a l'arret
//       et 0xFF en marche, etabli en commutant R2 a la main dans les deux sens
//       et en observant l'index 1 suivre. Publie en pourcentage (100/255).
//     - groupe 0x06 index 0 = mode de service : 0 automatique, 1 manuel,
//       verifie en basculant le menu du regulateur dans les deux sens
//     - groupe 0x03 = numero de schema hydraulique, en ASCII. Cette
//       installation diffuse "S10", soit le schema D.10 "Solaire + Thermostat"
//       du manuel, qui donne l'affectation des sondes et des sorties
//
//   PARTIELLEMENT ETABLI
//     - groupe 0x06 = statistiques : un index 0x08 a ete observe, au-dela des
//       huit index documentes par can-ltdc-logger. Unites non etablies
//
//   NON DECODE
//     - groupe 0x07 : compteurs cumules par periode, unites a etablir,
//       voir la note detaillee plus bas
//     - groupe 0x09 : MESSAGES TEXTE du regulateur, fragmentes. Chaque trame
//       porte un index en tete puis jusqu'a 7 octets de texte ; l'index 0xFE
//       marque le dernier fragment. Les accents utilisent un codage deux
//       octets propre a SOREL (0xC2 0x9A pour "e accent grave", 0xC2 0x9B pour
//       "e accent aigu"), qui n'est pas de l'UTF-8. Message reassemble sur
//       cette installation : "Probleme debit", reemis toutes les deux heures
//       environ - donc un defaut actif, pas un evenement isole.
//     - groupe 0x08
//     - trames dont le dernier octet vaut 0x85 ou 0x0B au lieu de 0x80
//     - identifiants 11 bits (0x000, 0x300, 0x304, 0x330, 0x334) dont les
//       charges utiles reprennent celles des trames etendues, parfois avec
//       l'adresse "1A 80" au milieu des donnees. Duplication ou mauvais
//       cadrage a la reception : non elucide.
//
// Les deux variantes de disposition peuvent etre chargees simultanement : elles
// se distinguent par leur prefixe, donc une seule peut correspondre. Si la
// mauvaise est chargee, elle ne publie rien - jamais de valeur fausse.
// ============================================================================
#include "decodage.h"
#include "config.h"

// ---------------------------------------------------------------------------
// Construction des identifiants SCBI
// ---------------------------------------------------------------------------
#define GRP_CAPTEURS  0x01
#define GRP_RELAIS    0x02
#define GRP_STATS     0x06
#define GRP_BILAN     0x07

#if SOREL_ADRESSE_AUTO
  // On ignore l'octet d'adresse : la table fonctionne quelle que soit
  // l'adresse du regulateur sur le bus.
  #define SOREL_MASQUE  0x1FFF00FFUL
#else
  #define SOREL_MASQUE  0x1FFFFFFFUL
#endif

#define SOREL_ID(groupe)                                                    \
  ((0x10000000UL | ((uint32_t)(groupe) << 16) |                             \
    ((uint32_t)(SOREL_ADRESSE) << 8) | 0x80UL) & SOREL_MASQUE)

#define NB_SIG(a) (uint8_t)(sizeof(a) / sizeof((a)[0]))

// Degre Celsius en UTF-8, ecrit en deux litteraux : "\xC2\xB0C" serait lu
// comme un unique echappement hexadecimal par le compilateur.
#define DEG_C "\xC2\xB0" "C"

// ---------------------------------------------------------------------------
// VARIANTE A - trame precedee d'un octet de type 0x05
//   data = [0x05][index][valeur LSB][valeur MSB]...
//   Observee sur un SOREL LTDC (trace : 10012480 : 05 00 CE 04 00 00
//   -> capteur 0, 0x04CE = 1230, soit 123,0 degC).
// ---------------------------------------------------------------------------
#if SOREL_VARIANTE == 0 || SOREL_VARIANTE == 1

// Temperatures : 16 bits little endian a l'octet 2, signe (gel possible), /10.
static const Signal SIG_A_S1[] = { { "s1", 16, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_A_S2[] = { { "s2", 16, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_A_S3[] = { { "s3", 16, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_A_S4[] = { { "s4", 16, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };

// Relais : octet de niveau a l'octet 3. Echelle a confirmer (le script
// Tasmota de reference applique un facteur -0,01 dont le sens n'est pas etabli).
static const Signal SIG_A_R1[] = { { "r1", 24, 8, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_A_R2[] = { { "r2", 24, 8, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_A_R3[] = { { "r3", 24, 8, false, false, 1.0f, 0.0f, "", "" } };

#endif

// ---------------------------------------------------------------------------
// VARIANTE B - index directement en tete de trame
//   data = [index][valeur LSB][valeur MSB]...
//   Observee sur un SOREL LTDC / Austria Email ECO SWIFT (can-ltdc-logger).
// ---------------------------------------------------------------------------
#if SOREL_VARIANTE == 0 || SOREL_VARIANTE == 2

static const Signal SIG_B_S1[] = { { "s1", 8, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_B_S2[] = { { "s2", 8, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_B_S3[] = { { "s3", 8, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };
static const Signal SIG_B_S4[] = { { "s4", 8, 16, false, true, 0.1f, 0.0f, DEG_C, "temperature", -50.0f, 300.0f } };

// Sorties : octet 1 = mode, octet 2 = etat.
//
// Affectation etablie par le schema hydraulique D.10 "Solaire + Thermostat"
// du manuel MTDC, confirme par la chaine ASCII "S10" que le regulateur diffuse
// sur le groupe 0x03 :
//     index 0 = R1, pompe solaire
//     index 1 = R2, fonction thermostat (chauffe d appoint)
//     index 2 = V1, sortie 0-10 V / PWM, presente sur les MTDC version 3 et 4
// Les sondes du meme schema : S1 collecteur, S2 bas du ballon, S3 haut du
// ballon - S3 etant la sonde de regulation de la chauffe.
//
// Table des modes reprise de l'integration Home Assistant officielle de SOREL
// (SOREL-GmbH/sorel_connect_ha, const.py RELAY_MODES) - bien plus complete que
// les quatre modes de can-ltdc-logger, et elle donne les echelles :
//
//     0, 6  tout ou rien       binaire, 0 = arret et 1000 = marche
//     1, 8  commande de phase  % , valeur / 10
//     2, 9  PWM                % , valeur / 10
//     3, 10 commande tension   V , valeur / 100
//     4     PWM direct         % , valeur / 10
//     5, 11 tension directe    V , valeur / 100
//     7     cycle tout-ou-rien % , valeur / 10
//     12    PWM direct         % , valeur / 10
//     13-16 n modes / interne / erreur / invalide
//
// ATTENTION : ces echelles decrivent la representation des points de donnees
// de SOREL Connect, ou un pourcentage va de 0 a 1000. L'encodage CAN est
// different : c'est un octet pleine echelle, 0x00 a 0xFF.
//
// L'OCTET 2 EST L'ETAT DU RELAIS - etabli par manipulation directe.
//
// Le regulateur a ete place en mode de service "Manuel" et les relais commutes
// a la main. L'octet 2 a suivi sur les trois index, et ne prend que deux
// valeurs. Les octets 3 et 4 restent a 0xFF en permanence : du remplissage.
//
// Deux fausses pistes ont precede ce resultat, et elles meritent d'etre
// signalees a qui reprendrait ce decodage :
//   - 0xFF a d'abord ete pris pour une sentinelle "pas de valeur", par analogie
//     avec le 0x8001 des sondes. La plage de vraisemblance avait ete bornee a
//     254, ce qui masquait justement l'etat "en marche" ;
//   - la polarite a ensuite semble incoherente, parce que le raisonnement
//     s'appuyait sur les consignes par defaut de la notice au lieu des reglages
//     reels de l'installation.
//
// POLARITE ETABLIE : 0x00 = arret, 0xFF = marche.
//
// Releve avec verite terrain, regulateur en mode Manuel, R2 commute a la main
// dans les deux sens, l'index 1 suivant a chaque fois :
//
//     R2 en marche   01 00 FF FF FF     octet 2 = 0xFF
//     R2 a l'arret   01 00 00 FF FF     octet 2 = 0x00
//
// L'index 1 correspond donc bien a R2, et l'octet 2 est l'etat du relais.
//
// Cela explique aussi une lecture qui paraissait contradictoire : en mode
// automatique, a 21h15, l'index 1 etait deja a 0xFF avec S3 a 67 degC. Le
// thermostat appelait donc legitimement de la chaleur - la consigne Tset S3
// de cette installation est simplement plus haute que les 50 degC par defaut
// de la notice, sur laquelle j'avais raisonne a tort.
//
// Le niveau est publie en pourcentage : facteur 100/255, soit 0 % a l'arret et
// 100 % en marche. Cette mise a l'echelle est verifiee aux deux extremes. Elle
// suppose en revanche une progression lineaire pour les valeurs intermediaires
// du mode PWM (index 2), ce qui reste a confirmer le jour ou le circulateur
// modulera vraiment.
//
// A confirmer aussi : en mode PWM (index 2), le niveau prend-il des valeurs
// intermediaires quand le circulateur module, ou reste-t-il binaire ?
static const Signal SIG_B_R1[] = {
  { "r1_mode",   8, 8, false, false, 1.0f, 0.0f, "", "", 0.0f, 16.0f },
  { "r1_niveau", 16, 8, false, false, 100.0f/255.0f, 0.0f, "%", "", 0.0f, 100.0f },
};
static const Signal SIG_B_R2[] = {
  { "r2_mode",   8, 8, false, false, 1.0f, 0.0f, "", "", 0.0f, 16.0f },
  { "r2_niveau", 16, 8, false, false, 100.0f/255.0f, 0.0f, "%", "", 0.0f, 100.0f },
};
static const Signal SIG_B_R3[] = {
  { "v1_mode",   8, 8, false, false, 1.0f, 0.0f, "", "", 0.0f, 16.0f },
  { "v1_niveau", 16, 8, false, false, 100.0f/255.0f, 0.0f, "%", "", 0.0f, 100.0f },
};

// Statistiques : 16 bits little endian a l'octet 1. Unites a confirmer.
//
// mode_fonctionnement est confirme : il suit le menu "Mode de service" du
// regulateur. Releve 0 en automatique, puis 1 des le passage en manuel, puis
// de nouveau 0 au retour en automatique. La notice decrit un troisieme mode,
// "Arret", qui vaut probablement 2 - non observe.
static const Signal SIG_B_ST0[] = { { "mode_fonctionnement", 8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST1[] = { { "puissance",           8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST2[] = { { "puissance_vfs1",      8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST3[] = { { "puissance_vfs2",      8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST4[] = { { "rendement_const",     8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST5[] = { { "rendement_vfs1",      8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST6[] = { { "rendement_vfs2",      8, 16, false, false, 1.0f, 0.0f, "", "" } };
static const Signal SIG_B_ST7[] = { { "rendement_total",     8, 16, false, false, 1.0f, 0.0f, "", "" } };

// Groupe 0x07 (DLC 8) : CARTOGRAPHIE FAITE, decodage volontairement absent.
//
// Structure etablie : [selecteur][sous-index][valeur LSB][valeur MSB][...]
// La valeur utile est aux octets 2-3, en 16 bits little endian - et non aux
// octets 4-5 comme l'indiquait can-ltdc-logger. Le selecteur code une periode
// dans ses bits 5-7 (valeurs 1 a 7) ; le sous-index prend 0, 1 ou 2.
//
// Releve complet sur cette installation (topics <base>/brut/0x10071A80/xxxx) :
//
//     selecteur  sous-index 0   1      2        periode (can-ltdc-logger)
//     0x21 (1)             1    11     1        jour
//     0x40 (2)             3    34     3        semaine
//     0x60 (3)            24   178    24        mois
//     0x80 (4)           744  4628   744        annee
//     0xA0 (5)      non emis par cet appareil   total
//     0xC0 (6)             0                    ?
//     0xE0 (7)             3 (+ 15 aux octets 4-5)   ?
//
// CE SONT DES COMPTEURS CUMULES PAR PERIODE - correction d'une conclusion
// precedente erronee.
//
// Une premiere observation, faite sur quelques minutes, avait conclu a des
// descripteurs statiques : les valeurs ne bougeaient pas, et 24 et 744
// tombaient pile sur le nombre d'heures d'un jour et d'un mois de 31 jours.
// C'etait une coincidence, et la fenetre d'observation etait trop courte.
//
// Releve du 09/09 (mercredi) puis du 14/09 (lundi) :
//
//     voie    09/09   14/09   delta
//     6000       24      75    +51
//     6001      178     291   +113
//     6002       24      75    +51
//     8000      744     795    +51
//     8001     4628    4741   +113
//     8002      744     795    +51
//     4000        3       0     -3    <- remise a zero
//     4001       34       0    -34    <- remise a zero
//     4002        3       0     -3    <- remise a zero
//
// Deux faits tranchent :
//   - les familles 0x60 et 0x80 s'incrementent regulierement, d'environ
//     10 par jour pour les sous-index 0 et 2, et 22 par jour pour le 1 ;
//   - la famille 0x40 est retombee a zero, et le 14/09 est un LUNDI.
//
// C'est donc bien la lecture de can-ltdc-logger qui est la bonne, avec les
// bits 5-7 du selecteur codant la periode : 0x2x jour, 0x4x semaine (remise a
// zero le lundi), 0x6x mois, 0x8x annee, 0xAx total (non emis ici). Les
// valeurs sont des cumuls qui repartent de zero a chaque debut de periode.
//
// Les unites restent a etablir. L'increment d'environ 10 par jour des
// sous-index 0 et 2 est compatible avec des heures de fonctionnement du relais
// de chauffe, qui tourne la nuit sur plage horaire. A confirmer en correlant
// avec la duree reelle de fonctionnement.
//
// Les bits 0-4 du selecteur de la famille "jour" changent eux aussi : 0x20 a
// 0x25 ont ete observes, mais un seul est emis a la fois - les autres ne
// subsistent que comme topics retenus des jours precedents. C'est donc un
// index d'enregistrement journalier. La correspondance avec le jour de la
// semaine n'est pas etablie : le 09/09, un mercredi, emettait 0x21, et le
// 14/09, un lundi, emet 0x20. Plutot un emplacement tournant qu'un jour fixe.
//
// Rien n'est encore publie en valeurs decodees : il faut d'abord savoir ce que
// comptent ces compteurs. Le groupe reste entierement visible en brut, une
// voie par couple (selecteur, sous-index).

#endif

// ---------------------------------------------------------------------------
// Table des trames
//   { identifiant, masque, etendu, {prefixe}, longueur prefixe, nom, signaux }
// ---------------------------------------------------------------------------
const Trame TRAMES[] = {

#if SOREL_VARIANTE == 0 || SOREL_VARIANTE == 1
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x05, 0x00, 0, 0}, 2, "capteurs", SIG_A_S1, NB_SIG(SIG_A_S1) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x05, 0x01, 0, 0}, 2, "capteurs", SIG_A_S2, NB_SIG(SIG_A_S2) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x05, 0x02, 0, 0}, 2, "capteurs", SIG_A_S3, NB_SIG(SIG_A_S3) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x05, 0x03, 0, 0}, 2, "capteurs", SIG_A_S4, NB_SIG(SIG_A_S4) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x05, 0x00, 0x00, 0}, 3, "relais", SIG_A_R1, NB_SIG(SIG_A_R1) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x05, 0x01, 0x00, 0}, 3, "relais", SIG_A_R2, NB_SIG(SIG_A_R2) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x05, 0x02, 0x00, 0}, 3, "relais", SIG_A_R3, NB_SIG(SIG_A_R3) },
#endif

#if SOREL_VARIANTE == 0 || SOREL_VARIANTE == 2
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x00, 0, 0, 0}, 1, "capteurs", SIG_B_S1, NB_SIG(SIG_B_S1) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x01, 0, 0, 0}, 1, "capteurs", SIG_B_S2, NB_SIG(SIG_B_S2) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x02, 0, 0, 0}, 1, "capteurs", SIG_B_S3, NB_SIG(SIG_B_S3) },
  { SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x03, 0, 0, 0}, 1, "capteurs", SIG_B_S4, NB_SIG(SIG_B_S4) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x00, 0, 0, 0}, 1, "relais", SIG_B_R1, NB_SIG(SIG_B_R1) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x01, 0, 0, 0}, 1, "relais", SIG_B_R2, NB_SIG(SIG_B_R2) },
  { SOREL_ID(GRP_RELAIS),   SOREL_MASQUE, true, {0x02, 0, 0, 0}, 1, "relais", SIG_B_R3, NB_SIG(SIG_B_R3) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x00, 0, 0, 0}, 1, "statistiques", SIG_B_ST0, NB_SIG(SIG_B_ST0) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x01, 0, 0, 0}, 1, "statistiques", SIG_B_ST1, NB_SIG(SIG_B_ST1) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x02, 0, 0, 0}, 1, "statistiques", SIG_B_ST2, NB_SIG(SIG_B_ST2) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x03, 0, 0, 0}, 1, "statistiques", SIG_B_ST3, NB_SIG(SIG_B_ST3) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x04, 0, 0, 0}, 1, "statistiques", SIG_B_ST4, NB_SIG(SIG_B_ST4) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x05, 0, 0, 0}, 1, "statistiques", SIG_B_ST5, NB_SIG(SIG_B_ST5) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x06, 0, 0, 0}, 1, "statistiques", SIG_B_ST6, NB_SIG(SIG_B_ST6) },
  { SOREL_ID(GRP_STATS),    SOREL_MASQUE, true, {0x07, 0, 0, 0}, 1, "statistiques", SIG_B_ST7, NB_SIG(SIG_B_ST7) },
  // Groupe 0x07 : voir la note ci-dessus, non decode pour l'instant.
#endif

};

const uint8_t NB_TRAMES = sizeof(TRAMES) / sizeof(TRAMES[0]);

// ---------------------------------------------------------------------------

bool trameCorrespond(const Trame& t, uint32_t id, bool etendu,
                     const uint8_t* data, uint8_t dlc) {
  if (t.etendu != etendu) return false;
  if ((id & t.idMasque) != t.id) return false;
  if (t.prefixeLong > dlc) return false;
  for (uint8_t i = 0; i < t.prefixeLong; i++) {
    if (data[i] != t.prefixe[i]) return false;
  }
  return true;
}

uint8_t longueurMux(uint32_t id, bool etendu) {
  if (!etendu) return 1;

  // Groupe 0x07 : selecteur sur deux octets. Le premier code la periode dans
  // ses bits 5-7 (valeurs 1 a 7), le second designe un champ dans
  // l'enregistrement. Un seul octet de cle melangerait ces champs.
  // Le masque ignore l'adresse du noeud et le dernier octet.
  if ((id & 0x1FFF0000UL) == (0x10000000UL | ((uint32_t)GRP_BILAN << 16))) {
    return 2;
  }
  return 1;
}

bool identifiantConnu(uint32_t id, bool etendu) {
  for (uint8_t i = 0; i < NB_TRAMES; i++) {
    if (TRAMES[i].etendu == etendu && (id & TRAMES[i].idMasque) == TRAMES[i].id) {
      return true;
    }
  }
  return false;
}

bool extraireSignal(const Signal& s, const uint8_t* data, uint8_t dlc, float& valeur) {
  if (s.longueur == 0 || s.longueur > 64) return false;

  uint64_t brut = 0;

  if (s.motorola) {
    // Parcours "dents de scie" : du bit de poids fort vers le poids faible,
    // en descendant dans l'octet puis en sautant a l'octet suivant.
    int bit = s.bitDepart;
    for (uint8_t i = 0; i < s.longueur; i++) {
      int octet = bit >> 3;
      if (octet < 0 || octet >= dlc) return false;
      brut = (brut << 1) | ((data[octet] >> (bit & 7)) & 0x01);
      if ((bit & 7) == 0) bit += 15; else bit -= 1;
    }
  } else {
    for (uint8_t i = 0; i < s.longueur; i++) {
      int bit = s.bitDepart + i;
      int octet = bit >> 3;
      if (octet >= dlc) return false;
      brut |= (uint64_t)((data[octet] >> (bit & 7)) & 0x01) << i;
    }
  }

  double v;
  if (s.signe && s.longueur < 64 && (brut & (1ULL << (s.longueur - 1)))) {
    // Extension de signe sur 64 bits.
    v = (double)(int64_t)(brut | (~0ULL << s.longueur));
  } else {
    v = (double)brut;
  }

  float r = (float)(v * s.facteur + s.offset);

  // Ecarte les valeurs sentinelles (sonde absente ou en defaut).
  if (s.mini != s.maxi && (r < s.mini || r > s.maxi)) return false;

  valeur = r;
  return true;
}
