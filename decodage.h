// ============================================================================
// decodage.h - Description des trames CAN (protocole SCBI de SOREL)
//
// Deux mecanismes, indispensables pour SOREL :
//
//   - Masque d'identifiant : les identifiants SCBI contiennent l'adresse du
//     noeud, qui varie d'une installation a l'autre. Le masque permet de la
//     ignorer.
//
//   - Prefixe (multiplexage) : un meme identifiant CAN transporte plusieurs
//     grandeurs, designees par un ou plusieurs octets d'index en tete de
//     trame. Plusieurs entrees de la table partagent donc le meme identifiant
//     et se distinguent par leur prefixe.
//
// Le reste reprend le format d'un fichier DBC : position en bits, endianness,
// signe, et transformation affine (valeur = brut * facteur + offset).
// ============================================================================
#pragma once

#include <Arduino.h>

// Un signal a l'interieur d'une trame.
struct Signal {
  const char* nom;        // suffixe du topic : <base>/valeur/<trame>/<nom>
  uint8_t     bitDepart;  // Intel : bit de poids faible. Motorola : bit de poids fort.
  uint8_t     longueur;   // nombre de bits (1..64)
  bool        motorola;   // true = big endian (Motorola), false = little endian (Intel)
  bool        signe;      // true = complement a deux
  float       facteur;
  float       offset;
  const char* unite;      // "V", "A", "degC", "%", "" ...
  const char* classeHA;   // classe Home Assistant : voltage, current, temperature...

  // Plage de vraisemblance. Une valeur en dehors n'est pas publiee : le SCBI
  // signale une sonde absente ou en defaut par une valeur sentinelle (0x8001,
  // soit -3276,7 apres mise a l'echelle), qu'il ne faut pas prendre pour une
  // mesure. mini == maxi desactive le controle.
  float       mini;
  float       maxi;
};

// Une trame CAN reconnue, eventuellement un seul cas de multiplexage.
struct Trame {
  uint32_t      id;          // identifiant deja masque par idMasque
  uint32_t      idMasque;    // 0x1FFFFFFF = identifiant complet
  bool          etendu;      // true = identifiant 29 bits
  uint8_t       prefixe[4];  // octets attendus a partir de data[0]
  uint8_t       prefixeLong; // 0 = pas de filtre sur le contenu
  const char*   nom;
  const Signal* signaux;
  uint8_t       nbSignaux;
};

extern const Trame   TRAMES[];
extern const uint8_t NB_TRAMES;

// true si la trame recue correspond a cette entree (identifiant + prefixe).
bool trameCorrespond(const Trame& t, uint32_t id, bool etendu,
                     const uint8_t* data, uint8_t dlc);

// true si au moins une entree de la table utilise cet identifiant.
bool identifiantConnu(uint32_t id, bool etendu);

// Nombre d'octets de tete servant d'index de multiplexage pour cet
// identifiant. Determine la finesse des topics <base>/brut et
// <base>/decouverte : une voie par valeur distincte de ces octets.
// La plupart des groupes SCBI utilisent un seul octet d'index ; le groupe 0x07
// en utilise deux.
uint8_t longueurMux(uint32_t id, bool etendu);

// Extrait un signal. Retourne false si le signal deborde de la longueur utile
// (dlc) de la trame recue, ou si la valeur sort de la plage de vraisemblance.
bool extraireSignal(const Signal& s, const uint8_t* data, uint8_t dlc, float& valeur);
