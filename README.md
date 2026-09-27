# Passerelle CAN → MQTT pour SOREL MTDC-E

Lecture **passive** du bus CAN d'un SOREL MTDC-E par un ESP32 + module MCP2515,
et publication des données vers un broker MQTT.

La passerelle **n'émet aucune trame** — il n'y a aucun appel d'envoi dans le
code. Le MCP2515 tourne en revanche en **mode normal**, et non en listen-only,
afin de fournir au MTDC-E le bit d'acquittement dont il a besoin : sur un bus à
deux nœuds, un observateur muet laisse le régulateur sans acquittement, ce qui
le fait retransmettre puis passer en *bus-off*. Le comportement se règle par
`CAN_ECOUTE_SEULE` dans `config.h`.

## L'appareil et son protocole

Le MTDC-E est un régulateur différentiel SOREL (*Medium Temperature Difference
Controller*) : 4 entrées PT1000, 2 relais 230 V, 1 sortie 0-10 V / PWM. Son bus
CAN sert normalement à relier plusieurs régulateurs ou un datalogger « SOREL
Connect ». Le protocole s'appelle **SCBI** (*SOREL CAN Bus Interface*).

SOREL ne publie pas la spécification SCBI. Le point de départ vient de deux
projets de rétro-ingénierie portant sur des LTDC (voir Sources) ; ce qui suit
distingue ce qui a été **vérifié sur ce MTDC-E** de ce qui reste hypothétique.

### L'appareil, et le manuel qui lui correspond

L'écran affiche **« MTDC v4 »** : c'est un MTDC version 4, **pas un MTDC-E**. La
distinction compte — le MTDC-E n'a que 3 sondes et aucune sortie 0-10 V, alors
que les MTDC version 3 et 4 en possèdent une.

Aucun manuel publié par SOREL ne correspond exactement à la numérotation des
menus de ce firmware (`4.19.x`). Le plus proche pour le **matériel** est le
[manuel MTDC anglais 16005](https://www.sorel.de/wp-content/uploads/shared-files/MTDC_16005_en.pdf),
qui mentionne explicitement « MTDC Version 3+4 » pour la sortie V1.

### Le schéma hydraulique, diffusé sur le bus

Le groupe `0x03` transporte le **numéro de schéma en ASCII** : cette
installation émet `53 31 30`, soit **« S10 »** — le schéma D.10 « Solaire +
Thermostat » du manuel. C'est lui qui donne toute l'affectation :

| Borne | Fonction | Voie CAN |
|---|---|---|
| S1 | sonde collecteur | `capteurs/s1` |
| S2 | sonde ballon **bas** | `capteurs/s2` |
| S3 | sonde ballon **haut** — régulation de la chauffe | `capteurs/s3` |
| R1 | pompe solaire | index 0 → `relais/r1_*` |
| R2 | fonction thermostat (chauffe d'appoint) | index 1 → `relais/r2_*` |
| V1 | sortie 0-10 V / PWM | index 2 → `relais/v1_*` |

L'index 2 était nommé `r3_*` jusqu'ici, ce qui était faux : ce n'est pas un
troisième relais mais la sortie de modulation V1.

### Confirmé sur cette installation

- Débit : **250 kbit/s**.
- Identifiants **29 bits** de la forme :

  ```
  0x1000_0000 | groupe << 16 | adresse << 8 | 0x80
  ```

  `adresse` = **0x1A**, lue dans le menu du régulateur : « CAN ID 26 », en
  **décimal**. Elle est figée dans `config.h` (`SOREL_ADRESSE_AUTO 0`), ce qui
  évite de décoder un autre nœud du bus — des trames d'adresse `0x00` ont été
  observées ici.

  Le dernier octet n'est pas toujours `0x80` : des trames en `0x85` et `0x0B`
  circulent aussi. Types de message non identifiés, laissés de côté par le
  masque mais visibles sur `<base>/brut`.

- **Disposition : variante B**, `data = [index][valeur LSB][valeur MSB]`, DLC 5.
  La variante A du fil Tasmota (`[0x05][index]…`, DLC 6) ne correspond pas à cet
  appareil. `SOREL_VARIANTE` est figé à `2`.

- **Groupe `0x01` — capteurs.** Entier 16 bits little-endian **signé**, ÷ 10.
  Relevé : `10011A80 : 00 BE 00 00 00` → capteur 0, `0x00BE` = 190 → **19,0 °C**.
  Les index 0 à 3 correspondent à S1..S4.

- **Sonde absente** : le régulateur remonte la sentinelle `0x8001`, soit
  −3276,7 °C après mise à l'échelle. Écartée par la plage de vraisemblance
  (−50 à 300 °C) des signaux, donc jamais publiée comme une mesure.

### Partiellement établi

- **Groupe `0x02` — relais** : `[index][mode][niveau]`. La table complète des
  modes vient de l'intégration Home Assistant **officielle** de SOREL
  (`const.py`, `RELAY_MODES`) :

  | Mode | Nom | Unité | Échelle |
  |---|---|---|---|
  | 0, 6 | tout ou rien | binaire | 0 = arrêt, **1000 = marche** |
  | 1, 8 | commande de phase | % | ÷ 10 |
  | 2, 9 | PWM | % | ÷ 10 |
  | 3, 10 | commande en tension | V | ÷ 100 |
  | 4, 12 | PWM direct | % | ÷ 10 |
  | 5, 11 | tension directe | V | ÷ 100 |
  | 7 | cycle tout-ou-rien | % | ÷ 10 |
  | 13-16 | n modes / interne / erreur / invalide | — | — |

  **L'encodage CAN n'est pas celui de SOREL Connect.** Là où les points de
  données du service utilisent une échelle 0-1000, le bus transporte un simple
  **octet pleine échelle** :

  | Octet 2 | État | Publié |
  |---|---|---|
  | `0x00` | arrêt | `0` % |
  | `0xFF` | marche | `100` % |

  Établi avec vérité terrain : régulateur en mode Manuel, R2 commuté à la main
  dans les deux sens, l'index 1 suivant à chaque fois.

  ```
  R2 en marche  :  01 00 FF FF FF
  R2 à l'arrêt  :  01 00 00 FF FF
  ```

  Le niveau est donc publié en pourcentage (facteur 100/255), avec l'unité `%`.
  Les octets 3 et 4 restent à `0xFF` en permanence : du remplissage.

  Reste à confirmer : la **linéarité aux valeurs intermédiaires** du mode PWM
  (index 2), déduite des seuls deux extrêmes. Ça se verra le jour où le
  circulateur modulera vraiment.

  Deux fausses pistes ont précédé ce résultat, signalées ici pour qui
  reprendrait le décodage :

  - `0xFF` a d'abord été pris pour une sentinelle « pas de valeur », par
    analogie avec le `0x8001` des sondes. La plage de vraisemblance bornée à 254
    masquait alors précisément l'état « en marche » ;
  - la polarité a ensuite semblé incohérente, parce que le raisonnement
    s'appuyait sur les consignes **par défaut de la notice** plutôt que sur les
    réglages réels de l'installation, dont la consigne `Tset S3` est nettement
    plus haute.

- **Groupe `0x06` — statistiques** : `[index][valeur LSB][valeur MSB]`. Un index
  `0x08` a été observé, au-delà des huit index documentés par `can-ltdc-logger`.
  Unités non établies, valeurs publiées brutes.

### Non décodé

- **Groupes `0x03` et `0x08`.** Le `0x03` transporte de l'ASCII : `53 31 30` =
  « S10 ». Probablement une identification de version ou de schéma hydraulique.

### Groupe `0x07` : cartographié, mais sans intérêt

C'est le plus bavard du bus, environ une trame par seconde. Sa structure est
maintenant établie :

```
[sélecteur][sous-index][valeur LSB][valeur MSB][...]
```

La valeur utile est aux **octets 2-3** en 16 bits little-endian — et non aux
octets 4-5 comme l'indiquait `can-ltdc-logger`. Le sélecteur code une période
dans ses bits 5-7, le sous-index prend 0, 1 ou 2. Relevé complet :

| Sélecteur | sous-index 0 | 1 | 2 | Période (d'après `can-ltdc-logger`) |
|---|---|---|---|---|
| `0x21` (1) | 1 | 11 | 1 | jour |
| `0x40` (2) | 3 | 34 | 3 | semaine |
| `0x60` (3) | 24 | 178 | 24 | mois |
| `0x80` (4) | 744 | 4628 | 744 | année |
| `0xA0` (5) | *non émis par cet appareil* | | | total |
| `0xC0` (6) | 0 | | | ? |
| `0xE0` (7) | 3 (+ 15 aux octets 4-5) | | | ? |

**Ce sont des compteurs cumulés par période** — et cela corrige une conclusion
antérieure erronée. Une première observation, faite sur quelques minutes, avait
conclu à des descripteurs statiques : rien ne bougeait, et 24 et 744 tombaient
pile sur le nombre d'heures d'un jour et d'un mois de 31 jours. Coïncidence, et
fenêtre d'observation beaucoup trop courte.

Relevé du 09/09 (mercredi) puis du 14/09 (lundi) :

| Voie | 09/09 | 14/09 | Δ |
|---|---|---|---|
| `6000` | 24 | 75 | +51 |
| `6001` | 178 | 291 | +113 |
| `6002` | 24 | 75 | +51 |
| `8000` | 744 | 795 | +51 |
| `8001` | 4628 | 4741 | +113 |
| `8002` | 744 | 795 | +51 |
| `4000` | 3 | **0** | remise à zéro |
| `4001` | 34 | **0** | remise à zéro |
| `4002` | 3 | **0** | remise à zéro |

Deux faits tranchent : les familles `0x60` et `0x80` s'incrémentent
régulièrement, et la famille `0x40` **est retombée à zéro un lundi**.

C'est donc la lecture de `can-ltdc-logger` qui est la bonne : les bits 5-7 du
sélecteur codent la période — `0x2x` jour, `0x4x` semaine (remise à zéro le
lundi), `0x6x` mois, `0x8x` année, `0xAx` total. Les valeurs sont des cumuls
qui repartent de zéro à chaque début de période.

Les **unités restent à établir**. L'incrément d'environ 10 par jour des
sous-index 0 et 2 est compatible avec des heures de fonctionnement du relais de
chauffe, qui tourne la nuit sur plage horaire. Rien n'est publié en valeurs
décodées tant que ce n'est pas confirmé.

### Groupe `0x09` : les messages texte du régulateur

Le régulateur diffuse ses messages en clair, fragmentés : chaque trame porte un
index en tête puis jusqu'à 7 octets de texte, l'index `0xFE` marquant le dernier
fragment.

```
0x09041A80/00 : 00 50 72 6F 62 6C C2 9A     "Probl" + è
0x09041A80/01 : 01 6D 65 20 64 C2 9B 62     "me d" + é + "b"
0x09041A80/FE : FE 69 74                    "it"
```

Reassemblé : **« Problème débit »**. Les accents utilisent un codage deux octets
propre à SOREL (`C2 9A` = è, `C2 9B` = é) qui n'est **pas** de l'UTF-8.

Ce message est réémis toutes les deux heures environ — c'est donc un défaut
actif, pas un événement isolé.

### Publication par voie, pas par identifiant

Le protocole étant fortement multiplexé, l'anti-flood et l'inventaire sont
indexés sur le couple **(identifiant, octets d'index de tête)**, appelé « voie »
dans le code. Les topics ont donc deux niveaux :

```
<base>/brut/0x10011A80/00       capteur S1
<base>/brut/0x10011A80/03       capteur S4
<base>/brut/0x10071A80/8001     groupe 0x07, sélecteur 0x80, sous-index 1
```

Sans ce second niveau, toutes les grandeurs portées par un même identifiant
s'écrasaient sur un seul topic et on ne voyait que la dernière.

La **longueur de l'index dépend du groupe** : un octet pour la plupart, mais
deux pour le groupe `0x07`, qui utilise un sélecteur et un sous-index. C'est
`longueurMux()` dans `decodage.cpp` qui en décide. Le prendre sur un seul octet
mélangeait les sous-index du `0x07` et empêchait de le cartographier ; le
prendre systématiquement sur deux créerait une voie par valeur de température,
le second octet étant l'octet de poids faible de la mesure dans le groupe
`0x01`. Une trame de DLC 0 n'a pas de second niveau.

Conséquence : jusqu'à quelques centaines de voies. La table est dimensionnée par
`MAX_IDS_SUIVIS` (256), et `<base>/decouverte` publie `voies`, `max` et
`voies_perdues` pour signaler une saturation. L'inventaire périodique ne
republie que les voies ayant reçu de nouvelles trames, les topics étant déjà
retenus côté broker.

### Sources

- [SOREL CAN BUS Interface (SCBI) — Tasmota discussion #20598](https://github.com/arendst/Tasmota/discussions/20598)
  (script SML complet, traces CAN, débit, terminaison)
- [ddimension/can-ltdc-logger](https://github.com/ddimension/can-ltdc-logger)
  (décodeur C : identifiants, groupes, mise à l'échelle)
- [Page produit SOREL MTDC](https://www.sorel.de/en/controller/tdc-solar-controls/mtdc/)
  (4 PT1000, 2 relais, sortie PWM/0-10 V)
- [SOREL-GmbH/sorel_connect_ha](https://github.com/SOREL-GmbH/sorel_connect_ha) —
  intégration Home Assistant **officielle**, pour les appareils « Sorel Smart »
  compatibles du service Sorel Connect. Elle n'est **pas réutilisable ici** : elle
  attend que l'appareil publie lui-même en MQTT, adresse les points de données
  par registre (`…/dp/00/40037`) dans un topic à 9 segments lié au MAC et à
  l'identifiant réseau du matériel, et récupère les unités auprès de l'API
  `connect.sorel.de`. Un MTDC-E nu sur CAN n'a rien de tout cela.

  En revanche ses deux tables de constantes (`const.py`) sont de l'information
  officielle directement exploitable : `RELAY_MODES` (17 modes avec unités et
  échelles, contre 4 chez `can-ltdc-logger`) et `SENSOR_TYPES` (27 types
  d'entrée : contact, température, humidité, luminosité, rayonnement global,
  débit en L/min, fréquence, pression…). C'est de là que vient le tableau des
  modes de relais ci-dessus.

Un troisième projet, `aprospero/cansorella` (parseur SCBI en C, MTDC → MQTT),
est encore référencé par les moteurs de recherche mais le dépôt renvoie
maintenant 404 : je n'ai pas pu l'exploiter.

## Fichiers

| Fichier | Rôle |
|---|---|
| `mtdce_can_mqtt.ino` | Programme principal (WiFi, MQTT, tâche CAN, publication) |
| `config.h` | **Tous les réglages** : WiFi, broker, brochage, débit, adresse SOREL, variante |
| `decodage.h` / `decodage.cpp` | Table de décodage SCBI et extraction des signaux |

## Matériel

- **ESP32** (WROOM-32 / DevKit v1). Le WiFi est indispensable pour MQTT, un
  Arduino Uno ne convient pas ici.
- **Module MCP2515 + TJA1050** (le module chinois classique).

À noter : dans le fil Tasmota, les participants recommandent d'utiliser le
contrôleur CAN **intégré à l'ESP32** (TWAI) avec un simple transceiver plutôt
qu'un MCP2515. À 250 kbit/s le MCP2515 est largement suffisant, et il a
l'avantage d'exposer les registres d'erreur du bus, précieux au diagnostic ;
c'est le choix retenu ici.

### Câblage

| ESP32 | MCP2515 |
|---|---|
| GPIO18 | SCK |
| GPIO19 | SO (MISO) |
| GPIO23 | SI (MOSI) |
| GPIO5 | CS |
| GPIO4 | INT |
| GND | GND |
| voir ci-dessous | VCC |

Côté bus : `CANH` → CANH du MTDC-E, `CANL` → CANL, et **masse commune**.

### Trois pièges à traiter avant de brancher

1. **Terminaison.** Le MTDC-E a deux ports CAN. Si tu n'en utilises qu'un, place
   un **bouchon 120 Ω dans le port resté libre** — sans cela, rien n'est décodé
   sur le bus. C'est le point le plus souvent cité dans le fil Tasmota. Garde
   alors la résistance du module MCP2515 (elle forme la seconde terminaison). À
   l'inverse, si tu te branches en dérivation sur un bus déjà bouché aux deux
   bouts, retire celle du module.

2. **Niveaux logiques.** Le TJA1050 exige 4,75 à 5,25 V : le module doit être
   alimenté en 5 V (`VIN`), pas en 3,3 V. Deux lignes seulement posent alors
   problème, celles qui vont **du module vers l'ESP32** :

   | Ligne | Sens | Risque |
   |---|---|---|
   | `SCK`, `SI`, `CS` | ESP32 → module | aucun, l'ESP32 pilote en 3,3 V |
   | `SO`, `INT` | module → ESP32 | **oui**, ~5 V sur une entrée 3,3 V |

   Le maximum absolu d'une broche ESP32 est VDD + 0,3 V ≈ 3,6 V.

   | Solution | Problème résiduel |
   |---|---|
   | Module « 3,3 V compatible » (régulateur + `TJA1051T/3`) | aucun |
   | Module en 5 V + pont diviseur sur `SO` et `INT` (1 kΩ / 2 kΩ → 3,33 V) | seulement les 3,3 V vers les entrées du MCP2515 : sous-spec (VIH = 0,7 × VDD = 3,5 V), **sans risque de destruction**, fonctionne en pratique |
   | Double alimentation (MCP2515 en 3,3 V, TJA1050 en 5 V) | **à éviter** : la sortie `RXD` du TJA1050 bascule à 5 V sur une entrée de MCP2515 alimentée en 3,3 V. Le problème n'est pas résolu, il est déplacé à l'intérieur du module |

   Cette installation utilise la deuxième solution.

3. **Quartz.** `CAN_QUARTZ` doit correspondre au quartz soudé sur le module
   (`MCP_8MHZ` ou `MCP_16MHZ`, lisible sur le boîtier métallique). Une erreur ici
   donne un module qui s'initialise correctement mais ne reçoit **aucune** trame.

## Bibliothèques

Gestionnaire de bibliothèques de l'IDE Arduino :

- **mcp_can** par *coryjfowler* (`MCP_CAN_lib`) — pas la version Seeed, l'API diffère
- **PubSubClient** par *Nick O'Leary*

Plus le support ESP32 (Boards Manager → *esp32 by Espressif Systems*).

## Mise en service

1. Renseigner SSID / mot de passe / broker dans `config.h`.
2. Compiler et téléverser (carte : *ESP32 Dev Module*).
3. Ouvrir le moniteur série à 115200 bauds. Le démarrage affiche la détection
   automatique du débit — elle doit retenir 250k :

```
[CAN] Detection du debit...
[CAN]   250k  : 143 trame(s)
[CAN] Debit retenu : 250k
```

Si toutes les lignes affichent 0, c'est la terminaison, le câblage, le quartz ou
la polarité CANH/CANL — pas le débit.

## Diagnostic : aucune trame ne remonte

Le MTDC-E **n'émet pas en continu**. La notice (menu `6. Réglages → Réseau →
Envoi de l'intervalle du sonde`, p. 29) précise qu'une valeur n'est envoyée que
lorsqu'elle change, puis plus rien jusqu'à expiration de l'intervalle réglé —
et que si aucune valeur ne change, rien n'est envoyé. Un intervalle non
configuré est donc la première chose à vérifier sur le régulateur.

Côté passerelle, `<base>/etat/stats` expose les registres d'erreur du MCP2515,
qui distinguent les causes :

| Observation | Interprétation |
|---|---|
| `trames` monte | tout va bien, le reste est du décodage |
| `mode_ok:false` | le MCP2515 **n'a pas pris le mode demandé** et reste probablement en loopback : il ne reçoit alors que ses propres émissions, soit zéro trame et zéro erreur. Piège de la bibliothèque `mcp_can`, dont `begin()` laisse la puce en `MCP_LOOPBACK` ; `mode_ok` est le retour de `setMode()`, qui relit `CANSTAT` pour confirmer — c'est du matériel, pas une constante de compilation |
| `erreurs:"aucune"`, `rec:0`, `tec:0`, `trames:0`, `mode_ok:true` | le bus est **électriquement silencieux** : personne n'émet, ou le transceiver n'est pas relié à la paire. Voir l'intervalle d'envoi du régulateur, puis la mesure des 60 Ω |
| `rec` monte, `erreurs` contient `EWARN`/`RXWAR`/`RXEP` | il y a du signal mais il n'est pas décodable : mauvais débit, quartz erroné, ou H/L inversés |
| `BUSOFF` | le contrôleur s'est retiré du bus, en général faute d'acquittement |
| `debordements` contient `RX0OVR`/`RX1OVR` | tampon de réception débordé, la tâche CAN n'a pas suivi |

`rec` et `tec` sont les compteurs d'erreurs normalisés du CAN : au-dessus de 96
les drapeaux d'avertissement apparaissent, au-dessus de 127 le nœud passe
passif, à 256 il se retire du bus.

Les débordements sont publiés dans un champ **distinct** des erreurs, car ils ne
se comportent pas pareil : les bits d'erreur suivent les compteurs et
redescendent d'eux-mêmes, alors que les bits de débordement **restent collés**
jusqu'à une remise à zéro du composant.

En fonctionnement normal, la réception par interruption n'en produit aucun —
`debordements` doit rester à `aucune`. S'il apparaît, c'est que des trames sont
perdues *avant* d'arriver dans la file, et le compteur `perdues` ne le verra
pas : il ne compte que les débordements de la file interne, pas ceux des tampons
du MCP2515.

Seul un **redémarrage** efface durablement le drapeau. La commande `scan`
réinitialise bien le composant, mais elle suspend la tâche CAN pendant la
détection de débit, ce qui provoque aussitôt un nouveau débordement : ne
l'utilise pas pour nettoyer l'indicateur, utilise `reboot`.

## Perte de réseau : que fait la passerelle ?

La reconnexion est automatique, à trois niveaux, et **sans limite de tentatives**.

1. **Reconnexion native de l'ESP32** (`setAutoReconnect`) : retente en continu le
   dernier SSID connu, en quelques secondes.
2. **Reparcours complet de la liste** : si le WiFi est absent depuis plus de
   `WIFI_RELANCE_MS` (60 s), la recherche repart de zéro et essaie chaque réseau
   de `RESEAUX[]` pendant `WIFI_ESSAI_MS` (12 s). Indispensable : le niveau 1 ne
   retente que le *dernier* réseau, donc il ne trouve pas le repli si c'est
   celui-là qui est tombé.
3. **MQTT** : dès que le WiFi est revenu, une nouvelle session est tentée toutes
   les 5 s. À la reconnexion, `online` est republié en retained et l'abonnement
   aux commandes rétabli.

Délai de retour au pire : 60 s de détection + 24 s de balayage + 5 s pour MQTT,
soit environ **90 s**. Aucun redémarrage n'est déclenché : les attentes
utilisent `delay()`, qui rend la main, donc aucun chien de garde ne se réveille.

Pendant la coupure, la file CAN **continue d'être vidée** et le suivi des voies
se poursuit ; seules les publications MQTT de la fenêtre sont perdues, et
`<base>/etat` passe à `offline` par le *last will*.

### Compteurs de supervision

Pour une installation laissée seule, `uptime_s` ne dit que s'il y a eu
redémarrage — pas combien de fois la liaison est tombée. D'où quatre champs :

| Champ | Ce qu'il révèle |
|---|---|
| `connexions_wifi` | liaisons WiFi établies depuis le démarrage. À `1`, aucune coupure. Compté sur la transition, donc les retours dus à la reconnexion native sont inclus |
| `connexions_mqtt` | sessions MQTT établies. Monte plus vite que le précédent si c'est le broker ou la qualité du lien qui lâche, pas le WiFi |
| `demarrages` | compteur **persistant** en mémoire non volatile : le seul témoin qui survit à une coupure secteur, là où `uptime_s` repart de zéro |
| `raison_reset` | cause du dernier démarrage : `mise sous tension` (coupure secteur), `brownout` (alimentation faible), `panique logicielle`, `chien de garde`, `reset logiciel` (OTA ou commande `reboot`) |

C'est le couple `demarrages` + `raison_reset` qui distingue une coupure de
courant d'un plantage — deux événements indiscernables autrement.

## Topics MQTT

Base configurable, par défaut `mtdc/mtdce`.

| Topic | Contenu |
|---|---|
| `<base>/etat` | `online` / `offline` (retained, LWT) |
| `<base>/etat/stats` | uptime, RSSI, SSID, IP, débit, mode, trames, trames perdues, RAM, les registres d'erreur du MCP2515 (`eflg`, `erreurs`, `debordements`, `rec`, `tec`) et les compteurs de supervision (`connexions_wifi`, `connexions_mqtt`, `demarrages`, `raison_reset`) |
| `<base>/brut/<id>/<index>` | trame brute : `{"id":"0x10011A80","mux":"0x00","dlc":5,"data":"00 BE 00 00 00","periode_ms":32555,"compte":14}` |
| `<base>/decouverte/<id>/<index>` | inventaire par voie : période, occurrences, dernier contenu, `connu` |
| `<base>/decouverte` | résumé : `voies`, `max`, `voies_perdues`, débit |
| `<base>/valeur/capteurs/s1`…`s4` | températures en °C |
| `<base>/valeur/relais/…` | état des relais |
| `<base>/valeur/statistiques/…` | puissance, rendement (variante B) |
| `<base>/valeur/bilan/…` | bilans jour / semaine / mois / année / total (variante B) |
| `<base>/cmd` | commandes entrantes |

Anti-flood : un identifiant donné n'est jamais republié plus souvent que
`INTERVALLE_MIN_MS` (1 s), et l'est au moins toutes les `INTERVALLE_MAX_MS`
(60 s) même sans changement.

### Commandes (`<base>/cmd`)

| Payload | Effet |
|---|---|
| `brut off` / `brut on` | coupe / rétablit la publication des trames brutes |
| `decouverte` | republie l'inventaire immédiatement |
| `reset` | vide le tableau des identifiants observés |
| `scan` | relance la détection automatique du débit |
| `reboot` | redémarre l'ESP32 |

```bash
mosquitto_sub -h <IP_DU_BROKER> -u <UTILISATEUR> -P <MOT_DE_PASSE> -v -t 'mtdc/#'
```

Aucune commande n'écrit sur le bus CAN — le MCP2515 n'y place que le bit
d'acquittement, jamais une trame. C'est volontaire : émettre vers un
MTDC-E demande de connaître la partie commande du protocole SCBI, et une trame
mal formée sur un bus de régulation peut avoir des conséquences physiques.

## Vérifier le décodage

Au premier démarrage, compare les deux flux :

```bash
mosquitto_sub -h <IP_DU_BROKER> -u <UTILISATEUR> -P <MOT_DE_PASSE> -v -t 'mtdc/mtdce/decouverte/#'
mosquitto_sub -h <IP_DU_BROKER> -u <UTILISATEUR> -P <MOT_DE_PASSE> -v -t 'mtdc/mtdce/valeur/#'
```

1. **Relever l'adresse réelle.** Les identifiants remontés dans `decouverte`
   ont la forme `0x1001XX80` : `XX` est l'adresse du régulateur. Tu peux la
   figer dans `config.h` (`SOREL_ADRESSE_AUTO 0` + `SOREL_ADRESSE 0xXX`).
2. **Identifier la variante.** Si `<base>/valeur/capteurs/s1` publie des
   températures plausibles, la variante active est la bonne. Si rien n'apparaît
   alors que `<base>/brut/0x1001XX80` défile, c'est que la trame de ton MTDC-E
   ne correspond à aucune des deux : relève quelques trames et le décodage se
   décrit en trois lignes dans `decodage.cpp`.
3. **Croiser avec l'afficheur du régulateur.** C'est la seule vérification qui
   vaille pour l'affectation index → capteur.
4. Une fois les valeurs confirmées, passer `HA_DISCOVERY` à `1` pour créer les
   capteurs Home Assistant, et éventuellement `brut off` pour alléger le trafic.

### Ajouter ou corriger un signal

Le format reprend celui d'un fichier DBC, avec deux ajouts propres à SCBI : un
**masque d'identifiant** (pour ignorer l'octet d'adresse) et un **préfixe**
(pour le multiplexage — un même ID CAN transporte plusieurs grandeurs).

```cpp
static const Signal SIG_A_S1[] = {
  // nom  bit  len  motorola  signé  facteur  offset  unité   classe HA
  { "s1",  16,  16, false,    true,  0.1f,    0.0f,   DEG_C,  "temperature" },
};

// { identifiant, masque, étendu, {préfixe}, longueur préfixe, nom, signaux, n }
{ SOREL_ID(GRP_CAPTEURS), SOREL_MASQUE, true, {0x05, 0x00, 0, 0}, 2,
  "capteurs", SIG_A_S1, NB_SIG(SIG_A_S1) },
```

`bitDepart` est le bit de poids faible en Intel (little-endian), le bit de poids
fort en numérotation Motorola.

## Notes d'implémentation

- Une **tâche FreeRTOS** dédiée (cœur 0, priorité 3) vide les deux tampons de
  réception du MCP2515 vers une file de 256 trames. Sans elle, une publication
  MQTT un peu lente ferait déborder le contrôleur et perdre des trames
  silencieusement. Le compteur `perdues` signale une saturation de la file.

- La file est vidée **quel que soit l'état du réseau**, seule la publication est
  conditionnée. C'est un correctif : la version précédente sortait de `loop()`
  tant que le WiFi était absent, si bien que la file se remplissait en deux
  minutes et perdait des trames en silence. Avec un signal à −84 dBm, une nuit
  de liaison instable avait suffi à afficher `perdues: 33399` pour
  112 625 trames reçues, soit près d'un quart du trafic. Le suivi des voies
  continue donc hors connexion ; seules les publications MQTT sont perdues, ce
  qui est inévitable, et `<base>/etat` passe à `offline` pour le signaler.

- Cette tâche est réveillée par une **interruption** sur la broche `INT`, et non
  par une attente périodique. La différence est mesurable : le SCBI émet par
  rafales de trames consécutives, séparées d'environ 0,5 ms à 250 kbit/s. Avec
  un `vTaskDelay`, une rafale de trois trames remplissait les deux tampons du
  MCP2515 avant le réveil de la tâche, ce que trahissait un `RX0OVR` permanent
  dans `debordements`. Avec l'interruption, la latence tombe à quelques
  microseconds et le drapeau reste propre.
- Les températures sont décodées en **entier signé**, pour que le gel donne une
  valeur négative plutôt que ~6500 °C. Les sources de référence utilisaient un
  entier non signé.
- **OTA** activé : mises à jour par WiFi (`mtdce-gateway.local` dans l'IDE), utile
  une fois le boîtier en place.
- La reconnexion WiFi et MQTT est automatique, avec `online`/`offline` retenus
  pour que le broker signale une passerelle absente.
