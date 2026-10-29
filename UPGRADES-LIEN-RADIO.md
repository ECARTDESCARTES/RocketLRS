# RocketLRS — Plan d'upgrades débit et portée du lien radio

> Analyse du 29/10/2026. Les gains sont **calculés** (formule de temps d'antenne Semtech SX1280, sensibilités de la fiche technique) et ont été relus par des contre-vérificateurs chargés de les réfuter. **Aucun chiffre n'a été mesuré sur le matériel** : les modules n'ont pas encore été flashés. Les points marqués *à vérifier* restent à confirmer sur la fiche technique ou au banc.

## 1. État actuel

### Transport A — pont autonome (`src/examples/rlrs-radio/main.cpp`)

| Paramètre | Valeur actuelle | Conséquence |
|---|---|---|
| Modulation | LoRa SF7, bande 812 kHz, CR 4/7, préambule 12 | Sensibilité d'environ −112 dBm |
| Trame | Longueur fixe de 69 o, dont le remplissage par des zéros | 24,77 ms d'antenne par trame, quelle que soit la taille du message |
| Port série | 9600 bauds, 8N1 (`main.cpp:198`) | **960 o/s au maximum** : premier goulot |
| Puissance | `POWER_OUTPUT_VALUES[0]` (`main.cpp:102`) | Premier niveau du profil, donc le plus bas |
| Fréquence | 2440 MHz fixe, sans saut de fréquence | Recouvre le canal Wi-Fi 6 (2437 MHz) |
| Wi-Fi | Point d'accès laissé actif | Brouillage sur la même bande de 2,4 GHz |
| Accès au canal | Half-duplex, sans créneaux, sans retransmission | Collisions possibles entre les deux sens |

- À 27 messages/s, l'antenne est occupée 669 ms par seconde (67 %).
- Portée de base estimée : environ **6,2 km** en espace libre, avec 10 dB de marge.

### Transport B — RlrsLink sur le lien ExpressLRS (`src/lib/RocketLRS/RlrsLink.*`, `src/lib/OTA/OTA.cpp`)

- Les messages RocketLRS sont glissés dans les paquets DATA d'ExpressLRS : **10 o** par paquet au format OTA8, **5 o** au format OTA4.
- Les profils `RlrsRadioPreset` (FlightLoRa / TransferFLRC) de `RlrsProfiles.h` ne sont **branchés nulle part**.

### Besoin réel

Environ **320 à 340 o/s**, en-têtes RocketLRS de 5 o compris :

| Flux | Fréquence |
|---|---|
| Gyro | 20 Hz |
| GPS | 5 Hz |
| Pression | 1 Hz |
| Batterie | 1 Hz |
| Commandes (armement, parachute) | Ponctuelles, sens sol → fusée |

## 2. Quick wins

| # | Upgrade | Gain | Effort | Où |
|---|---|---|---|---|
| 1 | Port série 9600 → 115200 bauds | 960 → 11 520 o/s (×12) ; une trame de 69 o passe de 71,9 à 6,0 ms sur le port série | 1 ligne + réglage côté Java/STM32 | `main.cpp:198` |
| 2 | Corriger le bug des fragments quand la place restante est de 1 octet *(probable)* | Les messages découpés ne se perdent plus ; le format OTA4 redevient utilisable | Environ 1 ligne | `RlrsLink.cpp:378-388` |
| 3 | Longueur de trame variable + préambule 8 + CR 4/5 | 24,77 → 7,13 ms pour 20 o (−71 %) | En-tête explicite ou octet de longueur | `main.cpp` (`Config`) |
| 4 | Regrouper plusieurs messages par trame (fenêtres de 50 ms) | Antenne occupée 669 → environ 253 ms/s (−62 %). **Prérequis** des gains de portée | Parseur multi-messages | `main.cpp`, `RlrsParser` |
| 5 | Transport B : télémétrie 1 paquet sur 2 à 100 Hz, format OTA8 | Environ 31 → 470 o/s (×15) : gyro et GPS couverts | Un paramètre ; sens sol → fusée réduit à 50 Hz | `rx_main.cpp:506-556` |
| 6 | Couper le Wi-Fi en vol (`WiFi.mode(WIFI_OFF)`) et quitter 2440 MHz | Moins de brouillage (non chiffré) | 1 ligne | `main.cpp` |

**Bug n°2** : quand la place restante vaut 1 octet, l'émetteur écrit un fragment vide (chunk = 0) et fait quand même avancer son numéro de séquence. Le scénario exact de désynchronisation côté réception n'a pas encore été rejoué en test.

## 3. Gains de portée

| # | Upgrade | Gain | Coût / condition |
|---|---|---|---|
| 7 | Transport B : mode 100 Hz Full SF7, 2 mesures gyro par paquet | +7 dB par rapport à 500 Hz : portée **8,7 → 19,5 km**. Charge de 42 paquets sur 50 par seconde | Latence gyro de 100 ms |
| 8 | Puissance maximale (100 mW / 20 dBm) + saut de fréquence | **+8 dB** dans chaque sens (portée ×2,5), dans la limite de 20 dBm PIRE | Valeur réelle de l'index 0 et amplificateur du RP1 *à vérifier* ; sans amplificateur sur le RP1, seulement +2,5 dB dans le sens fusée → sol. Saut de fréquence obligatoire au-delà de 10 dBm (EN 300 328, *à vérifier*) |
| 9 | Antenne hélice à polarisation circulaire + suiveur au sol | +8 dB en réception (13 dBi − 3 dB de pertes linéaire/circulaire − 2 dBi). En émission, plafonné par la PIRE | 50 à 150 € |
| 10 | Transport A : SF8 avec trames de 24 o | +3 dB (−115 contre −112 dBm), 18,7 ms par trame, canal occupé à 52 % | Longueur variable requise (n°3) |
| 10 bis | Transport A : SF9 | +5 dB | **Seulement** en regroupant les mesures gyro par 4 ; sans cela, le canal est occupé à 92 % |

## 4. Pistes écartées

| Piste | Pourquoi |
|---|---|
| Mode 333 Hz Full | C'est du SF5 à −105 dBm : aucun gain de portée |
| +13 / +17 dB de puissance | Dépasse la limite légale de 20 dBm PIRE |
| Paquet sol → fusée de 11 o sans statistiques de lien | Faux : 10 o est déjà le maximum |
| LR1121 en 2,4 GHz | Régression : environ −7 dB en émission, et pas de FLRC |
| 915 MHz | Bande américaine, illégale en France |
| FLRC en vol | Moins sensible que LoRa SF7 (de −8 à −16 dB selon le vérificateur, *à vérifier*). À garder pour le transfert au sol |
| Boost du récepteur (registre 0x891) | Déjà actif dans `SX1280.cpp:92` : aucun gain à attendre |
| SF9/BW406 à 10 Hz, SF10/BW406 à 5 Hz | Canal occupé à 87 % et 114 % : impossible en half-duplex |

## 5. Architecture cible

- **Lien principal 2,4 GHz** en créneaux temporels alternés (TDMA) avec saut de fréquence. Créneaux de 33 o au maximum. Le sens sol → fusée est réservé à l'armement et au parachute.
- **Modulation de vol** : SF8, ou SF9 avec regroupement des mesures gyro par 4. Le FLRC sert uniquement au transfert de fichiers au sol.
- **Profils** `RlrsRadioPreset` réellement branchés : choix de la modulation selon la phase de vol (désarmé, armé, après le vol, transfert).
- **Antennes** : hélice à polarisation circulaire au sol sur un suiveur ; deux antennes côté fusée (gain *à vérifier*).
- **Option longue portée** : second lien 868 MHz, sous-bande g3 (LR1121 + amplificateur, antenne de 8,6 cm), dédié au GPS.
  - Gain théorique d'environ **+24 dB** dans le sens fusée → sol : +9 dB d'affaiblissement en moins, +13 dB de sensibilité, +2 dB en émission.
  - Contraintes : deux puces radio dans la fusée et un temps d'émission limité à 10 % par heure.

**Cumul réaliste dans le sens sol → fusée : environ +13 dB, soit près de 28 km théoriques.** Le sens fusée → sol est équivalent, ou meilleur avec l'antenne hélice.

## 6. Prochaines étapes

1. Lire la vraie valeur de `POWER_OUTPUT_VALUES[0]` et la puissance maximale du RP1 dans leurs profils matériels.
2. Appliquer les quick wins 1, 3, 4 et 6 dans le pont autonome.
3. Écrire un test natif qui reproduit le bug des fragments, puis le corriger.
4. Faire un test de portée au sol, en relevant le RSSI, le SNR et le taux de paquets perdus selon la distance.
5. Brancher les profils `RlrsRadioPreset`, puis passer aux créneaux alternés avec saut de fréquence.
6. Revérifier toutes les sensibilités sur la fiche Semtech SX1280 rév. 3.2.

## 7. État d'implémentation (29/10/2026)

| Proposition | État | Où |
|---|---|---|
| Port série à 420000 bauds | Fait (`RLRS_UART_BAUD`) | `examples/rlrs-radio/main.cpp` |
| Bugs de fragmentation | Corrigés (épissure de messages, reconnexion, débordement du compteur d'ordre, réutilisation des numéros de séquence) | `lib/RocketLRS/RlrsLink.*` |
| Agrégation de messages | Faite dans les deux transports | `RlrsLink`, `RlrsAir` |
| Longueur variable, préambule 8, CR 4/5, CRC matériel | Fait | `RlrsProfiles.h`, `main.cpp` |
| Créneaux alternés + saut de fréquence sur 40 canaux, bascule de profil | Fait (sol maître, fusée esclave) | `lib/RocketLRS/RlrsHop.*` |
| Profil de vol SF8 / profil de transfert FLRC branchés | Fait (message 0xF1) | `RlrsProfiles.h`, `main.cpp` |
| Puissance maximale | Fait, plafonnée à 20 dBm PIRE (`RLRS_ANTENNA_GAIN_DB`) | `main.cpp` |
| Wi-Fi coupé | Fait (`RLRS_KEEP_WIFI` pour le garder) | `main.cpp` |
| Statistiques de lien renvoyées au sol (RSSI, SNR, LQ) | Fait (message 0xF0, toutes les secondes) | `main.cpp` |
| Lien ExpressLRS : 100 Hz Full + télémétrie 1:2 par défaut | Fait (`RLRS_LINK_DEFAULTS`) | `lib/CONFIG/config.*` |
| Accès concurrents entre interruptions et boucle principale, placement en IRAM | Corrigés | `RlrsLink`, `OTA.cpp`, `main.cpp` |
| Antenne hélice + suiveur au sol | Matériel : à acheter | — |
| Second lien 868 MHz (LR1121) | Non faisable : le RP1 n'a pas de LR1121 | — |

Vérifié : 4 builds PlatformIO réussis (ESP32 TX et ESP8285 RX, en version Wi-Fi et UART). Tous les tests sur PC passent : `test_rlrs_link`, `test_rlrs_air` (5620 vérifications), `test_rlrs_bridge`, `examples/rlrs-radio`.

**Rien n'a été flashé ni mesuré sur les modules.** À valider sur le matériel :
- le délai réel entre la minuterie et le début d'émission (marge de synchronisation de 106 µs dans le pire cas simulé) ;
- `SetPacketParams` envoyé en mode FS ;
- la longueur lue par `GetRxBufferStatus` ;
- les valeurs `POWER_OUTPUT_VALUES` réelles ;
- la sensibilité FLRC.
