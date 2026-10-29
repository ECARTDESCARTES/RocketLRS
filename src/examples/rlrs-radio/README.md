# Pont radio RocketLRS

Ce firmware remplace le site de démonstration par un **pont série ↔ radio ↔ série**. Il transporte les paquets RocketLRS définis dans `microfusee/protocole.md` et lus par le parseur Java du contrôleur NLFC.

- **Sol** : module TX RadioMaster Ranger Nano, ESP32, `TARGET_TX`. Il est relié au programme Java par USB et joue le rôle de **maître** RlrsHop.
- **Fusée** : récepteur RP1, ESP8285, `TARGET_RX`. Il est relié au STM32 par UART et joue le rôle d'**esclave** RlrsHop.

Le code est dans `main.cpp`. La couche air vient de `lib/RocketLRS/RlrsAir.*` et `RlrsHop.*`, les presets radio de `lib/RocketLRS/RlrsProfiles.h`.

## Trajet d'un message

1. Le programme Java (ou le STM32) écrit sur l'UART des paquets RocketLRS : `AA, longueur, ID, payload, CRC haut, CRC bas`.
2. Le pont vérifie le CRC avec `RlrsParser`, puis place le message (ID + payload) dans la file `RlrsAirQueue`. La file classe les messages par priorité : commandes et ACK d'abord, puis heartbeat, LED, servo et LinkStats, puis la télémétrie, puis le reste.
3. À son créneau, le pont regroupe plusieurs messages dans une seule trame radio de longueur variable.
4. En face, le pont lit la trame avec `RlrsAirReader`. Il reconstruit chaque paquet RocketLRS avec son `AA` et son CRC (`rlrsEncode`), puis l'écrit sur son UART.

Le pont ne fabrique pas de réponse métier. Par exemple, le heartbeat Java (ID `0x00`, payload `02`) passe tel quel, et c'est au STM32 d'y répondre.

## Format air (RlrsAir)

```
[seq][flags][hop si HOP][preset, switchSeq si PRESET] puis {[len][id][payload]}*
```

- `flags` : `HOP` = 0x01 (balise du maître), `PRESET` = 0x02 (annonce d'un changement de preset), `ACK` = 0x04 (accusé de l'esclave).
- L'en-tête fait de 2 à 5 octets. Chaque message ajoute 2 octets (`len`, `id`) à son payload.
- La longueur de trame est portée par l'**en-tête explicite** du SX1280. Le CRC matériel est actif : 16 bits en LoRa, 24 bits en FLRC. Le CRC RocketLRS n'est pas transmis, il est recalculé à la réception.
- Il n'y a ni fragmentation ni retransmission radio. Une trame perdue perd ses messages, et l'autorité des commandes reste le couple CMD/ACK RocketLRS.

Tailles maximales :

| Sens | Trame entière | Payload d'un message |
|---|---|---|
| Sol → fusée | 33 o | 26 o (au-delà, le message est refusé) |
| Fusée → sol | 72 o en LoRa, 127 o en FLRC | 64 o |

## TDMA et saut de fréquence (RlrsHop)

- Une trame dure 50 ms, soit 20 trames par seconde.
- Au début de chaque trame, le maître (sol) émet sa balise sur le canal k. La balise contient le numéro de séquence, l'index de saut et les commandes.
- L'esclave (fusée) attend 500 µs après la balise, puis répond sur le même canal. Sa trame est taillée pour finir avant la fin de la trame TDMA, moins une garde de 1 ms.
- Les deux côtés passent ensuite au canal k + 1. Il y a 40 canaux de 2400,4 à 2479,6 MHz. Leur ordre est permuté par une graine 32 bits, utilisée une fois par cycle. La fréquence est réglée avec `SetFrequencyReg` avant chaque émission et chaque écoute.
- La graine vient de l'UID de la phrase de binding si elle existe. Sinon, c'est une valeur fixe (`0x524C5253`). `-DRLRS_LINK_SEED=...` l'impose. **Les deux modules doivent avoir la même graine.**
- **Acquisition** : l'esclave écoute sur un canal fixe jusqu'à recevoir la balise de ce canal, puis il suit le maître. Après 20 balises manquées (1 s), il repasse en acquisition.
- **Horloge** : RlrsHop est piloté par `loop()` avec `micros()`, et non par le `hwTimer` d'ELRS. RlrsHop et RlrsAir ne sont pas du code d'interruption : il faudrait les placer en IRAM, qui est rare sur l'ESP8285, et faire le SPI dans l'ISR du timer. L'ISR DIO1 du driver fait elle-même du SPI (`GetIrqStatus`, lecture du FIFO, `ClearIrqStatus`) ; nos callbacks y ajoutent `GetRxBufferStatus`, datent TxDone et RxDone avec `micros()` et déposent la trame dans une boîte aux lettres (variables `volatile`, lues sous `noInterrupts()`). La synchronisation sur la balise ne dépend donc pas de la gigue de `loop()`, qui ne contient plus de `delay()`.
- **Exclusion SPI** : le HAL SX1280 n'est pas réentrant. DIO1 ne se lève que sur TxDone/RxDone, donc quand la radio est en TX ou en RX. Chaque séquence SPI de `loop()` commence par `radioIdle()` : passage en FS (`SetTxIdleMode`) sous `noInterrupts()`, quelques µs. Une ISR arrivée pendant ce temps s'exécute dès `interrupts()`, avant la commande SPI suivante ; ensuite la radio reste muette jusqu'à `TXnb()`/`RXnb()`, dernière commande de la séquence. `Config()` (environ 2 ms) tourne donc interruptions actives.
- **Latence RxDone** : `RLRS_RX_LATENCY_US` (150 µs par défaut : entrée ISR + `GetIrqStatus` + lecture du FIFO) est retranchée à la datation de la balise. Le maître écoute la voie descendante jusqu'à `endGuard / 2` (500 µs) avant la trame suivante : l'autre moitié de la garde absorbe le retard de datation de l'esclave (départ TX du maître, latence IRQ non compensée, départ TX de l'esclave), jusqu'à 500 µs au total.
- **IRQ HEADER_ERROR** : `Config()` ne l'active pas dans le masque, et c'est voulu. Une erreur d'en-tête LoRa termine la réception sans RX_DONE ; le drapeau, non routé sur DIO1, resterait levé jusqu'au prochain paquet correct et le ferait rejeter. Seuls RX_DONE et CRC_ERROR sont testés.
- **Temps d'antenne** en SF8, BW 812,5 kHz, CR 4/5 : balise de 8 o en 11,1 ms, réponse de 48 o en 26,9 ms. La trame complète tient en 39,5 ms sur 50 ms.

## Presets radio (`RlrsProfiles.h`)

| Preset | Index | Modulation | Usage |
|---|---|---|---|
| `FlightLoRa` | 0 (défaut) | LoRa SF8, BW 800 (812,5 kHz, `SX1280_LORA_BW_0800`), CR 4/5 non entrelacé (`SX1280_LORA_CR_4_5`), préambule 8, en-tête explicite, CRC ON | Vol, environ −115 dBm |
| `TransferFLRC` | 1 | FLRC 1,3 Mbit/s (`SX1280_FLRC_BR_1_300_BW_1_2`), CR 1/2, BT 1.0, préambule 32 bits, mot de synchro = graine, CRC 24 bits, longueur variable | Transfert au sol uniquement : moins sensible que LoRa |

- La table `RLRS_RADIO_PRESETS` décrit chaque preset de façon indépendante de la puce. `main.cpp` la convertit en registres SX1280.
- `SX1280Driver::Config()` force un paquet de longueur fixe et coupe le CRC LoRa. Après chaque `Config()`, le pont réécrit donc les paramètres de paquet (`SetPacketParams`) pour obtenir l'en-tête explicite, la longueur variable et le CRC matériel. Il les réécrit aussi avant chaque émission (longueur exacte) et chaque écoute (longueur maximale acceptée).
- À la réception, la vraie longueur est lue avec `GetRxBufferStatus`. En LoRa, les drapeaux CRC et en-tête sont vérifiés dans le callback, car le driver ne les contrôle qu'en FLRC.
- RlrsHop ne modélise que LoRa. Pour le FLRC, il utilise un modèle LoRa (SF5, BW 1625 kHz) dont le temps d'antenne est toujours supérieur ou égal à celui du FLRC : les créneaux sont donc dimensionnés large. Le pont corrige la date de synchronisation avec `rlrsHopSyncShiftUs()`.

### Changer de preset

Pour changer de preset, le programme Java ou le STM32 envoie le message RocketLRS `0xF1` (`RLRS_PROFILE_REQUEST_ID`), avec `payload[0]` = profil :

| Valeur | Profil | Preset |
|---|---|---|
| 0 | Désarmé | `FlightLoRa` |
| 1 | Armé | `FlightLoRa` |
| 2 | Après le vol | `FlightLoRa` |
| 3 | Transfert | `TransferFLRC` |

- Le pont sol intercepte ce message : il ne le transmet à aucun UART. Il appelle alors `RlrsHop::requestPreset()`.
- Le maître annonce le changement pendant 10 trames. L'esclave accuse réception, puis les deux basculent dans la même trame.
- Si le lien est coupé plus de 1 s, les deux côtés reviennent au preset par défaut (`FlightLoRa`).
- La demande est refusée si le lien est coupé ou si un changement est déjà en cours.

## Statistiques de lien (LinkStats, ID `0xF0`)

Chaque seconde, le pont sol écrit sur l'UART Java un message `0xF0` de 8 octets, au format de `RlrsLink::enqueueLinkStats` :

| Octet | Contenu |
|---|---|
| 0, 1, 2 | Montée (sol → fusée) : RSSI (en −dBm), SNR (dB), LQ (%), mesurés par la fusée |
| 3, 4, 5 | Descente (fusée → sol) : RSSI, SNR, LQ, mesurés au sol |
| 6 | Niveau de puissance du sol (`PowerLevels_e` : 3 = 100 mW) |
| 7 | Preset actif |

- La fusée mesure la montée sur les balises reçues. Elle envoie ces valeurs par radio toutes les secondes dans un `0xF0`.
- Le pont sol intercepte ce message et le fusionne avec sa propre mesure. Si la mesure de la fusée a plus de 3 s, les champs de montée valent 0.
- La LQ est le pourcentage de trames valides sur la période. Le RSSI et le SNR sont des moyennes sur les trames valides.
- Le STM32 ne reçoit pas de `0xF0`.

## Port série

- Débit : **420000 bauds, 8N1** par défaut (comme le CRSF d'ELRS), réglable avec `-DRLRS_UART_BAUD=...`. **Le programme Java et le STM32 doivent être réglés sur la même valeur.** STM32 (USART, suréchantillonnage 16) : BRR = f_horloge / 420000, soit 200 exactement à 84 MHz et 0,25 % d'erreur à 72, 48, 16 ou 8 MHz ; vérifier que l'erreur reste sous 2 %. ESP8266 : diviseur entier, 80 MHz / 190 = 421053 bauds (+0,25 %). Côté PC, l'adaptateur USB-série doit accepter un débit non standard : CH340, CP2102N et FTDI conviennent ; un ancien CP2102/CP2104 peut demander de reprogrammer sa table d'alias de débits. Avant ce changement, le pont tournait à 9600 puis 115200 bauds.
- Sur le Ranger Nano et le RP1, le pont utilise l'UART0 aux broches du profil matériel. Relier TX et RX croisés, avec une masse commune et des niveaux logiques compatibles.
- N'envoyer aucun log sur cet UART : il transporte des trames binaires. Les logs vont dans un `NullStream`.

## Puissance d'émission

- Par défaut, le pont prend le niveau calibré le plus haut du profil matériel, c'est-à-dire le dernier index de `POWER_OUTPUT_VALUES` (l'index 0 correspond à `power_min`).
- Ce niveau est plafonné par `power_max`, par `-DRLRS_POWER_INDEX=n` s'il est défini, et par **20 dBm PIRE** (EN 300 328, 2,4 GHz, FHSS actif). Les niveaux `PowerLevels_e` sont des puissances **conduites** (100 mW = 20 dBm au connecteur d'antenne, valeur nominale), pas des PIRE : le pont garde le plus haut niveau tel que puissance conduite + `RLRS_ANTENNA_GAIN_DB` <= 20 dBm. Avec `-DRLRS_ANTENNA_GAIN_DB=0` (défaut) : 100 mW conduits ; avec 3 dBi : 50 mW.
- Le plafond de 20 dBm PIRE ne se lève qu'avec `-DRLRS_POWER_ALLOW_ABOVE_100MW`.
- EN 300 328 (bande 2,4 GHz) limite la PIRE à 20 dBm, gain d'antenne compris. Au-delà de 10 dBm, le **saut de fréquence actif** est la condition de conformité. Ce pont saute en permanence sur 40 canaux, il n'émet jamais sur un canal fixe. Avec une antenne de plus de 0 dBi, régler `-DRLRS_ANTENNA_GAIN_DB=<gain en dBi>` : le plafond est recalculé automatiquement.
- La nouvelle puissance n'est appliquée qu'après la première émission : la première trame part au niveau minimal réglé par `Begin()`.
- Les profils à commande de puissance externe (`power_control` ≠ 0, DAC) sont refusés.

## Wi-Fi

`setup()` appelle `WiFi.persistent(false)` puis `WiFi.mode(WIFI_OFF)`, et en plus `forceSleepBegin()` sur l'ESP8285. Le SDK restaure en effet au démarrage le dernier mode enregistré en flash : le point d'accès restait actif sur la même bande de 2,4 GHz. Pour garder le Wi-Fi, compiler avec `-DRLRS_KEEP_WIFI`.

## Compilation

Les deux cibles Wi-Fi habituelles sélectionnent ce pont (`targets/rlrs-radio-test.ini`) :

- TX Java : `Unified_ESP32_2400_TX_via_WIFI` → `.pio/build/Unified_ESP32_2400_TX_via_WIFI/firmware.bin`
- RX STM32 : `Unified_ESP8285_2400_RX_via_WIFI` → `.pio/build/Unified_ESP8285_2400_RX_via_WIFI/firmware.bin.gz`

`SX12XX_RX_BUFFER_SIZE` vaut 128. Il doit rester supérieur ou égal à `RLRS_RADIO_FRAME_LIMIT` (127) : un `static_assert` le vérifie.

**Les deux modules doivent être reflashés ensemble.** Ce pont n'est pas compatible avec l'ancienne trame fixe de 69 o.

## Retour arrière

- **Revenir aux cibles ELRS** : commenter la ligne `targets/rlrs-radio-test.ini` dans `extra_configs` de `platformio.ini`. Elle doit rester **la dernière** de la liste : ses sections remplacent celles d'ELRS pour les deux boutons `Unified_ESP32_2400_TX_via_WIFI` et `Unified_ESP8285_2400_RX_via_WIFI`.
- **Revenir à l'ancien pont** (trame fixe de 69 o, 2440 MHz, 9600 bauds) : restaurer la version précédente de `examples/rlrs-radio/main.cpp`, remettre `-DSX12XX_RX_BUFFER_SIZE=69` dans `targets/rlrs-radio-test.ini`, et remettre le Java et le STM32 à 9600 bauds.
- La configuration séparée `platformio.radio-test.ini` (même pont, `SX12XX_RX_BUFFER_SIZE=128`) reste utilisable avec `pio run -c platformio.radio-test.ini`.

## Vérification

Ce pont n'a **pas** été testé sur le matériel : rien n'a été compilé avec PlatformIO ni flashé.

Le banc `test/test_rlrs_bridge/` a été compilé et exécuté sur PC (g++ `-std=c++11 -Wall -Wextra -Werror -fno-exceptions -fno-rtti`), avec des bouchons pour Arduino, le Wi-Fi, les options et le driver SX1280. Le test fait tourner le pont, en rôle sol puis en rôle fusée, contre un pair RlrsHop simulé. Il couvre :

- l'acquisition et le passage de `micros()` par zéro ;
- les messages dans les deux sens ;
- le passage en FLRC puis le retour en LoRa ;
- 30 % de pertes simulées ;
- les LinkStats fusionnées ;
- la puissance plafonnée à 100 mW ;
- le Wi-Fi coupé et le débit à 420000 bauds ;
- des latences injectées avec gigue : départ TX 20 à 120 µs après le SPI, entrée ISR 5 à 20 µs, datation RxDone 60 à 260 µs, blocages de `loop()` jusqu'à 250 µs, minuteries du pair en retard jusqu'à 200 µs ;
- un RX ouvert **avant** le début de chaque paquet reçu et encore ouvert à sa fin (aucune fermeture anticipée) ;
- aucun SPI de `loop()` pendant que la radio peut lever DIO1 sans interruptions masquées ;
- les trames FLRC d'au moins 6 o (bourrage `0xFF` ignoré par le lecteur), avec une phase sans trafic en FLRC ;
- les deux voies saturées de petits messages numérotés, reçus dans l'ordre ;
- la puissance avec une antenne de 3 dBi (50 mW conduits).

Lancer : `python test/test_rlrs_bridge/run_host_tests.py` (banc du pont) et `python test/test_rlrs_air/run_host_tests.py` (RlrsAir/RlrsHop, dont `testMasterRxMargin` : balayage de la période de trame, voie descendante limitée par le temps, 450 µs de retard cumulé ; il échoue avec l'ancienne fermeture à `endGuard`).
