# Pont radio RocketLRS

Ce firmware remplace le site de d?monstration par un **pont s?rie ? radio ? s?rie**. Il utilise les paquets RocketLRS d?finis dans `microfusee/protocole.md` et dans le parseur Java du contr?leur NLFC.

## Trajet du heartbeat

1. Le programme Java ouvre le port du TX ? **9600 bauds**. Il construit d?j? les paquets RocketLRS : `AA, longueur, ID, payload, CRC haut, CRC bas`.
2. Le TX v?rifie ce paquet s?rie avec son parseur, puis place le paquet complet dans une trame radio SX1280 fixe de **69 octets**.
3. Le RX v?rifie la trame et le CRC, retire le bourrage radio ajout? apr?s le CRC, puis ?crit les octets du paquet RocketLRS sur l?UART ? **9600 bauds** vers le STM32.
4. La r?ponse du STM32 suit le trajet inverse et arrive au port s?rie Java. Le listener Java actuel la parse d?j? dans le m?me format.

Le heartbeat Java actuel a l?ID `0x00` et le payload `02`. Il est donc transport? tel quel ; le pont ne le remplace pas par un heartbeat de test, n?interpr?te pas sa s?mantique et ne fabrique pas de r?ponse. Le STM32 doit produire la r?ponse RocketLRS pour que le programme Java observe un heartbeat re?u.

Le paquet radio contient le paquet RocketLRS original en t?te, puis des octets nuls jusqu?? 69 octets. Le CRC ne couvre pas le bourrage. Les deux radios doivent utiliser les m?mes param?tres SX1280 configur?s dans ce firmware.

## Compilation avec les boutons PlatformIO habituels

Les deux cibles Wi-Fi s?lectionnent temporairement ce pont :

- TX Java : `Unified_ESP32_2400_TX_via_WIFI`
- RX STM32 : `Unified_ESP8285_2400_RX_via_WIFI`

Dans PlatformIO, choisir la cible puis cliquer sur **Build**. Les binaires sont g?n?r?s ici :

- TX : `.pio/build/Unified_ESP32_2400_TX_via_WIFI/firmware.bin`
- RX : `.pio/build/Unified_ESP8285_2400_RX_via_WIFI/firmware.bin.gz`

La configuration s?par?e `platformio.radio-test.ini` reste disponible pour recompiler l?ancien banc ? page web. Pour revenir aux cibles ELRS habituelles, commenter `targets/rlrs-radio-test.ini` dans `platformio.ini`.

## Connexions s?rie

Le TX Java et le RX STM32 utilisent l?UART principal du module, ? **9600 bauds, 8N1**. Sur le Ranger Nano et le RP1, le firmware du test utilise UART0 aux broches du profil mat?riel. Relier TX et RX crois?s, avec masse commune et niveaux logiques compatibles. ?viter d?envoyer des logs sur cet UART : il transporte des trames binaires.

Sur le TX, s?lectionner dans le programme Java le port COM correspondant au Ranger Nano. Sur le RX, raccorder l?UART du RP1 au port s?rie du STM32. Le STM32 doit r?pondre avec un paquet valide incluant son CRC.

## Limites du pont actuel

Les messages sont limit?s ? 64 octets de payload pour tenir dans une trame radio de 69 octets. Ils sont trait?s un par un, sans accus? radio, retransmission ni saut de fr?quence. Le transfert se fait entre deux appareils utilisant la m?me fr?quence, modulation et pr?ambule. Les r?ponses m?tier restent ? la charge du STM32.
