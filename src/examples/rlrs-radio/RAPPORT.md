# Rapport — test radio RocketLRS de 69 octets

## Résultat

Un firmware de test autonome a été ajouté pour le **RadioMaster Ranger Nano ESP32 (TX)** et le **RadioMaster RP1 ESP8285 (RX)**. Il transmet chaque message RocketLRS en une seule trame LoRa de 69 octets, puis vérifie son retour par écho. Une interface Wi-Fi affiche les compteurs, permet de saisir des messages et conserve la possibilité de remplacer le firmware.

Les boutons Build habituels des cibles `Unified_ESP32_2400_TX_via_WIFI` et `Unified_ESP8285_2400_RX_via_WIFI` compilent maintenant ce test. `platformio.ini` charge `targets/rlrs-radio-test.ini` en dernier pour sélectionner son point d'entrée. La configuration séparée `platformio.radio-test.ini` reste utilisable. Le firmware ELRS principal et ses structures OTA expérimentales ne sont pas utilisés par ces deux builds de test.

## Protocole utilisé

La référence est `C:/Users/ElitGaimix/Documents/microfusee/protocole.md`.

| Champ | Taille |
|---|---:|
| Synchronisation AA | 1 octet |
| Longueur du payload | 1 octet |
| ID | 1 octet |
| Payload | 0 à 64 octets |
| CRC CCITT-FALSE, octet fort puis faible | 2 octets |

Le CRC utilise le polynôme 0x1021 et la valeur initiale 0xFFFF, sans réflexion ni XOR final. Il couvre l'en-tête et les données, sans le CRC.

La référence autorise jusqu'à 255 octets de payload ; ce prototype impose la limite demandée de **64**. Un message court est complété avec des zéros **après son CRC** pour atteindre la longueur radio fixe. Ces zéros ne sont ni inclus dans le CRC, ni transmis au parseur.

L'ID 00 des émissions automatiques sert ici de message synthétique de test. Son payload est un motif binaire variable, et non le champ de mode d'un heartbeat de production. Le RX se limite à vérifier puis renvoyer les octets ; aucune commande métier n'est exécutée.

## Changements

| Fichier | Rôle |
|---|---|
| `platformio.ini` et `targets/rlrs-radio-test.ini` | Raccordement du test aux deux boutons Build Wi-Fi habituels |
| `src/rlrs_radio_test.cpp` | Point d'entrée du test et déclaration de ses dépendances pour PlatformIO |
| `platformio.radio-test.ini` | Configuration séparée optionnelle, avec builds et dépendances isolés |
| `examples/rlrs-radio/main.cpp` | Initialisation SX1280, émission, réception, écho et page Wi-Fi |
| `lib/RocketLRS/RlrsProtocol.h` | CRC, sérialisation explicite et validation des messages |
| `lib/RocketLRS/RlrsParser.cpp` | Reconnaissance de AA, CRC octet fort en premier, rejet des CRC invalides |
| `lib/SX12xxDriverCommon/SX12xxDriverCommon.h` | Taille du buffer RX configurable ; valeur normale de 16 octets conservée |
| `lib/SX1280Driver/SX1280_hal.cpp` | Transferts FIFO compatibles avec la limite SPI de 64 octets |
| `examples/rlrs-radio/test_protocol.cpp` | Tests natifs du protocole et du parseur |
| `examples/rlrs-radio/run_host_tests.py` | Exécution des tests, y compris les méthodes SPI réelles avec un bus simulé |
| `examples/rlrs-radio/README.md` | Instructions de compilation, installation, utilisation et retour à ELRS |

Les sources du parseur existaient déjà localement sans être suivies par Git. Elles ont été adaptées au document fourni ; leur API et le comportement de `reset()` qui efface le callback ont été conservés.

## Pourquoi le SPI a été adapté

Le pilote SPIEx utilise directement une FIFO matérielle de 64 octets. Une écriture radio de 69 octets demande en plus 2 octets de commande ; une lecture en demande 3. Les anciennes méthodes dépassaient donc la capacité de cette FIFO.

Les écritures sont maintenant limitées à 62 octets de données par commande SPI, les lectures à 61. Les adresses de la mémoire radio avancent à chaque commande. Les écritures intermédiaires sont synchrones, afin que le contrôle BUSY de la commande suivante arrive après la fin du transfert précédent. La dernière écriture conserve le comportement asynchrone existant.

Ces opérations chargent ou lisent la mémoire de la puce. **Un seul appel d'émission déclenche ensuite une seule trame radio** : ce découpage SPI n'est pas une fragmentation du message sur les ondes.

## Vérifications exécutées

- CRC de référence ASCII `123456789` : 0x29B1.
- Les deux exemples complets du document, dont `AA 04 00 54 45 53 54 C9 24`.
- Toutes les tailles de payload de 0 à 64 octets, avec toutes les positions de découpage d'entrée : **2 472 callbacks de réception vérifiés** au total.
- Rejet des CRC erronés et des CRC avec octets inversés.
- Bruit avant synchronisation, longueur excessive, resynchronisation sur AA et messages consécutifs.
- Refus des capacités de sortie insuffisantes et des messages incomplets.
- **258 cas SPI simulés** : longueurs de 0 à 128 octets, adresses de départ 0 et 128, vérification des données, des octets sentinelles, des limites de FIFO et des écritures intermédiaires synchrones.
- Compilation des deux cibles finales : voir les résultats ci-dessous.

Les tests SPI compilent les méthodes extraites du véritable fichier du pilote. Le bus et la mémoire de la puce sont simulés ; ils ne prouvent pas le fonctionnement électrique ou les timings d'interruption sur le matériel.

## Résultats de compilation

| Cible | Résultat | RAM statique | Flash programme | Fichier à installer |
|---|---|---:|---:|---|
| Ranger Nano ESP32 TX | SUCCESS | 49 408 / 327 680 octets | 901 413 / 1 966 080 octets | `.pio/build/Unified_ESP32_2400_TX_via_WIFI/firmware.bin` |
| RP1 ESP8285 RX | SUCCESS | 31 916 / 81 920 octets | 372 327 / 991 216 octets | `.pio/build/Unified_ESP8285_2400_RX_via_WIFI/firmware.bin.gz` |

Taille des fichiers livrés, incluant les métadonnées et la compression éventuelle : **910 724 octets pour le TX**, **265 787 octets pour le RX**.

Empreintes SHA-256 des fichiers de cette compilation :

```text
TX  a42e448e8485e88a6d38e6ab9aa0a4c3bd67157bf2a56eb0ed6d18ab58025c87
RX  ba5e9b691b9780a61b057c3bd11acf6a524a3f37b9861e3345e36188f71a61a3
```

Ces tailles et empreintes peuvent changer lors d'une nouvelle compilation (options générées et métadonnées de compression). La RAM dynamique du serveur Wi-Fi n'est pas comprise dans la mesure de RAM statique ci-dessus.

Les binaires ont également été inspectés pour vérifier l'identifiant de cible reconnu par la mise à jour ELRS et la configuration matérielle ajoutée à l'image. Le firmware RX compressé a été décompressé en mémoire pour cette vérification.

## Utilisation

Le guide complet est dans [README.md](README.md).

Après installation via le Wi-Fi ELRS existant, les points d'accès s'appellent **RocketLRS Test TX** et **RocketLRS Test RX**, mot de passe `expresslrs`, adresse **http://10.0.0.1/**. Le TX envoie automatiquement 64 octets de payload toutes les secondes. Le compteur `echo_ok` doit augmenter lorsque les deux modules échangent correctement.

Les tailles 0, 1, 63 et 64, ainsi que le message TEST fourni dans le protocole, constituent les essais manuels à effectuer sur les modules. Un retour au firmware habituel se fait depuis le formulaire de mise à jour du test avec le fichier correspondant au module.

## Limites et état matériel

**Aucun module n'a été flashé et aucun aller-retour radio réel n'a été mesuré pendant cette intervention.** La page web embarquée et la mise à jour Wi-Fi doivent également être validées sur les modules.

Ce prototype utilise une fréquence fixe de 2440 MHz, sans association ELRS, saut de fréquence ni retransmissions. Il conserve le Wi-Fi actif et ne constitue pas un test de portée. La puissance dépend du premier niveau calibré du profil matériel, avec application différée par le pilote après la première émission.

La mise à jour conserve le mécanisme Update de la plateforme, mais pas le contrôle du modèle matériel de l'interface ELRS complète. Il faut sélectionner le binaire du bon module. En cas d'échec de mise à jour, la radio reste arrêtée ; la page permet de réessayer, ou un redémarrage relance le test.

Le parseur UART n'a pas encore de délai d'abandon pour les messages tronqués. Le test radio évite de mélanger les messages en réinitialisant le parseur à chaque trame.

Les deux cibles Wi-Fi habituelles compilent désormais le test autonome. Les autres cibles ELRS conservent leur comportement précédent et peuvent encore rencontrer les erreurs du transport de production inachevé. Pour rétablir les anciennes cibles Wi-Fi, commenter la ligne `targets/rlrs-radio-test.ini` dans `platformio.ini`.

## Référence radio

La [fiche Semtech SX1280/SX1281, révision 3.2](https://www.mouser.com/datasheet/2/761/Semtech_04152020_DS_SX1280-1_V3.2-1832709.pdf), tableau 14-52, définit une longueur LoRa de 1 à 255 octets. La trame fixe de 69 octets se situe dans cette plage. La validation sur matériel reste nécessaire pour vérifier l'intégration du pilote et des modules.
