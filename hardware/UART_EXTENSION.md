# UART A/B dédié : contrat et révision

## Extension UART du 10 novembre 2026

Révisions **A0.4.2 / B0.5.2** : J10 est un JST-XH3 supplémentaire assemblé
sur chaque PCB, pas2,50mm, même MPN B3B-XH-A(LF)(SN) que J6.

| Broche J10 | Carte A | Carte B température |
| --- | --- | --- |
| 1 | GND | GND |
| 2 | TX_A, GPIO17, pad U1.22 | Non connectée ; réserve RX de future B |
| 3 | RX_A, GPIO16, pad U1.21 | Non connectée ; réserve TX de future B |

Câblage **droit 1→1, 2→2, 3→3** ; TX/RX sont nommés du point de vue de A.
UART logique **3,3V**, sans alimentation dans ce connecteur : jamais RS232
ni5V directement. Les transceivers RS232/RS485 éventuels appartiennent aux
futures B. Le port série USB de la DevKit reste indépendant ; les deux PWM
et ENABLE/FAULT_N/DIN0 restent disponibles. GPIO16/17 doivent être libres
sur la variante de DevKit employée (modules à PSRAM utilisant ces broches
non couverts par cette interface). Aucun UART n’est activé par le firmware
de diagnostic actuel ; aucun essai de boucle matérielle n’est déclaré fait.

B température n’a pas de périphérique UART : les deux pastilles sont
intentionnellement NC et le connecteur est néanmoins à souder. Le câble UART
est **facultatif pour ce prototype** et n’est pas ajouté au devis LCSC en cours.
Si nécessaire plus tard, reprendre la construction H02 XHP-3/XHP-3,300mm AWG26
avec un repérage UART distinct. Ne pas confondre J6 I²C et J10 UART, mécaniquement
identiques. Les alias fabricant J8/J9 des deux sockets de A restent inchangés.

Référence de restriction PSRAM : [fiche Espressif ESP32-WROVER](https://www.espressif.com/sites/default/files/documentation/esp32-wrover_datasheet_en.pdf). Les broches16/17 y sont réservées.
