# Carte A commune aux trois DevKit ESP32 USB-C

Décision utilisateur du 5 novembre 2026 : **A doit recevoir les trois cartes
ESP32 disponibles**, qui ont le même brochage et le même connecteur USB-C.
L'entraxe retenu est 25,4 mm, confirmé par mesure. La carte A reste un support
de DevKit enfichable, indépendant de son fabricant et du pont USB/série.

## Interface commune déjà prévue

- Deux barrettes femelles **1×15, pas 2,54 mm**, avec **25,4 mm entre axes**.
- Brochage de référence : les 30 contacts de `module_selection.json` ; VIN/5V,
  GND et GPIO restent aux mêmes contacts physiques. L'utilisateur confirme
  le brochage commun des trois cartes ; ce n'est pas un relevé automatique.
- A fournit **5 V régulé** à la DevKit à travers JP1 ; son régulateur local
  alimente séparément les périphériques en 3,3 V. La sortie 3V3 de la DevKit
  reste non connectée sur A pour éviter de mettre les régulateurs en parallèle.
- Le pont CP210x ou CH340 appartient à la DevKit. Il ne nécessite aucune piste
  différente sur A lorsque le brochage des contacts est identique.
- Le port USB-C latéral du boîtier rejoint la DevKit par une rallonge de
  données flexible ; le connecteur n'est pas directement soudé sur A.

**La position des contacts et le routage de A restent communs.** La préparation
finale emploie deux sockets Harwin dans une empreinte documentée, avec
perçages1,05mm. L’ancienne empreinte `Diymore`/`ASSUMED` reste historique.
Le schéma et le STEP conservent pour l'instant la DevKit de référence comme
représentation, sans prétendre montrer les trois variantes physiques.

## Compatibilité à distinguer pour chaque carte

| Point | État |
| --- | --- |
| Brochage commun / USB-C | Confirmé par l'utilisateur pour les trois cartes. |
| Entraxe 25,4 mm | Confirmé par mesure et conforme à l'empreinte routée. |
| Communication USB et lecture de flash | À consigner séparément par puce ; le numéro COM peut être réutilisé par une autre carte. |
| Contour, hauteur montée, blindage et antenne | À vérifier sur chaque variante dans le volume disponible. La cote vendeur 54 × 27 mm ne devient pas une mesure des trois cartes. |
| Position du connecteur USB-C et prise du câble | À vérifier avec la rallonge réelle, y compris lors de l'extraction de A et de sa façade. |
| Chemin VIN/VBUS et alimentation du pont série | À qualifier par variante ; même brochage externe ne démontre pas un circuit d'alimentation interne identique. |

La cible est une **même carte A, les mêmes sockets et les mêmes faisceaux**.
Si une variante dépasse le volume de référence, ajuster la réserve mécanique
et la position du câble à partir de ses cotes, avant d'imprimer le boîtier
définitif. Ne pas figer la hauteur USB par rapport à une seule marque.

Le fonctionnement avec USB et source externe simultanés n'est pas qualifié.
La procédure commune reste celle de
`manufacturing/usb_service_test.md`, avec résultats propres à chaque variante.
Une difficulté de connexion au bootloader ne démontre pas une incompatibilité
avec les sockets de A ; elle doit être diagnostiquée sur la carte concernée.

La préparation finale utilise `LabHardware:ESP32_30P_HCTL_PM254_Rows25.40mm`, avec deux HarwinPM254-1-15-Z-8.5, trous1,05mm et entraxe confirmé. L’ancienne empreinte `_ASSUMED` ne sert qu’aux révisions archivées. Le STEP v0.8 inclut le plastique des broches mâles sous forme d’une enveloppe2,54mm, à vérifier sur les modules.

## Révision UART et retour d’usage du 10 novembre 2026

A0.4.2 utilise GPIO17 TX et GPIO16 RX sur J10. Ces deux broches doivent être
libres sur la DevKit : les modules WROVER qui les réservent à la PSRAM ne
sont pas couverts par cette affectation. Le brochage USB UART0 n’est pas modifié.

L’utilisateur rapporte des utilisations simultanées USB et VIN sans problème
observé. Cela documente un fonctionnement réel, sans identifier encore le
chemin VIN/VBUS de chacune des trois cartes. JP1 reste un isolement de banc
pour cette révision ; aucune incompatibilité USB/VIN générale n’est affirmée.
