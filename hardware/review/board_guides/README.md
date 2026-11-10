# Guide visuel des cartes A et B

Vues explicatives des révisions natives **A 0.4.2 / B 0.5.2**, créées le
10 novembre 2026. Les positions, contours d’empreintes et pastilles proviennent
du KiCad actuel, vérifié par SHA-256. Les pistes ne sont pas représentées :
ces vues servent à identifier les fonctions, sans remplacer les plans de
fabrication ni l’aperçu de placement du fabricant.

- [Guide PDF, trois pages A3 paysage](output/pdf/cartes_A_B_guide_annote.pdf) :
  A, B, puis circulation de la puissance, trois liaisons nécessaires et UART facultatif.
- [Carte A annotée](carte_A_annotee.png), [source vectorielle SVG](carte_A_annotee.svg).
- [Carte B annotée](carte_B_annotee.png), [source vectorielle SVG](carte_B_annotee.svg).
- [Liaisons A/B](liaisons_A_B.png), [source vectorielle SVG](liaisons_A_B.svg).

Les flèches pointent vers le composant ou le connecteur réel. Une seule
flèche peut représenter un ensemble fonctionnel dont les autres repères sont
indiqués dans la légende. Les contours des composants sont des représentations
d’implantation, pas des photographies. La DevKit sur A est représentée comme
un module enfichable ; elle n’est pas fournie/soudée par l’assembleur.

## Carte A : interface commune

| Repères | Rôle |
| --- | --- |
| U1 | DevKit ESP32 enfichable, firmware, interface utilisateur et communications. USB-C sur la DevKit, pas sur le PCB porteur. Deux sockets au pas 2,54 mm, entraxe 25,4 mm. |
| J1 | Entrée 5 V régulée venant de B, indépendamment de la tension d’entrée de B. 1 GND, 2 +5V_AB. |
| J10 | UART logique3,3V dédié :1 GND,2 TX_A/GPIO17,3 RX_A/GPIO16. Signaux2/3 non reliés sur B température ; câble facultatif. |
| JP1 | Cavalier entre +5V_AB et VIN de la DevKit. Livré séparément ; point d’isolement pour le mode de banc documenté. Ce n’est pas une protection automatique contre le retour USB. |
| U2, C1, C2, C3 | Régulateur AP2112 5 V vers 3,3 V pour les périphériques, avec découplage/filtrage. La broche 3V3 de la DevKit n’alimente pas ce rail. |
| J2 | LCD 20×4 avec backpack I²C externe, alimenté en 3,3 V dans ce prototype. |
| J3, R1, R2, R3 | Encodeur A/B et bouton, avec résistances de rappel sur les entrées. J3 : 1 A, 2 B, 3 bouton, 4 GND. |
| J4 | Module TMP117 externe optionnel, adresse 0x48. |
| J5 | Module de luminosité BH1750 externe, adresse 0x23. |
| J6, R4, R5 | Bus I²C commun à A/B et résistances de rappel vers 3,3 V. J6 : 1 GND, 2 SDA, 3 SCL. |
| J7 | Commandes de B : 1 GND, 2 PWM0, 3 ENABLE, 4 FAULT_N, 5 PWM1, 6 DIN0. Les deux dernières positions ne sont pas connectées sur B température. |
| D1/R6, D2/R7 | LED et résistances : présence du 5 V puis du 3,3 V. |
| TP1 à TP10 | Pastilles de mesure au dos, sans composant ajouté. |

J2/J4/J5 partagent le brochage **1 GND, 2 +3V3_A, 3 SDA, 4 SCL**.
Le brochage de J3 est différent. Les J8/J9 cités dans les fichiers JLCPCB sont
des alias d’import pour les deux sockets de U1, pas deux nouveaux connecteurs
repérés ainsi sur la sérigraphie native.

## Carte B : température et puissance

| Repères | Rôle |
| --- | --- |
| J2 | Entrée DC : 1 VIN_RAW positif, 2 PGND. La révision est étudiée pour la limite 24 V / 100 W ; le choix nominal 12/24 V et le chauffage associé restent à qualifier. |
| F1, Q2, R8, D3, D2 | Fusible 6,3 A, protection d’inversion de polarité et réseau de grille ; D2 est la TVS d’entrée dont le comportement en transitoires reste à qualifier. |
| U1, D9, L1 | Buck LM2596 5 V, diode Schottky et inductance de 100 µH. Remplace le module XL4015 externe. |
| C8, C9, C10, R13 | Filtrage entrée/sortie du buck. R13 est la résistance série de retour de C10 utilisée par le réseau simulé, **pas un shunt de mesure du chauffage**. |
| F2, J1 | Fusible 1 A du 5 V partagé et départ vers A. J1 : 1 GND, 2 +5V_AB. |
| U2, C1, C2, C3, C5 | Régulateur 3,3 V et découplages de l’acquisition/logique. |
| J4, R5, R6, C4 | Sonde NTC passive 15 kΩ / B≈3920, polarisation, filtre analogique. 1 +3V3_B, 2 NTC_RAW. |
| U4, R7, J6 | ADS1115 : convertit NTC_ADC puis transmet la mesure en I²C à l’adresse 0x49. R7 rappelle FAULT_N au 3,3 V ; ce retour exige une configuration logicielle et ne remplace pas une sécurité indépendante. |
| J7, R1, R2, U3 | Entrée PWM0 et ENABLE, rappels à zéro, porte logique ET SN74LVC1G08. |
| U5, C6, C7, R3, R4 | Driver UCC27517 alimenté en 5 V, découplage, résistance de grille et rappel de grille à zéro. |
| Q1, J3, D4 | MOSFET IRLZ44N commandant le retour du chauffage, bornier de charge et diode de roue libre. J3 : 1 VIN_PROTECTED positif, 2 HEATER_LOW commuté. |
| D5/R9, D6/R10, D7/R11, D8/R12 | Voyants entrée protégée, 5 V, 3,3 V et activité de commande de grille. D8 ne mesure ni le courant, ni la puissance réellement reçue par la charge. |
| J10 | Même brochage droit que A :1 GND,2 TX_A,3 RX_A ; contacts2/3 NC sur ce prototype. Connecteur à souder, aucun périphérique UART sur B température. |
| NT1 | Liaison de cuivre entre GND et PGND, sans isolation galvanique et sans composant à monter. |
| TP1 à TP12 | Pastilles de mesure au dos. Aucun point de test sur BUCK_SW. |

Le courant de chauffage circule **J2 → protections → J3 / chauffage → Q1 → PGND**.
Il ne traverse pas les faisceaux A/B. La protection thermique indépendante
reste extérieure au PCB. Les deux cartes et leur firmware doivent encore
passer les essais matériels de mise au point.

## Compléments et régénération

- [Points de test, légendes et valeurs attendues](../../TEST_POINTS.md).
- [A au dos, vue miroir correcte](../test_points_A_back.png) et
  [B au dos](../test_points_B_back.png).
- [Câblage et numéros des pastilles](../../manufacturing/wiring_and_harnesses.md).
- [KiCad A](../../pcb_a_display/pcb_a_display.kicad_pcb) et
  [KiCad B](../../pcb_b_temperature/pcb_b_temperature.kicad_pcb).

Sources modifiables : [générateur Python](../../scripts/build_board_guides.py).
Il lit `manufacturing/output/assembly_geometry.json`, refuse une géométrie
qui ne correspond plus aux empreintes natives, puis produit SVG, PDF et PNG.
Les SVG sont modifiables dans un éditeur vectoriel ; les modifications manuelles
seront remplacées lors de la régénération.

```powershell
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/scripts/build_board_guides.py
```

Dépendances : ReportLab, fontes Arial Windows, Poppler `pdftoppm` sur PATH.
`source_manifest.json` conserve les empreintes des sources. Les fichiers
électriques et les archives destinées aux fabricants ne sont pas modifiés.
