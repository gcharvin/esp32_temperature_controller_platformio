# PCB A et B température - prototype routé 0.3

Révision du **5 novembre 2026**. Les deux cartes sont routées dans les fichiers natifs KiCad, avec empreintes physiques et plans de masse. Cible de B : **24 V continu / chauffage résistif 100 W**, soit 4,17 A pour la charge. A porte un seul ESP32 ; B reste sans microcontrôleur.

**Prototype à contrôler, pas dossier de fabrication libéré.** À la demande de l'utilisateur, le routage avance avec **25,40 mm entre rangées de la DevKit comme hypothèse**, à vérifier sur la carte achetée. Le dimensionnement thermique et les essais de défaut restent à réaliser. Aucun Gerber/CPL final n'est généré.

## Fichiers à ouvrir

| Objet | Fichier |
| --- | --- |
| Projet complet A | `pcb_a_display/pcb_a_display.kicad_pro` |
| Pistes de A | `pcb_a_display/pcb_a_display.kicad_pcb` |
| Projet complet B | `pcb_b_temperature/pcb_b_temperature.kicad_pro` |
| Pistes de B | `pcb_b_temperature/pcb_b_temperature.kicad_pcb` |
| Vues cuivre dessus/dessous | `review/routing/pcb_a_display_routed.pdf`, `review/routing/pcb_b_temperature_routed.pdf` |
| Schémas à lire | `review/pcb_a_display.pdf`, `review/pcb_b_temperature.pdf` |
| Rapport de routage | `review/routing/routing_validation.json` |
| Pack KiCad de revue | `review/routing/prototype_v03_kicad_bundle.zip` |

Dans l'éditeur PCB, afficher F.Cu/B.Cu et masquer temporairement les zones remplies si elles cachent les pistes dans la vue choisie. Les fichiers natifs `.kicad_sch` et `.kicad_pcb` sont la référence ; les scripts ne doivent pas écraser des modifications faites ensuite dans KiCad.

## Modules achetés et empreintes

- **DevKit diymore, USB-C, CH340, 30 contacts**, [ASIN B0DGLCWR76](https://www.amazon.fr/dp/B0DGLCWR76). Elle remplace l'hypothèse ELEGOO/CP2102. Empreinte locale `LabHardware:Diymore_ESP32_30P_Rows25.40mm_ASSUMED`, pour deux barrettes femelles 1×15 au pas 2,54 mm. Brochage relevé sur les vues du vendeur. Encombrement annoncé : 54 × 27 mm. L'entraxe de 25,40 mm est une **inférence**, pas une cote fabricant certifiée : le vérifier avant fabrication. Voir `module_selection.json` pour les 30 contacts et la correspondance aux GPIO.
- **Buck TECNOULAB XL4015 CC/CV**, [ASIN B0C6R3FMT3](https://www.amazon.fr/dp/B0C6R3FMT3), modèle annoncé RED5AXL4015. Le vendeur donne 51 × 26,3 × 14 mm et une entrée de 5 à 32 V. Module **hors PCB B**, raccordé par fils à ses borniers ; aucune empreinte enfichable ou position de trous inventée. U1 reste présent au schéma et à la BOM, avec `on_board no`.
- Les ports périphériques A et la sonde B ont des JST-XH physiques. U3 est une SN74LVC1G08DBVR en SOT-23-5 ; U5 une UCC27517DBVR en SOT-23-5. Q1 est une IRLZ44NPBF TO-220 ; Q2 une DMP6023LE-13 SOT-223. Les anciennes enveloppes sans pastilles sont conservées dans leur bibliothèque historique, mais **aucune n'est utilisée dans les PCB routés**.

Les caractéristiques publicitaires du module XL4015 ne remplacent pas une qualification du module complet. La [fiche du circuit XL4015](https://www.xlsemi.com/datasheet/XL4015-EN.pdf) décrit la puce, pas toute la carte TECNOULAB. Régler et mesurer **5,00 V** avec le module déconnecté d'A/B avant raccordement, puis contrôler la tension sous charge.

## Alimentation et chauffage sur B

J2.1 reçoit le 24 V et J2.2 le retour **PGND**. F1 est un candidat fusible rapide **6,3 A / 125 VDC, Littelfuse 045106.3MRL**. Il protège contre les surintensités selon son I²t ; il ne limite pas automatiquement la puissance à 100 W. Son déclenchement, l'appel de courant et le courant de court-circuit de l'alimentation doivent être validés.

Q2 assure l'anti-inversion : drain sur `VIN_FUSED`, source sur `VIN_PROTECTED`, grille tirée vers PGND par R8 ; D3 BZT52C12 limite Vgs. D2 SMBJ24CA est placée entre alimentation protégée et PGND. **L'ensemble n'est pas qualifié en surtension/transitoires** : son écrêtage peut dépasser la plage continue annoncée du module acheté. Employer une alimentation 24 V régulée ; revoir la protection avant de revendiquer une tenue à des surtensions industrielles.

Le chauffage se branche entre J3.1 (`VIN_PROTECTED`) et J3.2 (`HEATER_LOW`). Q1 commute son retour vers PGND. La porte AND U3 combine PWM0 et ENABLE à 3,3 V ; U5 commande ensuite Q1 avec le rail 5 V. R3 = 22 ohms et R4 = 100 kohms. La cible de mise au point reste un PWM de **500 Hz**, pas une fréquence arbitrairement élevée. D4 **STPS5L60U** reprend l'inductance des fils à la coupure ; C8 **EEUFR1H101, 100 µF / 50 V** assure le découplage local de puissance.

J2, J3 et J8 : borniers Phoenix Contact **1715721, MKDS 1,5/2-5,08**, pas 5,08 mm. Aucun courant de chauffage ne traverse la DevKit ou les JST A/B. Le contrôle suit le circuit de puissance par intersections des formes de cuivre réelles : continuité sans via discret sur le trajet principal, pistes de 2,4 mm avec courts rétrécissements aux composants. **B demande 70 µm de cuivre externe sur les deux faces (2 oz), indiqué dans son empilage KiCad.** A emploie 35 µm. Cela ne constitue pas une qualification thermique.

## Raccordement du XL4015

| PCB B | Raccordement module |
| --- | --- |
| J8.1 | IN+ du XL4015, alimentation protégée 24 V |
| J8.2 | IN- du XL4015, PGND |
| J9.1 | OUT- du XL4015, GND logique |
| J9.2 | OUT+ du XL4015, +5V_BUCK réglé à 5,00 V |

J8 est un bornier 5,08 mm ; J9 un JST-XH 2 positions. La sortie du module traverse F2, **0451001.MRL, 1 A / 125 VDC**, puis alimente `+5V_AB`.

**PGND et GND ne sont pas court-circuitées par du cuivre sur B.** Le module CC/CV peut avoir un shunt entre IN- et OUT- ; conserver les deux retours évite de le contourner sur le PCB. Le module reste non isolé. La logique se réfère à OUT- ; le MOSFET et son driver se réfèrent à PGND. Vérifier le décalage de masse réel et le fonctionnement de la limitation sous charge, notamment avec le PC branché.

## Trois faisceaux A/B

Petits JST-XH verticaux au pas **2,50 mm**, faisceaux droits. Aucun 24 V ni rail 3,3 V dans ces faisceaux. Brancher/débrancher hors tension ; aucun hot-plug n'est prévu.

| Connecteur | Brochage |
| --- | --- |
| J1, POWER, 2 positions | 1 GND ; 2 +5V_AB, de B vers A |
| J6, I2C, 3 positions | 1 GND ; 2 SDA ; 3 SCL |
| J7, CTRL, 6 positions | 1 GND ; 2 PWM0 ; 3 ENABLE ; 4 FAULT_N ; 5 PWM1 ; 6 DIN0 |

J7.5 et J7.6 restent non connectées sur B température. Les empreintes sont B2B-XH-A, B3B-XH-A et B6B-XH-A. Les références de câble sont XHP-2/-3/-6 avec contacts adaptés. `interface_ab.json` conserve le contrat électrique. Le pas JST-XH 2,50 mm diffère des barrettes/Dupont 2,54 mm.

## A, périphériques et firmware

J2 LCD, J4 TMP117 et J5 BH1750 : **1 GND, 2 +3V3_A, 3 SDA, 4 SCL**, JST-XH 4 positions. J3 encodeur : **1 A, 2 B, 3 bouton, 4 GND**, JST-XH 4 positions. Adapter les câbles au brochage réel des modules. Le LCD conserve le 3,3 V utilisé dans le prototype ; vérifier fonctionnement et rétroéclairage avec le module réel. Les deux AP2112K-3.3 alimentent les périphériques locaux, sans relier leurs sorties ni la sortie 3V3 de la DevKit entre elles.

| Fonction | GPIO ESP32 |
| --- | --- |
| SDA / SCL | 21 / 22 |
| Encodeur A / B / bouton | 4 / 19 / 23 |
| PWM0 / PWM1 | 25 / 26 |
| ENABLE / FAULT_N / DIN0 | 27 / 33 / 32 |

**Le firmware n'a pas été modifié par ce routage.** Le GPIO chauffage historique 5 doit devenir 25, avec gestion d'ENABLE, ADC, défauts et perte de mesure. L'ADS1115 a l'adresse **0x49** (ADDR=VDD), le TMP117 0x48 et le BH1750 0x23. Les pull-ups I2C principaux sont uniquement R4/R5 sur A, 4,7 kohms ; tenir compte de ceux des modules.

NTC : sonde 15 kohms, B proche de 3920 K, entre +3V3_B et NTC_RAW ; R5 = 8,45 kohms vers GND. Filtre 1 kohm / 100 nF vers AIN0. Adapter la constante historique 8400 ohms et calibrer la résistance et l'excitation. FAULT_N est ALERT/RDY de l'ADC, à configurer ; aucune coupure thermique indépendante n'est intégrée dans cette révision.

USB : la prise latérale et sa rallonge utilisent le pont CH340 de la DevKit. JP1 reste un cavalier de service, **pas une sélection automatique des alimentations**. Ne pas présumer que 5 V externe et VBUS peuvent être mis en parallèle sur cette variante. Pour le flash USB seul : B hors tension, J1/J6/J7 déconnectés et JP1 retiré. Le monitoring alimenté en 24 V nécessite de vérifier la gestion des sources sur la carte réelle.

## Mécanique et contrôles

A reste à **110 × 75 × 1,6 mm**, B à **90 × 65 × 1,6 mm**. Les bandes de clips haut/bas de 4 mm et les appuis latéraux de 1,5 mm sont exclus des pistes, vias, remplissages et composants. La réserve d'antenne est provisoire et doit être confrontée au module réel et à son environnement.

Les [STEP du boîtier v0.4](mechanical/README.md) sont une **étude antérieure**, non réconciliée avec la DevKit diymore et le buck de 51 mm désormais séparé. Le support du XL4015, le rangement des fils et la position précise de l'USB interne restent à reprendre avant impression finale. Les PCB ont leurs contours et réserves conservés ; le modèle mécanique complet n'est pas requalifié par ce routage.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_schematics.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_routing.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_placement.py
```

Résultat livré : **ERC zéro erreur/avertissement, DRC zéro violation, zéro connexion restante et zéro écart schéma/PCB sur les deux cartes**. Les pads sont comparés aux netlists et les trajets principaux de puissance sont contrôlés. `review/routing/` contient les rapports et les PDF relus. Le routage a été amorcé avec Freerouting 2.5.0 **localement**, puis corrigé dans les fichiers natifs ; aucune API de routage distante n'a reçu les cartes. Les `.dsn/.ses` et journaux sont des intermédiaires ignorés par Git. Les scripts initiaux restent des étapes historiques et refusent d'écraser un PCB déjà routé.

Références : [JST-XH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf), [driver UCC27517](https://www.ti.com/lit/gpn/UCC27517), [IRLZ44N](https://www.infineon.com/part/IRLZ44N), [DMP6023LE](https://www.diodes.com/datasheet/download/DMP6023LE.pdf), [fusibles 451/453](https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1), [STPS5L60](https://www.st.com/resource/en/datasheet/stps5l60.pdf), [bornier Phoenix](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721?type=pdf), [ADS1115](https://www.ti.com/lit/ds/symlink/ads1115.pdf).
