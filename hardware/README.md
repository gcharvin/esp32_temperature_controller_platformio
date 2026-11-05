# PCB A et B température — A v0.4 / B v0.5 buck intégré

Révision du **5 novembre 2026**. Les deux cartes sont routées dans les fichiers natifs KiCad, avec empreintes physiques et plans de masse. Cible de B : **24 V continu / chauffage résistif 100 W**, soit 4,17 A pour la charge. A porte un seul ESP32 ; B reste sans microcontrôleur.

**Prototype à contrôler, pas dossier de fabrication libéré.** L'entraxe des deux rangées de la DevKit est **25,40 mm, confirmé par mesure de l'utilisateur le 5 novembre 2026** ; il correspond à l'empreinte déjà routée. Les autres cotes de montage, le dimensionnement thermique et les essais de défaut restent à vérifier. Aucun Gerber/CPL final n'est généré.

Une [étude de buck intégré simulable dans KiCad](power_study/README.md) est disponible séparément : variante LM2596 fixe 5 V, six bancs transitoires réussis sous ngspice, sources et classeurs KiCad. Elle est intégrée au schéma et au routage de B v0.5 ; la maquette mécanique v0.7 reprend ce placement, sans module externe.

**Contrat de plateforme : A reçoit toujours un 5 V régulé**, indépendamment de
l'entrée 24 V, 12 V ou 5 V des futures cartes métier. Chaque B adapte sa propre
alimentation. B température v0.5 reste une variante 24 V ; les autres tensions
ne sont pas des modes qualifiés de cette carte. Voir la
[revue alimentation/USB](manufacturing/power_and_usb_review.md) et le contrat
`interface_ab.json`. Aucun 12/24 V sur A et aucune mise en parallèle de sources.

Le [dossier de préparation de fabrication](manufacturing/README.md) fournit
une BOM d'approvisionnement candidate, la répartition CMS/traversants, des
CPL et Gerber/perçages **provisoires pour revue/devis**. Les deux sockets de
la DevKit sont comptés séparément ; NT1 est exclu des achats. Le dossier
n'est pas libéré pour commande : références, cotes DevKit, modes USB et
aperçu d'assemblage restent à approuver.

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
| Pack KiCad de revue courant | `review/routing/prototype_v05_integrated_buck_kicad_bundle.zip` |

Dans l'éditeur PCB, afficher F.Cu/B.Cu et masquer temporairement les zones remplies si elles cachent les pistes dans la vue choisie. Les fichiers natifs `.kicad_sch` et `.kicad_pcb` sont la référence ; les scripts ne doivent pas écraser des modifications faites ensuite dans KiCad.

## Modules achetés et empreintes

**Cible commune : les trois DevKit ESP32 USB-C de l'utilisateur**, au même
brochage et à l'entraxe 25,4 mm. A conserve deux sockets 1×15 et n'est pas
réservée à une marque ou à un pont USB/série. Voir le
[contrat de compatibilité](devkit_compatibility.md) : les contacts sont communs,
mais l'encombrement et le comportement VIN/VBUS restent à vérifier par variante.
La diymore ci-dessous reste la représentation de référence du schéma et du STEP.

- **DevKit diymore, USB-C, CH340, 30 contacts**, [ASIN B0DGLCWR76](https://www.amazon.fr/dp/B0DGLCWR76). Elle remplace l'hypothèse ELEGOO/CP2102. Empreinte locale `LabHardware:Diymore_ESP32_30P_Rows25.40mm_ASSUMED`, pour deux barrettes femelles 1×15 au pas 2,54 mm. Brochage relevé sur les vues du vendeur. Encombrement annoncé : 54 × 27 mm. **L'entraxe de 25,40 mm est maintenant confirmé par mesure de la carte achetée.** Le suffixe `_ASSUMED` et les anciennes annotations natives restent des identifiants/notes historiques ; le statut courant est dans `module_selection.json` et le dossier de fabrication régénéré. Aucun changement de géométrie ni de pistes n'est nécessaire pour cette cote. Contour, brochage et hauteur montée/USB ne sont pas validés par cette seule mesure. Voir `module_selection.json` pour les 30 contacts et la correspondance aux GPIO.
- **B v0.5 : LM2596S-5.0/NOPB intégré**, self 100 µH SRP1770TA-101M, diode SS36, C8 d’entrée 100 µF / 50 V, C9 de découplage 100 nF / 50 V, C10 de sortie 330 µF / 35 V et résistance R13 de 56 mΩ. Le module XL4015 acheté n’est plus utilisé dans cette révision. J8/J9 sont supprimés ; NT1 matérialise la jonction des masses.

- Les ports périphériques A et la sonde B ont des JST-XH physiques. U3 est une SN74LVC1G08DBVR en SOT-23-5 ; U5 une UCC27517DBVR en SOT-23-5. Q1 est une IRLZ44NPBF TO-220 ; Q2 une DMP6023LE-13 SOT-223. Les anciennes enveloppes sans pastilles sont conservées dans leur bibliothèque historique, mais **aucune n'est utilisée dans les PCB routés**.

La sortie du buck intégré est fixe, nominalement 5 V ; aucun potentiomètre de réglage. Le budget système est 0,8 A et F2 reste à 1 A. Les références, empreintes, hypothèses et essais sont documentés dans [la révision B](pcb_b_temperature/README.md).

## Alimentation et chauffage sur B

J2.1 reçoit le 24 V et J2.2 le retour **PGND**. F1 est un candidat fusible rapide **6,3 A / 125 VDC, Littelfuse 045106.3MRL**. Il protège contre les surintensités selon son I²t ; il ne limite pas automatiquement la puissance à 100 W. Son déclenchement, l'appel de courant et le courant de court-circuit de l'alimentation doivent être validés.

Q2 assure l'anti-inversion : drain sur `VIN_FUSED`, source sur `VIN_PROTECTED`, grille tirée vers PGND par R8 ; D3 BZT52C12 limite Vgs. D2 SMBJ24CA est placée entre alimentation protégée et PGND. **L'ensemble n'est pas qualifié en surtension/transitoires** : son écrêtage doit être confronté aux limites du LM2596 et au transitoire réel. Employer une alimentation 24 V régulée ; revoir la protection avant de revendiquer une tenue à des surtensions industrielles.

Le chauffage se branche entre J3.1 (`VIN_PROTECTED`) et J3.2 (`HEATER_LOW`). Q1 commute son retour vers PGND. La porte AND U3 combine PWM0 et ENABLE à 3,3 V ; U5 commande ensuite Q1 avec le rail 5 V. R3 = 22 ohms et R4 = 100 kohms. La cible de mise au point reste un PWM de **500 Hz**, pas une fréquence arbitrairement élevée. D4 **STPS5L60U** reprend l'inductance des fils à la coupure ; C8 **EEUFR1H101, 100 µF / 50 V** assure le découplage local de puissance.

J2 et J3 : borniers Phoenix Contact **1715721, MKDS 1,5/2-5,08**, pas 5,08 mm. Aucun courant de chauffage ne traverse la DevKit ou les JST A/B. Le contrôle suit le circuit de puissance par intersections des formes de cuivre réelles : continuité sans via discret sur le trajet principal, pistes de 2,4 mm avec courts rétrécissements aux composants. **B demande 70 µm de cuivre externe sur les deux faces (2 oz), indiqué dans son empilage KiCad.** A emploie 35 µm. Cela ne constitue pas une qualification thermique.

## Buck intégré et masses — B v0.5

Le 24 V protégé alimente U1 et C8/C9. U1 commute le nœud BUCK_SW, repris par D9 et L1. L1 produit +5V_BUCK ; C10 filtre la sortie et son retour passe par R13 = 56 mΩ. F2 protège ensuite +5V_AB. La broche 5 de U1 est reliée à PGND : le buck fonctionne dès que le 24 V est présent.

**GND et PGND sont désormais reliées par NT1**, empreinte de cuivre près de l’entrée. Leurs noms distincts servent à organiser les retours. Le trajet principal du chauffage reste sur sa piste PGND de puissance, sans passer par NT1 ni les JST. Il n’y a plus de shunt CC/CV à préserver. NT1 n’est pas un composant à acheter ni une barrière d’isolation. Le partage USB/VBUS reste un point distinct à résoudre.

J8 et J9 sont supprimés. Aucun module externe ni faisceau de buck n’est à raccorder.

## Trois faisceaux A/B

Petits JST-XH verticaux au pas **2,50 mm**, faisceaux droits. Aucun 24 V ni rail 3,3 V dans ces faisceaux. Brancher/débrancher hors tension ; aucun hot-plug n'est prévu.

| Connecteur | Brochage |
| --- | --- |
| J1, POWER, 2 positions | 1 GND ; 2 +5V_AB, de B vers A |
| J6, I2C, 3 positions | 1 GND ; 2 SDA ; 3 SCL |
| J7, CTRL, 6 positions | 1 GND ; 2 PWM0 ; 3 ENABLE ; 4 FAULT_N ; 5 PWM1 ; 6 DIN0 |

J7.5 et J7.6 restent non connectées sur B température. Les empreintes sont B2B-XH-A, B3B-XH-A et B6B-XH-A. Les références de câble sont XHP-2/-3/-6 avec contacts adaptés. `interface_ab.json` conserve le contrat électrique. Le pas JST-XH 2,50 mm diffère des barrettes/Dupont 2,54 mm.

## Voyants de diagnostic et sonde

Six LED 0805 sont montées côté composants, avec résistances de limitation individuelles. Elles fonctionnent sans firmware ni GPIO supplémentaire. Les références sélectionnées sont [Kingbright APT2012LZGCK, verte](https://www.kingbrightusa.com/images/catalog/spec/apt2012lzgck.pdf) et [APT2012LSECK/J4-PRV, orange](https://www.kingbrightusa.com/images/catalog/spec/apt2012lseck-j4-prv.pdf), caractérisées à 2 mA. Leurs courants ci-dessous sont des estimations à tension nominale avec Vf typique, pas des mesures.

| Carte / repère sérigraphié | LED / résistance | Point observé / retour | Courant typique estimé |
| --- | --- | --- | --- |
| A : `5V_AB` | D1 verte / R6 1,5 kohm | +5V_AB / GND | 1,57 mA |
| A : `3V3_A` | D2 verte / R7 470 ohms | +3V3_A / GND | 1,38 mA |
| B : `24V` | D5 verte / R9 15 kohms, 0805 ≥0,125 W | VIN_PROTECTED / PGND, après F1 et Q2 | 1,42 mA |
| B : `5V` | D6 verte / R10 1,5 kohm | +5V_AB / GND, après F2 | 1,57 mA |
| B : `3V3` | D7 verte / R11 470 ohms | +3V3_B / GND | 1,38 mA |
| B : `HEAT` | D8 orange / R12 2,2 kohms | GATE_DRV / PGND, sortie du driver U5 | 1,45 mA pendant ON |

Les autres résistances de voyant sont des 0603, 1 %, ≥0,1 W. R9 dissipe environ 30 mW à 24 V ; même en supposant une LED court-circuitée et 32 V, la borne de calcul reste inférieure à 69 mW. Cela ne qualifie pas les transitoires du reste du circuit.

Lecture : `24V` éteinte malgré une source présente → vérifier entrée, F1 et Q2. `24V` allumée mais `5V` éteinte → vérifier U1, L1, D9, les condensateurs et F2. `5V` allumée mais `3V3` éteinte → vérifier le régulateur local et sa charge. Une LED allumée confirme une présence de tension, sans certifier sa valeur ni son ondulation. Les voyants de A surveillent son alimentation externe et ses périphériques ; ils n'attestent ni la présence de JP1, ni l'exécution du firmware de la DevKit.

`HEAT` suit le driver **après PWM ET ENABLE**, avec luminosité dépendant du rapport cyclique. Une charge absente peut laisser ce voyant allumé. Il ne mesure ni courant, ni watts, ni température ; la carte ne comporte toujours pas de mesure de courant du heater.

**La sonde se branche sur B/J4**, JST-XH 2 contacts, en bas à droite vu côté composants. La sérigraphie porte `TEMP / NTC 15k` et `1=3V3  2=NTC` : broche 1 = +3V3_B, broche 2 = NTC_RAW. Raccorder les deux contacts de la prise sonde du panneau arrière à J4. La NTC passive 15 kohms / B≈3920 n'est pas polarisée : les deux fils sont interchangeables. Ce connecteur n'est pas une entrée I2C ou pour sonde numérique.

Les voyants sont internes : B se lit panneau arrière retiré ; déposer B facilite l'accès visuel aux voyants de A. Aucun perçage ou guide de lumière de façade n'est ajouté. `diagnostic_leds.json` décrit les circuits et positions. Les dimensions des PCB, les supports, l'antenne et les connecteurs A/B sont conservés ; les nouveaux composants restent hors de leurs réserves mécaniques.

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

La [maquette du boîtier v0.7](mechanical/README.md) conserve **155 × 100 × 115 mm**,
la façade amovible avec LCD/A clipsés, l'USB latéral et les passe-paroi arrière.
Elle reprend B v0.5 et supprime la baie XL4015. Les quatre pièces imprimées, les
hauteurs de composants, les 17 étapes de montage/service et les 20 ouïes
latérales sont contrôlées géométriquement. Aucun débit ou échauffement n'est
qualifié. Les [essais physiques restants](review/routing/electronics_bench_plan.md)
sont consignés ; le dernier audit confirme les simulations existantes.


```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_schematics.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_routing.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_placement.py
```

Résultat livré : **ERC zéro erreur/avertissement, DRC zéro violation, zéro connexion restante et zéro écart schéma/PCB sur les deux cartes**. Les pads sont comparés aux netlists et les trajets principaux de puissance sont contrôlés. Les chaînes série des six voyants, leur polarité, leurs rails/retours et l'absence de charge ajoutée au pont NTC sont vérifiés dans `review/routing/diagnostic_validation.json`. `review/routing/` contient les rapports et les PDF relus. Le routage initial a été amorcé avec Freerouting 2.5.0 **localement**, puis corrigé dans les fichiers natifs. La révision 0.4 ajoute uniquement des branches de faible courant, avec le routeur local conservateur `route_diagnostics.py` puis remplissage et DRC KiCad, en conservant toutes les pistes/vias préexistants. Aucune API distante n'a reçu les cartes. Les `.dsn/.ses` et journaux sont des intermédiaires ignorés par Git. Les scripts initiaux restent des étapes historiques et refusent d'écraser un PCB déjà routé ; `add_diagnostic_leds.py` est également une migration unique, pas un générateur à relancer sur les fichiers livrés.

Les packs v03/v04 et les STEP mécaniques jusqu’à v0.6 sont historiques ; les fichiers natifs A v0.4 / B v0.5 et le pack v0.5 sont la référence électrique actuelle ; la maquette v0.7 reprend B v0.5 et la ventilation latérale.

Références : [JST-XH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf), [driver UCC27517](https://www.ti.com/lit/gpn/UCC27517), [IRLZ44N](https://www.infineon.com/part/IRLZ44N), [DMP6023LE](https://www.diodes.com/datasheet/download/DMP6023LE.pdf), [fusibles 451/453](https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1), [STPS5L60](https://www.st.com/resource/en/datasheet/stps5l60.pdf), [bornier Phoenix](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721?type=pdf), [ADS1115](https://www.ti.com/lit/ds/symlink/ads1115.pdf).

## Buck intégré et simulation KiCad

La [variante LM2596 fixe 5 V](power_study/lm2596/README.md), simulée dans KiCad, est intégrée à B v0.5. Les essais non convergents du LMR36520 sont conservés comme historique. Les six simulations réussies et le routage intégré ne remplacent pas les essais thermiques et de défaut.

B v0.5 conserve les dimensions, les connecteurs restants et les bandes de clips.
Le STEP v0.7 est la référence mécanique courante, avec B réimplantée et sans
berceau XL4015 ; les versions antérieures restent historiques. Le pack v0.5 et
les fichiers natifs sont la référence électrique ; A reste à v0.4.
