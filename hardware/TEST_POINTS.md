# Points de test — A 0.4.1 et B température 0.5.1

Révision diagnostic du **6 novembre 2026**. **10 pastilles sur A, 12 sur B**,
diamètre1,5mm, au **dos B.Cu/B.Mask**, sans pâte à braser ni composant rapporté.
Leur numéro TP est sérigraphié au dos ; ce tableau en donne le signal.
Elles sont exclues des achats et des placements usine. Les schémas et PCB
contiennent les mêmes TP et nets ; les pistes d'origine sont conservées.

Mesurer les cartes sorties de leur berceau, alimentation limitée en courant,
chauffage débranché. Pour un test de continuité/résistance, couper toutes les
alimentations et attendre la décharge. Les pastilles ne sont pas des sorties
d'alimentation ni des cavaliers ; ne pas les ponter avec une pointe de mesure.

| Carte A | Signal | Observation attendue au banc |
| --- | --- | --- |
| TP1 | GND | Référence locale des mesures de A |
| TP2 | +5V_AB | Alimentation centrale reçue par A/J1 |
| TP3 | +5V_DEVKIT | VIN5 de la DevKit, après JP1 ; dépend du mode d'alimentation |
| TP4 | +3V3_A | Alimentation locale des périphériques de A |
| TP5 | I²C SDA | Au repos proche du3,3V ; activité lors des échanges |
| TP6 | I²C SCL | Horloge pendant les échanges, cible100kHz du diagnostic |
| TP7 | PWM0 / GPIO25 | Zéro par défaut ;500Hz lors de la commande logique `probe` |
| TP8 | ENABLE / GPIO27 | **Zéro en permanence avec le firmware de diagnostic** |
| TP9 | FAULT_N / GPIO33 | Retour brut, tiré vers3,3V par B si raccordée ; entrée active bas |
| TP10 | ENC_SW | Bouton de l'encodeur, actif bas |

| Carte B | Signal | Observation attendue au banc |
| --- | --- | --- |
| TP1 | PGND | Référence du circuit de puissance et de la grille Q1 |
| TP2 | VIN_PROTECTED | Entrée24V après fusible et protection d'inversion |
| TP3 | +5V_BUCK | Sortie du convertisseur avant F2 |
| TP4 | +5V_AB |5V après F2, envoyé à A |
| TP5 | +3V3_B | Alimentation logique locale de B |
| TP6 | NTC_RAW | Pont de mesure avant filtre ; sonde entre J4.1(+3V3) et J4.2(NTC_RAW) |
| TP7 | NTC_ADC | Entrée AIN0 après R6/C4, valeur affichée en mV par le diagnostic |
| TP8 | PWM0 | Commande arrivant de A |
| TP9 | ENABLE | Autorisation arrivant de A ; zéro avec le diagnostic |
| TP10 | FAULT_N | Ligne ADS/retour brut ; comparateur désactivé dans le diagnostic |
| TP11 | GATE_DRV | Sortie du driver5V vers R3/Q1 ; doit rester nulle avec ENABLE=0 |
| TP12 | GND | Référence logique et acquisition analogique |

GND et PGND sont reliées intentionnellement par NT1, sans isolation galvanique.
Employer GND pour les mesures analogiques/logiques, et PGND pour le driver de
puissance. **HEATER_LOW n'est pas une masse.** Les sondes d'oscilloscope reliées
à la terre doivent garder une référence correcte ; ne pas raccorder leur masse
à un nœud commuté. Aucun TP n'est ajouté au nœud rapide BUCK_SW.

Avec une résistance d'essai15kΩ à la place de la sonde, et3,3V réel, la valeur
idéale de NTC_RAW/NTC_ADC est environ **1,189V**, avec R5=8,45kΩ vers GND.
Ce repère suppose le câblage courant, hors tolérances ; ce n'est pas une
calibration de température. Sonde débranchée : proche0V ; court-circuit aux
deux contacts de la sonde : proche3,3V. Faire ces configurations hors tension.

Les cartes de repérage [A vue de dos](review/test_points_A_back.png) et
[B vue de dos](review/test_points_B_back.png) représentent un retournement
autour de l'axe vertical. Le [CSV des positions](review/test_point_map.csv)
conserve au contraire les coordonnées KiCad **vues de face**, pour éviter
une double inversion dans les données fabricant.

Le [firmware de diagnostic](../firmware/diagnostic/README.md) fournit la lecture
série et une page Wi-Fi locale sans USB sous alimentation centrale. Il n'autorise
jamais le chauffage. Suivre ensuite le [plan de banc](review/routing/electronics_bench_plan.md)
pour les essais de puissance et de défaut. Aucun test physique n'est déclaré fait.
