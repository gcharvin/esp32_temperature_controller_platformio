# Premiers schémas KiCad A et B température

Révision 0.2 du 4 novembre 2026. Cette révision remplace le JST 12 positions par trois petits JST-XH de fonctions distinctes. Schémas de conception à examiner, pas des fichiers prêts à fabriquer. Les projets contiennent aussi une première implantation mécanique non routée : [dimensions et maintien](IMPLANTATION.md), [PDF de revue](review/implantation_v01.pdf). Aucun Gerber ou dossier d'assemblage définitif n'est fourni à ce stade.

Une [maquette 3D du boîtier](mechanical/README.md) fournit l'assemblage STEP compatible avec l'import Fusion 360, les quatre pièces STL d'essai dont la façade amovible portant le LCD et A, les rainures continues et clips latéraux pour A, B et le LCD, le panneau arrière amovible avec deux presse-étoupes et une prise sonde à deux contacts, et un accès USB-C de maintenance sur le côté droit. La sélection des sources 5 V USB/externe reste à mettre au point avant monitoring sous 12 V.

## Ouvrir les projets

- `pcb_a_display/pcb_a_display.kicad_pro` : façade, ESP32 et périphériques.
- `pcb_b_temperature/pcb_b_temperature.kicad_pro` : alimentation, acquisition NTC et puissance chauffage.
- `review/pcb_a_display.pdf` et `review/pcb_b_temperature.pdf` : exports de lecture.

Sources natives éditables et versionnées : les fichiers `.kicad_sch` sont la référence. Les symboles utilisés y sont embarqués. `lab_draft.kicad_sym` contient trois interfaces logiques provisoires (DevKit, convertisseur et MOSFET), référencées par les tables locales de bibliothèques. Il ne faut pas en déduire un brochage physique.

## Architecture retenue

Un seul ESP32, enfichable sur A. B n'a pas de microcontrôleur. Le 12 V continu entre sur B par J2 et traverse F1 puis D1 (protection série contre l'inversion). `VIN_PROTECTED` alimente le chauffage et le module buck 12 V vers 5 V. D2 est une TVS bidirectionnelle dont la tension et l'énergie admissible restent à choisir.

B fournit le 5 V à A par J1 (2 positions). J6 transporte le bus I2C et J7 les commandes. Les deux cartes ont chacune leur régulateur AP2112K-3.3 pour les périphériques locaux. La DevKit reçoit le 5 V et utilise sa propre régulation interne pour l'ESP32. Sa sortie 3V3 n'est pas mise en parallèle avec le régulateur de A. Les rails `+3V3_A` et `+3V3_B` ne sont jamais reliés, les masses le sont.

Le courant chauffage circule de l'entrée protégée vers J3.1, traverse la charge externe puis revient par J3.2 et le MOSFET Q1 à la masse d'entrée. Il ne doit pas traverser J1 ni la DevKit. Le routage devra séparer ce trajet des retours d'acquisition. Les protections et sections de la branche logique seront dimensionnées avec le buck et la charge, avant fabrication.

## Connecteurs communs A et B

La Rev 0.2 remplace le connecteur 12 broches de la Rev 0.1 par **trois JST-XH au pas 2,50 mm**. Chaque faisceau est droit : même référence et même numéro de broche sur A et B. Les tailles 2, 3 et 6 positions distinguent les fonctions de la liaison A-B. Ne pas les confondre avec d'autres connecteurs périphériques du même format.

| Connecteur sur A et B | Broche | Signal | Sens / usage sur B température |
| --- | --- | --- | --- |
| J1 — alimentation, 2 positions | 1 | GND | Retour de l'alimentation logique |
| J1 | 2 | +5V_AB | B vers A, sortie du buck |
| J6 — I2C, 3 positions | 1 | GND | Référence commune |
| J6 | 2 | I2C_SDA | Bidirectionnel, ADS1115 SDA |
| J6 | 3 | I2C_SCL | A vers B, ADS1115 SCL |
| J7 — commandes, 6 positions | 1 | GND | Référence commune |
| J7 | 2 | PWM0 | A vers B, commande chauffe |
| J7 | 3 | ENABLE | A vers B, autorisation active à 1 |
| J7 | 4 | FAULT_N | B vers A, ADS1115 ALERT/RDY actif à 0 |
| J7 | 5 | PWM1 | A vers B ; non connectée sur B température |
| J7 | 6 | DIN0 | B vers A ; non connectée sur B température |

Signaux logiques 3,3 V exclusivement. Aucun rail 3,3 V n'est transporté : les deux régulateurs locaux restent séparés. Aucun 12 V ni courant chauffage ne passe dans ces faisceaux. Les broches de masse sont communes électriquement ; dimensionner le retour J1 pour l'alimentation de A sans compter sur les fils de référence des deux autres câbles pour augmenter le courant admissible.

`interface_ab.json` donne les broches par connecteur et les réserves non connectées sur B. Le vérificateur contrôle les réseaux et les empreintes exportés, notamment l'absence de l'ancien J1 à 12 broches. Le firmware et les GPIO ne changent pas par rapport à la proposition précédente.

| Connecteur | Embase proposée sur chaque PCB | Boîtier de câble à chaque extrémité |
| --- | --- | --- |
| J1 | B2B-XH-A, verticale, 2 positions | XHP-2 |
| J6 | B3B-XH-A, verticale, 3 positions | XHP-3 |
| J7 | B6B-XH-A, verticale, 6 positions | XHP-6 |

Les empreintes KiCad JST-XH verticales correspondantes sont assignées sur les deux cartes. Choisir les contacts à sertir et les fils compatibles avec les boîtiers, le courant et les recommandations fabricant ; des câbles préassemblés sont préférables quand leur brochage est vérifié. L'orientation verticale reste à confronter au boîtier avant routage. Vérifier la broche 1 côté accouplement : les vues des cartes peuvent apparaître en miroir.

Câbles internes courts uniquement. **Brancher et débrancher les trois faisceaux avec les deux cartes hors tension** : il n'y a ni séquencement de connexion ni protection contre l'alimentation parasite par les signaux. Marquer les ports POWER, I2C et CTRL, avec la broche 1 et les signaux sur la sérigraphie prévue. Les anciens faisceaux 12 broches ne sont pas directement compatibles avec cette révision.

### Petits connecteurs courants

Préférence utilisateur : connecteurs et câbles accessibles au grand public, avec famille, pas, nombre de contacts et sens de numérotation explicites. Privilégier les formats 2 à 6 positions ; ne pas augmenter le nombre de broches d'un connecteur unique pour couvrir toutes les extensions possibles. Des ports séparés sont autorisés pour faciliter le câblage, le boîtier et le remplacement de B.

- Liaison interne finale : privilégier de petits JST-XH au pas 2,50 mm, sous réserve de l'encombrement, du courant et de câbles adaptés. « JST » seul n'est pas une référence suffisante.
- Prototypage, mesures et modules : barrettes au pas 2,54 mm pour les cordons couramment appelés Dupont. Ne pas supposer leur interchangeabilité mécanique avec les JST-XH de 2,50 mm, ni utiliser une même empreinte pour les deux sans validation.
- Entrée 12 V et sortie chauffage : borniers ou connecteurs de puissance courants, choisis selon le courant, les fils et la place dans le boîtier. Le choix Dupont ne doit pas être appliqué automatiquement au trajet chauffage.
- Choisir si possible les faisceaux déjà assemblés ; sinon, spécifier aussi les boîtiers, contacts à sertir et fils. Prévoir marquages et détrompage, surtout si plusieurs connecteurs ont la même taille. Un connecteur de même famille ne garantit pas le même brochage entre modules du commerce.

La répartition J1/J6/J7 est désormais appliquée aux schémas. Les ports I2C de capteurs sur A conservent leurs 4 positions avec alimentation locale ; J6 A-B a 3 positions et ne transporte pas de rail 3,3 V. Ne pas utiliser un câble de capteur à la place du faisceau J6.

Références de pas : [JST-XH 2,50 mm](https://www.jst-mfg.com/product/index.php?lang=2&series=277) ; [barrettes et cordons de prototypage 2,54 mm](https://www.pololu.com/category/19/connectors).

## Carte A

- U1 est le **bloc logique d'une DevKit ELEGOO**, pas un module ESP32 nu. Ses numéros correspondent aux GPIO / noms de rails ; les vrais pads de headers restent à relever. Aucune empreinte de DevKit supposée n'est assignée.
- J2 : GND, 3V3_A, SDA, SCL vers le LCD 20x4 et son backpack, comme le prototype existant. Vérifier le fonctionnement garanti en 3,3 V, le courant du rétroéclairage et l'ordre réel du connecteur LCD.
- J3 : encodeur A, B, bouton, masse. Trois pull-ups de 10 kohms.
- J4/J5 : modules TMP117 et BH1750, respectivement aux adresses 0x48 et 0x23. Ce sont des connecteurs pour modules externes, pas les circuits intégrés eux-mêmes.
- R4/R5 : pull-ups I2C 4,7 kohms sur A uniquement. Retirer ou calculer les pull-ups supplémentaires des modules et du backpack.
- U2 : alimentation des périphériques ; budget de courant et dissipation à vérifier, notamment avec le LCD. Condensateurs d'entrée et de sortie de 1 uF, plus 10 uF sur le 5 V.
- JP1 : lien amovible du 5 V vers la DevKit. Ce header doit recevoir un cavalier en fonctionnement normal. Il ne commute pas automatiquement les alimentations.

USB : ne pas mettre en parallèle VBUS et le 5 V externe sans connaître le circuit de la DevKit exacte. Pour le flash USB seul, couper B, déconnecter J1, J6 et J7 et retirer JP1 ; ce mode n'alimente pas les périphériques. Pour un debug série avec B alimentée, utiliser une liaison USB dont VBUS est réellement isolé et vérifier l'absence de retour d'alimentation sur cette DevKit. Une sélection automatique des sources n'est pas implémentée.

| Fonction | GPIO proposé | État du firmware existant |
| --- | --- | --- |
| SDA / SCL | 21 / 22 | Conservés |
| Encodeur A / B / bouton | 4 / 19 / 23 | Conservés |
| PWM0 chauffage | 25 | Remplace le GPIO5 actuel |
| PWM1 réserve | 26 | Nouveau |
| ENABLE | 27 | Nouveau |
| DIN0 réserve | 32 | Nouveau, laisser inutilisé sur B température |
| FAULT_N | 33 | Nouveau |

Le firmware n'est pas modifié par cette livraison. Ne pas flasher le firmware actuel en supposant ce brochage compatible.

## Carte B température

- J2 : entrée 12 V / masse. J3 : chauffage + / retour commuté. J4 : sonde NTC externe, entre 3V3_B et NTC_RAW.
- NTC 15 kohms à 25 °C, B proche de 3920 K ; résistance basse R5 de 8,45 kohms à 1 %. Filtre R6 = 1 kohm et C4 = 100 nF vers AIN0.
- U4 : ADS1115IDGS ; ADDR relié à VDD pour **0x49**, afin d'éviter le TMP117 à 0x48. AIN1–3 non utilisées. Découplage local de 100 nF.
- Calcul : `R_NTC = R5 * (Vexc / Vnode - 1)`. Employer la valeur réelle de R5 et une excitation mesurée/calibrée : la référence interne de l'ADS1115 ne rend pas cette mesure automatiquement ratiométrique. La constante historique 8400 ohms doit être adaptée.
- U3 : 74LVC1G08 alimentée en 3,3 V. `GATE_CMD = PWM0 AND ENABLE`. Pull-downs 47 kohms sur les deux entrées, 100 ohms en série de grille et 100 kohms grille-masse. Q1 doit avoir un Rds(on) garanti à une tension compatible avec 3,3 V, avec un brochage et une dissipation vérifiés.
- FAULT_N est l'ALERT/RDY de l'ADC avec pull-up 10 kohms. Programmer le mode comparateur, les seuils et la polarité ; sans cette configuration, ce fil ne constitue pas une alarme température opérationnelle. La logique NTC exige de vérifier le sens du seuil de tension.
- Il n'y a pas encore de coupure thermique indépendante. La porte AND bloque la chauffe en l'absence de A ou d'ENABLE, mais ne détecte pas un ESP32 figé avec ses sorties actives. Le firmware devra arrêter sur sonde ouverte/court-circuitée, mesure périmée, défaut ADC et dépassement de température ; une protection matérielle adaptée sera à définir avant usage sans surveillance.

## Éléments à arrêter avant implantation

1. Courant nominal/maximal du chauffage, tension réelle et conditions thermiques. Dimensionner Q1, F1, D1, D2, borniers, câblage, pistes et protection de la branche logique.
2. Modèle exact du buck, courant 5 V disponible et brochage physique. Le bloc U1 de B est une interface fonctionnelle, pas une empreinte de module validée.
3. Mesures de la DevKit ELEGOO : nombre de broches, numérotation, entraxes, contour, accès USB, zone libre antenne et alimentation USB.
4. Références/empreintes des connecteurs périphériques et composants provisoires, budget du LCD, dissipation des LDO, découplage final.
5. Coupure thermique indépendante et stratégie de redémarrage/réarmement.
6. Enveloppe du boîtier imprimé en 3D, fixation par clips, accès de démontage et zones mécaniques réservées sur A et B, suivant les contraintes ci-dessous.

Les CSV `*_bom_draft.csv` sont des inventaires de revue. Les champs de choix fournisseur sont à compléter ; ils ne sont pas des BOM d'assemblage JLCPCB. Aucun CPL n'existe avant placement.

## Boîtier imprimé en 3D et maintien sans vis

Contrainte utilisateur retenue : les deux cartes seront logées dans un boîtier imprimé en 3D, avec un maintien privilégiant des clips imprimés plutôt que des vis. La carte métier B doit rester remplaçable. Le boîtier et les PCB seront conçus ensemble avant de figer les contours, les empreintes et le routage. Les dimensions, le matériau d'impression, les jeux et la géométrie des clips restent à définir ; aucun ajustement mécanique n'est encore qualifié.

Disposition de départ à étudier : A derrière la façade écran/encodeur, B dans un logement distinct, reliée à A par les trois faisceaux courts J1, J6 et J7. Les faisceaux permettent d'adapter la position relative des cartes sans imposer un empilage ou un connecteur rigide carte à carte. Cette disposition n'est pas un plan coté définitif.

Exigences pour la conception mécanique et le placement :

- Prévoir des appuis et des butées pour positionner les cartes, puis des clips accessibles pour les retenir et les libérer sans détériorer les PCB. L'insertion et le retrait de B doivent être possibles après ouverture du boîtier, sans déposer toute la façade.
- Réserver aux bords des cartes les surfaces de contact des glissières, appuis et clips : volumes sans composants ni soudures exposées aux contacts, avec exclusions de cuivre et de pistes définies selon la géométrie finale. Prévoir aussi le passage des clips pendant l'insertion et l'accès pour les déverrouiller. Ne pas utiliser la DevKit ou un composant comme point d'appui du boîtier.
- Conserver une zone libre autour de l'antenne ESP32, cohérente avec les prescriptions du module exact ; coordonner sa position avec la paroi, le câblage et les éléments voisins.
- Aligner les interfaces utilisateur et externes avec les ouvertures : LCD, axe et bouton de l'encodeur, USB, entrée d'alimentation, sortie chauffage et sonde. Vérifier l'accès aux verrous JST, aux vis des borniers, à JP1 et aux boutons de la DevKit.
- Prévoir le volume des fiches accouplées, leur trajectoire d'insertion, la courbure du faisceau et le maintien des câbles. Les efforts de branchement des câbles externes devront être repris par la structure prévue, sans déclipser la carte ni solliciter uniquement les soudures.
- Réserver les volumes des composants hauts des deux côtés, notamment DevKit, backpack LCD, buck et étage de puissance. Prévoir les distances au plastique et la dissipation de B après dimensionnement ; ne pas placer un clip dans un volume destiné à un composant chaud ou à un dissipateur.

Conséquences sur le schéma : le sens de sortie des connecteurs, leur version droite ou coudée, le choix entre composant directement monté et élément déporté par câble, ainsi que les besoins de refroidissement pourront conduire à modifier les références et les empreintes. Les brochages J1/J6/J7 restent la base actuelle ; toute évolution électrique nécessaire sera répercutée sur les deux schémas et le contrat d'interface. Aucune modification des réseaux n'est nécessaire pour enregistrer cette contrainte à ce stade.

Avant lancement des PCB, vérifier l'assemblage boîtier/cartes avec des modèles 3D aux dimensions réelles et une impression d'essai des zones de clips. Valider l'accès, l'insertion, le démontage répété et le maintien lors du branchement des câbles. Un essai thermique de l'ensemble assemblé complétera cette validation une fois la puissance définie.

## Vérifier et exporter

Avec KiCad installé à l'emplacement actuel sous Windows :

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_schematics.py
```

Ce script lance l'ERC sans exclusions ajoutées au projet, exporte les PDF et netlists, puis contrôle sur les réseaux exportés les trois connecteurs A/B et leurs empreintes JST, les rails séparés, les broches ADC, la commande de grille et le trajet de puissance. Les rapports sont sous `review/`. L'ERC ne valide ni le dimensionnement ni les empreintes ou la sécurité thermique.

`scripts/create_drafts.py` est le générateur initial conservé pour traçabilité. Il refuse d'écraser les schémas existants sans `--overwrite`. Après édition dans KiCad, conserver les fichiers natifs comme référence et ne pas régénérer les schémas ; les exports/BOM devront suivre les sources modifiées.

## Références techniques

- [KiCad : fichiers de schéma](https://dev-docs.kicad.org/en/file-formats/sexpr-schematic/)
- [KiCad : ligne de commande et ERC](https://docs.kicad.org/10.0/en/cli/cli.html)
- [TI : ADS1115, brochage, adressage et comparateur](https://www.ti.com/lit/gpn/ads1115)
- [Diodes : AP2112](https://www.diodes.com/assets/Datasheets/AP2112.pdf)
- [TI : SN74LVC1G08](https://www.ti.com/product/SN74LVC1G08)
