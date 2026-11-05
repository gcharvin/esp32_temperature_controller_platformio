# PCB B v0.5 — buck 5 V intégré

Révision du 5 novembre 2026. Ouvrir `pcb_b_temperature.kicad_pro`, puis le
schéma ou `pcb_b_temperature.kicad_pcb`. La carte garde **90 × 65 × 1,6 mm**,
deux couches externes de cuivre **70 µm**, les réserves de clips et les positions
des connecteurs conservés. A reste à v0.4.

## Alimentation

```text
J2 24V → F1 → Q2 anti-inversion → VIN_PROTECTED → J3 chauffage
                                   │
                                   └→ U1 LM2596 → L1 → +5V_BUCK → F2 1A → J1/A et logique B
                                                  │
                                                  └→ C10 → R13 → PGND

PGND puissance ── NT1, jonction cuivre près de l'entrée ── GND logique
```

| Repère | Élément | Empreinte / rôle |
| --- | --- | --- |
| U1 | LM2596S-5.0/NOPB | TO-263-5, languette **broche 3 / PGND** |
| L1 | Bourns SRP1770TA-101M, 100 µH | 16,9 × 16,9 mm, hauteur 7 mm |
| D9 | Vishay SS36-E3/57T, 60 V / 3 A | SMC, cathode sur BUCK_SW |
| C8 | Panasonic EEUFR1H101, 100 µF / 50 V | Condensateur d'entrée existant, déplacé ; Ø8 mm, pas 3,5 mm |
| C9 | 100 nF / 50 V X7R | 0805, découplage d'entrée local |
| C10 | Panasonic EEUFR1V331UB, 330 µF / 35 V | Ø10 mm, pas 5 mm, hauteur 12,5 mm |
| R13 | Vishay WSL1206R0560FEA, 56 mΩ / 1 % / 0,25 W | Empreinte locale `LabHardware:Vishay_WSL1206_6m_to_200m`, **dans le retour du condensateur C10**, pas en série avec la charge |
| NT1 | NetTie-2_SMD_Pad2.0mm | Cuivre de liaison des masses, exclu de la BOM d'assemblage |

La revue de fabrication adapte les pastilles R13 au dessin Vishay :
1,65 × 1,93 mm, centres espacés de 3,45 mm, intervalle intérieur 1,80 mm.
R13 est légèrement déplacée, de (20,5 ; 45) à (21 ; 44,5) mm dans le repère
local du PCB, pour dégager les zones de C10 et F2. Les pistes existantes restent
connectées dans les nouvelles plages. Les contrôles DRC et de démontage
mécanique ont été régénérés. Voir le [dossier de fabrication](../manufacturing/README.md)
et son plan d'assemblage pour les polarités et les coordonnées.

J8 et J9 et le module XL4015 sont supprimés. U1 a une sortie fixe 5 V : aucune
consigne à régler. Sa broche 5 ON/OFF est reliée à PGND ; le buck est activé
avec le 24 V. La limite étudiée est **0,8 A pour le système** ; F2 reste à 1 A.
La revue d'approvisionnement propose C9 **C0805C104K5RACTU** et R13
**WSL1206R0560FEA**. Leurs caractéristiques nominales correspondent aux
exigences ; disponibilité, pastilles recommandées et procédé restent à
approuver dans `../manufacturing/README.md` avant gel des MPN dans KiCad.

A reçoit toujours 5 V régulé. Les futures B pourront recevoir 12 V ou 5 V
avec leur propre adaptation d'alimentation ; cette B v0.5 n'est pas déclarée
multitension. Le démarrage du cœur du buck à 12 V est étudié séparément ;
aucune puissance de chauffage 100 W à 12 V/5 V n'est attribuée à cette carte.
Voir `../manufacturing/power_and_usb_review.md`.

Les masses gardent deux noms pour organiser leurs retours. NT1 les relie
électriquement près de l'entrée ; ce n'est pas une isolation. Le trajet du
chauffage jusqu'à J2 reste sur sa piste PGND de puissance, sans passer dans
NT1, les JST ou la DevKit. Les cinq trajets principaux de chauffage sont
contrôlés géométriquement et restent sans via discret.

## Placement, routage et contrôle

Le buck occupe la partie gauche et centrale de B. Plusieurs composants de
logique et voyants sont déplacés. Les JST A/B J1/J6/J7, la sonde J4 et les
borniers J2/J3 gardent leurs positions et brochages. `../placement.json` décrit
le placement courant et les hauteurs, sans enveloppes provisoires sans pads.

Le schéma natif, la BOM et le routage sont mis à jour ensemble. Résultat des
contrôles : **ERC 0 erreur / 0 avertissement, DRC 0 violation, 0 connexion
restante, 0 écart schéma/PCB**. Le rapport du buck et la conservation des
connecteurs sont dans `../review/routing/integrated_buck_validation.json`.
Les PDF de revue sont `../review/pcb_b_temperature.pdf` et
`../review/routing/pcb_b_temperature_routed.pdf`.

Le routage des boucles de commutation est explicite, avec un plan PGND local
sous la languette thermique. Les liaisons restantes ont été amorcées localement
avec Freerouting 1.9.0 en mode **un seul thread**, analytics désactivées, puis
corrigées et contrôlées dans KiCad. Aucune API de routage distante n'a reçu la
carte. Les scripts de migration sont des étapes historiques à appliquer à une
copie de v0.4 ; ils ne régénèrent pas les fichiers natifs livrés après édition.

## Simulation et limites

Le circuit de puissance reprend l'[étude LM2596](../power_study/lm2596/README.md) :
six essais d'acceptation complets sous ngspice, plus deux sensibilités ESR.
C8 fournit les 100 µF d'entrée étudiés ; C9 ajoute le découplage HF. Le banc
représente les ESR/DCR, mais pas les parasites des pistes, vias, connecteurs,
la protection amont ni le couplage du PWM chauffage. Le rattachement des nets
et les valeurs principales sont contrôlés sur la netlist exportée de B ; la
simulation n'est pas une extraction parasite du PCB routé.

La pointe de démarrage simulée approche **4,9 A dans L1**. Saturation réelle,
diode, fusibles, transitoires d'entrée, thermique, bruit sur la NTC et défauts
restent à vérifier sur prototype. Ce n'est pas un dossier libéré pour fabrication.
Le partage USB/VBUS et la mise à jour du firmware restent des sujets distincts.

**Mécanique :** le [STEP v0.8](../mechanical/README.md) représente maintenant ce
placement de B et ses hauteurs. Le berceau du XL4015 est retiré ; quatre pièces
imprimées et 17 étapes de montage/service passent les contrôles géométriques.
J1 reste proche de C10 : la fiche XHP documentée et sa marge passent, mais la
prise en main, les fils, la flexion des clips et la thermique exigent des essais
réels. Voir le [plan de mise au point](../review/routing/electronics_bench_plan.md).

Sources : [TI LM2596](https://www.ti.com/lit/ds/symlink/lm2596.pdf),
[Bourns SRP1770TA](https://www.bourns.com/docs/Product-Datasheets/srp1770ta.pdf),
[Vishay SS36](https://www.vishay.com/docs/88751/ss32.pdf),
[Panasonic C10](https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFR1V331UB).

La préparation3 fige les MPN dans KiCad, emploie les pastilles Kingbright pour les voyants et décale D7 de0,25mm en Y pour dégager SDA. Les borniers sont modélisés à14mm de hauteur enveloppe. Les valeurs électriques et les pistes sont conservées. Les instructions de câblage, de test et d’assemblage sont dans le dossier de fabrication.
