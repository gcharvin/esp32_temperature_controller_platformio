# Maquette 3D v0.7 — PCB B avec buck intégré

Révision du **5 novembre 2026**. Boîtier nu **155 × 100 × 115 mm**, paroi 3 mm.
La carte B v0.5 porte maintenant son LM2596 : le XL4015, son berceau, ses guides,
la cloison de sa baie et sa butée arrière sont supprimés. Le volume libéré à
côté de B peut servir au passage des faisceaux. A reste à v0.4.

## Ouvrir dans Fusion 360

- **`exports/enclosure_v07_integrated_buck.step`** : assemblage complet courant.
- `exports/enclosure_v07_printed_parts.step` : les quatre pièces imprimées seules.
- `exports/enclosure_exploded.step` et `enclosure_printed_parts_exploded.step` : vues éclatées courantes.
- `exports/parts_v07/` : STEP et STL individuels.
- `exports/preview_service.png`, `preview_b_power.png`, `preview_exploded.png` : arrière ouvert, B équipée et vue éclatée.
- `exports/validation.json`, `assembly_sequence_validation.json`, `source_manifest_v07.json` : résultats et traçabilité.
- `exports/enclosure_v07_integrated_buck_fusion_bundle.zip` : STEP, STL, sources, documentation et contrôles regroupés.

Les fichiers numérotés v01 à v06 et `parts_v06/` sont historiques. Les exports
sans numéro listés ci-dessus correspondent à v0.7. Ne pas employer l'ancienne
vue `preview_buck.png`, qui représente encore le module externe.

Le STEP contient une arborescence de solides nommés : **99 solides valides** dans
le complet, **4 solides connectés** dans les pièces imprimées. Il ne transporte
pas l'historique paramétrique natif Fusion. Les sources paramétriques sont
`build_enclosure.py`, `enclosure_parameters.json` et `../placement.json`.
L'import STEP est vérifié par OpenCascade ; l'ouverture dans Fusion n'a pas été
exécutée pour cette révision. Aucun fichier `.f3d` n'est fourni.

## Quatre pièces imprimées

| Pièce | Fonction |
| --- | --- |
| 01 Coque ouverte | Guides et butées de B, fenêtre USB latérale, ventilation sur les deux flancs |
| 02 Façade équipée | Supports à rainures et clips du LCD et de A, intégrés à la façade |
| 03 Berceau B | Rainure basse, guides, deux clips et poignées ; coulissement vers l'arrière |
| 04 Panneau arrière | Lèvre, clips, butée de B, deux presse-étoupes et prise sonde |

Chaque pièce est un seul solide connecté. Le LCD et A se montent sur la façade
hors du boîtier ; B se monte dans son berceau hors du boîtier. Les guides de la
coque et les butées avant/arrière retiennent le berceau. Jeu nominal de guidage
0,25 mm ; jeu de butée arrière 0,3 mm.

Les supports capturent le bord inférieur des PCB sur 3 mm, avec appuis latéraux
1,5 mm et clips latéraux. PCB nominal 1,6 mm, jeu total de rainure 0,6 mm ; bras
de clips 16 × 1,2 mm, crochet 1,8 mm. Les enveloppes de libération sont de 2,4 mm
pour les clips PCB, 1,2 mm pour la façade et 1,5 mm pour le panneau arrière.
Ces jeux et déformations sont des hypothèses d'impression, sans qualification
élastique ou de fatigue.

## Électronique et encombrement

A reste **110 × 75 × 1,6 mm**, B **90 × 65 × 1,6 mm**. Positions conservées :
A à x=8/z=13, face composants y=45 ; B à x=50/z=17, face composants y=85.
Les coordonnées sont x gauche→droite en vue de face, y avant→arrière, z bas→haut.

Les enveloppes de B sont relues depuis l'implantation courante, avec les nouvelles
positions et hauteurs de U1 LM2596 (4,8 mm), L1 (7 mm), D9 (2,4 mm), C8 (11,5 mm)
et C10 (12,5 mm). Q1 demeure le plus haut, 17 mm. Elles restent dans la réserve
nominale de 20 mm ; le plan principal intérieur du panneau arrière est à y=112,
soit 10 mm derrière le sommet de Q1. La place locale des passe-paroi et clips
est contrôlée par intersection, pas seulement par cette distance au plan.

Les composants sont des enveloppes conservatrices de leurs empreintes et hauteurs,
pas des modèles détaillés certifiés. NT1 est du cuivre et n'ajoute aucun volume.
Aucun dissipateur supplémentaire n'est représenté ; un dissipateur éventuel
imposerait une nouvelle revue du STEP.

La DevKit diymore reste enfichable sur A : corps vendeur 54 × 27 mm, deux supports
femelles 1×15, entraxe **25,40 mm confirmé par mesure utilisateur le 5 novembre 2026**,
hauteur des supports 8,5 mm supposée. La géométrie de cet entraxe était déjà correcte.
La cible est désormais le support des trois DevKit au brochage commun confirmé
par l'utilisateur. Les sockets restent identiques ; la rallonge USB-C flexible
est conservée. Le STEP représente seulement la carte de référence : contour,
hauteur, antenne et prise du câble des deux autres variantes restent à comparer
avant de qualifier leur montage. Voir `../devkit_compatibility.md`.
Le LCD de référence a un PCB 98 × 60 mm et un backpack jusqu'à y=28. Ces cotes
restent à mesurer sur le matériel acheté, à l'exception de l'entraxe confirmé.

## Accès aux connecteurs

Les positions de J1/J2/J3/J4/J6/J7 sont celles de B v0.4 ; J8/J9 sont supprimés.
Panneau arrière retiré, les borniers J2/J3 disposent de corridors pour une tige
de tournevis Ø6 mm. Les JST J1/J4/J6/J7 disposent de 35 mm de course réservée
à la fiche depuis l'arrière.

**J1 est proche de C10.** L'ancienne réserve générique (empreinte augmentée de
1 mm de chaque côté) chevauchait l'enveloppe du condensateur. Elle est remplacée
par le corps de fiche **JST XHP** documenté, avec **0,25 mm de marge de chaque
côté** : largeur `(n−1) × 2,5 + 4,8 mm`, profondeur 5,7 mm. Le contrôle de ce
passage réussit ; cela ne représente pas la place des doigts. Source :
[fiche JST-XH, page 4](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf).
Mesurer et essayer les fiches réelles ; tirer sur leur corps, et prévoir du mou
pour intervenir sur B hors de la coque si nécessaire.

Les faisceaux A/B sont des tracés indicatifs dont les extrémités suivent les
connecteurs de l'implantation courante. Leurs rayons de courbure, leur flexibilité
et le mou de service ne sont pas qualifiés par le modèle.

## Montage et démontage

1. **Façade hors coque : LCD avant A et encodeur.** Faire passer le LCD dans le
   cadre A encore vide, incliné de 4°, avec décalage x=+4 mm/z=−1 mm. Après
   passage, recentrer, relever de 4,5 mm, approcher le logement, abaisser de
   3,5 mm dans la rainure puis rabattre jusqu'aux clips. Ne pas charger le verre.
2. Monter l'encodeur, puis A : bord inférieur relevé de 3,5 mm, inclinaison de
   4°, approche, abaissement dans la rainure, rabattement et enclenchement.
3. Charger B dans son berceau sur l'établi, par le même principe. Installer la
   rallonge USB de panneau, fiche interne encore débranchée et câble hors guides.
4. Introduire la façade équipée par l'avant et engager ses clips. Depuis
   l'arrière, **avant B**, brancher l'USB sur la DevKit (course latérale 15 mm).
5. Introduire B par l'arrière jusqu'aux butées, puis raccorder les trois JST A/B,
   le 24 V, le chauffage et la sonde, hors tension. Aucun module externe ni
   faisceau J8/J9 n'est à installer.
6. Ranger les fils avec leur mou de service et clipser le panneau arrière équipé.

Pour démonter : débrancher l'alimentation, libérer le panneau par ses fenêtres,
le reculer selon le mou réel des fils, puis déposer les raccordements nécessaires.
Débrancher B et extraire son berceau de 50 mm vers l'arrière ; la façade et l'USB
restent en place. Déclipser le PCB sur l'établi en inversant sa pose. Pour déposer
la façade, retirer B, débrancher l'USB côté DevKit et les autres faisceaux, libérer
les clips avant et extraire la façade équipée (course vérifiée 100 mm). Sur
l'établi, déposer A puis l'encodeur si nécessaire, enfin le LCD.

## USB et passe-paroi

USB-C de panneau à droite : centre y=64/z=30, trou Ø16, corps Ø15, collerette Ø22,
empiètement intérieur 28 mm. Rallonge **de données** vers le CH340 de la DevKit.
Fiche interne réservée : 14 × 4 × 10 mm. Ces volumes sont des hypothèses non
mesurées ; câble et courbures doivent être vérifiés.

Le panneau arrière conserve le choix utilisateur :

| Fonction | Réservation | Trou | Centre x / z |
| --- | --- | --- | --- |
| Entrée 24 V | Presse-étoupe M12, corps Ø20 | Ø12,5 | 22 / 75 |
| Chauffage | Presse-étoupe M12, corps Ø20 | Ø12,5 | 22 / 35 |
| Sonde NTC | Prise GX12-2, corps Ø17 | Ø12,4 | 108 / 85 |

Les presse-étoupes sont des passages et retenues de câbles, pas des prises
électriques. Choisir les références avec leur plage de câble, filetage et écrou
avant de figer les perçages d'une paroi de 3 mm. Filetages et contacts sont
simplifiés. L'accès USB mécanique ne résout pas la sélection entre VBUS et 5 V
externe ; suivre la procédure électrique de service du dépôt.

## Ventilation latérale

Les **20 ouïes latérales** restent inchangées : cinq basses et cinq hautes par
flanc, fentes oblongues 14 × 4 mm au pas 17 mm. Débuts en profondeur y=22, 39,
56, 73, 90 ; rangées z=5,5 et 90,5. Dessus et dessous restent pleins.
Surface libre totale d'entrée **5,26 cm²**, et autant en sortie. Les volumes
enlevés et l'absence d'intersection avec les guides sont vérifiés.

Le débit d'air et la température ne sont pas simulés. Garder les flancs et le
chemin d'air dégagés ; aucune étanchéité, filtration ou performance thermique
n'est revendiquée.

## Contrôles et suite

Le générateur vérifie les solides, leurs connexions, les racines des clips et les
collisions, puis **17 étapes** de montage/service : mise en place du LCD, de A et
de B, insertion de façade, branchement USB, extraction de B, retrait du panneau
et accès aux connecteurs. Les translations utilisent des volumes balayés ; les
pivots sont échantillonnés tous les 0,1° de 4° à 0°. Les mouvements inverses
supposent les mêmes déconnexions et clips libérés. Les deux rapports ne doivent
contenir aucune collision ni étape en échec.

Les quatre pièces imprimées sont réimportées du STEP avec conservation de leur
volume et un solide connecté par pièce ; le complet est également réimporté.
Ces contrôles ne simulent ni les efforts, les doigts, les tolérances, les fils,
la fatigue des clips ou la dissipation. Essayer un coupon de rainure/clip et un
montage câblé avant impression définitive.

L'[audit électronique](../review/routing/electronics_readiness_v05.json) confirme
les simulations existantes sans nouveau changement du circuit. Les prochains
[essais physiques](../review/routing/electronics_bench_plan.md) portent sur le
buck réel, le chauffage/NTC, les défauts et la thermique en boîtier fermé.

```powershell
uv run hardware/mechanical/build_enclosure.py
```

CadQuery 2.6.1 et Pillow sont déclarés dans le script. Le générateur produit les
STEP, STL, GLB, rendus et rapports sans modifier KiCad. Le pack ZIP est un
instantané à recréer après régénération.
