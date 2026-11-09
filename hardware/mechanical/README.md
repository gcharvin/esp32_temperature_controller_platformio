# Maquette 3D v0.10 — jack de sonde compact et PCB B avec buck intégré

Révision du **9 novembre 2026**. Boîtier nu **155 × 100 × 115 mm**, paroi 3 mm.
La carte B v0.5 porte maintenant son LM2596 : le XL4015, son berceau, ses guides,
la cloison de sa baie et sa butée arrière sont supprimés. Le volume libéré à
côté de B peut servir au passage des faisceaux. A reste à v0.4.

## Ouvrir dans Fusion 360

- **`exports/enclosure_v10_integrated_buck.step`** : assemblage complet courant.
- `exports/enclosure_v10_printed_parts.step` : les six pièces imprimées seules.
- `exports/enclosure_exploded.step` et `enclosure_printed_parts_exploded.step` : vues éclatées courantes.
- `exports/parts_v10/` : STEP et STL individuels.
- `exports/preview_service.png`, `preview_b_power.png`, `preview_exploded.png` : arrière ouvert, B équipée et vue éclatée.
- `exports/preview_light_sensor.png`, `preview_light_sensor_front.png` : fixation intérieure et fenêtre extérieure, vues locales de la géométrie réelle.
- `exports/validation.json`, `assembly_sequence_validation.json`, `source_manifest_v10.json` : résultats et traçabilité.
- `exports/enclosure_v10_integrated_buck_fusion_bundle.zip` : STEP, STL, sources, documentation et contrôles regroupés.

Les fichiers numérotés v01 à v09 et leurs dossiers de pièces sont historiques. Les exports
sans numéro listés ci-dessus correspondent à v0.10. Ne pas employer l'ancienne
vue `preview_buck.png`, qui représente encore le module externe.

Le STEP contient une arborescence de solides nommés : **105 solides valides** dans
le complet, **6 solides connectés** dans les pièces imprimées. Il ne transporte
pas l'historique paramétrique natif Fusion. Les sources paramétriques sont
`build_enclosure.py`, `light_sensor_mount.py`, `enclosure_parameters.json` et `../placement.json`.
L'import STEP est vérifié par OpenCascade ; l'ouverture dans Fusion n'a pas été
exécutée pour cette révision. Aucun fichier `.f3d` n'est fourni.

## Six pièces imprimées

| Pièce | Fonction |
| --- | --- |
| 01 Coque ouverte | Guides et butées de B, fenêtre USB latérale, ventilation sur les deux flancs |
| 02 Façade équipée | Supports à rainures et clips du LCD et de A, intégrés à la façade |
| 03 Berceau B | Rainure basse, guides, deux clips et poignées ; coulissement vers l'arrière |
| 04 Panneau arrière | Lèvre, clips, butée de B, deux presse-étoupes et prise sonde |
| 05 Fenêtre translucide | Insert à collerette, membrane extérieure fermée de 0,6 mm ; filament translucide |
| 06 Cassette BH1750 | Porte le petit PCB avec deux clips, puits opaque vers la fenêtre ; retenue par deux clips de façade |

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
3. Monter la fenêtre puis la cassette BH1750 suivant la séquence détaillée ci-dessous.
   Charger B dans son berceau sur l'établi, par le même principe. Installer la
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
l'établi, déposer A puis l'encodeur si nécessaire, enfin le LCD. La cassette BH1750
se déclipse séparément, depuis l'intérieur ; la fenêtre se retire ensuite vers l'intérieur.

## BH1750 derrière une zone translucide fermée

La fenêtre est centrée à **x=126 / z=77 mm**, au-dessus de l'encodeur et à droite
du LCD. L'insert a une face de 8 × 8 mm, une membrane de **0,6 mm** et une
collerette intérieure de 12 × 12 mm. Il entre depuis l'intérieur dans un
logement étagé ; la cassette le retient. L'extérieur reste fermé par la membrane,
sans ouverture directe devant le capteur. L'ajustement périphérique n'est pas
un joint d'étanchéité : aucune protection IP n'est revendiquée.

Imprimer cette petite pièce seule dans un matériau **réellement translucide**,
par exemple un PETG naturel à qualifier. Une paroi opaque ne devient pas une
fenêtre par calibration. Les cinq autres pièces peuvent être opaques ; la cassette
et son puits doivent bloquer la lumière du LCD. Pour 0,6 mm, trois couches de
0,2 mm donnent une première éprouvette, à confirmer au tranchage et à la mesure.
Orienter la face extérieure de l'insert à plat sur le plateau : l'épaisseur est
suivant Y dans le modèle, et non Z. Les STL sont exportés en position d'assemblage,
sans orientation d'impression automatiquement qualifiée.

Le PCB du capteur est une **hypothèse 23 × 16 × 1,6 mm**, détecteur centré
et face sensible vers l'avant. La référence exacte du breakout n'est pas connue.
Le détecteur représenté n'est pas un modèle fournisseur ; le connecteur et son
câble restent à relever. Ne pas lancer l'impression définitive de la cassette
sur la seule base de cette hypothèse. Les cotes sont dans `light_sensor` du JSON ;
le générateur demande une revue explicite des clips si l'on change le format PCB.

La membrane est séparée de la face du détecteur par **5,6 mm d'air**, sans appui
mécanique sur son boîtier. Le puits intérieur opaque de 6,8 × 6,8 mm limite le
champ angulaire. La mesure sera une indication de lumière ambiante à calibrer,
pas un luxmètre qualifié avec réponse angulaire conservée.

Montage, façade déposée :

1. Insérer l'insert depuis l'arrière jusqu'à sa collerette dans le lamage.
2. Sur l'établi, écarter les deux clips latéraux de la cassette et poser le PCB,
   capteur tourné vers le puits. Relâcher les clips ; vérifier l'appui et le jeu.
3. Écarter les deux clips de façade vers le haut/bas et approcher la cassette
   équipée depuis l'arrière. La plaquer contre ses quatre appuis puis relâcher
   les clips. Leur crochet retient la plaque arrière de cassette, qui capture
   la collerette de fenêtre. Raccorder le câble à **A/J5**, selon son brochage.
4. Pour déposer, débrancher le câble, libérer les deux clips de façade et reculer
   la cassette ; ses clips PCB sont alors accessibles sur l'établi. La fenêtre
   se retire librement une fois la cassette retirée.

Les trois translations sont vérifiées par volumes balayés, clips représentés
en position libérée. L'insertion de la façade équipée, l'accès USB et l'extraction
de B sont aussi recalculés avec le capteur en place. Les doigts, le connecteur
réel et le comportement élastique des nouveaux clips restent des essais physiques.

Après impression : relever plusieurs niveaux et angles de lumière avec/sans
fenêtre, puis comparer LCD allumé/éteint à ambiance constante. Déterminer la
correction et le bruit utile ; modifier matériau/épaisseur/puits si le signal est
trop faible ou pollué. Le rétroéclairage reste fixe avec extinction/réveil selon
la décision utilisateur. Voir l'[étude du LCD Freenove et du PWM](../display_backlight_study.md).

## USB et passe-paroi

USB-C de panneau à droite : centre y=64/z=30, trou Ø16, corps Ø15, collerette Ø22,
empiètement intérieur 28 mm. Rallonge **de données** vers le pont USB/série de la DevKit.
Fiche interne réservée : 14 × 4 × 10 mm. Ces volumes sont des hypothèses non
mesurées ; câble et courbures doivent être vérifiés.

Le panneau arrière conserve le choix utilisateur :

| Fonction | Réservation | Trou | Centre x / z |
| --- | --- | --- | --- |
| Entrée 24 V | Presse-étoupe M12, corps Ø20 | Ø12,5 | 22 / 75 |
| Chauffage | Presse-étoupe M12, corps Ø20 | Ø12,5 | 22 / 35 |
| Sonde NTC | Jack PJ-392, corps enveloppe Ø8,2 | Ø6,3, logement extérieur Ø14 | 108 / 85 |

### Montage du jack PJ-392

Le GX12 est remplacé par le **jack 3,5 mm PJ-392**, candidat Amazon
[DAOKI B08XM6DR96](https://www.amazon.fr/dp/B08XM6DR96), avec fiche TRS assortie.
Le [dessin Tayda A-2585/PJ-392](https://www.taydaelectronics.com/datasheets/files/PJ-392.pdf)
a été lu visuellement : filetage **M6×0,5, longueur 4,5 mm**, corps **11,8 mm**,
longueur totale avec pattes **22,8 mm**. Il reste **18,3 mm** derrière la face
d'appui du corps, hors marge de soudure. Ce modèle est plus étroit que le GX12 ;
il n'est pas prouvé plus court en profondeur avec ses pattes intactes.

Perçage nominal **Ø6,3 mm** (jeu d'impression 0,3 mm sur M6), avec logement
extérieur **Ø14 mm, profondeur 1,5 mm**. La paroi locale est ainsi **1,5 mm**.
La longueur de filetage libre est 3 mm : le modèle réserve 1,5 mm pour l'écrou,
0,3 mm pour la rondelle et au moins 0,5 mm de marge. Les dimensions d'écrou et
rondelle sont des hypothèses d'enveloppe, pas des cotes certifiées DAOKI.
Le modèle conserve 23 mm libres derrière le panneau comme objectif de soudure ;
les fils et les doigts ne sont pas simulés. Orienter les pattes pour faciliter
la soudure sans traction ; sortir le panneau avant démontage de B.

**L'équivalence dimensionnelle du lot Amazon avec le dessin Tayda n'est pas
vérifiée.** La référence alternative documentée est Tayda **A-2585**. Imprimer
d'abord `98_Optional_jack_fit_coupon.stl` dans `parts_v10/`, puis tester le jack,
sa rondelle et son écrou réels. Le coupon est séparé des six pièces du boîtier.
Le jeu FDM, la rigidité locale et la tenue au serrage restent à essayer.

La sonde utilise **tip et sleeve**, ring isolé : B/J4.1 (+3V3_B) vers tip,
B/J4.2 (NTC_RAW) vers sleeve. Sleeve n'est donc **pas une masse** ; ne pas relier
le corps du jack à GND ou à un châssis conducteur. Un jack peut court-circuiter
des contacts pendant son insertion : raccorder hors tension uniquement.

Presse-étoupes : référence Amazon [HB-Digital B0CLLQCCWW](https://www.amazon.fr/dp/B0CLLQCCWW),
M12 pour gaine **3–6,5 mm**, deux par appareil. Le perçage reste Ø12,5 mm.
Aucun plan coté spécifique HB-Digital n'a été obtenu : les corps Ø20 et réserves
12/14 mm restent des enveloppes à vérifier. La liste d'achat ne transforme pas
ces volumes en pièces fournisseur qualifiées.

Le [tableau détaillé des faisceaux](../manufacturing/wiring_and_harnesses.md)
sépare câbles finis, adaptateurs accouplés et soudures restantes. La maquette
ne qualifie pas le mou d'un câble commercial de 15 cm ou son rayon de courbure.

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
collisions, puis **20 étapes** de montage/service : mise en place du LCD, de A et
de B, insertion de façade, branchement USB, extraction de B, retrait du panneau
et accès aux connecteurs. Les translations utilisent des volumes balayés ; les
pivots sont échantillonnés tous les 0,1° de 4° à 0°. Les mouvements inverses
supposent les mêmes déconnexions et clips libérés. Les deux rapports ne doivent
contenir aucune collision ni étape en échec.

Les six pièces imprimées sont réimportées du STEP avec conservation de leur
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
python hardware/mechanical/package_enclosure.py
```

CadQuery 2.6.1 et Pillow sont déclarés dans le script. Le générateur produit les
STEP, STL, GLB, rendus et rapports sans modifier KiCad. Le pack ZIP est un
instantané à recréer après régénération avec `package_enclosure.py` ; ce script
vérifie les sources et les SHA256 des membres de l'archive.

## Révision de préparation v0.8

Les deux sockets utilisent désormais les enveloppes maximales Harwin38,85×2,65×8,65mm. La pile réserve aussi2,54mm pour le plastique des broches mâles et0,15mm de montage, ce qui recule le PCB et la fiche USB. Les borniers Phoenix1715721 utilisent14mm de hauteur enveloppe au lieu de9,8mm, pour leur hauteur montée13,8mm. Les quatre pièces imprimées restent identiques ; les enveloppes électroniques et les chemins de service sont régénérés. La cote de l’entretoise mâle et les corps des trois DevKit restent à contrôler sur le matériel. Les sources sont figées pendant toute génération ; une modification concurrente fait échouer le manifeste.
