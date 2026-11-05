# Première implantation des PCB et du boîtier

Étude mécanique 0.1 du 4 novembre 2026, associée aux schémas électriques Rev 0.2. Les deux cartes ont maintenant un contour et une implantation KiCad non routée. Les dimensions ci-dessous sont une proposition à confronter aux pièces réelles et à une impression de contrôle.

Le [dossier PDF de revue](review/implantation_v01.pdf) présente le placement de A, celui de B, la façade et une coupe du boîtier. Ouvrir les projets KiCad existants pour éditer les nouveaux fichiers `.kicad_pcb` ; le JSON `placement.json` conserve les positions proposées et les hypothèses d'assemblage.

## Dimensions et disposition

| Élément | Proposition | Statut |
| --- | --- | --- |
| PCB A | 110 × 75 mm, épaisseur 1,6 mm | Contour dessiné dans KiCad |
| PCB B | 90 × 65 mm, épaisseur 1,6 mm | Contour dessiné dans KiCad |
| Boîtier extérieur | 145 mm de large × 100 mm de haut × 115 mm de profondeur | Première [maquette 3D avec STEP et clips](mechanical/README.md), non qualifiée pour fabrication |
| Parois | 3 mm | Hypothèse de départ ; intérieur théorique 139 × 94 × 109 mm hors nervures et clips |
| LCD et backpack | Réserve frontale 98 × 60 mm, profondeur maximale réservée jusqu'à 28 mm depuis la face avant extérieure | À mesurer sur le module réel ; pas une référence figée |
| DevKit | Enveloppe au sol 60 × 32 mm | À mesurer, aucun header physique percé à ce stade |
| Buck | Enveloppe au sol 30 × 22 mm | Convient à une étude de petit module ; un LM2596 plus grand imposerait une révision |
| Hauteur des composants de A / B | Budget de 20 mm vers l'arrière depuis chaque face composants | Inclure les supports de DevKit ; à vérifier sur pièces et dissipateurs |
| Service des JST | Réserver 25 mm dans l'axe pour manipulation, câbles et extraction | Budget proposé, à essayer avec les vrais faisceaux et panneau arrière ouvert |

Les cartes sont **verticales et parallèles à la façade**, composants et JST verticaux dirigés vers l'arrière. A et le LCD ont des supports indépendants, intégrés à une façade amovible équipée hors de la coque : LCD d'abord, puis A. La façade équipée entre et se retire par l'avant après déconnexion des faisceaux et de la fiche USB interne. L'encodeur se monte en façade et se raccorde par câble à A. B se fixe dans un berceau extractible par l'arrière ; sa dépose ne demande pas de déposer A ni le LCD.

Coordonnées du boîtier : x depuis la gauche, y depuis le haut, d depuis la face avant extérieure. Les coordonnées des cartes désignent leur coin haut gauche sur la face composants ; le stratifié de 1,6 mm s'étend vers l'avant.

| Élément | x | y | d |
| --- | --- | --- | --- |
| Coin haut gauche A, face composants | 8 mm | 12 mm | 45 mm |
| Coin haut gauche B, face composants | 40 mm | 18 mm | 85 mm |
| Coin haut gauche de la réserve LCD | 8 mm | 12 mm | Réserve de d = 3 à 28 mm |
| Axe de l'encodeur en façade | 126 mm | 45 mm | Dépend du modèle et du bouton |

Ces positions laissent, dans l'enveloppe proposée, 15,4 mm entre le fond réservé au LCD et le stratifié de A, 18,4 mm entre les composants arrière de A (20 mm) et le stratifié de B, et 7 mm entre les composants arrière de B (20 mm) et la face intérieure du panneau arrière. Les fiches/câbles ont leur propre budget de service de 25 mm ; leur rayon de courbure peut imposer plus de profondeur. Les volumes sont des budgets, pas une détection complète de collisions 3D.

## Placement de A

Vue côté composants, origine du PCB au coin supérieur gauche. DevKit à gauche, antenne orientée vers le bord gauche. Réserve RF initiale x = 0 à 20 mm, y = 17 à 63 mm sans cuivre, vias, pastilles ni autres composants ; cette réserve sera redéfinie selon le module exact et sa position sur la DevKit. Le LCD et son cadre métallique, le câblage et B devront aussi être confrontés aux recommandations RF. La réserve n'est pas une validation d'antenne.

- J1, J6 et J7 sont alignés dans la partie droite, pour guider les trois faisceaux vers le passage latéral/arrière.
- J2 LCD, J4 TMP117 et J5 BH1750 sont en partie haute ; J3 encodeur est en partie basse droite. Les capteurs d'ambiance eux-mêmes restent déportés, à éloigner de la chaleur du régulateur et de B selon la mesure voulue.
- Régulateur et découplages près du groupe d'alimentation ; pull-ups I2C au voisinage du bus.
- Réserver l'accès au cavalier JP1 et aux boutons de la DevKit. La maquette mécanique 0.2 ajoute un accès USB-C sur le côté droit (profondeur 64 mm, hauteur 30 mm), relié à la DevKit par une rallonge de données. Le brochage réel de la DevKit, les encombrements de la fiche et la gestion de VBUS restent à qualifier ; voir le dossier mécanique.

L'implantation ne suppose pas que les trous du LCD correspondent au PCB A. Son berceau imprimé supporte le module indépendamment et ne touche pas les composants du backpack.

## Placement de B

- Partie haute : entrée 12 V J2, fusible F1, protections D1/D2 et sortie chauffage J3. Le futur chemin de courant doit rester court et hors des zones de clips.
- Partie gauche : buck U1 et régulation 3,3 V. Le budget du buck ne doit pas être interprété comme un dissipateur prévu pour une puissance donnée.
- Centre : MOSFET Q1 et commande de grille. Prévoir son choix réel, le cuivre thermique et le volume d'un éventuel dissipateur avant de figer la taille de B.
- Partie droite : ADS1115 U4, filtre et entrée sonde J4. Le routage devra traiter séparément les retours de puissance et la mesure analogique ; le seul placement ne le garantit pas.
- Partie basse : J1, J6, J7. Ils sont accessibles depuis l'arrière après retrait du panneau. Débrancher tous les faisceaux hors tension avant de libérer le berceau.

Les entrées/sorties externes restent sur la face arrière du boîtier, via des ouvertures ou de courts câbles maintenus par le boîtier. Les découpes et le dispositif anti-traction dépendent des borniers et connecteurs encore à choisir. Le panneau arrière doit s'enlever pour donner accès à **toute la section de B**, puis laisser le berceau reculer ; une petite trappe de quelques centimètres ne suffira pas. Prévoir la place libre sur la paillasse derrière le boîtier pour cette extraction.

## Clips et surfaces de maintien

Les deux PCB réservent des **bandes de 4 mm en haut et en bas**, sur toute la largeur. Les fichiers KiCad contiennent des zones d'exclusion sur les deux couches de cuivre pour les pistes, vias, pastilles, remplissages et composants. Ce sont des contacts sur des bords droits, sans trou ni encoche spécifique à ce stade.

| PCB | Contacts proposés, coordonnées locales du coin haut gauche | Taille de chaque zone |
| --- | --- | --- |
| A | Bandes (0, 0) et (0, 71) mm | 110 × 4 mm |
| B | Bandes (0, 0) et (0, 61) mm | 90 × 4 mm |

Mécanisme de la maquette 0.4 : rainure inférieure continue, guides latéraux, appui supérieur ouvert et deux clips libérables sur les côtés. Pour B, ces éléments sont intégrés au berceau amovible qui se retire vers l'arrière. Les languettes se déverrouillent depuis le panneau ouvert, en les écartant vers l'extérieur. Le berceau doit reprendre les efforts de branchement des connecteurs ; le PCB n'est pas fléchi pour le déclipser.

La révision 0.4 ajoute des lèvres d'appui latérales continues sur 1,5 mm des bords de PCB : compléter leurs exclusions KiCad avant routage. La séquence d'assemblage et ses limites sont décrites dans `mechanical/README.md`.

Deux réserves locales de 2 × 3 mm ont été ajoutées aux bords gauche et droit de chaque PCB, à y = 8 mm depuis le haut, pour les contacts des clips latéraux. Elles sont présentes dans les fichiers KiCad et `placement.json` ; les bandes haut/bas de 4 mm restent réservées pour les rainures.

Les cotes des languettes, leur flèche, le matériau, le sens d'impression et les jeux de rainure restent à définir par des coupons imprimés. Mesurer aussi l'épaisseur réelle des PCB avec masque. Les bandes réservées permettent cette mise au point ; elles ne constituent pas une géométrie de clip déjà testée.

## Fichiers et limites de l'étude

Les empreintes déjà renseignées dans les schémas sont reprises avec leurs pastilles et réseaux. Les composants sans empreinte validée ont un rectangle pointillé **TBD**, stocké dans `placement_only.pretty`, **sans pastilles**. Ce sont des volumes de réservation, pas des composants prêts à router : DevKit, buck, MOSFET, protections, certains connecteurs et porte logique générique notamment.

Chaque référence du schéma est présente dans l'étude, mais les blocs logiques provisoires ne sont pas des empreintes de fabrication. L'outil « mettre à jour le PCB depuis le schéma » devra être utilisé après remplacement de ces blocs et vérification des brochages physiques. Ne pas générer un dossier de fabrication à partir de ces études.

Vérification de l'état livré : contours et épaisseur relus dans les fichiers natifs ; bandes mécaniques dégagées ; réseaux des pastilles réelles comparés aux netlists ; DRC sans violation de géométrie, **avec des connexions non réalisées attendues**, puisqu'il n'y a aucune piste. Ce contrôle ne valide ni les volumes 3D réels, ni la dissipation, ni les clips.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_placement.py
```

`create_placement.py` est le générateur initial des études et refuse de les écraser sans `--overwrite`. Après retouches dans KiCad, ne pas le relancer pour une simple vérification. `render_placement.py` produit le PDF depuis le JSON et nécessite reportlab : après retouche native, les positions du JSON et le PDF devront être réconciliés avec les PCB.

Avant de réduire les dimensions ou de router : mesurer DevKit, LCD complet et buck ; arrêter la charge et son étage de puissance ; essayer les connecteurs accouplés ; imprimer une maquette du maintien et vérifier l'accès réel.

## Références des hypothèses

- Le format LCD 98 × 60 mm est un exemple documenté par [Winstar WH2004A](https://www.winstar.com.tw/uploads/files/6e5109c5ba979085d164c6b48a5bb66d.pdf), pas une identification du LCD existant. Son backpack ajoute son propre encombrement.
- La hauteur accouplée annoncée de la famille [JST-XH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf) est 9,8 mm ; le budget de 25 mm inclut une réserve proposée pour les fils et la manipulation, à vérifier.
- Les contraintes de placement de l'antenne seront validées avec les [recommandations Espressif du module exact](https://documentation.espressif.com/esp-hardware-design-guidelines/en/latest/esp32/index.html).
