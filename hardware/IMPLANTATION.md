# Implantation et routage - révision 0.4

État du 5 novembre 2026 : A et B sont routées. Les positions réelles, rotations, empreintes et réserves sont relues depuis les fichiers natifs vers `placement.json`. Le PDF `review/implantation_v01.pdf` représente l'ancienne étude et n'est pas la référence du routage actuel. Utiliser les vues `review/routing/*_routed.pdf` (dessus puis dessous).

| Carte | Dimensions | Cuivre externe | Empreintes / état |
| --- | --- | --- | --- |
| A | 110 × 75 × 1,6 mm | 35 µm par face | 22 empreintes physiques, dont 2 LED et 2 résistances ajoutées |
| B | 90 × 65 × 1,6 mm | **70 µm par face** | 43 empreintes physiques, dont 4 LED et 4 résistances ajoutées ; XL4015 hors PCB |

Les deux empilages sont présents dans KiCad. Pas de Gerbers libérés avant vérification des modules et essais thermiques.

## Carte A

DevKit diymore CH340 à 30 broches : antenne vers la gauche, USB vers la droite. Centre de l'empreinte locale proposé à (32, 40) mm, origine du PCB au coin haut gauche vu côté composants. Encombrement vendeur de 54 × 27 mm ; espacement entre rangées **25,40 mm supposé**, autorisé pour avancer, à vérifier avant fabrication. Chaque rangée reçoit une barrette femelle 1×15 au pas 2,54 mm. Le premier contact de la rangée inférieure (côté antenne) est EN ; celui de la rangée supérieure est GPIO23 ; le contact d'alimentation 5 V est le numéro 15. Le plan de broches complet est dans `module_selection.json`.

J2 LCD, J4 TMP117 et J5 BH1750 sont en haut ; J3 encodeur en bas ; J1/J6/J7 sur la droite. Les capteurs eux-mêmes sont déportés, pour éviter de mesurer la chaleur de l'électronique. JP1 reste accessible côté USB.

La réserve RF actuelle couvre x=0 à 12,5 mm, y=17 à 63 mm, sans cuivre ni pistes, avec le module orienté antenne à gauche. C'est une réserve de prototype, pas une validation RF fabricant ; confronter la position de l'antenne, les rangées de broches, le LCD métallique et les faisceaux au matériel réel. La réserve a été adaptée à l'empreinte physique, qui remplace l'ancienne enveloppe 60 × 32 mm.

## Carte B

L'entrée 24 V, F1, l'anti-inversion Q2, la TVS et la sortie chauffage sont dans la moitié haute. Q1 est une IRLZ44NPBF en TO-220 verticale, commandée par un driver 5 V U5. C8 découple l'alimentation locale ; D4 reprend l'inductance des fils de charge.

Le trajet principal de courant est maintenu en cuivre large : J2 positif -> F1 -> Q2 -> J3 positif -> charge externe -> J3 retour -> Q1 -> J2 PGND. Le contrôle des formes de cuivre vérifie un trajet continu sans via discret pour chacun de ces segments, avec les éventuels courts rétrécissements listés dans `review/routing/routing_validation.json`. Le cuivre de dissipation de Q2 est sur son drain, `VIN_FUSED`, en face avant. Les pertes et la température restent à mesurer dans le boîtier fermé.

L'ADC et le filtre NTC sont à droite, la régulation et l'autorisation logique en bas/au centre ; les trois JST A/B restent dans la partie basse. Les retours logiques GND et de puissance PGND sont distincts sur B, afin de ne pas contourner le shunt éventuel du XL4015 CC/CV. Le plan de masse arrière se réfère à GND ; le retour chauffage large se réfère à PGND. Le module n'est pas isolé.

Le **XL4015 acheté (51 × 26,3 × 14 mm)** est retiré de la surface du PCB B : U1 est un module hors carte au schéma. J8, bornier 5,08 mm, va à IN+/IN- ; J9, JST-XH 2 positions, reçoit OUT-/OUT+. Prévoir son maintien dans le boîtier et un mou de service pour extraire B. Les cotes de bornes et trous de cette carte de commerce ne sont pas présumées compatibles avec un header enfichable.

## Maintien par clips

Révision diagnostic : A/D1 `5V_AB` et A/D2 `3V3_A` sont à droite, aux centres locaux (99,55) et (99,64) mm. Sur B, D5 `24V` est en (22,37), D6 `5V` en (28,37), D7 `3V3` en (45,34) et D8 `HEAT` en (66,39) mm. Leurs résistances sont à proximité, hors des réserves d'appui. Les LED dépassent de 0,75 mm et les résistances sont réservées à 0,6 mm. La DevKit ne masque pas les voyants de A. Le boîtier fermé peut les masquer : ce sont des voyants de service internes.

B/J4 reçoit les mentions `TEMP / NTC 15k` au-dessus et `1=3V3  2=NTC` au-dessous ; son repère J4 est déplacé sur le côté pour éviter de chevaucher le texte. Le brochage électrique est conservé. La révision ajoute des branches de diagnostic ; les pistes/vias existants, notamment le trajet de puissance, sont conservés.

Les deux PCB conservent les bandes sans composants ni cuivre de **4 mm en haut et en bas**. Les côtés ont des appuis continus de **1,5 mm**, plus les réserves locales de 2 × 3 mm à 8 mm du haut pour les crochets. Ces zones sont effectivement présentes dans KiCad sur les deux couches ; les plans de cuivre les respectent.

Les cartes restent verticales et parallèles à la façade. A et le LCD se montent sur la façade amovible hors coque ; B s'extrait par l'arrière. La [maquette mécanique v0.5](mechanical/README.md) représente la DevKit diymore, les composants hauts de B et un berceau indépendant pour le XL4015, extractible par l'arrière. La coque passe à **155 × 100 × 115 mm** ; l'origine de B dans l'assemblage passe de x=40 à x=50 mm, sans déplacement des composants sur son PCB. Dimensions supposées des modules, clips, câbles et dissipation restent à essayer sur le prototype.

## Lecture et vérification

Les empreintes sans pastilles de `placement_only.pretty` ne sont plus utilisées par les PCB routés. Les bibliothèques standard sont celles de KiCad 10 ; la seule empreinte locale nécessaire est le support diymore dans `lab_hardware.pretty`, référencé par `pcb_a_display/fp-lib-table`.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_placement.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/check_routing.py
```

Les DRC et la parité au schéma sont sans violation ; cela n'atteste ni la correspondance des achats aux photos, ni l'ajustement mécanique, ni la dissipation. Voir `README.md` et `review/routing_preparation.md` pour les hypothèses et essais restants. Les fichiers natifs restent la référence ; ne pas relancer les générateurs initiaux pour une simple revue.
