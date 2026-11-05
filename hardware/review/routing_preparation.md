# Routage livré - prototype 0.3 du 5 novembre 2026

Les deux PCB sont routés, avec leurs empreintes physiques et plans de cuivre. Les contrôles finaux sont dans `routing/routing_validation.json` et les DRC natifs dans `routing/*_drc.json`.

## Hypothèse conservée sur la DevKit

Le matériel retenu est la **diymore CH340 USB-C**, [ASIN B0DGLCWR76](https://www.amazon.fr/dp/B0DGLCWR76), et non l'ancienne hypothèse ELEGOO CP2102. Les vues vendeur donnent 30 contacts, le brochage et un contour annoncé de 54 × 27 mm. La recherche n'a pas fourni de plan mécanique coté du fabricant pour l'entraxe exact de cette variante.

Sur la vue photographique de dessus, l'écartement des rangées est proche de dix fois le pas longitudinal de 2,54 mm, ce qui favorise **25,40 mm** plutôt que 22,86 mm. **C'est une inférence**, pas une mesure physique ou une cote fabricant certifiée. À la demande explicite de l'utilisateur, le routage a été poursuivi avec cette valeur, marquée `ASSUMED` dans le nom d'empreinte, les propriétés et la documentation. Ne pas lancer la fabrication avant comparaison avec la carte achetée. L'entraxe reste paramétré dans `prepare_routing_revision.py`, mais changer l'empreinte après routage impose de reprendre les pistes et les contrôles.

Numérotation choisie : vue de dessus avec USB en bas, 1..15 de haut en bas à gauche, 16..30 de bas en haut à droite. Donc VIN=15, GND=14 et 17, SDA/GPIO21=26, SCL/GPIO22=29 ; le rail 3V3 de la DevKit (16) n'est pas relié au régulateur de A. La rotation place EN en bas à gauche, VIN en bas à droite et GPIO23 en haut à gauche ; un contrôle géométrique vérifie cette orientation pour éviter une inversion des rangées. Tous les contacts sont enregistrés dans `../module_selection.json`.

## Module XL4015

Le [module TECNOULAB acheté](https://www.amazon.fr/dp/B0C6R3FMT3) est conservé pour alimenter la logique en 5 V depuis le 24 V. Il est hors PCB et câblé à J8 (entrée) et J9 (sortie) ; les bornes du module n'ont pas été transformées en pads enfichables fictifs. L'enveloppe annoncée est de 51 × 26,3 × 14 mm. Son support dans la maquette doit encore être repris.

Les retours IN- et OUT- sont raccordés séparément sur B (PGND et GND), afin de préserver un éventuel shunt de limitation CC. Le module est non isolé et sa valeur de shunt/schéma complet reste inconnue. Mesurer 5,00 V avant raccordement et contrôler sous charge le budget, la limitation, le décalage de référence et la chauffe. La fiche commerciale annonce 32 V maximum en entrée ; la tenue aux transitoires du module complet n'est pas qualifiée par la présence d'une TVS sur B.

## Choix de puissance et contrôles

- Cible : 24 V régulé, chauffage résistif 100 W (4,17 A), PWM de mise au point 500 Hz.
- B : 70 µm de cuivre externe par face, empilage enregistré ; A : 35 µm par face.
- F1 : 045106.3MRL, 6,3 A / 125 VDC ; F2 : 0451001.MRL, 1 A / 125 VDC. Déclenchement, appels et défauts à essayer.
- Q2 : DMP6023LE-13, anti-inversion avec zener 12 V ; Q1 : IRLZ44NPBF avec driver UCC27517DBVR à 5 V, après la porte PWM AND ENABLE.
- D4 : STPS5L60U, reprise de l'inductance des fils ; C8 : EEUFR1H101, 100 µF / 50 V, D8/P3,5/H11,5 mm.
- Borniers J2/J3/J8 : Phoenix Contact 1715721, MKDS 1,5/2-5,08. JST périphériques : petits XH de 2 à 6 positions, brochages inchangés entre A/B.
- Le contrôle suit le trajet principal de chauffe dans les formes effectives des pistes/pads : continu sans via discret, largeur principale 2,4 mm ; courts passages de 1,50 mm au fusible/PMOS et 1,80 mm à Q1, détaillés dans le rapport. La branche impulsionnelle D4 utilise un via ; elle ne porte pas le courant continu du chauffage.
- ERC A/B : zéro erreur, zéro avertissement. DRC : zéro violation, zéro connexion restante, zéro écart de parité entre schéma et PCB. Les réseaux des pads sont comparés aux netlists.

Le moteur local Freerouting 2.5.0 a amorcé les pistes ; les connexions de retour, le trajet de puissance et les largeurs ont ensuite été corrigés dans les sources natives. Une erreur du binaire natif de Freerouting a conduit à utiliser son JAR officiel avec un JRE portable ; aucune API de routage distante n'a été employée. Une assertion `PCB_VIA.GetWidth()` du vérificateur Python a aussi été corrigée en demandant explicitement la couche pour les vias.

## Vérifications restantes avant fabrication et usage

Comparer entraxe, brochage, position USB et contour de la DevKit réelle ; vérifier le LCD à 3,3 V, les modules de capteurs et le budget du 5 V. Reprendre la maquette du boîtier pour le buck séparé et les composants hauts. Vérifier le circuit USB/5 V de la DevKit, JP1 ne constituant pas un multiplexeur automatique.

Faire les essais à alimentation limitée : coupure et inversion, appels de courant, réglage du module, commande de grille, surcharge, arrêt sur sonde ouverte/court-circuit et sur mesure périmée, puis chauffe à 100 W avec mesure des températures et oscilloscopie des commutations. La limitation à 100 W n'est pas automatique ; il n'y a toujours pas de coupure thermique indépendante dans cette révision. Les plans sont des prototypes routés, pas une qualification de sécurité ou de tenue aux surtensions.

Références fabricant : [JST-XH](https://www.jst-mfg.com/product/pdf/eng/eXH.pdf), [SN74LVC1G08](https://www.ti.com/lit/ds/symlink/sn74lvc1g08.pdf), [UCC27517](https://www.ti.com/lit/gpn/UCC27517), [DMP6023LE](https://www.diodes.com/datasheet/download/DMP6023LE.pdf), [IRLZ44N](https://www.infineon.com/part/IRLZ44N), [XL4015 circuit](https://www.xlsemi.com/datasheet/XL4015-EN.pdf), [fusibles 451/453](https://www.littelfuse.com/assetdocs/fuse-451-and-453-datasheet?assetguid=533cd5cc-956c-4243-867f-6ab5a62f6ba1), [STPS5L60](https://www.st.com/resource/en/datasheet/stps5l60.pdf), [Panasonic EEUFR1H101](https://industrial.panasonic.com/tw/products/pt/aluminum-cap-lead/models/EEUFR1H101).
