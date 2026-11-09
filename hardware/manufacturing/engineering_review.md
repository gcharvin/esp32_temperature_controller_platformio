# Décisions techniques pour le premier lot

Cette revue termine les choix techniques locaux pour **A v0.4 et B v0.5**.
Elle prépare un lot de prototypes assemblés ; elle ne transforme ni les
simulations ni le modèle 3D en qualification du matériel réel.

## Références et empreintes figées

`component_selection.json` fixe les MPN des 68 composants physiques natifs.
Ils sont inscrits dans les schémas et PCB, puis comparés aux netlists fraîches.
Les deux barrettes Harwin et le cavalier livré séparément sont des articles
supplémentaires explicites. La DevKit reste fournie par l'utilisateur et NT1
reste une jonction de cuivre. Le fabricant doit sourcer les références exactes,
notamment C10, R13 et Harwin ; une référence sans ID JLC n'est pas une pièce
libre de substitution.

Les empreintes locales remplacent les hypothèses de perçage de la DevKit et
les pastilles de soudure manuelle des LED. Les recommandations R13 sont déjà
intégrées. Les dimensions et sources sont dans `land_pattern_review.json`.
Le perçage des sockets est nominalement 1,05 mm ; le fournisseur confirmera
sa tolérance de trou fini et son procédé de soudure au DFM.

Sources : [Harwin](https://content.harwin.com/asset/53851ba8-b32e-4ac5-a0c5-09d9271c0c0a/DRG-00385-Technical-Drawing-Datasheet-M20-782-pdf.pdf),
[LED verte](https://www.kingbrightusa.com/images/catalog/spec/apt2012lzgck.pdf),
[LED orange](https://www.kingbrightusa.com/images/catalog/spec/apt2012lseck-j4-prv.pdf),
[R13](https://www.vishay.com/docs/30100/wsl.pdf).

## Condensateurs et régulateurs 3,3 V

Choix conservé pour ce prototype : **C1608X7R1C105K080AC**, 1 µF nominal,
16 V, X7R, ±10 %, sur les découplages 1 µF existants. Le circuit typique AP2112
emploie 1 µF en entrée et sortie et recommande X5R/X7R. La sélection suit cette
application nominale. La valeur effective après polarisation, température et
vieillissement n'est pas égale à la capacité nominale et aucune borne de
stabilité garantie n'est déduite d'une courbe typique non garantie. Ce point
est accepté pour le **lot de mise au point**, avec vérification des rails et
des transitoires au banc ; il ne reste pas un choix de MPN à effectuer.
Les empreintes 0603 permettent une révision de valeur après mesures si besoin.
Le 10 µF d'entrée A/C3 reste un découplage supplémentaire, pas la compensation
du convertisseur LM2596. C10/R13 conservent leur réseau de retour simulé.

Sources : [AP2112, circuit typique et note4](https://www.diodes.com/datasheet/download/AP2112.pdf),
[TDK, référence sélectionnée et caractère indicatif des courbes](https://product.tdk.com/en/search/capacitor/ceramic/mlcc/info?part_no=C1608X7R1C105K080AC).

Le courant nominal 600 mA de l'AP2112 n'est pas un budget thermique utilisable
sans calcul sur ce PCB. Pour la mise au point, limiter les périphériques 3,3 V
de A à **100 mA** et ceux de B à **20 mA**, en conservant **0,8 A total** sur
le 5 V partagé, DevKit comprise. Ces plafonds sont des contraintes d'essai,
pas des mesures des modules existants. Relever les courants et températures
réels avant d'augmenter ces budgets.

## Chutes de tension et faisceaux

Les faisceaux A/B proposés mesurent 300 mm. POWER utilise AWG22, les signaux
AWG26. À 0,8 A, une boucle cuivre AWG22 de 0,6 m représente environ 26 mV de
chute à 20°C, avant les contacts ; cet ordre de grandeur ne garantit pas
la résistance réelle d'un câble fini. Le critère doit porter sur **A/J1**.

Un B/J1 à 4,75 V ne peut pas garantir A/J1 à 4,75 V après un faisceau résistif.
Le contrôle du prototype impose donc **4,90–5,15 V sur B/J1**, en continu à
température de contrôle, puis **4,75–5,25 V sur A/J1** avec le faisceau réel.
La première plage est un critère de réception plus serré que l'enveloppe
complète du LM2596 ; elle n'est pas une nouvelle garantie TI. L'intégration
et les essais en température restent nécessaires. Ne pas relever la sortie
du buck par un montage improvisé si un exemplaire échoue à ce contrôle.

## Mécanique et service

La maquette v0.10 réserve les dimensions maximales documentées des sockets,
un espace de montage de 0,15 mm et une entretoise plastique de broches mâles
supposée de 2,54 mm. Cette dernière cote reste une enveloppe de conception,
pas une mesure des trois DevKit. La hauteur montée des borniers Phoenix est
13,8 mm ; l'enveloppe utilise 14 mm. Les accès et les 20 séquences rigides sont
recontrôlés. Le modèle ne valide ni le sertissage, ni les câbles réels, ni les
efforts et tolérances des clips. Les vérifications d'insertion et le coupon
d'impression appartiennent à l'intégration du prototype, pas au contrôle
électrique du PCB nu.

Source : [Phoenix1715721](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721).

Le service USB retenu pour ce lot est **USB seul, sources externes coupées,
J1/J6/J7 débranchés, JP1 retiré**. Le monitoring USB pendant le chauffage reste
une fonction non qualifiée, à tester ensuite avec séparation de VBUS. Cela
ne demande pas de changer le PCB pour commander le lot de mise au point.
Voir `power_and_usb_review.md` pour la procédure et les restrictions.

## Portée des vérifications et suite

Les valeurs électriques du buck, le brochage et les pistes sont conservés.
Les simulations existantes restent pertinentes pour leur périmètre ; les
relancer sans changement de circuit n'apporterait pas une preuve thermique.
Le DRC, les empreintes modifiées, les MPN natifs, les vrais perçages Excellon,
les contours et la pâte Gerber sont vérifiés après cette révision.

Avant lancement, le fabricant doit rendre son DFM/panneau, son accord sur le
profil thermique/MSL et les références, son aperçu de placement et son devis.
Après réception viennent les essais du plan de banc, le firmware adapté et
la protection thermique indépendante correspondant à l'expérience. Le
chauffage 100 W n'est pas un essai initial d'alimentation des cartes.
