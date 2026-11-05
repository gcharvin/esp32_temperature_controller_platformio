# Préparation de fabrication et d'assemblage

État du 5 novembre 2026 : **dossier de revue/devis, pas de commande autorisée
à partir de ces exports**. Références électriques : A v0.4 et B température
v0.5, entrée 24 V, cible chauffage 100 W. A reste alimentée en 5 V régulé pour
les futures cartes B 12 V/5 V également ; voir la
[revue alimentation et USB](power_and_usb_review.md).

## Ce qui est préparé

`review_A04_B05/A/` et `B/` contiennent chacun :

- `procurement_review.csv` : une ligne par repère natif, valeur et empreinte
  lues dans la netlist fraîche, MPN candidat, fabricant, exigences, source,
  méthode d'assemblage et état d'approbation.
- `BOM_SMT_RFQ_NOT_FOR_ORDER.csv` et `CPL_SMT_RFQ_ROTATIONS_UNVERIFIED.csv` :
  mêmes repères pour les composants CMS à faire chiffrer. Les références
  fournisseur sont renseignées lorsque le MPN exact et le fabricant ont été
  vérifiés sur une fiche JLC publique. Les autres restent vides. Aucun prix
  global ou stock de commande n'est supposé.
- `THT_assembly_quote.csv` : connecteurs traversants et autres pièces à faire
  **assembler par le fabricant**, sous réserve de son devis et de ses moyens.
  Leur soudure n'est pas implicitement transférée à l'utilisateur.
- `gerbers_DRAFT_NOT_FOR_ORDER/` : cuivre, masques, pâtes, sérigraphies,
  contour, perçages métallisés/non métallisés et plans de perçage.
- Netlist, ERC et DRC fraîchement produits pour cette revue.

`manifest.json` lie les sources et les exports par SHA256. Le ZIP
`RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip` regroupe ces éléments avec les présentes
instructions, la revue d'alimentation et la liste des points ouverts. Le statut
de libération est explicite dans `release_checklist.json`. Les rapports ERC/DRC
et les concordances sont des contrôles automatiques ; ils ne remplacent pas
la revue électrique, mécanique et du procédé d'assemblage.
`system_bom_review.csv` recense aussi les modules, faisceaux, passe-paroi,
alimentation, charge et pièces imprimées, avec leurs références connues ou
les choix encore ouverts. Le jeu A+B représente 51 CMS et 19 pièces
traversantes à souder (dont les deux sockets), plus la DevKit à enficher.

## Revue fournisseur et procédé

`supplier_review.json` / `.csv` enregistrent **36 correspondances exactes
fabricant + MPN** vérifiées sur les fiches publiques JLCPCB, avec lien et date.
Le contrôle de RC0603FR-0747KL a rencontré une limitation HTTP429 : son ID
repéré C105579 reste à confirmer. C10 EEUFR1V331UB, R13 WSL1206R0560FEA,
les deux sockets et les accessoires Harwin nécessitent une demande de sourcing.
Une recherche sans résultat ne démontre pas une indisponibilité.

Ces fiches ne publient pas dans les données récupérées le stock principal
nécessaire pour une commande. Les prix bruts du catalogue ne constituent pas
un devis avec réglages, pertes, assemblage, transport et taxes. La disponibilité
réelle reste à confirmer par l'assembleur ; les pièces de fabricants différents
portant des noms proches ne sont pas substituées automatiquement.

Pour B, la base de devis est **Standard**, avec profil composant validé :
le LM2596 sélectionné est limité à **245 °C de pic de refusion**. Le profil
Economic annoncé à 255 ± 5 °C dépasse cette limite. Les exigences et leurs
sources sont dans `process_requirements.json` ; elles ne constituent pas une
qualification du profil thermique réel. Source :
[TI, tableau des boîtiers](https://www.ti.com/lit/ds/symlink/lm2596.pdf#page=37).

Comme B mesure 90 × 65 mm, le devis propose des rails amovibles externes de
5 mm en haut et en bas, soit 90 × 75 mm au total pour l'assemblage. Voir
`panel_B_proposal.svg`. Le panneau, ses attaches et son outillage doivent être
acceptés par le fabricant ; le contour fonctionnel et le boîtier restent
inchangés. Les cartes sont demandées **dépanélisées et ébavurées** pour les clips.

`manufacturing_fit_audit.json` contrôle sur les cartes natives les largeurs
de pistes, les perçages de vias et les couronnes des pastilles traversantes.
Les critères contrôlés passent sur A et B. Ce contrôle limité ne vaut ni
revue complète des ouvertures de masque ni validation DFM par le fabricant.
La tolérance d'épaisseur publiée conduit à 1,44–1,76 mm pour un PCB nominal
de 1,6 mm : tester un coupon de rainure/clips à ces limites. Sources :
[fabrication JLC](https://jlcpcb.com/capabilities/pcb-capabilities),
[assemblage JLC](https://jlcpcb.com/capabilities/pcb-assembly-capabilities).

`assembly_RFQ_draft.md` est une demande de devis concrète **rédigée localement,
pas envoyée**. `usb_service_test.md` décrit l'essai électrique sur les cartes
réelles et le candidat PortaPow PPUSBBP1. Les sauvegardes de firmware et les
logs bruts restent hors du dépôt, car ils peuvent contenir des secrets.

Les CSV `*_bom_draft.csv` à côté des projets KiCad restent les anciens exports
de conception. La sélection d'achat courante est ici, **sans injection aveugle
de MPN non approuvés dans les schémas**. Au gel, reporter les références
approuvées dans KiCad, puis régénérer tous les livrables liés. Ne pas importer
ce dossier provisoire dans un panier pour passer commande.

## Choix d'assemblage

A cible les **trois DevKit USB-C au brochage commun confirmé par l'utilisateur**.
Le fabricant assemble les mêmes deux sockets ; la DevKit reste fournie et
enfichée par l'utilisateur. Le MPN diymore historique ne restreint pas le
support à cette marque. Le contrat est dans `../devkit_compatibility.md` ;
les enveloppes et circuits VIN/VBUS ne sont pas tous qualifiés par le seul
brochage commun. Les essais série des variantes sont consignés dans
`../review/hardware_usb_checks_20261105.json`.

Le périmètre demandé au fabricant est : **tous les CMS, puis les traversants
et les deux barrettes femelles de la DevKit**. Demander deux lignes de devis
si son service distingue CMS et soudure manuelle. Une solution d'assemblage
alternative ne sera retenue qu'après comparaison des devis.

- A/U1 est la DevKit déjà achetée. Elle sera **enfichée après assemblage**.
  Son empreinte de 30 pastilles représente en réalité **deux barrettes 1×15**,
  pas un composant unique à placer au centroïde U1. Le fichier
  `A/U1_socket_assembly_quote.csv` donne deux articles candidats
  Harwin **M20-7821546**, hauteur 8,5 mm et pas 2,54 mm, avec leurs groupes de
  pastilles. **L'entraxe de 25,40 mm a été confirmé par mesure de l'utilisateur
  le 5 novembre 2026.** Perçages, insertion et hauteur montée/USB restent à vérifier ; ce
  choix n'est pas encore intégré comme nouvelle géométrie dans le STEP.
  Le nom d'empreinte `_ASSUMED` et les anciennes notes natives sont conservés
  pour éviter une migration sans changement géométrique ; le statut de cette
  cote dans le dossier courant prévaut sur ces annotations historiques.
- B/NT1 est du cuivre, **zéro composant à acheter ou placer**. Il est exclu
  explicitement de la BOM d'assemblage et du CPL.
- JP1 reçoit une barrette candidate **Harwin M20-9990246**, avec cavalier
  séparé **M7566-05** à mettre en place après assemblage. Vérifier sa hauteur
  et son accès dans la pile réelle ; ce choix ne constitue pas une sélection
  automatique des alimentations USB/externe.
- Les JST-XH soudés sur les PCB ne comprennent ni les boîtiers XHP des câbles,
  ni leurs contacts sertis. Chiffrer des faisceaux terminés : A/B 2, 3 et
  6 contacts droits, puis sonde 2 contacts et périphériques A 4 contacts selon
  le câblage réel des modules. Les longueurs devront être validées dans le
  boîtier ; ne pas commander des faisceaux au brochage « standard » supposé.
- La DevKit, le LCD/backpack, encodeur, TMP117/BH1750, sonde NTC, alimentation,
  chauffage, rallonge USB, prise sonde de panneau, presse-étoupes et protection
  thermique indépendante relèvent de la BOM système. Plusieurs sont possédés
  ou dépendants de l'expérience : ils ne sont pas inclus comme composants CMS
  de ces cartes. Les références de passe-paroi ne sont pas encore figées.

Les deux nouveaux candidats du buck sont **C9 : C0805C104K5RACTU** (100 nF,
50 V, X7R, 0805) et **R13 : WSL1206R0560FEA** (56 mΩ, 1 %, 0,25 W, 1206).
Leurs caractéristiques correspondent aux exigences nominales. Vérifier les
plages de pastilles recommandées, le profil de soudure et l'approvisionnement
avant approbation. R13 reste dans la branche de C10, jamais en série avec
la charge. La capacité effective sous polarisation des petits condensateurs
CMS, notamment autour des AP2112, reste à confronter aux minima du circuit.

Sources : [C9 KEMET/Yageo](https://yageogroup.com/download/specsheet/C0805C104K5RACTU),
[R13 Vishay](https://www.vishay.com/docs/30100/wsl.pdf),
[barrette Harwin](https://www.harwin.com/products/M20-7821546),
[JP1](https://www.harwin.com/products/M20-9990246),
[cavalier](https://www.harwin.com/products/M7566-05).

## Paramètres à reporter dans le devis

| Paramètre | A | B température |
| --- | --- | --- |
| Contour nominal | 110 × 75 mm | 90 × 65 mm |
| Épaisseur | 1,6 mm | 1,6 mm |
| Couches cuivre | 2 | 2 |
| Cuivre extérieur | 35 µm / 1 oz | **70 µm / 2 oz, deux faces** |
| Composants CMS | Dessus | Dessus |
| Assemblage complémentaire | JST, JP1, deux sockets | JST, borniers, C8/C10 et Q1 |
| Proposition de lot | 5 pièces pour devis | 5 pièces pour devis |

FR-4 sans plomb et finition HASL sans plomb sont des hypothèses de devis,
à confirmer avec le fabricant pour ces empreintes ; aucun empilage ni
procédé spécifique n'est présumé disponible au prix de base. Faire vérifier
les tolérances de contour/épaisseur pour les clips et ne pas ajouter de
chanfrein, rails de panélisation ou trous dans leurs réserves sans revue.

Les coordonnées du CPL sont exportées avec KiCad en millimètres, origine
absolue identique à celle des Gerber/perçages, rotation native. **Les rotations
ne sont pas corrigées ni approuvées pour une bibliothèque d'assembleur.**
Vérifier dans son aperçu la broche 1, cathodes, condensateurs polarisés,
MOSFET et languette du LM2596, ainsi que l'orientation des connecteurs.
Le CPL CMS ne peut pas servir à placer automatiquement les traversants ni
les deux sockets virtuellement regroupés dans U1.
`positions_native.csv` est l'intermédiaire brut de KiCad, qui peut contenir
NT1 comme objet SMD ; il ne doit pas être utilisé comme CPL d'assemblage.

Les noms de colonnes BOM/CPL reprennent la documentation
[JLCPCB BOM/CPL](https://jlcpcb.com/help/article/advice-for-bom-and-cpl-files-preparation).
La correspondance exacte repère/MPN/fournisseur, les éventuels articles fournis
par le client et les options d'assemblage sont à confirmer au devis ; aucun
numéro LCSC n'est inventé. Ces fichiers peuvent aussi servir de base à un
autre fabricant, avec adaptation de son format.

## Régénération

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/prepare_manufacturing_review.py
```

Ce script lit les cartes, vérifie la parité des repères, valeurs, empreintes
et nets des pastilles sur une netlist fraîche, exécute ERC/DRC et exporte le
dossier. Il ne modifie aucun schéma, routage ni modèle mécanique. Après une
modification native, la revue humaine et les artefacts de simulation/mécanique
liés peuvent exiger leur propre mise à jour ; le script ne les libère pas.
