# Préparation de fabrication A v0.4 / B v0.5

Révision courante **A0.4.1 / B0.5.1**, ajout de **22 pastilles de test au dos**,
sans changement des68 composants natifs achetés, des51 CMS ou des19 traversants
physiques à monter. Les anciens noms A04/B05 désignent la famille de fichiers.
Les TP ne sont ni à approvisionner ni à souder. Voir [le plan de mesure](../TEST_POINTS.md)
et [le firmware de diagnostic séparé](../../firmware/diagnostic/README.md).
Le paquet JLCPCB courant est dans `jlcpcb/` et son archive dédiée.

Le point d’entrée est [START_HERE.md](START_HERE.md). Le dossier prépare un
**lot de cinq A et cinq B assemblées**, B température24V/100W, A commune aux
futures variantes B12V/5V via son interface5V régulée. Il ne constitue pas
une commande. `preparation_status.json` consigne la fin des travaux locaux
après vérifications automatiques et revue visuelle finale ; les accords du
fabricant restent distincts.

## Sélection et sources

[component_selection.json](component_selection.json) fixe les68 composants
physiques natifs, avec fabricant/MPN exacts, empreintes et sources. Les champs
sont inscrits dans les schémas et PCB, et vérifiés sur les netlists fraîches.
Deux sockets HarwinM20-7821546 et un cavalier M7566-05 livré séparément sont
spécifiés en plus. A/U1 représente une DevKit fournie par l’utilisateur,
pas un module à souder ; B/NT1 est du cuivre sans pièce à acheter.

Les37 identités distinctes du catalogue JLC vérifiées couvrent50 des51 CMS.
R13, C10 et les articles Harwin exigent un sourcing exact au devis ; absence
d’ID catalogue ne signifie pas indisponibilité. Aucun stock réservé ou prix
final n’est déduit des pages publiques. Aucune substitution automatique.

La [revue technique](engineering_review.md) documente les choix du prototype,
les capacités1µF conformes à l’application nominale AP2112, les budgets de
courant, la marge de chute dans les câbles et le périmètre USB. Les essais
physiques suivent la réception, sans être présentés comme déjà effectués.
[land_pattern_review.json](land_pattern_review.json) détaille les recommandations
Vishay, Kingbright, Harwin et Phoenix vérifiées et les corrections natives.

## Fichiers pour l’assembleur

`review_A04_B05/A/` et `B/` contiennent chacun :

- `procurement_review.csv` : tous les repères natifs, achat/assemblage et traçabilité.
- `BOM_SMT_RFQ_NOT_FOR_ORDER.csv` et `CPL_SMT_RFQ_ROTATIONS_UNVERIFIED.csv` :
  CMS seulement, mêmes repères, coordonnées/angles natifs à approuver dans son aperçu.
- `THT_assembly_quote.csv` : toutes les pièces traversantes à faire souder en usine.
- `A/U1_socket_assembly_quote.csv` : les deux barrettes distinctes ;
  `loose_accessories.csv` : cavalier JP1 à fournir hors PCB.
- `gerbers_DRAFT_NOT_FOR_ORDER/` : neuf couches, perçages PTH/NPTH et plans.
- `bare_board_test.d356` : netlist de contrôle électrique du PCB nu ;
  `bare_board_test_raw_kicad.d356` : export natif conservé.
- Netlist de schéma, ERC et DRC de la révision exportée.

Le nom `A/U1_socket...` désigne le fichier placé uniquement dans A.
Pour B, PGND/GND sont fusionnés dans la **seule netlist de contrôle du PCB nu**,
car NT1 les relie physiquement ; noms des autres nets, enregistrements et
coordonnées restent identiques. Les pistes et les schémas ne sont pas modifiés.
L’IPC conserve son unité native0,0001pouce ; les Gerber/perçages/CPL sont métriques.
Ne pas appliquer un décalage d’origine manuel.

`assembly_review/` contient le PDF trois pages, la carte des pastilles/nets,
les19 placements traversants physiques et les contrôles liés aux sources.
Les deux barrettes ne sont pas à placer au centroïde de la DevKit. Les
coordonnées locales partent du coin supérieur gauche vu côté composants ;
les coordonnées natives sont également fournies. `positions_native.csv`
reste un intermédiaire brut, pas le CPL à utiliser pour assembler NT1 ou U1.

`gerber_review/` contient quatre rendus des **vrais Gerber**, interprétés
indépendamment avec PyGerber2.4.3. Les vues dessous ne sont pas miroir et sont
étiquetées comme telles. Ces rendus montrent cuivre/sérigraphie/contour,
sans simuler les trous ou le masque. `fabrication_output_validation.json`
compare tous les perçages et les positions des ouvertures de pâte au PCB natif.

## Procédé demandé

[fabrication_specification.json](fabrication_specification.json) est la fiche
de fabrication : FR4 compatible sans plomb, Tg≥150°C demandé,1,6mm, ENIG,
masque vert, sérigraphie blanche ; A35µm et B70µm sur les deux faces.
Les51 CMS et les19 traversants sont à assembler en usine ; cavalier séparé.
Les modules/périphériques externes restent dans la BOM système d’intégration.

Pour B : devis Standard avec profil réel **pic LM2596≤245°C**, manutention MSL
selon lots, y compris les LED MSL3. Le profil Economic annoncé255±5°C ne
convient pas au LM2596 retenu. Proposer le panneau hors contour fonctionnel,
par exemple rails5mm →90×75mm ; transmettre sa conception/attaches/outillage
pour accord. Livrer les contours110×75 et90×65mm **dépanélisés, ébavurés**, sans
résidus dans les réserves de clips. Les trous de sockets sont nominaux1,05mm.

Sources : [TI](https://www.ti.com/lit/ds/symlink/lm2596.pdf#page=37),
[JLC fabrication](https://jlcpcb.com/capabilities/pcb-capabilities),
[JLC assemblage](https://jlcpcb.com/capabilities/pcb-assembly-capabilities).
Le contrôle géométrique limité est dans `manufacturing_fit_audit.json` ; il
ne remplace pas le DFM réel du fournisseur ni son aperçu de placement.

## Câblage, contrôle et réception

[wiring_and_harnesses.md](wiring_and_harnesses.md) et `harness_pin_map.csv`
fixent les trois câbles A/B300mm, POWER AWG22 et signaux AWG26, JST-XH2,50mm,
ainsi que le branchement NTC sur B/J4. Le [plan de contrôle usine](factory_acceptance_test.md)
décrit les inspections et essais à chiffrer, sans ESP32 ni chauffage.
La [BOM système](system_bom_review.csv) distingue les modules existants,
faisceaux, passe-paroi, alimentation et protection thermique d’application.

Le USB du prototype s’utilise avec toutes sources externes coupées,
J1/J6/J7 débranchés et JP1 retiré. Le monitoring USB alimenté reste à qualifier
sur chaque DevKit, comme le décrit [la revue USB](power_and_usb_review.md).
L’accès latéral est conservé dans la maquette v0.9 ; la géométrie réelle des
modules, les câbles et les clips imprimés se vérifient à l’intégration.

## Achats à notre charge et liens

Liste regroupée le **9 novembre 2026** selon le choix de l’utilisateur :
**Amazon.fr + GoTronic**, avec un seul compte GoTronic accepté. La
[BOM système complète](system_bom_review.csv) comporte 25 lignes et indique
pour chaque article le fournisseur préféré, le statut d’approvisionnement et
ce qui reste à faire sur les faisceaux. Les quantités sont **par appareil** ;
commander cinq paires de PCB n’oblige pas à acheter cinq jeux d’accessoires.
La [politique d’approvisionnement](customer_supply_policy.json) fixe ce périmètre.

**Déjà disponibles / fournis par l’usine :** réutiliser la DevKit
[diymore B0DGLCWR76](https://www.amazon.fr/dp/B0DGLCWR76) et le LCD
[Freenove B0B76VGXYH](https://www.amazon.fr/dp/B0B76VGXYH).
Le fabricant PCBA fournit les composants montés sur A et B, dont les deux
sockets ESP32 ; il livre le cavalier JP1 séparément. Ne pas acheter ces pièces
en double. Le XL4015 déjà acheté reste à quantité **zéro**, B intégrant le LM2596.

### Panier GoTronic — références identifiées

| Article | Quantité par appareil | Référence et lien direct | État / adaptation restante |
| --- | --- | --- | --- |
| Encodeur avec poussoir et capuchon | 1 si absent | [ADA377, code38383](https://www.gotronic.fr/art-encodeur-24-impulsions-ada377-37481.htm), PEC11-4215F-S24 | Même candidat Adafruit377. Composant nu à souder ; faisceau A/B/SW/GND encore à réaliser ou trouver. Vérifier axe et fixation de façade |
| TMP117 de température ambiante | 1 **optionnel** | [ADA4821, code37274](https://www.gotronic.fr/art-capteur-de-t-haute-precision-tmp117-ada4821-33489.htm) | Même module Adafruit4821, 3,3V et adresse0x48 ; raccordement vers A à compléter |
| Cordon du TMP117 | 1 seulement si TMP117 | [QW-4C, code49152](https://www.gotronic.fr/art-cordon-qwiic-stemma-qw-4c-30786.htm) | Fini : SH1mm quatre contacts vers quatre broches mâles, 15cm. Remplace la piste d’achat direct Adafruit4209 ; **le XH4 côté A manque encore** |
| Cordon interne de sonde | 1 **sous réserve de longueur** | [JST1164, code49197](https://www.gotronic.fr/art-cordon-jst-xh-femelle-2-cts-15-cm-jst1164-33095.htm) | XH2 femelle2,50mm vers fils dénudés, **15cm**, section non publiée. Deux soudures au jack ; vérifier le mou de démontage avant achat. Ce n’est pas le précédent30cm ni un faisceau complet |

Le [catalogue XH GoTronic](https://www.gotronic.fr/cat-jst-1509.htm) comporte
des boîtiers et contacts nus : ils ne constituent pas une solution sans pince.
La rallonge XH2 mâle-femelle50cm ne convient pas aux deux embases PCB A/B.

### Panier Amazon.fr — conserver les modèles identifiés

| Article | Quantité par appareil | Lien | État / adaptation restante |
| --- | --- | --- | --- |
| BH1750 GY-302 | 1 pour la luminosité, si absent | [AZ-Delivery B07TKWNGZ4](https://www.amazon.fr/dp/B07TKWNGZ4) | ASIN identifié par la [documentation AZ-Delivery](https://m.media-amazon.com/images/I/81N6D6v-dYL.pdf), offre non relue. Choisir barrette déjà soudée si possible, mesurer le module et adapter la cassette au réel |
| Prise de sonde, embase + fiche TRS3,5mm | 1 paire, selon contenu du lot | [DAOKI B08XM6DR96](https://www.amazon.fr/dp/B08XM6DR96) | Candidat PJ-392 ; équivalence dimensionnelle au [dessin Tayda](https://www.taydaelectronics.com/datasheets/files/PJ-392.pdf) non prouvée. Essayer le coupon v0.10. Deux soudures à l’embase et deux à la fiche ; ring isolé |
| Presse-étoupes, joints et contre-écrous | 2 | [HB-Digital M12 B0CLLQCCWW](https://www.amazon.fr/dp/B0CLLQCCWW) | Gaine3–6,5mm, trou12,5mm ; vérifier contenu du lot et cotes réelles du corps, faute de dessin HB-Digital |
| Filament si absent | Selon impression | [Recherche PETG naturel translucide](https://www.amazon.fr/s?k=PETG+naturel+translucide+filament+1.75mm) | Six pièces v0.10 : insert optique translucide, cassette et autres pièces opaques. Matériau et clips à qualifier |

Amazon.fr est une plateforme : conserver marque, référence, variante exacte,
vendeur et origine d’expédition pour les réachats. Les pages des offres Amazon
ci-dessus n’ont pas pu être relues directement ; prix, stock, expédition France
et équivalence restent à confirmer. Un lien de recherche ne vaut pas référence
qualifiée. Aucun total de panier livré n’est annoncé.

Le BH1750 [GoTronic ADA4681](https://www.gotronic.fr/art-capteur-de-lumiere-bh1750-ada4681-33469.htm)
fait25,3×17,7×4,5mm : ce n’est pas le GY-302 de l’enveloppe actuelle23×16mm.
**Ne pas le substituer pour regrouper les achats** sans revoir cassette,
fixation et câblage. Aucune substitution n’est faite dans cette révision.

### Faisceaux complets sans sertissage encore à trouver

Le [tableau détaillé des liaisons et de leurs deux extrémités](wiring_and_harnesses.md#liste-explicite-des-câbles-prêts-à-brancher--revue-du-9-novembre-2026)
reste le document de câblage. Dans le périmètre Amazon.fr/GoTronic, les besoins
suivants n’ont **pas encore de référence complète qualifiée** :

| Faisceau | Quantité | Spécification à conserver |
| --- | --- | --- |
| A/B POWER | 1 | XHP-2 femelle ↔ femelle, 300±10mm, AWG22, droit1→1/2→2 |
| A/B I²C | 1 | XHP-3 femelle ↔ femelle, 300±10mm, AWG26, droit1→1 à3→3 |
| A/B CTRL | 1 | XHP-6 femelle ↔ femelle, 300±10mm, AWG26, droit1→1 à6→6 ; positions5/6 NC sur B |
| LCD et BH1750 | 2 | XHP-4 femelle ↔ quatre Dupont femelles, barrette mâle déjà soudée sur les modules |
| TMP117, si monté | 1 | XHP-4 femelle ↔ quatre Dupont femelles pour joindre le QW-4C GoTronic déjà identifié |
| Encodeur | 1 | XHP-4 femelle ↔ A/B/SW/GND, avec commun du poussoir adapté ; soudures probables sur ADA377 |
| Sonde interne et externe | 1 de chaque | Interne XHP-2 ↔ embase PJ-392 : cordon GoTronic partiel, deux soudures restantes ; externe NTC15k/B≈3920 ↔ fiche TRS montée introuvable qualifiée |
| Alimentation et chauffage | 2 | Cuivre souple0,75mm² avec embouts **déjà sertis** côté Phoenix, gaine et autre terminaison adaptées aux appareils |
| Protection thermique | Selon montage | Terminaisons à figer avec la protection et son seuil, puis cordon équipé |
| USB latéral | 1 | USB-C femelle panneau ↔ USB-C mâle DevKit, données, plan compatible avec ouverture16mm ; aucun modèle retenu |

Les **JST-XH sont au pas2,50mm**, distinct des Dupont2,54mm. Ne pas acheter
un lot de boîtiers et de contacts non sertis en pensant obtenir des faisceaux.
Des contacts déjà sertis aux deux bouts puis insérés à la main seraient une
solution sans pince, mais aucun kit qualifié couvrant2/3/6 voies dans ce panier
n’est actuellement retenu. Sans sertissage ne signifie pas sans soudure.

### Autres achats différés dans le même périmètre fournisseurs

**Cordon de sonde robuste :** exiger deux conducteurs multibrins sous une
gaine extérieure commune continue, souple, avec maintien anti-arrachement
côté sonde et serre-câble de fiche adapté au diamètre extérieur. Gaine et
raccord sonde doivent supporter la température réelle de l’expérience,
encore à préciser. Préférer l’ensemble sonde/câble gainé/fiche monté en usine.
Deux fils isolés seuls ne répondent pas à cette exigence. Le JST1164 interne
reste une solution partielle : ajouter une gaine isolante de protection
retenue mécaniquement (une longueur par appareil, référence à préciser),
isoler les soudures et vérifier le mou de démontage. Voir
[les exigences de câble de sonde](wiring_and_harnesses.md#câble-de-sonde-gainé-et-robuste--exigence-dachat).

La sonde exige **15kΩ à25°C et une courbe documentée**, cible B≈3920 :
[recherche Amazon](https://www.amazon.fr/s?k=sonde+NTC+15K+temperature+etanche),
aucun article exact qualifié. Alimentation et chauffage restent à choisir
après résolution du nominal12/24V et de la puissance réelle. L’étude native B
reste24V/100W ; elle ne qualifie pas100W sous12V. Prévoir aussi une coupure
thermique indépendante avec seuil et pouvoir de coupure DC adaptés ; une
valeur AC seule ne suffit pas. Ces lignes restent à définir, sans fournisseur
supplémentaire ajouté.

Le PortaPow PPUSBBP1 reste une **option de banc**, non nécessaire au diagnostic
Wi-Fi initial : [fonction fabricant](https://portablepowersupplies.co.uk/product/usb-power-blocker),
[recherche Amazon.fr](https://www.amazon.fr/s?k=PortaPow+PPUSBBP1+USB+power+blocker).
Aucune offre .fr qualifiée ni compatibilité avec chaque DevKit n’est établie.

Les anciennes pistes Makers Electronics, eBay, TinyTronics, Reichelt, Tayda
et Adafruit direct ne sont plus des instructions d’achat. Les dessins et
fiches fabricants restent des sources techniques. Aucun troisième fournisseur
régulier n’est proposé. Les PCB, le brochage et le STEP v0.10 restent inchangés.

## Régénération complète

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/prepare_fabrication.py --pdf-python 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
```

KiCad10, uv/CadQuery et l’interpréteur PDF avec ReportLab/pypdf sont nécessaires.
Les15 étapes s’arrêtent au premier échec. Les sources mécaniques sont figées
pendant la génération. Les dessins et rapports périmés sont refusés avant
constitution du ZIP de devis. La sélection n’est appliquée aux sources que
par l’outil explicite `finalize_prototype_parts.py --apply` ; ne pas relancer
les anciens générateurs de conception ou routage sur ces cartes natives.

Après revue des PDF et images, le manifeste de revue visuelle puis
`build_fabrication_handoff.py` produisent le dossier final local et son statut.
`verify_manufacturing_review.py` contrôle aussi séparément le ZIP RFQ, ses
SHA256, les ensembles BOM/CPL et les sources actuelles. Les accords usine
à recevoir sont réunis dans `factory_response.csv` ; une préparation locale
complète ne vaut ni acceptation fournisseur, ni autorisation d’achat.
