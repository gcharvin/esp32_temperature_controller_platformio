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

Liens revus le **7 novembre 2026**. La [BOM système](system_bom_review.csv)
contient maintenant les colonnes d'action d'achat, fiche fournisseur, lien
Amazon, statut du lien et points à vérifier. Ses quantités sont **par appareil** :
le devis de cinq paires de PCB n'impose pas d'acheter immédiatement cinq jeux
de tous les accessoires.

Les fiches ci-dessous sont des **candidats d'approvisionnement** lorsqu'aucune
référence n'est encore arrêtée. Les liens « Recherche Amazon » sont des recherches,
pas des articles dont la compatibilité serait validée. Les offres, variantes,
prix et stocks restent à vérifier au moment de l'achat. Les pages Amazon de
la DevKit et du LCD n'ont pas pu être relues ; leurs liens sont ceux fournis
par l'utilisateur. Aucun composant du PCB ni dimension du boîtier n'est changé
par cette liste.

| Article à fournir | Quantité | Fiche ou référence | Amazon | Point à vérifier |
| --- | --- | --- | --- | --- |
| DevKit ESP32 USB-C | 1, déjà acheté | Diymore B0DGLCWR76 | [Lien d'achat existant](https://www.amazon.fr/dp/B0DGLCWR76) | 30 contacts, entraxe 25,40 mm, brochage et corps réels |
| LCD 20×4 avec backpack I²C | 1, déjà acheté | Freenove B0B76VGXYH | [Lien d'achat existant](https://www.amazon.fr/dp/B0B76VGXYH) | Alimentation prévue 3,3 V, contraste et cotes |
| Encodeur avec poussoir et bouton de façade | 1 si absent | [Adafruit 377](https://www.adafruit.com/product/377) | [Recherche Amazon](https://www.amazon.fr/s?k=encodeur+rotatif+EC11+bouton+poussoir+axe+6mm) | Encodeur passif, câblage A/B/SW/GND ; filetage et axe. Adafruit indiquait une rupture à la revue |
| TMP117, température ambiante | 1 optionnel | [Adafruit 4821](https://www.adafruit.com/product/4821) | [Recherche Amazon](https://www.amazon.fr/s?k=Adafruit+TMP117+4821) | Faisceau XH côté A ; pastilles ou JST-SH STEMMA QT côté module, à adapter |
| BH1750, luminosité | 1 pour cette fonction | [GY-302 chez Reichelt](https://www.reichelt.com/fr/fr/shop/produit/cartes_de_developpement_-_capteur_de_lumiere_numerique_bh1750-224217) | [AZ-Delivery B07TKWNGZ4](https://www.amazon.fr/dp/B07TKWNGZ4) | ASIN issu de la documentation AZ-Delivery, offre Amazon.fr non relue. Mesurer le PCB pour adapter la cassette |
| Sonde NTC passive | 1 | Référence encore ouverte | [Recherche Amazon](https://www.amazon.fr/s?k=sonde+NTC+15K+temperature+etanche) | **15 kΩ à 25 °C et courbe documentée**, cible B≈3920. Aucune sonde exacte qualifiée |
| Faisceau A/B POWER | 1 | [XHP-2](https://www.digikey.com/en/products/detail/jst-sales-america-inc/XHP-2/555485), deux boîtiers ; [fil JST préserti 305 mm AWG22](https://www.digikey.com/en/products/detail/jst-sales-america-inc/ASXHSXH22K305/6684932), deux conducteurs | [Recherche Amazon](https://www.amazon.fr/s?k=JST+XH+2+broches+300mm+22AWG) | Ces articles séparés ne sont pas un faisceau fini ; continuité 1→1 et 2→2 |
| Faisceau A/B I²C | 1 | [XHP-3](https://www.digikey.com/en/products/detail/jst-sales-america-inc/XHP-3/1651017), deux boîtiers + trois fils sertis | [Recherche Amazon](https://www.amazon.fr/s?k=JST+XH+3+broches+300mm+26AWG+femelle+femelle) | 300±10 mm, AWG26, GND/SDA/SCL droits |
| Faisceau A/B CTRL | 1 | [XHP-6 et contacts SXH : documentation JST](https://www.jst.com/wp-content/uploads/2021/01/eXH.pdf) | [Recherche Amazon](https://www.amazon.fr/s?k=JST+XH+6+broches+300mm+26AWG+femelle+femelle) | Deux boîtiers, six conducteurs AWG26 ; broches 5/6 libres sur B température |
| Sonde interne, embase arrière vers B/J4 | 1 | [XHP-2 côté B](https://www.digikey.com/en/products/detail/jst-sales-america-inc/XHP-2/555485) | [Recherche Amazon](https://www.amazon.fr/s?k=JST+XH+2+broches+cable+pre+serti+300mm) | Une extrémité XH, l'autre à souder à l'embase ; mou de démontage |
| Faisceaux LCD, encodeur, TMP117, BH1750 | Jusqu'à 4 | [XHP-4 et contacts SXH : documentation JST](https://www.jst.com/wp-content/uploads/2021/01/eXH.pdf) | [Recherche Amazon](https://www.amazon.fr/s?k=JST+XH+4+broches+cable+pre+serti+Dupont+femelle) | Adapter l'ordre des fils aux modules ; ne pas se fier aux couleurs |
| Prise de sonde arrière | 1 paire embase + fiche | [Famille GX12-2P](https://connector-sensor.com/product/gx12-2p/) | [Recherche Amazon](https://www.amazon.fr/s?k=GX12-2+connecteur+panneau+male+femelle+2+broches+paire) | Comparer les plans : trou prévu 12,4 mm, corps 17 mm ; acheter les deux parties compatibles |
| Presse-étoupes et écrous/joints | 2 | [LAPP 53111000 M12×1,5](https://www.digikey.com/en/products/detail/lapp/53111000/18878806) | [Recherche Amazon](https://www.amazon.fr/s?k=presse+etoupe+M12x1.5+3.5+7mm+ecrou) | Gaine 3,5–7 mm ; longueur et dépassement à comparer aux réserves du panneau |
| Alimentation externe 24 V | 1 si absente | [MEAN WELL GST160A24-R7B, 160 W](https://www.digikey.com/en/products/detail/mean-well-usa-inc/GST160A24-R7B/7703602) | [Recherche Amazon](https://www.amazon.fr/s?k=Mean+Well+GST160A24-R7B+24V) | Ajouter cordon IEC C13 et câble/adaptateur Power-DIN 4 contacts correctement dimensionné et câblé ; aucun branchement direct à B/J2 |
| Chauffage résistif | 1 selon l'expérience | Référence encore ouverte | [Recherche Amazon](https://www.amazon.fr/s?k=element+chauffant+silicone+24V+100W) | 24 V, 100 W maximum ; géométrie, support et température d'usage à définir |
| Câbles de puissance et embouts | Selon montage | Cuivre souple 0,75 mm², isolation adaptée | [Recherche Amazon](https://www.amazon.fr/s?k=cable+silicone+cuivre+2x0.75+mm2+embouts+0.75mm2) | Gaine compatible avec les presse-étoupes et embouts compatibles avec les borniers |
| Rallonge USB-C de panneau | 1 | Référence encore ouverte | [Recherche Amazon](https://www.amazon.fr/s?k=rallonge+USB+C+panneau+ronde+femelle+male+donnees+30cm) | Données USB requises ; trou prévu 16 mm. Aucun modèle actuellement qualifié pour cette ouverture |
| Accessoire USB de service alimenté | 1 option de banc | [PortaPow PPUSBBP1](https://portablepowersupplies.co.uk/product/usb-power-blocker) | [Amazon EU publié par PortaPow](https://www.amazon.de/dp/B094FYL9QT) | **Power blocker**, +5 V coupé et données conservées ; compatibilité DevKit encore à tester. Le diagnostic Wi-Fi peut s'en passer |
| Coupure thermique indépendante | 1 avant essais du chauffage | Seuil et référence à définir | [Recherche Amazon](https://www.amazon.fr/s?k=thermostat+securite+normalement+ferme+rearmement+manuel+courant+continu) | Pouvoir de coupure **DC 24 V / ≥4,17 A** et température adaptés ; une valeur AC ne suffit pas |
| Filament et pièces imprimées | 6 pièces | [STEP/STL et sources du boîtier](../mechanical/README.md) | [Recherche de filament translucide](https://www.amazon.fr/s?k=PETG+naturel+translucide+filament+1.75mm) | Insert optique translucide ; cassette et autres pièces opaques. Matériau et clips à qualifier |

Les **JST-XH sont au pas de 2,50 mm**. Les intitulés Amazon « XH 2.54 » sont
souvent imprécis : demander confirmation de la série et du pas réels, des
sections et des longueurs ; un assortiment de boîtiers seuls n'est pas un lot
de câbles prêt à brancher. Le détail broche à broche reste dans
[le plan de câblage](wiring_and_harnesses.md).

La [fiche AZ-Delivery du BH1750](https://m.media-amazon.com/images/I/81N6D6v-dYL.pdf)
identifie l'ASIN lié ci-dessus. Le GY-302 Reichelt annoncé 18,5×13,5 mm n'est
pas une correspondance mécanique démontrée avec le berceau prévu 23×16 mm :
adapter ce dernier au module réel. Les cotes des prises, presse-étoupes et de
l'USB restent également à confronter à leurs plans avant impression finale.
La [fiche MEAN WELL](https://www.meanwell.com/Upload/PDF/GST160A/GST160A-SPEC.PDF)
documente le candidat 24 V / 6,67 A et sa sortie Power-DIN.

Les sockets ESP32, les LED et tous les composants montés sur les PCB restent
dans la BOM **usine**. Le cavalier JP1 est demandé livré séparément par l'usine.
Le XL4015 [déjà acheté](https://www.amazon.fr/dp/B0C6R3FMT3) reste à **quantité
zéro** pour ce prototype à buck intégré : ne pas le racheter pour ce montage.

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
