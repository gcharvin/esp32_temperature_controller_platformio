# Préparation de fabrication A v0.4 / B v0.5

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

## Régénération complète

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/scripts/prepare_fabrication.py --pdf-python 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
```

KiCad10, uv/CadQuery et l’interpréteur PDF avec ReportLab/pypdf sont nécessaires.
Les13 étapes s’arrêtent au premier échec. Les sources mécaniques sont figées
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
