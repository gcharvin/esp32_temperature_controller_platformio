# Préparation de fabrication A v0.4 / B v0.5

Révision courante **A0.4.2 / B0.5.2**, ajout du **JST UART J10** sur chaque carte.
Les22 pastilles de test au dos sont conservées. Le lot comprend désormais
**70 composants natifs achetés, plus les deux sockets**. Cela représente
**51 CMS et21 traversants physiques à assembler**, sockets incluses. Les anciens noms A04/B05 désignent la famille de fichiers.
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

[component_selection.json](component_selection.json) fixe les70 composants
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
les21 placements traversants physiques et les contrôles liés aux sources.
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
Les51 CMS et les21 traversants sont à assembler en usine ; cavalier séparé.
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

**Plan économique courant du9novembre2026 :** catalogue Amazon.fr/GoTronic
en priorité ; sur-mesure LCSC limité aux quatre références sans équivalent
catalogue exact qualifié. L’utilisateur accepte des soudures à faire réaliser,
**aucun sertissage manuel**. Le [plan détaillé et son devis réduit](economy_cabling/README.md)
remplace la stratégie tout-sur-mesure du premier dossier LCSC.

La [BOM système](system_bom_review.csv) conserve25lignes et les quantités
par appareil ; le [plan de câblage](wiring_and_harnesses.md) distingue
enfichage, soudures et raccordement par borne. La [politique fournisseur](customer_supply_policy.json)
garde deux canaux catalogue et un câbleur uniquement pour les manques.

### Sur-mesure restant : quatre références partagées

| Référence | Pour10appareils | Pourquoi |
| --- | --- | --- |
| H01 POWER | 10 | XHP-2 femelle-femelle300mm AWG22 droit, pas de câble catalogue exact qualifié |
| H02 I2C | 10 | XHP-3 femelle-femelle300mm AWG26 droit, même limite d’approvisionnement |
| H03 CTRL | 10 | XHP-6 femelle-femelle300mm AWG26 droit, six fils dont5/6NCsurB |
| U04 universel | 30 en base, **40au total** si TMP117 | XHP-4 vers quatre femelles individuelles2,54mm, AWG26/300mm, broches numérotées1..4. Une référence pour LCD, BH1750, encodeur et optionTMP |

Le câbleur livre les contacts sertis et les boîtiers montés. Les fils isolés
et les repères de cavités sont standards : pas de surmoulage, de gaine commune
spéciale ni de sous-ensemble soudé industriel à commander. Le [ZIP économique](LCSC_CABLE_RFQ_ECONOMY_NOT_FOR_ORDER.zip)
est le dossier actuel de consultation, sans envoi ni commande. L’ancien ZIP
de13lignes est un comparatif historique, pas la liste à déposer maintenant.

### Achats catalogue regroupés

Prix publics observés le9novembre2026, à revérifier avec stock, port et
contenu des lots. Ils ne sont pas des prix LCSC.

| Article | Fournisseur / référence | Besoin pour10appareils | Intervention |
| --- | --- | --- | --- |
| Encodeur | [GoTronic ADA377/38383](https://www.gotronic.fr/art-encodeur-24-impulsions-ada377-37481.htm),5,90EURTTC | 10siabsents,59EUR | Couper les femelles côté module du U04 puis souder A/B/SW/GND, communC-SW2 ponté |
| Cordon sonde interne | [GoTronic JST1164/49197](https://www.gotronic.fr/art-cordon-jst-xh-femelle-2-cts-15-cm-jst1164-33095.htm),2,10EURTTC | 10,21EUR **si15cm suffisent** | Deux soudures au jack, isolation séparée et protection du faisceau |
| Protection interne sonde | [GoTronic GAIT06/48620](https://www.gotronic.fr/art-gaine-tressee-gait06-15337.htm),0,80EUR/mTTC | Proposition2m,1,60EUR | Polyester expansible3..9mm, à retenir aux extrémités ; abrasion, pas double isolation |
| Adaptateur TMP117, option | [GoTronic QW-4C/49152](https://www.gotronic.fr/art-cordon-qwiic-stemma-qw-4c-30786.htm),2,10EURTTC | 10optionnels,21EUR | Fini SH vers4mâles, à joindre à10U04 supplémentaires ; pas de transition de calibres à fabriquer |
| Jack embase+fiche | [Amazon DAOKI B08XM6DR96](https://www.amazon.fr/dp/B08XM6DR96),candidat | 10paires utiles, nombre de lots à confirmer | Soudures ; dimensions au coupon et serre-câble à vérifier, offre non relue |
| Regroupement interne, option | [GoTronic SW06B/48441](https://www.gotronic.fr/art-gaine-cache-fils-sw06b-205.htm),3,90EUR/10mTTC | 1bobine si utile | Pose de spirale seulement si adaptée au diamètre ; réutiliser les accessoires existants |
| DevKit/LCD | Déjà achetés, réutiliser | Pas de nouvel achat par défaut | Sockets et composantsPCB déjà au devis usine ; JP1 fourni séparément |

Le sous-total public connu est **81,60EURTTC** (encodeurs+cordons sonde+tresse),
ou102,60EUR avec les10adaptateursTMP, hors port et autres articles. Les15cm
du JST1164 ne sont pas encore validés pour le démontage : ne pas acheter le
lot avant essai du trajet, et reconsulter pour30cm s’il manque du mou.

### Coût et achats différés

Le [budget détaillé](economy_cabling/README.md#budget-et-main-dœuvre) vise
environ **250-450EUR de débours pour10jeux internes sansTMP**, ou280-500EUR
avec l’option, **hors main-d’œuvre de soudure et hors périphériques/appareils**.
Ce sont des hypothèses et provisions, pas un devis ni une économie garantie.
Compter cinq points de soudure encodeur et deux au jack par appareil,
soit70points pour10jeux, plus les fiches des sondes extérieures quand retenues.
Le coût de cet atelier doit être chiffré, pas supposé nul.

Sonde extérieure15k, alimentation, chauffage, protection thermique et rallonge
USB de panneau restent à qualifier : pas de commande sur-mesure anticipée
pour ces cinq lignes. La sonde garde impérativement une **gaine extérieure
commune robuste**, des retenues anti-arrachement et une classe thermique
adaptée. Le BH1750 reste dans la famille GY-302 ; le plus grand ADA4681 n’est
pas substitué. Les presse-étoupesM12 et le coupon PJ-392 restent inchangés.

Pour la puissance, les Phoenix1715721 acceptent du conducteur souple nu
0,14..1,5mm2 : le0,75mm2 peut donc être **dénudé et serré directement**, sans
embout obligatoire ni sertissage (7mm,0,5..0,6Nm, bloc soutenu, aucun brin
libre, aucun étamage). Les autres extrémités et la classe thermique restent
à choisir avec les appareils. [Données fabricant](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721).

Les XH2,50mm, SH1mm et Dupont2,54mm ne sont pas interchangeables. Aucun kit
Amazon15cm ou à contacts nus n’est compté comme câble300mm déjà terminé.
La liste n’affirme pas que le panier soit entièrement prêt à commander.

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
