# Dossier prototype A04 / B05 — point d’entrée

Dernière révision native : **A0.4.2 / B0.5.2**, ajout du JST UART J10 sur A et B,
avec les22 points de test au dos conservés. Les archives A04/B05 sont régénérées.
Voir `../TEST_POINTS.md` et `../../firmware/diagnostic/README.md` : le programme
de banc maintient ENABLE à zéro, sans autoriser le chauffage. Il est compilé
mais n'a pas été flashé/testé sur le matériel. Le dossier JLCPCB dédié est dans
`jlcpcb/README.md`.

Préparation technique pour **5 cartes A et 5 cartes B assemblées**.
B est la variante température24V/100W ; A reçoit toujours5V régulé et garde
son interface commune aux futures B12V/5V. L'ESP32 est enfichable et fourni
par l'utilisateur. Aucun achat ni ordre de production n'est autorisé par ce dossier.

## Fichiers à utiliser

- `FABRICATION_HANDOFF_A04_B05.zip` : dossier final local, sources KiCad et
  mécaniques, fichiers de devis, contrôles et revue visuelle liés par SHA256.
- `RFQ_A04_B05_DRAFT_NOT_FOR_ORDER.zip` : sous-dossier électrique pour devis,
  avec Gerber/perçages, BOM/CPL, traversants, tests et plans d'assemblage.
- `output/pdf/assembly_review_A04_B05.pdf` : première lecture des implantations,
  broches1, polarités et montage des sockets.
- `fabrication_specification.json` : options de fabrication et périmètre précis.
- `assembly_RFQ_draft.md` : demande de devis prête à transmettre, rédigée
  localement et **non envoyée**.
- `preparation_status.json` : état final calculé des travaux locaux et des
  accords externes ; `handoff_integrity_validation.json` contrôle l'archive finale.

Les noms de fichiers portant `NOT_FOR_ORDER` restent volontaires : les fichiers
sont préparés, mais le fabricant n'a pas encore confirmé son DFM, son processus,
ses orientations et le devis. Ne pas transmettre un ancien packv03/v04 ou
un ancien STEP à la place du dossier courant.

## Périmètre figé

Deux couches, A110×75mm /35µm et B90×65mm /70µm, épaisseur nominale1,6mm,
ENIG demandé. **51 CMS +21 traversants** à assembler en usine, plus un cavalier
à livrer séparément. Ni DevKit ni périphériques à acheter/souder comme U1.
Les70 composants natifs physiques et les articles Harwin sont fixés par MPN.

Les trois JST A/B requis sont POWER2, I2C3 et CTRL6. J10 UART3 est ajouté
sur les deux PCB, signaux NC sur B température, câble facultatif. Tous sont au pas2,50mm.
Le connecteur de sonde est **B/J4**, NTC passive15k. Voir le schéma et
`wiring_and_harnesses.md` ; ne pas déterminer la polarité à la couleur des fils.

Le STEP courant est `enclosure_v11_integrated_buck.step` : façade amovible
avec LCD/A, rails/clips, berceau B, panneau arrière, USB latéral et ventilation
sur les côtés. Il représente des enveloppes de composants, avec les dimensions
documentées des sockets/borniers et une réserve pour le plastique des broches
mâles. La révision v0.9 ajoute une fenêtre translucide fermée de 0,6 mm et une
cassette opaque amovible pour le BH1750 : six pièces imprimées au total.
La v0.10 remplace le GX12 par un jack PJ-392 : trou6,3 mm et logement d'écrou
dans une paroi localement1,5 mm. Dessin Tayda documenté ; équivalence du lot
Amazon DAOKI non qualifiée, coupon d'essai fourni séparément. Les références
et les deux extrémités des faisceaux sont dans `wiring_and_harnesses.md` ; les
soudures restant au jack et les raccordements non figés y sont explicites.
Le breakout BH1750 reste une enveloppe non mesurée ; transmission optique,
connecteur, clips et trois corps de DevKit restent à vérifier sur le matériel.
Voir `../display_backlight_study.md` pour le LCD Freenove, l'extinction/réveil
retenus et les possibilités PWM non ajoutées à cette BOM.

## Ce qui doit revenir du fabricant

Renseigner `factory_response.csv` avec les pièces justificatives :

1. Sourcing exact, disponibilité et devis détaillé, sans substitution automatique.
2. DFM/panneau/outillage, trous finis1,05mm des sockets, bords dépanélisés propres.
3. Accord de profil réel LM2596≤245°C, MSL et assemblage des traversants.
4. Aperçu final BOM/CPL validant les orientations et polarités.
5. Périmètre des inspections/tests et conditions de livraison.

L'autorisation d'achat intervient ensuite. Aucun résultat externe n'est simulé
ou marqué confirmé à la place du fabricant. Ces échanges ne sont pas effectués
par les scripts de préparation.

## Après réception

Commencer sans ESP32 et sans chauffage selon `factory_acceptance_test.md`,
puis suivre `../review/routing/electronics_bench_plan.md`. Vérifier les rails,
les transitoires, la température et les modules réels. Le firmware historique
doit être adapté avant pilotage du chauffage ; la protection thermique
indépendante dépend de l'expérience. Le service USB retenu reste USB seul,
toutes sources externes coupées, J1/J6/J7 débranchés, JP1 retiré.
