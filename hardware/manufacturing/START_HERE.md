# Dossier prototype A04 / B05 — point d’entrée

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
ENIG demandé. **51 CMS +19 traversants** à assembler en usine, plus un cavalier
à livrer séparément. Ni DevKit ni périphériques à acheter/souder comme U1.
Les68 composants natifs physiques et les articles Harwin sont fixés par MPN.

Les trois petits JST A/B sont POWER2, I2C3 et CTRL6, au pas2,50mm.
Le connecteur de sonde est **B/J4**, NTC passive15k. Voir le schéma et
`wiring_and_harnesses.md` ; ne pas déterminer la polarité à la couleur des fils.

Le STEP courant est `enclosure_v08_integrated_buck.step` : façade amovible
avec LCD/A, rails/clips, berceau B, panneau arrière, USB latéral et ventilation
sur les côtés. Il représente des enveloppes de composants, avec les dimensions
documentées des sockets/borniers et une réserve pour le plastique des broches
mâles. Il ne prétend pas avoir mesuré les trois corps de DevKit ou imprimé les clips.

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
