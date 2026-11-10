# Devis JLCPCB du 10 novembre 2026 - variantes disponibles

Les devis en ligne ont atteint l'etape **Quote & Order** pour **5 A et 5 B
assemblees**. Aucun panier, commande, achat de composants ou paiement n'a ete
valide. Le remplacement des references indisponibles a ete autorise par
l'utilisateur. Les stocks ci-dessous ont ete lus dans le configurateur, sans
reservation ; ils peuvent changer.

**Intégration effectuée : A0.4.3 / B0.5.3, STEP v12.** Les références
ci-dessous sont maintenant celles des sources KiCad et des dossiers de
fabrication régénérés. Les BOM `QUOTE_ONLY`, captures et résultats de ce
répertoire conservent l'instantané du devis initial. Les sept simulations sont
reliées à la révision native par `../../power_study/lm2596/current_acceptance.json`.
Le contrôle complet actuel du boîtier est dans `../../mechanical/exports/`.
Les anciens contrôles `mechanical/` de ce répertoire restent ceux de l'étude
préalable, avec leurs hashes d'origine. Aucune approbation de fabrication ni
commande n'est déduite de cette intégration.

## Budget, en USD

| Poste | 5 cartes A | 5 cartes B | Total |
| --- | ---: | ---: | ---: |
| PCB nus | 33.38 | 63.76 | 97.14 |
| PCBA, composants et assemblage | 67.87 | 169.44 | 237.31 |
| Total affiche par le configurateur | **101.25** | **233.20** | **334.45** |
| Etuvage affiche separement | 7.94 | 7.94 | 15.88 |
| Depanellisation affichee separement | 3.31 | 3.31 | 6.62 |
| Budget avec ces options chiffrées | **112.50** | **244.45** | **356.95** |

Hors transport, taxes et eventuels autres frais apres revue. Le site precise
que certains frais d'options avances sont exclus et seront chiffres apres
revue. **356.95 USD est donc un budget provisoire, pas un montant final payable.**
Le detail brut et les captures sont dans `quote_A.txt`, `quote_B.txt`,
`quote_A.png` et `quote_B.png`. Les lignes PCBA comprennent les composants,
chargements de feeders, soudure SMT, soudure manuelle et assemblage manuel :
les JST, borniers et supports femelles selectionnes ne sont pas omis.

Options : Standard PCBA, face superieure, 5 PCB/5 assembles par carte, FR4
TG155, 1.6 mm, vert, ENIG 1 micro-inch ; cuivre A 1 oz, B 2 oz. Rails ajoutes
par JLCPCB : A 110 x 85 mm et B 90 x 75 mm, pour des cartes fonctionnelles
110 x 75 mm et 90 x 65 mm. Depanellisation demandee avant livraison.
Confirmation du placement demandee, sans confirmation automatique.
Delais affiches : PCB A 3 jours / B 24 h, assemblage 4-5 jours, plus 3 jours
pour les options ; hors revue, approvisionnement et livraison.

## Equivalents retenus pour chiffrage

| Positions | Reference du devis | Code | Stock observe | Comparaison |
| --- | --- | --- | ---: | --- |
| A/C1,C2 ; B/C1,C2,C7 | Walsin 0603B105K160CT | C108463 | 67244 | 1 uF, 16 V, X7R, 10 %, 0603 comme le TDK |
| A/C3 | Samsung CL10A106KP8NNNC | C19702 | 9747393 | 10 uF, 10 V, X5R, 10 %, 0603 comme le TDK |
| A/J8,J9 | HCTL PM254-1-15-Z-8.5 | C2897378 | 2859 | 1x15, pas 2.54 mm, hauteur 8.5 mm ; remplace Harwin |
| B/C10 | Panasonic EEUFS1V331LB | C1584641 | 755 | 330 uF, 35 V, 20 %, pas 5 mm ; plus haut, diametre plus petit |
| B/R13 | Vishay WSL1206R0500FEA | C844686 | 7527 | **50 mOhm au lieu de 56**, 1 %, 0.25 W ; meme serie et empreinte |
| B/D8 HEAT | Kingbright APT2012LSYCK/J3-PRV | C5879382 | 1583 | **Jaune au lieu d'orange**, faible courant 2 mA, meme empreinte |

A/JP1 conserve son MPN Harwin M20-9990246 : code identifie C3332353,
stock observe 3222. Ce n'est pas une substitution physique. Les 5 cavaliers
amovibles M7566-05 ne sont pas dans le CPL ni dans ce prix d'assemblage :
les fournir separement au montage final. DevKit, ecran, capteurs externes,
encodeur, cables et boitier restent les fournitures externes deja documentees.

Deux candidats ont ete rejetes : LED rouge C5879380, stock reel nul ;
condensateur C2960251 propose initialement par l'automatching, non conforme
au Panasonic specifie. Aucune substitution automatique non revue n'est retenue.

## Verifications techniques

- **72 positions controlees** : 24 A et 48 B, MPN et code LCSC compares aux
  BOM de devis, toutes cochees pour assemblage. Voir `live_BOM_A.json` et
  `live_BOM_B.json`. Les nombres d'articles factures (14 A / 36 B) sont des
  references regroupees, pas le nombre de composants poses.
- MLCC : valeurs, tension, dielectrique, tolerance et taille conserves.
  Walsin : enveloppe maximale 1.75 x 0.95 x 0.95 mm ; Samsung : 1.70 x 0.90
  x 0.90 mm. Leur capacite nominale ne garantit pas la capacite effective sous
  polarisation. La validation physique de stabilite/transitoires des rails
  AP2112 reste necessaire, comme dans le prototype initial.
- Socket HCTL : longueur `15 x 2.54 + 0.40 = 38.50 +/-0.30 mm`, largeur
  `2.50 +/-0.15`, hauteur `8.50 +/-0.15`, broche 0.64 x 0.40 mm, longueur
  sous boitier 3.20 +/-0.25, percage recommande 1.02 mm. Compatible avec le
  percage 1.05 mm et l'enveloppe existante 38.85 x 2.65 x 8.65 mm. L'entraxe
  de 25.40 mm entre les deux rangees reste inchange. Verifier l'enfichage reel.
- C10 FS : corps nominal D8 x L15, hauteur maximale retenue 16.5 mm ; suffixe
  **LB a conserver pour le pas forme de 5 mm**. Impedance maximale 41 mOhm
  a 100 kHz/20 C, courant d'ondulation 1240 mA a 100 kHz/105 C. L'ancien FR
  etait D10 x L12.5, 43 mOhm. Le controle mecanique conserve volontairement
  l'ancienne enveloppe D10, plus large, avec la nouvelle hauteur maximale.
- R13 est une **resistance d'amortissement dans le retour de C10**, pas une
  resistance dans le retour de puissance du chauffage. Sa nouvelle valeur
  impose une simulation specifique, realisee ci-dessous.
- D8 jaune : cathode pad 1, meme dessin de lands 1.25 x 1.10 mm, intervalle
  1.10 mm. Vf typique 1.85 V a 2 mA ; R12 reste 2.2 kohm, courant nominal
  voisin de 1.4 mA sous 5 V. MSL 3, precautions d'etuvage conservees.

### Simulation du buck

`qualify_buck_alternatives.py` reprend les modeles TI LM2596 et Vishay SS36
non modifies et les criteres existants : sortie 4.75-5.25 V, ondulation <50 mV.
Sept simulations completes passent : demarrage, echelon de charge,
L/C a -20 %, ESR basse 49 mOhm (marge sur 50 mOhm +/-1 %), echelon de tension,
arret, et stress ESR haute 180 mOhm avec L/C a -20 %.

Plage regulee globale mesuree : **4.9575-5.0128 V** ; ondulation maximale
**35.31 mV crete a crete**. Resultats et courbes dans `simulation/results/`.
Les hashes des entrees et formes d'onde sont conserves dans le rapport.
Les fichiers `.dat` volumineux sont regenerables et ignores par Git.
Les ESR constantes sont une approximation : ni parasites PCB, ni thermique,
ni EMI, ni plage de temperature complete ne sont qualifies par ces essais.

Reproduire depuis la racine :

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/manufacturing/online_quote_20261110/qualify_buck_alternatives.py
uv run hardware/power_study/evaluate_simulations.py hardware/manufacturing/online_quote_20261110/simulation/validation_plan.json
uv run hardware/manufacturing/online_quote_20261110/qualify_mechanical_alternatives.py
```

### Montage et revue de fabrication a terminer

Le controle BRep de la variante avec C10 a 16.5 mm donne **0 collision statique
de C10 et 20 etapes de montage/service sans collision**. Voir `mechanical/`.
Il reutilise les enveloppes conservatrices du boitier v11 ; il ne remplace pas
un essai imprime ni une validation des tolerances, des doigts ou des faisceaux.

Les previsualisations JLCPCB restent indicatives. Elles ne constituent pas une
approbation des orientations : verifier avec le fabricant tous les pads 1,
polarites, detrompeurs et conventions de centre/rotation CPL, notamment les
JST, Q1, U1, C8/C10 et les diodes, sur ses plans d'assemblage finaux. Aucun
placement final ni dossier de production n'a ete approuve par cette session.

## Sources techniques

- [Walsin, caracteristiques de la reference 1 uF](https://www.ic-components.se/files/d4/0603B105K160CT.pdf)
- [Samsung, fiche de la reference 10 uF](https://static6.arrow.com/aropdfconversion/f24e86c802cdb5d71185c025570539bb11b8d108/specsheetcl10a106kp8nnn20250808.pdf)
- [HCTL, dessin fabricant fourni par LCSC](https://www.lcsc.com/datasheet/C2897378.pdf)
- [Panasonic, reference exacte FS](https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFS1V331LB)
- [Panasonic FS, tolerances et impedance](https://industrial.panasonic.com/cdbs/www-data/pdf/RDF0000/ABA0000C1255.pdf)
- [Vishay WSL, meme dessin pour 6 a 200 mOhm](https://www.vishay.com/docs/30100/wsl.pdf)
- [Kingbright, LED jaune faible courant](https://www.kingbrightusa.com/images/catalog/spec/apt2012lsyck-j3-prv.pdf)
