# Proposition de buck intégré à B — étude 0.1

Étude du 5 novembre 2026. **Candidat retenu : TI LMR36520ADDAR**, avec circuit et bancs de simulation documentés. Le PCB B routé v0.4 n'est pas modifié par cette étude : il conserve son XL4015 externe jusqu'à validation du remplacement.

## Résultat et limite actuelle

- Schéma KiCad autonome et PDF : `buck_5v/buck_5v.kicad_sch`, `buck_5v/buck_5v.pdf` ; projet `buck_5v.kicad_pro`.
- ERC du schéma d'étude : **0 erreur, 0 avertissement**. Le contrôle indépendant `check_proposal.py` vérifie les connexions et calcule les marges statiques.
- Cinq bancs PSpice sont préparés dans `benches/`, avec modèle transitoire constructeur non chiffré et paramètres traçables dans `simulation_plan.json`.
- **Aucune validation fonctionnelle SPICE réussie à ce stade.** Les tentatives avec ngspice 46, fourni par KiCad 10, chargent le modèle mais s'arrêtent sur un pas de temps trop petit dans ses blocs comportementaux internes. Les données partielles ne constituent pas un résultat valide. Voir `ngspice_attempts.json`.

Le prochain jalon électrique est l'exécution complète des bancs dans le moteur pris en charge par le constructeur (PSpice), ou la résolution documentée de la compatibilité ngspice. Les marges calculées et l'ERC ne remplacent pas ce jalon. Cette proposition n'est pas un dossier de fabrication.

## Choix et frontière de l'étude

Le [LMR36520](https://www.ti.com/product/LMR36520) permet un convertisseur synchrone, à compensation interne, dans un boîtier HSOIC à pad thermique. Sa plage de composant jusqu'à 65 V apporte de la marge pour un système 24 V. Le [modèle TI SNVMBX7A](https://www.ti.com/lit/zip/snvmbx7) est disponible sans chiffrement. Le [guide de la carte d'évaluation](https://www.ti.com/lit/ug/snvu677/snvu677.pdf) fournit un circuit, une implantation et une BOM de référence.

La tension du **système complet reste 24 V régulé** : un composant 65 V ne rend pas le PMOS d'entrée, C8, les protections ou le reste de B compatibles avec une alimentation 65 V. Le banc commence **après les protections amont**, et se termine **avant F2**. Fusible, anti-inversion, TVS, câbles d'entrée, firmware et partage USB ne sont pas simulés ici.

Objectif proposé : 5 V pour la logique, budget de travail **0,8 A** à confirmer par mesure de l'écran, de la DevKit et des périphériques. Le circuit est classé 2 A, mais **F2 reste à 1 A sur le PCB existant** ; il n'est pas question d'autoriser 2 A dans le faisceau A/B sans une nouvelle revue.

## Circuit proposé

```text
VIN_PROTECTED (B, 24 V)
    ├── C1/C2/C3 ── GND
    ├── U1 VIN       U1 SW ── L1 ── +5V_BUCK ── F2 existant ── +5V_AB
    └── R1 ── EN                         │
               └── JP1 vers GND          ├── C6/C7/C8/C9 ── GND
                                         └── R2 ── FB ── R3 ── GND

C4 : BOOT ↔ SW ; C5 : VCC_INTERNAL ↔ GND
R4 : +5V_BUCK ↔ PG_5V_TP (point de mesure seulement)
```

Les repères de cette étude sont indépendants de ceux de B. Les deux connecteurs J1/J2 du schéma matérialisent des frontières électriques pour la revue, sans imposer deux nouveaux connecteurs au PCB final.

| Repère | Choix de départ | Base documentaire |
| --- | --- | --- |
| U1 | LMR36520ADDAR, 400 kHz | [Fiche TI, rév. C](https://www.ti.com/lit/ds/symlink/lmr36520.pdf) |
| L1 | 10 µH, Coilcraft XAL6060-103MEC | [Coilcraft](https://www.coilcraft.com/en-us/products/power/shielded-inductors/molded-inductor/xal/xal60xx/xal6060-103/) : DCR max. 29,8 mΩ à 25 °C ; vérifier tenue et pertes réelles |
| C1/C2 | 2 × 4,7 µF / 100 V, 1210 | Exemple de référence : TDK CGA6M3X7S2A475K200AB |
| C3 | 220 nF / 100 V, 0805 | Exemple : Murata GRM21AR72A224KAC5L |
| C4 | 100 nF / 25 V, 0603 | Exemple : KYOCERA AVX 06033C104KAT2A |
| C5 | 1 µF / 25 V, 0603 | Exemple : TDK C1608X5R1E105K080AC |
| C6…C9 | 4 × 22 µF / 25 V, 1210 | Exemple : Samsung CL32A226KAJNNNE ; banc de sensibilité à 44 µF effectifs au total |
| R2 / R3 | 100 kohms / 24,9 kohms, 1 % | Pont de réglage fixe ; pas de potentiomètre |
| R1 / R4 | 100 kohms | Activation par défaut / rappel de PG ; choix du domaine PG à revoir à l'intégration |

Les exemples de condensateurs proviennent de la BOM de l'[EVM TI](https://www.ti.com/lit/ug/snvu677/snvu677.pdf). Stock assembleur, courbes sous polarisation DC et références finales restent à confirmer. Le choix de quatre condensateurs de sortie reprend cette référence et donne une réserve pour étudier la perte de capacité sous tension. Les résistances ESR de 5 mΩ dans les bancs sont des hypothèses par condensateur ; elles ne sont pas des composants à acheter. L1 est représentée linéairement avec sa résistance série ; saturation et échauffement ne sont pas reproduits.

**VCC_INTERNAL n'alimente aucun périphérique.** C'est l'alimentation interne du régulateur, découplée uniquement par C5. **PG_5V_TP est dans le domaine 5 V** : il ne doit pas être relié directement à un GPIO ESP32. Pour une surveillance par l'ESP32, prévoir un rappel de PG vers le 3,3 V lors de l'intégration. JP1 fermé force EN à zéro ; il ne commande pas ENABLE du chauffage.

## Calculs vérifiés, sans simulation de commutation

`analytical_checks.json` est produit depuis la netlist réelle du schéma :

- Réglage nominal : `1 × (1 + 100000 / 24900) = 5,016 V`.
- Avec résistances à 1 % et référence à ±1,5 %, coins statiques calculés : **4,862 à 5,174 V**, avant pertes de distribution, transitoires et autres erreurs.
- À 24 V, 10 µH et 400 kHz, ondulation idéale d'inductance ≈ **0,990 A crête à crête** ; pour 0,8 A de sortie, crête idéale ≈ **1,295 A**. Ce calcul à inductance/fréquence nominales n'est pas un essai de courant limite.
- Inductance de 10 µH, ou 8 µH à −20 %, supérieure au minimum de calcul de 5,25 µH pour ces paramètres.
- À titre illustratif, 5 W utiles avec un rendement supposé de 90 % donneraient **0,56 W de pertes**. Ce n'est ni une mesure ni une prédiction de température.

## Bancs et critères à valider

| Banc | Essai préparé | Critère proposé |
| --- | --- | --- |
| `startup.cir` | Montée du 24 V et activation, charge 0,5 A | Démarrage complet ; 4,75–5,25 V après stabilisation |
| `load_step.cir` | 0,1 → 0,8 → 0,1 A | Rester dans 4,75–5,25 V ; relever dépassement et temps de retour |
| `load_step_derated.cir` | Même essai, capacité de sortie effective divisée par deux | Même plage, absence d'oscillation durable |
| `line_step.cir` | 24 → 26,4 → 18 V | Régulation et retour à l'équilibre ; ce n'est pas un test de la TVS amont |
| `disable.cir` | EN abaissé après fonctionnement | Arrêt de la commutation, décharge sans réactivation spontanée |

L'ondulation stabilisée visée est inférieure à 50 mV crête à crête ; c'est un objectif à mesurer, pas une performance démontrée. Le modèle de charge réduit la demande sous 4,75 V pour éviter qu'une source de courant idéale force une tension négative au démarrage. Il ne représente pas le firmware réel de l'ESP32. Le modèle TI indique que les effets de température ne sont pas modélisés.

Le modèle constructeur conserve son en-tête et ses équations d'origine. Son archive est téléchargée à la demande, avec empreintes SHA-256 enregistrées. Elle est conservée localement et exclue de Git ; aucun modèle idéal inventé ne remplace discrètement le régulateur.

```powershell
# Générer l'étude et télécharger le modèle officiel si absent
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/power_study/build_proposal.py
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/power_study/check_proposal.py

# Essai de compatibilité ngspice ; actuellement susceptible d'échouer
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/power_study/run_ngspice.py hardware/power_study/benches/startup.cir
```

Les `.cir` peuvent être utilisés comme bancs dans PSpice avec le modèle TI. L'ouverture du schéma dans KiCad n'affecte pas automatiquement les modèles SPICE aux symboles ; les bancs explicites sont la référence de simulation actuelle. Le passage à un schéma directement simulable dans l'interface KiCad nécessite la résolution de compatibilité et l'affectation des modèles.

## Intégration envisagée dans B après validation

1. Supprimer le module externe et ses raccordements J8/J9 ; connecter le buck entre VIN_PROTECTED et +5V_BUCK. Conserver F2, les voyants 5 V et le contrat d'alimentation de A.
2. Revoir délibérément la liaison **GND/PGND** : le shunt éventuel du XL4015 disparaît. Une référence commune devient possible, avec un retour chauffage direct vers l'entrée et des retours de mesure maîtrisés. Ne pas ajouter un pont arbitraire aux fichiers actuels sans reprendre l'implantation.
3. Réserver environ 25 × 25 mm à confirmer à l'implantation, en déplaçant au besoin les voyants. Garder les boucles de commutation courtes, les condensateurs d'entrée contre VIN/PGND, le retour de feedback éloigné de SW, et respecter l'implantation TI. Le pad thermique n'est pas un substitut à la broche PGND ; procédé de vias/pâte à valider avec l'assembleur.
4. Vérifier les courbes de capacité, le courant réel, l'échauffement et le bruit sur la NTC ; maintenir les capteurs d'ambiance hors du flux chaud.
5. Fermer le partage USB/5 V et adapter le firmware séparément. Le remplacement du buck ne résout pas ces deux points.

La maquette ventilée v0.6 conserve la baie et le berceau XL4015 pour les essais existants. Leur suppression éventuelle accompagne une future implantation intégrée validée ; le volume estimé du nouveau buck n'est pas présenté comme déjà implanté ou testé dans le STEP.
