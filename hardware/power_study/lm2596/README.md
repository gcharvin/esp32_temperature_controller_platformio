# Variante simulable dans KiCad : LM2596 fixe 5 V

Cette variante d'étude permet d'utiliser le moteur ngspice livré avec KiCad, sans installer PSpice. Le composant est **LM2596S-5.0**, sortie fixe 5 V ; le modèle ne représente pas une version ajustable. Les modèles du régulateur et de la diode ne sont pas modifiés. Cette variante est maintenant intégrée au PCB B v0.5 ; voir `../../pcb_b_temperature/README.md`. Les bancs restent séparés du PCB et ne reproduisent pas ses parasites, protections amont ou câblages.

`prepare.py` télécharge les deux modèles officiels, contrôle leurs SHA-256, puis génère les bancs et plans d'évaluation. Les modèles restent dans le dossier local ignoré `../vendor_model/` :

- [TI LM2596](https://www.ti.com/product/LM2596), modèle [SNVMA62](https://www.ti.com/lit/zip/snvma62), `LM2596_5P0_TRANS`, Final 1.00 du 6 juillet 2015.
- [Vishay SS36](https://www.vishay.com/docs/88751/ss32.pdf), diode 60 V / 3 A, modèle [SPICE3 officiel](https://www.vishay.com/docs/88171/ss36.txt).

Le mode de compatibilité PSpice de KiCad correspond à `psa`. La méthode d'intégration **Gear d'ordre maximal 2** est utilisée : les jonctions capacitives du modèle de diode rendaient l'intégration trapézoïdale excessivement lente dans ces essais. Ce réglage ne change ni les seuils ni les équations du modèle constructeur. Une analyse qui atteint réellement la fin du transitoire est exigée avant toute évaluation électrique.

## Circuit et paramètres

Entrée après protection : 24 V, étudiée de 18 à 26,4 V ; sortie avant F2 : 5 V / 0,8 A. Le LM2596 est limité à 40 V, avec moins de marge en entrée que le LMR36520 proposé précédemment. Sa fréquence de 150 kHz et sa diode externe entraînent des passifs plus encombrants et des pertes à revoir.

- Inductance : **Bourns SRP1770TA-101M, 100 µH ±20 %** ; DCR maximale 118 mΩ utilisée dans les bancs. La [fiche constructeur](https://www.bourns.com/docs/Product-Datasheets/srp1770ta.pdf) donne Isat1 = 6,5 A pour une perte d'inductance de 20 %. Le modèle reste linéaire : il ne démontre pas la tenue thermique ou la saturation durant le démarrage.
- Entrée : 100 µF avec ESR 100 mΩ, approximation à remplacer par la référence finale et son comportement en fréquence. Le courant d'appel depuis l'alimentation, le fusible d'entrée et les protections amont restent hors périmètre.
- Sortie : **330 µF / 35 V**, par exemple [Panasonic EEUFR1V331U](https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFR1V331UB), ±20 %, impédance maximale 43 mΩ à 100 kHz et 20 °C.
- **Résistance physique de 56 mΩ ajoutée en série avec le condensateur de sortie**, dans sa branche vers la masse, afin d'encadrer son impédance minimale. Elle ne se trouve pas en série avec la charge. Le LM2596 demande une ESR de sortie adaptée à sa boucle de compensation ; substituer directement une banque de céramiques à cette branche ne correspond pas au circuit étudié.
- Le modèle de branche utilise 100 mΩ au total, arrondi au-dessus de 43 + 56 mΩ. Le banc `esr_lower_bound` utilise 56 mΩ au total, en supposant l'ESR interne du condensateur nulle. Cette plage est une approximation autour de la fréquence de commutation à température ambiante, et non une qualification de température, vieillissement ou ESL.

Les bancs `load_step_derated` et `esr_lower_bound` utilisent **L = 80 µH et C = 264 µF**, soit les tolérances basses à −20 %. Ce sont des coins de passifs, pas une analyse statistique ou une couverture de toutes les tolérances internes du régulateur.

Le port modèle `ON_OFF_N` est **actif haut pour l'arrêt** : zéro permet le fonctionnement ; 3,3 V le coupe. La charge comportementale réduit sa consommation sous 4,75 V afin de ne pas forcer une tension négative au démarrage. Elle ne reproduit pas le firmware ni les pointes de courant réelles de l'ESP32.

## Exécution et résultats

Générer les modèles et bancs avec `python prepare.py`, puis utiliser `../run_ngspice.py` sur chaque `.cir` et `../evaluate_simulations.py validation_plan.json`. Le projet KiCad de simulation est généré séparément dans `../kicad_simulation/` ; ses modèles sont affectés aux symboles, contrairement à un simple schéma de fabrication.

Les six cas d'acceptation sont décrits dans `validation_plan.json` : démarrage, saut de charge 0,1 ↔ 0,8 A, tolérances des passifs, borne ESR basse, variation d'entrée et arrêt. `results/simulation_results.json` contient les valeurs mesurées sur les transitoires complets et indique explicitement les critères réussis ou manqués. Les objectifs sont 4,75–5,25 V après démarrage et une ondulation stabilisée inférieure à 50 mV crête à crête.

Les deux essais séparés de `sensitivity_plan.json` explorent **20 mΩ et 150 mΩ** de branche de sortie avec L/C à −20 %. Le premier correspondrait notamment à l'absence de la résistance physique de 56 mΩ ; le second dépasse l'approximation nominale de condensateur à température ambiante. Leur réussite ou échec doit être rapporté comme sensibilité, sans transformer les hypothèses nominales en garantie universelle.

Résultats du 5 novembre 2026, ngspice 46 de KiCad 10, compatibilité `psa`, modèles inchangés : **les six essais nominaux atteignent leur fin et satisfont les critères définis**. Pendant le saut de charge nominal, la sortie reste entre 4,9615 et 5,0128 V, avec 26,9 mV crête à crête en charge stabilisée. Les cas L/C à −20 % et ESR basse atteignent respectivement 23,3 et 15,2 mV. La commande d'arrêt laisse 0,210 V en sortie à 15 ms, sans impulsions hautes de SW pendant la dernière milliseconde.

Les deux sensibilités supplémentaires atteignent aussi 12 ms. À 20 mΩ : sortie 4,9692–4,9998 V, ondulation 8,35 mV. À 150 mΩ : 4,9597–5,0081 V, ondulation 31,78 mV. Elles ne montrent pas d'oscillation durable sur cette durée ; elles ne démontrent pas une marge de phase sur toute la plage de température. Régénérer ces mesures avec `uv run measure_sensitivity.py` ; elles sont enregistrées séparément dans `sensitivity_results.json`. Ne pas utiliser le répertoire de résultats nominaux pour ces mesures.

Le courant crête de démarrage observé dans ces modèles est **4,88 à 4,93 A**, nettement supérieur au courant de service de 0,8 A. Le choix de l'inductance ci-dessus donne une marge de courant, mais la saturation réelle, l'énergie de la diode et le comportement du fusible restent à vérifier avant intégration.

Le buck est maintenant intégré au schéma et au routage B v0.5 et au boîtier
v0.7. La préparation de fabrication, les références candidates et les points
de revue sont dans `../../manufacturing/README.md`. La simulation ne qualifie
ni tous les coins de composants, ni les parasites/EMI, ni la thermique,
ni la tenue réelle au démarrage et en défaut de la diode et des fusibles.
Les essais sur prototype avec alimentation limitée en courant suivent la
revue de conception ; ils ne sont pas présentés comme déjà réalisés.

## Exploration d'une future alimentation 12 V

Le contrat de plateforme garde A sous 5 V régulé, même avec une carte B
alimentée en 12 V ou 5 V. B v0.5 reste une carte température 24 V. Un essai
**séparé** `benches/integrated_B_startup_12V.cir` reprend exactement le cœur
24 V avec C9 et ne change que l'entrée pour un démarrage 12 V / charge 0,5 A.
Il atteint 8 ms ; les critères nominaux passent : sortie stabilisée entre
4,9810 et 4,9920 V, ondulation 10,31 mV crête à crête, maximum au démarrage
5,0122 V et courant crête de self 4,7994 A. Résultats et limites :
`startup_12V_results.json`. Ce n'est ni une qualification complète à 12 V,
ni une extension de la puissance chauffage 100 W aux autres tensions.

```powershell
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/power_study/run_ngspice.py hardware/power_study/lm2596/benches/integrated_B_startup_12V.cir --vectors 'v(vout)' 'v(vin)' 'v(sw)' 'l1#branch'
& 'C:/Program Files/KiCad/10.0/bin/python.exe' hardware/power_study/lm2596/evaluate_12v_startup.py
```

La variante 5 V doit disposer d'une distribution adaptée : le LM2596 fixe
5 V n'est pas un chemin passant 5 V -> 5 V. Aucun essai 5 V équivalent n'est
revendiqué. Voir `../../manufacturing/power_and_usb_review.md`.
