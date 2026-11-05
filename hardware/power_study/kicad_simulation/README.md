# Simuler le buck dans KiCad 10

Ouvrir `startup/lm2596_5v_startup.kicad_pro`, puis son schÃ©ma dans l'Ã©diteur. Choisir **Inspection â†’ Simulateur**. Le projet sÃ©lectionne le classeur `lm2596_5v_startup.wbk` ; s'il n'est pas chargÃ© automatiquement, l'ouvrir avec **Fichier â†’ Ouvrir un classeur** dans le simulateur. Cliquer sur **ExÃ©cuter**. Les tracÃ©s prÃ©parÃ©s sont VOUT, VIN et le courant de L1.

Le mode de compatibilitÃ© du projet est **PSpice**, dans les rÃ©glages du simulateur KiCad. Il s'agit du mode de lecture de modÃ¨les de **ngspice fourni avec KiCad**, sans installer PSpice ni demander de licence TI. KiCad 10 traduit ce choix en `ngbehavior=psa`.

Le modÃ¨le officiel TI `LM2596_5P0_TRANS` et le modÃ¨le Vishay `ss36` sont chargÃ©s par des directives `.include`. Les champs `Sim.Device`, `Sim.Params` et `Sim.Pins` des symboles sont dÃ©jÃ  renseignÃ©s. Le mode Â« code SPICE brut Â» permet de transmettre les sous-circuits au moteur ngspice sans dÃ©pendre du parseur de l'Ã©diteur de modÃ¨les KiCad.

| Dossier | ExpÃ©rience |
| --- | --- |
| `startup` | DÃ©marrage Ã  24 V, charge commandÃ©e de 0,5 A |
| `load_step` | Ã‰chelons de charge 0,1 â†’ 0,8 â†’ 0,1 A |
| `load_step_derated` | MÃªme essai avec L et C de sortie Ã  âˆ’20 % |
| `line_step` | EntrÃ©e 24 â†’ 26,4 â†’ 18 V, charge 0,8 A |
| `disable` | ArrÃªt par ON/OFF Ã©levÃ© Ã  3,3 V aprÃ¨s 5 ms |
| `esr_lower_bound` | L et C Ã  âˆ’20 %, rÃ©sistance de sortie totale 56 mÎ© |
| `stress_esr_20m` / `stress_esr_150m` | SensibilitÃ© hors du cas nominal ; consulter les rÃ©sultats avant toute conclusion |

Chaque dossier contient un projet, un schÃ©ma, un classeur, le PDF, le rapport ERC et la netlist SPICE **rÃ©ellement exportÃ©e par KiCad**. Les valeurs proviennent des bancs de rÃ©fÃ©rence `../lm2596/benches/*.cir`. Le symbole U1 conserve le brochage physique : 1 VIN, 2 OUT, 3 GND, 4 FB, 5 ON/OFF. L'ordre du modÃ¨le est VIN, FB, OUT, GND, ON_OFF_N ; le remappage est explicite. L'anode de SS36 est Ã  la masse et sa cathode au nÅ“ud de commutation.

R1 reprÃ©sente la rÃ©sistance de l'inductance, R4 l'ESR d'entrÃ©e, et R2 la rÃ©sistance sÃ©rie totale de la branche du condensateur de sortie. Dans la proposition physique, celle-ci comprend une rÃ©sistance ajoutÃ©e de 56 mÎ© et l'ESR propre du condensateur. Le cas nominal Ã  100 mÎ© est une hypothÃ¨se de calcul ; ce n'est pas une caractÃ©risation du condensateur achetÃ©. La charge B1 rÃ©duit la demande sous 4,75 V. R3 charge uniquement le gÃ©nÃ©rateur idÃ©al de consigne V3 ; elle fournit une connexion explicite au nÅ“ud de commande pour l'ERC, sans changer la consigne.

Ces fichiers sont des **bancs de simulation sÃ©parÃ©s**, pas une modification du PCB B routÃ©. Les propriÃ©tÃ©s affichÃ©es par le schÃ©ma (R2 compris) dÃ©crivent le modÃ¨le Ã©lectrique ; elles ne constituent pas une BOM de fabrication. Les conclusions et limites fonctionnelles sont dans `../lm2596/validation_plan.json` et les rÃ©sultats de l'Ã©tude. Ã‰chauffement, parasites du PCB, protection amont et partage USB restent en dehors de ce modÃ¨le.

Pour reconstruire les fichiers aprÃ¨s un clonage, depuis la racine du dÃ©pÃ´t :

```powershell
$python = 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
& $python hardware/power_study/lm2596/prepare.py
& $python hardware/power_study/build_kicad_simulation.py
```

Pour vÃ©rifier directement la netlist exportÃ©e avec la mÃªme bibliothÃ¨que ngspice et le mÃªme mode de compatibilitÃ© que le projet :

```powershell
& $python hardware/power_study/run_ngspice.py `
  hardware/power_study/kicad_simulation/startup/lm2596_5v_startup.cir `
  --compatibility psa --vectors 'v(/vout)' 'v(/vin)' 'v(/sw)' 'l1#branch'
```

Les sources sont Ã©ditables dans KiCad. Une rÃ©gÃ©nÃ©ration Ã©crase les fichiers des bancs ; conserver les modifications manuelles dans une copie avant de relancer le gÃ©nÃ©rateur.

Documentation : [simulateur KiCad 10](https://docs.kicad.org/10.0/en/eeschema/eeschema.html#simulator), [modes ngspice employÃ©s par KiCad](https://gitlab.com/kicad/code/kicad/-/raw/10.0/eeschema/sim/ngspice.cpp), [modÃ¨le LM2596 officiel TI](https://www.ti.com/lit/zip/snvma62), [modÃ¨le SS36 officiel Vishay](https://www.vishay.com/docs/88171/ss36.txt).

Vérification du 5 novembre 2026 : le classeur de démarrage a été chargé puis exécuté dans l’interface KiCad 10. Le calcul a terminé avec 1 793 217 points et les courbes VIN/VOUT affichées. Les commandes `.save all` du classeur suivent le format natif KiCad ; une directive de sauvegarde personnalisée concaténée au transitoire empêchait auparavant le chargement. La netlist réellement exportée a également terminé via le lanceur ngspice avec vérification de sa durée de 8 ms. Les autres cas ont été vérifiés par leurs bancs automatisés et par ERC.
