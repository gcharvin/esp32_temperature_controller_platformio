# Essais de compatibilité LMR36520A / ngspice du 5 novembre 2026

**Aucune simulation transitoire complète obtenue.** Ces essais ne valident ni le
régulateur ni le circuit. La bibliothèque constructeur originale reste inchangée.
Les bibliothèques dérivées et les données partielles sont ignorées par Git.

Les essais ciblés de lissage des buffers, portes logiques, comparateurs, monostable
et bascules ont échoué. L'erreur revient principalement sur le nœud
`xu1.x_u6_u847.yint1`, piloté par le monostable interne.

`try_smooth_port.py` permet de reproduire les essais plus larges : tous les `IF`
du modèle sont remplacés par des décisions `tanh`, avec priorités des conditions
imbriquées conservées algébriquement. Largeurs de transition essayées : 100 µV,
1 mV et 10 mV. Deux résistances de fuite de 1 GΩ sont ajoutées aux nœuds de la
bascule `SRLATCHRHP_BASIC_NEW` afin de supprimer son état flottant au calcul DC.
Ces modifications sont des hypothèses de portage, pas une modification approuvée
par TI. Les singularités DC disparaissent, mais les transitoires échouent encore.

Le dernier essai à 1 mV réduit le pas maximal de 100 ns à 5 ns et utilise Gear,
`maxord=2 trtol=1 itl4=200`. Il échoue aussi ; un pas plus fin ne constitue donc
pas une solution démontrée. Les points partiels sont archivés uniquement comme
diagnostic, jamais comme un essai réussi.

Les valeurs et erreurs exactes sont dans `failed_attempts.json`. Les empreintes
de l'original et des dérivés produits sont dans `port_manifest.json`.

Reproduction (à partir de la racine du dépôt, après téléchargement du modèle
officiel par `build_proposal.py`) :

```powershell
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/power_study/experiments/compat_20261105/try_smooth_port.py
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/power_study/run_ngspice.py hardware/power_study/experiments/compat_20261105/all_if_smooth_fine.cir
```

Conclusion : ce portage serait une étude de modèle à part entière. Préférer pour
le prochain prototype un régulateur dont le modèle constructeur peut être simulé
sans ces adaptations, plutôt que présenter ce dérivé comme une garantie.
