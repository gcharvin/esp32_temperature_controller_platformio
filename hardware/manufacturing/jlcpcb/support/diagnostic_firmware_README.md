# Diagnostic de banc — A 0.4.1 / B température 0.5.1

Projet PlatformIO **séparé du firmware historique**, destiné à la mise au point.
Il maintient **ENABLE/GPIO27 à zéro en permanence**, et PWM0/GPIO25 à zéro par
défaut. Aucun mode ne permet d'autoriser le chauffage. La compilation ne prouve
pas l'état électrique réel : vérifier ENABLE et la grille de Q1 au multimètre.
Commencer avec **le chauffage débranché**.

## Compiler et installer ultérieurement

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/platformio.exe" run --project-dir firmware/diagnostic -e esp32-bench
```

La cible est un ESP32 classique 30 contacts/4 Mo avec le brochage décrit dans
`hardware/module_selection.json`, pas un ESP32-C3/S3. Plateforme épinglée
`espressif32@7.1.3`, Arduino2.0.17 utilisé lors de la compilation locale.
Le code contient aussi la branche LEDC Arduino3, non testée dans cette livraison.
Il n'utilise que les bibliothèques du framework ; pas de dépendance externe.

Le projet principal n'est pas remplacé. Les binaires de compilation sont dans
`release/`, avec SHA256 dans `hardware/review/diagnostic_firmware_validation.json`.
Pour un premier flash, utiliser le projet PlatformIO complet, qui applique les
adresses de boot/partition appropriées, plutôt qu'un flash manuel du seul binaire.

**Aucune carte branchée n'a été flashée pendant la préparation.** Pour installer
plus tard : conserver/sauvegarder son firmware si nécessaire, utiliser le mode
USB de service du dépôt (sources externes coupées, J1/J6/J7 débranchés, JP1 retiré),
puis ajouter `-t upload --upload-port COMx` à la commande avec le port vérifié.

## Utilisation sans USB pendant les mesures alimentées

Après flash, débrancher l'USB. Raccorder hors tension A/B et leurs périphériques,
JP1 remis pour le fonctionnement externe, puis alimenter selon le plan de banc,
chauffage toujours absent. Un point d'accès local **LabDiag-xxxx** apparaît,
mot de passe **LabDiag2026**. Se connecter depuis un ordinateur/téléphone et
ouvrir **http://192.168.4.1**. Il n'y a ni accès Internet ni besoin de Home Assistant.
Le mot de passe est public et réservé au banc de mise au point ; modifier le
code avant tout déploiement dans un environnement partagé.

La page et `/status` sont en lecture seule : présence I²C attendue, mesure ADS1115,
TMP117, BH1750, position brute de l'encodeur, bouton et niveau FAULT_N.
Le navigateur signale une connexion perdue ; ne pas considérer un ancien relevé
comme une mesure en cours. Cette méthode évite de dépendre d'une séparation
USB/VBUS encore non qualifiée sur les DevKit.

## Ce qui est mesuré

| Élément | Configuration du diagnostic |
| --- | --- |
| I²C | SDA21/SCL22, 100 kHz, timeout25ms |
| ADS1115 sur B | Adresse0x49, AIN0 par rapport à GND, pleine échelle±4,096V, conversion unique128SPS ; résultat brut et mV |
| TMP117 sur A | Adresse0x48, registre température ; module optionnel |
| BH1750 sur A | Adresse0x23, mode haute résolution continu ; module optionnel, valeur non calibrée pour la fenêtre |
| LCD Freenove | Adresse0x27, sinon0x3F, mapping PCF8574 P3=rétroéclairage ; 20×4 |
| Encodeur | A4/B19/bouton23 ; compteur de transitions, pas un nombre garanti de crans |
| Chauffage | ENABLE27 toujours0 ; PWM25 nul sauf test logique série limité |
| FAULT_N33 | Niveau brut ; sans B il peut flotter. Le comparateur ADS est désactivé : ce n'est pas une protection thermique |

Le rétroéclairage reste à intensité fixe ; extinction après60s sans interaction,
réveil par rotation ou appui. Vérifier le fonctionnement sur le LCD réel.
Un ACK I²C est un indice de présence, pas une preuve d'identité du composant.
Ce programme n'effectue ni PID, ni conversion/calibration NTC, ni tests100W.

## Console série à 115200 bauds

Utilisable en USB seul au départ ; en fonctionnement externe, le monitoring USB
exige la solution VBUS séparée validée dans la revue électrique. Ne pas brancher
un câble USB ordinaire pour consulter les logs sous alimentation centrale.

- `status` : état JSON, dont `adc_valid` ; `null` signale une mesure indisponible.
- `scan` : liste des ACK puis réinitialisation des capteurs/LCD, PWM arrêté.
- `stop` : PWM nul, ENABLE maintenu bas.
- `probe 10` : **PWM logique seulement**,500Hz à10% pendant environ2s. Seules les
  valeurs1…20 sont acceptées. ENABLE reste bas : la LED HEAT et la grille Q1
  doivent rester inactives. Ce n'est pas un essai de la charge de puissance.

Les fonctions de politique vérifient à la compilation la durée, le débordement
de `millis()`, les limites de rapport cyclique et le décodage signé ADS.
La tolérance réelle des temporisations, les communications et les niveaux sur
les PCB restent à essayer. Le premier essai du chauffage demandera un autre
firmware adapté et les protections prévues dans le plan de banc.

Sources techniques : [PlatformIO](https://docs.platformio.org/en/latest/projectconf/sections/env/options/build/build_src_filter.html),
[API LEDC Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html),
[ADS1115 TI](https://www.ti.com/lit/ds/symlink/ads1115.pdf),
[mapping LCD Freenove](https://github.com/Freenove/Freenove_LCD_Module/blob/main/Freenove_LCD_Module_for_ESP32/Python/Python_Codes/2.1_IIC_LCD2004/I2C_LCD.py).
