# Protocole de réception — cartes A et B température

Pour les cartes **A 0.4.3** et **B 0.5.3** du prototype. Tester d'abord une
carte de chaque type hors du boîtier, sur une surface isolante, puis répéter les
contrôles de réception sur les autres exemplaires. Les mesures décrites ici sont
à effectuer à la réception : elles ne sont pas des résultats déjà obtenus.

Le chauffage, la sonde réelle, l'écran et les autres modules restent débranchés
jusqu'à leur étape respective. Ce protocole vérifie l'alimentation et la mesure
sans commander le chauffage. La qualification de la sortie de puissance est
une étape ultérieure.

## Matériel

- Alimentation de laboratoire réglable jusqu'à 24 V, avec affichage et limite
  de courant réglable, et câbles adaptés aux borniers de B. Régler hors connexion.
- Source de laboratoire 5 V régulée et limitée en courant pour A seule ; la
  même alimentation réglée à 5 V convient.
- Multimètre numérique, pointes fines isolées et cordon de masse à pince adapté.
  Les pastilles de mesure font 1,5 mm et sont au dos des cartes.
- Résistance de précision **15 kΩ, 1 %**, fils et fiche adaptés à B/J4.
- DevKit ESP32 30 broches compatible, câble USB-C de données et ordinateur avec
  PlatformIO. Sauvegarder le firmware existant de la DevKit si nécessaire.
- Téléphone ou ordinateur Wi-Fi pour lire la page de diagnostic locale ; loupe,
  support isolant et fiche de relevés ci-dessous.
- Facultatifs pour les essais correspondants : LCD 20×4 avec backpack, BH1750,
  encodeur, leurs faisceaux, sonde réelle et thermomètre de référence.

Avant de brancher un câble, vérifier son brochage et sa continuité hors tension.
Identifier le numéro de chaque pastille à l'aide des vues de dos [A](../review/test_points_A_back.png)
et [B](../review/test_points_B_back.png), et du [tableau des points](../TEST_POINTS.md).
GND et PGND sont reliées sur B par NT1. Pour mesurer les tensions logiques,
utiliser B/TP12 comme référence ; pour la puissance et la grille, B/TP1.
HEATER_LOW n'est pas une masse. Garder les pointes immobiles et éviter de
ponter deux pastilles. Couper les sources avant chaque modification de câblage.

## 1. Identification et inspection hors tension

1. Relever la révision, le numéro attribué à chaque carte et la date. Comparer
   les références, le sens des LED, diodes, circuits intégrés, condensateurs
   polarisés et connecteurs avec les plans d'assemblage reçus.
2. Chercher soudures manquantes, ponts d'étain, broches tordues et dommages.
   Vérifier l'insertion de la DevKit et de ses barrettes sans forcer ; retirer
   la DevKit pour les premières mesures de rails.
3. Sans alimentation, après décharge des condensateurs, rechercher au multimètre
   un court-circuit franc entre A/TP2 et TP1, A/TP4 et TP1, B/TP2 et TP1,
   B/TP3 et TP1, B/TP5 et TP12. Vérifier la continuité B/TP1–TP12 via NT1.
   La valeur ohmique peut évoluer pendant la charge par le multimètre : une
   lecture basse instantanée seule ne suffit pas à conclure.
4. Si un pont ou un court-circuit persistant est trouvé, arrêter et documenter
   le défaut avant toute alimentation.

## 2. B seule — alimentation et convertisseur

1. A, chauffage J3, sonde J4 et USB débranchés. Brancher la source **24 V CC**
   sur l'entrée B/J2 en respectant le repérage 24V IN et la polarité. Vérifier
   la tension et la polarité avant de connecter la carte.
2. Commencer avec une limitation de courant basse, par exemple **0,2 A** à
   l'entrée, puis mettre sous tension brièvement. Relever courant et tensions.
   Une limitation peut empêcher le démarrage à cause de l'appel du convertisseur ;
   si elle s'active, couper et examiner le montage avant d'ajuster graduellement
   la limite. Ne pas conclure à un court-circuit sur cette seule indication.
3. Pointe noire sur B/TP1 (PGND), mesurer B/TP2 : tension proche de l'entrée
   24 V, après protection. Mesurer B/TP3 puis TP4 : **4,75 à 5,25 V** en régime
   établi. Le voyant 24V et le voyant 5V doivent s'allumer.
4. Pointe noire sur B/TP12 (GND), mesurer B/TP5 : environ **3,3 V**. Le voyant
   3V3 doit s'allumer. Noter la valeur réelle pour l'étape de mesure NTC.
5. Relever le courant, surveiller odeur, fumée et échauffement. Couper aussitôt
   en cas d'anomalie et rechercher sa cause. Couper ensuite l'alimentation.

La tension 5 V est un critère de l'interface ; « environ 3,3 V » est un repère
de diagnostic, à comparer à la tolérance du régulateur et aux conditions de
mesure avant de prononcer une conformité métrologique.

## 3. A seule — rails et DevKit

1. B et USB débranchés, DevKit retirée. Source coupée, raccorder **5 V régulés**
   à A/J1 : broche 1 GND, broche 2 +5V_AB. Ne jamais envoyer 12 ou 24 V sur A.
2. Limiter le courant au banc et alimenter. Pointe noire sur A/TP1, mesurer
   A/TP2 : **4,75 à 5,25 V** ; A/TP4 : environ **3,3 V**. Les voyants 5V_AB
   et 3V3_A doivent s'allumer. Avec JP1 fermé, A/TP3 doit être proche de TP2.
   Couper l'alimentation.
3. Vérifier le sens d'insertion et le brochage réel de la DevKit, puis l'enficher
   hors tension. Les deux rangées ont un entraxe de 25,4 mm. L'alignement des
   signaux et les dimensions sont à vérifier sur l'exemplaire utilisé.

## 4. Flasher le firmware de diagnostic

1. Couper toutes les alimentations externes. Débrancher les liaisons A/B et
   A/J1 ; retirer le cavalier JP1. Brancher l'USB-C de la DevKit à l'ordinateur.
2. Identifier le bon port COM et sauvegarder le firmware si on souhaite le
   restaurer. La commande suivante **écrit dans la mémoire de la DevKit** :

   ```powershell
   & "$env:USERPROFILE/.platformio/penv/Scripts/platformio.exe" run --project-dir firmware/diagnostic -e esp32-bench -t upload --upload-port COMx
   ```

   Remplacer COMx par le port identifié. En cas d'échec, vérifier le câble de
   données et le mode BOOT selon la DevKit.
3. Débrancher l'USB. Remettre JP1 hors tension. Le diagnostic maintient ENABLE
   bas et n'autorise pas le chauffage ; vérifier physiquement cette condition
   pendant l'étape suivante. Voir le [mode d'emploi du firmware](../../firmware/diagnostic/README.md).

## 5. A et B ensemble — sans charge

1. Sources et USB débranchés, raccorder les faisceaux POWER, I²C et CTRL selon
   leurs repères et brochages. Le câble UART est facultatif. Ne pas relier une
   deuxième source 5 V à A. Laisser chauffage et sonde débranchés.
2. Alimenter uniquement B en 24 V avec la limite de courant adaptée au montage.
   Mesurer B/TP4 puis A/TP2 et A/TP3 : **4,75 à 5,25 V** à l'interface A, avec
   JP1 fermé. Mesurer A/TP4 et B/TP5 : environ 3,3 V chacun. Noter le courant.
3. Connecter le téléphone ou l'ordinateur au Wi-Fi **LabDiag-xxxx**, mot de passe
   de banc **LabDiag2026**, et ouvrir **http://192.168.4.1**. Vérifier une
   réponse à jour et la présence I²C attendue de l'ADS1115 à **0x49**. Un ACK
   indique une réponse sur le bus ; il ne prouve pas à lui seul l'identité ni
   la justesse de la mesure.
4. Mesurer A/TP8 et B/TP9 (ENABLE) par rapport à leur GND : niveau proche de
   **0 V**. Mesurer B/TP11 (GATE_DRV) par rapport à B/TP1 : proche de **0 V** ;
   le voyant HEAT doit rester éteint. En cas contraire, couper et arrêter.
5. Si la page est inaccessible, contrôler 5 V, 3,3 V, JP1, orientation de la
   DevKit et faisceaux. SDA A/TP5 et SCL A/TP6 doivent être proches de 3,3 V
   au repos. Un multimètre ne permet pas de qualifier la forme des échanges.

Le raccordement simultané de l'USB ordinaire et du 5 V externe n'est pas encore
caractérisé sur les différentes DevKit. Pour ces essais, utiliser la page Wi-Fi
avec l'USB débranché ; consulter la [revue alimentation et USB](power_and_usb_review.md).

## 6. Simulation de la sonde

1. Couper l'alimentation. Brancher la résistance **15 kΩ** entre les deux
   contacts de B/J4, repérés TEMP / NTC. Réalimenter et attendre la stabilisation.
2. Pointe noire sur B/TP12, mesurer B/TP6 et TP7. Avec exactement 3,3 V sur
   TP5, l'attendu théorique est **environ 1,189 V** ; adapter proportionnellement
   à la tension réelle mesurée et tenir compte des tolérances des résistances.
3. La page de diagnostic doit indiquer une mesure ADS valide et une tension
   proche de celle de TP7. Une différence persistante ou une mesure indisponible
   appelle un contrôle du câblage, du pont et de l'ADS1115.
4. Couper, retirer la résistance et réalimenter : TP6/TP7 doivent être proches
   de 0 V. Couper de nouveau avant toute modification. Le firmware fournit une
   tension brute, pas une température NTC calibrée ni une protection thermique.

## 7. Périphériques, un par un

Hors tension, retirer la résistance si nécessaire, puis raccorder successivement
le LCD, l'encodeur, le BH1750 et la sonde réelle avec leurs faisceaux vérifiés.
À chaque ajout, réalimenter et noter le résultat : affichage lisible, extinction
après environ 60 s et réveil par l'encodeur ; variation du compteur et bouton ;
variation de la luminosité mesurée ; tension NTC plausible et variable avec la
température. La conversion et l'étalonnage de la NTC restent à établir avec un
thermomètre de référence. Couper avant chaque branchement ou débranchement.

## 8. Consignation et décision

Répéter les contrôles appropriés sur les cinq exemplaires A et B. Conserver les
résultats par carte, y compris les échecs et les corrections. Comparer les
cartes entre elles aide au diagnostic sans remplacer les critères de conception.

| Date / opérateur | ID A / B | Alim B V / A | B TP2 / TP3 / TP4 / TP5 V | A TP2 / TP3 / TP4 V | ENABLE / GATE V | NTC TP6 / TP7 / ADS V | Observations et décision |
| --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | |

Noter aussi la polarité et l'inspection, les LED, le scan I²C, le comportement
des périphériques, le firmware et la limite de courant. Une carte non conforme
reste identifiée et isolée pour diagnostic ; ne pas poursuivre jusqu'au chauffage.

Les essais de charge graduelle du buck, de commande du MOSFET avec charge
adaptée, de défaut et d'échauffement sont décrits dans le [plan de banc](../review/routing/electronics_bench_plan.md).
Ils nécessitent notamment une charge appropriée et des critères thermiques.
Ils ne sont pas couverts par l'allumage des voyants ni par ce protocole de réception.
