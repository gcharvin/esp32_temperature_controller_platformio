# Cartes métier B souhaitées

Synthèse du **6 novembre 2026**, issue des demandes utilisateur et de la relecture
du dépôt. Elle décrit les extensions à prévoir autour de A ; elle ne lance pas
leur conception. **Le premier prototype reste A + B température.**

## Catalogue des besoins

| Carte B | Usage demandé | Entrées / sorties à prévoir sur B | Alimentation / puissance connues | État |
| --- | --- | --- | --- | --- |
| **Température** | Mesurer une sonde et réguler un chauffage, notamment pour les échantillons de microscopie | Sonde NTC 15 kΩ, ADS1115 ; PWM chauffage, autorisation et défaut | Entrée 24 V DC ; chauffage résistif jusqu'à 100 W, soit 4,17 A ; buck 5 V intégré | **B v0.5 routée**, préparation de fabrication locale effectuée ; essais physiques et accord fabricant restants |
| **Pompe piézo avec driver dédié** | Piloter la pompe en conservant son électronique de commande | Interface vers le driver : marche/arrêt et réglage ; PWM, analogique ou liaison série selon sa documentation ; éventuel retour d'état | Tension, courant et signaux du driver inconnus | Souhaitée ; référence du driver à relever |
| **Moteur pas à pas** | Commander un moteur et son déplacement | Driver moteur ; STEP/DIR/ENABLE ou interface propre au driver ; défaut et éventuelles fins de course | Tension moteur, courant par phase, nombre d'axes et performances à préciser | Souhaitée |
| **Illumination LED** | Commander l'allumage et l'intensité d'un système d'illumination | Entrée de commande d'un éclairage existant, ou étage LED à courant constant ; PWM et/ou consigne analogique ; nombre de canaux à définir | Caractéristiques des LED ou du driver à relever | Souhaitée ; distincte du rétroéclairage LCD de A |
| **Équipement à interface RS232** | Envoyer les commandes ASCII connues et lire les réponses d'un équipement | UART logique vers transceiver RS232 sur B ; connecteur, niveaux électriques et protocole propres à l'appareil | Alimentation de B à définir ; équipement possiblement alimenté séparément | Souhaitée ; référence, brochage, débit et protocole à documenter |
| **MOSFET pour électrovannes** | Ouvrir/fermer plusieurs vannes solénoïdes Lee Company ou d'un autre fabricant | Un étage MOSFET protégé par voie ; commande individuelle, autorisation globale ; retour de courant/défaut à étudier | **12 V DC et environ 100 mA par vanne**, indication utilisateur à confirmer, soit environ 1,2 W par vanne | Carte distincte confirmée ; commande MOSFET retenue, références, nombre de voies et temps de commutation à préciser |
| **Relais génériques** | Disposer de sorties tout-ou-rien réutilisables pour divers équipements | Contacts secs ; COM/NO, éventuellement NC ; drivers et protection des bobines ; indicateur de commande par voie | Rail des bobines, tension/courant commutés et nombre de voies à définir | Carte distincte confirmée ; dédiée aux commutations génériques par contacts secs |

Les six extensions après température sont des besoins de plateforme, pas des
cartes déjà schématisées, routées, simulées ou ajoutées à la commande du prototype.
L'ordre de réalisation après température n'est pas encore décidé.

## Décision : deux cartes distinctes, relais génériques et MOSFET pour vannes

Décision utilisateur du **6 novembre 2026** : prévoir une **B relais générique**
à contacts secs et une **B MOSFET pour les électrovannes**, séparées dans le
catalogue. Le premier prototype reste exclusivement **A + B température** ;
cette décision n'ajoute aucune de ces cartes à la fabrication en cours.

La B MOSFET comportera des sorties adaptées aux charges inductives, avec protection
contre la surtension de coupure. La topologie exacte, les composants et un éventuel
pilotage d'appel/maintien seront déterminés à partir des références des vannes.
La B relais conserve les contacts secs pour les commutations génériques d'autres
circuits. Elle n'est plus l'architecture prévue pour la carte électrovannes.

Le courant de 100 mA est une estimation de travail, pas une spécification Lee.
Il faut relever la référence exacte, la résistance de bobine, le courant d'appel
et de maintien, le régime admissible, la polarité et le type monostable/bistable.
Une vanne bistable peut demander une inversion de courant ; une vanne à driver
dédié peut demander seulement un signal logique, dont le niveau est à adapter.
Certains produits Lee utilisent une commande « spike and hold » : cela ne permet
pas de conclure que les vannes possédées acceptent toutes un 12 V continu.
[Source constructeur : Lee, drivers spike and hold](https://www.theleeco.com/uploads/2021/11/INIX0500200A-D1.pdf).

Une diode de roue libre simple ralentit la démagnétisation ; si le temps de
fermeture est important, étudier un écrêtage adapté au driver et à la bobine.
Sur la B relais, la protection des bobines de relais ne protège pas, à elle seule,
une éventuelle charge inductive externe branchée sur les contacts. Pour les deux
cartes, la protection doit intégrer l'énergie inductive et la fréquence de commutation.
[Source technique : TI, commande et décharge des charges inductives](https://www.ti.com/document-viewer/lit/html/snvaa45).

Avec l'hypothèse utilisateur, quatre vannes simultanées demanderaient environ
**0,4 A / 4,8 W sous 12 V** ; huit, **0,8 A / 9,6 W**. Ajouter la logique, la
conversion 5 V et les appels de courant. Le budget des bobines de relais relève
séparément de la B relais générique.
Ces calculs ne fixent ni le nombre de voies ni le calibre d'une future carte.

## Ce que A doit permettre aux futures B

Le socle reste commun : une seule DevKit ESP32 sur A, LCD, encodeur, TMP117 et
BH1750. Les étages de puissance, protections de charges, interfaces analogiques
ou transceivers RS232 appartiennent à B. Une B peut contenir des circuits I²C,
des drivers ou une EEPROM sans recevoir un second ESP32.

Le [contrat A/B actuel](interface_ab.json) repose sur trois petits JST-XH au
pas **2,50 mm**, raccordés hors tension :

| Faisceau | Contrat actuel | Usage pour les extensions |
| --- | --- | --- |
| J1, POWER, 2 contacts | 1 GND ; 2 +5V_AB, de B vers A | A reçoit toujours un **5 V régulé**, même si la puissance métier est alimentée en 12 ou 24 V ; une B à entrée 5 V doit prévoir une distribution adaptée |
| J6, I²C, 3 contacts | 1 GND ; 2 SDA ; 3 SCL, logique 3,3 V | Mesures, expandeur de sorties lentes, contrôleur PWM matériel ou identification optionnelle sur B ; pas d'alimentation dans ce faisceau |
| J7, CTRL, 6 contacts | 1 GND ; 2 PWM0 ; 3 ENABLE ; 4 FAULT_N ; 5 PWM1 ; 6 DIN0 | Signaux directs rapides, autorisation globale et défaut ; les deux dernières voies ne sont pas utilisées sur B température |

Les GPIO câblés sur A sont PWM0=25, ENABLE=27, FAULT_N=33, PWM1=26 et DIN0=32.
`FAULT_N` est une entrée de défaut à collecteur/drain ouvert actif bas ; les
signaux A/B restent en 3,3 V. Aucun courant de moteur, vanne, relais ou LED ne
doit transiter par ces GPIO ou le faisceau POWER destiné à A.

Quelques profils **candidats**, à valider avec la future B et son firmware :

- **Pas à pas, un axe** : PWM0 comme STEP, PWM1 comme DIR, ENABLE pour le driver,
  FAULT_N pour un défaut compatible et DIN0 pour un retour. La cadence et les
  accélérations doivent être produites par un périphérique/logiciel approprié,
  sans dépendre d'échanges MQTT ni d'un expandeur I²C lent.
- **RS232** : réaffecter PWM1/GPIO26 à UART TX et DIN0/GPIO32 à UART RX, puis
  adapter les niveaux sur B. Le faisceau existe, mais ce profil n'est pas encore
  implémenté. Des lignes RTS/CTS supplémentaires demanderaient une nouvelle
  solution ; les deux signaux disponibles suffisent seulement au cas TX/RX.
- **Vannes ou relais multivoies** : sorties adressées par I²C sur B, plus ENABLE
  câblé pour bloquer matériellement les étages. Le nombre de voies ne doit pas
  imposer autant de GPIO supplémentaires sur A. Prévoir les états inactifs au
  démarrage/reset et le comportement à la perte de commande ; le nombre de
  sorties, les adresses et les fonctions de sécurité restent à concevoir.
- **LED ou pompe à driver** : GPIO direct pour une commande temporelle, ou
  contrôleur/DAC sur B si plusieurs canaux ou une consigne analogique sont requis.
  Ne pas présumer que les entrées du driver acceptent directement le 3,3 V.

Ces profils utilisent les broches existantes mais ne constituent pas une preuve
de compatibilité universelle de A. La logique de sélection, les niveaux, les
débits, le budget électrique et les références réelles restent à vérifier par B.
Une éventuelle identification par EEPROM I²C n'est pas présente sur le prototype
température ; aucune reconnaissance automatique ou connexion à chaud n'est acquise.

## Fonctions communes et limites du prototype

- **PC** : communication série via le pont USB de la DevKit et l'USB-C latéral.
  Ce port n'est pas une interface électrique RS232 ; celle des équipements
  externes est le métier de la B correspondante. Le monitoring USB avec
  alimentation centrale suit la [qualification VBUS restante](manufacturing/power_and_usb_review.md).
- **Home Assistant** : ESPHome/MQTT est le socle souhaité pour les configurations
  métier ; les extensions et protocoles particuliers pourront demander du code.
  Une application Android par Bluetooth reste une perspective à définir.
- **Alimentation** : A fonctionne depuis le 5 V central vers VIN/5V de la DevKit,
  pas depuis un 12/24 V brut. Le budget partagé actuel de 0,8 A n'est pas une
  capacité disponible pour les charges métier. Chaque B dimensionne sa puissance
  et fournit un 5 V conforme ; B température v0.5 n'est pas qualifiée sous 12 ou 5 V.
- **Mécanique** : conserver la façade A/LCD, l'accès USB et une B accessible par
  l'arrière. Revoir encombrement, dissipation, connecteurs de sortie et panneau
  arrière pour chaque métier ; le contour B actuel de 90 × 65 mm est une base,
  pas une garantie que toutes les variantes y tiendront.

Le TMP117 d'ambiance et le BH1750 sont des périphériques communs de A, pas deux
cartes métier à ajouter. Le rétroéclairage LCD reste fixe avec extinction
automatique et réveil par encodeur comme comportement cible ; voir
l'[étude dédiée](display_backlight_study.md).

## Origine et documents de référence

Les besoins pompe piézo, pas à pas, LED, RS232, électrovannes et relais sont
consignés ici depuis la discussion utilisateur. La relecture des documents
Markdown, des listes de pièces et du document d'assemblage n'a pas fait ressortir
d'autre carte métier explicitement souhaitée. Les anciens capteurs DHT11 et
thermistances mentionnés dans [src/README.md](../src/README.md) relèvent du
prototype thermique historique ; aucune carte humidité distincte n'est déduite.

Références locales : [électronique courante A/B](README.md),
[B température](pcb_b_temperature/README.md),
[contrat d'alimentation](manufacturing/power_and_usb_review.md),
[séparation logicielle UI/métier existante](../src/pkg/README.md).
Le fichier [chatgpt.md](../chatgpt.md) conserve des propositions anciennes de
connecteur 20 broches et d'alimentation sur A : elles ne remplacent pas le
contrat actuel à petits JST et 5 V régulé.
