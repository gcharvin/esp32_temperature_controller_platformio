# LCD Freenove LCD2004 : rétroéclairage et option PWM

Étude du 6 novembre 2026. Module indiqué par l'utilisateur :
[Freenove, ASIN B0B76VGXYH](https://www.amazon.fr/dp/B0B76VGXYH),
famille documentaire [FNK0079](https://docs.freenove.com/projects/fnk0079/en/latest/).
La page commerciale n'a pas fourni de schéma exploitable. Les ressources
constructeur ont été examinées ; aucune continuité n'a été mesurée sur l'exemplaire acheté.

## Décision pour le prototype

Conserver le LCD et son backpack, J2 à quatre fils, alimentation **3,3 V** déjà
utilisée par l'utilisateur. Éclairage à niveau fixe, extinction après inactivité,
réveil à la rotation ou à l'appui sur l'encodeur. Cette politique reste à porter
et tester dans le firmware de la nouvelle électronique ; cette étude ne prétend
pas avoir ajouté ces fonctions au firmware historique.

Le BH1750 reste sur A/J5 : 1 GND, 2 +3V3_A, 3 SDA, 4 SCL. Le futur réglage de
luminosité n'est pas nécessaire pour exploiter sa mesure dans Home Assistant.
La maquette v0.9 prévoit sa fenêtre fermée et son support amovible.

## Ce que permettent réellement les quatre fils I²C

Dans le [pilote LCD2004 Freenove](https://github.com/Freenove/Freenove_LCD_Module/blob/main/Freenove_LCD_Module_for_ESP32/Python/Python_Codes/2.1_IIC_LCD2004/I2C_LCD.py),
P0/P1/P2 commandent RS/RW/E, P3 le rétroéclairage, P4 à P7 les données D4 à D7.
Les huit sorties sont donc affectées. La
[bibliothèque C++ distribuée par Freenove](https://github.com/Freenove/Freenove_LCD_Module/blob/main/Freenove_LCD_Module_for_ESP32/C/Libraries/LiquidCrystal_I2C.zip)
utilise le masque `0x08` pour le rétroéclairage. Son `setBacklight(uint8_t)` teste
seulement zéro/non-zéro : **128 et 255 donnent le même état allumé**.

Le [PCF8574/PCF8574A](https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf)
est un expandeur GPIO, sans générateur PWM. Un clignotement par écritures I²C
répétées dépendrait du temps logiciel et chargerait le bus partagé avec les
mesures. Ce n'est pas la solution retenue pour obtenir une variation stable.

La commande marche/arrêt existe dans
[ESPHome lcd_pcf8574](https://esphome.io/components/display/lcd_display/) :
`backlight()` et `no_backlight()`. Elle coupe la lumière sans interrompre
l'alimentation du contrôleur LCD. Le cavalier du module doit permettre cette
commande ; son rôle électrique exact doit être vérifié sur le backpack réel.

## Un PWM reste possible en conservant l'affichage I²C

Il faut commuter **le courant des LED**, séparément du circuit logique LCD.
L'I²C continue à transmettre le texte. Il ne faut pas moduler tout le VCC,
la masse du backpack ou la tension de contraste pour faire varier la lumière.

Première solution à vérifier : remplacer le cavalier d'éclairage par un petit
interposeur. **Seulement si une mesure confirme que ce cavalier relie le rail
positif au circuit LED**, un interrupteur MOSFET côté positif peut le remplacer :

```text
rail LCD ---- source P-MOS drain ---- ancien cavalier côté LED
                 |
             grille + rappel vers source
                 |
          transistor de commande <-- PWM 3,3 V

limitation de courant existante + LED + commutation du backpack conservées
VCC logique LCD et SDA/SCL : continus, inchangés
```

C'est un schéma de principe conditionnel, pas un circuit libéré pour câblage.
Le transistor de commande permet de séparer le niveau PWM du rail LED ; la
polarité logicielle, l'état au démarrage, RDS(on), VGS, courant et limitation
doivent être dimensionnés après relevé. Le bit P3 reste actif pour laisser passer
le PWM externe ; sa mise à zéro garde la fonction extinction si la topologie le
permet. Si le cavalier concerne un retour ou un autre nœud, ce montage doit être
adapté. Ne pas relier une sortie GPIO en parallèle avec P3, ni faire passer le
courant du rétroéclairage dans une broche de l'ESP32.

Deux façons de produire ce PWM :

| Option | Connexions supplémentaires | Conséquence |
| --- | --- | --- |
| PWM matériel ESP32 | Un GPIO disponible jusqu'à l'interposeur, plus masse | Simple pour un essai ; aucun fil PWM n'existe dans J2 actuel. Prévoir un petit connecteur supplémentaire lors d'une révision de A, sans détourner J7 réservé aux cartes B. |
| Contrôleur PWM sur I²C | Repiquage du bus et de son alimentation, commande locale du MOSFET | Pas de GPIO ni fil de commande supplémentaire entre A et l'ensemble écran ; petit circuit ajouté derrière le LCD. |

Pour le second cas, le [PCA9685 est pris en charge par ESPHome](https://esphome.io/components/output/pca9685/).
Un module d'évaluation permettrait l'essai avant une éventuelle petite carte.
Ses sorties commanderaient l'étage de commutation, pas directement une charge
de courant inconnu. Son nombre de canaux est excessif pour un seul écran.
Le [PCA9632](https://www.nxp.com/docs/en/data-sheet/PCA9632.pdf) offre quatre voies
PWM 8 bits à 1,5625 kHz ; il demanderait une intégration logicielle vérifiée.
Sa limite de courant interdit de présumer qu'il alimente directement ce LCD.
Ni l'un ni l'autre n'est ajouté à la BOM du prototype.

## Essai qui permettra de choisir sans remplacer l'écran à l'aveugle

1. Photographier le backpack, relever le circuit, les résistances et le cavalier.
   Hors tension, relever les liaisons cavalier/VCC/LED+/LED−/transistor ; vérifier
   aussi où se trouvent les résistances de rappel SDA/SCL.
2. Sur alimentation limitée à 3,3 V, mesurer le courant du module et celui des
   LED, puis vérifier extinction/allumage par P3. Confirmer lecture et contraste.
3. Si le cavalier s'y prête, essayer l'interposeur séparé avec PWM matériel,
   initialement autour de 1 kHz, aux niveaux 0/10/50/100 %. Vérifier l'absence de
   défaut d'affichage et d'échauffement, ainsi que l'état au redémarrage.
4. Mesurer le BH1750 à éclairage ambiant constant, écran allumé puis éteint.
   Corriger le masquage optique avant toute boucle automatique de luminosité.

**Conclusion : le backpack n'empêche pas le PWM, mais ne le fournit pas.**
On peut préserver l'écran acheté et ajouter une interface si son câblage le
permet. L'examen documentaire ne prouve pas la topologie de son cavalier :
le point restant est un relevé électrique court, pas une nouvelle simulation
du chauffage. Aucun remplacement de LCD ni modification de A n'est nécessaire
pour le mode fixe avec extinction choisi aujourd'hui.

## Alimentation centrale et port USB

Le chemin normal est : B/buck **5 V régulés** → A/J1 → JP1 → VIN/5V de la
DevKit. Le 3,3 V des périphériques est produit sur A ; aucune entrée 12/24 V
sur VIN dans cette plateforme. Le port USB-C latéral sert aux données et au
service. Ce choix d'usage ne supprime pas une éventuelle liaison électrique
interne VIN–VBUS dans une DevKit. Le mode série sous alimentation centrale
reste à qualifier suivant la [revue alimentation/USB](manufacturing/power_and_usb_review.md).

Les cotes LCD du STEP restent des enveloppes de référence ; le lien d'achat
ne vaut pas validation dimensionnelle de l'exemplaire et de son backpack.
