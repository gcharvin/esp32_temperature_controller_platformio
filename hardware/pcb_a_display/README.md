# Carte A v0.4 — hôte commun et affichage

Ouvrir `pcb_a_display.kicad_pro` puis le PCB ou le schéma. Format110×75×1,6mm,
deux couches35µm, réserves mécaniques et antenne conservées.

La préparation finale utilise deux sockets **HarwinM20-7821546**,15 contacts
chacun, pas2,54mm, rangées25,40mm confirmées par l'utilisateur. L'empreinte
locale `LabHardware:ESP32_30P_Harwin_M20_7821546_Rows25.40mm` emploie des trous
nominaux1,05mm et des zones de dégagement dimensionnées pour les boîtiers Harwin. La DevKit
reste amovible, fournie par l'utilisateur ; ses trois variantes partagent le
brochage confirmé. Le nom historique diymore/ASSUMED n'est plus l'empreinte active.

Les MPN/fabricants des composants à assembler sont inscrits dans KiCad et
contrôlés avec `../manufacturing/component_selection.json`. Les voyants
emploient les pastilles recommandées Kingbright, cathode1. Le fabricant
soude les CMS, JST, JP1 et les deux sockets ; le cavalier JP1 est livré séparé.

J1 = **1GND,2+5V régulé**. Ne pas appliquer12V ou24V à cette carte. J6 I²C =
1GND,2SDA,3SCL ; J7 =1GND,2PWM0,3ENABLE,4FAULT_N,5PWM1,6DIN0. Les périphériques
I²C sur J2/J4/J5 utilisent1GND,2+3V3_A,3SDA,4SCL ; J3 encodeur utilise
1A,2B,3bouton,4GND. Voir les schémas et le tableau de faisceaux pour le câblage.

USB seul pour le service : sources externes coupées, J1/J6/J7 débranchés,
JP1 retiré. JP1 ne gère pas automatiquement VBUS/VIN. Les circuits USB des
DevKit et leur hauteur réelle restent des vérifications d'intégration.

La maquette v0.8 inclut les sockets maximaux documentés et une enveloppe
d'entretoise mÃ¢le2,54mm. Voir le [dossier de fabrication](../manufacturing/START_HERE.md)
pour les plans, le routage, le contrôle usine et les accords externes requis.

## Révision diagnostic du 6 novembre 2026

Les pastilles TP au dos sont du cuivre seul, sans pâte ni pièce à acheter.
Voir [le plan de mesure](../TEST_POINTS.md) et le firmware autonome dans
`../../firmware/diagnostic/`. Les pistes antérieures et le circuit de puissance
sont conservés ; le diagnostic maintient ENABLE à zéro.

## Extension UART du 10 novembre 2026

Révisions **A0.4.2 / B0.5.2** : J10 est un JST-XH3 supplémentaire assemblé
sur chaque PCB, pas2,50mm, même MPN B3B-XH-A(LF)(SN) que J6.

| Broche J10 | Carte A | Carte B température |
| --- | --- | --- |
| 1 | GND | GND |
| 2 | TX_A, GPIO17, pad U1.22 | Non connectée ; réserve RX de future B |
| 3 | RX_A, GPIO16, pad U1.21 | Non connectée ; réserve TX de future B |

Câblage **droit 1→1, 2→2, 3→3** ; TX/RX sont nommés du point de vue de A.
UART logique **3,3V**, sans alimentation dans ce connecteur : jamais RS232
ni5V directement. Les transceivers RS232/RS485 éventuels appartiennent aux
futures B. Le port série USB de la DevKit reste indépendant ; les deux PWM
et ENABLE/FAULT_N/DIN0 restent disponibles. GPIO16/17 doivent être libres
sur la variante de DevKit employée (modules à PSRAM utilisant ces broches
non couverts par cette interface). Aucun UART n’est activé par le firmware
de diagnostic actuel ; aucun essai de boucle matérielle n’est déclaré fait.

B température n’a pas de périphérique UART : les deux pastilles sont
intentionnellement NC et le connecteur est néanmoins à souder. Le câble UART
est **facultatif pour ce prototype** et n’est pas ajouté au devis LCSC en cours.
Si nécessaire plus tard, reprendre la construction H02 XHP-3/XHP-3,300mm AWG26
avec un repérage UART distinct. Ne pas confondre J6 I²C et J10 UART, mécaniquement
identiques. Les alias fabricant J8/J9 des deux sockets de A restent inchangés.
