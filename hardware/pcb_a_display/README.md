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
d'entretoise mâle2,54mm. Voir le [dossier de fabrication](../manufacturing/START_HERE.md)
pour les plans, le routage, le contrôle usine et les accords externes requis.

## Révision diagnostic du 6 novembre 2026

Les pastilles TP au dos sont du cuivre seul, sans pâte ni pièce à acheter.
Voir [le plan de mesure](../TEST_POINTS.md) et le firmware autonome dans
`../../firmware/diagnostic/`. Les pistes antérieures et le circuit de puissance
sont conservés ; le diagnostic maintient ENABLE à zéro.
