# CÃ¢blage du prototype A/B

Les numéros ci-dessous sont ceux des **pastilles PCB**. Ne pas les déduire
du sens de lecture des fils sur une photographie : la vue côté entrée de
câble est inversée par rapport à la face d'accouplement. Faire un contrôle
de continuité broche à broche avant branchement. Tous les raccordements et
changements de carte métier se font hors tension.

## Faisceaux A/B à acheter déjà équipés

| Faisceau | Extrémités | Longueur de devis | Conducteurs |
| --- | --- | --- | --- |
| POWER | A/J1 ↔ B/J1, XHP-2 | 300 ±10 mm | AWG22, pin1 noir/GND, pin2 rouge/+5V_AB |
| I2C | A/J6 ↔ B/J6, XHP-3 | 300 ±10 mm | AWG26, 1 GND, 2 SDA, 3 SCL |
| CTRL | A/J7 ↔ B/J7, XHP-6 | 300 ±10 mm | AWG26, 1 GND, 2 PWM0, 3 ENABLE, 4 FAULT_N, 5 PWM1, 6 DIN0 |

Câblage droit **1→1, 2→2, etc.**, sans croisement ni pont. Les positions5/6
du faisceau CTRL sont présentes ; elles restent non connectées sur cette
carte B température. Elles ne doivent pas être reliées à la masse.
Le tableau machine `harness_pin_map.csv` est contrôlé contre les deux netlists.

Boîtiers sélectionnés : JST **XHP-2, XHP-3, XHP-6**. Contacts **SXH-001T-P0.6**
adaptés aux AWG22–28 et à l'isolant spécifié par JST ; faire confirmer la
combinaison fil/contact par le cÃ¢bleur. Il faut 4 contacts pour POWER, 6 pour
I2C et 12 pour CTRL, soit **22 contacts** et six boîtiers par jeu A/B.
Les XH sont au pas **2,50 mm** ; ne pas commander des boîtiers Dupont2,54 mm
comme substituts. Les fils de 300 mm réservent du mou de service et seront
disposés sans traction dans le boîtier. Leur routage câblé réel reste à essayer.

Source : [JST XH, contacts et boîtiers](https://order.jst-mfg.com/InternetShop/app/pdf_show.php?kbn=1&key=eXH.pdf).

## Plan économique courant — revue du9novembre2026

L’utilisateur accepte les soudures à faire réaliser au jack et à l’encodeur,
mais aucun sertissage manuel. Voir le [dossier économique](economy_cabling/README.md)
et la [BOM système](system_bom_review.csv). Le contrat électrique A/B300mm,
AWG22pourPOWER/AWG26signaux et brochage droit du tableau initial est conservé.

Sur-mesure limité àH01/H02/H03,10pièces chacun, et un U04 commun de30pièces
(40au total avecTMP). Aucun câbleXH4versfemelles individuelles exact n’a été
qualifié chezAmazon/GoTronic ; il reste donc la seule quatrième référence
spéciale. Finir les fils avec leurs contacts et boîtiers chez le câbleur.
Pas de gaine spéciale ou de sous-ensembles soudés sur-mesure à faire produire.

| Liaison | Solution retenue pour le devis réduit | Intervention restante |
| --- | --- | --- |
| A/J1,B/J1 | 10H01 XH2-XH2,300mmAWG22 | Enfichage et contrôle de continuité |
| A/J6,B/J6 | 10H02 XH3-XH3,300mmAWG26 | Enfichage, aucun croisement |
| A/J7,B/J7 | 10H03 XH6-XH6,300mmAWG26 | Les6fils sont présents ;5/6NCsurB |
| A/J2 LCD | 10U04 numérotés1..4 vers femelles individuelles | 1GND,2VCC3,3V,3SDA,4SCL ; barrette mâle déjà soudée requise |
| A/J5 BH1750 | 10U04 identiques | Mêmes fonctions queLCD ; ADDRbas0x23, module réel à vérifier |
| A/J3 encodeur | 10U04 + [ADA377GoTronic38383](https://www.gotronic.fr/art-encodeur-24-impulsions-ada377-37481.htm) | Couper uniquement les4femelles côté module, dénuder, souder et isoler ;1A,2B,3SW1,4GNDversC+SW2 pontés |
| A/J4 TMP117, option | 10U04 supplémentaires + [QW-4C49152](https://www.gotronic.fr/art-cordon-qwiic-stemma-qw-4c-30786.htm) | Cordons finis joints par fonctionsGND/VIN/SDA/SCL, jonction retenue ; environ450mm cumulés, I2C/mou à essayer ; aucun câbleSH spécial |
| B/J4 vers jack arrière | [JST1164GoTronic49197](https://www.gotronic.fr/art-cordon-jst-xh-femelle-2-cts-15-cm-jst1164-33095.htm),15cm conditionnels | 2soudures :1+3V3_B->tip,2NTC_RAW->sleeve ;ringisolé, sleevepasGND ; vérifier mou de dépose avant achat |
| Sonde extérieure | Candidat catalogue15k/courbe exacte à trouver | Gaîne extérieure robuste continue et serre-câble ;2soudures sur fiche si nécessaire, pas de sertissage |
| Puissance | Câble catalogue gainé2x0,75mm2, référence à préciser | CôtéPhoenix :7mm dénudés,0,5..0,6Nm, pas d’embout obligatoire ; autres terminaisons à qualifier |
| Protection/USB | Choix final différé, câbleUSBde données existant au banc | Protection indépendante non supprimée ; réservationUSB latérale16mm inchangée, aucun passe-paroi précis déclaré compatible |

**U04 est numérique, pas fonctionnel :** cavité1versprise1,2vers2,3vers3,
4vers4. SurLCD/BH/TMP,1estGND ; surencodeur,4estGND. Ne pas suivre une couleur
ou un repèreGND commun aux quatre usages. Étiqueter les fonctions après
affectation ; aucune inversion sur les PCB. Les femelles individuelles sont
proposées pour broches carrées0,64mm/2,54mm, engagement à vérifier sur modules.

### Protection et soudures

Chaque fil interne garde son isolant. Une tresse/spirale légère posée à
l’intégration est possible ; la deuxième gaine commune isolante de tous
les faisceaux internes n’est plus une opération industrielle obligatoire.
Pour la liaison interne de sonde, [GAIT06GoTronic48620](https://www.gotronic.fr/art-gaine-tressee-gait06-15337.htm)
protège de l’abrasion (polyester3..9mm), sans remplacer l’isolant des fils.
Isoler chaque soudure et retenir la tresse ; vérifier le passage du boîtier
XH et le mou des15cm. Si le trajet échoue, demander30cm avant le lot.

Le cordon de sonde **extérieur** garde une gaine commune robuste continue,
une classe thermique adaptée à l’expérience et un vrai maintien à la sonde
et au serre-câble de fiche. Une paire isolée ou une tresse seule ne suffit
pas à l’extérieur. NTC15k/B≈3920 à courbe documentée, pas10k/DS18B20 par défaut.

Prévoir5points de soudure de terminaux encodeur (avec pontC-SW2) et2aujack
par appareil :70points pour10appareils, sans compter les fiches de sondes
externes. L’utilisateur accepte que ce travail soit réalisé, ce qui ne veut
pas dire qu’une main-d’œuvre payante est gratuite. Contrôler continuité,
absence de court-circuit, tenue des raccords et fonctionnement du poussoir.

Les cinq anciennes lignes de devis externes H09-H13 sont différées ; elles
ne sont pas transformées en achats catalogue déjà qualifiés. Aucun achat
ou envoi au câbleur n’est effectué.

## Puissance et sonde sur B

| Connecteur | Pin1 | Pin2 |
| --- | --- | --- |
| J2, entrée | +24 V / VIN_RAW | PGND / retour alimentation |
| J3, chauffage | VIN_PROTECTED, positif permanent | HEATER_LOW, retour commandé par Q1 |
| J4, sonde | +3V3_B | NTC_RAW |

Le chauffage résistif 24 V / 100 W appelle environ4,17 A. Prévoir des fils
souples **0,75 mm²**, isolés pour la température de l’installation.
Le fabricant autorise le fil souple directement dans le bornier : les embouts
ne sont pas obligatoires côté PCB. S’ils sont utilisés, les faire fournir
déjà sertis et compatibles. Le choix du câble extérieur et du presse-étoupe dépend
de sa gaine et reste dans la BOM système, hors assemblage des PCB. Pour les
borniers1715721 : dénudage7 mm, serrage0,5–0,6 Nm en soutenant leur corps.
Vérifier l'adaptation de l'embout à la borne ; ne pas étamer les extrémités
souples sous une vis. La puissance de chauffage ne passe pas par les JST A/B.

Source : [Phoenix1715721, raccordement](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721).

J4 reçoit la sonde **NTC passive15k/B≈3920** via la prise arrière à deux
contacts. La NTC elle-même n'est pas polarisée ; conserver les repères dans
le faisceau de panneau pour diagnostiquer les fils. Ce connecteur ne reçoit
ni24 V, ni5 V, ni sonde numérique. Le brochage de la prise de panneau dépend
de sa référence finale ; il ne change pas le PCB ou ce contrat J4.

## Périphériques et service

A/J2 LCD, A/J4 TMP117, A/J5 BH1750 : **1GND, 2+3V3_A, 3SDA, 4SCL**,
boîtier XHP-4 côté A. A/J3 encodeur : **1A, 2B, 3bouton, 4GND**. L'autre
extrémité est à adapter aux modules existants, sans présumer leur brochage.
Ces modules ne sont pas à commander comme composants de la BOM PCB.

Déconnecter les faisceaux de B avant extraction de son berceau. JP1 est livré
ouvert, cavalier séparé. Pour le premier flash USB : couper24 V et toute source
5 V, débrancherJ1/J6/J7, retirerJP1. Pour fonctionner sur B, débrancher l'USB,
raccorder les trois faisceaux hors tension puis placerJP1.

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
