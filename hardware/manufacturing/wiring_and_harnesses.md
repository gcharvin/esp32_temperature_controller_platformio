# Câblage du prototype A/B

Les numéros ci-dessous sont ceux des **pastilles PCB**. Ne pas les déduire
du sens de lecture des fils sur une photographie : la vue côté entrée de
câble est inversée par rapport à la face d'accouplement. Faire un contrôle
de continuité broche à broche avant branchement. Tous les raccordements et
changements de carte métier se font hors tension.

## Faisceaux A/B à faire sertir

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
combinaison fil/contact par le câbleur. Il faut 4 contacts pour POWER, 6 pour
I2C et 12 pour CTRL, soit **22 contacts** et six boîtiers par jeu A/B.
Les XH sont au pas **2,50 mm** ; ne pas commander des boîtiers Dupont2,54 mm
comme substituts. Les fils de 300 mm réservent du mou de service et seront
disposés sans traction dans le boîtier. Leur routage câblé réel reste à essayer.

Source : [JST XH, contacts et boîtiers](https://order.jst-mfg.com/InternetShop/app/pdf_show.php?kbn=1&key=eXH.pdf).

## Puissance et sonde sur B

| Connecteur | Pin1 | Pin2 |
| --- | --- | --- |
| J2, entrée | +24 V / VIN_RAW | PGND / retour alimentation |
| J3, chauffage | VIN_PROTECTED, positif permanent | HEATER_LOW, retour commandé par Q1 |
| J4, sonde | +3V3_B | NTC_RAW |

Le chauffage résistif 24 V / 100 W appelle environ4,17 A. Prévoir des fils
souples **0,75 mm²**, isolés pour la température de l'installation, avec
embouts compatibles. Le choix du câble extérieur et du presse-étoupe dépend
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
