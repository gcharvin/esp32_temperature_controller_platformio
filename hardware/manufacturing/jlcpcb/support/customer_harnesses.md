# Câblage du prototype A/B

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
combinaison fil/contact par le câbleur. Il faut 4 contacts pour POWER, 6 pour
I2C et 12 pour CTRL, soit **22 contacts** et six boîtiers par jeu A/B.
Les XH sont au pas **2,50 mm** ; ne pas commander des boîtiers Dupont2,54 mm
comme substituts. Les fils de 300 mm réservent du mou de service et seront
disposés sans traction dans le boîtier. Leur routage câblé réel reste à essayer.

Source : [JST XH, contacts et boîtiers](https://order.jst-mfg.com/InternetShop/app/pdf_show.php?kbn=1&key=eXH.pdf).

## Liste explicite des câbles prêts à brancher — revue du 9 novembre 2026

Périmètre retenu : **Amazon.fr + GoTronic**, compte GoTronic accepté par
l’utilisateur. Aucun achat ni compte créé pendant cette revue. Le
[guide d’achat regroupé](../README.md) et la
[BOM système](customer_purchase_list.csv) distinguent article identifié et liaison
complète. Les anciennes offres hors de ces deux canaux ne sont plus proposées
comme panier. Aucun faisceau commercial n’a été essayé sur les cartes réelles.

**Aucun sertissage à faire soi-même** reste l’objectif. « Préserti » peut
désigner un cordon fini d’un seul côté. Deux soudures, une insertion manuelle
de contacts déjà sertis et un câble directement enfichable sont des états
différents, indiqués ci-dessous. Toutes les longueurs doivent garder le mou
de démontage ; les couleurs ne prouvent pas le brochage.

| Liaison, quantité par appareil | Les deux extrémités nécessaires | Référence dans le panier retenu | Ce qui reste à trouver ou faire |
| --- | --- | --- | --- |
| A/J1 ↔ B/J1, 1 | XHP-2 femelle ↔ XHP-2 femelle, 300±10mm, AWG22 | **À trouver** chez Amazon.fr/GoTronic | Aucun câble complet qualifié. Demander câblage droit et +5V repéré. Un kit nu ne répond pas au besoin |
| A/J6 ↔ B/J6, 1 | XHP-3 femelle ↔ XHP-3 femelle, 300±10mm, AWG26 | **À trouver** chez Amazon.fr/GoTronic | Aucun câble complet qualifié ; droit1→1 à3→3. Rallonge mâle-femelle non adaptée |
| A/J7 ↔ B/J7, 1 | XHP-6 femelle ↔ XHP-6 femelle, 300±10mm, AWG26 | **À trouver** chez Amazon.fr/GoTronic | Droit1→1 à6→6. Positions5/6 présentes mais NC sur B. Un SH6 ne convient pas |
| A/J2 ↔ LCD Freenove, 1 | XHP-4 femelle ↔ quatre Dupont femelles | **À trouver**, longueur à qualifier avec mou de service | Barrettes mâles du backpack déjà soudées requises. Raccorder GND/3V3/SDA/SCL selon le module |
| A/J3 ↔ encodeur, 1 | XHP-4 femelle ↔ A/B/SW/GND | [GoTronic ADA377/code38383](https://www.gotronic.fr/art-encodeur-24-impulsions-ada377-37481.htm) pour le composant, **pas le cordon** | Aucun faisceau complet qualifié : trois contacts rotatifs et deux bouton, mettre en commun les retours. Soudures probables ; moduleKY-040 non substitué |
| A/J4 ↔ TMP117, 1 optionnel | XHP-4 femelle ↔ SH1mm4contacts du module | [GoTronic QW-4C/code49152](https://www.gotronic.fr/art-cordon-qwiic-stemma-qw-4c-30786.htm) : SH vers quatre broches mâles,15cm, cordon fini | **Il manque le XH4 vers quatre Dupont femelles** pour joindre ce cordon à A. Immobiliser la jonction ; GND/VIN/SDA/SCL à contrôler. Aucun SH directement dans XH |
| A/J5 ↔ BH1750 GY-302, 1 | XHP-4 femelle ↔ quatre Dupont femelles | **À trouver** chez Amazon.fr/GoTronic | Barrette mâle du module déjà soudée requise ; ADDR bas pour0x23. ADA4681 GoTronic plus grand, non substitué au GY-302 |
| B/J4 ↔ embase jack arrière, 1 | XHP-2 femelle ↔ tip/sleeve du PJ-392 | [GoTronic JST1164/code49197](https://www.gotronic.fr/art-cordon-jst-xh-femelle-2-cts-15-cm-jst1164-33095.htm) : XH2 femelle2,50mm vers fils dénudés,**15cm** | **Partiel**, aucune pince mais deux soudures au jack. Section non publiée ; vérifier longueur et mou, ne pas remplacer automatiquement le30cm. J4.1(+3V3_B)→tip, J4.2(NTC_RAW)→sleeve, ring isolé ; sleeve pas GND |
| Sonde externe ↔ prise arrière, 1 | NTC15k/B≈3920 ↔ ficheTRS3,5mm | [Lot DAOKI B08XM6DR96](https://www.amazon.fr/dp/B08XM6DR96) candidat pour embase+fiche ; sonde exacte non retenue | **Sonde avec fiche déjà montée à trouver** ; sinon deux soudures sur fiche, ring isolé. Fiche audio ne veut pas dire sonde adaptée. Insertion/retrait hors tension |
| Alimentation centrale ↔ B/J2, 1 | Sortie de l’alimentation ↔ bornierPhoenix, via presse-étoupe | **À trouver** : cordon cuivre0,75mm² avec embouts déjà sertis côté PCB | Choix12/24V, puissance et sortie alimentation non figés. Gaine3..6,5mm pour presse-étoupe ; longueur, calibre et autre terminaison à définir |
| B/J3 ↔ chauffage, 1 | BornierPhoenix ↔ chauffage, via presse-étoupe | **À trouver** : cordon cuivre0,75mm² avec embouts montés côté PCB | Température d’isolation et terminaison chauffage à définir. Ne pas acheter câble nu + embouts nus si on veut éviter la pince |
| Coupure thermique ↔ chauffage | Contacts adaptés ↔ circuit chauffage | **À définir** avec protection retenue | Seuil et pouvoir de coupure DC à choisir avant de figer le cordon équipé |
| USB latéral ↔ DevKit, 1 | USB-C femelle de panneau ↔ USB-C mâle, données | **À trouver sur plan** chez Amazon.fr/GoTronic | Trou actuel16mm ; beaucoup de modèles M20/M22 incompatibles. Aucun passe-paroi précis qualifié pour cette ouverture |
| Ordinateur ↔ USB latéral, 1 câble de service | USB du PC ↔ USB-C mâle | Réutiliser câble de données existant ; recherche Amazon.fr si absent | Premier service USB seul selon procédure. Option PortaPowPPUSBBP1 ultérieure, pas nécessaire au diagnosticWi-Fi et non qualifiée avec chaqueDevKit |

**Bilan :** trois faisceaux A/B et les quatre liaisons périphériques (dont
TMP117 optionnel) ne sont pas encore des références complètes achetables
dans ce panier. Deux cordons partiels sont identifiés chez GoTronic : QW-4C
pour TMP117 et JST1164 pour la sonde interne. Le second nécessite des soudures
et un essai du mou ; le premier nécessite encore l’adaptateur XH4. Restent
aussi à figer la sonde à fiche montée, les deux cordons de puissance équipés,
la protection et la rallonge USB de panneau.

Un assortiment de boîtiers XH avec contacts nus demande une pince. Des fils
sertis aux deux bouts puis insérés à la main pourraient convenir, mais aucun
kit documenté couvrant les2/3/6voies, les sections22/26AWG et300mm n’est
actuellement retenu chez Amazon.fr/GoTronic. Ne pas remplacer les XH2,50mm par
Dupont2,54mm ni changer le contrat pour s’adapter à un câble non qualifié.
Contrôler la continuité broche à broche avant alimentation et essayer le
démontage câblé réel. Sans pince ne signifie pas sans soudure.

## Câble de sonde gainé et robuste — exigence d’achat

Pour le cordon de sonde **extérieur**, exiger deux conducteurs multibrins
sous une **gaine extérieure commune continue**, souple et résistante aux
manipulations. Deux fils isolés séparés ne constituent pas le cordon demandé.
La gaine et le raccord côté sonde doivent être spécifiés pour la température
réelle de l’expérience ; cette plage et la longueur restent à définir avec
la sonde retenue. Préférer sonde, câble gainé et fiche TRS montés en usine.

Prévoir un maintien anti-arrachement côté sonde et un serre-câble dans la
fiche jack, compatible avec le diamètre extérieur du câble. La gaine commune
doit aller jusque dans cette retenue ; isoler chaque soudure séparément.
Une gaine thermorétractable sur des soudures ne remplace pas cette retenue.
La courbe électrique reste NTC15k/B≈3920 : la robustesse mécanique ne qualifie
pas une sonde d’une autre valeur. Aucun ensemble exact n’est encore retenu.

Pour le trajet **interne B/J4 ↔ jack arrière**, le JST1164 GoTronic reste
un cordon partiel à deux fils isolés : aucune gaine commune n’est documentée.
Ajouter une gaine isolante de protection souple, retenue mécaniquement, et
isoler séparément les deux soudures au jack. Prévoir une longueur de protection
par appareil dans le panier Amazon/GoTronic, référence et diamètre à préciser
avec le cordon. Vérifier que les15cm gardent le mou nécessaire pour déposer
le panneau arrière sans tirer sur le XH ou les soudures. Aucun cordon complet
gainé XH2↔PJ-392 n’est actuellement qualifié.

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
