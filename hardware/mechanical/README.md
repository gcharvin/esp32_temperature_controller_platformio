# Maquette 3D : façade amovible et berceau B

Révision **0.4 du 4 novembre 2026**. Boîtier nu : **145 × 100 × 115 mm** (largeur × hauteur × profondeur), hors bouton et prises. A : 110 × 75 mm ; B : 90 × 65 mm ; LCD de référence : 98 × 60 mm. Les enveloppes électroniques restent à mesurer sur les modules réels.

## Fichiers pour Fusion

- `exports/enclosure_v04_removable_front.step` : assemblage complet, électronique de référence comprise.
- `exports/enclosure_v04_printed_parts.step` : exactement **quatre pièces imprimées**, chacune constituée d'un solide connecté.
- `exports/enclosure_exploded.step` : façade équipée avancée de 85 mm, berceau B reculé de 50 mm, panneau arrière reculé de 85 mm. Ces distances servent à lire le modèle ; elles ne représentent pas le mou des câbles.
- `exports/parts_v04/` : STEP et STL séparés des quatre pièces.
- `exports/preview_exploded.png`, `preview_front_lcd.png`, `preview_front_module.png` : vues de l'assemblage et des étapes hors boîtier.
- `exports/assembly_sequence_validation.json` et `exports/validation.json` : résultats des contrôles géométriques.
- `exports/enclosure_v04_removable_front_fusion_bundle.zip` : livraison regroupée avec sources et contrôles.

Les fichiers v01/v02/v03 sont conservés comme anciennes révisions. Les exports sans numéro correspondent à la révision courante.

Le STEP contient des solides organisés, **sans historique paramétrique natif Fusion**. Les sources paramétriques sont `build_enclosure.py`, `enclosure_parameters.json` et `../placement.json`. Aucun `.f3d` ni script de reconstruction de timeline Fusion n'est fourni. L'export est réimporté par OpenCascade ; son ouverture dans Fusion n'a pas été exécutée pour cette révision.

## Quatre pièces mécaniques

| Pièce imprimée | Fonction et sens de dépose |
| --- | --- |
| Coque ouverte aux deux extrémités | Porte les guides de B et la prise USB latérale ; aucun support LCD/A fixé au fond |
| Façade avec supports LCD et A intégrés | Porte les deux modules indépendamment, puis entre par l'avant ; deux clips latéraux accessibles depuis les fenêtres de la coque |
| Berceau B avec rainure et clips intégrés | Porte B, coulisse dans les guides et s'extrait par l'arrière avec ses deux poignées |
| Panneau arrière avec clips intégrés | Porte les deux presse-étoupes et la prise sonde ; sa butée limite le recul de B |

Le LCD n'est donc plus à installer au fond d'une coque fermée. On équipe la façade sur l'établi avant de la présenter dans la coque. La traverse supérieure du cadre A est ouverte pour permettre le passage initial du LCD dans le support A encore vide.

Les PCB sont verticaux et parallèles à la façade. Les faces composants de A et B sont à 45 et 85 mm de la face avant extérieure. Leurs composants et JST sont orientés vers l'arrière. Le LCD a sa propre rainure et ses propres clips : il ne porte pas A.

## Montage à respecter

1. **Façade seule, hors coque : poser le LCD avant A et l'encodeur.** Le présenter depuis l'arrière à travers le cadre A vide, incliné de 4° : décalage latéral de +4 mm et vertical de −1 mm. Après passage du cadre A, le recentrer et le relever de 4,5 mm pour franchir la lèvre basse du support LCD. Approcher de son logement, l'abaisser de 3,5 mm dans la rainure, puis le rabattre contre les appuis. Relâcher les deux clips latéraux derrière le PCB. Ne pas charger le verre ou le backpack.
2. Monter l'encodeur sur la façade et le raccorder. **Poser A ensuite**, avec le même principe : approche inclinée de 4°, bord inférieur initialement relevé de 3,5 mm, abaissement dans la rainure puis rabattement et enclenchement des deux clips. Raccorder LCD/encodeur et préparer les trois faisceaux A/B sur A. La retenue réelle de l'encodeur dépend de sa référence, non figée dans cette maquette.
3. Clipser B dans son berceau **hors boîtier**, par insertion inclinée du bord inférieur puis rabattement. Installer la prise USB de panneau dans la coque, fiche interne laissée débranchée et câble tenu à l'écart du passage.
4. Introduire la façade équipée **par l'avant**, dans l'axe de profondeur. Sa lèvre centre la pièce ; ses deux languettes latérales la verrouillent dans les fenêtres avant de la coque. Le contrôle géométrique représente les crochets repoussés vers l'intérieur pendant le mouvement.
5. Depuis l'arrière encore ouvert et sans B, enficher la rallonge USB dans la DevKit ; course latérale de 15 mm réservée dans le modèle. Ranger son câble pour laisser le passage de B libre.
6. Introduire le berceau B par l'arrière dans ses guides, jusqu'aux butées avant. Raccorder les trois JST A/B, l'alimentation, le chauffage et la sonde, hors tension.
7. Présenter le panneau arrière équipé de ses passe-paroi, ranger les câbles avec leur mou de service, puis enclencher les clips du panneau. Les presse-étoupes retiennent les câbles ; ils ne les déconnectent pas électriquement.

Les décalages ci-dessus décrivent la trajectoire testée, pas un gabarit d'assemblage industriel. L'accès manuel aux clips devra être essayé sur une impression.

## Démontage et entretien

1. Couper et débrancher l'alimentation. Déverrouiller le panneau par les deux fenêtres latérales arrière. Le reculer prudemment selon le mou réel des câbles, puis déconnecter les terminaisons internes nécessaires pour le déposer complètement.
2. Débrancher de B tous ses faisceaux : trois JST A/B, puissance, chauffage et sonde. Tirer le berceau B de **50 mm vers l'arrière** pour le sortir entièrement de la coque. A et l'écran restent en place. Pour séparer B du berceau, écarter ses deux clips et inverser le mouvement de pose.
3. Pour déposer la façade : débrancher la fiche USB **côté DevKit** et dégager tous les fils qui relient la façade à la coque ou à B. La prise USB de panneau reste sur le côté de la coque.
4. Repousser vers l'intérieur les deux clips avant depuis les fenêtres latérales, puis tirer la façade équipée vers l'avant. La maquette vérifie une course de 100 mm sans câble attaché.
5. Façade sur l'établi : déclipser et retirer A en inversant la pose, puis retirer l'encodeur si nécessaire avant de déposer le LCD par le trajet inverse dans le cadre A vide. Aucun PCB n'est forcé au travers d'une paroi fermée.

## Rainures, clips et jeux

Chaque support comporte une rainure inférieure continue en C prenant 3 mm du bord du PCB, des appuis frontaux, des guides et deux clips latéraux libérables vers l'extérieur. Le guide supérieur est ouvert à l'arrière pour permettre le pivotement. Les crochets chevauchent les bords du PCB de 1,8 mm. Bras de clips : longueur 16 mm, épaisseur 1,2 mm ; épaisseur PCB nominale : 1,6 mm ; jeu total de rainure : **0,6 mm**, soit 0,3 mm de chaque côté au repos.

Les déplacements de libération employés par le contrôle sont de 2,4 mm vers l'extérieur pour les clips de PCB, 1,2 mm vers l'intérieur pour les clips de façade et 1,5 mm pour ceux du panneau arrière. Ce sont des enveloppes de dégagement, pas des flèches élastiques qualifiées.

Les lèvres latérales d'appui portent sur 1,5 mm de bord de PCB. Avant routage, compléter les exclusions KiCad latérales sur ces zones de contact ; les réserves actuelles haut/bas et les deux zones locales des crochets ne décrivent pas à elles seules tout cet appui continu. Les contacts doivent également être confrontés au PCB réel du LCD.

## Accès USB latéral pour flash et monitoring

La face arrière regroupe l'expérimentation (sonde, 12 V et chauffage). Une prise **USB-C femelle de panneau sur le côté droit** est dédiée à l'électronique. Elle se raccorde au port USB-C de la DevKit ELEGOO par une rallonge USB de données. On conserve ainsi le convertisseur CP2102 et le dispositif de programmation de la DevKit ; aucun second convertisseur USB-UART n'est ajouté sur A.

| Réservation mécanique | Proposition de la maquette |
| --- | --- |
| Position de la prise | Paroi droite, profondeur y = 64 mm depuis la face avant, hauteur z = 30 mm depuis le bas |
| Perçage rond | Ø16 mm |
| Corps intérieur réservé | Ø15 mm, prolongé par une sortie de câble, jusqu'à 28 mm à l'intérieur |
| Collerette / écrou simplifiés | Ø22 mm |
| Dépassement extérieur | 4 mm hors câble PC |
| Fiche interne DevKit | Volume provisoire 14 × 6 × 6 mm ; position du vrai port à mesurer |

Le port et le câble restent fixés à la coque quand on extrait B. La maquette vérifie le dégagement du corps de prise et de la fiche par rapport à la structure et aux volumes électroniques, ainsi que le retrait de B avec l'USB en place. Le câble est un tracé indicatif, sans qualification du rayon de courbure ni de l'encombrement d'une longueur excédentaire. Les cotes de la DevKit et de ses boutons BOOT/RESET restent à mesurer.

Une [rallonge USB-C ronde de panneau, comme l'exemple Adafruit 6069](https://www.adafruit.com/product/6069), illustre cette famille de montage à écrou ; le fabricant indique une ouverture de 12 à 18 mm et une compatibilité avec une paroi de quelques millimètres. Les dimensions de corps utilisées ici restent des réservations, pas une reproduction certifiée de cette référence. Retenir une rallonge qui transporte effectivement les données, vérifier les connexions USB-C nécessaires et tester l'énumération, le flash et le débit série avant de figer la pièce. Un câble de charge seul ne convient pas.

**L'accès est mécanique à ce stade ; la gestion de VBUS n'est pas résolue par une rallonge.** Les schémas gardent le cavalier JP1 et ne comportent pas encore de commutation automatique entre 5 V externe et 5 V USB. Les [DevKit Espressif documentent des alimentations exclusives](https://documentation.espressif.com/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html) ; il faut vérifier le circuit de la carte ELEGOO exacte. Ne pas présumer que la nouvelle prise autorise de brancher simultanément PC et 12 V.

Pour le prototype actuel, le flash alimenté uniquement par USB conserve la procédure JP1/faisceaux décrite dans le dossier électrique, qui peut encore demander une ouverture. Pour le monitoring en fonctionnement sans ouvrir, ajouter ou qualifier une sélection/isolation des sources compatible avec le CP2102 et la détection USB ; la seule coupure arbitraire du fil VBUS n'est pas tenue pour validée. Les masses et les lignes de données doivent rester traitées correctement. La mise au point électrique de ce mode est un point à fermer avant usage.

## Passages de paroi retenus

Choix utilisateur : **deux presse-étoupes** pour les câbles d'entrée 12 V et de chauffage, plus **une prise sonde à deux contacts**. Ils sont placés sur le panneau arrière, pas directement sur les PCB. Les JST-XH restent internes.

| Fonction | Réservation proposée | Perçage de l'étude | Centre x / z |
| --- | --- | --- | --- |
| Entrée 12 V | Presse-étoupe M12, corps/nut modélisé Ø20 mm | Ø12,5 mm | 22 / 75 mm |
| Chauffage | Presse-étoupe M12, corps/nut modélisé Ø20 mm | Ø12,5 mm | 22 / 35 mm |
| Sonde NTC | Prise circulaire 2 contacts, enveloppe type GX12-2 Ø17 mm | Ø12,4 mm | 108 / 85 mm |

Coordonnées : x depuis la gauche en vue de face du boîtier, z depuis le bas. **En regardant le panneau depuis l'extérieur à l'arrière, gauche et droite sont inversées** : sonde en haut à gauche, entrée 12 V en haut à droite, chauffage en bas à droite.

Les dimensions de ces pièces sont des **réservations réglables et non des références de perçage fabricant**. Les filetages, méplats, écrous et contacts sont simplifiés. Sélectionner les références exactes avec leur plage de serrage de câble et leur longueur de filetage pour une paroi de 3 mm, puis adapter les perçages avant impression du panneau final. Le connecteur sonde a été déplacé dans la maquette pour dégager le guide supérieur de B et la lèvre du panneau.

Le diamètre de câble représenté est de 5,5 mm pour puissance/chauffage et 3,5 mm pour la sonde. Il sert uniquement à visualiser les sorties ; le calibre électrique des conducteurs reste à dimensionner selon le courant.

## Vérifications réalisées et limites

Le générateur contrôle les solides, leur connexion et les intersections des quatre pièces imprimées avec les enveloppes de cartes, composants et prises en position assemblée. Il contrôle aussi la liaison des racines de clips et l'absence d'intersection de leurs parties libres avec les guides fixes.

La vérification de montage comprend **16 étapes géométriques** : passage du LCD dans le cadre A vide et alignement, mise en rainure et pivotement du LCD/A/B, insertion de la façade équipée, branchement de la fiche USB, extraction du berceau B et retrait du panneau arrière. Les translations sont contrôlées par volumes balayés construits à partir des faces ; les trois rotations sont échantillonnées tous les **0,1°**, de 4° à 0°. Les trajets sont réversibles dans le même état de déconnexion et de clips libérés. Les résultats détaillés sont enregistrés dans les JSON de validation, qui doivent avoir des listes de collisions/échecs vides.

Le contrôle modélise les bras/crochets dans une enveloppe libérée. Il ne simule ni leur flexion pendant l'enclenchement, ni les efforts, ni les doigts/outils, ni les câbles souples. Ceux-ci sont dessinés comme tracés indicatifs et sont exclus des trajets de démontage. Leur mou, leur rangement et leurs rayons de courbure doivent être testés physiquement.

Le STEP complet est réimporté : **69 solides valides**. Le STEP des pièces imprimées est réimporté : **4 solides**, volume conservé. Ces contrôles portent sur les réservations actuelles, sans qualifier les composants réels, la tenue thermique, la rigidité, la fatigue ou la qualité d'impression.

## Régénérer et imprimer une maquette

```powershell
uv run hardware/mechanical/build_enclosure.py
```

Le script déclare CadQuery 2.6.1 et Pillow ; il régénère STEP, STL, GLB, rapports et rendus sans modifier KiCad. Les sources restent la référence ; les retouches faites dans Fusion ne les mettent pas à jour.

Les STL sont destinés à une impression d'essai. Définir l'orientation et les supports d'impression, puis essayer un coupon de rainure/clip avec le matériau retenu avant d'imprimer tout le boîtier. Mesurer LCD, DevKit et buck, choisir les vrais passe-paroi et vérifier le montage avec les câbles réels avant de figer les dimensions.
