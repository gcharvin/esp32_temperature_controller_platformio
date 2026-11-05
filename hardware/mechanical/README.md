# Maquette 3D v0.5 : DevKit diymore et XL4015 séparé

Révision du **5 novembre 2026**. Boîtier nu : **155 × 100 × 115 mm** (largeur × hauteur × profondeur), paroi nominale 3 mm. La largeur augmente de 10 mm pour créer une baie indépendante pour le buck à côté de B. Les PCB conservent leurs dimensions : A 110 × 75 × 1,6 mm ; B 90 × 65 × 1,6 mm. Seule la position de B dans le boîtier est décalée de 10 mm vers la droite.

## Fichiers pour Fusion 360

- `exports/enclosure_v05_xl4015_diymore.step` : **assemblage complet à ouvrir**, avec électronique et passe-paroi de référence.
- `exports/enclosure_v05_printed_parts.step` : uniquement les **cinq pièces imprimées**, chacune constituée d'un solide connecté.
- `exports/enclosure_exploded.step` et `enclosure_printed_parts_exploded.step` : vue éclatée ; façade avancée de 85 mm, B reculé de 50 mm, buck de 60 mm, panneau arrière de 85 mm.
- `exports/parts_v05/` : STEP et STL individuels des cinq pièces.
- `exports/preview_service.png`, `preview_buck.png`, `preview_exploded.png` : arrière ouvert, détail du berceau XL4015 et vue éclatée.
- `exports/validation.json` et `assembly_sequence_validation.json` : contrôles des solides, collisions et trajectoires.
- `exports/enclosure_v05_xl4015_diymore_fusion_bundle.zip` : livraison regroupée avec sources et contrôles.

Les fichiers v01 à v04 sont historiques. Les exports sans numéro correspondent à la révision courante. Le STEP fournit une arborescence de solides **sans historique paramétrique natif Fusion**. Les sources paramétriques sont `build_enclosure.py`, `enclosure_parameters.json` et `../placement.json`. Aucun `.f3d` n'est fourni. L'export est réimporté par OpenCascade ; son ouverture dans Fusion n'a pas été exécutée pour cette révision.

## Cinq pièces imprimées

| Pièce | Fonction |
| --- | --- |
| 01 Coque ouverte | Guides séparés de B et du buck, butées avant, prise USB latérale |
| 02 Façade équipée | Supports du LCD et de A intégrés, pose des cartes hors coque |
| 03 Berceau B | Rainure basse, guides et deux clips ; extraction par l'arrière |
| 04 Panneau arrière | Deux presse-étoupes, prise sonde, clips de fermeture, butées derrière les deux berceaux |
| 05 Berceau XL4015 | Rainure et deux clips propres au buck ; coulissement dans la baie gauche et poignée arrière |

Les guides de coque portent les berceaux en haut et en bas ; une joue sépare leurs baies. Les butées avant et celles du panneau arrière limitent le déplacement longitudinal. Jeu nominal de guidage : 0,25 mm ; jeu aux butées arrière : 0,3 mm. Ces jeux restent à essayer avec l'imprimante retenue.

La rainure de chaque support prend le bord inférieur du PCB ; les crochets retiennent les bords latéraux. Le guide supérieur reste ouvert à l'arrière pour permettre une pose inclinée puis un rabattement. Le LCD a son propre support et ne porte pas A.

LCD/A/B : prise basse 3 mm, appuis latéraux 1,5 mm. Buck : **prise basse réduite à 1,5 mm**, avec une réserve électronique supposée de 2 mm au bord. La compatibilité des clips avec les pistes et composants du buck doit être vérifiée sur la carte réelle ; aucun serrage sur un composant n'est prévu.

PCB nominal 1,6 mm, jeu total de rainure 0,6 mm. Bras de clips : longueur 16 mm, épaisseur 1,2 mm ; crochet : 1,8 mm sur le bord. Les dégagements représentés (2,4 mm pour les clips PCB, 1,2 mm pour la façade, 1,5 mm pour le panneau arrière) sont des enveloppes de libération, sans qualification élastique.

## Modules et réservations

Coordonnées : x de gauche à droite en vue de face ; y de l'avant vers l'arrière ; z du bas vers le haut, en mm. Depuis l'arrière, gauche et droite sont inversées.

| Élément | Position et hypothèses |
| --- | --- |
| A | Origine inférieure x=8/z=13 ; face composants y=45 |
| B | Origine inférieure x=50/z=17 ; face composants y=85 |
| DevKit diymore CH340, ASIN B0DGLCWR76 | Corps vendeur 54 × 27 mm ; antenne à gauche, USB-C à droite ; deux supports femelles 1×15 ; entraxe **25,40 mm supposé**, hauteur des supports 8,5 mm supposée |
| XL4015 CC/CV, ASIN B0C6R3FMT3 | PCB tourné dans son plan : 26,3 mm en x, 51 mm en z ; origine x=8/z=22 ; face composants y=83 ; profondeur totale de référence 14 mm |
| LCD | PCB de référence 98 × 60 mm ; face arrière y=15,1 ; backpack jusqu'à y=28 |

La DevKit reste enfichable sur A. Le XL4015 reste hors PCB B : quatre fils relient son entrée au bornier B/J8 (IN+, IN−/PGND) et sa sortie au JST B/J9 (OUT−/GND, OUT+/5 V). Respecter le brochage électrique et régler la sortie à 5,00 V avant raccordement. Les détails du buck, borniers et trimmers sont des volumes d'étude, pas une reproduction certifiée.

Les enveloppes de B sont relues depuis l'implantation routée, notamment Q1 TO-220 (17 mm de haut) et C8 (11,5 mm). Aucun dissipateur supplémentaire n'est représenté. L'encombrement n'atteste pas la tenue thermique en boîtier fermé à 24 V / 100 W.

## Montage

1. **Façade hors coque : poser le LCD avant A et l'encodeur.** Le faire passer dans le cadre A vide incliné de 4°, avec décalage x=+4 mm/z=−1 mm. Après passage, recentrer de 4 mm, relever de 4,5 mm, approcher du logement, abaisser de 3,5 mm dans la rainure et rabattre jusqu'aux clips. Ne pas charger le verre ou le backpack.
2. Monter l'encodeur puis A : bord inférieur relevé de 3,5 mm, inclinaison 4°, approche, abaissement dans la rainure puis rabattement et enclenchement. Préparer les faisceaux de façade. La retenue réelle de l'encodeur dépend de sa référence.
3. Sur l'établi, charger B et le XL4015 dans leurs berceaux respectifs, par le même principe bord inférieur puis pivot. Vérifier les marges du buck avant enclenchement. Installer l'USB de panneau dans la coque, fiche interne débranchée et câble hors passage.
4. Introduire la façade équipée par l'avant. Sa lèvre la centre ; ses deux languettes la verrouillent dans les fenêtres avant.
5. Depuis l'arrière ouvert, **avant insertion de B**, brancher la rallonge USB dans la DevKit ; course latérale de 15 mm vérifiée. Ranger son câble hors des guides.
6. Introduire B et le buck par l'arrière jusqu'aux butées avant. Les berceaux peuvent être insérés/retirés indépendamment. Raccorder J8/J9 du buck, les trois JST A/B, la puissance, le chauffage et la sonde, hors tension.
7. Présenter le panneau arrière équipé, ranger les fils avec leur mou de service et enclencher les clips. Les presse-étoupes retiennent les câbles sans les déconnecter électriquement. Les distances de la vue éclatée ne garantissent aucun mou de câble.

## Démontage et accès

1. Couper et débrancher l'alimentation. Libérer le panneau arrière par ses fenêtres latérales, le reculer selon le mou réel des fils puis déconnecter les terminaisons nécessaires pour le déposer.
2. **B seule :** débrancher tous ses faisceaux, y compris J8/J9 allant au buck ; tirer le berceau de 50 mm vers l'arrière. Le buck, la façade et l'USB restent en place. Libérer les clips du PCB sur l'établi et inverser sa pose.
3. **Buck seul :** déconnecter ses quatre fils ou leurs extrémités côté B et dégager le câblage ; tirer sa poignée de 60 mm vers l'arrière. B peut rester dans son berceau. Déclipser le module hors coque.
4. **Façade :** retirer B pour accéder plus facilement à l'USB, débrancher la fiche côté DevKit et les fils reliant la façade au reste du boîtier. Libérer les clips avant et extraire la façade équipée ; course testée 100 mm. Le buck peut rester en place.
5. Sur l'établi, déposer A puis l'encodeur si nécessaire, enfin le LCD par la trajectoire inverse dans le cadre A vide.

Panneau retiré, les quatre vis de borniers et deux trimmers du buck ont des corridors réservés pour une tige de tournevis Ø6 mm. Leurs positions sont **approximatives**. B dispose de corridors Ø6 mm au-dessus de J2/J3/J8 et de corridors axiaux de 35 mm au-dessus des JST J1/J4/J6/J7/J9. Ces contrôles concernent l'espace libre au-dessus des enveloppes ; ils ne décrivent ni l'engagement réel de l'outil ni les entrées latérales de fils. L'accès des doigts et les faisceaux doivent être essayés physiquement.

## USB et passe-paroi

USB-C femelle de panneau à droite : centre y=64/z=30, trou Ø16, corps Ø15, collerette Ø22, empiètement intérieur 28 mm, dépassement extérieur 4 mm. Rallonge **de données** vers le CH340 de la DevKit. Fiche interne réservée : **14 × 4 × 10 mm**, relativement plate ; vérifier sa compatibilité réelle avec JP1 et le port diymore. Le port, la fiche et leur position sont des enveloppes non mesurées ; le câble reste indicatif, sans rayon de courbure qualifié.

L'accès USB est mécanique. La sélection/isolation entre 5 V externe et VBUS reste à qualifier ; JP1 est un cavalier de service, pas un multiplexeur automatique. La procédure de flash USB seul et les limites du monitoring sous 24 V sont décrites dans `../README.md`.

Le panneau arrière conserve le choix utilisateur : **deux presse-étoupes et une prise sonde à deux contacts**.

| Fonction | Réservation | Trou | Centre x / z |
| --- | --- | --- | --- |
| Entrée 24 V | Presse-étoupe type M12, corps Ø20 | Ø12,5 | 22 / 75 |
| Chauffage | Presse-étoupe type M12, corps Ø20 | Ø12,5 | 22 / 35 |
| Sonde NTC | Prise type GX12-2, corps Ø17 | Ø12,4 | 108 / 85 |

Ces cotes sont des réservations ajustables ; choisir les références finales avec leur plage de câble et longueur de filetage pour une paroi de 3 mm avant de figer les perçages. Filetages, écrous et contacts sont simplifiés.

## Contrôles et régénération

Le générateur contrôle les solides, leur connexion, les racines de clips, les collisions statiques et **22 étapes de montage/service** : mise en place des quatre PCB (LCD, A, B, buck), insertion de façade, branchement USB, extraction indépendante des deux berceaux, retrait du panneau et corridors d'accès. Translations par volumes balayés ; pivots échantillonnés tous les 0,1° de 4° à 0°. Les listes `collisions_mm3` et `failures` des rapports doivent être vides.

Les STEP sont réimportés ; les cinq pièces imprimées doivent donner cinq solides valides avec conservation du volume. Les mouvements sont réversibles dans le même état de déconnexion et de libération des clips.

```powershell
uv run hardware/mechanical/build_enclosure.py
```

CadQuery 2.6.1 et Pillow sont déclarés dans le script. Celui-ci régénère STEP, STL, GLB, rendus et rapports, sans modifier KiCad. Le ZIP est un instantané à recréer après modification.

Les contrôles portent sur les enveloppes déclarées. Ils ne simulent ni la flexion des clips, ni les efforts, les tolérances d'impression, les fils, les doigts, la fatigue ou la dissipation. Mesurer DevKit, supports, fiche USB, bords du buck et LCD ; essayer un coupon de rainure/clip et un montage câblé avant l'impression définitive.
