# Dépôt JLCPCB — devis et revue préalable, 5 A + 5 B assemblées

Dossier du **6 novembre 2026**, A v0.4 / B température v0.5, sans changement
des schémas ni des pistes. **Demande de devis uniquement ; aucune commande,
réservation payante de composants ou fabrication n'est autorisée.**

## Commencer par le support technique

La [FAQ PCBA de JLCPCB](https://jlcpcb.com/help/article/pcb-assembly-faqs-part-2)
indique que l'assemblage ne bénéficie pas du parcours « Review Before Payment »
habituel des PCB nus. Le devis affiché et l'aperçu automatique ne valent pas
revue technique complète. Pour obtenir une réponse **sans lancer de commande**,
demander d'abord une revue avant-vente manuelle ; sa disponibilité reste à
confirmer par le support. Ne pas payer pour déclencher une revue.

1. Extraire `JLCPCB_A04_B05_RFQ_DFM_ONLY.zip` dans un dossier local.
2. Ouvrir [le support JLCPCB](https://jlcpcb.com/help/contact) puis le chat,
   ou écrire à **support@jlcpcb.com**, adresse publiée sur
   [leur page d'assistance](https://jlcpcb.com/help/article/how-do-i-get-support).
3. Copier le texte de `request_support_EN.txt` et joindre l'archive complète.
   Si le chat ne reçoit pas les ZIP, demander le canal de transfert accepté.
4. Demander un numéro de ticket, les réponses au formulaire
   `support/factory_response_REQUIRED.csv` et un devis détaillé.
5. Si JLCPCB exige une commande ou un paiement pour poursuivre le DFM,
   conserver leur réponse et s'arrêter à ce stade. Aucune revue n'est déclarée
   obtenue par le simple dépôt de fichiers.

## Fichiers du configurateur : un dépôt distinct par carte

| Champ / contrôle | Carte A | Carte B température |
| --- | --- | --- |
| ZIP à placer dans « Add Gerber file » | `upload_A/A04_GERBER_DFM_ONLY.zip` | `upload_B/B05_GERBER_DFM_ONLY.zip` |
| BOM, au stade assemblage | `upload_A/A04_BOM_ALL_ASSEMBLED.csv` | `upload_B/B05_BOM_ALL_ASSEMBLED.csv` |
| CPL, au stade assemblage | `upload_A/A04_CPL_ALL_ASSEMBLED.csv` | `upload_B/B05_CPL_ALL_ASSEMBLED.csv` |
| Quantité de PCB / quantité assemblée | **5 / 5** | **5 / 5** |
| Contour fonctionnel détecté | **110 × 75 mm** | **90 × 65 mm** |
| Couches / épaisseur | 2 / 1,6 mm | 2 / 1,6 mm |
| Cuivre extérieur, chaque face | **1 oz / 35 µm** | **2 oz / 70 µm** |
| Finition / masque / sérigraphie | ENIG / vert / blanc | ENIG / vert / blanc |
| Mode d'assemblage demandé pour ce devis | **Standard**, dessus | **Standard**, dessus |
| Articles à monter par carte | **23 = 13 CMS + 10 traversants** | **47 = 38 CMS + 9 traversants** |

FR4 compatible sans plomb, Tg ≥150 °C demandé ; à confirmer avec le fabricant.
Les paramètres précis sont dans `support/fabrication_specification.json`.
L'archive complète et les STEP ne vont **pas** dans le champ Gerber : ce champ
reçoit seulement le petit ZIP Gerber/perçages de la carte concernée.

Le mode Standard est demandé pour les deux devis afin de garder un parcours
commun. Pour **B**, le profil retenu doit respecter le pic LM2596 **≤245 °C** ;
le mode Economic ne convient pas au composant sélectionné. Les
[capacités actuelles JLCPCB](https://jlcpcb.com/capabilities/pcb-assembly-capabilities)
annoncent aussi des rails et fiduciels en Standard, et un minimum 70 × 70 mm.

**B fait réellement 90 × 65 mm : le ZIP représente la carte fonctionnelle, pas
un panneau prêt pour Standard.** La proposition de rails amovibles de 5 mm
donnerait un panneau 90 × 75 mm ; ce n'est pas un Gerber de panneau validé.
Demander à JLCPCB le panneau, les attaches et les fiduciels pour approbation,
sur A aussi lorsque nécessaires. Ne pas changer la dimension affichée pour
contourner le contrôle, ni choisir Economic pour faire passer B.
Si le configurateur bloque sur ces points, revenir au ticket support avec
le ZIP fonctionnel. Faire livrer les cartes dépanélisées et ébavurées aux
dimensions fonctionnelles, sans résidu d'attaches sur leurs bandes de clips.

## Parcours en ligne limité au chiffrage

Après la première réponse du support, ou pour voir le devis automatique de A :

1. Se connecter puis ouvrir [le configurateur](https://cart.jlcpcb.com/quote).
   Charger le ZIP de **A seulement**, contrôler le contour et choisir les
   valeurs du tableau. Le titre de catégorie « Standard PCB/PCBA » ne garantit
   pas que le sous-mode d'assemblage **Standard** est sélectionné.
2. Activer PCB Assembly, dessus, quantité assemblée 5, et poursuivre jusqu'aux
   écrans d'import BOM/CPL si le site le permet sans engagement.
3. Charger les deux CSV de A, associer les colonnes si demandé, puis contrôler
   les références et fabricants. Conserver les articles sans correspondance
   dans la demande manuelle ; ne pas les supprimer pour terminer le parcours.
4. Contrôler l'aperçu, les polarités et les deux rangées de sockets. Les images
   `support/placement_A_REFERENCE.png` et `placement_B_REFERENCE.png` montrent
   nos positions de départ, pas une orientation déjà approuvée par JLCPCB.
5. Conserver les captures du prix détaillé, de la liste de composants et de
   l'aperçu. Un enregistrement de brouillon/panier peut servir si proposé,
   mais **s'arrêter avant « Secure Checkout », « Submit Order » ou « Pay »**,
   et avant toute réservation payante de pièces.
6. Préparer le second article B avec ses propres fichiers seulement selon
   les indications reçues pour la panélisation. Ne pas fusionner A et B en
   un panneau commun improvisé. Transmettre les références de brouillon au support.

Les libellés peuvent varier selon le compte. Cette procédure a été documentée
depuis les sources publiques ; aucun import dans un compte JLCPCB n'a été testé.

## Points à faire traiter avant toute commande

- **Toutes les pièces montées** : le devis doit couvrir les CMS et les
  traversants, y compris connecteurs, sockets, C8/C10 et Q1. Le lot représente
  **350 placements**, plus **5 cavaliers JP1 à livrer séparément**.
  DevKit, écran, modules capteurs, sonde, chauffage et boîtier ne sont pas à
  acheter ni à assembler par JLCPCB dans cette demande.
- **Sourcing** : `support/exact_sourcing_REQUIRED.csv` liste les MPN sans ID
  catalogue vérifié : C10, R13 et les trois articles Harwin. Les quantités sont
  nettes pour le lot ; demander les pertes et minimums d'achat séparément.
  Les autres codes catalogue proviennent de la revue locale du 5 novembre :
  aucun stock ni tarif actuel n'est garanti. Demander une solution exacte ou
  une proposition documentée, sans substitution automatique. Les services
  [d'approvisionnement JLCPCB](https://jlcpcb.com/help/article/pcba-parts-sourcing-instruction)
  restent à chiffrer ; ne pas déclencher leur achat à ce stade.
- **Deux sockets sur A** : J8 et J9 sont des **alias d'import seulement**,
  remplaçant les deux rangées de l'empreinte virtuelle A/U1. J8 correspond aux
  pads U1.1…15, J9 aux pads U1.16…30. Ces repères ne sont pas ajoutés à la
  sérigraphie. Voir `support/upload_to_native_mapping.csv` et faire confirmer
  cette convention par l'assembleur ; **aucune DevKit ESP32 à souder sur A**.
  Le U1 de B est bien le LM2596 et doit être monté.
- **Coordonnées** : les CMS reprennent le CPL natif contrôlé. Les traversants
  utilisent le centre du corps F.Fab, et les sockets le centre de leur rangée,
  plutôt que la broche 1 ou le centre de la DevKit complète. Le mapping conserve
  origine et pin 1 natives. Les angles sont normalisés à 0…360°, positifs
  antihoraires ; leur correspondance avec les modèles JLCPCB reste à approuver.
  Les X sont absolus, les Y sont les Y KiCad inversés, en mm, comme le CPL
  existant et les Gerber. Ne pas appliquer un décalage arbitraire.
- **Procédé et essais** : confirmer profil thermique/MSL, soudure des traversants,
  perçages finis des sockets, tests PCB nus, inspection et essais des rails
  proposés dans `support/factory_acceptance_test.md`. Les résultats physiques
  n'existent pas encore ; l'acceptation DFM ne prouve pas la régulation thermique.

Les spécifications de données sont celles des pages officielles
[BOM](https://jlcpcb.com/help/article/bill-of-materials-for-pcb-assembly) et
[CPL, qui couvre aussi les traversants](https://jlcpcb.com/help/article/pick-place-file-for-pcb-assembly).
Le package est vérifié localement par `prepare_jlcpcb_submission.py --check` ;
le manifeste lie les sources et les fichiers préparés par SHA256.

## Retour attendu et arrêt

Récupérer devis, stocks/MPN, schéma de panneau, positions/polarités, procédé et
périmètre des tests. Renseigner le formulaire joint avec les justificatifs et
les références de ticket. Une approbation de commande viendra après cette revue.
Le prototype reste A + B température ; aucune autre carte métier n'est incluse.
