# Devis LCSC Cable - 10 exemplaires de chaque faisceau

Dossier RFQ-CABLE-01 du **9 novembre 2026**, préparé localement. **Aucun envoi,
compte créé, commande, paiement ni lancement de production.**

Le ZIP à transmettre est [LCSC_CABLE_RFQ_10_EACH_NOT_FOR_ORDER.zip](../LCSC_CABLE_RFQ_10_EACH_NOT_FOR_ORDER.zip).
Ouvrir d’abord le [plan PDF](output/pdf/LCSC_CABLE_RFQ_10_EACH.pdf) : deux pages
de cadrage, puis une fiche par référence. Le texte anglais prêt à copier est
[request_quote_EN.txt](request_quote_EN.txt). Cette consultation est l’exception
LCSC autorisée par l’utilisateur pour les faisceaux ; Amazon/GoTronic restent
les canaux catalogue. Aucune référence du panier existant n’est remplacée ni
aucun fichier KiCad/STEP modifié par ce dossier séparé.

## Quantités à faire chiffrer

**10 de chaque référence**, prix séparés : huit liaisons internes = 80 unités,
dont le TMP117 optionnel, plus cinq lignes provisoires = 50 unités. Le total
de 130 est une portée de devis si tout était retenu, pas une commande.

| Référence | Liaison | Définition pour le devis |
| --- | --- | --- |
| H01 | A/J1 vers B/J1, alimentation 5 V | Deux XHP-2, AWG22, 300 +/-10 mm, droit |
| H02 | A/J6 vers B/J6, I2C | Deux XHP-3, AWG26, 300 +/-10 mm, droit |
| H03 | A/J7 vers B/J7, commandes | Deux XHP-6, AWG26, 300 +/-10 mm, six fils présents ; 5/6 NC sur B |
| H04 | A/J2 vers LCD | XHP-4 vers quatre femelles individuelles Harwin M20, AWG26, 300 +/-10 mm proposés, repérage par fonction |
| H05 | A/J3 vers encodeur | XHP-4, AWG26, 300 +/-10 mm proposés, encodeur soudé en atelier, commun poussoir/encodeur réalisé et isolé |
| H06 | A/J4 vers TMP117, option | XHP-4 vers SHR-04V-S-B, 300 +/-10 mm proposés, transition en atelier AWG26 vers AWG28 |
| H07 | A/J5 vers BH1750 GY-302 | XHP-4 vers quatre femelles individuelles Harwin M20, AWG26, 300 +/-10 mm proposés, repérage par fonction |
| H08 | B/J4 vers prise sonde arrière | XHP-2 vers embase PJ-392 soudée en atelier, AWG26, 300 +/-10 mm proposés, gaine commune |
| H09 | Sonde extérieure et fiche jack | **Provisoire** : ensemble sonde NTC15k, câble robuste gainé et fiche TRS montée ; 1 m de chiffrage, courbe/cotes/température à définir |
| H10 | Alimentation DC vers B/J2 | **Provisoire** : câble 2 x0,75 mm2 gainé, embouts montés côté PCB ; 1 m de chiffrage, sortie alimentation non arrêtée |
| H11 | B/J3 vers chauffage | **Provisoire** : câble 2 x0,75 mm2 gainé, embouts montés ; 1 m de chiffrage, chauffage/température/terminaison à préciser |
| H12 | Protection thermique indépendante | **Provisoire** : 10 jeux de deux fils 0,75 mm2, chacun 300 mm de chiffrage ; contacts, seuil et intégration à H11 ouverts |
| H13 | USB-C de panneau vers DevKit | **Provisoire** : extension USB2 données complète, 300 mm de chiffrage ; plan compatible avec réservation latérale 16 mm à proposer |

Le câble courant PC vers USB-C peut être réutilisé : il n’est pas commandé
en dix exemplaires sur mesure. Les cordons retail QW-4C et JST1164 ne sont
pas ajoutés aux quantités des futurs ensembles complets H06/H08.

## Connecteurs, brochage et gaines

- [quote_lines.csv](quote_lines.csv) : 13 lignes, quantité 10 chacune,
  connecteurs et finitions, colonnes laissées vides pour prix/délai/écarts.
- [pin_to_pin.csv](pin_to_pin.csv) : 38 correspondances électriques,
  notamment le branchement commun de l’encodeur ; pas de brochage USB-C inventé.
- [connector_requirements.csv](connector_requirements.csv) : références
  boîtiers/contacts, pas, plages de fils et quantités pour les huit lignes internes.
- [rfq_specification.json](rfq_specification.json) : définition complète,
  sources techniques et conditions encore à faire confirmer.

Les XH sont **2,50 mm**, les SH **1,00 mm**, les barrettes de modules **2,54 mm**.
Les vues du PDF sont des interconnexions logiques, pas des vues physiques de
faces de connecteurs. Demander les deux vues fabricant, côté accouplement et
entrée de fils, avec détrompeur et cavité 1 repérés. La continuité demandée
est définie par numéros de cavités, jamais par une photographie ou les couleurs.

Les femelles individuelles LCD/BH1750 utilisent une proposition documentée
Harwin M20-1060100 + M20-1160042 pour broches mâles carrées 0,64 mm. Elles
gardent l’ordre adaptable aux inscriptions du module ; l’engagement et
l’encombrement doivent être vérifiés sur échantillon réel. Pas de connecteur
mâle libre ou d’embase à souder sur la carte à fournir à la place.

**Gaine commune souple et isolante**, repérage aux deux bouts, isolation des
soudures et retenue des transitions sont demandés. Une tresse PET seule ne
remplace pas une gaine isolante. LCSC doit proposer diamètre extérieur,
rayon de courbure et classe thermique : les connecteurs et les fils ont
chacun leur propre limite, le minimum proposé pour l’isolant du fil interne
ne qualifie pas une température de fonctionnement complète de 80 degC.
Pour la sonde extérieure, gaine robuste continue et véritables retenues
anti-arrachement côté sonde et fiche. La température réelle d’application
reste à définir, aucune sonde exacte n’est déjà qualifiée.

H06 demande une transition en atelier : les contacts XH sélectionnés
acceptent une isolation OD0,9-1,9 mm, les SH OD0,4-0,8 mm / AWG28-32.
Un même fil ordinaire ne satisfait pas ces deux plages de diamètre. Le
devis prévoit AWG26 principal et queues AWG28 pour SH, quatre transitions
isolées et retenues ; tout autre montage doit être documenté et approuvé.

**Sonde : tip = B/J4.1 (+3V3_B), sleeve = B/J4.2 (NTC_RAW), ring isolé.**
Sleeve n’est pas GND. L’atelier doit identifier les pattes de l’embase par
continuité avec une fiche connue, et fournir son plan de raccordement.
Demander H08 complet avec embase, écrou/rondelle et soudures ; H05 complet
avec encodeur soudé. Les pièces elles-mêmes et la main-d’œuvre seront
chiffrées séparément, avec variante de pièces confiées si nécessaire.

## Dépôt et réponse à demander

1. Ouvrir [LCSC Cable](https://lcsccable.com/customize), sélectionner la
   demande personnalisée et joindre le ZIP avec le texte anglais. Ne pas
   importer ces trois CSV comme une BOM d’assemblage PCBA JLCPCB.
2. Si le configurateur ne sait pas décrire les soudures ou le câble gainé,
   employer la demande sur plan. La [procédure officielle](https://lcsccable.com/faqs/ordering/submit-custom-cable-order-or-rfq)
   publie aussi `cable@lcsc.com`. Le dépôt/envoi restera à faire par l’utilisateur.
3. Demander prix de chaque ligne à 10 unités, frais de préparation/outillage,
   coût d’un premier article, délais, prix livré en France et taxes/frais
   distincts. L’adresse exacte sera ajoutée à la demande réelle, pas inventée ici.
4. Faire confirmer toutes les références, calibres, diamètres, vues et les
   écarts. H09-H13 sont des demandes de proposition : s’il manque une donnée,
   demander la liste des informations nécessaires et un chiffrage indicatif
   avec hypothèses explicites, sans les considérer prêts pour fabrication.
5. Examiner le plan et un premier article, vérifier accouplement, brochage,
   passage dans le boîtier, mou de démontage et retenues. Le nombre d’échantillons
   et leur inclusion dans les 10 unités sont à préciser au devis.

Un seul prestataire est consulté pour l’ensemble. Ni stock, ni devis,
ni délai de fabrication n’ont encore été obtenus. Le nominal12/24V et la
charge réelle restent ouverts ; l’étude B24V/100W n’autorise pas100W sous12V.

## Vérification et sources

Le générateur contrôle les cartes contre les netlists liées aux sources
KiCad et compare H01-H03 au contrat A/B existant. Les références, tableaux,
quantités et l’intégrité du ZIP sont contrôlés ; les pages PDF sont rendues
et relues visuellement. Voir [validation.json](validation.json) et
[visual_review.json](visual_review.json). Cela ne vaut pas validation du
câblage physique ou acceptation du fabricant. Le dossier PCBA précédemment
publié reste séparé et inchangé.

Régénération depuis la racine du dépôt avec les runtimes de cette machine :

```powershell
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe' hardware/scripts/export_lcsc_cable_tables.mjs
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/scripts/build_lcsc_cable_rfq.py --documents
# Rendre et relire le PDF, puis enregistrer son SHA256 dans visual_review.json.
& 'C:/Users/Gilles/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' hardware/scripts/build_lcsc_cable_rfq.py --package
```

L’export CSV utilise Artifact Tool du runtime partagé ; variable
`ARTIFACT_NODE_MODULES` possible si son emplacement diffère. La génération
de documents nécessite ReportLab et pypdf, la relecture Poppler.
