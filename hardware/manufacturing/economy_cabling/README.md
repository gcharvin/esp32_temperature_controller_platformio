# Câblage économique — RFQ-CABLE-02

Révision du **9novembre2026**. Catalogue Amazon.fr/GoTronic d’abord,
sur-mesure seulement pour les manques documentés. L’utilisateur accepte
des soudures à faire réaliser ; **aucun sertissage manuel**. Aucun devis
reçu, envoi, achat ou lancement de production.

Le [ZIP économique actuel](../LCSC_CABLE_RFQ_ECONOMY_NOT_FOR_ORDER.zip)
et le [PDF](output/pdf/LCSC_CABLE_RFQ_ECONOMY.pdf) remplacent le tout-sur-mesure
RFQ-CABLE-01 comme demande courante. Le [message anglais](request_quote_EN.txt)
est prêt à copier dans la demande LCSC, sans envoi automatique.

## Quatre références spéciales au lieu de treize

| Référence | Quantité de base pour10appareils | Avec TMP117 |
| --- | --- | --- |
| H01 XH2-XH2,300mmAWG22 | 10 | 10 |
| H02 XH3-XH3,300mmAWG26 | 10 | 10 |
| H03 XH6-XH6,300mmAWG26 | 10 | 10 |
| U04 XH4 vers4femelles individuelles,300mmAWG26 | 30 | **40au total**, pas30+40 |
| Total de câbles spéciaux | **60** | **70** |

Les broches A/B, longueurs300+/-10mm et sections existantes sont conservées.
Les six fils CTRL sont présents, dont5/6NCsurB température. Les quatre
types restent sur-mesure faute de référence catalogue exacte qualifiée
chezAmazon/GoTronic. Les kits repérés à15cm et/ouAWG22 ne sont pas des
substituts validés aux câbles signaux300mmAWG26.

U04 est une seule référence partagée :10LCD,10BH1750,10encodeurs, puis10TMP
optionnels. Les quatre femelles sont indépendantes et portent les numéros
1..4. **Aucune fonction GND/VCC commune ne doit être imprimée sur U04** :
pourLCD/BH/TMP,1=GND ; pourencodeur,4=GND. Les couleurs ne font pas le brochage.

Les contacts/boîtiers sont livrés sertis et montés. Demander une finition
standard sans surmoulage ni gaine industrielle spéciale ; chaque conducteur
reste isolé. Harwin M20-1060100/M20-1160042 sont une référence documentée
pour les femelles2,54mm/broches carrées0,64mm. LCSC peut proposer un fabricant
compatible moins cher, avec MPN et dimensions à approuver, pas une copie
présumée identique sur photographie. Les outils qualité de sertissage et
le contrôle de continuité ne sont pas supprimés pour réduire le prix.

## Ce qui passe en catalogue

| Besoin pour10appareils | Référence | Prix public observé TTC | Travail restant |
| --- | --- | --- | --- |
| 10encodeurs siabsents | [GoTronicADA377/38383](https://www.gotronic.fr/art-encodeur-24-impulsions-ada377-37481.htm) | 59,00EUR | Couper les femelles de10U04 côté encodeur, dénuder et souder5terminaux avec pontC-SW2 |
| 10cordons sonde internes **conditionnels** | [GoTronicJST1164/49197](https://www.gotronic.fr/art-cordon-jst-xh-femelle-2-cts-15-cm-jst1164-33095.htm) | 21,00EUR | Longueur15cm à essayer avant achat du lot ;2soudures au jack |
| Proposition2m de protection sonde interne | [GoTronicGAIT06/48620](https://www.gotronic.fr/art-gaine-tressee-gait06-15337.htm) | 1,60EUR | Polyester expansible3..9mm, vendu au mètre ; à retenir sans rigidifier raccords/clips |
| 10adaptateursTMP117 **optionnels** | [GoTronicQW-4C/49152](https://www.gotronic.fr/art-cordon-qwiic-stemma-qw-4c-30786.htm) | 21,00EUR | Relier les4mâles aux4femelles U04 selon fonctions et immobiliser jonction |
| 10paires jack utiles, lot à vérifier | [AmazonDAOKIB08XM6DR96](https://www.amazon.fr/dp/B08XM6DR96) | Non qualifié | Offre, nombre parlot, coupon, filetage et serre-câble à confirmer ;2soudures internes et2surfiche externe |

Les prix sont des observations catalogue du9novembre2026, pas un devis
LCSC ni une garantie de stock. Vérifier les offres, la livraison et le lot
Amazon, dont la page n’a pas pu être relue directement. Le sous-total
catalogue connu est81,60EURTTC, ou102,60EURavec les10adaptateursTMP. Les
modulesTMP117/BH1750/LCD ne sont pas inclus dans ces sous-totaux.

### Affectation et opérations

- **LCD/BH1750 :** enficher U04 par fonctions1GND/2VCC3,3V/3SDA/4SCL sur
  des barrettes mâles déjà soudées. Le module réel et le mou restent à vérifier.
- **Encodeur :** U04 côtéA/J3 garde1A/2B/3bouton/4GND. Couper seulement
  les femelles côté module, souder les fils aux terminaux et ponterC-SW2,
  puis isoler. La variante exacte de l’encodeur et sa vue des contacts
  restent à confirmer sur fiche/échantillon.
- **TMP117 :** option à chiffrer séparément, aucune référence spécialeSH
  ni transition de calibres sur-mesure. U04+QW-4C donnent environ450mm,
  à essayer pour l’I2C et le démontage, pas une longueur déjà qualifiée.
- **Jack interne :** JST1164.1versTIP(+3V3_B),.2versSLEEVE(NTC_RAW),
  ringisolé. Sleeve n’est pas GND. Identifier les pattes par continuité,
  isoler séparément chaque soudure, retenir la tresse. Ne pas supposer
  que15cm suffisent ; demander une variante30cm si l’essai de service échoue.
- **Puissance :** câble catalogue gainé2x0,75mm2, référence finale encore
  ouverte. CôtéPhoenix1715721, un conducteur souple parpôle peut être dénudé
  de7mm puis serré0,5..0,6Nm en soutenant le bornier, sans brins libres et
  sans étamage. Les embouts ne sont pas obligatoires : plage fabricant
  souple0,14..1,5mm2. Les autres terminaisons et l’isolation à chaud restent
  à qualifier avec les appareils. [Données fabricant](https://www.phoenixcontact.com/en-gb/products/printed-circuit-board-terminal-mkds-15-2-508-1715721).

Les faisceaux internes ne demandent plus une deuxième gaine isolante spéciale
sur tous les fils. Les fils gardent leur isolant et un regroupement léger
peut être posé si utile. La tresse de sonde interne est une protection
mécanique, pas une isolation électrique. La **sonde extérieure** garde au
contraire une gaine commune continue robuste, température admissible
documentée et vraies retenues aux deux bouts. Rien n’est retiré des besoins
de protection thermique indépendante ou de serviceUSB hors alimentation externe.

## Budget et main-d’œuvre

Estimation de travail, non confirmée par LCSC :

| Poste |10jeux sansTMP | SupplémentTMP pour10jeux |
| --- | --- | --- |
| Câbles spéciaux hors frais/taxes, hypothèse | 95-205EUR | 15-30EUR pour10U04 supplémentaires |
| Catalogue aux prix publics TTC observés | 81,60EUR | 21EUR pour10QW-4C |
| Allocation jack Amazon, prix non vérifié | 10-20EUR | Sans supplément |
| Consommables atelier, hypothèse | 5-15EUR | Sans supplément prévu |
| Provision préparation/port/taxes, non devisée | 50-120EUR | Éventuel écart à confirmer |
| Débours estimés, arrondis | **250-450EUR** | **280-500EUR au total avecTMP** |

Le calcul exact des bornes est241,60-441,60EUR et277,60-492,60EUR avecTMP.
La provision englobe des frais encore inconnus ; ce n’est pas un calcul
fiscal définitif. Ces bornes ne sont pas des tarifs fabricants.

**Main-d’œuvre de soudure exclue et à chiffrer.** Compter5points de soudure
encodeur et2aujack parappareil, soit70points pour10jeux, plus les fiches de
sondes extérieures quand choisies. Préparation, identification et contrôle
prennent aussi du temps. L’estimation antérieure350-750EUR incluait les
sous-ensembles soudés chez le câbleur : on ne peut pas présenter le transfert
en atelier comme une économie certaine si cet atelier est payant. Le plan
réduit les références et opérations spéciales ; le gain net attend le devis
réduit et le prix de la soudure. Réutiliser encodeurs/accessoires existants
réduit encore l’achat réel, sans les compter deux fois.

L’alimentation, le chauffage, la sonde NTC extérieure, le moduleTMP117,
le BH1750, la protection thermique, la rallongeUSB définitive et le boîtier
ne sont pas compris. Les cinq anciennes lignes H09-H13 sont **différées**,
pas déclarées disponibles dans un panier prêt à commander. La référence
NTC15k/courbe, le nominal12/24V et la charge réelle restent à déterminer.

## Fichiers et vérifications

[custom_quote_lines.csv](custom_quote_lines.csv) est la demande LCSC courante,
[custom_pin_map.csv](custom_pin_map.csv) ses15correspondances ;
[retail_basket.csv](retail_basket.csv) liste les achats catalogue et les
conditions ; [purchase_plan.csv](purchase_plan.csv) réaffecte toutes les
13liaisons du dossier initial. La [sourceJSON](economy_plan.json) distingue
prix observés, hypothèses, opérations et achats non résolus.

Générateurs : `hardware/scripts/export_economy_cabling.mjs` (ArtifactTool,
quatreCSV et BOM système), puis `build_economy_cabling.py --documents` ;
après relecture du PDF, `build_economy_cabling.py --package`. Les contrôles
comparent H01-H03 au contrat et aux netlistsKiCad ; U04 est contrôlé par
numéros et par affectationLCD/BH/TMP/encodeur. La pose et le fonctionnement
réels restent à essayer. Aucun câble ni capteur n’est modifié sur les PCB.
