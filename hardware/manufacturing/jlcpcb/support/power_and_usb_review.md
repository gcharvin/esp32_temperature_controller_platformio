# Contrat d'alimentation de la plateforme — revue avant fabrication

Décision du 5 novembre 2026 : les cartes métier pourront recevoir **24 V,
12 V ou 5 V**, selon leur fonction. La carte A doit rester commune. Ces tensions
sont des variantes de cartes B, pas trois positions déjà disponibles sur B v0.5.

## Interface stable : 5 V régulé vers A

```text
B, entrée 24 V -> conversion/protection propres à B -> 5 V régulé -> A/J1
B, entrée 12 V -> conversion/protection propres à B -> 5 V régulé -> A/J1
B, entrée  5 V -> protection et distribution adaptées -> 5 V régulé -> A/J1
                                                                |
                                                DevKit + 3,3 V périphériques
```

A/J1 reste : **1 = GND ; 2 = +5V_AB**. Cible à son connecteur : 4,75–5,25 V,
chutes dans le câble incluses. Le budget actuel de 0,8 A est partagé entre A
et la logique B, pas une réservation de 0,8 A pour chaque carte. Mesurer A en
Wi-Fi et avec le LCD réel avant de figer le budget des prochaines cartes B.
Ni 12 V, ni 24 V, ni alimentation 3,3 V directe sur J1. Le 3,3 V local reste
produit séparément sur chaque carte ; ne pas relier les sorties des régulateurs.

Une source 5 V dédiée peut alimenter A sur J1 **à la place du faisceau POWER
de B**, avec son propre câble correctement repéré et protégé. Ne pas mettre
deux sources en parallèle. B v0.5 ne possède ni entrée 5 V auxiliaire, ni
blocage de courant inverse sur sa sortie F2 : ne pas l'alimenter à rebours
depuis J1. I²C/CTRL ne sont à raccorder que lorsque les deux logiques sont
alimentées ; aucune tolérance à une carte éteinte n'est qualifiée. Toutes
les connexions A/B s'effectuent hors tension, sans hot-plug.

| Variante | État | Travail propre à B |
| --- | --- | --- |
| Température 24 V / 100 W | B v0.5 actuelle, prototype | Essais physiques et revue de fabrication restants. |
| Entrée 12 V | Préparation seulement | Le cœur LM2596 peut être étudié à 12 V ; revoir TVS, voyant VIN, fusibles, charge, courant et thermique avant de déclarer la carte compatible. |
| Entrée 5 V | Nouvelle variante à concevoir | Distribution protégée à faible chute ; supprimer/remplacer le chemin abaisseur. Aucun strap de contournement improvisé sur B v0.5. |

Le LM2596 fixe 5 V est spécifié par TI avec VIN de 7 à 40 V pour la régulation
5 V dans les conditions de sa table électrique. Il ne garantit donc pas
5 V en sortie depuis une entrée 5 V. Un buck-boost ne devient utile que si
la plage réelle d'une future source impose de remonter la tension.
Source : [TI, section 7.6](https://www.ti.com/lit/ds/symlink/lm2596.pdf).

La limite de courant ne suit pas automatiquement la tension : 100 W
représenteraient 8,33 A sous 12 V et 20 A sous 5 V, contre 4,17 A sous 24 V.
**Aucune capacité 100 W à 12 V ou 5 V n'est attribuée au routage actuel.**
Chaque carte B définit séparément tension, puissance et courant maximal.

## USB latéral : conserver le port, maîtriser le VBUS

Précision utilisateur du **6 novembre 2026** : en fonctionnement normal,
A est alimentée par la source centrale, via **5 V régulés → A/J1 → JP1 →
VIN/5V de la DevKit**. L'USB-C sert aux données, au flash et au diagnostic,
pas d'entrée d'énergie prévue pour l'ensemble. Aucun 12/24 V sur ce VIN.
Ce choix ne modifie pas les liaisons internes des trois variantes de DevKit :
le contrôle de VBUS ci-dessous reste nécessaire pour le monitoring alimenté.

Le port accessible est celui de la DevKit enfichable (CP210x ou CH340 selon carte), pas un connecteur USB câblé
sur A. A n'a accès qu'au VIN de la DevKit, à travers JP1 ; le VBUS du PC
n'est pas disponible séparément sur A. Les documents du CH340 ne donnent
pas le schéma d'alimentation de cette DevKit particulière.

**Conclusion de revue :** ajouter une diode ou un multiplexeur uniquement à
la place de JP1 ne suffit pas à démontrer l'absence de retour de courant vers
le PC : une liaison VIN–VBUS peut exister à l'intérieur de la DevKit.
Ne pas modifier A sur cette hypothèse. La tension d'entrée de B ne change
pas ce problème : il existe dès que le rail 5 V externe est présent.

| Mode | Raccordement retenu pour le prototype | État |
| --- | --- | --- |
| Fonctionnement complet | Source B ou source 5 V dédiée unique ; JP1 en place ; USB débranché | Architecture courante, essais physiques restants. |
| Flash USB seul | Couper toutes les sources externes ; débrancher J1/J6/J7 ; retirer JP1 ; brancher l'USB | Mode de service conservé ; A/périphériques et B ne fonctionnent pas dans ce mode. |
| Série USB pendant fonctionnement | Nécessite une solution vérifiée de séparation du VBUS dans la rallonge latérale, ou une gestion de sources démontrée sur la DevKit réelle | **Non qualifié ; câble USB ordinaire interdit avec le 5 V externe.** |

Le choix à qualifier en priorité est un interposeur USB de service qui coupe
physiquement le VBUS du PC en conservant données et masse. Ce n'est pas un
« data blocker », qui coupe les données. Il faut vérifier que le CH340 reste
alimenté et que l'énumération, le flash et le moniteur série fonctionnent.
Un interposeur à VBUS coupé ne constitue ni une isolation galvanique, ni une
solution USB universelle : comportement à l'extinction, détection de VBUS,
et, pour USB-C, câblage CC/orientation doivent être qualifiés. Si ce mode ne
fonctionne pas, il faudra accéder aux deux sources USB/externe en amont de
la DevKit via un interposeur de sélection, ou changer de DevKit documentée.

### Vérification à effectuer avec la DevKit achetée

1. Hors tension, repérer la référence exacte du CH340 et relever le chemin
   entre VBUS, VIN et régulateur 3,3 V (continuité et mode diode dans les deux
   sens). Le seul bip de continuité ne démontre pas le comportement sous tension.
2. Avec un interposeur de mesure et une alimentation limitée, vérifier les
   tensions et courants entre ces nœuds, PC déconnecté. Ne pas utiliser le PC
   comme instrument de détection d'un retour de courant.
3. Si la séparation de VBUS est adéquate, essayer données/énumération avec
   source externe, puis extinction de cette source alors que le PC reste
   connecté. Consigner le montage exact et l'absence de rétroalimentation.
4. Reporter le résultat dans le dossier de revue, puis seulement déclarer le
   mode série alimenté disponible. Un changement matériel impose de réviser
   schéma, routage, BOM, mécanique et contrôles concernés.

Source sur le composant, sans prétendre documenter la DevKit :
[WCH, CH340](https://www.wch-ic.com/downloads/CH340DS1_PDF.html).

## Pic de démarrage du buck

Le pic nominal simulé de B est de 4,88 A. La fiche Bourns de L1 annonce à
25 °C 5,3 A Irms (critère d'échauffement), 6,5 A Isat1 (perte d'inductance de
20 %) et 118 mΩ DCR maximale. Le pic nominal est inférieur à Isat1 ; ce
constat n'est pas une garantie aux tolérances, à chaud ou en court-circuit.
Le modèle linéaire ne représente pas la saturation. Mesurer démarrage et
températures sur prototype ; conserver l'étude ESR/tolérances existante.
Source : [Bourns SRP1770TA](https://www.bourns.com/docs/Product-Datasheets/srp1770ta.pdf).
