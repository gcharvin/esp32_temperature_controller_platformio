# Mise au point physique — A v0.4 / B v0.5

État au 5 novembre 2026 : schémas et PCB contrôlés, six transitoires nominaux,
deux sensibilités ESR et démarrage du buck lié aux valeurs de B réussis. Les
empreintes des sources et modèles ont été revérifiées. Aucun appareil ni PCB
physique n'a été mesuré dans cette session. Réexécuter les mêmes simulations
sans changement de circuit ne qualifie pas l'échauffement ou le câblage.

## Avant la première alimentation

Confirmer les références d'achat de C9/R13, les composants réellement montés,
la polarité C8/C10 et D9, et le brochage/languette GND du LM2596 fixe 5 V.
Contrôler hors tension l'absence de court-circuit entre rails et la continuité
PGND–GND à NT1. Les deux noms de masse ne constituent pas une isolation.
Conserver les résultats ERC/DRC avec les fichiers effectivement fabriqués.

## Essais à consigner

| Étape | Montage et mesures | Critère / décision |
| --- | --- | --- |
| Buck seul | A et chauffage débranchés ; alimentation de laboratoire limitée en courant ; montée à 24 V. Mesurer avant/après F2, puis charger progressivement à 0,1 / 0,5 / 0,8 A. | 4,75–5,25 V en régime établi ; absence d'oscillation durable. Vérifier l'appel de courant avant de choisir une limite d'alimentation définitive. |
| Démarrage et entrée | Oscilloscope sur 5 V et SW avec boucle de masse courte ; plusieurs démarrages à froid ; entrée 18 / 24 / 26,4 V. Relever Vout min/max, durée de montée et, si sonde de courant adaptée, pic de L1. | Pas de dépassement de 5,25 V ; enveloppe des transitoires à confronter au banc. Le pic simulé de L1 approche 4,9 A : saturation et limitation réelles restent à établir. |
| Dynamique de charge | Charge électronique 0,1 ↔ 0,8 A ; observation de la réponse et ondulation avec méthode de sondage documentée. | 4,75–5,25 V après démarrage et objectif d'ondulation <50 mV crête à crête. Documenter bande passante et point de mesure. |
| A et périphériques | Raccorder A, écran et capteurs ; relever les rails à B et à A pendant démarrage, affichage et activité Wi-Fi. | Rails dans leurs tolérances, pas de reset ESP32 ; vérifier les courants réels par rapport au budget 0,8 A et à F2 1 A. |
| Chauffage / NTC | Charge résistive appropriée hors boîtier, jusqu'à 24 V / 100 W ; PWM 500 Hz, plusieurs rapports cycliques ; comparer NTC avec et sans commutation. | Retour de puissance maîtrisé, absence de perturbation entraînant une mauvaise commande ; limites de bruit/température à fixer selon l'usage puis consigner. |
| Thermique | D'abord boîtier ouvert, puis fermé avec ouïes latérales dégagées ; fonctionnement prolongé aux charges réelles. Relever ambiance, U1, L1, D9, condensateurs, borniers, cuivre et plastique jusqu'à stabilisation. | Critères à fixer à partir des limites des références finales et du filament choisi, avec marge ; aucune température admissible universelle n'est supposée. Un dissipateur éventuel exige une nouvelle revue 3D. |
| Arrêts et défauts | Vérifier le chauffage au démarrage, à la perte de commande, et avec NTC ouverte/court-circuitée. Essais de surcharge du buck uniquement avec montage et limitation adaptés. | Pas d'activation intempestive du chauffage ; réponse de défaut documentée. Le firmware historique n'implémente pas encore tout ce comportement. |

USB : procéder d'abord en alimentation USB seule selon la procédure de service
du dépôt. La mise en parallèle VBUS/5 V externe n'est pas qualifiée ; l'ouverture
latérale du boîtier ne résout pas la sélection des sources.
Ce point vaut aussi pour les futures alimentations 12 V ou 5 V de B : A reste
sous 5 V régulé. La stratégie, les mesures de la DevKit et le mode série
alimenté restant à qualifier sont détaillés dans
`../../manufacturing/power_and_usb_review.md`. Les points avant commande sont
séparés des essais après réception dans
`../../manufacturing/release_checklist.json`.

Les surtensions, le déclenchement des fusibles, le court-circuit prolongé et la
coupure thermique indépendante restent des qualifications distinctes. La
simulation du buck n'établit pas la sécurité de l'ensemble en défaut.

## Suite mécanique autorisée

La mise à jour du boîtier peut avancer avec les dimensions actuelles de B. Elle
doit conserver accès et démontabilité, modéliser les hauteurs réelles choisies,
retirer la baie XL4015, et garder les ouvertures uniquement sur les côtés. Un
coupon de clips/rainure puis un montage câblé réel restent nécessaires avant
de considérer le boîtier comme prêt à imprimer en série.
