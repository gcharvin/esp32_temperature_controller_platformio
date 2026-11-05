# Essai USB de service sur les DevKit réelles

Le branchement au PC permet d'identifier l'ESP32, de lire la flash et de
tester la liaison série. Il ne mesure pas les chemins VBUS/VIN ni le retour
de courant. Des cartes USB-C d'entraxe identique peuvent différer sur ces points.
La confirmation utilisateur de 25,4 mm reste valable pour la carte mesurée ;
elle n'identifie pas à elle seule le fabricant ou le pont USB.

## Accessoire concret à essayer

Candidat de test : **PortaPow USB Power Blocker PPUSBBP1**, USB-A, données
USB2 et masse conservées, 5 V coupé. Il se place côté PC, avant un câble
USB-A vers USB-C de données. La rallonge latérale du boîtier peut ainsi rester
USB-C. Le fabricant avertit que certains appareils ne fonctionnent pas sans
VBUS : la compatibilité de chaque DevKit reste à essayer. Ce produit n'est
pas un data blocker et n'assure pas d'isolation galvanique.
Source : [PortaPow](https://portablepowersupplies.co.uk/product/usb-power-blocker).

```text
PC USB-A -> PPUSBBP1 (VBUS coupé) -> câble de données A/C -> DevKit USB-C
Source 5 V régulée, limitée -> VIN/5V et GND de la DevKit
```

Ce test commence sur la DevKit seule, sans chauffage ni carte B et sans
périphériques susceptibles d'être alimentés par les GPIO. Il ne nécessite
pas un autre firmware : l'énumération USB est assurée par le pont USB/série,
et le firmware existant peut fournir les logs. Si des essais logiciels
deviennent nécessaires, sauvegarder toute la flash et comparer la sauvegarde
avec la mémoire avant toute écriture. Conserver les images hors Git : elles
peuvent contenir des identifiants Wi-Fi/API et des données NVS.

## Mesures à consigner pour chaque carte, sans généraliser à ses voisines

1. Identifier physiquement la carte, ses 30 contacts et leur brochage. Noter
   pont USB, port COM, puce et flash détectées. Les CP210x observés ne valident
   pas automatiquement le CH340 annoncé dans la fiche de la diymore retenue.
2. Hors tension, vérifier à l'ohmmètre la coupure du 5 V dans l'accessoire et
   la continuité de masse. Employer un interposeur/breakout de mesure adapté,
   sans sonder des broches USB-C serrées avec un montage sous tension.
3. PC déconnecté, alimenter VIN avec une alimentation de laboratoire à 5 V
   limitée en courant et relever le 3,3 V et le courant consommé. Vérifier
   que la carte démarre normalement et que les rails restent admissibles.
4. Avec l'adaptateur dans le chemin, vérifier l'absence de remontée du 5 V
   côté PC à l'aide de l'interposeur, puis connecter le PC. Relever
   l'énumération, les logs et le comportement du pont série. Si le pont
   n'est plus alimenté, l'essai est négatif ; ne pas retirer le bloqueur en
   laissant VIN alimenté.
5. Couper le 5 V externe en laissant la liaison de données connectée : relever
   les rails résiduels et l'absence de rétroalimentation. Débrancher le PC
   avant de modifier le câblage. Consigner modèle/câble/alimentation exacts.

Si ce montage ne fonctionne pas, étudier un interposeur qui sépare les sources
et fournit le 5 V de la carte au côté périphérique de l'USB ; cette variante
nécessite son schéma et une revue dédiée. Ne pas remplacer cet essai par un
simple câble USB ordinaire + VIN externe, ni conclure à partir du seul firmware.

Les opérations `flash-id`, `read-flash` et `verify-flash` ne remplacent aucun
de ces essais électriques. Source :
[Espressif esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/basic-commands.html).
