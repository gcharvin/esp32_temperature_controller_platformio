# Architecture refactor UI vs metier

## Objectif
- UI (LCD, menu, encodeur) decouplee du metier.
- Metier selectionne a la compilation via un flag (`APP_DOMAIN_TEMPERATURE`).
- Pins et timings centralises dans `src/config/app_config.h`.

## Points d'entree
- `src/main.cpp` : orchestration (boot, UI, serial, loop principale).
- `src/menu.cpp` : UI generique (liste de `Parameter`, edition, affichage de `UiSnapshot`).
- `src/pkg/domain/domain_api.*` : facade de selection du metier actif.
- `src/pkg/temperature/temperature_domain.*` : implementation metier temperature.

## Ajouter un nouveau metier
1. Creer `src/pkg/<nouveau>/<nouveau>_domain.h/.cpp` avec la meme API que `temperature_domain`.
2. Ajouter le routage dans `src/pkg/domain/domain_api.cpp`.
3. Ajouter un flag de build dans `platformio.ini`.
4. Definir ses pins dans `src/config/app_config.h` (ou un header de config dedie).

## Contrat metier <-> UI
- Le metier expose:
  - un tableau `Parameter` (menus dynamiques),
  - un `UiSnapshot` (donnees d'affichage),
  - un callback `onParameterChanged()` (logique metier appliquee apres edition).

