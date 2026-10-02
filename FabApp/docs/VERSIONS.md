# Versions de FabOS

**Règle** (2026-10-02) : `0.MINEUR.CORRECTIF` tant que FabOS n'est pas en service
à l'ENSEA ; **1.0.0 = la mise en service**.
- **MINEUR** : un lot visible pour les usagers, livré et testé ensemble
  (ex. : les pages revues d'après les planches).
- **CORRECTIF** : des corrections entre deux lots, sans changement de parcours.

Le numéro vit à un seul endroit : `config/packages/twig.yaml` → `app_version`
(affiché en pied de page). Chaque version a son étiquette git `vX.Y.Z`.

| Version | Date | Ce qu'elle contient |
|---|---|---|
| **0.5.0** | en cours | Les pages revues d'après les 52 planches (Phase V), mises en place pour les tests usagers. |
| **0.4.0** | 2026-10-01 | Tout ce qui précède : phases J à U (réservations, espaces et accès, formations, e-mails, thèmes, comptes et identité OIDC), jeu de démo du lab, fiches « tâche d'abord ». Point de départ de la numérotation. |

Les numéros 0.1 à 0.3 ne sont pas attribués rétroactivement : l'historique par
phase reste dans `HISTORY.md`.
