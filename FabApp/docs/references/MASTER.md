# FabOS — master handoff de conception

Tous les packs sont prêts à être remis à l’agent d’implémentation :

- `fabos-training-references/` — parcours LMS, quiz, pratiques, validation staff.
- `fabos-equipment-references/` — machines, matériaux, boîtiers et accès.
- `fabos-spaces-references/` — espaces, réservation, points d’accès et kiosks d’entrée.
- `fabos-users-references/` — création de compte, adhésion, droits et administration utilisateurs.
- `fabos-product-wide-references/` — événements, prêts, stock, maintenance, configuration, créations, kiosks.
- `fabos-coordination-references/` — calendrier, rendez-vous, groupes, badge, recherche, e-mail.
- `fabos-final-surface-references/` — accueil configurable, accès exceptionnel, rapports, récupération.

## Ordre d’exécution conseillé

1. Système partagé : shell admin, footer, design tokens, filtres, états, formulaires, navigation, traductions.
2. Sécurité et droits : access points/fail closed, comptes, rôles, adhésion, formations, accès temporaires, audit.
3. Parcours principaux : machines/équipement, espaces, formation, événements, prêts.
4. Exploitation : maintenance, incidents, rapports, kiosks et configuration.
5. Éditorial et découverte : pages personnalisées, accueil, recherche, créations, préférences.

## Critère de sortie de chaque lot

Une action quotidienne se fait avec le minimum de clics, sans CSS/navigation locale ; tous les objets ouvrent leur fiche ; les états vides et erreurs expliquent la suite ; les droits sont justifiables ; la page respecte `/admin/design`; docs, tests, Artemis et commit sont effectués.
