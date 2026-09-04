# FabOS — référentiel visuel Équipement / Accès

Références de conception uniquement ; aucune image ne constitue une implémentation ni un nouvel écran hors du système FabOS. Le développeur doit conserver shells, composants, droits et thèmes canoniques existants.

## Rapport de revue

- `REVIEW-SOL.md` — revue UX/Sécurité complète, parcours, priorités et prompt d'implémentation.

## Maquettes, dans l'ordre recommandé

1. `equipment-operational-overview.png` — entrée admin Équipement ; éléments à corriger.
2. `equipment-access-incidents.png` — incidents RFID actionnables.
3. `equipment-reader-commissioning.png` — création, association, secret API-only révélé une fois, première connexion.
4. `equipment-reader-health.png` — santé, test, rotation/révocation et événements d'un lecteur.
5. `equipment-machine-member-detail.png` — fiche publique task-first : droit, réservation, matériaux et sécurité.
6. `equipment-material-detail.png` — fiche matériau et machines réellement compatibles.
7. `equipment-machine-operations.png` — zone Exploitation staff/admin d'une machine.
8. `equipment-machine-kiosk.png` — kiosk public thémé, sans données personnelles.

## Invariants de mise en œuvre

- Un seul workspace admin **Équipement** : Vue d'ensemble, Machines, Matériaux, Accès.
- Aucune navigation, footer, CSS ou thème de page local ; les patterns durables vont dans `/admin/design`.
- La relation `Material ↔ Machine` est la source de vérité pour la compatibilité.
- API/device fail-closed en production ; secret de boîtier API-only, unique, révocable et rotatable ; jamais de credentials base de données.
- Kiosk public sans PII ; shell kiosk consommant le thème publié.
- Toute action affichée doit avoir un chemin réel, vérifié et autorisé côté serveur.
