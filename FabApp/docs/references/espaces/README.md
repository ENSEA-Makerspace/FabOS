# FabOS — référentiel Espaces et accès

Référentiel de conception, sans changement de code FabOS. Les maquettes expriment le parcours cible : **découvrir → vérifier → réserver → accéder → terminer**.

## Écrans

1. `01-catalogue-espaces.png` — catalogue et disponibilité lisible dès la carte.
2. `02-detail-espace-reservation.png` — décision : contenu, contraintes, créneau et accès.
3. `03-parcours-reservation.png` — confirmation courte et explicite.
4. `04-exploitation-espaces.png` — file d’action de l’équipe, pas un dashboard décoratif.
5. `05-mise-en-service-point-acces.png` — appairage sécurisé d’un futur boîtier de porte.
6. `06-incidents-acces.png` — incidents actionnables, avec confidentialité minimale.
7. `07-kiosque-entree.png` — écran public d’entrée, sans identité ni secret.
8. `08-mes-reservations.png` — prochaine réservation et accès temporaire immédiatement visibles.

## Règles de produit à préserver

- **Lieu** : site ou sous-lieu et ses horaires/politiques d’entrée.
- **Espace** : salle ou zone réservable située dans un lieu.
- **Point d’accès** : porte, portail, casier ou zone, distinct d’une machine ; un boîtier lui est associé.
- Une réservation peut donner un accès temporaire, limité à une courte marge avant/après le créneau et révoqué dès annulation.
- Toute action courante doit être atteignable en un clic après le filtre de lieu ; la recherche seule nécessite une saisie.
- Les kiosques sont publics : aucune identité membre, UID badge, journal ni secret ne doit y figurer.

Voir `REVIEW-SOL.md` pour les constats, priorités et le prompt d’implémentation.
