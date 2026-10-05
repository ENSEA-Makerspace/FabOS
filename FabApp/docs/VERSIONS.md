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
| **0.6.2** | 2026-10-06 | Ménage interne, sans changement visible : quatre motifs recopiés à la main deviennent des gabarits communs (`_next_card`, `_dated_rows`, `_steps`, `_hcard`) ; HTML des pages identique avant/après. |
| **0.6.1** | 2026-10-06 | Le tutoriel des designs (`/admin/design`) documente les motifs apparus en 0.5 et 0.6 (tête de fiche, lignes datées, À faire, étapes, tuiles de choix, liste de motifs, page de compte, fonction activable) ; formulaire public étroit (`.form-narrow`) sur le signalement de panne. |
| **0.6.0** | 2026-10-05 | Phase W, six fonctions activables reprises de la comparaison avec FabtrackJS : signaler une panne par QR code (S205), stocks de consommables — éteints par défaut, aucune mention quand éteints (S206), check-in à paliers — éteint par défaut (S207), avertissements (S208), charte de sécurité — éteinte par défaut (S209), bouton « Signaler » sur chaque page (S210). ⚠️ Demande une migration. |
| **0.5.1** | 2026-10-02 | « Mon compte » : `/profil` refait — en-tête compact, vrais onglets (Aperçu, Mes accès, Mon activité, Réglages), réglages en lignes qui s'ouvrent sur place ; aucun champ ni réglage perdu (inventaire contrôlé), tableaux vides tus. |
| **0.5.0** | 2026-10-02 | Les pages revues d'après les 52 planches (Phase V), mises en place pour les tests usagers : accueil (hero + horaires + événements, puis « À faire » et ce qui est libre), espaces, mes réservations, recherche, événements, mes formations, lecture d'une étape, badge, profil (sécurité et e-mails), fiche machine (zone équipe), confirmation de réservation, rendez-vous, borne `/kiosk` ; admin : tableau de bord « ce qui demande votre attention », incidents d'accès, annuaire, maintenance, rapports, prêts au comptoir, fiches lecteur et point d'accès, dossier de validation pratique, parcours d'une formation. |
| **0.4.0** | 2026-10-01 | Tout ce qui précède : phases J à U (réservations, espaces et accès, formations, e-mails, thèmes, comptes et identité OIDC), jeu de démo du lab, fiches « tâche d'abord ». Point de départ de la numérotation. |

Les numéros 0.1 à 0.3 ne sont pas attribués rétroactivement : l'historique par
phase reste dans `HISTORY.md`.
