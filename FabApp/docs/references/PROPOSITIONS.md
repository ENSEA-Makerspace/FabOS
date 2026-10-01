# Propositions de pages — le tri des planches

Demande de l’opérateur (2026-10-01) : revoir chaque planche de `public/images/references/` face à la page qui existe, en garder les bonnes idées (contenu, ordre, mise en page) — **jamais une copie** — et poser une proposition en sous-page du menu Développement quand la planche fait mieux.

Tri du 2026-10-01 (lecture des planches et des gabarits ; la valeur /5 = gain pour l'utilisateur). Légende : ✅ déjà appliqué · 🟡 proposition à regarder · 🟢 notre page fait déjà aussi bien ou mieux · ⏳ pas encore comparée · 🅿️ pas de page en face (fonction absente).

## espaces

| Planche | Verdict |
|---|---|
| `01-catalogue-espaces.png` | 🟡 [Proposition](/admin/propositions/espaces-catalogue) — « Libre maintenant » en tête, la carte dit quand, bouton « Réserver ». |
| `02-detail-espace-reservation.png` | ✅ Appliqué (2026-10-01) — tête partagée `_detail_hero`. |
| `03-parcours-reservation.png` | 🟡 à proposer (2/5) — politique d'annulation et nombre de participants (borné par la capacité) dans le panneau de réservation. |
| `04-exploitation-espaces.png` | 🟡 [Proposition](/admin/propositions/admin-attention) (4/5) — fusionné avec « vue d'ensemble Équipement » : `/admin` en blocs « ce qui demande une action ». |
| `05-mise-en-service-point-acces.png` | 🟡 à proposer (2/5) — la checklist `_commissioning` sur la fiche du point d'accès. |
| `06-incidents-acces.png` | 🟡 [Proposition](/admin/propositions/incidents-acces) (3/5) — tuiles par cause avec compteur, « À traiter » par défaut ; santé des points. |
| `07-kiosque-entree.png` | 🅿️ pas de page (2/5) — écran de porte « Badgez votre carte » par lieu. |
| `08-mes-reservations.png` | 🟡 [Proposition](/admin/propositions/mes-reservations) — la prochaine en tête (quand, quoi, fenêtre de porte), le reste en lignes datées. |

## equipement

| Planche | Verdict |
|---|---|
| `equipment-access-incidents.png` | 🟡 [Proposition](/admin/propositions/incidents-acces) (3/5) — même page que espaces/06. Notre verbe correctif (`_cell_fix`) fait déjà mieux que « Action recommandée ». |
| `equipment-machine-kiosk.png` | 🟢 égal (1/5) — disponibilité, badge, prérequis, matériaux, horaires : déjà là. |
| `equipment-machine-member-detail.png` | ✅ Appliqué (2026-09-30) — fiche machine « tâche d’abord ». |
| `equipment-machine-operations.png` | 🟡 à proposer (3/5) — zone Exploitation en cartes actionnables (hors service, prochaine vérification, lecteur, incidents). |
| `equipment-material-detail.png` | ✅ Appliqué (2026-10-01) — tête partagée `_detail_hero`. |
| `equipment-operational-overview.png` | 🟡 [Proposition](/admin/propositions/admin-attention) (5/5) — `/admin` : groupes d'action par domaine, chacun avec le verbe qui mène à la ligne. |
| `equipment-reader-commissioning.png` | 🟡 à proposer (2/5) — remonter la checklist de mise en service au-dessus du formulaire. |
| `equipment-reader-health.png` | 🟡 à proposer (3/5) — en-tête d'état (prêt / hors ligne), machine associée, derniers événements. |

## users

| Planche | Verdict |
|---|---|
| `01-creation-compte.png` | 🟢 égal (1/5) — « Et après ? », étapes, CGU, mot de passe visible : déjà. |
| `02-confirmation-email.png` | 🟢 égal (1/5) — renvoyer, corriger l'adresse, « pas reçu ? » : déjà. |
| `03-profil-adhesion.png` | 🅿️ pas de page (1/5) — pas d'adhésion payante dans FabOS. |
| `04-accueil-membre.png` | 🟡 [Proposition](/admin/propositions/accueil-membre) (4/5) — « À faire » avec un verbe par ligne, prochaine réservation, mes accès. |
| `05-profil-securite.png` | 🟡 à proposer (2/5) — une section « Sécurité » à part, une ligne par chose ; export de mes données. |
| `06-droits-et-acces-admin.png` | 🟢 égal (1/5) — le même composant que le membre voit (`_usage_rights_summary`). |
| `07-validation-inscription.png` | 🅿️ pas de page (2/5) — pas de flux valider / demander / refuser une inscription. |
| `08-annuaire-utilisateurs.png` | 🟡 [Proposition](/admin/propositions/annuaire-utilisateurs) (3/5) — tuiles « À valider / Suspendu / Expire bientôt », formations en « 3/4 », dernière activité. |

## productwide

| Planche | Verdict |
|---|---|
| `01-evenements.png` | 🟡 [Proposition](/admin/propositions/evenements) (3/5) — « Mes inscriptions » avant la grille ; places restantes sur les cartes. |
| `02-prets.png` | 🟡 à proposer (3/5) — « Enregistrer le retour » sur la fiche de l'objet ; tuile « À rendre aujourd'hui ». |
| `03-materiaux-equipement.png` | 🟢 égal (1/5) — pas de donnée de stock ; la fiche a les machines compatibles. |
| `04-maintenance.png` | 🟡 à proposer (2/5) — verbe par ligne (« Démarrer » / « Terminer ») et tuile « Cette semaine ». |
| `05-configuration.png` | 🟢 égal (1/5) — cartes par domaine avec état résumé : déjà (S132). |
| `06-creations.png` | 🟢 égal (1/5) — grille, auteur, épingle, partage : déjà. |
| `07-kiosque.png` | 🅿️ pas de page (2/5) — accueil de borne : trois grosses tuiles. |

## coordination

| Planche | Verdict |
|---|---|
| `01-calendrier.png` | 🟢 égal (1/5) — semaine/mois, prochain créneau : déjà ; filtres retirés exprès (S146c). |
| `02-rendez-vous-personne.png` | 🟡 à proposer (2/5) — « prochain créneau » en tête, bande de jours cliquables. |
| `03-groupes.png` | 🟢 égal (1/5) — membres, rôle, validité, lots de droits : déjà. |
| `04-mon-badge.png` | 🟢 égal (1/5) — `_badge_reach` dans le profil ; à ajouter peut-être « dernière utilisation ». |
| `05-recherche.png` | 🟡 [Proposition](/admin/propositions/recherche) (3/5) — accès rapides avant saisie, filtre par type avec compteurs. |
| `06-preferences-email.png` | 🟡 à proposer (2/5) — regrouper par domaine, « toujours activé » non modifiable. |

## formations

| Planche | Verdict |
|---|---|
| `lms-course-module.png` | 🅿️ pas de page (2/5) — pas de modèle de module. |
| `lms-my-trainings.png` | 🟡 [Proposition](/admin/propositions/mes-formations) (4/5) — « Mes formations » : à reprendre, en cours, terminées. |
| `lms-practical-exercise-file.png` | 🅿️ pas de page (1/5) — exercices pratiques non modélisés. |
| `lms-practical-validations-queue.png` | 🟢 égal (1/5) — bâtie sur cette planche (S180). |
| `lms-quiz-question-types.png` | 🟢 égal (1/5) — types de question et progression : déjà. |
| `lms-quiz-result-retry.png` | 🟢 égal (1/5) — score, points à revoir, rejouer : déjà. |
| `lms-staff-practical-validation.png` | 🟡 à proposer (3/5) — checklist de compétences ; exige un modèle de checklist. |
| `lms-training-builder.png` | 🟡 à proposer (2/5) — un seul parcours ordonné (modules, quiz, validation). |
| `lms-training-catalogue.png` | 🟡 à proposer (3/5) — verbe de carte selon l'état (Commencer / Continuer) et « À reprendre ». |
| `lms-training-certificate-badge.png` | 🟡 à proposer (2/5) — page de réussite : « vous êtes autorisé à… ». |
| `lms-training-overview.png` | 🟢 égal (1/5) — prochaine étape et parcours : déjà (S179). |

## finalsurface

| Planche | Verdict |
|---|---|
| `01-accueil-configurable.png` | 🟡 à proposer (2/5) — « Ouvert aujourd'hui », espaces et équipements libres du jour. |
| `02-acces-exceptionnels.png` | 🟢 égal (1/5) — motif, portée, validité, révocation : déjà. |
| `03-rapports.png` | 🟡 à proposer (2/5) — lignes cliquables, bande « à retenir » avec un verbe. |
| `04-recuperation-compte.png` | 🟢 égal (1/5) — réponse non divulgante : déjà. |
