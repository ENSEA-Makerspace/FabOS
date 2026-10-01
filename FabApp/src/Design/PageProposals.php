<?php

declare(strict_types=1);

namespace App\Design;

/**
 * Les pages revues d'après les planches de référence, proposées à l'opérateur
 * AVANT d'être construites (demande du 2026-10-01 : « des sous-pages dans le
 * menu dev », pas des branches).
 *
 * Une proposition = une entrée ici + un gabarit `site/proposals/{slug}.html.twig`.
 * Le menu Développement en fait une sous-page chacune ; l'index
 * (`/admin/propositions`) rend le tri complet des planches
 * (`docs/references/PROPOSITIONS.md`).
 *
 * ⚠️ Une proposition ne copie pas la planche : elle en garde les bonnes idées
 * (contenu, ordre, mise en page) et les pose sur NOS composants et NOS jetons.
 * `demo` = des liens de démonstration (paramètres lus par la seule proposition).
 * `take` = ce qu'on reprend de la planche ; `keep` = ce qu'on garde de chez nous
 * parce que c'est mieux.
 *
 * 🔴 **Retenue ou refusée, une proposition se SUPPRIME** — entrée, gabarit, CSS
 * `pp-*` qui ne sert qu'à elle. Le raisonnement va dans `HISTORY.md`.
 */
final class PageProposals
{
    /**
     * @return list<array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>, demo?: array<string, string>}>
     */
    public function all(): array
    {
        return [
            [
                'slug' => 'espaces-catalogue',
                'title' => 'Catalogue des espaces',
                'planche' => 'espaces/01-catalogue-espaces.png',
                'current' => ['route' => 'app_places', 'params' => [], 'label' => '/places'],
                'shell' => 'public',
                'take' => [
                    'Un bandeau « Libre maintenant » en tête : ce qu’on peut réserver tout de suite, sans parcourir la grille.',
                    'Des cartes qui portent le verbe : un vrai bouton « Réserver » quand vous le pouvez, au lieu d’un petit lien « Voir ».',
                    'Capacité et emplacement lisibles d’un coup d’œil, chacun avec son pictogramme.',
                ],
                'keep' => [
                    'La séparation par POSITION : la tête dit l’état de la salle, le pied dit votre droit. La planche met « Indisponible » sur un bouton sans dire si c’est la salle ou vous.',
                    'Une image par défaut propre à un espace (le plan), plus l’icône d’imprimante empruntée aux machines.',
                ],
                'demo' => ['?vue=ouvert' => 'labo ouvert'],
            ],
            [
                'slug' => 'mes-reservations',
                'title' => 'Mes réservations',
                'planche' => 'espaces/08-mes-reservations.png',
                'current' => ['route' => 'app_my_reservations', 'params' => [], 'label' => '/mes-reservations'],
                'shell' => 'public',
                'take' => [
                    'LA prochaine réservation (ou celle en cours) en tête : quand (« Aujourd’hui · 14:00–17:00 »), quoi, et pour un espace, la fenêtre où votre badge ouvre la porte.',
                    'Le reste en lignes datées — « À venir », puis « Passées » estompées : une réservation se lit par sa date, pas par une affiche.',
                    'Un seul verbe par ligne (« Gérer »).',
                ],
                'keep' => [
                    'Le détail d’une réservation (déplacer, annuler, rétablir) reste sur sa page : pas de mur de boutons dans la liste.',
                    'Les annulées et refusées restent consultables, repliées en bas — la planche les oublie.',
                ],
                'demo' => ['?membre=45' => 'avec les réservations d’alice'],
            ],
            [
                'slug' => 'accueil-membre',
                'title' => 'Accueil d’un membre connecté',
                'planche' => 'users/04-accueil-membre.png',
                'current' => ['route' => 'app_home', 'params' => [], 'label' => '/'],
                'shell' => 'public',
                'take' => [
                    '« À faire » en tête : ce qui bloque ou attend (objet à rendre, formation à terminer, demande en attente, premier badge), chacun avec le verbe qui le règle.',
                    '« Ma prochaine réservation » à un clic, et « Mes accès » à côté.',
                ],
                'keep' => [
                    '« Mes accès » est le composant du profil (`_badge_reach`), pas une copie : ce que le badge ouvre se dit à un seul endroit.',
                    'L’accueil actuel (événements, horaires, personnalisation) reste dessous, inchangé. La planche n’a que le bloc personnel.',
                    'Pas d’« adhésion valide jusqu’au » : FabOS n’a pas d’adhésion payante.',
                ],
                'demo' => ['?membre=45' => 'alice', '?membre=55' => 'frank'],
            ],
            [
                'slug' => 'mes-formations',
                'title' => 'Mes formations',
                'planche' => 'formations/lms-my-trainings.png',
                'current' => ['route' => 'app_profile', 'params' => [], 'label' => '/profil#stats'],
                'shell' => 'public',
                'take' => [
                    'Une page à soi : « À reprendre » (la plus avancée, ses étapes en ligne, le verbe de l’étape suivante), « En cours », « Terminées ».',
                    'Chaque ligne porte SON verbe (« Continuer le cours », « Passer les quiz »), au lieu d’un « Voir » identique partout.',
                    'En bas, « Découvrir d’autres formations » : les cartes du catalogue, dont le verbe dit « Commencer » (planche `lms-training-catalogue.png`).',
                ],
                'keep' => [
                    'Les étapes et le verbe viennent de `LearnerJourney` (S179), les mêmes que la fiche formation : aucune seconde arithmétique.',
                    'Pas de « Demander une évaluation pratique » : finir la théorie place déjà dans la file de l’équipe (S180).',
                    'Pas de certificat PDF : il n’existe pas ; « Terminées » mène au badge obtenu.',
                ],
                'demo' => ['?membre=3' => 'un membre avec 4 formations', '?membre=2' => 'un autre'],
            ],
            [
                'slug' => 'admin-attention',
                'title' => 'Accueil admin : ce qui demande votre attention',
                'planche' => 'equipement/equipment-operational-overview.png',
                'current' => ['route' => 'app_admin_dashboard', 'params' => [], 'label' => '/admin'],
                'shell' => 'admin',
                'take' => [
                    '« N éléments demandent votre attention » en tête, puis un groupe par chose à faire, avec son compteur dans le titre.',
                    'Une ligne par élément (quoi, où/quand, état) et UN verbe qui mène à la bonne fiche ou liste.',
                    'Un groupe vide disparaît ; tout vide, une ligne « Rien ne demande votre attention ».',
                ],
                'keep' => [
                    'Les sept compteurs actuels et les écrans dédiés ne disparaissent pas : ils descendent, en une ligne chacun.',
                    'Le verbe correctif des refus d’accès est le composant `_cell_fix` / `AccessIncident`, pas une copie.',
                    'Pas de couleurs de la planche : états par `_state_chip`, icônes par `_icon`, jetons du thème.',
                ],
            ],
            [
                'slug' => 'recherche',
                'title' => 'Recherche',
                'planche' => 'coordination/05-recherche.png',
                'current' => ['route' => 'app_recherche', 'params' => [], 'label' => '/recherche'],
                'shell' => 'public',
                'take' => [
                    'Avant la saisie, des accès rapides (mes réservations, machines, formations, événements) au lieu de trois conseils sur la façon de chercher.',
                    'Les types deviennent un filtre à un clic, « Tout » en premier, chacun avec son compteur.',
                    'Des lignes avec le pictogramme du type, et « Voir tout » vers le catalogue déjà filtré par la saisie.',
                ],
                'keep' => [
                    'Les résultats sont ceux de `SiteSearch` tels quels, « Pages du site » en tête (taper « horaires » mène aux horaires).',
                    'Pas de « récents » : FabOS ne garde pas l’historique de navigation, et n’a pas à le faire.',
                ],
                'demo' => ['?q=laser' => '« laser »', '?q=laser&type=machines' => '« laser », machines seulement'],
            ],
            [
                'slug' => 'evenements',
                'title' => 'Événements',
                'planche' => 'productwide/01-evenements.png',
                'current' => ['route' => 'app_events', 'params' => [], 'label' => '/events'],
                'shell' => 'public',
                'take' => [
                    '« Mes inscriptions » AVANT la grille : le prochain événement auquel on est inscrit en grand (quand, où, place ou liste d’attente) avec ses vraies actions, « Billet » et « Gérer ».',
                    'Ses autres inscriptions à venir en lignes datées, un seul verbe par ligne.',
                    'Les places restantes lisibles sur la carte.',
                ],
                'keep' => [
                    'La grille actuelle, inchangée (même partiel `_catalogue_card`, mêmes signaux) : états ouvert/complet/terminé/annulé, regroupement par mois, droit d’usage au pied.',
                    'Le prix n’existe pas sur `Event` : il n’est pas inventé.',
                    'L’annulation reste sur la fiche de l’événement (« Gérer »), pas de mur de boutons dans la liste.',
                ],
                'demo' => [],
            ],
            [
                'slug' => 'incidents-acces',
                'title' => 'Incidents d’accès',
                'planche' => 'espaces/06-incidents-acces.png',
                'current' => ['route' => 'app_admin_access_rfid_logs', 'params' => [], 'label' => '/admin/access-rfid-logs'],
                'shell' => 'admin',
                'take' => [
                    'Des tuiles par CAUSE avec compteur (badge manquant, formation manquante, badge inconnu, lecteur ou machine, compte inactif, erreur serveur).',
                    '« À traiter » sélectionné par défaut (les refus des 7 derniers jours) au lieu du seul filtre Oui/Non.',
                    'Un petit panneau « Santé des lecteurs » à côté : en ligne / hors ligne / à configurer.',
                    'Période, lecteur et machine repliés sous « Affiner ».',
                ],
                'keep' => [
                    'Le verbe correctif par ligne (« Accorder le badge », « Réactiver le boîtier »…) : `AccessIncident`, mieux que la planche et son lien générique.',
                    'Aucune donnée personnelle de plus que la page actuelle.',
                    'Pas de tuile « hors plage horaire » : le journal n’écrit pas ce statut, on ne l’invente pas.',
                ],
                'demo' => ['?cause=badge' => 'tuile « Badge manquant »', '?cause=all&days=30' => 'tout le journal, 30 jours'],
            ],
            [
                'slug' => 'annuaire-utilisateurs',
                'title' => 'Annuaire des utilisateurs',
                'planche' => 'users/08-annuaire-utilisateurs.png',
                'current' => ['route' => 'app_admin_users', 'params' => [], 'label' => '/admin/utilisateurs'],
                'shell' => 'admin',
                'take' => [
                    'Des tuiles orientées travail, chacune avec son compteur : « À valider », « Suspendus ou inactifs », « Expire bientôt » (appartenance de groupe qui prend fin dans 30 jours), « Sans badge ».',
                    'Les formations en « 3 / 4 » avec leur jauge, au lieu d’un entier « progressions ».',
                    '« Dernière activité » (dernier passage de badge ou dernière connexion) au lieu du compteur brut de passages RFID.',
                    'Un verbe qui dit la tâche : « Examiner » quand la ligne demande une décision, « Ouvrir » sinon.',
                ],
                'keep' => [
                    'La coquille de liste admin : recherche, compteur, bouton de création, `_data_table` avec action épinglée.',
                    'Les filtres Groupe et Forfait de la page réelle (non rejoués ici) : ils passent par `AudienceResolver`.',
                    'Le statut « adresse non confirmée », que la planche n’a pas.',
                ],
                'demo' => ['?tuile=valider' => 'à valider', '?tuile=expire' => 'expire bientôt', '?tuile=sans-badge' => 'sans badge'],
            ],
            [
                'slug' => 'prets-admin',
                'title' => 'Prêts (comptoir)',
                'planche' => 'productwide/02-prets.png',
                'current' => ['route' => 'app_admin_loans', 'params' => [], 'label' => '/admin/loans'],
                'shell' => 'admin',
                'take' => [
                    'Des tuiles de TRAVAIL avec compteur : « À rendre aujourd’hui », « En retard », « En cours » (vue par défaut), « Rendus ».',
                    'Le plus pressé en tête (retard le plus ancien, puis échéance la plus proche), avec « +N jours » sous l’échéance d’un retard.',
                    'UN verbe par ligne non rendue : « Enregistrer le retour ». « Nouveau prêt » reste en haut à droite.',
                ],
                'keep' => [
                    'Le retour se fait sur la fiche de l’objet, ancrée sur le prêt (S193) : on y voit qui l’a et depuis quand.',
                    'La coquille `_admin_list`, `_data_table` et `_state_chip` : pas de seconde liste.',
                    'Pas d’accessoires cochés au retour ni de « Relancer » : ni l’un ni l’autre n’existe (les rappels de retard partent seuls).',
                ],
                'demo' => ['?tuile=aujourdhui' => 'à rendre aujourd’hui', '?tuile=retard' => 'en retard', '?tuile=rendus' => 'rendus'],
            ],
            [
                'slug' => 'fiche-lecteur',
                'title' => 'Fiche d’un lecteur',
                'planche' => 'equipement/equipment-reader-health.png',
                'current' => ['route' => 'app_admin_rfid_readers', 'params' => [], 'label' => '/admin/rfid-readers/{id}/edit'],
                'shell' => 'admin',
                'take' => [
                    'L’état d’abord : « Prêt » / « Hors ligne depuis … » / « À configurer », le dernier contact, et la machine ou le point d’accès commandé.',
                    'Les 5 derniers événements du lecteur, avec un lien vers le journal filtré.',
                    'Tant que le lecteur n’est pas en service, la mise en service monte en haut, étape bloquante dite en clair (planche `equipment-reader-commissioning.png`).',
                    'Le formulaire d’édition descend : un lien « Modifier les réglages ».',
                ],
                'keep' => [
                    'L’état calculé par `ReaderHealth` et la checklist `ReaderCommissioning`.',
                    'Aucun secret ni jeton affiché ; pas de « Tester la connexion » ni de version du boîtier : rien ne les alimente.',
                ],
                'demo' => [],
            ],
            [
                'slug' => 'exploitation-machine',
                'title' => 'Exploitation d’une machine',
                'planche' => 'equipement/equipment-machine-operations.png',
                'current' => ['route' => 'app_machines', 'params' => [], 'label' => '/machines/{id} (zone staff)'],
                'shell' => 'public',
                'take' => [
                    'Des cartes qui portent chacune un fait ET un verbe : « Mettre hors service » / « Remettre en service », « Ouvrir la tâche » / « Planifier », « Ouvrir le lecteur ».',
                    'La prochaine maintenance avec son échéance, au lieu d’un compteur de lignes de journal.',
                    'Le lecteur associé avec son état et son dernier contact, et les derniers refus de CETTE machine avec le verbe correctif.',
                ],
                'keep' => [
                    'La zone reste réservée au personnel ; les boutons mènent aux pages réelles qui écrivent.',
                    'L’état du lecteur vient de `ReaderHealth`, le verbe correctif d’`AccessIncident` / `_cell_fix`.',
                ],
                'demo' => [],
            ],
            [
                'slug' => 'maintenance',
                'title' => 'Maintenance : une file d’intervention',
                'planche' => 'productwide/04-maintenance.png',
                'current' => ['route' => 'app_admin_maintenance', 'params' => [], 'label' => '/admin/maintenance'],
                'shell' => 'admin',
                'take' => [
                    'Des tuiles À faire / En retard / Cette semaine / Fait, cliquables : on ouvre la file par ce qui presse.',
                    'Chaque ligne dit la machine (photo ou pictogramme, lien vers sa fiche), la tâche, l’échéance et l’état.',
                    'Le verbe principal d’une tâche ouverte est « Marquer faite » ; « Modifier » passe en lien secondaire.',
                ],
                'keep' => [
                    'Mêmes données et même statut effectif que la liste actuelle ; « Cette semaine » = échéance dans 7 jours.',
                    'Pas de pourcentage de disponibilité, pas de panneau de détail ni de checklist : FabOS n’a pas ces données.',
                ],
                'demo' => ['?tuile=overdue' => 'En retard', '?tuile=week' => 'Cette semaine', '?tuile=done' => 'Fait'],
            ],
            [
                'slug' => 'profil-securite-emails',
                'title' => 'Sécurité et e-mails du profil',
                'planche' => 'users/05-profil-securite.png',
                'current' => ['route' => 'app_profile', 'params' => [], 'label' => '/profil#settings'],
                'shell' => 'public',
                'take' => [
                    'Une section « Sécurité » à part, UNE ligne par chose avec son état et son verbe : mot de passe, adresse vérifiée ou non, double authentification (« Recommandé » tant qu’elle n’est pas active), sessions ouvertes. « Supprimer mon compte » en bas, discret.',
                    'Les e-mails regroupés par domaine (planche `coordination/06-preferences-email.png`) ; les essentiels en « Toujours envoyé », les optionnels avec une phrase qui dit ce qu’on reçoit.',
                ],
                'keep' => [
                    'Les états viennent des vrais services du profil (`MfaService`, `SessionRegistry`, `NotificationPreferences`).',
                    '« Essentiel » = hors `NotificationCategory::OPTOUTABLE`, pas un libellé inventé ; l’interrupteur général d’e-mail est respecté.',
                    'Fermer une session, activer la double authentification et supprimer le compte restent chacun sur leur page.',
                ],
                'demo' => ['?membre=45' => 'alice', '?membre=48' => 'bob'],
            ],
            [
                'slug' => 'parcours-reservation',
                'title' => 'Réserver un espace (confirmation)',
                'planche' => 'espaces/03-parcours-reservation.png',
                'current' => ['route' => 'app_places', 'params' => [], 'label' => '/places/{id} (panneau de réservation)'],
                'shell' => 'public',
                'take' => [
                    'Une confirmation courte avant d’envoyer : l’espace, le jour, l’horaire, la durée et la capacité côte à côte.',
                    'La politique d’annulation dite AVANT de réserver (« Annulable jusqu’au … »), et ce que le badge ouvrira (« de HH:MM à HH:MM »).',
                    'L’écran d’après : « C’est réservé » avec les prochaines étapes.',
                ],
                'keep' => [
                    'Le calendrier semaine/mois et le panneau ouvert au clic sur un créneau libre : pas de parcours « étape 1 sur 2 ».',
                    'Le motif reste optionnel. Pas de « nombre de participants » : il n’existe pas en base.',
                    'Politique et porte ne s’affichent que si la donnée existe.',
                ],
                'demo' => ['?membre=45' => 'avec alice (délai d’annulation, iCal)'],
            ],
            [
                'slug' => 'badge-obtenu',
                'title' => 'Badge obtenu',
                'planche' => 'formations/lms-training-certificate-badge.png',
                'current' => ['route' => 'app_badges', 'params' => [], 'label' => '/badges/{id}'],
                'shell' => 'public',
                'take' => [
                    '« Vous êtes autorisé·e à … » en tête : ce que le badge ouvre (machines et pièces), chacune avec son image et un bouton « Réserver ».',
                    '« Obtenu le … » et la formation qui l’a donné, avec son parcours en étapes faites.',
                    'Pour qui ne le détient pas : « Ce que ce badge ouvrirait » et « Commencer la formation ».',
                ],
                'keep' => [
                    'Pas de « Télécharger mon certificat » : le certificat PDF n’existe pas.',
                    'Le parcours est celui de `LearnerJourney` (mêmes libellés), pas une seconde logique.',
                    'Les pièces comptent aussi (`PLACE_BADGE`), pas seulement les machines.',
                ],
                'demo' => ['?membre=45' => 'vu par alice'],
            ],
            [
                'slug' => 'rendez-vous',
                'title' => 'Rendez-vous avec une personne',
                'planche' => 'coordination/02-rendez-vous-personne.png',
                'current' => ['route' => 'app_trainers', 'params' => [], 'label' => '/personnes/{id}/reserver'],
                'shell' => 'public',
                'take' => [
                    'En tête, la personne avec « Prochain créneau disponible » en évidence et un bouton qui le prend.',
                    'Une bande de 14 jours cliquables avec le nombre de créneaux libres ; seulement les créneaux du jour choisi, au lieu de tous les jours empilés.',
                    'À droite, « Votre rendez-vous » qui se remplit au choix du créneau.',
                ],
                'keep' => [
                    'Les créneaux viennent de `PersonAvailabilityService::dailySlots()`, le même appel que la page actuelle.',
                    'La demande libre pour un autre moment et la confirmation (droits d’usage) restent sur la vraie page.',
                ],
                'demo' => [],
            ],
            [
                'slug' => 'rapports',
                'title' => 'Rapports : ce qu’il faut retenir, puis agir',
                'planche' => 'finalsurface/03-rapports.png',
                'current' => ['route' => 'app_admin_reporting', 'params' => ['workspace' => 'equipment'], 'label' => '/admin/reporting/equipment'],
                'shell' => 'admin',
                'take' => [
                    'Une bande « À retenir » en tête : trois constats courts tirés des mêmes chiffres, chacun avec UN verbe vers la fiche ou la liste où l’on agit.',
                    'Chaque ligne du classement ouvre la fiche de la machine ou de l’espace, avec sa part des réservations.',
                    'La période en trois tuiles (7 / 30 / 90 jours), comparée à la période précédente.',
                ],
                'keep' => [
                    'Les chiffres viennent du même adaptateur que la page actuelle (`ReportingRegistry`).',
                    'Dates libres, tableau jour par jour et export CSV restent sur la page réelle.',
                    'Pas de graphique inventé : le reporting n’a pas de série à tracer.',
                ],
                'demo' => ['?espace=spaces' => 'espaces', '?jours=90' => '90 jours'],
            ],
            [
                'slug' => 'validation-pratique',
                'title' => 'Validation pratique',
                'planche' => 'formations/lms-staff-practical-validation.png',
                'current' => ['route' => 'app_admin_practical_queue', 'params' => [], 'label' => '/admin/validations-pratiques'],
                'shell' => 'admin',
                'take' => [
                    'Un ÉCRAN de validation pour une personne × une formation, ouvert depuis la file : qui, quelle formation, ce qu’elle a déjà fait (théorie, quiz x/y).',
                    '« À observer sur la machine » : des points à regarder, « Acquis / À revoir » par ligne.',
                    '« Ce que cette validation ouvre » : le badge et ses machines, avant de décider.',
                ],
                'keep' => [
                    'La file se DÉDUIT (finir la théorie est la demande) : rien à demander.',
                    'Le vrai geste reste sur la fiche utilisateur : « Valider » y mène, l’écran n’écrit rien.',
                    '⚠️ Une vraie checklist enregistrée (points, note, évaluateur, tentatives) demanderait un modèle NEUF : ici les points viennent du texte « objectifs » de la formation, et « Acquis / À revoir » est un exemple visuel.',
                ],
                'demo' => ['?dossier=1' => 'dossier suivant'],
            ],
        ];
    }

    /** @return array{slug: string, title: string, planche: string, current: array{route: string, params: array<string, mixed>, label: string}, shell: string, take: list<string>, keep: list<string>, demo?: array<string, string>}|null */
    public function find(string $slug): ?array
    {
        foreach ($this->all() as $proposal) {
            if ($proposal['slug'] === $slug) {
                return $proposal;
            }
        }

        return null;
    }
}
