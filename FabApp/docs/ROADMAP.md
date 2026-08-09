# FabOS — roadmap active

**Mise à jour : 2026-08-09 · état livré jusqu'à S102.** Les sessions terminées et leurs enseignements vivent dans `HISTORY.md`. Cette page ne contient que les décisions actuelles, les contradictions à arbitrer et le travail restant.

## Cap produit

FabOS doit permettre à tout fablab, école, atelier partagé ou réseau de lieux de déployer uniquement les fonctions dont il a besoin, avec une expérience cohérente et simple :

- une installation FabOS, plusieurs **sous-lieux** physiques ;
- aucun portail ;
- groupes locaux modifiables et packages assignés à une personne ou un groupe ;
- grants Use, Report et Manage par feature, sous-lieu et scope métier ;
- réservations, quotas et reporting présentés dans chaque feature, mais moteurs communs ;
- profils publics volontaires et échanges inter-FabOS consentis ;
- badges cumulatifs, vérifiables et fédérables ;
- un onglet **Configuration → Thèmes** réunissant identité visuelle, images, noms/ordre des menus et contenu/ordre de la page d'accueil ;
- un seul système central de listes, filtres, workspaces, composants et CSS.

La spécification détaillée, les contradictions avec le code actuel et les décisions opérateur encore ouvertes sont dans [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage).

## Contradictions qui changent l'ancien plan

| Ancienne direction | Nouvelle direction | Conséquence roadmap |
|---|---|---|
| portails par hostname avec réglages/features/packages propres | un seul FabOS avec sous-lieux | S105 puis S126 retirent les portails progressivement ; aucune suppression directe |
| packages portal-scopés | packages FabOS-wide, grants scopés par sous-lieu | S111 migre S97–S99 après création des sous-lieux |
| User/Admin + groupes, responsabilités et packages présentés comme objets séparés | groupes locaux + packages contenant Use/Report/Manage | une seule expérience package, mais Use et l'administration restent deux moteurs de sécurité internes |
| Venue futur distinct des portails | sous-lieu canonique confirmé, portail abandonné | S106–S108 deviennent le socle de tous les scopes et filtres |
| Réservations et quotas comme pages transversales | onglets contextualisés de chaque feature | services communs avec adaptateurs ; ancienne page retirée seulement en S120 |
| Institution comme simple reconnaisseur | connexion possible à une autre instance FabOS | S122–S125 ajoutent confiance, provenance et synchronisation |
| badges supprimables et attribution minimale | attributions globales append-only avec révocation auditée | S116 remplace immédiatement le delete/cascade par archivage et FK non destructive ; S124 ajoute la fédération |
| Pages du Lab | Pages personnalisées | renommage et workspace en S117 |
| Matériaux comme section autonome | Matériaux sous Équipement | présentation déplacée ; le feature gate interne reste à arbitrer |
| ancien menu « Le lieu » | Utilisateurs ; Horaires sous Lieux ; Interface/contenu d'accueil sous Thèmes | navigation reconstruite par le registre S103 |
| couleurs, images, menus et accueil dispersés entre Réglages, Portails, Interface accueil et éditeur Homepage | un seul onglet Configuration → Thèmes | S103 définit le contrat central ; S105 consolide le branding Portal ; S117 livre l'éditeur et migre l'accueil |
| S58/S62/S65/S67/S74 comme chantiers séparés | détail, staff/manage, espace membre, packages et catégories intégrés aux workspaces | anciens numéros retirés du plan actif ; les livraisons existantes restent dans HISTORY |
| Report/Manage comme permissions générales | signalement, analytics, export, édition, suppression et sécurité sont des opérations différentes | trois niveaux simples dans l'UX, capacités atomiques namespacées dans voters/services |
| profil public opt-in futur | plusieurs annuaires/API/statistiques publient déjà des identités | S121 inventorie chaque exposition et demande une règle explicite avant de promettre « privé par défaut » |

## Règles de construction

1. **Une source de métadonnées.** `FeatureWorkspaceRegistry` décrit navigation, onglets, niveaux Use/Report/Manage, scopes, filtres, réservations, quotas et reporting. Les capacités atomiques, voters, services et adaptateurs restent l'autorité métier.
2. **Une liste, un shell.** `_admin_list`, `_data_table`, catalogue partagé, composants et CSS centraux ; colonnes/données propres à la page.
3. **Pas de surcharge.** Sous-lieu, tâche et filtre de liste sont trois axes visuels différents. Six filtres rapides maximum, le reste dans `Plus de filtres` et en chips actives.
4. **URL explicable.** Sous-lieu, recherche, filtres et pagination sont partageables ; la préférence profil ne devient jamais un droit caché.
5. **Sécurité côté serveur.** Navigation et UI lisent le même verdict que voters/services. Un lien caché n'autorise ni n'interdit rien.
6. **Migrations expand/backfill/contract.** Pas de FK obligatoire avant rapport des lignes orphelines. Aucun `schema:update --force`.
7. **Simulation avant activation.** Groupes, grants, packages v2 et politiques comparent leurs verdicts au legacy avant enforcement. Chaque route mutante doit être couverte par un voter/service atomique.
8. **Cinq langues, sombre, mobile et clavier.** Toute nouvelle primitive est montrée avec le vrai composant dans `/admin/design`.
9. **Artemis est la définition de done.** Documentation, commit, archive étroite CT210, cache, restart et vérification réelle à chaque session. Jamais `deploy.sh`.

## Répartition

- **Terra** : modèle de domaine, migrations, repositories, services, voters, adaptateurs, protocoles et performance.
- **Luna** : architecture d'information, composants centraux, listes, filtres, workspaces, progressive disclosure, responsive et accessibilité.
- **Sol** : revue indépendante obligatoire de migrations, permissions, absence de lockout/fuite, parité legacy, tests, déploiement et critères de sortie.

Une session peut être codée conjointement par Terra et Luna, mais Sol ne valide jamais sa propre implémentation.

## Phase A — figer la cible et construire les fondations

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S102 ✅** | décisions, contradictions et roadmap nettoyée ; S100–S101 marqués remplacés | Terra + Luna | cohérence documentation/code |
| **S103** | registre Feature Workspace v2 + contrat Thèmes central + maquettes Développement à jour, aucun enforcement | Terra + Luna | matrice feature/route/scope/capacité et inventaire branding/menu/accueil |
| **S104** | fondation quotas réparée : compteurs par type, contraintes dures avant passes, grandfathering testé | Terra | verdicts de régression, aucun merge de politiques |
| **S105** | gel des portails + rapport de consolidation de chaque hostname/réglage/feature/package | Terra | collisions, 301 canonique, sauvegarde/rollback |
| **S106** | entité Sous-lieu, sous-lieu par défaut, horaires et interface d'accueil migrés, rendu inchangé | Terra + Luna | backfill, rollback, timezone/DST |
| **S107** | rattachement machines, espaces, événements sur site, prêts, lecteurs et sessions physiques | Terra | zéro orphelin avant contraintes |
| **S108** | préférence profil, contexte sous-lieu URL et composant partagé | Terra + Luna | préférence ≠ permission, refus hors scope, mobile/clavier |

## Phase B — groupes et packages v2

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S109** | groupes locaux, audiences Admin/Guest protégées, migration complète des consommateurs Staff/Trainer | Terra + Luna | concurrence dernier Admin, CLI recovery, parité des rôles |
| **S110** | grants Use/Report/Manage mappés vers capacités atomiques et scopes en simulation | Terra | route→voter/service, CSRF/IDOR, anti-escalade |
| **S111** | packages v2, attributions individu/groupe, restrictions de sous-lieu et migration S97–S99 | Terra + Luna | union, politique complète, restriction, dates, rollback |

## Phase C — un shell puis tous les workspaces

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S112** | shell central listes/filtres/facettes : contexte, tabs, filtres rapides/avancés, chips | Luna + Terra | URL, requêtes bornées, a11y, sombre, cinq langues |
| **S113** | Équipement : machines, catégories, modèles, matériaux, maintenance et réservations ; emplacements Quotas/Reporting non affichés avant S118–S119 | Terra + Luna | scopes/sécurité et aucune copie de shell |
| **S114** | Événements et Prêts | Terra + Luna | Guest/public et vrais adaptateurs inscription/prêt |
| **S115** | Espaces | Terra + Luna | sous-lieu, catégorie, responsable et département |
| **S116** | Formations et Badges ; définition archivable, retrait du delete/cascade, FK non destructive, attribution append-only locale | Terra + Luna | sessions, règles qualification, historique préservé |
| **S117** | Galerie, Pages personnalisées, Utilisateurs, Lieux, Packages, Réseau, Configuration et éditeur Thèmes | Terra + Luna | aucune feature/route perdue ; preview/publish/rollback du thème |

Ordre d'une liste : titre/action, contexte sous-lieu, onglets de feature, un axe rapide, recherche/avancé, chips actives, résultats. Maximum cinq colonnes ; aucune largeur minimale locale ; ligne entière cliquable seulement si elle n'a qu'une destination.

S113–S117 ne montrent aucun onglet Quotas/Reporting vide. S118 branche les politiques avancées, puis S119 ajoute le socle Reporting et ses onglets aux workspaces déjà migrés.

## Phase D — réservations, quotas et reporting

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S118** | politiques par feature : horizon, durée, caps, délai d'annulation, granularité et buffers | Terra | chemins complets, contraintes physiques séparées |
| **S119** | socle Reporting, adaptateur initial et capacités analytics.view/export | Terra + Luna | scopes, agrégations et exports sans fuite |
| **S120** | redirections puis retrait visuel de Réservations globale après parité | Terra + Luna | liens historiques et actions préservés |

`Reservation`, `EventRegistration`, inscriptions de formation et prêts ne deviennent pas artificiellement une seule table. Un contrat commun orchestre des adaptateurs propres à chaque feature.

## Phase E — profil public et réseau FabOS

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S121** | `/m/{slug}` opt-in, visibilité champ par champ, aperçu et inventaire des expositions actuelles | Terra + Luna | annuaires/API/kiosk, confidentialité, indexation |
| **S122** | identité d'instance, confiance, clés et API FabOS versionnée | Terra | rotation, replay, SSRF, panne distante, audit |
| **S123** | import QR ponctuel inter-FabOS, signé, expirant, usage unique et consenti | Terra + Luna | POST atomique, replay, logs/referrer, provenance |
| **S124** | badges/formations fédérés, append-only, révocation et règles dérivées | Terra + Luna | doublons, preuve, aucune suppression |
| **S125** | marques/modèles machines fédérés, provenance et overrides locaux | Terra + Luna | aucun token/statut/sécurité locale écrasé |

La synchronisation ne transmet jamais mots de passe, RFID, rôles, groupes, packages ou données non consenties. La resynchronisation d'un profil membre est manuelle par défaut ; une sync continue est un produit distinct.

## Phase F — retrait et audit final

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S126** | retrait technique des portails après un cycle complet, rapport nul, sauvegarde et routes de transition | Terra | aucun consommateur/row restant, restauration testée |
| **S127** | audit transversal de toutes listes, workspaces, permissions et traductions | Luna | Sol valide feature × onglet × scope × filtre × capacité |

## Décisions opérateur demandées

1. L'Admin recovery contourne-t-il badges/formation et arrêt de sécurité ? **Recommandation : non.**
2. Manage implique-t-il Report sur le même scope ? **Recommandation : oui ; jamais Use.**
3. Guest désigne-t-il l'anonyme, et non un groupe de comptes ?
4. Formateurs devient-il un groupe seedé supplémentaire ou est-il fusionné avec Staff ?
5. Institution signifie-t-il exclusivement « autre FabOS », ou garde-t-on aussi des émetteurs externes descriptifs ?
6. Une attribution de badge erronée reste-t-elle visible comme révoquée ? **Recommandation : oui.**
7. Formation = catalogue global + sessions localisées ? **Recommandation : oui.**
8. Matériau = catalogue global + stocks par sous-lieu ? **Recommandation : oui.**
9. Confirmation du menu : « Le lieu » devient « Utilisateurs » ; Horaires passe sous « Lieux » et Interface/contenu d'accueil sous « Configuration → Thèmes ».
10. Champs proposés lors d'un import QR : nom, prénom, e-mail, avatar, bio et langue, chacun confirmé ?
11. `guestsAllowed` reste-t-il la règle des événements publics ou devient-il un grant Guest ?
12. Les annuaires, leaderboard/API, kiosk, historiques et galerie suivent-ils le consentement du profil public ou une règle séparée ?
13. Si plusieurs chemins package/politique passent, affiche-t-on le plus spécifique comme chemin gagnant ?

## Travaux transversaux conservés

Ces besoins ne contredisent pas la nouvelle architecture mais passent après les fondations dont ils dépendent :

- sécurité et confidentialité restantes de Phase H, notamment test réel du booking et requêtes groupées ;
- mot de passe oublié et suppression/anonymisation de compte ;
- verrou d'annulation, no-show uniquement sur ressources disposant d'un signal ;
- archivage plutôt que suppression ;
- files d'attente, stockage/retrait, défauts déclarés et motif d'utilisation ;
- exceptions d'horaires intégrées au workspace Lieux ;
- audit et notes nécessaires à toute action Manage sur autrui.

RFID/cartes physiques, facturation, crédits et 2FA restent hors scope. La réservation d'un pool de machines n'est pas impliquée par les catégories.

## Nettoyage effectué

Les longues sections S59, S77, S78, S80 et S81–S85, déjà livrées, ont quitté cette roadmap ; leurs décisions utiles sont dans `HISTORY.md` et `UI-CONSISTENCY.md`. Les anciens chantiers S58, S62, S65, S67 et S74 ne sont plus planifiés séparément : leur travail restant est absorbé par S103–S125. Les hypothèses de portail de S100–S101 restent dans l'historique, mais ne pilotent plus aucune migration future.
