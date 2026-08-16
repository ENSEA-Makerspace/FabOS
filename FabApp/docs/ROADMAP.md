# FabOS — roadmap active

**Mise à jour : 2026-08-10 · état livré jusqu'à S128.** Les sessions terminées et leurs enseignements vivent dans `HISTORY.md`. Cette page ne contient que les décisions actuelles et le travail restant.

## Cap produit

FabOS doit permettre à tout fablab, école, atelier partagé ou réseau de lieux de déployer uniquement les fonctions dont il a besoin, avec une expérience cohérente et simple :

- une installation FabOS, plusieurs **sous-lieux** physiques ;
- aucun portail ;
- une connexion transparente entre instances via un fournisseur d'identité commun, sans partager automatiquement droits ou données ;
- sept audiences/groupes intégrés protégés (Admin, Manager, Staff, Super user, User, Guest et Formateurs), groupes locaux supplémentaires et packages assignés à une personne ou un groupe ;
- grants Use et Manage par feature, sous-lieu et scope métier, le reporting étant inclus dans Manage ;
- réservations, quotas et reporting présentés dans chaque feature, mais moteurs communs ;
- profils publics volontaires et échanges inter-FabOS consentis ;
- badges cumulatifs, vérifiables et fédérables ;
- un futur module Paiements facultatif pour acheter packages, matériaux et crédits de temps machine/personne/formation, sans contourner droits, quotas ou sécurité ;
- beaucoup plus tard, une messagerie Formation entre formateurs et étudiants, conservée dans FabOS et dupliquée par e-mail ;
- un onglet **Configuration → Thèmes** réunissant identité visuelle, images, noms/ordre des menus et contenu/ordre de la page d'accueil ;
- un seul système central de listes, filtres, workspaces, composants et CSS.

La spécification détaillée du modèle cible est dans [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage).

## Frontière d'une instance FabOS

**Même FabOS** uniquement lorsque les unités partagent gouvernance, administrateurs de confiance, annuaire métier, politiques/packages, moteur de réservation, cycle de vie et source de vérité. Elles deviennent alors des sous-lieux physiques : mêmes données et workspaces, vue agrégée par défaut, filtre de sous-lieu facultatif.

**Autre FabOS** dès qu'un service veut son propre administrateur, thème, catalogue, règles, rétention ou capacité à évoluer et s'arrêter indépendamment. La distance physique ne décide pas. Une authentification commune LDAP/OIDC/SAML évite un nouveau mot de passe, mais chaque FabOS garde comptes locaux, groupes, packages, quotas, audit et recovery Admin. Le partage métier passe uniquement par le Réseau FabOS/Institution, objet par objet et avec provenance.

Chaque ressource, événement, réservation ou credential possède une instance autoritaire. Entre instances, la règle par défaut est catalogue/projection en lecture seule puis lien vers l'instance propriétaire. Une réservation ou billetterie distribuée n'est pas implicite dans la synchronisation.

## Règles de construction

1. **Une source de métadonnées.** `FeatureWorkspaceRegistry` décrit navigation, onglets, niveaux Use/Manage, scopes, filtres, réservations, quotas et reporting. Les capacités atomiques, voters, services et adaptateurs restent l'autorité métier.
2. **Une liste, un shell.** `_admin_list`, `_data_table`, catalogue partagé, composants et CSS centraux ; colonnes/données propres à la page.
3. **Pas de surcharge.** Sous-lieu, tâche et filtre de liste sont trois axes visuels différents. Six filtres rapides maximum, le reste dans `Plus de filtres` et en chips actives.
4. **URL explicable.** La vue par défaut agrège tous les sous-lieux autorisés ; sous-lieu, recherche, filtres et pagination sont partageables ; la préférence profil ne devient jamais un droit caché.
5. **Sécurité côté serveur.** Navigation et UI lisent le même verdict que voters/services. Un lien caché n'autorise ni n'interdit rien.
6. **Migrations expand/backfill/contract.** Pas de FK obligatoire avant rapport des lignes orphelines. Aucun `schema:update --force`.
7. **Simulation avant activation.** Groupes, grants, packages v2 et politiques comparent leurs verdicts au legacy avant enforcement. Chaque route mutante doit être couverte par un voter/service atomique.
8. **Cinq langues, sombre, mobile et clavier.** Toute nouvelle primitive est montrée avec le vrai composant dans `/admin/design`.
9. **Artemis est la définition de done.** Documentation, commit, archive étroite CT210, cache, restart et vérification réelle à chaque session. Jamais `deploy.sh`.

## Décisions opérateur désormais fixées

- **Guest est l'audience anonyme sans compte.** Visibilité et action sont distinctes. Le réglage FabOS est un **défaut**, pas un plafond ; chaque événement peut `inherit`, `allow` ou `deny` séparément pour `view` et `register`. La migration préserve le live en marquant les événements existants visibles et en recopiant `guestsAllowed` vers l'inscription avant d'activer les nouveaux défauts fermés.
- **Les groupes intégrés sont protégés.** Admin, Manager, Staff, Super user, User et Formateurs ont des clés stables et ne peuvent pas être supprimés ; leurs libellés/descriptions locaux et leurs attributions restent configurables. Formateurs demeure lié au workspace Formations. User est l'audience système de tout compte local actif, sans membership retirable ; Guest est l'audience virtuelle sans compte. Les labs peuvent ajouter leurs propres groupes.
- **Une Institution peut rester descriptive ou devenir connectée.** L'interface demande une URL HTTPS, normalisée en origin unique. FabOS tente une découverte standard ; si une instance compatible est détectée, l'administrateur confirme la confiance, sinon l'Institution reste un simple organisme reconnaisseur. Changer d'origin suspend la connexion et impose une nouvelle confirmation.
- **Le partage distingue les personnes des catalogues.** Claims et credentials personnels sortent seulement si le FabOS source les autorise **et** si le membre consent à cette donnée et cette destination. Les catalogues non personnels suivent la politique de publication de l'instance et la confiance du destinataire, sans identifiant membre.
- **Un badge n'est jamais effacé.** Sa définition est archivable et une attribution erronée, expirée ou retirée reste dans le journal avec une révocation auditée.
- **L'import QR est automatique après un consentement récapitulatif unique.** Il importe les champs généraux autorisés et les badges acquis avec provenance, expiration et révocation ; un badge révoqué/expiré n'est jamais réactivé et une nouvelle lecture applique la révocation source à la copie. Les valeurs locales en conflit ne sont jamais remplacées silencieusement.
- **Les packages sont cumulatifs et fermés par défaut.** Tous les grants actifs d'une personne et de ses groupes s'unissent : un accès mardi plus un accès mercredi donne les deux. Aucun package ne retire un droit ; suspension et bannissement sont des mécanismes séparés. Chaque chemin complet grant + horaire + politique de quota reste évalué séparément, sans fusionner les champs de plusieurs politiques.
- **Les matériaux forment un catalogue FabOS partageable.** Des définitions peuvent venir d'une Institution ; disponibilités, emplacements et futurs stocks restent locaux à chaque sous-lieu. « Matériaux sous Équipement » est une décision de navigation, pas une confusion entre catalogue et stock.
- **Navigation confirmée :** l'ancien groupe « Le lieu » devient « Utilisateurs » ; Horaires passe sous « Lieux » et Interface/contenu d'accueil sous « Configuration → Thèmes ».

## Répartition

- **Terra** : modèle de domaine, migrations, repositories, services, voters, adaptateurs, protocoles et performance.
- **Luna** : architecture d'information, composants centraux, listes, filtres, workspaces, progressive disclosure, responsive et accessibilité.
- **Sol** : revue indépendante obligatoire de migrations, permissions, absence de lockout/fuite, parité legacy, tests, déploiement et critères de sortie.

Une session peut être codée conjointement par Terra et Luna, mais Sol ne valide jamais sa propre implémentation.

## Phase A — figer la cible et construire les fondations

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S102 ✅** | décisions et roadmap nettoyée ; S100–S101 marqués remplacés | Terra + Luna | cohérence documentation/code |
| **S103 ✅** | registre Feature Workspace v2 + contrat Thèmes central + maquettes Développement à jour, aucun enforcement | Terra + Luna | matrice feature/route/scope/capacité et inventaire branding/menu/accueil |
| **S104 ✅** | fondation quotas réparée : compteurs par type, contraintes dures avant passes, grandfathering testé | Terra | verdicts de régression, aucun merge de politiques |
| **S105 ✅** | gel des portails + rapport de consolidation de chaque hostname/réglage/feature/package | Terra | collisions, 301 canonique, sauvegarde/rollback |
| **S106 ✅** | entité Sous-lieu, sous-lieu par défaut, identité physique et horaires migrés, rendu inchangé | Terra + Luna | backfill, rollback, timezone/DST |
| **S107 ✅** | machines, espaces et objets prêtables rattachés au sous-lieu par défaut ; événements sur site rattachés, externes laissés sans sous-lieu | Terra | zéro orphelin avant contraintes ; prêts et lecteurs héritent de leur objet/machine |
| **S108 ✅** | préférence de sous-lieu du profil, contexte `?location=` et composant partagé sur les listes machines/espaces/événements | Terra + Luna | préférence ≠ permission, URL invalide refusée, mobile/clavier |

## Phase B — groupes et packages v2

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S109 ✅** | sept groupes/audiences intégrés protégés, avec clés stables et compatibilité des rôles actifs | Terra + Luna | recovery Admin et rôles Staff/Formateurs inchangés |
| **S110 ✅** | modèle de grant Use/Manage et scopes en simulation ; reporting/export inclus dans Manage | Terra | Manage ne confère jamais Use ; aucun enforcement actif |
| **S111 ✅** | packages v2 : grants persistants, attributions individu/groupe et restrictions de sous-lieu en shadow | Terra + Luna | union, dates, scope AND ; aucun enforcement actif |

## Phase C — un shell puis tous les workspaces

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S112 ✅** | shell central listes/filtres/facettes : contexte, tabs, filtres rapides/avancés, chips | Luna + Terra | URL, requêtes bornées, a11y, sombre, cinq langues |
| **S113 ✅** | Équipement : machines, catégories, modèles, matériaux, maintenance et réservations ; emplacements Quotas/Reporting non affichés avant S118–S119 | Terra + Luna | scopes/sécurité et aucune copie de shell |
| **S114 ✅** | Événements et Prêts | Terra + Luna | Guest/public et vrais adaptateurs inscription/prêt |
| **S115 ✅** | Espaces | Terra + Luna | sous-lieu, catégorie, responsable et département |
| **S116 ✅** | Formations et Badges ; définition archivable, retrait du delete/cascade, FK non destructive, attribution append-only locale | Terra + Luna | sessions, règles qualification, historique préservé |
| **S117 ✅** | Galerie, Pages personnalisées, Utilisateurs, Lieux, Packages, Réseau, Configuration et éditeur Thèmes | Terra + Luna | aucune feature/route perdue ; preview/publish/rollback du thème |

Ordre d'une liste : titre/action, contexte sous-lieu, onglets de feature, un axe rapide, recherche/avancé, chips actives, résultats. Maximum cinq colonnes ; aucune largeur minimale locale ; ligne entière cliquable seulement si elle n'a qu'une destination.

S113–S117 ne montrent aucun onglet Quotas/Reporting vide. S118 branche les politiques avancées, puis S119 ajoute le socle Reporting et ses onglets aux workspaces déjà migrés.

## Phase D — réservations, quotas et reporting

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S118 ✅** | politiques par feature : horizon, durée, caps, délai d'annulation, granularité et buffers | Terra | chemins complets, contraintes physiques séparées |
| **S119 ✅** | socle Reporting, adaptateur initial et capacités analytics.view/export | Terra + Luna | scopes, agrégations et exports sans fuite |
| **S120 ✅** | redirections puis retrait visuel de Réservations globale après parité | Terra + Luna | liens historiques et actions préservés |

`Reservation`, `EventRegistration`, inscriptions de formation et prêts ne deviennent pas artificiellement une seule table. Un contrat commun orchestre des adaptateurs propres à chaque feature.

## Phase E — identité, profil public et réseau FabOS

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S121 ✅** | fédération d'authentification : ProviderRegistry, OIDC d'abord, liens `(issuer, subject)`, provisioning local | Terra + Luna | aucune fusion e-mail/claim Admin ; pannes, rotation, revoke-all |
| **S122 ✅** | `/m/{slug}` opt-in, visibilité champ par champ, aperçu et inventaire des expositions actuelles | Terra + Luna | annuaires/API/kiosk, confidentialité, indexation |
| **S123 ✅** | identité d'instance, confiance, clés et API FabOS versionnée | Terra | rotation, replay, SSRF, panne distante, audit |
| **S124 ✅** | import QR ponctuel inter-FabOS, signé, expirant, usage unique et consenti | Terra + Luna | POST atomique, replay, logs/referrer, provenance |
| **S125 ✅** | badges/formations fédérés, append-only, révocation et règles dérivées | Terra + Luna | doublons, preuve, aucune suppression |
| **S126 ✅** | marques/modèles machines fédérés, provenance et overrides locaux | Terra + Luna | aucun token/statut/sécurité locale écrasé |

La synchronisation ne transmet jamais mots de passe, RFID, rôles, groupes, packages ou données personnelles non consenties. Les filtres de publication de l'instance et les choix du membre s'appliquent ensemble aux claims/credentials personnels ; les catalogues non personnels suivent la publication de l'instance et la confiance du peer. La resynchronisation d'un profil membre est manuelle par défaut ; une sync continue est un produit distinct.

SSO et synchronisation restent deux projets séparés. S121 authentifie l'utilisateur auprès d'un issuer autorisé puis ouvre un compte local minimal ; changements d'e-mail ne changent pas l'identité et deux subjects ayant le même e-mail restent deux comptes. Aucun claim externe ne crée un Global Admin. Une suspension locale, un bouton revoke-all et un recovery hors IdP subsistent. S123–S126 échangent ensuite seulement des objets autorisés, versionnés et consentis.

Cas limites explicitement reportés à des sessions dédiées : ressource partagée entre instances (un propriétaire, deep-link par défaut), événement coorganisé (source + projection UUID), split/fusion d'instances, déménagement d'un sous-lieu vers une autre gouvernance, et véritable réservation distribuée avec holds/saga/compensation. Aucun quota ou chevauchement global n'est promis par la simple synchronisation.

## Phase F — retrait et audit du socle

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S127 ✅** | retrait technique des portails après un cycle complet, rapport nul, sauvegarde et routes de transition | Terra | schéma et consommateurs retirés sur Artemis ; routes historiques en transition |
| **S128 ✅** | audit transversal de toutes listes, workspaces, permissions et traductions | Luna | 30 tests / 208 assertions, 188 Twig et 29 YAML valides ; 14 workspaces rendus 200 sur Artemis |

## Phase G — stabilisation multi-lieux et navigation admin

**Constat post-S128 (2026-08-10).** La cible décrite par S106–S117 n'est pas encore cohérente dans l'interface : `FeatureWorkspaceRegistry` annonce « Lieux → Sous-lieux, Horaires », alors que `NavBuilder` conserve le groupe « Le lieu » et que les écrans visibles permettent de gérer les **Espaces** (`/admin/places`), pas les entités **Sous-lieu**. Le sélecteur partagé `_venue_context.html.twig` est présent sur quelques listes seulement et doit devenir le contrat unique, sans être confondu avec un filtre métier ni un droit.

Les actions rapides redondantes ont été retirées du tableau de bord : la navigation canonique et les actions propres à chaque workspace sont les seuls raccourcis d'administration.

| Session | Résultat attendu | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S129 ✅** | workspace **Lieux** réellement opérable : liste, création, édition, archivage sécurisé et accès clair aux horaires ; un sous-lieu par défaut non supprimable | Terra + Luna | migration sans perte, slug/nom/timezone, objets rattachés, aucun lockout ; route, onglets et sidebar issus de la même métadonnée |
| **S130 ✅** | navigation admin dédoublonnée et conforme à la décision : « Utilisateurs » remplace « Le lieu » ; un workspace **Lieux** porte Sous-lieux et Horaires ; **Matériaux** rejoint **Équipement** ; « Pages du Lab » devient **Pages personnalisées** ; **Réglages** devient l'entrée par défaut de Configuration, qui contient aussi État installation et Thèmes ; Réseau et Packages ont chacun une seule entrée canonique | Luna + Terra | inventaire de toutes routes, active-state correct sur créations/éditions, redirections historiques conservées, mobile/clavier/sombre/cinq langues |
| **S130b ✅** | **une seule sous-navigation admin.** Les vingt pages d'administration en dessinaient **deux** : la bande de `NavBuilder` et les onglets de `FeatureWorkspaceRegistry`, en désaccord sur le contenu d'Équipement, sur le parent d'Institutions, et — une seule des deux lisant les catalogues — **dans deux langues à la fois sur le même écran**. Les onglets sont supprimés ; les six écrans qu'eux seuls atteignaient sont repris par `NavBuilder` ; les libellés passent aux catalogues ; une section à une entrée ne dessine plus de bande. Plus le footer remonté dans `base.html.twig`, l'action « Modifier » des utilisateurs qui ouvrait une fiche, l'objet d'un prêt enfin cliquable, et les deux boutons dupliqués de Lecteurs RFID | Terra | 34 pages rendues : `tabs=0`, une bande, une entrée allumée ; 50 pages : `footers=1` ; 31 tests / 780 assertions ; comparaison de hash sur 94 fichiers |
| **S130d ✅** | **Réseau FabOS passe dans Configuration** (décision opérateur). C'était une section de premier niveau ne contenant qu'un écran : une ligne entière de sidebar, au même rang qu'Équipement et ses onze écrans, et sous la règle de S130b elle ne dessinait aucune bande. Ce qu'on y règle, c'est l'identité de l'instance, ses fournisseurs OIDC et ses pairs de confiance — de la configuration, pas un workspace. Placé après E-mails, avant Configuration initiale qui reste en dernier | Terra | 10 pages rendues : plus de section Réseau dans la sidebar, `/admin/network` allumé dans la bande Configuration et **doté d'une sous-navigation qu'il n'avait pas** ; 31 tests / 769 assertions |
| **S130e ⏸** | **trois propositions de panneau de filtres**, à `/admin/design#filtres`, préfixées `dzf-` donc sans effet réel. La **forme de page est validée** : titre à gauche, sous-lieu à droite du bandeau, et « Créer » descend dans l'en-tête de liste en **`+` vert** à côté du compte. Les trois ne varient plus que sur le panneau : A tiroir simple, B deux étages sans rien de caché, C rangée bornée + tiroir qui se compte. ⚠️ Le vert veut déjà dire « disponible » dans les pastilles d'état — variante accent à côté | Terra + opérateur | hauteurs et projections **mesurées** dans le navigateur, clone par clone : 394 px aujourd'hui, 124 px pour les trois, et à 12 catégories 176 / 202 / 124 |
| **S130c ✅** | **le filtre sous-lieu devient un filtre un-clic.** C'était un `<select>` dans une barre pleine largeur *au-dessus* du panneau qui contient tous les autres filtres ; c'est maintenant une rangée de tuiles `ml-tile` comme les catégories et les états, premier groupe dans le panneau. **Rien ne s'affiche si l'installation n'a qu'un sous-lieu**, et le test vit dans le partial. 🔴 Au passage : `allow_all\|default(true)` valait toujours vrai (Twig `default` se déclenche sur *vide*, et `false` est vide), donc `/admin/horaires` proposait « tous les sous-lieux » depuis S131 — l'opérateur éditait alors la semaine du sous-lieu par défaut pendant que l'URL disait `location=all` | Terra | 13 pages rendues : ancien `<select>` absent partout, 3 tuiles et une seule allumée, `?location=fabshop` allume FabShop, Horaires n'a plus que 2 tuiles ; 31 tests / 780 assertions ; hash sur 75 fichiers |
| **S131 ✅** | contexte sous-lieu uniforme sur toutes les listes/workspaces qui portent un sous-lieu, avec conservation de `location`, recherche, facettes, onglets et pagination | Terra + Luna | `?location=` valide/refus explicite hors scope, défaut agrégé, préférence profil seulement comme défaut, aucun filtre local concurrent, tests des URLs partageables |
| **S132 ⬅️** | shell/design system admin : retirer menus, footers, scaffolds et CSS locaux ; extraire les patterns Lecteurs RFID vers `/admin/design` et refaire Logs RFID, Réglages, E-mails et Fonctionnalités | Luna + Terra | capture Artemis de chaque workspace, tests Twig/routes, aucun CSS/markup local, footer systématique, une seule sidebar + sous-navigation, contrastes/mobile/i18n |
| **S133 ⬅️** | parité fonctionnelle des workspaces : vraies surfaces pour catégories machine, objets/Prêts, Événements (aperçu, inscriptions, tickets) et Configuration ; corriger « À venir » et tous les filtres un-clic/traduits | Terra + Luna | routes/chokepoints, objets archivés, sous-lieu/all/ailleurs/fuseau/pagination, URLs partageables, tests de régression |
| **S133b** | gestion des droits réellement conforme : groupes intégrés/protégés et locaux, attributions personne/groupe, grants v2 Use/Manage et scopes administrables ; comparaison shadow visible et explicable | Terra + Luna | dernier Admin, User/Guest virtuels, dates, union des packages, scope AND, CSRF/IDOR/mass-assignment, aucune élévation par UI |
| **S134** | activation graduelle des grants v2 sur les seuls chokepoints audités, puis retrait du package legacy binaire | Terra + Sol | parité shadow sans différence inexpliquée, voters/services atomiques pour chaque écriture, Manage ≠ Use, refus hors scope et rollback par feature |
| **S134b** | passe finale de cleanup avant Commerce : revue de code, dette/copies mortes, routes orphelines, traductions, accessibilité et conformité de **toutes** les pages au design system | Luna + Terra, validation Sol | inventaire exhaustif route/template **et action opérateur** : chaque objet/configuration annoncée est créable, éditable, archivable ou révocable depuis son workspace ; parcours normal mesuré en clics, sans choix ou écran superflu ; progressive disclosure et design guidelines respectés ; zéro menu/footer/shell/CSS local injustifié ; composants démontrés dans `/admin/design` ; lint/tests/captures desktop-mobile/sombre ; Artemis déployé et vérifié page par page |

Ces sessions sont **bloquantes avant le commerce** : ne pas commencer paiement, catalogue d'offres ou ledger tant que l'opérateur ne peut pas découvrir puis administrer un sous-lieu depuis l'interface canonique, et que S134b n'a pas validé le cleanup transversal.

### Inventaire à vérifier pendant S129–S134 (ne pas marquer livré par la seule présence dans le registre)

- `locations` annonce **Sous-lieux** et **Horaires**, mais seule la route Horaires est enregistrée : créer la route et l'onglet Sous-lieux ou corriger le contrat avant toute navigation.
- La sidebar actuelle garde les groupes autonomes **Matériaux** et **Pages du Lab** : les déplacer respectivement sous Équipement et Pages personnalisées sans doubler leurs routes. **Espaces** reste un workspace métier distinct de Lieux.
- `configuration` annonce État installation, Fonctionnalités, Thèmes, Réglages, E-mails, Initialisation et Développement ; la sidebar laisse encore État installation hors Configuration et ne reflète pas ce contrat.
- `network` annonce Connexions, Synchronisations, Journal et expose aussi Institutions dans ses onglets, alors que la navigation principale n'offre que Réseau : confirmer les routes réellement opérables et ne pas fabriquer de faux onglets.
- `packages` annonce Packages, Attributions, Quotas et Audit, mais n'expose aujourd'hui qu'une liste/édition de package : soit livrer les surfaces, soit réduire temporairement les onglets déclarés.
- Les contrats Utilisateurs, Événements, Formations, Badges, Galerie et Pages annoncent aussi des sous-onglets : produire une matrice `onglet → route → capacité → état` et retirer/masquer tout onglet sans surface réelle. Les routes d'édition doivent conserver l'onglet actif.
- Le contexte `VenueContext` n'est injecté que dans les listes machines, taxonomie machines, événements, espaces et reporting ; vérifier systématiquement prêts, matériaux, formations/sessions, réservations, maintenance, lecteurs RFID et tout écran de création/édition portant un sous-lieu.
- `_admin_list` n'inclut pas le footer, tandis que certains templates l'incluent et d'autres recopient leur propre scaffold/CSS (dont des media queries) ; faire de `base_public`/un shell admin unique l'unique propriétaire header, navigation, footer, largeur, responsive et thèmes. Toute exception doit être justifiée dans `/admin/design`.
- **Catégories de machines** est aujourd'hui une vue dérivée des libellés saisis dans les machines ; décider et livrer un vrai catalogue CRUD (création, renommage, archivage/suppression avec impact explicite) ou retirer l'onglet administrable. Ne jamais présenter une simple facette comme gestion de catégorie.
- **Événements** : `Tous les sous-lieux` doit agréger chaque sous-lieu autorisé ; les événements sans sous-lieu restent dans une option explicite « Ailleurs / externe », jamais mêlés ou substitués silencieusement au filtre agrégé.
- Régression à corriger et couvrir pendant la stabilisation : le filtre Événements **« À venir »** ne doit jamais vider artificiellement la liste. Tester son intersection avec `location=all`, chaque sous-lieu, « Ailleurs / externe », recherche, pagination et fuseau horaire ; l'URL résultante doit être partageable et le filtre actif visible.
- **Prêts** : chaque objet prêté/louable doit ouvrir sa fiche canonique depuis les listes et réservations ; rétablir les liens de ligne et action, conserver le retour avec filtre/contexte et tester les objets supprimés/archivés.
- **Configuration → Fonctionnalités** doit être reconstruite sur le shell/registre des workspaces : présenter les workspaces et leurs sous-ensembles réels, leurs dépendances et l'effet de désactivation ; ne pas maintenir une taxonomie parallèle `SiteFeatureRegistry` ni un template/CSS autonome.
- **Configuration → Réglages** est la page par défaut. La remplacer par des cartes/sections courtes et liées (Général, localisation/langue, alertes et communication, exploitation/sécurité, développement avancé), avec résumé d'état, recherche/ancrages et sauvegarde claire par section. Les contrôles dangereux — notamment l'enforcement des droits — restent dans une divulgation avancée avec préflight, conséquence et confirmation. Elle doit passer par le shell admin commun et ne conserver aucun CSS/scaffold local.
- **Configuration → E-mails** suit le même shell et devient une surface lisible en trois tâches : état/diagnostic d'envoi, modèles et préférences/notifications, puis journal consultable. Séparer les réglages et actions de test du journal ; fournir recherche, période, statut et destinataire dans les filtres, détails à la demande et liens vers la ressource source. Les secrets ne sont jamais affichés, les erreurs sont expliquées sans exposer de données personnelles, et aucun CSS/scaffold local ne subsiste.
- **Logs RFID** doit utiliser exactement la navigation admin canonique (sidebar, sous-navigation Équipement et état actif), sans menu local ni seconde variante de shell. Refaire la page comme une liste d'exploitation : résumé court, filtres date/lecteur/machine/résultat, recherche, colonnes bornées, détail/audit à la demande et états vides/erreurs lisibles. Toute règle visuelle vient du design system ; aucune couleur, largeur, footer ou navigation ne peut être recopiée localement.
- **Lecteurs RFID** sert de référence à extraire, pas de modèle à recopier : inventorier ses bons éléments (hiérarchie d'en-tête, actions, cartes/états, tableau, responsive et retours), les rendre génériques dans `/admin/design` puis dans les composants du shell, avec un exemple réel pour chacun. Après extraction, Lecteurs RFID doit lui-même ne plus posséder de CSS/scaffold local ; les Logs RFID et les listes d'Équipement les adoptent par composition.
- **Règle de filtres site entière** : un filtre à ensemble fini (sélecteur, état, sous-lieu, date prédéfinie, catégorie, switch) applique son changement immédiatement en conservant l'URL ; la recherche textuelle est la seule commande qui exige une saisie puis validation explicite. Le **sous-lieu apparaît dans la même barre de filtres et avec les mêmes conventions visuelles** que les autres filtres, en première position lorsqu'il est applicable ; son aide « ne modifie pas vos droits » reste discrète. `Plus de filtres` révèle des contrôles, pas un second formulaire à confirmer. Le composant partagé doit porter ce comportement, les chips et le reset ; aucune liste — dont `/admin/machines` — ne le réimplémente. Tout libellé, option, aide et état vide de filtre passe par les cinq catalogues de traduction, sans texte français local.
- **Configuration → Thèmes** remplace le champ texte `logoPath` par une médiathèque d'identité : upload contrôlé et choix d'assets existants pour logo clair/sombre, logo compact, favicon et image de partage. Les fichiers sont validés (MIME, dimensions, taille), renommés côté serveur, privés jusqu'à publication, référencés par ID stable et supprimables seulement après contrôle des références ; aucun chemin `public/images/…` libre.
- L'éditeur Thèmes doit proposer des choix guidés plutôt que des réglages épars : identité/noms, variantes de logo, favicon, palette accessible avec contrastes, rayon/typographie/densité limités à des presets, images d'accueil, menus et ordre/visibilité des blocs. `/admin/design` reste la référence des composants, pas une seconde configuration opérateur.
- Le workflow Thèmes conserve brouillon → aperçu → publication → retour arrière, mais l'aperçu doit rendre de vraies surfaces (accueil, catalogue, page détail, admin) en desktop/mobile et clair/sombre ; publication atomique des réglages **et** assets, cache invalidé, anciennes versions récupérables. Le favicon et les métadonnées/OG doivent lire le thème publié partout, y compris kiosks et e-mails lorsque pertinent.
- **Kiosks** (événements, machine, entrées, statistiques et ticket) sont des consommateurs obligatoires du thème publié : logo/variante compacte, palette contrastée, favicon, nom du lieu et styles de statut viennent du même contrat, avec un mode lisibilité/contraste renforcé si nécessaire. Aucun `images/favicon.png`, logo ou couleur statique ne doit survivre dans un template kiosk ; l'aperçu Thèmes inclut au moins un écran kiosk.
- **Configuration → Thèmes → Navigation & accueil** doit permettre d'organiser le menu principal par drag-and-drop accessible et contrôles de visibilité : ordre, libellé local, entrée de menu et destination parmi les routes/pages publiées autorisées. Les entrées système indispensables (accès, profil, recovery/admin) restent protégées ; une destination désactivée, non publiée ou sans droit ne peut être choisie. L'aperçu montre desktop/mobile et le menu publié conserve un fallback sûr.
- Le même espace permet de composer l'accueil : activer/désactiver, ordonner et configurer les widgets/blocs disponibles, puis choisir la **page d'accueil** parmi Accueil FabOS ou une Page personnalisée publiée. Le changement est brouillonné, prévisualisé et publié atomiquement ; une page archivée/dépubliée rétablit automatiquement l'accueil FabOS avec audit, sans page blanche ni boucle de redirection.

### Horaires : ce que le modèle actuel ne peut pas exprimer (constat S131)

`OPENING_HOUR` est désormais correctement scindé par sous-lieu — la contrainte
`UNIQ_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek)` existe depuis S106 et l'écran
sait enfin l'utiliser (S131 : il appelait `findDefault()` et n'éditait donc que le
sous-lieu par défaut). Trois limites restent, par ordre de coût :

1. **Une seule plage par jour et par sous-lieu.** La contrainte unique porte sur
   `(venueId, dayOfWeek)`, donc une fermeture méridienne, un service du soir ou un
   créneau réservé au personnel sont inexprimables. ⚠️ C'est un **contract** au
   sens migration : passer à plusieurs lignes par jour supprime l'unicité, et tout
   code qui suppose « une ligne = un jour » (`ensureOpeningHourRows`,
   `OpeningHoursProvider`, les deux calendriers) doit être repris **avant**.
2. **Aucune granularité sous le sous-lieu.** Un atelier laser qui ferme plus tôt
   que le bâtiment, un magasin de prêt ouvert le mardi seulement, une salle
   réservable hors horaires : le modèle n'a pas de portée entre « sous-lieu » et
   « rien ». La forme cible est un horaire **rattachable** — sous-lieu par défaut,
   surchargeable par workspace (Équipement, Espaces, Prêts) puis par ressource
   (machine, espace, objet) — avec résolution par héritage et une seule réponse
   effective par instant. ⚠️ **Ne pas dupliquer la table par type de ressource** :
   c'est la même question posée à des portées différentes.
3. **Aucune exception datée.** Jours fériés, fermeture annuelle, ouverture
   exceptionnelle : le besoin est déjà listé dans les travaux transversaux, et il
   se pose au même endroit que la portée ci-dessus. Les deux se livrent ensemble
   ou la seconde réécrit la première.

⚠️ **Prérequis commun** : la résolution effective doit être un service unique que
lisent l'admin, les deux calendriers, les cartes de catalogue (« labo fermé » est
vérifié avant « occupée », cf. S59) et les kiosks. Aujourd'hui chacun interroge la
table. Tant que ce service n'existe pas, ajouter une portée multiplie les endroits
qui peuvent se contredire.

## Phase G2 — le produit honnête (avant le commerce)

**Ajoutée le 2026-08-11, après revue indépendante du socle livré.** L'ordre
relatif Commerce (S150–S154) puis Messagerie Formation (S155–S157) reste bon. Ce
qui ne l'est plus, c'est d'attaquer le commerce juste après S134b : on vendrait du
temps machine contre un modèle d'horaires incapable d'exprimer un jour férié, dans
des sous-lieux qui ne peuvent contenir aucune machine, via un back-office que la
moitié des langues cibles ne peut pas lire, alors qu'un membre ne peut toujours pas
réinitialiser son mot de passe. Le commerce est optionnel par installation ; les
manques ci-dessous concernent **toutes** les installations, tous les jours.

| Session | Résultat attendu | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S132b ✅** | réparation : le mismatch `venueContext`/`venue_context` écrasait la semaine d'un sous-lieu par celle d'un autre ; clé `workspace.tab.app_admin_venues` absente des cinq catalogues | Terra + Luna | rendu avec **deux** sous-lieux actifs, sélecteur présent, action de formulaire portant `location` |
| **S134c ✅** | le back-office parle la langue de l'opérateur : tous les templates admin passent par les cinq catalogues, `lang` honnête, zéro chaîne codée en dur | Luna | rendus en/de/es/it des ~15 écrans principaux sans résidu français ; catalogues toujours alignés ; `base.html.twig` ne fixe plus `lang="fr"` |
| **S134c2** | **FabOS cesse d'inventer le contenu d'une formation.** Voir la fiche ci-dessous. | Luna + opérateur | une formation dont les champs sont vides n'affiche ni horaire, ni session, ni objectif que personne n'a écrits |
| **S134d** | **une seule vérité horaire (modèle)** : un `ScheduleResolver` répond « X est-il ouvert à l'instant T » pour l'admin, les deux calendriers, les cartes de catalogue, les kiosks et l'API ; le schéma gagne plusieurs plages par jour, une portée attachable entre sous-lieu et rien (workspace puis ressource) et des exceptions datées — **livrés ensemble** | Terra | ⚠️ expand → backfill → **contract réel** ; le resolver doit être lu par tous avant que `UNIQ_OPENING_HOUR_VENUE_DAY` ne saute ; tests DST |
| **S134e** | **une seule vérité horaire (surfaces)** : Lieux édite plages, portées et exceptions avec aperçu de l'effet ; public, calendriers et kiosks affichent la fermeture **avec sa raison** | Luna + Terra | une exception datée créée une fois apparaît partout sans toucher un autre écran |
| **S134f** | archiver plutôt que supprimer appliqué aux workspaces qui exposent encore une suppression dure (événement, espace, matériau, objet prêtable, institution, page, création, lecteur RFID) ; tout chemin d'archivage d'une ressource réservable annule explicitement ses réservations à venir | Terra + Luna | plus aucune route de suppression dure sans justification écrite ; objets archivés visibles mais inertes |
| **S134g ✅** | le compte appartient au membre : mot de passe oublié (moitié 1, 2026-08-12) **et suppression/anonymisation (moitié 2, 2026-08-16)** | Terra + Luna | aller-retour complet de réinitialisation ; token haché, expirant, à usage unique ; **anonymisation irréversible, statistiques intactes, dernier admin refusé** |

⚠️ **Ce que S133 doit explicitement contenir, et qui manquait :** un **champ
sous-lieu sur les formulaires de création/édition** de machine, espace, objet
prêtable et événement. Les colonnes existent et sont `NOT NULL` depuis S107, les
listes filtrent par sous-lieu, S129 rend un sous-lieu créable — mais aucun
formulaire ne permet de choisir lequel, donc **tout atterrit à jamais sur le
sous-lieu par défaut et un second sous-lieu ne peut rien contenir**. Le critère de
sortie de la Phase G — « l'opérateur peut administrer un sous-lieu depuis
l'interface canonique » — n'est pas atteint sans cela. Aucune migration : seulement
les formulaires et la validation au chokepoint.

⚠️ **Deux régressions datées à corriger dans S133 :** `/admin/machines` affiche la
clé littérale `admin_list.all` comme libellé de tuile ; et `admin-events.html.twig`
calcule « À venir » avec `date()`, donc en **UTC serveur** contre des dates
saisies en heure murale — près de minuit un événement change d'onglet à deux heures
près, côté admin **et** sur la page publique.

## S141 — la carte fusionnée devient LE format de liste — **à faire**

Le format a été validé par l'opérateur le 2026-08-16, en quatre tours sur
`/admin/machines` (récit dans `HISTORY.md`, S140). Il est **retenu** : reste à le
généraliser, à le documenter et à passer les listes en revue une par une.

**Le format retenu — une seule carte, quatre bandes :**

| Bande | Contenu | Fond |
|---|---|---|
| bandeau | le **nom du menu**, le compte en sous-titre, la recherche, **un** bouton vert nommé | dégradé d'accent |
| filtres | une rangée de tuiles pour la facette qui définit la page, puis « Affiner » | `--surface-ground` (en creux) |
| message | le flash, quand il y en a un | sa teinte de ton |
| lignes | le tableau | la surface de la carte |

Les bandes sont séparées par un filet de 1 px et un fond, **jamais par un vide** :
plus de cartes flottant à 24 px les unes des autres. Mesuré à 1440 px sur
`/admin/machines` : **267 px** de la première ligne au haut du bandeau, contre
430 px dans l'ancien format à trois cartes.

⚠️ **Le titre est le nom du menu, pas une phrase écrite pour la page.**
`/admin/machines` affiche « Équipement » (`admin_nav.section.equipment`), la clé
que lit la barre de sous-menu elle-même. Décision opérateur : « quotas » plutôt
que « gestion des quotas », **et autant de clés de traduction en moins** — les
`*.title` par page disparaissent des cinq catalogues.
⚠️ **Une décision reste à prendre au moment de généraliser** : section
(`admin_nav.section.*`) ou entrée (`admin_nav.entry.*`) ? Machines a pris la
*section* parce que c'est la page d'atterrissage d'Équipement ; Catégories et
Matériaux ne peuvent pas s'appeler « Équipement » toutes les trois.
**Recommandation : l'entrée**, unique par page, en gardant Machines sur la
section puisque c'est ce qui a été validé à l'écran.

**Le travail, dans cet ordre :**

| Étape | Ce qu'elle fait | Vérification |
|---|---|---|
| **S141a** | `hero: 'merged'` devient le défaut ; `hero`/`compact` et `.is-merged` disparaissent, les règles s'attachent au shell lui-même | les 31 pages du shell rendues et regardées, pas seulement lintées |
| **S141b** | le CSS devient commun : le bloc S140 d'`admin.css` sort de son statut de variante, un seul endroit pour les quatre bandes | ⚠️ un partial partagé se style dans `components.css` — voir plus bas |
| **S141c** | `/admin/design#filtres` montre la carte fusionnée ; la maquette à trois cartes est retirée | la page embarque le **vrai** composant, pas une copie |
| **S141d** | titres = noms de menu, suppression des `*.title` devenues mortes dans `messages.{fr,en,de,es,it}.yaml` | `lint:yaml` + une passe de rendu |
| **S141e** | revue **contenu** des 25 listes : colonnes, doublons, types de cellules | une liste par écran, comparée à `/admin/utilisateurs` |
| **S141f** | les 5 tableaux hors shell rejoignent le format | voir la liste nommée plus bas |

**Périmètre chiffré, compté et pas estimé (2026-08-16) :**
- **31 pages** passent par `_admin_list` : 1 fusionnée (machines), 10 en
  `compact`, **20 encore sur le grand en-tête historique**.
- **25 d'entre elles** portent un `_data_table` ; les 6 autres sont des
  formulaires ou des pages « vision ».
- **5 tableaux vivent hors du shell** : `admin-emails`, `admin-homepage`,
  `admin-opening-hours`, `admin-utilisateur-detail`, plus la maquette
  `admin-design`.
- **Aucun autre `<table>` du site n'échappe à `_data_table`** — vérifié :
  hors `templates/emails/`, il n'en existe pas. Les « listes texte » sont donc
  exactement ces 30 écrans, et la liste est close.

**S141e — la revue de contenu, et pourquoi elle n'est pas cosmétique.**
Le vocabulaire de colonnes existe depuis S134i (`_cell_media`, `_cell_title`,
`_cell_state`, `_cell_meter`, `_cell_date`, `_cell_actions`, `_cell_empty`) et la
forme de référence est `/admin/utilisateurs`. Ce qui manque est la **discipline** :
peu de types, toujours les mêmes, et rien qui se répète d'une colonne à l'autre.

🔴 **Le cas signalé, et il est double — `/admin/formations`.** Le nom du badge est
affiché **deux fois** : en sous-titre du titre (`_cell_title`) *et* dans sa propre
colonne `is-tight`, en texte brut, où il se casse sur plusieurs lignes. Le remède
demandé par l'opérateur : **une pastille compacte qui LIE au badge** — un « 1 »
cliquable plutôt qu'un nom qui déborde. Cela veut dire un type de cellule de plus,
`_cell_chip` (jeton court + lien + nom complet en libellé accessible), et une
règle : *un fait est montré une fois par ligne*.

⚠️ **« CSS commun » a un sens précis ici, et il a déjà mordu trois fois**
(S135 `_cell_state`, S139c `_cell_title`, S139e `_breadcrumb`) : **la feuille d'un
partial partagé doit être chargée par toutes les pages qui l'utilisent.**
`components.css` est émise par `base.html.twig` partout ; `admin.css` ne l'est que
sur les pages d'admin. Le shell de liste est admin-only, donc ses quatre bandes
restent légitimement dans `admin.css` — mais **tout `_cell_*`, y compris le
`_cell_chip` à créer, va dans `components.css`**, parce qu'un `_cell_*` peut être
appelé depuis n'importe quelle page, publique comprise.

**Deux points ouverts, à trancher en commençant :**
1. 🔴 **Les libellés de tuiles passent en gris en thème sombre** dès que le
   panneau de filtres entre dans un `.admin-panel` : `style.css` y repeint tout
   `<span>` en `--color-text-light` avec `!important`. Mesuré 8,89:1 sur la tuile
   active et 9,32:1 sur les autres — c'est un **ton**, pas une lisibilité, et la
   hiérarchie tient à la bordure et à la teinte. À corriger à la source
   (`:not()`) ou à assumer, mais une fois pour les 41 listes.
2. Le bandeau porte maintenant la recherche : sur les listes **sans** recherche il
   n'y a rien entre le titre et le bouton, ce qui est bien — mais aucune des 30
   n'a encore été regardée dans cet état.

⚠️ **Ce qui a déjà été fait dans S140 et n'est pas à refaire** : `_admin_list`
sait rendre les deux formats ; le groupe de filtres et la recherche sont capturés
(`_filter_group`, `_list_search`) et imprimés à des endroits différents selon le
format ; `.admin-hero-note` est **exclu à la source** des deux blankets sombres de
`style.css` ; le flash est devenu une bande (plus de vide de 16 px sous « la
machine a bien été mise à jour »).

## S139 — la recherche cherche dans tout le produit — **en cours**

Signalée par l'opérateur les 2026-08-12 et 2026-08-16, en cinq rapports qui
disaient tous la même chose : `usb` (objet prêtable), `valentin` (projet),
`D251` (espace réservable), `anniversaire` (événement passé), les matériaux et
le corps des pages personnalisées ne renvoyaient rien.

| Étape | Résultat | État |
|---|---|---|
| **S139a** | `SiteSearch` : dix catalogues au lieu de quatre, chacun derrière sa propre feature ; thème sombre réparé | ✅ 2026-08-16 |
| **S139b** | les **destinations** : « horaires », « heures », « calendrier », « connexion » sont des concepts, pas des enregistrements | ✅ 2026-08-16 |
| **S139c** | la page passe aux composants partagés ; ce qui reste vraiment local entre dans `/admin/design#recherche` | ✅ 2026-08-16 |
| **S139c bis** | **toutes les routes legacy supprimées** — 44 chemins, 163 routes → 119 | ✅ 2026-08-16 |
| **S139d** | un événement passé se voit : `spent` sur `_catalogue_card`, une seule horloge, le pied d'inscription retiré | ✅ 2026-08-16 |

⚠️ **Le diagnostic « thème sombre » de la todo précédente était faux dans sa
cause.** Il pointait vers le balayage général de `style.css` ; c'était en fait
`background: white` écrit deux fois en clair dans le `<style>` local de
`search.html.twig`. Le balayage n'y était pour rien.

⚠️ **S139b — pourquoi la couverture ne suffit pas.** « horaires » ne matchera
jamais : aucune ligne ne porte ce nom, les horaires sont sept `OpeningHour`
rendus dans une carte du deck d'accueil, et personne ne tape « lundi
09:00–18:00 ». Il manque un **second type de résultat** : une liste déclarée des
surfaces du produit, chacune avec ses synonymes traduits dans les cinq
catalogues et derrière la même feature. La carte des horaires a besoin d'une
ancre pour qu'un résultat puisse y atterrir.

✅ **Un événement passé se voit — livré le 2026-08-16 (S139d).** `_catalogue_card`
apprend `spent` (affiche désaturée, titre éteint), le pied d'inscription
disparaît sur une carte passée, `past` se calcule sur **une seule** horloge, et
« 0 / 3 disponibles » devient « 0 / 3 à venir ». Détail et mesures dans
`HISTORY.md`.

⚠️ **Reste ouvert, séparément :** `/events` sans paramètre rend 0 carte quand il
n'y a aucun événement à venir, alors que « Tous » en compte 3. La vue par défaut
est « À venir » **délibérément** (cf. le commentaire du contrôleur) — ce n'est
pas un défaut, mais un état vide qui renvoie vers les événements passés vaudrait
mieux qu'une page nue.

⚠️ **todo, opérateur 2026-08-16 — balayer tout le CSS local des pages.** « Si tu
as besoin de CSS custom, ça vaut probablement le coup d'en faire une règle du
design system. » Pour chaque bloc `<style>` : soit il existe déjà un équivalent
partagé et on l'utilise, soit c'est une forme réellement nouvelle et elle entre
dans `/admin/design` **et** dans `components.css`. Dans les deux cas, **on évite
le CSS local à tout prix.**

**Mesuré le 2026-08-16, avant d'écrire cette ligne** (⚠️ et pas estimé — cf. la
prémisse fausse de S134j) : **1 232 règles CSS locales réparties sur 87
gabarits**. Les cinq plus gros en portent **432**, soit 35 % à eux seuls :

| Gabarit | Règles |
|---|---|
| `formation-suivi.html.twig` | 128 |
| `event-detail.html.twig` | 103 |
| `admin-design.html.twig` | 93 |
| `machine-historique.html.twig` | 65 |
| `admin-dashboard.html.twig` | 43 |

⚠️ **`admin-design.html.twig` est une exception légitime** : c'est le guide
lui-même, son chrome (`.dz-*`) n'est utilisé nulle part ailleurs et ne doit pas
l'être. Il reste dans le compte pour que le total soit honnête, pas parce qu'il
faut le vider.

⚠️ **La leçon de S139c/e est le critère de tri.** Trois fois dans une session, le
défaut n'était pas « cette page a du CSS » mais « les règles d'un composant
**partagé** vivaient dans une feuille que la page n'ouvre pas » —
`_cell_title` dans `admin.css`, puis sa couleur laissée à l'héritage, puis
`_breadcrumb` dans `details.css`. **Commencer par là** : chercher les partials
partagés dont les règles ne sont pas dans `components.css`, avant de compter des
lignes. C'est le sous-ensemble qui casse réellement des pages.

✅ **Routes legacy : supprimées le 2026-08-16.** 44 chemins, 163 routes → 119.
Tous les `.html`, tous les `_legacy`, `/machine` singulier, `/calendar` anglais,
les six redirections d'administration, et `/search` (doublon anglais de
`/recherche` — l'en-tête pointait dessus et pointe maintenant sur le canonique).
`LegacyAdminController` supprimé en entier. Détail dans `HISTORY.md`.

⚠️ **S139c — l'état mesuré.** `search.html.twig` porte **23 règles CSS locales**
pour **15 classes** à elle. Trois de ses cinq formes ont déjà un équivalent
livré : la carte de résultat est `_cell_title` + `_cell_state`, la pastille de
catégorie est `_cell_state` en signal `muted`, le panneau par catégorie est le
`frame: 'full'` de `_catalogue`. Seules les deux formes de « conseils » (carte
icône + titre + aide, et leur grille) sont réellement nouvelles — **ce sont les
seules qui méritent d'entrer dans `/admin/design`**. Documenter les cinq
bénirait une copie.

## S138 — la grille de cartes, partout — **en cours**

Format d'en-tête validé le 2026-08-12 (`/admin/design#catalogue`). Règle : **un
cadre autour de ce qui décide, rien autour de ce qu'on regarde.**

| Étape | Résultat | État |
|---|---|---|
| **S138a** | `_catalogue` apprend `frame: 'full'` | ✅ |
| **S138b** | les neuf grilles publiques le portent ; sous-lieu sur espaces, événements, prêts | ✅ |
| **S138c** | `frame: 'full'` devient le défaut, le paramètre disparaît | à faire |

⚠️ Seules `Place`, `Event` et `LoanableItem` portent un sous-lieu. Matériaux,
formations et badges n'en ont pas : pas de sélecteur.
⚠️ Le couple « libre/total » n'a pas de sens partout — `available_word` porte le
nom ; sans couple, l'en-tête retombe sur le total simple.

## S135 — le même objet partout — ✅ **2026-08-11**

Le format et le vocabulaire sur **toutes** les listes d'administration, plus les
six pages qui dessinaient leur propre chrome (RFID ×2, inscriptions, pages
introuvables, lieux, accès exceptionnels).

- Sept familles de pastilles d'état fusionnées en une (`_cell_state`, par signal).
- `/admin/access-rfid-logs` 10 colonnes → 5 ; `/admin/utilisateurs/{id}` 8 → 5.
- Le vocabulaire vit dans `components.css` : ⚠️ un `_cell_*` peut servir sur
  **n'importe quelle** page, y compris publique, qui ne charge pas `admin.css`.

⚠️ **Forme de référence : `/admin/utilisateurs`** — pastille d'identité, titre
fort, sous-titre discret, pastille d'état, nombres à droite, actions épinglées.
⚠️ Hors périmètre : les grilles de **cartes** publiques (voir S138).

## Phase G3 — les listes (S134h–S134j) — ✅ **LIVRÉE le 2026-08-11**

Format de liste appliqué depuis un composant unique, vocabulaire de colonnes
défini, listes remappées. Récit et défauts trouvés : `HISTORY.md`.

**Le format** (`/admin/design#filtres`) :
- **Bandeau** — titre à gauche, **un seul bouton vert nommé** à droite.
- **Panneau** — une rangée de tuiles pour la facette qui définit la page, puis
  « Affiner » en listes déroulantes, **sous-lieu en premier**, groupe absent sur
  une installation à un sous-lieu.
- **En-tête de liste** — titre, **compte, puis recherche**. Le compte devient
  « 3 sur 11 » dès qu'un filtre porte.

**Le vocabulaire** (`_cell_*`) : `media` · `titre + sous-titre` · `pastille
d'état` · `jauge` · `date` · `actions` · `vide`. Chacun définit son cas manquant.

⚠️ `_cell_date` **exige** `convention` (`machine` | `wall`) et ne devine pas :
se tromper est silencieux et faux du décalage du lab.
⚠️ Contrastes mesurés, onze états + bouton : 5,01:1 à 8,96:1 sombre, 5,01:1 à
6,23:1 clair.

🔴 **La prémisse chiffrée de S134j était fausse** : les « ~590 lignes de `<style>`
local » étaient les totaux de lignes des fichiers. Vérifier une mesure avant d'en
faire une session.

⚠️ **Reporté :** « sous-lieu » est un mauvais mot (opérateur). Renommage de
**catalogue**, pas de schéma — `Venue`/`VENUE` restent. Mot non choisi.

### S134c2 — FabOS invente le contenu d'une formation quand il est vide

**Constat, 2026-08-11, trouvé en terminant S134c.** `FormationPageContentService::DEFAULTS`
et `formation-detail.html.twig` fournissent, pour toute formation dont les champs
ne sont pas remplis :

- un **programme horaire en quatre points** — « 00:00 Accueil et sécurité »,
  « 00:30 Préparation du projet », « 01:15 Démonstration machine », « 02:00 Mise en
  pratique » — avec leurs descriptions ;
- **trois sessions à venir** — « Mardi prochain 14:00-16:30 · Places disponibles »,
  « Jeudi prochain », « Vendredi prochain · Complet » ;
- **trois objectifs pédagogiques**, **deux prérequis** et **trois éléments de
  matériel fourni**.

Rien de tout cela n'existe. Ce sont des exemples de démonstration, servis à un
membre comme s'ils décrivaient la formation qu'il s'apprête à suivre — y compris
des créneaux auxquels il ne peut pas s'inscrire, puisqu'ils ne sont rattachés à
aucune donnée. C'est **faux dans les cinq langues** : ce n'est pas un manque de
traduction, et c'est précisément pourquoi S134c a laissé ces valeurs littérales
plutôt que de les traduire.

**Règle appliquée (opérateur, 2026-08-11) :** on traduit l'interface, jamais le
contenu ; une formation existe dans une langue et c'est tout. Le corollaire est
qu'un contenu que l'opérateur n'a pas écrit ne doit pas exister du tout.

**Attendu :** un champ vide n'affiche pas de bloc. Pas de programme inventé, pas de
session inventée, pas d'objectif inventé. Là où un bloc vide serait déroutant pour
l'opérateur, l'écran d'édition du contenu peut proposer ces exemples comme
*point de départ à remplir* — jamais la page publique comme *fait*.

⚠️ **Ne pas confondre avec les libellés.** Les titres de section (« Description
détaillée », « Objectifs pédagogiques », « Programme horaire », « Prochaines
sessions »), les trois cartes expliquant le parcours guidé, la formulation de la
validation pratique et les deux cartes de navigation en pied de page sont
l'interface : ils sont déjà des clés de catalogue depuis S134c et doivent le
rester. Seules les **valeurs** sous ces titres sont en cause.

**Fichiers :** `src/Service/FormationPageContentService.php` (constante `DEFAULTS`,
blocs `program` et `sessions`) et `templates/site/formation-detail.html.twig`
(les trois `{% set %}` de `objectives`, `prerequisites`, `material`).

**Si la capacité impose une coupe :** S134g puis S134f peuvent passer après le
commerce sans casser de dépendance. Le champ sous-lieu de S133, S134c et
S134d/S134e ne le peuvent pas — ce sont respectivement le critère de sortie de la
Phase G, la promesse des cinq langues et le manque de modèle signalé par
l'opérateur.

## Phase H — commerce facultatif

🔴 **Renumérotée S135–S139 → S150–S154 le 2026-08-16, et la Phase I S140–S142 →
S155–S157.** Les quatre numéros S135, S136, S137 et S138 avaient déjà été
*livrés* comme sessions d'interface les 11 et 12 août pendant que cette table les
réservait au commerce : deux sessions différentes portaient le même nom dans le
même document, et `HISTORY.md` en décrit une pendant que cette page décrivait
l'autre. Les numéros livrés gardent leur sens — ce sont eux qui existent dans le
code et dans l'historique ; c'est le travail **non commencé** qui se déplace.
L'écart jusqu'à S150 est délibéré : il laisse dix numéros à la suite de
l'interface sans reproduire la collision. **S139 est pris par la recherche**
ci-dessous ; le prochain numéro libre est donc **S140**.

Le commerce reste entièrement désactivable. Les offres apparaissent dans leur workspace métier ; commandes, paiements, remboursements et rapprochement utilisent un moteur commun. Le retour navigateur ne confirme jamais un paiement : seul un webhook fournisseur vérifié ou sa réconciliation peut le faire. Chaque événement fournisseur a une clé unique et chaque ligne de commande garde un fulfillment persistant/outbox pour produire un effet métier exactement une fois malgré les retries et crashs. La livraison passe par le service métier normal — attribution de package, stock ou ledger de temps — sans modifier directement voter, badge, qualification, quota ou réservation.

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S150** | catalogue d'offres et prix : package, matériau, temps machine/personne, formation ; aucune transaction | Terra + Luna | références stables, devises/taxes, archivage, aucune permission implicite |
| **S151** | commandes, paiements et adaptateurs fournisseur ; checkout, webhooks, réconciliation, remboursements/chargebacks et audit | Terra + Luna | signature, event ID unique, at-least-once, outbox, pannes, aucun secret de paiement stocké |
| **S152** | livraison packages et matériaux ; hold stock atomique ou backorder explicite ; compensations par ligne | Terra + Luna | attribution via Usage Rights, zéro survente, refund partiel/avant-après livraison, aucune autre source révoquée |
| **S153** | ledger append-only des crédits de temps machine/personne et achats de formation | Terra + Luna | grant/hold/consume/release/expire/refund, concurrence, unités/scopes, annulation/no-show, aucune réservation automatique |
| **S154** | reporting commerce, rapprochement et audit UX transversal | Terra + Luna | totaux, remboursements, exports scoped, sombre/mobile/i18n |

## Phase I — communication Formation avancée

Cette phase est volontairement placée très loin après le workspace Formation initial. FabOS reste la source de vérité de la conversation ; l'e-mail est une copie de notification par destinataire et une panne d'envoi ne perd jamais le message interne. Les trois visibilités sont distinctes : annonce formateur→cohorte sans exposer la liste, fil privé formateur↔un étudiant, ou groupe explicitement composé. Une réponse à une annonce devient privée par défaut ; aucun message privé ne peut basculer implicitement vers la cohorte.

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S155** | conversations privées/annonces/groupes, messages texte bornés, participants, non-lus et permissions | Terra | IDOR, aucune promotion privé→collectif, changements d'inscription, rate-limit, échappement, audit |
| **S156** | interface formateur/étudiant et duplication e-mail asynchrone par destinataire | Terra + Luna | revalidation avant envoi, confidentialité, retry/déduplication, préférences, cinq langues, mobile/a11y |
| **S157** | modération, archivage, export et politique de conservation de la messagerie Formation | Terra + Luna | suppression/anonymisation, abus, pièces jointes si ajoutées, conformité |

## Décisions opérateur complémentaires

- **Admin recovery n'est pas un bypass de sécurité.** Il récupère et administre FabOS hors packages, mais ne contourne ni badges/formations requis pour utiliser une ressource, ni arrêt de sécurité. Une attribution ou révocation administrative reste une action explicite et auditée.
- **Deux droits seulement : Use et Manage.** Il n'existe aucun droit Report ; consultation, reporting et export sont inclus dans Manage sur le même scope, sans jamais conférer Use.
- **Formation est un catalogue global au FabOS.** Seules les sessions physiques sont rattachées à un sous-lieu.
- **L'exposition publique suit une politique par surface.** Profil, annuaire public, galerie, leaderboard et API publique exigent activation opérateur et consentement membre ; les vues internes nécessaires à une tâche autorisée suivent leurs droits et leur finalité métier. Le consentement image/galerie reste distinct du profil public.
- **Les sessions IdP ont une grâce bornée.** Une désactivation connue provoque une révocation immédiate. Une panne empêche toute nouvelle connexion mais laisse les sessions existantes valides au plus 24 heures, avec revalidation dès le retour de l'IdP ; suspension locale et revoke-all restent disponibles.

## Travaux transversaux conservés

Ces besoins ne contredisent pas la nouvelle architecture mais passent après les fondations dont ils dépendent :

- sécurité et confidentialité restantes de Phase H, notamment test réel du booking et requêtes groupées ;
- mot de passe oublié et suppression/anonymisation de compte ;
- verrou d'annulation, no-show uniquement sur ressources disposant d'un signal ;
- archivage plutôt que suppression ;
- files d'attente, stockage/retrait, défauts déclarés et motif d'utilisation ;
- exceptions d'horaires intégrées au workspace Lieux ;
- audit et notes nécessaires à toute action Manage sur autrui.

RFID/cartes physiques et 2FA restent hors scope. La stabilisation multi-lieux (Phase G) précède le commerce et les crédits (Phase H), puis la messagerie Formation avancée (Phase I). La réservation d'un pool de machines n'est pas impliquée par les catégories.

## Nettoyage effectué

Les longues sections S59, S77, S78, S80 et S81–S85, déjà livrées, ont quitté cette roadmap ; leurs décisions utiles sont dans `HISTORY.md` et `UI-CONSISTENCY.md`. Les anciens chantiers S58, S62, S65, S67 et S74 ne sont plus planifiés séparément : leur travail restant est absorbé par S103–S126. Les hypothèses de portail de S100–S101 restent dans l'historique, mais ne pilotent plus aucune migration future.
