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

| Session | Résultat attendu | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S129** | workspace **Lieux** réellement opérable : liste, création, édition, archivage sécurisé et accès clair aux horaires ; un sous-lieu par défaut non supprimable | Terra + Luna | migration sans perte, slug/nom/timezone, objets rattachés, aucun lockout ; route, onglets et sidebar issus de la même métadonnée |
| **S130** | navigation admin dédoublonnée et conforme à la décision : « Utilisateurs » remplace « Le lieu » ; un workspace **Lieux** porte Sous-lieux et Horaires ; **Matériaux** rejoint **Équipement** ; « Pages du Lab » devient **Pages personnalisées** ; **Réglages** devient l'entrée par défaut de Configuration, qui contient aussi État installation et Thèmes ; Réseau et Packages ont chacun une seule entrée canonique | Luna + Terra | inventaire de toutes routes, active-state correct sur créations/éditions, redirections historiques conservées, mobile/clavier/sombre/cinq langues |
| **S131** | contexte sous-lieu uniforme sur toutes les listes/workspaces qui portent un sous-lieu, avec conservation de `location`, recherche, facettes, onglets et pagination | Terra + Luna | `?location=` valide/refus explicite hors scope, défaut agrégé, préférence profil seulement comme défaut, aucun filtre local concurrent, tests des URLs partageables |
| **S132** | audit de parité et retrait des dernières copies de menus/filtres/templates admin ; footer systématique et CSS responsive centralisés | Luna + Sol | capture Artemis de chaque workspace, tests Twig/routes, aucun CSS/markup local de shell, aucun footer copié/absent, une seule sidebar + sous-navigation, documentation et matrice registry à jour |
| **S133** | gestion des droits réellement conforme : groupes intégrés/protégés et locaux, attributions personne/groupe, grants v2 Use/Manage et scopes administrables ; comparaison shadow visible et explicable | Terra + Luna | dernier Admin, User/Guest virtuels, dates, union des packages, scope AND, CSRF/IDOR/mass-assignment, aucune élévation par UI |
| **S134** | activation graduelle des grants v2 sur les seuls chokepoints audités, puis retrait du package legacy binaire | Terra + Sol | parité shadow sans différence inexpliquée, voters/services atomiques pour chaque écriture, Manage ≠ Use, refus hors scope et rollback par feature |

Ces sessions sont **bloquantes avant le commerce** : ne pas commencer paiement, catalogue d'offres ou ledger tant que l'opérateur ne peut pas découvrir puis administrer un sous-lieu depuis l'interface canonique.

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

## Phase H — commerce facultatif

Le commerce reste entièrement désactivable. Les offres apparaissent dans leur workspace métier ; commandes, paiements, remboursements et rapprochement utilisent un moteur commun. Le retour navigateur ne confirme jamais un paiement : seul un webhook fournisseur vérifié ou sa réconciliation peut le faire. Chaque événement fournisseur a une clé unique et chaque ligne de commande garde un fulfillment persistant/outbox pour produire un effet métier exactement une fois malgré les retries et crashs. La livraison passe par le service métier normal — attribution de package, stock ou ledger de temps — sans modifier directement voter, badge, qualification, quota ou réservation.

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S135** | catalogue d'offres et prix : package, matériau, temps machine/personne, formation ; aucune transaction | Terra + Luna | références stables, devises/taxes, archivage, aucune permission implicite |
| **S136** | commandes, paiements et adaptateurs fournisseur ; checkout, webhooks, réconciliation, remboursements/chargebacks et audit | Terra + Luna | signature, event ID unique, at-least-once, outbox, pannes, aucun secret de paiement stocké |
| **S137** | livraison packages et matériaux ; hold stock atomique ou backorder explicite ; compensations par ligne | Terra + Luna | attribution via Usage Rights, zéro survente, refund partiel/avant-après livraison, aucune autre source révoquée |
| **S138** | ledger append-only des crédits de temps machine/personne et achats de formation | Terra + Luna | grant/hold/consume/release/expire/refund, concurrence, unités/scopes, annulation/no-show, aucune réservation automatique |
| **S139** | reporting commerce, rapprochement et audit UX transversal | Terra + Luna | totaux, remboursements, exports scoped, sombre/mobile/i18n |

## Phase I — communication Formation avancée

Cette phase est volontairement placée très loin après le workspace Formation initial. FabOS reste la source de vérité de la conversation ; l'e-mail est une copie de notification par destinataire et une panne d'envoi ne perd jamais le message interne. Les trois visibilités sont distinctes : annonce formateur→cohorte sans exposer la liste, fil privé formateur↔un étudiant, ou groupe explicitement composé. Une réponse à une annonce devient privée par défaut ; aucun message privé ne peut basculer implicitement vers la cohorte.

| Session | Résultat livré | Réalisation | Contrôle Sol |
|---|---|---|---|
| **S140** | conversations privées/annonces/groupes, messages texte bornés, participants, non-lus et permissions | Terra | IDOR, aucune promotion privé→collectif, changements d'inscription, rate-limit, échappement, audit |
| **S141** | interface formateur/étudiant et duplication e-mail asynchrone par destinataire | Terra + Luna | revalidation avant envoi, confidentialité, retry/déduplication, préférences, cinq langues, mobile/a11y |
| **S142** | modération, archivage, export et politique de conservation de la messagerie Formation | Terra + Luna | suppression/anonymisation, abus, pièces jointes si ajoutées, conformité |

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
