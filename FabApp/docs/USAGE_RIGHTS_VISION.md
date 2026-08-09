# FabOS multi-lieux, droits et réseau — vision cible

**Statut :** décisions produit approuvées, architecture et migrations à construire · **Mise à jour :** 2026-08-09

Ce document remplace la vision S100–S101 qui associait encore les packages et certains scopes aux portails. Le code actif reste celui de S97–S99 tant que chaque migration n'a pas été construite, comparée en mode simulation, déployée et explicitement activée.

## Décisions produit enregistrées

1. **Les portails s'arrêtent.** Un service autonome reçoit sa propre installation FabOS, son thème, ses features et ses administrateurs, au lieu de publier un sous-ensemble des données d'une autre instance.
2. **Une installation gère plusieurs sous-lieux physiques uniquement quand tout le reste est commun.** Un sous-lieu partage gouvernance, comptes locaux, données, règles, packages, réservation et audit ; il ne sert pas à isoler un service autonome.
3. **Le sous-lieu est un contexte visible.** Un membre peut choisir ses sous-lieux préférés dans son profil et les remplacer sur une page. La préférence n'est jamais une autorisation.
4. **Un package appartient au FabOS.** Ses grants peuvent être limités à un ou plusieurs sous-lieux et à des scopes métier plus fins.
5. **Un package est attribuable à un individu ou à un groupe.** Les droits effectifs sont l'union des grants actifs ; il n'existe pas de deny implicite.
6. **L'administration de récupération est hors packages.** Un administrateur global ne peut pas perdre l'accès de récupération à cause d'un package supprimé ou mal configuré.
7. **Les groupes métier sont locaux et modifiables.** Manager, Staff, Super user et User sont amorcés par défaut ; Stagiaire, Bénévoles ou toute autre organisation peuvent être ajoutés.
8. **Les packages présentent trois types de droit :** Use, Report et Manage.
9. **Les droits suivent les features et leurs sous-sections.** Une source centrale décrit navigation, scopes, filtres, réservations, quotas et reporting.
10. **Les réservations et quotas apparaissent dans chaque workspace de feature**, tout en conservant des services et modèles communs derrière les écrans.
11. **Les badges sont globaux au FabOS, cumulatifs et sans scope de sous-lieu.** Une attribution n'est jamais effacée ; une erreur se corrige par révocation auditée.
12. **Les utilisateurs peuvent publier un profil public opt-in** et déclencher un import ponctuel et consenti vers un autre FabOS par QR éphémère.
13. **Les institutions deviennent des connexions à d'autres FabOS.** Les synchronisations sont explicites, signées, idempotentes et limitées aux objets choisis.
14. **Le design reste centralisé.** Les listes personnalisent leurs colonnes et données, jamais leur shell, leur CSS ou leur mécanique de filtres.
15. **Configuration → Thèmes rassemble l'identité publique.** Couleurs, logos/images, nom et ordre des menus, blocs/contenu/ordre de la homepage se règlent et se prévisualisent au même endroit.
16. **SSO partage l'authentification, jamais l'autorité ni les données.** LDAP/OIDC/SAML simplifient la connexion ; chaque FabOS conserve comptes locaux, groupes, packages, quotas, audit et recovery Admin.

## Règle de topologie

Créer un **sous-lieu dans le même FabOS** seulement si les unités partagent gouvernance, administrateurs de confiance, annuaire métier, politiques, moteur de réservation, cycle de vie, responsabilité des données et besoin de vue agrégée. La vue par défaut montre alors toutes les machines/ressources autorisées de tous les sous-lieux ; le filtre sert à affiner, pas à simuler des tenants.

Créer un **autre FabOS** dès qu'un service veut sa propre administration, son thème, son catalogue, ses règles, sa rétention ou son rythme de déploiement. La distance physique n'est pas le critère. Une identité commune rend la connexion fluide, tandis que le Réseau FabOS/Institution transporte uniquement les données explicitement partagées.

Chaque objet possède une instance propriétaire. Une autre instance reçoit au plus une projection signée/read-only et renvoie vers la source pour réserver, s'inscrire, annuler ou gérer. Une réservation distribuée, un calendrier global ou une billetterie cross-instance nécessiteraient un protocole distinct ; ils ne sont pas promis par la synchronisation Institution.

## Contradictions et arbitrages visibles

Cette table est volontairement explicite. Une ligne marquée **À décider** bloque l'activation de la partie concernée ; elle ne bloque pas la construction de ses fondations en mode simulation.

| Sujet | Nouvelle décision | Contradiction actuelle | Résolution proposée | État |
|---|---|---|---|---|
| Portails | arrêt complet | `PORTAL`, `PortalContext`, hostnames, `SITE_SETTING`, `SITE_MODULE`, packages, mails, logs et Twig sont portal-scopés | gel, rapport de différences, consolidation dans le scope global, retrait des consommateurs, puis migration destructive séparée | Décidé |
| Packages | valables sur un FabOS, grants par sous-lieu | `USAGE_PACKAGE.portalId` impose actuellement un portail | migrer tous les packages vers l'instance ; remplacer `portalId` par scopes de grants | Décidé |
| Sous-lieu / espace | le sous-lieu est physique ; un espace est réservable | `Place.localisation` est du texte et l'ancienne vision appelait Venue « lieu » | entité canonique `Venue` affichée « Sous-lieu » ; `Place` reste « Espace » et référence un sous-lieu | Décidé |
| Filtre profil | préférence de présentation | un filtre global caché pourrait masquer des données ou sembler accorder un droit | choisir `URL explicite valide > préférence valide > Tous les sous-lieux autorisés`, puis évaluer séparément le droit de l'action ; une URL hors scope produit un refus explicite | Décidé |
| Plusieurs préférences | l'utilisateur peut filtrer par sous-lieux | « un sous-lieu préféré » et « plusieurs favoris » sont deux UX différentes | stocker une sélection préférée ; garder un sous-lieu actif par page et proposer « Tous » seulement si autorisé | Proposition |
| Groupes par défaut | Admin, Manager, Staff, Super user, User, Guest demandés | Admin est une autorité de récupération ; Guest peut être anonyme et n'a aucun compte à mettre dans un groupe | afficher Admin et Guest dans les audiences, mais Admin reste système/protégé et Guest une audience virtuelle ; seuls les groupes métier sont librement éditables | Proposition forte |
| Formateur | n'apparaît plus dans les groupes par défaut | `ROLE_TRAINER` alimente aujourd'hui plusieurs parcours | migrer les membres vers un groupe Formateurs automatiquement, puis laisser l'opérateur le conserver, renommer ou fusionner | À confirmer |
| « Admin tout le temps » | éviter tout lockout | l'ancienne décision préservait badges/formation et arrêts de sécurité même pour Admin ; le code actuel contourne déjà certaines qualifications | Admin contourne packages et quotas pour recovery, mais pas les arrêts techniques ni exigences physiques de sécurité ; compte recovery local, verrou transactionnel du dernier Admin, procédure CLI/offline et audit ; override sécurité séparé, temporaire et supervisé | À confirmer |
| Use / Report / Manage | trois niveaux lisibles dans les packages | « Report » peut signifier déclarer une panne, lire des métriques ou exporter des données ; « Manage » peut modifier, supprimer, publier ou remettre en service | garder ces trois niveaux dans l'UX, mais les mapper vers des capacités atomiques namespacées par feature/section/opération ; chaque route mutante garde son voter/service | Décidé |
| Implications | Manage agit sur autrui | Manage peut avoir besoin de lire les rapports, mais ne doit pas qualifier personnellement l'opérateur | Manage implique la lecture opérationnelle nécessaire et peut impliquer Report sur le même scope ; il n'implique jamais Use | À confirmer |
| Scope package / attribution | même package, bénéficiaires dans des lieux différents | scope uniquement dans le package oblige à dupliquer le package | grants portent la portée normale ; une attribution peut seulement la **restreindre**, jamais l'élargir | Proposition |
| Union des packages | all access lieu 1 + user lieu 2 | avec une union pure, un package global User pourrait ré-élargir le lieu 2 | aucun package de base ne doit être global par accident : chaque grant est explicitement scopé ; les unions restent monotones, sans deny caché | Décidé |
| Plusieurs politiques de quotas | plusieurs grants/packages sont évalués en OR | fusionner horizon, caps et délais champ par champ fabriquerait une politique que personne n'a configurée | évaluer chaque chemin complet grant + calendrier + politique ; autoriser si un chemin entier passe et journaliser le gagnant ; ordre stable si plusieurs passent | Décidé |
| Guest et événements publics | Guest devient une audience système | `guestsAllowed` contourne aujourd'hui Usage Rights pour les événements publics | préserver les événements déjà publics pendant la migration ; décider si `guestsAllowed` reste l'autorité publique ou devient un grant Guest, en séparant visibilité et inscription | À décider |
| Badges | jamais supprimés, globaux | l'UI actuelle permet de supprimer une définition ; `UTILISATEUR_BADGE` ne garde pas provenance/formation | distinguer définition archivable et attribution append-only ; révocation auditée sans effacement | Décidé |
| Badge et formations | une ou plusieurs formations débloquent un badge | `Formation` pointe vers un seul Badge et l'attribution utilisateur ne mémorise aucune formation source | relation many-to-many Badge–Formation + preuve d'attribution avec sources et émetteur | Décidé |
| Institution | autre FabOS reconnu | `Institution` est seulement nom + URL et peut représenter un organisme non-FabOS | conserver éventuellement Institution descriptive et introduire un objet séparé `FederatedPeer` pour une instance de confiance ; décider si les deux restent visibles | À décider |
| Profil QR | import ponctuel | une URL publique permanente ne prouve ni consentement actuel ni authenticité ; un scanner peut ouvrir le GET avant l'utilisateur | token aléatoire ≥128 bits, hash seul stocké, purpose/audience/expiration ; GET ne consomme rien, confirmation POST atomique, no-referrer, rate-limit et révocation | Décidé |
| Sync membres | l'institution peut synchroniser des membres | une réplication de comptes peut écraser l'identité locale et exposer des données personnelles | ne synchroniser que profils consentis et claims sélectionnés ; jamais mot de passe, RFID, rôles ou packages locaux | Proposition forte |
| « Badge jamais supprimé » | accumulation | fraude, erreur ou retrait d'habilitation doivent rester corrigibles | historique immuable + statut révoqué/expiré ; le fait historique reste, l'autorisation cesse | À confirmer |
| Machines fédérées | partager make/model, photo, matériaux, specs | `Machine` n'a pas marque/modèle ; catégorie libre ; nom contient souvent la marque | `MachineMake`, `MachineModel`, catégorie canonique et exemplaire local avec overrides/provenance | Décidé |
| Matériaux | sous Équipement | `materials` est aujourd'hui une feature et un menu autonomes | déplacer la présentation dans le workspace Équipement ; conserver une capacité interne distincte si le feature gate doit rester configurable | À confirmer |
| Réservations | onglet de chaque feature | page admin transversale ; EventRegistration et formations n'utilisent pas tous `Reservation` | moteur/adaptateurs communs, vues par feature ; retirer la vue globale seulement après parité et redirections | Décidé |
| Quotas | propres à chaque feature | matrice centrale `BOOKING_POLICY` liée aux tiers ; événements/formations ont d'autres règles | profils par feature avec champs communs et extensions ; aucune fusion champ par champ entre profils | Décidé |
| Reporting | onglet de chaque feature et droit Report | aucun socle reporting et aucune définition stable des métriques | construire le shell et le contrat d'adaptateur avant la première métrique ; ne jamais afficher un onglet vide | Décidé |
| Filtres demandés | nombreux scopes métier | plusieurs relations n'existent pas : catégorie utilisateur/espace, département, responsable, organisateur, institution de formation | créer d'abord taxonomies et relations canoniques ; aucun filtre ne doit analyser du texte libre comme autorité | Décidé |
| « Le lieu devient Utilisateurs » | réorganisation navigation | formulation ambiguë entre groupe de menu et modèle physique | interprétation mise à jour : « Le lieu » devient « Utilisateurs » ; Horaires passe dans « Lieux » ; Interface/contenu d'accueil dans « Configuration → Thèmes » | À confirmer |
| Thème et accueil | couleurs, images, menus et homepage doivent être réunis | données aujourd'hui dispersées entre Site settings, Portal overrides, NavBuilder, HomepageSectionVisibility et éditeur Homepage | un workspace Thèmes central au niveau FabOS, avec modèle structuré, preview, publication et rollback ; aucun CSS/menu libre injecté | Décidé |
| Formations et sous-lieux | filtres catégorie/département/institution/formateur | on ne sait pas si le catalogue Formation ou ses futures sessions sont localisées | recommandation : catalogue global, sessions physiques rattachées à un sous-lieu | À confirmer |
| Matériaux et stocks | matériaux sous Équipement | une fiche globale et un stock local ne sont pas le même objet | recommandation : catalogue matière global, stocks par sous-lieu | À confirmer |
| Prototype S100–S101 | montrait portails et deux plans d'autorisation séparés | il est désormais partiellement obsolète | conserver S100–S101 dans l'historique ; remplacer les maquettes Développement lors de la première session UX | Décidé |
| Profils privés par défaut | nouveau profil public opt-in | Équipe/Formateurs, leaderboard/API, historiques, galerie et kiosk exposent déjà des identités ou statistiques selon d'autres règles | inventorier toutes les expositions et décider lesquelles rejoignent le consentement profil ou gardent une base séparée explicitement documentée | À décider |
| SSO transparent | l'usager retrouve plusieurs FabOS sans nouveau mot de passe | LDAP/OIDC/SAML ne provisionnent pas automatiquement les droits, ne révoquent pas toujours les sessions actives et l'e-mail n'est pas une identité stable | `ExternalIdentityLink(issuer, subject)` unique vers un compte local ; JIT minimal, issuer allowlisté, aucune fusion e-mail ni claim vers Admin/Manage | Décidé |

## Modèle cible

### Instance, sous-lieu et espace

- **Instance FabOS :** installation autonome, identité fédérée et configuration globale, avec une seule autorité sur ses données.
- **Sous-lieu (`Venue`) :** implantation physique avec slug stable, nom, adresse, fuseau, état et horaires ; jamais une frontière d'administration ou de données.
- **Espace (`Place`) :** salle, atelier ou poste réservable rattaché à un sous-lieu.

La migration crée un sous-lieu par défaut et lui rattache les données existantes avant de rendre les clés étrangères obligatoires. Les événements externes ou en ligne peuvent rester sans sous-lieu. Sans préférence explicite, les catalogues agrègent tous les sous-lieux autorisés.

### Identité externe et compte local

`ProviderRegistry` décrit les fournisseurs autorisés ; OIDC est livré d'abord, LDAP/SAML restent des adaptateurs fondés sur des bibliothèques éprouvées. L'identité canonique est `(issuer/provider, subject immuable)`, jamais e-mail ou username. Un lien externe unique pointe vers un compte FabOS local qui porte seul suspension, groupes, packages, préférences et audit. Deux subjects partageant un e-mail restent deux comptes ; un changement d'e-mail conserve le même lien.

Le provisioning JIT crée seulement le compte local minimal. Aucun claim externe ne confère Global Admin, Manage ou un package. La liaison de deux comptes demande preuve des deux identités ou action admin auditée ; retirer le dernier moyen de connexion est refusé. Le recovery Admin reste local, hors IdP et disponible pendant une panne.

OIDC valide issuer, audience, redirect URI, signature/algorithme, horloge, state, nonce et PKCE. La politique de sessions couvre TTL, revalidation, back-channel logout si disponible, revoke-all local, rotation des clés et comportement lorsque l'IdP est indisponible. Une nouvelle connexion échoue fermée ; le délai de grâce des sessions déjà ouvertes reste une décision opérateur.

### Groupes et audiences système

- **Administrateur global :** recovery système, protégé, jamais dépendant d'un package, dernier admin impossible à retirer.
- **Guest anonyme :** audience virtuelle pour les actions réellement publiques.
- **Groupes locaux :** Manager, Staff, Super user, User, puis groupes libres. Un compte peut appartenir à plusieurs groupes.

L'interface peut afficher Admin et Guest dans la même liste d'audiences, mais doit les marquer `Système · protégé` et ne pas simuler un groupe éditable.

### Packages et grants

Un package est un modèle réutilisable de l'instance. Use, Report et Manage sont ses trois niveaux **d'édition UX**. Le registre de sécurité les décompose en capacités atomiques, par exemple `maintenance.report.create`, `maintenance.analytics.view`, `maintenance.analytics.export`, `maintenance.intervention.update` ou `maintenance.return_to_service`. Une déclaration membre n'est jamais une permission de changer le statut d'une machine, et Manage générique ne remplace jamais la protection d'une opération sensible.

Chaque package contient des grants :

| Champ | Sens |
|---|---|
| feature | domaine FabOS central |
| section | sous-section optionnelle, par exemple Maintenance |
| action | `use`, `report` ou `manage` |
| scope | sous-lieu et dimensions métier autorisées par la feature |
| calendrier | horaires normaux, 24/7 ou fenêtre future |
| politique | profil de quotas/délais éventuel |

Les dimensions d'un grant se combinent avec **AND** : « catégorie Laser au sous-lieu Nord ». Plusieurs grants, packages et sources se combinent avec **OR**. Une attribution individuelle ou de groupe peut ajouter une restriction, jamais élargir le grant. Si plusieurs chemins portent des politiques de quotas, chaque chemin grant + calendrier + politique est évalué entièrement ; les champs de politiques ne sont jamais fusionnés. Le chemin gagnant est journalisé selon un ordre stable.

### Ordre d'évaluation

1. Compte, feature, sous-lieu et objet actifs.
2. Résolution Admin recovery ou des packages individuels/groupes/audiences.
3. Grant Use, Report ou Manage couvrant l'action, l'instant et tous les scopes.
4. Qualification, badge, formation et invariants métier.
5. Contraintes physiques non contournables : fermeture technique, capacité, chevauchement, alignement, buffer.
6. Chaque politique complète de quotas/délais de la feature, sans assemblage champ par champ.
7. Journalisation de la source, du package, du grant, du scope et de la politique gagnants.

Les refus nomment la première couche actionnable. L'UI consomme le même verdict que le service ou voter ; la navigation cachée n'est jamais la sécurité. Les réservations existantes restent acquises lors d'un changement de package ou de politique, sauf opération de réconciliation séparée et explicitement lancée.

## Source centrale des features

Un `FeatureWorkspaceRegistry` doit être l'unique définition **de métadonnées** : label, icône, feature gate, sections, actions disponibles, scopes permis, filtres, onglets, routes, réservations, quotas et reporting. Il ne devient pas une autorité métier monolithique : adaptateurs de feature, voters et services restent les chokepoints. NavBuilder, packages, listes et contrôleurs ne maintiennent pas quatre mappings parallèles, et un test automatique exige que chaque route mutante possède un contrôleur d'autorisation déclaré.

| Workspace | Onglets / sous-sections cibles | Scopes et filtres métier |
|---|---|---|
| Équipement | Machines, Catégories, Modèles & marques, Matériaux, Maintenance, Réservations, Quotas, Reporting | sous-lieu, catégorie, machine |
| Événements | Événements, Organisateurs, Lieux d'événement, Inscriptions, Quotas/capacités, Reporting | sous-lieu, organisateur, lieu d'événement |
| Prêts | Objets, Catégories, Prêts, Quotas, Reporting | sous-lieu, catégorie, objet |
| Espaces | Espaces, Catégories, Réservations, Quotas, Reporting | sous-lieu, catégorie, responsable, département |
| Formations | Formations, Catégories, Sessions/progression, Formateurs, Reporting | catégorie, département, institution, formateur ; sous-lieu pour session physique |
| Badges | Badges, Attributions, Reconnaissances/émetteurs, Journal | global FabOS, aucun sous-lieu |
| Galerie de projets | Projets, Modération, Reporting | à déclarer global ou sous-lieu, jamais implicite |
| Pages personnalisées | Pages, Navigation, Publication | global par défaut |
| Utilisateurs | Utilisateurs, Groupes, Profils publics, Échanges QR, Reporting | catégorie utilisateur |
| Lieux | Sous-lieux, Horaires | sous-lieu |
| Packages | Packages, Attributions, Quotas, Audit | sous-lieu dans les grants |
| Réseau FabOS | Connexions, Synchronisations, Journal | instance distante et objets partagés |
| Configuration | État installation, Features, Thèmes, Réglages, E-mails, Initialisation, Développement | administration technique globale |

### Contrat de l'onglet Thèmes

L'opérateur dispose d'un seul éditeur pour : nom public du FabOS, logos et images de marque, couleurs/tokens autorisés, libellé et ordre des entrées de menu, choix/ordre/contenu des blocs de homepage. L'écran utilise les vrais composants du site dans une prévisualisation desktop/mobile et sombre/clair, puis sépare **Enregistrer le brouillon** de **Publier** et permet de revenir à la dernière version publiée.

Le stockage peut rester techniquement séparé entre identité, navigation et contenu, mais un service de thème versionné en est l'unique façade. L'ordre des menus référence des clés stables du `FeatureWorkspaceRegistry`, jamais des routes ou du HTML libres : une entrée désactivée ou non autorisée ne réapparaît pas par réordonnancement. Les images passent par `ImageNormalizer`; couleurs et contenu sont validés/échappés. Les valeurs de branding encore portées par les portails sont inventoriées en S105 avant consolidation, sans choisir silencieusement un gagnant.

Use, Report et Manage sont des permissions de grants, **pas trois onglets**. Les onglets représentent les tâches. Un onglet absent signifie que le verdict ne permet pas de l'ouvrir ; un workspace sans donnée autorisée affiche un état vide explicable.

## Contrat commun des listes et filtres

Toutes les listes suivent cet ordre :

1. titre, compteur, action primaire ;
2. contexte sous-lieu compact si pertinent ;
3. onglets du workspace ;
4. un seul axe de filtres primaires en un clic, six choix maximum ;
5. recherche et `Plus de filtres` pour les dimensions secondaires ;
6. chips supprimables des filtres actifs, compteur et `Tout effacer` ;
7. table ou cartes partagées.

La résolution d'affichage du sous-lieu est `?location=` explicite et valide, puis préférence valide du profil, puis **Tous les sous-lieux autorisés**. Le scope d'autorisation de chaque action est évalué séparément. Une URL hors scope affiche un refus explicite ; elle ne bascule jamais silencieusement. `Tous` dépend du droit de lister les objets concernés, pas du droit Use. Le contexte reste dans l'URL, la pagination, la recherche et les onglets.

Le shell reste `_admin_list` + `_data_table`, étendu par des partials partagés (`scope_context`, `feature_tabs`, `applied_filters`, filtre avancé) et un registre de définitions. Chaque liste fournit ses colonnes, données et facettes ; aucune ne copie CSS ou JavaScript. Maximum cinq colonnes, aucune largeur minimale locale, ligne entière cliquable seulement avec une destination unique.

## Profils publics et échange ponctuel

Le profil public est désactivé par défaut. L'utilisateur choisit séparément identité, avatar, statistiques, formations et badges, avec aperçu exact. La route cible est `/m/{slug}`.

Import ponctuel : QR contenant un token aléatoire d'au moins 128 bits dont seul le hash est stocké, un purpose, une audience, une expiration courte et l'identité de l'instance source. Le GET d'aperçu ne consomme jamais le token — les scanners de QR ouvrent parfois les liens. La confirmation POST consomme atomiquement le token, après aperçu des seuls champs autorisés. Le token est révocable, rate-limité, absent des logs/referrers et protégé par `Referrer-Policy: no-referrer`. Provenance et horodatage sont conservés. Une resynchronisation est une nouvelle action manuelle ; la synchronisation continue reste distincte, consentie et révocable.

## Réseau FabOS, badges et machines

Une connexion FabOS (`FederatedPeer`) est distincte d'une éventuelle Institution descriptive. Elle possède identifiant d'instance, URL d'API allowlistée, clé publique, rotation/révocation des clés, état de confiance, capacités annoncées, règles de partage et journal de synchronisation. L'API est versionnée, signée, protégée contre replay, idempotente, bornée en taille/MIME et résistante aux contenus hostiles, SSRF et DNS rebinding. Inbox/outbox, quarantaine, conflits, tombstones et retries rendent les pannes partielles explicables.

Partage explicite : badges, formations, modèles de machines et membres consentants, par destination, catégorie et durée. Ne jamais synchroniser mots de passe/hashes, secrets MFA/recovery, assertions ou tokens SSO, credentials LDAP, RFID, tokens machines, rôles/groupes/Admin/packages locaux, thèmes/features, arrêts/sécurité machine, bans/suspensions, réservations détaillées, présences/logs d'accès, notes privées, données financières ou champs non consentis. Aucun rapprochement silencieux par e-mail : une identité externe est `(instanceId, subjectId)` et sa liaison locale demande une confirmation explicite.

Un utilisateur présent dans plusieurs FabOS possède des droits, quotas et audits locaux différents. Un badge ou catalogue peut circuler avec provenance, version, signature, expiration/révocation et tombstone ; une réservation ou un événement reste administré par son instance propriétaire. En cas de peer indisponible, la sync attend en outbox/quarantaine et ne devient jamais une autorisation fail-open.

Une attribution de badge conserve UUID global, utilisateur, définition versionnée/archivable, instance émettrice, preuve, date, formations sources, mode d'obtention, signature/provenance, expiration et éventuelle révocation. La simple relation many-to-many Badge–Formation ne suffit pas : une règle de qualification versionnée exprime alternatives, conjonctions, seuils et version de programme. Les règles « A + B offre C » créent une nouvelle attribution idempotente ; elles ne retirent rien. La suppression ou pseudonymisation d'un utilisateur suit une politique RGPD de rétention sans réécrire l'historique de l'émetteur.

Une machine locale référence une marque et un modèle partageables, mais conserve nom local, sous-lieu, photo locale, statut, token, règles de sécurité et overrides. Une synchronisation ne modifie jamais silencieusement la sécurité locale.

## Plan de livraison et responsables

Chaque session est migrable, testée, déployée sur Artemis et vérifiée indépendamment par **Sol**. **Terra** porte le domaine, les migrations et services ; **Luna** porte les contrats UX, composants et validation visuelle.

| Session | Livraison | Principal | Vérification Sol |
|---|---|---|---|
| S102 | consigner décisions, contradictions, nouvelle roadmap ; marquer S100–S101 obsolètes sans changer le live | Terra + Luna | cohérence docs/code |
| S103 | registre central Feature Workspace v2, contrat Thèmes et nouvelle maquette Développement, sans enforcement | Terra + Luna | matrice feature/route/scope/capacité + inventaire identité/menu/accueil |
| S104 | réparer la fondation quotas : comptes par type, hard constraints avant passes, tests et grandfathering | Terra | verdicts de régression et politiques complètes |
| S105 | geler les portails et produire le rapport de consolidation, sans suppression | Terra | collisions, canonical 301, sauvegarde et rollback |
| S106 | créer le sous-lieu par défaut, identité physique et horaires ; rendu inchangé | Terra, UI Luna | migration/backfill/rollback |
| S107 | rattacher machines, espaces, événements sur site, prêts, lecteurs et futures sessions | Terra | aucune ligne orpheline avant contraintes |
| S108 | contexte sous-lieu, préférence profil, URL et composant central | Terra + Luna | autorisation ≠ préférence ; mobile/clavier |
| S109 | groupes, migration des rôles, audiences système, protection dernier Admin | Terra | concurrence/lockout et parité Staff/Trainer |
| S110 | grants Use/Report/Manage atomiques et scopes en simulation | Terra | couverture routes, anti-escalade, différences |
| S111 | packages v2, attributions individu/groupe et restrictions par sous-lieu | Terra + Luna | union/restriction/temps et migration S97–S99 |
| S112 | shell central listes/filtres/facettes, maquette réelle | Luna + Terra | URL, requêtes, a11y, sombre, cinq langues |
| S113 | workspace pilote Équipement hors Quotas/Reporting, ajoutés seulement en S118–S119 | Terra + Luna | sécurité, aucune duplication ni onglet vide |
| S114 | workspaces Événements et Prêts | Terra + Luna | inscriptions/loans via adaptateurs réels |
| S115 | workspace Espaces | Terra + Luna | scopes lieu/catégorie/responsable/département |
| S116 | workspace Formations/Badges ; archivage/version, retrait delete/cascade, attribution append-only locale | Terra + Luna | sessions, règles qualification, historique préservé |
| S117 | Galerie, Pages personnalisées, Utilisateurs, Lieux, Packages, Réseau, Configuration et éditeur Thèmes | Terra + Luna | aucune feature/route oubliée ; preview/publish/rollback |
| S118 | politiques réservations/annulations/quotas par feature | Terra | hard constraints séparées des quotas souples |
| S119 | socle Reporting + premier adaptateur ; capacités analytics atomiques | Terra + Luna | lecture/export scoped sans fuite |
| S120 | retirer visuellement Réservations globale après parité et redirections | Terra + Luna | anciens liens et historiques préservés |
| S121 | fédération d'authentification : ProviderRegistry, OIDC d'abord, liens `(issuer, subject)` et provisioning local | Terra + Luna | identité, pannes, rotation, revoke-all, aucun claim Admin |
| S122 | profil public opt-in, slug, confidentialité par champ et inventaire des expositions existantes | Terra + Luna | vie privée, indexation, annuaires/API/kiosk |
| S123 | identité d'instance, confiance et API FabOS versionnée | Terra | crypto, rotation, SSRF, erreurs partielles |
| S124 | import QR inter-FabOS ponctuel signé et consenti | Terra + Luna | consommation atomique, replay, provenance |
| S125 | badges/formations fédérés append-only et règles dérivées | Terra + Luna | révocation sans effacement, doublons, provenance |
| S126 | marques/modèles machines fédérés et overrides locaux | Terra + Luna | aucune donnée locale/sécurité écrasée |
| S127 | retirer techniquement les portails après un cycle complet, audit nul et sauvegarde | Terra | inventaire consommateurs vide, restauration testée |
| S128 | audit transversal final de toutes listes/workspaces | Luna | Sol valide permissions, filtres, mobile, sombre, i18n |

Ordre obligatoire : S104 avant tout nouveau moteur de droits ; S106–S108 avant tout scope de sous-lieu ; S109 avant attribution de groupe ; S110 avant enforcement ; S112 avant les workspaces pour éviter les copies ; S118 avant retrait de la vue Réservations ; S121 rend la connexion multi-instance fluide ; S122 précède le QR ; S123 authentifie toute donnée inter-FabOS avant S124–S126 ; S127 arrive après un cycle complet sans dépendance Portal.

Les workspaces S113–S117 livrent leurs listes et actions existantes sans afficher de faux onglet. Quotas avancés arrive en S118 et Reporting en S119 ; ces sessions branchent ensuite leurs onglets sur chaque workspace déjà migré.

S104 corrige explicitement deux défauts live avant les packages v2 : les compteurs active/daily/weekly mélangent aujourd'hui plusieurs `ReservableType`, et un AccessPass sort avant les contrôles d'alignement/buffer alors qu'il ne devrait lever que les quotas souples. Les tests couvrent les verdicts resserrés et desserrés ; les réservations déjà créées restent grandfathered.

### Grille de vérification Sol

- **S103 :** inventaire feature × route GET/POST/API/worker × gate × scope × capacité ; test de couverture route mutante → voter/service.
- **S104–S108 :** règles de collision et sauvegarde Portal ; backfill Sous-lieu à 100 %, zéro orphelin ; timezone/DST ; URL hors scope refusée ; préférence sans effet sur l'autorisation ; pagination/onglets/mobile/clavier/i18n.
- **S109–S111 :** matrice sujet direct/groupe/Guest/Admin × feature × section × capacité atomique × scope × temps ; dernier Admin protégé sous concurrence ; restriction d'attribution sans élargissement ; CSRF, IDOR, mass assignment ; shadow log sans différence inexpliquée.
- **S112–S120 :** une implémentation commune par fonction ; total global égal à l'union des vues feature ; anciens liens redirigés ; aucune fuite reporting/export ; N+1, états vides, sombre, mobile et cinq langues.
- **S121 :** même e-mail/deux subjects restent deux comptes ; changement d'e-mail conserve le lien ; liaison concurrente atomique ; issuer/audience/nonce/signature/horloge invalides refusés ; JIT, suspension, panne IdP, rotation JWKS, logout et recovery hors IdP testés.
- **S122–S124 :** privé par défaut après inventaire des surfaces publiques ; identité source authentifiée avant QR ; deux consommations concurrentes donnent exactement un succès ; GET non consommant ; expiré/révoqué/replay refusés ; secret absent des logs/referrers.
- **S123–S126 :** trust explicite, rotation/revocation clés, replay/idempotence/out-of-order/tombstones, quarantaine, SSRF, taille/MIME/HTML hostile, consentement révocable ; aucune fusion par e-mail ; un badge révoqué ne qualifie plus ; aucune sécurité machine locale écrasée.
- **S127–S128 :** recherche code/SQL/Twig/config/workers sans consommateur Portal ; sauvegarde/restauration testée ; parcours E2E de chaque persona et scope ; chaque verdict explicable dans l'audit ; mode Développement désactivé avant production.

## Questions opérateur encore nécessaires

1. Admin recovery respecte-t-il badges/formation et arrêts de sécurité machine ? Recommandation : **oui**, il contourne packages/quotas mais pas la sécurité physique.
2. Manage implique-t-il Report sur le même scope ? Recommandation : **oui**, jamais Use.
3. Confirme-t-on que Guest est l'audience anonyme et non un groupe de comptes ?
4. Le groupe Formateurs est-il seedé en plus, ou fusionné avec Staff ?
5. Une institution non équipée de FabOS peut-elle rester un émetteur descriptif, ou Institution signifie-t-il exclusivement « autre FabOS » ?
6. Une attribution de badge erronée reste-t-elle visible comme révoquée, puisque l'effacement est interdit ?
7. Les formations sont-elles un catalogue global avec sessions localisées ? Recommandation : **oui**.
8. Les matériaux sont-ils un catalogue global avec stocks par sous-lieu ? Recommandation : **oui**.
9. Confirmer l'interprétation mise à jour : l'ancien groupe « Le lieu » devient « Utilisateurs » ; Horaires passe sous « Lieux » ; Interface/contenu d'accueil passe sous « Configuration → Thèmes ».
10. Pour l'import membre, quels champs généraux sont proposés par défaut : nom, prénom, e-mail, avatar, bio, langue ?
11. `guestsAllowed` reste-t-il l'autorité des événements publics ou doit-il être migré vers un grant Guest ?
12. Les annuaires Équipe/Formateurs, leaderboard/API, kiosk, historiques et galerie suivent-ils le consentement du profil public ou une règle séparée ?
13. Si plusieurs chemins package + politique passent, quel chemin est affiché comme gagnant ? Recommandation : le plus spécifique, puis l'identifiant stable, sans modifier le verdict.
14. Après désactivation au fournisseur d'identité ou panne IdP, combien de temps une session FabOS déjà ouverte reste-t-elle valide ?

## Hors scope maintenu

RFID et cartes physiques restent différés. Facturation, crédits et 2FA restent hors produit tant qu'une nouvelle décision ne les réintroduit pas. La réservation par pool de machines n'est pas impliquée par les catégories ou scopes. Les calendriers, quotas, tickets/check-in, waitlists et réservations distribués entre plusieurs FabOS restent hors scope : l'instance propriétaire demeure la seule autorité et les autres redirigent vers elle.
