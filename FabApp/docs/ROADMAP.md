# FabOS — roadmap active

**Mise à jour : 2026-08-16.** ⚠️ **Cette page ne contient que le travail
restant.** Tout ce qui est livré en est sorti : les récits sont dans
`HISTORY.md`, et son **index en fin de fichier** donne une ligne par session de
S102 à S142. Une session livrée qui reste ici finit par être refaite.

Livré à ce jour : phases A à F (S102–S128), **toute la Phase G sauf S134b**
(S129–S134), S134c, S134g, toute l'interface S134h–S141, puis S138c, S142 et S143.
⚠️ **Les droits d'usage sont APPLIQUÉS et les quatre chokepoints sont sur grants
v2.** Le package legacy n'est pas retiré et deux tables doublonnées attendent une
migration de contract — lire « L'état vivant du modèle de droits » ci-dessous
avant d'y toucher.

## Cap produit

Tout fablab, école ou atelier partagé déploie **les seules fonctions dont il a
besoin**, avec une expérience cohérente.

- une installation, plusieurs **lieux** physiques ; aucun portail ;
- SSO entre instances sans partager droits ni données ;
- sept audiences intégrées protégées + groupes locaux + packages assignables ;
- deux droits, **Use et Manage**, par feature/lieu/scope ; le reporting est
  dans Manage ;
- réservations, quotas et reporting montrés dans chaque feature, moteurs
  communs ;
- profils publics volontaires, échanges inter-FabOS consentis, badges
  cumulatifs et fédérables ;
- plus tard : Paiements facultatif, puis messagerie Formation ;
- **Configuration → Thèmes** réunit identité visuelle, images, menus et accueil ;
- **un seul** système central de listes, filtres, workspaces, composants et CSS.

Modèle cible détaillé : [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage).

## Frontière d'une instance

**Même FabOS** = gouvernance, admins, annuaire, politiques, moteur de réservation
et source de vérité partagés → lieux. **Autre FabOS** dès qu'un service veut
son propre admin, thème, catalogue ou cycle de vie ; la distance ne décide pas.
Le SSO évite un mot de passe, il ne partage rien : le métier passe par le Réseau,
objet par objet, avec provenance. Chaque ressource a **une** instance
autoritaire ; ailleurs, projection lecture seule + lien. **Une réservation
distribuée n'est jamais implicite dans une synchronisation.**

## Règles de construction

1. **Une source de métadonnées.** `FeatureWorkspaceRegistry` décrit la
   navigation et les scopes ; voters et services restent l'autorité métier.
2. **Une liste, un shell.** `_admin_list` + `_data_table` + `_cell_*` ; la page
   n'apporte que ses colonnes et ses données.
3. **Pas de surcharge.** Lieu, tâche et filtre sont trois axes différents.
   Six filtres rapides maximum, le reste dans `Plus de filtres` et en chips.
4. **URL explicable.** Défaut agrégé ; lieu, recherche, filtres et
   pagination partageables ; une préférence de profil n'est jamais un droit.
5. **Sécurité côté serveur.** L'UI lit le même verdict que les voters. Un lien
   caché n'autorise ni n'interdit rien.
6. **Migrations expand/backfill/contract.** Jamais `schema:update --force`.
7. **Simulation avant activation.** Chaque route mutante a son voter atomique.
8. **Cinq langues, sombre, mobile, clavier.** Toute primitive est montrée avec
   le vrai composant dans `/admin/design`.
9. **Artemis est la définition de done.** Doc, commit, deploy CT210, lint,
   restart, vérification réelle. Jamais `deploy.sh`.

## Décisions opérateur fixées

- **Guest** = anonyme sans compte. Visibilité ≠ action. Le réglage FabOS est un
  défaut, pas un plafond : par événement, `inherit`/`allow`/`deny`, séparément
  pour `view` et `register`.
- **Groupes intégrés protégés** : clés stables, non supprimables ; libellés et
  attributions configurables. User = tout compte actif, Guest = virtuel.
- **Institution** : descriptive, ou connectée après découverte sur une URL HTTPS
  et **confirmation explicite** de confiance. Changer d'origin la suspend.
- **Partage** : le personnel sort seulement si l'instance l'autorise **et** que
  le membre consent à cette donnée et cette destination.
- **Un badge n'est jamais effacé** : définition archivable, retrait = révocation
  auditée qui reste au journal.
- **Import QR** automatique après un consentement récapitulatif unique. Un badge
  révoqué n'est jamais réactivé ; aucun conflit local écrasé en silence.
- **Packages cumulatifs, fermés par défaut.** Aucun package ne retire un droit.
  Aucune fusion de champs entre politiques.
- **Matériaux** = catalogue partageable ; disponibilités et emplacements locaux.
  Sous Équipement par navigation, pas par confusion catalogue/stock.
- **Navigation** : « Le lieu » → « Utilisateurs » ; Horaires sous Lieux ;
  interface et accueil sous Configuration → Thèmes.
- **Le titre d'une liste = son entrée de menu** (`admin_nav.entry.<route>`), lu
  depuis `NavBuilder`, jamais recopié. « Quotas », pas « Gestion des quotas ».
  ⚠️ Une exception validée à l'écran : **`/admin/machines` porte sa SECTION**.
- **Admin recovery n'est pas un bypass** : ni badge requis ni arrêt de sécurité.
- **Deux droits.** Pas de Report : consultation/export sont dans Manage, et
  Manage ne confère **jamais** Use.
- **Formation = catalogue global** ; seules les sessions ont un lieu.
- **Exposition publique par surface** : activation opérateur **et** consentement.
- **Sessions IdP** : révocation immédiate si connue, 24 h max en cas de panne.

## Répartition

**Terra** domaine, migrations, services, voters, protocoles · **Luna**
architecture d'information, composants, listes, a11y · **Sol** revue
indépendante — et Sol ne valide jamais sa propre implémentation.

---

# Ce qui reste

## Phase G — bloquante avant le commerce

Ne commencer ni paiement, ni catalogue d'offres, ni ledger tant que S134b n'a
pas validé le cleanup transversal.

| Session | Attendu | Qui |
|---|---|---|
| ✅ **S132** | shell/design system : Logs RFID, Réglages, E-mails et Fonctionnalités refaits ; quatre `<style>` locaux supprimés | Luna + Terra |
| ✅ **S133** | parité : CRUD des catégories machine, « Ailleurs / externe », fiche canonique d'un objet, Packages réduit à ce qu'il expose | Terra + Luna |
| ✅ **S133b** | groupes, audiences, grants v2 Use/Manage — **en ombre**, visible et explicable | Terra + Luna |
| ✅ **S134** | mécanisme d'activation graduelle + garde-fou ; **les quatre chokepoints sont basculés**, l'éditeur de grants et l'application du lieu sont livrés | Terra + Sol |
| **S134b ⬅️** | cleanup final : dette, routes orphelines, traductions, a11y, **et l'inventaire action-opérateur** — chaque objet annoncé est créable, éditable, archivable depuis son workspace | Luna + Terra, Sol valide |

✅ **Le critère de sortie de la Phase G est atteint** (vérifié 2026-08-16) : le
**champ lieu existe sur les huit formulaires** machine/espace/événement/objet
via `VenueChoiceType`.

### 🔴 L'état vivant du modèle de droits — à lire avant d'y toucher

**L'enforcement est ON et les quatre chokepoints sont sur grants v2**
(`usage_rights_v2_machines|places|person_booking|events`). L'écran d'ombre lit
**0 perdraient / 0 gagneraient / 12 identiques / 24 recovery admin** sur 9
membres : v1 et v2 sont d'accord partout.

⚠️ **Le package legacy n'est pas retiré** et les deux tables doublonnées
(`USAGE_GRANT`, `USAGE_PACKAGE_GROUP_ASSIGNMENT`) sont toujours là, non lues.
**La prochaine étape sûre est une migration de contract** qui les supprime, une
fois le modèle convergé observé quelque temps.

🔴 **`USAGE_PACKAGE_GRANT` est LA table de grants** (S111,
`Version20260809150000`). S133b en a créé une seconde, `USAGE_GRANT`, faute
d'avoir grepé `migrations/`. Convergé en S134b. ⚠️ Sa colonne est `sectionKey`,
pas `section`.

🔴 **Un grant limité à un lieu n'a rien fait pendant deux sessions** : `verdict()`
n'avait pas de paramètre de lieu, tous les appelants passaient `null`, et la
branche `:venue IS NULL` matchait tout. Corrigé en S134b ; `VenueScopedGrantTest`
épingle le câblage. ⚠️ `null` reste **permissif** par décision.

**L'éditeur de grants existe** : `/admin/usage-rights/{id}/edit`, section
« Grants v2 ». ✅ **Le formulaire d'attribution à un GROUPE existe depuis S144a.**

### 🔴 S144 — le système de packages, fini (2026-08-17)

Demande opérateur, mot pour mot : *« finish the all package system. One package
must be able to allow users to 3D print on monday afternoon for exemple. Another
one must allow X hours machine reservations per week. »*

⚠️ **DEUX MIGRATIONS À LANCER** (aucune n'a encore tourné) :
`Version20260817100000` (portée + `USAGE_GRANT_WINDOW`) et
`Version20260817110000` (`USAGE_PACKAGE_ALLOWANCE`). Les deux sont additives et
ne suppriment rien. **Le code peut vivre sans elles** : `UsageGrantSchema` et
`UsageAllowanceRepository::tableExists()` sondent avant de nommer une colonne, et
échouent vers l'ANCIEN comportement. Tant qu'elles n'ont pas tourné, les
nouvelles dimensions n'existent simplement pas.

Ce qu'un package sait dire maintenant :

| Dimension | Où | Depuis |
|---|---|---|
| fonctionnalité, action (use/manage), section | `USAGE_PACKAGE_GRANT` | S111/S133b |
| lieu | `USAGE_PACKAGE_GRANT.venueId` | S134b |
| **ressource** : un type, une machine précise, une catégorie | `reservableType` / `reservableId` / `categoryLabel` | **S144b** |
| **créneau hebdomadaire** : « lundi 14:00–18:00 » | `USAGE_GRANT_WINDOW` | **S144b** |
| **combien** : X heures ou X réservations par jour/semaine/mois/total | `USAGE_PACKAGE_ALLOWANCE` | **S144c** |
| à qui : un membre **ou un groupe** | `USAGE_RIGHT_ASSIGNMENT.userId` / `.groupId` | S111 / **S144a** |

🔴 **Couverture, pas chevauchement.** « Lundi 14:00–18:00 » REFUSE une
réservation de 17:00 à 22:00. Un test de chevauchement aurait vendu de l'accès
complet avec des étapes en plus. `GrantWindowSet` fait l'union des créneaux d'un
jour puis marche la réservation jour par jour ; minuit **en fin** de journée vaut
1440 et non 0.

⚠️ **Les créneaux ne sont évalués que si l'appelant donne un intervalle.** Une
réservation en donne un ; un aperçu (« ce membre a-t-il ce droit ? ») n'en donne
pas et n'est donc jamais refusé « parce qu'on est mardi ». La page des droits
montre les créneaux en toutes lettres à la place.

⚠️ **Aucune allocation = aucun plafond**, jamais un plafond de zéro. Les
allocations de plusieurs packages **s'additionnent** (5 h + 10 h = 15 h) mais
5 h/semaine et 20 h/mois sont deux budgets qui doivent tenir tous les deux.
**Un pass d'accès ne lève PAS une allocation** : il exempte des quotas du labo,
pas des heures achetées.

⚠️ **Ajouter une allocation est la seule écriture de cet écran qui RESTREINT**, et
en retirer une rend des heures. Les deux confirmations le disent dans ce sens.

**Ce que S144 n'a PAS fait, volontairement :**
- Aucun paiement, aucun prix, aucune commande — c'est la Phase H (S150–S154), et
  elle reste facultative. S144 livre l'**entitlement** qui rend un package
  vendable, pas le commerce.
- Pas de `categoryLabel` sur les allocations, alors que les grants en ont un :
  rien ne l'appliquerait, et une colonne que seul un formulaire écrit est
  exactement la faute que S144a a réparée sur `groupId`.
- `readiness()` (préflight de `/admin/settings`) compte toujours
  `COUNT(DISTINCT a.userId)` : un package tenu **uniquement** par un groupe y
  compte pour 0 personne. Voir S144e ci-dessous.

### S144e — « ce package touche N personnes » (à faire)

`readiness()` et la liste des packages comptent les attributions **directes**.
Depuis S144a un package peut n'être tenu que par un groupe, et l'écran annonce
alors 0. Le compte honnête demande un `AudienceResolver::memberIdsFor($groupKey)`
qui soit l'inverse exact de `keysFor()` : rôles (`ROLE_STAFF`…) **plus** lignes
`USER_GROUP_MEMBER` **plus** l'audience virtuelle `user` = tous les comptes
actifs. ⚠️ Ne pas réécrire cette logique à côté — c'est ainsi que deux réponses
divergent.

### ✅ S132 — livré

- **Fonctionnalités** ne décrit plus l'effet d'une désactivation, elle le
  **mesure** : `SiteFeatureService::simulate()` rejoue l'état des fonctionnalités
  le temps d'un appel, `FeatureSurfaces` construit la navigation deux fois et la
  différence est la réponse. Ordre = ordre des workspaces ; les trois sections
  Ressources/Activités/Annuaires sont supprimées. ⚠️ **Ne pas réintroduire une
  description en prose de ces effets** — la précédente avait déjà tort.
- **Réglages** : cinq cartes ancrées, résumé d'état, sauvegarde par section.
  🔴 Ce n'était pas de la mise en forme : toutes les écritures étaient à
  l'intérieur du contrôle de la langue, donc un POST avec une langue non reconnue
  jetait en silence le fuseau, le vocabulaire, le règlement, les rôles et le
  bandeau.
- **E-mails** : trois tâches, et le journal filtre **côté serveur**.
- **Logs RFID** : filtres serveur (période/lecteur/machine/résultat) et le détail
  à la demande dans un `<details>`, cinquième colonne.
- ✅ **`/admin/rfid-readers` n'a plus aucun CSS local.** Sa dernière règle,
  `.token`, ne stylait plus rien depuis S135. Les primitives extraites sont dans
  `/admin/design#reglages`.
- ⚠️ `.color-dot` est monté dans `components.css` : `_rfid_result` est un partial
  **partagé** et ses deux appelants en portaient chacun une copie.

### ✅ S133 — livré

- **Catégories de machines** est un vrai CRUD. `MACHINE_CATEGORY` en expand pur :
  `MACHINE.categoryLabel` ne bouge pas et reste la clé de jointure. Renommer vers
  un nom existant est une **fusion** et le dit ; archiver ne touche aucune machine
  et le dit aussi. ⚠️ Le champ du formulaire machine reste **libre** avec un
  `datalist` : un `ChoiceType` rendrait une machine insauvable le jour où sa
  catégorie est archivée.
- **Événements** : `VenueContext::forRequest()` a une troisième réponse en opt-in,
  « Ailleurs / externe ». ⚠️ `['venue' => null]` est un critère Doctrine réel et
  n'est pas `[]`.
- **Prêts** : `/prets/{id}` est la fiche canonique — chaque carte du catalogue
  pointait sur le catalogue. Et l'objet **s'archive** : `remove()` emportait ses
  prêts avec lui.
- **Packages** annonce désormais ce qu'il expose. Quotas vit déjà par type de
  ressource ; Attributions et Audit arrivent avec grants v2.

### Thèmes — le chantier entier (session dédiée, avant ou dans S134b)

- **Médiathèque d'identité** au lieu du champ texte `logoPath` : logo
  clair/sombre/compact, favicon, image de partage. Validés (MIME, dimensions,
  taille), renommés serveur, référencés par ID stable, supprimables seulement
  après contrôle des références. **Aucun chemin `public/images/…` libre.**
- **Éditeur guidé** : identité, variantes de logo, palette avec contrastes,
  rayon/typo/densité en presets. Pas de réglages épars.
- **Workflow** brouillon → aperçu → publication → retour arrière. L'aperçu rend
  de **vraies** surfaces (accueil, catalogue, détail, admin, **un kiosk**),
  desktop/mobile, clair/sombre. Publication atomique réglages **et** assets.
- **Kiosks** consomment le thème publié. Aucun `images/favicon.png`, logo ou
  couleur statique ne survit dans un kiosk.
- **Navigation & accueil** : ordre et visibilité par drag-and-drop accessible,
  destinations limitées aux routes/pages publiées autorisées, entrées système
  protégées. Page d'accueil au choix ; une page dépubliée rétablit l'accueil
  FabOS avec audit, sans page blanche ni boucle.

## Phase G2 — le produit honnête

Ajoutée le 2026-08-11 : on ne peut pas vendre du temps machine contre un modèle
d'horaires incapable d'exprimer un jour férié. Ces manques concernent **toutes**
les installations, tous les jours ; le commerce est facultatif.

| Session | Attendu | Qui |
|---|---|---|
| **S134c2** | **FabOS cesse d'inventer le contenu d'une formation** — fiche ci-dessous | Luna + opérateur |
| **S134d** | **une seule vérité horaire (modèle)** : un `ScheduleResolver` répond « X est-il ouvert à T » pour l'admin, les deux calendriers, les cartes, les kiosks et l'API ; plusieurs plages par jour, portée attachable, exceptions datées — **livrés ensemble** | Terra |
| **S134e** | **une seule vérité horaire (surfaces)** : Lieux édite plages, portées et exceptions avec aperçu ; public, calendriers et kiosks affichent la fermeture **avec sa raison** | Luna + Terra |
| **S134f** | archiver plutôt que supprimer partout où une suppression dure subsiste (événement, espace, matériau, objet, institution, page, création, lecteur) ; tout archivage d'une ressource réservable annule explicitement ses réservations à venir | Terra + Luna |

### Horaires — ce que le modèle ne peut pas exprimer (constat S131)

`UNIQ_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek)` existe depuis S106 et l'écran
sait enfin l'utiliser. Trois limites restent, par ordre de coût :

1. **Une seule plage par jour et par lieu.** Fermeture méridienne, service
   du soir, créneau personnel : inexprimables. ⚠️ **Contract au sens migration** :
   supprimer l'unicité oblige à reprendre **avant** tout code qui suppose « une
   ligne = un jour » (`ensureOpeningHourRows`, `OpeningHoursProvider`, les deux
   calendriers).
2. **Aucune granularité sous le lieu.** Cible : un horaire **rattachable** —
   lieu par défaut, surchargeable par workspace puis par ressource — avec
   héritage et une seule réponse effective par instant. ⚠️ **Ne pas dupliquer la
   table par type de ressource** : c'est la même question à des portées
   différentes.
3. **Aucune exception datée.** Jours fériés, fermeture annuelle. Se livre avec
   la portée ou la réécrit.

⚠️ **Prérequis commun** : la résolution effective doit être **un** service que
lisent l'admin, les deux calendriers, les cartes (« labo fermé » avant
« occupée », cf. S59) et les kiosks. Aujourd'hui chacun interroge la table ;
tant que ce service n'existe pas, ajouter une portée multiplie les endroits qui
peuvent se contredire.

### S134c2 — FabOS invente le contenu d'une formation

Quand les champs sont vides, la page publique sert un **programme horaire en
quatre points**, **trois sessions à venir** (« Mardi prochain 14:00-16:30 ·
Places disponibles »), **trois objectifs**, **deux prérequis** et **trois
éléments de matériel**. Rien n'existe : ce sont des exemples de démonstration
servis à un membre comme s'ils décrivaient sa formation — créneaux auxquels il ne
peut pas s'inscrire compris.

**Règle (opérateur) : on traduit l'interface, jamais le contenu. Un contenu que
l'opérateur n'a pas écrit ne doit pas exister.** Un champ vide n'affiche pas de
bloc ; l'écran d'ÉDITION peut proposer ces exemples comme point de départ, la
page publique jamais comme un fait.

⚠️ Les titres de section et les cartes d'explication sont l'interface, sont des
clés depuis S134c, et **restent**. Seules les **valeurs** sont en cause. Fichiers:
`FormationPageContentService::DEFAULTS` (blocs `program`, `sessions`) et les trois
`{% set %}` d'`objectives`/`prerequisites`/`material` dans
`formation-detail.html.twig`.

## Petits restes datés

- **Le tableau de bord a perdu son caractère (opérateur, 2026-08-16).** Ses mots :
  *« in a way the old homepage looked more "special", we'll find a way to make it
  pop again later »*. S143b a eu raison de supprimer le bandeau magenta — il
  était la cinquième copie du même slab, il portait vingt couleurs littérales et
  un faux avatar « AD » — mais **la cohérence a coûté la singularité** : la page
  d'accueil de l'admin ressemble maintenant à n'importe quelle liste. ⚠️ **Ce
  n'est pas une régression à annuler, c'est un travail à faire** : rendre le
  tableau de bord distinctif *sans* réintroduire un bandeau pleine largeur ni une
  couleur en dur. Pistes non tranchées : une bande d'accueil qui reste dans la
  carte mais respire plus (hauteur, salutation, heure/état de l'instance) ; les
  sept chiffres traités comme la figure de la page plutôt que comme sept tuiles
  égales ; une seule surface accentuée réservée à cet écran. **À montrer en
  propositions comparables dans `/admin/design` avant de construire** — c'est le
  protocole qui a marché pour le format de liste (quatre tours, S130e).

- **`/events` sans paramètre rend 0 carte** quand il n'y a aucun événement à
  venir, alors que « Tous » en compte 3. Le défaut « À venir » est délibéré ; ce
  qui manque est un état vide qui renvoie vers les événements passés.
- **`/admin/homepage` porte six colonnes** (bloc + quatre audiences + ordre).
  C'est une matrice d'audiences, pas une liste d'enregistrements : le plafond de
  cinq ne lui répond peut-être pas. Non tranché.
- ✅ **« Sous-lieu » est devenu « lieu » — *location* en anglais (S143).**
  Renommage de catalogue fait dans les cinq langues : FR lieu, EN location, DE
  Standort, ES ubicación, IT sede. `Venue`/`VENUE`, `venue_context`, `?location=`
  et les noms de routes n'ont pas bougé — c'est un renommage de mots, pas de
  schéma. ✅ **Collision tranchée : on garde** (opérateur, 2026-08-16 —
  *« laissons comme cela pour l'instant, je n'ai pas mieux »*). La section de
  menu s'appelait déjà « Lieux », donc le mot paraît quatre fois sur
  `/admin/lieux` : barre latérale, libellé du sous-menu, un de ses deux liens, et
  le titre de la carte. C'est la forme `/admin/machines` — une section dont la
  page d'atterrissage porte son nom — avec le même mot des deux côtés. ⚠️ **Ne
  pas « corriger » ça dans une session future** sans le redemander : les trois
  sorties (laisser, renommer la section, distinguer l'entrée) ont été posées et
  aucune n'était meilleure.

## Phase H — commerce facultatif (S150–S154)

🔴 Renumérotée depuis S135–S139 le 2026-08-16 : ces numéros avaient déjà été
livrés comme sessions d'interface. Prochain numéro libre après S141 : **S142**.

Entièrement désactivable. Offres dans leur workspace métier, moteur commun pour
commandes/paiements/rapprochement. **Le retour navigateur ne confirme jamais un
paiement** — seul un webhook vérifié ou sa réconciliation. Clé unique par
événement fournisseur ; fulfillment persistant/outbox par ligne pour un effet
**exactement une fois** malgré retries et crashs. La livraison passe par le
service métier normal, sans toucher voter, badge, quota ni réservation.

| Session | Livre |
|---|---|
| **S150** | catalogue d'offres et prix ; aucune transaction |
| **S151** | commandes, paiements, webhooks, réconciliation, remboursements, audit |
| **S152** | livraison packages et matériaux ; hold stock atomique ou backorder explicite |
| **S153** | ledger append-only des crédits de temps et achats de formation |
| **S154** | reporting commerce, rapprochement, audit UX |

## Phase I — messagerie Formation (S155–S157)

Très loin après le workspace Formation. FabOS est la source de vérité ; l'e-mail
est une copie et une panne d'envoi ne perd jamais le message interne. Trois
visibilités : annonce formateur→cohorte sans exposer la liste, fil privé, groupe
explicite. Une réponse à une annonce est privée par défaut ; **aucun message
privé ne bascule implicitement vers la cohorte.**

| Session | Livre |
|---|---|
| **S155** | conversations, participants, non-lus, permissions |
| **S156** | interface formateur/étudiant + duplication e-mail asynchrone |
| **S157** | modération, archivage, export, rétention |

## Travaux transversaux conservés

Après les fondations dont ils dépendent : sécurité restante de Phase H
(**test réel du booking**, requêtes groupées) · verrou d'annulation et no-show
sur ressources qui ont un signal · files d'attente, stockage/retrait, motif
d'utilisation · exceptions d'horaires dans Lieux (voir S134d/e) · audit et notes
sur toute action Manage exercée sur autrui.

RFID physique et 2FA restent **hors scope**. Ordre : Phase G, puis G2, puis
commerce, puis messagerie. La réservation d'un pool de machines n'est pas
impliquée par les catégories.
