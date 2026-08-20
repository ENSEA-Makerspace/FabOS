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

✅ **Les deux migrations sont passées le 2026-08-19** (`Version20260817100000`,
`Version20260817110000`). ⚠️ **Le repli reste dans le code et doit y rester** :
`UsageGrantSchema` et `UsageAllowanceRepository::tableExists()` sondent avant de
nommer une colonne et échouent vers l'ANCIEN comportement. C'est ce qui a permis
de déployer le code AVANT les migrations sans retirer la réservation au labo —
vérifié à l'écran ce jour-là, pas supposé — et c'est ce qui protégera la
prochaine installation qui restaure une base plus vieille que son code.

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

### Ce qu'un package ne sait TOUJOURS pas dire — et ce que ça coûterait

L'opérateur a demandé de penser à *toutes* les façons dont un fablab voudrait
vendre un package. Voici l'inventaire complet, ce que S144 couvre et ce qu'il
laisse. **Rien ci-dessous n'est construit ; c'est une liste de choix, pas un
plan.**

🟡 **1. Assouplir un quota de palier — le manque le plus vendable.** « Premium :
réservez 30 jours à l'avance au lieu de 7 », « sessions de 8 h au lieu de 4 »,
« 5 réservations simultanées au lieu de 2 ». `BookingPolicy` porte déjà
`maxHorizonDays`, `maxDurationMinutes`, `maxActiveReservations` — mais **par
palier** (membre/formateur/équipe/admin), pas par package. Un labo qui veut
vendre l'un de ces trois doit aujourd'hui promouvoir la personne de palier, ce
qui change aussi ses droits d'administration. ⚠️ Le précédent existe :
`AccessPass` est déjà décrit comme « exemption de quota ». La forme serait un
package portant des **surcharges** de politique, appliquées vers le haut
seulement — jamais vers le bas, sinon un package retirerait un droit.

🟡 **2. Une validité relative à l'attribution.** « Trois mois à partir de
l'activation » n'existe pas : `validFrom`/`validUntil` sont deux dates absolues
saisies à la main. Vendre un abonnement demande aujourd'hui de calculer la date
de fin soi-même, à chaque attribution.

🟡 **3. Le report des heures non consommées.** Une allocation hebdomadaire non
utilisée est perdue le lundi. C'est un choix par défaut défendable, mais certains
labos vendent explicitement le report. Non exprimable.

🟡 **4. Une allocation par catégorie de machines.** Les grants savent dire
« les imprimantes 3D », les allocations non — volontairement, faute d'un
comptage qui l'honore (voir S144c).

⚪ **5. Priorité / préemption** (un package qui déloge une réservation) — pas
demandé, et contraire à « aucun package ne retire un droit ». À ne pas construire
sans une décision explicite.

⚪ **6. Prix, panier, paiement, remboursement, comptabilité** — Phase H
(S150–S154), facultative et non commencée. S144 livre l'**entitlement** qui rend
un package vendable ; le commerce reste un module séparé, et c'est le dessin.

⚪ **7. Matériaux et consommables inclus** — Phase H (S152).

⚪ **8. Formations incluses** — la certification est délibérément **hors** du
modèle de packages : c'est une question de sécurité, pas de commerce, et un
package ne doit jamais pouvoir en tenir lieu. Vendre « la formation laser » est
une commande de Phase H qui inscrit à une session ; elle ne fabrique pas un
badge.

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
| ✅ **S134c2** | **livré** — vérifié dans le code le 2026-08-19 (`FormationPageContentService.php:109` : « les valeurs que S134c2 a retirées de `DEFAULTS` »). Il était resté sur cette page ; c'est exactement ainsi qu'une session livrée finit par être refaite. | Luna + opérateur |
| ✅ **S145a** | **le lecteur d'horaires demande enfin OÙ.** `ScheduleResolver` remplace `OpeningHoursProvider` ; les douze appelants passent le lieu qu'ils connaissent. 🔴 Il résolvait la semaine via le lieu de slug `default` et `ReservationService` ne passait aucun lieu — une machine du second lieu était contrôlée contre les horaires du premier. Aucune migration. | Terra |
| ✅ **S134d** | **livré 2026-08-19** : plusieurs plages par jour (l'unicité `(venueId, dayOfWeek)` tombe), `SCHEDULE_EXCEPTION` pour les fermetures datées avec leur raison, et l'écran qui édite les deux. Couverture par UNE plage, jamais l'enveloppe. ✅ **La portée attachable est livrée** (2026-08-19) : `OPENING_HOUR.scopeType/scopeId`, trois niveaux — le lieu, un type de ressource, une ressource. 🔴 **Les niveaux s'INTERSECTENT** (décision opérateur) : une ressource ne peut que restreindre son lieu, jamais l'élargir, donc aucun niveau ne peut échouer en s'ouvrant. Un seul niveau répond, ils ne s'empilent pas. La colonne « Effectif » de `/admin/horaires` montre la résolution, parce que des heures plus larges que le lieu ne font rien. | Terra |
| ✅ **S134e** | **livré 2026-08-19** : la raison d'une fermeture atteint les catalogues, les deux calendriers et les kiosques. Côté JS une exception datée répond AVANT le jour de semaine, comme sur le serveur — c'est ce qui fait que le calendrier et la porte sont d'accord sur un jour férié. | Luna + Terra |
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

## S146 — chaque chose à sa page, et le calendrier redevient « ce qui se passe »

**Proposition opérateur (2026-08-20), revue et adaptée. Rien n'est construit.**
Ses mots : la fiche d'un objet devient le moyen de voir son calendrier et de le
réserver ; le calendrier principal devient le moyen de voir l'activité d'un
**espace** (ouvert/fermé, événements, activités) ; les sessions de formation sont
des « activités », donc un atelier déco d'Halloween pour dix personnes et un
cours hebdomadaire d'un mois se traitent pareil ; équipement, personnes et autres
réservations vivent dans leurs pages de feature.

⚠️ **C'est déjà la politique de la feuille de route**, pas une déviation : « les
réservations, quotas et reporting sont montrés **dans chaque feature**, moteurs
communs ». La proposition applique ce cap au calendrier, qui ne l'avait jamais
suivi.

### 🔴 Le constat qui change tout : les formations n'ont pas de sessions

`Formation` porte `duree` et `formateur` en **texte libre**, `placesTotales`, et
**aucune date**. Il n'existe ni entité session, ni table `FORMATION_SESSION`.
`Progression` enregistre l'achèvement d'un membre, pas une séance. C'est
d'ailleurs pourquoi S134c2 a dû **supprimer** le bloc « trois prochaines
sessions » de la page publique : il les inventait, faute d'avoir quoi que ce soit
de vrai à montrer.

`Event`, lui, a déjà exactement la forme d'une « activité » : titre, début, fin,
**lieu**, capacité, invités autorisés, annulation **avec motif**, affiche,
sur place / ailleurs — plus `EventRegistration`.

**Adaptation proposée : un événement peut être LA SÉANCE d'une formation.**
Un `Event.formation` nullable, et rien d'autre à inventer. Conséquences :

- un atelier d'Halloween = un événement sans formation ; une séance de découpe
  laser = un événement **avec** formation. Même surface, même code, même
  inscription, même calendrier — ce que l'opérateur demandait, sans second modèle ;
- la page d'une formation retrouve un bloc « prochaines séances » **vrai**, qui
  est le manque que S134c2 a laissé ouvert ;
- « seules les sessions ont un lieu » (décision déjà prise) tombe juste :
  `Event` a un `venue`, `Formation` n'en a pas.

🔴 **Ce qu'il ne faut PAS faire : fusionner les entités.** Un événement et une
formation n'ont pas la même *issue* — une séance participe à une qualification,
un atelier non. Fusionner tirerait la logique de badge dans les événements et les
règles d'accès invité dans les formations.
🔴 **Et assister ne qualifie pas.** La certification est une question de
sécurité : la présence à une séance ne doit jamais accorder un badge toute seule,
un formateur valide. Le lien événement→formation planifie et inscrit ; il ne
touche pas à la validation.

### Ce que je couperais pour que ça reste simple

- **Aucun moteur de récurrence.** « Un cours hebdomadaire d'un mois » = **quatre
  événements générés à la création** (« toutes les semaines, ×4 »), pas une règle
  évaluée à la lecture. Plus simple, et plus honnête : chaque séance se déplace
  ou s'annule individuellement, ce qu'une règle ne sait pas exprimer.
- **Pas de troisième mot dans le vocabulaire.** « Activité » à côté d'« événement »
  et de « formation » ferait trois mots pour deux idées. ⚠️ **Décision opérateur
  à prendre** : soit le calendrier dit « Au programme » sans nommer un type, soit
  « Événements » couvre les deux.
- **Le calendrier principal devient LECTURE SEULE.** S'il ne montre plus que
  l'espace et ce qui s'y passe, la réservation part avec les machines : le
  brouillon de réservation, `getVisibleMachines()` et la grille de ressources de
  `/calendrier` **se suppriment**. C'est du code en moins, pas en plus.
- **Ne pas perdre « qu'est-ce qui est libre maintenant ».** C'est le seul usage
  que le calendrier agrégé servait vraiment et que sa nouvelle mission ne couvre
  pas. ✅ Déjà servi ailleurs : `/machines` et `/places` affichent `freeNow`,
  `venueOpenNow` et depuis S134e le motif de fermeture. Rien à construire.

### Ajouts opérateur (2026-08-20), tous retenus

1. **Un seul calendrier, en code comme à l'écran.** Les calendriers de feature et
   le principal doivent partager code et apparence, selon le guide de design.
   🔴 **Mesuré, pas supposé** : `calendrier.html.twig` (664 l.) et
   `machine-calendrier.html.twig` (288 l.) portent **douze fonctions JS
   homonymes** — `timeToMinutes`, `formatDateKey`, `getWeekDays`, `isMinuteOpen`,
   `getSlotState`, `exceptionFor`, `buildTimeOptions`, `overlapsHour`,
   `reservationCardHtml`, `getOpeningHoursLabel`… Dans la seule session
   S134d/S134e, la **même** logique a été modifiée dans les deux fichiers
   **quatre fois** (plages, `isMinuteOpen`, `getSlotState`, exceptions datées).
   Ce n'est pas une dette esthétique : c'est le prochain endroit où les deux vont
   diverger en silence.
   ✅ Faisable en Stimulus : les deux étendent `base_public.html.twig`, qui émet
   `importmap('app')`.

2. **Vue semaine ET mois.** « Un enseignant qui planifie autour de la
   disponibilité d'une machine ou d'un espace » regarde des semaines, parfois des
   mois à l'avance. Les deux calendriers ne savent aujourd'hui afficher qu'**une
   semaine** (`getWeekDays()`). La vue mois est donc un vrai ajout, et elle
   appartient au composant partagé — sinon elle sera écrite deux fois.

3. **🔴 Le filtre « lieux » sur le calendrier principal.** Vérifié :
   `/calendrier` **n'a aucun `VenueContext`**. Les catalogues ont reçu le filtre
   en S137/S138 ; le calendrier ne l'a jamais eu. C'est, dans les mots de
   l'opérateur, « une des raisons d'avoir des lieux séparés au départ » — et sans
   lui, la nouvelle mission du calendrier (montrer l'activité d'un ESPACE) n'a
   pas de sens.

### Découpage proposé

⚠️ **Le composant partagé vient EN PREMIER.** Construire la fiche machine puis le
calendrier d'espace avant d'extraire le composant, c'est écrire deux fois le même
calendrier et le refactorer ensuite.

| Étape | Livre | Coût |
|---|---|---|
| ✅ **S146a** | **livré 2026-08-20.** Un seul composant : `assets/controllers/calendar_controller.js` + `_calendar.html.twig` + `_calendar_booking.html.twig`, alimentés par `App\Calendar\CalendarPayload`. Les douze fonctions homonymes ont disparu ; les deux gabarits de page passent de **952 lignes à 277** (664→146 et 288→131). ⚠️ Le total du code, lui, MONTE : le composant partagé fait 1 045 lignes de gabarit+JS et 217 de PHP, commentaires compris, et il contient une vue mois qui n'existait pas. Ce qui baisse, c'est le nombre d'endroits où une règle d'ouverture est écrite : deux, puis un. Vues **semaine et mois**, filtre **lieu** sur `/calendrier`. | moyen, surtout des suppressions |
| ✅ **S146b** | **livré 2026-08-21.** Le calendrier est un ONGLET de `/machines/{id}` ; `machine-calendrier.html.twig` est supprimé et `/machines/{id}/calendrier` renvoie un **301** vers `/machines/{id}#calendrier`. `/places/{id}` reçoit le même composant, à la place d'un formulaire date+heures saisi à l'aveugle et d'une liste d'horodatages. | faible (le composant existait) |
| ✅ **S146c** | **livré 2026-08-21.** `/calendrier` est l'activité d'un lieu et il est **en lecture seule** : la grille machines (liste à cocher + recherche + filtre statut) et le brouillon de réservation sont supprimés, ainsi que `app_place_reserve`. `booking: false` dans la charge utile est tout le mécanisme. | faible, surtout des suppressions |
| ✅ **S146d** | **complet 2026-08-21.** `Event.formation`, le bloc « prochaines séances » (vraies) sur la page formation, ET la génération de N séances à la création (`App\Calendar\EventSeries` : « toutes les semaines » / « une semaine sur deux », jusqu'à 12). Migration `Version20260820100000` passée. | moyen, **migration passée** |
| ✅ **S146e** | **livré 2026-08-21.** `App\Calendar\SessionEnrolment` : prendre une place à une séance crée une `Progression` *commencée* sur la formation. 🔴 `completed = false`, `score = 0`, aucun badge — la présence ne qualifie JAMAIS. | moyen |

⚠️ **Ordre** : a→b→c livrent le gain de clics sans toucher au modèle. d et e
n'ont aucune raison de commencer avant que le vocabulaire soit tranché.

### ✅ S146f — les catégories d'événement (livré 2026-08-20)

**Proposition opérateur, ses mots** : *« let's make events categories so people can
pick (ex: workshops, training sessions, openhouse, etc.) … It should solve the whole
dilemma no? »* — à propos du vocabulaire que S146d/e devaient trancher.

✅ **Ça règle la moitié du dilemme, et c'est la bonne moitié.** Le calendrier n'a plus
besoin d'un nom collectif : chaque carte dit son propre genre. Le troisième mot
(« activité ») n'est plus nécessaire.
🔴 **Mais une catégorie ne remplace PAS `Event.formation`.** Une catégorie est un
LIBELLÉ ; le lien est une CLÉ ÉTRANGÈRE. « Séance de formation » ne dit pas
LAQUELLE, donc elle ne peut ni lister les vraies séances d'une formation ni donner
un sens à une inscription. **L'opérateur a tranché : les deux** (2026-08-20).

**Ce qui existe** : `EVENT_CATEGORY` (libellé, slug, icône, position, archivage),
`EVENEMENT.categoryId`, un écran `/admin/evenements/categories` (créer, renommer,
réordonner, archiver, avec le nombre d'événements par catégorie), le champ dans le
formulaire d'événement, le menu « Affiner » sur `/events`, la ligne de genre sur la
carte et la catégorie sur la carte du calendrier.

🔴 **RÈGLE : aucun code ne branche sur une catégorie précise.** L'opérateur la
renomme quand il veut ; un `slug === 'formation'` ferait d'un libellé un enum que
personne n'a déclaré. `EventCategoryContractTest` échoue si quelqu'un l'écrit.
⚠️ Le libellé est un **contenu** : jamais traduit, comme une catégorie machine.
⚠️ Le **slug ne suit pas le renommage** — il est dans les liens de filtre partagés.
⚠️ **Archiver retire des SÉLECTEURS, jamais de l'affichage** : un événement qui porte
une catégorie archivée continue de la montrer.
⚠️ Un `?category=` inconnu montre **tout**, pas rien — une catégorie est un mot qu'on
renomme, alors qu'un `?location=` inconnu répond 400 parce qu'un lieu existe ou non.
⚠️ **Une installation sans aucune catégorie est identique au pixel près.**

🔴 **Le 500 que `php -l` n'a pas vu, encore.** `SiteController::events()` a reçu un
paramètre `EventCategoryRepository` sans son `use` : lint vert, `/events` en 500 au
premier appel (« type-hinted with the non-existent class `App\Controller\
EventCategoryRepository` »). Même famille que la méthode privée `run()` de S144 et le
renommage de paramètre de S145a. **Un lint vert ne dit rien du conteneur.**

**Vérifié** : 16/16 sur les chemins d'ÉCRITURE dans une transaction annulée (création,
doublon refusé, renommage qui ne bouge pas le slug, rattachement à un événement,
archivage/restauration, réordonnancement, séances d'une formation, filtre inconnu),
0 ligne survivante ; 118 tests / 2296 assertions ; 122 routes balayées ; les trois
pages rendues et lues à l'écran pendant que les données existaient.

### ✅ S146b — la fiche porte son calendrier (livré 2026-08-21)

**Machines** : « Calendrier » était un lien qui QUITTAIT la page ; c'est un onglet.
`machine-calendrier.html.twig` (131 lignes) est supprimé, et
`/machines/{id}/calendrier` renvoie un **301** vers `/machines/{id}#calendrier`.
⚠️ **301 et pas 404** : l'adresse est dans des favoris, dans les instructions
d'abonnement iCal et dans `ReservableResolver::calendarUrl` — qui pointe désormais
directement sur l'ancre, pour ne pas faire passer chaque lien par une redirection.
⚠️ Le bouton principal et le bouton flottant mobile ouvrent l'onglet
(`data-tab-link`) au lieu de charger une page.

**Espaces** : `/places/{id}` demandait une date et deux heures **saisies à
l'aveugle**, sur une page qui connaissait déjà les horaires et toutes les
réservations, et listait ces réservations à côté sous forme d'horodatages nus. Les
deux sont remplacés par le composant. ⚠️ **`app_place_reserve` existe toujours et
valide toujours ; plus rien ne pointe dessus.** Il se supprime en S146c — l'étape de
suppression — pas dans celle qui introduit son remplaçant.

⚠️ **Un onglet caché rend quand même.** Le contrôleur dessine dans un panneau en
`display: none` : la grille se calcule depuis la charge utile, pas depuis la mise en
page, donc rien n'a besoin d'être visible pour être correct.
⚠️ **Le hash pilote l'onglet** (`showTab`), sinon le 301 arriverait sur « Aperçu » —
une redirection qui atterrit à côté de ce qu'on demandait.

**Vérifié** : les 11 machines (page ET redirection) et les 2 espaces, 118 tests /
2295 assertions, l'onglet ouvert par le hash mesuré à l'écran (grille 10×7 = 70
cellules, panneau 1232×1258), et les deux appels à l'action rendus en tant que membre
autorisé pointent bien sur l'onglet.

### ✅ S146c — le calendrier montre, il ne réserve plus (livré 2026-08-21)

`/calendrier` portait une liste à cocher de **toutes** les machines et de tous les
espaces, une recherche, un filtre de statut et un brouillon de réservation par-dessus :
une deuxième façon de réserver, en concurrence avec la fiche de la ressource — laquelle
porte le même calendrier depuis S146b. Tout cela est supprimé. La page répond à la
question qu'elle sait bien traiter : **qu'est-ce qui se passe ici, et quand est-ce
ouvert**.

⚠️ **`booking: false` dans la charge utile est TOUT le mécanisme** : aucune cellule
cliquable, aucun `+`, et le dialogue n'est pas rendu du tout. Le composant n'a pas
changé — c'est le même calendrier que partout ailleurs (S146a). Mesuré sur la page en
ligne : 0 cellule `role="button"`, 0 affordance, aucun `.booking-panel` dans le DOM.

⚠️ **Les ressources voyagent toujours dans la charge utile.** Elles ne sont plus une
grille : elles servent à rattacher une réservation à ce lieu et à la nommer sur sa
carte. Ce qui est parti, c'est l'interface pour les filtrer.

✅ **`app_place_reserve` et `renderPlaceBookingError` supprimés** (~66 lignes), un pas
APRÈS l'arrivée de leur remplaçant — supprimer un chemin d'écriture dans l'étape même
qui introduit son successeur, c'est ainsi qu'on retire une route pendant que quelque
chose y poste encore. `suggestedSlot` et l'appel à `NextFreeSlotService` de
`placeDetail` partent avec : ils alimentaient le formulaire disparu.

⚠️ **« Au programme » est la décision que la feuille de route avait parquée** (« soit
le calendrier dit "Au programme" sans nommer un type, soit "Événements" couvre les
deux »). Les catégories de S146f la rendent sûre : chaque carte nomme désormais son
propre genre, donc le tableau n'a plus besoin d'un nom collectif. Le titre de la page
est le **nom du lieu** quand un lieu est choisi.
⚠️ Et la page **dit où la réservation est partie** — sous-titre explicite plus deux
boutons vers les catalogues. Un calendrier en lecture seule qui ne le dit pas est une
impasse.

**Vérifié** : 120 tests / 2305 assertions, 122 routes balayées, lecture seule mesurée
dans le DOM, la vue mois montre bien les événements avec leur catégorie en infobulle,
et la grille occupe toute la largeur (`.calendar-workspace.is-single`) au lieu de
laisser une gouttière vide de 280–340 px là où était le panneau.

### ✅ S146d complet — « toutes les semaines, ×4 » (livré 2026-08-21)

Quatre événements **créés d'un coup**, pas une règle de récurrence évaluée à la
lecture. `App\Calendar\EventSeries`, deux champs non mappés sur le formulaire de
CRÉATION seulement (répéter / nombre de séances).

🔴 **Ce sont des lignes indépendantes, pas une série.** Rien ne les relie : pas
d'identifiant de série, et modifier ou annuler l'une ne touche pas les autres. C'est
le but — sinon « déplacer la troisième séance » exigerait un modèle d'exceptions,
c'est-à-dire exactement le moteur qu'on évite. Vérifié en base : annuler la 2ᵉ laisse
les trois autres intactes.

⚠️ **Semaines uniquement, et c'est une décision.** `+1 mois` le 31 janvier tombe le
3 mars ; un rythme mensuel devrait d'abord répondre « quelle est la répétition
mensuelle du 31 ». Les semaines n'ont pas cette question. Ajouter les mois plus tard,
c'est trancher ça explicitement, pas élargir une constante.
⚠️ **Chaque occurrence est décalée depuis la PREMIÈRE date**, jamais depuis la
précédente : sinon la dérive s'accumule et, autour d'un changement d'heure, « +1
semaine » depuis une date déjà décalée n'est plus l'heure que l'opérateur a tapée.
⚠️ **L'affiche n'est pas recopiée** : un fichier appartient à une ligne, et partager
le nom casserait l'image des autres le jour où l'on en supprime une.
⚠️ **Un seul `flush()` pour toute la série** : la moitié d'un cours en base parce que
la 4ᵉ ligne a échoué est pire que rien.

🔴 **Le bug de S146f que cette étape a découvert : les deux formulaires d'événement
rendent leurs lignes UNE PAR UNE.** `category` et `formation` avaient été ajoutés au
type de formulaire en S146f et **ne s'affichaient nulle part** — il n'y a pas de
`form_rest()` pour les rattraper. Corrigé, et `EventSeriesTest` compare désormais les
`->add()` du type aux `form.<champ>` des deux gabarits, donc un champ ajouté sans être
rendu fait échouer la suite.
🔴 **Et le sélecteur de formation proposait les lignes internes de FabOS**
(`[FABOS SECTION] …`, `[FABOS BONUS] …`, catégories `Quiz interne` et
`Validation physique`) : `/formations/{id}` renvoie 404 pour elles, donc une séance
rattachée à l'une aurait pointé les membres vers une page inexistante. Filtrées.
🔴 Le menu « Répéter » portait aussi une option **vide** en tête, préselectionnée par
le navigateur, alors que « Une seule fois » EST déjà la réponse « pas de répétition ».

**Vérifié** : 127 tests / 2345 assertions (dont 7 unitaires sur les dates, le
plafond et la copie), **8/8 sur le vrai chemin d'écriture** — GET du formulaire, jeton
CSRF *stateless* et cookie double-submit repris, POST avec l'`Origin` — dans une
transaction annulée, 0 ligne survivante. 122 routes balayées.
⚠️ « Une seule fois crée bien UN seul » n'est pas testé en HTTP : Symfony met en cache
la liste de choix d'un formulaire par type, donc une 2ᵉ requête dans le même processus
réutilise des entités détachées par le `clear()` de la sonde — un artefact du harnais,
qu'une vraie requête ne produit jamais. La règle est couverte sans base par
`EventSeriesTest`.

### ✅ S146e — s'inscrire à une séance inscrit à la formation (livré 2026-08-21)

`App\Calendar\SessionEnrolment`, appelé depuis `EventRegistrationService`.

🔴 **ASSISTER NE QUALIFIE JAMAIS, et c'est là que la ligne est tracée.** Un badge dit
qu'une personne peut utiliser une machine sans surveillance ; être venue un soir n'en
est pas la preuve, et c'est un formateur qui valide. L'inscription écrit une
`Progression` **commencée** : `completed = false`, `score = 0`, aucune date de fin,
aucun badge. Une évolution qui poserait `completed` n'est pas une version plus large
de cette fonctionnalité, c'en est une autre, et dangereuse. `SessionEnrolmentContractTest`
échoue si quelqu'un l'écrit.

🔴 **Une progression existante est renvoyée INTACTE.** Quelqu'un qui a déjà trois quiz
derrière lui ne doit pas perdre son score en s'inscrivant à une séance — et
`unique_user_formation` ferait de toute façon échouer une insertion aveugle. Vérifié
en base : un score de 80 survit à une seconde inscription.

⚠️ **Les deux portes vers une place inscrivent** : `register()` et la promotion depuis
la liste d'attente. N'en câbler qu'une laisserait non inscrits tous ceux arrivés par
l'autre.
⚠️ **Dans la MÊME transaction que la place** : une place sans son inscription, ou
l'inverse, est un état que personne ne saura expliquer ensuite. Le service ne fait
donc pas de `flush()`.
⚠️ **Un invité ne peut pas être inscrit**, et c'est un fait de schéma :
`PROGRESSION.userId` est `NOT NULL`.
⚠️ **Annuler sa place ne DÉSINSCRIT PAS.** La progression peut déjà contenir du vrai
travail ; la supprimer pour refléter une place rendue le détruirait. Renoncer à une
place parle d'un soir, la formation dure plus longtemps.
⚠️ **Et c'est dit AVANT le bouton**, pas dans la confirmation : un membre qui
l'apprend après coup a été inscrit à quelque chose qu'il n'a pas sciemment choisi.

**Vérifié** : 133 tests / 2360 assertions, **11/11 sur la vraie base** dans une
transaction annulée (inscription → progression commencée mais non complétée, score 0,
pas de date de fin ; seconde inscription refusée et score de 80 conservé ; invité en
liste d'attente et aucune progression pour lui ; événement sans formation n'inscrit
personne), 0 ligne survivante. 122 routes balayées. La phrase vérifiée à l'écran sur
l'événement 10 et absente de l'événement 9.

### ✅ Revue de fin de phase S146 (2026-08-21, en ligne)

**Faite à la main dans la session** — celle-ci est configurée pour ne pas lancer de
sous-agent sans demande explicite ; l'opérateur a dit « run the review inline ».

🔴 **Un défaut trouvé et corrigé : l'écran des catégories vidait le formulaire.**
Un nom déjà pris renvoyait un `addFlash` + redirection, donc l'opérateur récupérait un
panneau **replié et vide** et devait retaper le nom ET l'icône. C'est exactement la
règle écrite en rouge. Corrigé : rendu sur place en 422 avec les valeurs saisies, et
le `<details>` s'ouvre. ⚠️ Le formulaire d'événement, lui, ne perdait rien — vérifié
champ par champ, y compris la catégorie et les réglages de répétition.
⚠️ **Et la première mesure était un FAUX NÉGATIF** : la sonde faisait son GET et son
POST dans deux sessions différentes, donc le jeton CSRF était refusé et c'est la
branche « CSRF invalide » — qui redirige aussi — qui était mesurée. Une session
partagée pour les deux requêtes. Se méfier d'un test de formulaire qui « échoue »
toujours de la même façon.

**Clics, avant → après** (comptés depuis la page de la ressource, ce qui a changé) :
- **réserver une machine** : 3 clics + 1 chargement de page → 3 clics, **0 chargement**.
  Le gain est une navigation, pas un clic — le dire honnêtement.
- **réserver un espace** : 1 clic + **jusqu'à 3 champs tapés** (date, début, fin) →
  2 clics, **0 frappe**. Un clic de plus, trois champs de moins.
- **« ce qui se passe au FabShop le mois prochain »** : **impossible** (ni filtre lieu
  ni vue mois) → **2 clics**. C'est une capacité nouvelle, pas une optimisation.
- **un cours de 4 semaines** : 4 formulaires complets → **1 formulaire + 2 champs**.

**Champs demandés** : la création d'événement en compte 14, dont **3 obligatoires**
(titre, début, où). Les 11 autres sont facultatifs.

**Suites données (opérateur, 2026-08-21 : « rename the Calendrier nav entry and hide
repeatCount ») :**
1. ✅ **L'entrée de menu s'appelle « Au programme »** (`nav.whats_on`), comme le
   tableau sur lequel elle tombe. 🔴 **`nav.calendar` n'a PAS été renommée** : la même
   clé est le libellé de l'onglet de la fiche machine, et là le calendrier est bien un
   calendrier depuis lequel on réserve. Une clé neuve, pas une valeur changée. Le
   groupe de navigation et la destination de recherche suivent ; ses enfants disent
   toujours « Réserver … », ce qui est exactement le panneau indicateur voulu.
2. ✅ **« Nombre de séances » ne s'affiche que lorsqu'une répétition est choisie**
   (`assets/controllers/conditional_field_controller.js`, générique : une source, des
   dépendants qui listent les valeurs qui les révèlent). ⚠️ **Il masque, il n'efface
   jamais** : choisir « toutes les semaines », taper 4, revenir à « une seule fois »
   puis y retourner rend le 4 — sinon la règle de non-ressaisie serait cassée par une
   autre porte. ⚠️ Sans JavaScript les deux champs restent visibles et le formulaire
   se comporte comme avant.
   ⚠️ **Ce qui n'est PAS prouvé** : que Stimulus démarre bien ce contrôleur sur la page
   admin en ligne. Le rendu local charge ses modules depuis `fabos.dstei.fr` et **CORS
   les bloque**, donc la logique a été exercée contre le vrai balisage (cible trouvée,
   `hidden` réduit bien la ligne à 0 px, valeur conservée) mais pas le démarrage. Ce
   mécanisme-là est celui qui fait déjà tourner `confirm`, `admin-list-filter`,
   `autosubmit`, `deck` et `clock` sur les pages admin.
   ⚠️ **Piège de méthode** : la première mesure disait « jamais masqué » parce que le
   script précédent avait laissé le `<select>` sur `two_weeks` et que la page n'avait
   pas été rechargée. **Un test d'interface qui ne recharge pas se mesure lui-même.**

🟡 **Reste à trancher, rien changé :** la légende de la semaine dit « Disponible » sur
un calendrier en lecture seule. Le mot voulait dire « cliquable » ; il veut dire « le
labo est ouvert ». Pas faux, légèrement chargé.

⚠️ **Aucune affordance morte introduite** : le seul `disabled` de la phase est une
`<option>` de ressource verrouillée, qui porte sa raison dans son libellé.

### 🟡 todo consigné 2026-08-21 — supprimer en masse ce qu'on a créé en masse

**Mots de l'opérateur** : *« if we can create X events, we have to have a way to mass
delete them »*. ⚠️ **Consigné, pas construit.** Et c'est juste : S146d crée jusqu'à
12 événements d'un seul envoi, et `/admin/events` ne sait les retirer qu'un par un.

🔴 **La tension à trancher d'abord.** S146d a délibérément fait des lignes
**indépendantes** : pas d'identifiant de série, pour que déplacer ou annuler une séance
ne demande pas un modèle d'exceptions. Une suppression en masse a donc besoin d'une
autre prise. Les deux voies, et ce qu'elles coûtent :
- **Sélection multiple sur `/admin/events`** (cases à cocher + une action groupée).
  Ne suppose aucune série, sert aussi à nettoyer n'importe quel lot, et c'est un
  motif que la liste peut réutiliser ailleurs. Plus de travail d'interface.
- **Un identifiant de série sur les lignes générées**, nullable, purement informatif :
  « supprimer les 4 séances ». Moins de clics, mais réintroduit la notion de série
  que S146d a évitée — et il faudra décider ce qu'il advient d'une séance déplacée
  ou annulée, ce qui est exactement la question qu'on ne voulait pas poser.
⚠️ Quoi qu'il arrive : **une séance à laquelle des gens sont inscrits ne se supprime
pas en silence** — S146e y attache maintenant des progressions, et l'annulation
motivée (`callOff`) existe précisément pour prévenir les inscrits. Supprimer et
annuler ne sont pas la même action.

### 🟡 todo consigné 2026-08-20 — une catégorie peut devenir une entrée de menu

**Mots de l'opérateur** : *« the events featureset should offer a settings where the
categories are displayed in the main menu to list only those (could even just be a
link to the event calendar with the filter on) »*. ⚠️ **Consigné, pas construit.**

**Ce que c'est, réduit à l'essentiel** : une entrée de menu EST un filtre enregistré.
`/events?category=<slug>` existe déjà et fonctionne depuis S146f, donc la
fonctionnalité est un **réglage de navigation**, pas une nouvelle page : « quelles
catégories apparaissent dans le menu principal », et le lien pointe sur le catalogue
déjà filtré. C'est l'observation de l'opérateur lui-même et elle est juste — elle
supprime tout le travail de gabarit.

**Ce qu'il faut vérifier avant de le faire :**
- ⚠️ **Où vit le réglage.** Le menu public n'est pas `NavBuilder::admin()` ; regarder
  qui construit le menu principal et si Configuration → Thèmes est déjà l'endroit des
  menus (la feuille de route dit « Thèmes réunit identité visuelle, images, menus et
  accueil » — si oui, c'est là, pas dans un écran Événements).
- 🔴 **Un menu ne doit pas nommer une catégorie archivée ni une catégorie vide.** Une
  entrée qui mène à zéro résultat est une affordance morte, et archiver une catégorie
  doit la retirer du menu sans que personne y pense.
- ⚠️ **Le slug est la clé, jamais le libellé** : renommer une catégorie ne doit pas
  casser l'entrée de menu (c'est déjà pour ça que le slug ne suit pas le renommage).
- ⚠️ **Traduction** : le libellé est un CONTENU, donc l'entrée de menu s'affichera
  dans la langue où l'opérateur l'a tapée, à côté d'entrées traduites. C'est la
  conséquence acceptée en S146f, mais elle se voit beaucoup plus dans un menu.
- ⚠️ Combien d'entrées au maximum ? Le menu principal a déjà cinq entrées ; quatre
  catégories en plus le doublent. Un plafond, ou un sous-menu sous « Activité ».

### ✅ S146a — ce qu'il faut SAVOIR maintenant (livré 2026-08-20)

**Il y a UN calendrier.** Le composant est `assets/controllers/calendar_controller.js`
(rendu, vues, règles d'ouverture, panneau de réservation), les gabarits sont
`site/_calendar.html.twig` (le tableau) et `site/_calendar_booking.html.twig` (le
dialogue), et la charge utile est construite une fois par `App\Calendar\CalendarPayload`.
⚠️ **L'élément contrôlé est celui de la PAGE** (`data-controller="calendar"` sur
`.calendar-shell`) : les cartes de synthèse, le panneau de filtres et le dialogue
sont des *targets* du même contrôleur, et Stimulus ne voit que ce qui est dans
l'élément. Le dialogue a donc été déplacé DANS le shell — il est `position: fixed`,
rien ne bouge à l'écran.

⚠️ **Deux changements de comportement assumés**, tout le reste est à l'identique :
1. **Un visiteur non connecté voit une grille OUVERTE sur les deux pages.** La fiche
   machine grisait toute sa semaine pour un anonyme, ce qui dit « le labo est
   indisponible » au lieu de « il vous faut un compte ». Le formulaire reste fermé
   côté serveur (`{% if signed_in %}`), donc rien n'est desserré.
2. **Le titre du tableau est neutre** (`cal.board_title` / `cal.board_desc`).
   « Semaine de réservation » mentait dès que le mois s'affichait dans la même boîte.

🔴 **Trois défauts trouvés à l'écran, pas dans le code** — et c'est la leçon :
- la barre de navigation passait sur **trois lignes** à 1440 px une fois le sélecteur
  de vue ajouté, les deux flèches finissant sur des lignes différentes. Corrigé par
  des flèches (`‹`/`›`, le mot reste en `aria-label` et en `title`) + une barre qui
  refuse de se couper (`flex-wrap: nowrap`) et descend entière ;
- 🔴 **`hidden` perd contre un `display` explicite.** Le contrôleur masque la bande
  d'horaires et la légende en vue mois via l'attribut `hidden`, mais
  `.week-hours-strip { display: flex }` de cette feuille battait la règle UA : les
  deux restaient à l'écran sous la grille du mois. Corrigé par
  `[data-calendar-week-only][hidden] { display: none }` ;
- 🔴 **`--cal-surface` sur `--cal-surface-soft` se lit À L'ENVERS en sombre.** Le
  segment sélectionné du sélecteur de vue devenait le plus FONCÉ, donc « Mois »
  paraissait éteint pendant que le mois était affiché. Il se marque maintenant comme
  les tuiles de lieu juste à côté (même accent, même `color-mix`).

⚠️ **Le filtre lieu est un lien rendu par le serveur, jamais une bascule client.**
Les horaires, les exceptions datées ET les ressources réservables changent avec le
lieu et viennent tous du serveur. La vue (`?view=month`), elle, est client et se
réécrit dans l'URL (`history.replaceState`) ; les tuiles de lieu la reportent.

⚠️ **Ce que S146b/c héritent** : `booking: false` dans la charge utile suffira à
rendre `/calendrier` lecture seule (S146c), et le sélecteur de ressource
n'apparaît que s'il y a plus d'une ressource — la fiche machine (S146b) n'aura donc
pas de menu à une seule entrée.

**Vérifié** : 112 tests / 2261 assertions, 121 routes sans paramètre balayées (seuls
`/.well-known/fabos` 503, `/desabonnement` 400 et `/api/me/favorite-machines` 401,
tous délibérés), les 11 calendriers machine en semaine ET en mois, les quatre valeurs
de `?location=` (plus un slug inconnu → 400, comme les catalogues), clair et sombre.
`?v=20260820-s146a`.

### ⚠️ Revue de fin de PHASE — obligatoire

**Demande opérateur, 2026-08-20**, resserrée le même jour : *« do the review at the
end of each phase to save on tokens »*. Donc **une fois, à la fin de la phase**, et
non après chaque étape — une revue complète par étape re-dérive cinq fois le même
contexte pour ce qu'un coup d'œil à l'écran pendant l'étape trouve déjà.
Le mandat reste celui d'un designer d'Apple. Ce qu'il faut compter et rapporter :

- **le nombre de clics** pour chaque parcours livré, avant et après ;
- **l'évidence du chemin** : est-il évident où se trouve l'information, sans
  l'avoir apprise ?
- **le nombre de frappes** pour interagir — et **surtout en cas d'erreur** :
  🔴 **un champ invalide ne doit JAMAIS obliger à ressaisir le reste du
  formulaire** ;
- **ce qu'on demande sans que ce soit absolument nécessaire** : tout champ non
  indispensable est une question qu'on n'aurait pas dû poser.

⚠️ Le sous-agent relit **le résultat**, pas le diff : il faut lui donner les URLs
et les parcours, pas seulement les fichiers.

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
