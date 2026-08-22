# FabOS — roadmap active

**Mise à jour : 2026-08-21.** ⚠️ **Cette page ne contient que le travail
restant.** Tout ce qui est livré en est sorti : les récits sont dans
`HISTORY.md`. Une session livrée qui reste ici finit par être refaite — c'est arrivé
deux fois. ⚠️ **Élagué le 2026-08-21** : 611 lignes de récit livré sont parties dans
`HISTORY.md`, il ne reste ici que le travail à faire et les règles à ne pas défaire.

**Livré à ce jour** : phases A à F (S102–S128), **toute la Phase G — S134b compris, donc
la phase est CLOSE**, la Phase G2, l'interface S134h–S143, S144, S145a, et **toute la
phase S146 (a→g)**.
⏭️ **La barrière avant le commerce est désormais la Phase J** (« boutonner »), en bas de
cette page. ⚠️ Les droits d'usage sont APPLIQUÉS, les quatre chokepoints sont sur grants
v2, et les deux tables doublonnées ont été supprimées le 2026-08-21.

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

## Phase G — ✅ CLOSE (2026-08-21)

Elle était la barrière du MODÈLE avant le commerce, et elle est franchie : S134b a livré
l'inventaire action-opérateur, les routes orphelines, la migration de contract et les
traductions. ⚠️ **La barrière est maintenant la Phase J**, plus bas.
Ce qui suit n'est plus un plan mais **l'état vivant du modèle de droits** — à lire avant
d'y toucher — et le peu qui reste ouvert.

| Session | Attendu | Qui |
|---|---|---|
| ✅ **S132** | shell/design system : Logs RFID, Réglages, E-mails et Fonctionnalités refaits ; quatre `<style>` locaux supprimés | Luna + Terra |
| ✅ **S133** | parité : CRUD des catégories machine, « Ailleurs / externe », fiche canonique d'un objet, Packages réduit à ce qu'il expose | Terra + Luna |
| ✅ **S133b** | groupes, audiences, grants v2 Use/Manage — **en ombre**, visible et explicable | Terra + Luna |
| ✅ **S134** | mécanisme d'activation graduelle + garde-fou ; **les quatre chokepoints sont basculés**, l'éditeur de grants et l'application du lieu sont livrés | Terra + Sol |
| ✅ **S134b** | **livré 2026-08-21.** Inventaire action-opérateur (machine et formation s'archivent enfin), routes orphelines (2 vraies trouvailles), migration de contract passée, **154 messages flash traduits dans les cinq langues**, a11y mesurée propre. | Luna + Terra, Sol valide |

✅ **Le critère de sortie de la Phase G est atteint** (vérifié 2026-08-16) : le
**champ lieu existe sur les huit formulaires** machine/espace/événement/objet
via `VenueChoiceType`.

### 🧠 Ce qu'il faut SAVOIR après S146 et S134b — les règles, pas le récit

**Le récit complet est dans `HISTORY.md` § « Session 2026-08-20/21 ».** Ce qui suit est
seulement ce qu'une session future doit savoir pour ne pas défaire le travail.

**Le calendrier.** Il n'y en a qu'UN : `assets/controllers/calendar_controller.js`,
`site/_calendar.html.twig`, `site/_calendar_booking.html.twig`, alimentés par
`App\Calendar\CalendarPayload`. Vues semaine et mois, filtre lieu server-side.
⚠️ **`booking: false` dans la charge utile rend le composant lecture seule** — c'est tout
le mécanisme, et c'est ce qui fait de `/calendrier` une page d'activité. La réservation
vit sur la fiche de la ressource (`/machines/{id}` onglet, `/places/{id}`).
⚠️ Les règles horaires du JS doivent rester le miroir de `ScheduleResolver` : exception
datée d'abord, puis les PLAGES, jamais l'enveloppe.

**Événements.** ⚠️ `Event.category` est un LIBELLÉ, `Event.formation` est un LIEN, et
l'un ne remplace pas l'autre. 🔴 **Aucun code ne branche sur une catégorie précise** —
`EventCategoryContractTest` échoue sinon. 🔴 **Assister ne qualifie JAMAIS** :
`SessionEnrolment` écrit une progression *commencée* (`completed = false`, score 0), un
formateur valide. Les deux portes vers une place inscrivent (inscription ET promotion
depuis la liste d'attente), dans la même transaction que la place.
⚠️ `EventSeries` génère N séances À LA CRÉATION : **semaines uniquement** (`+1 mois` le
31 janvier tombe le 3 mars), chaque occurrence décalée depuis la PREMIÈRE date, affiche
non recopiée, un seul `flush()`. Les lignes sont **indépendantes** : pas d'identifiant de
série, donc une suppression groupée demandera une autre prise (todo ci-dessous).
⚠️ **La table est `EVENEMENT`**, pas `EVENT`.

**Archivage (S134b).** `Machine` et `Formation` portent `archivedAt`.
🔴 **Masquer n'est pas refuser** : `MachineRepository::findLive()` les retire des surfaces
qui les PROPOSENT, et `ReservationService` refuse la réservation d'une machine archivée
(`MACHINE_ARCHIVED`) au point de passage unique — `/machines/{id}` répond toujours à qui a
le lien. ⚠️ `findBy()` reste la question de l'ADMIN. ⚠️ `countVisible()` doit porter les
mêmes conditions que `findVisible()`.
⚠️ Les packages s'archivent par leur case `active`, pas par `archivedAt` : **un audit par
nom de route ne voit pas un verbe qui est un champ.**

**Horaires.** `SCHEDULE_EXCEPTION.endDate` permet une fermeture de plusieurs jours — UNE
ligne, pas N. ⚠️ **`null` veut dire « un jour » et chaque lecteur le dit avec `COALESCE`** ;
`betweenFor` est un CHEVAUCHEMENT, pas une inclusion ; `exceptionsBetween()` déplie la
plage par date et la borne à la fenêtre demandée.

**Messages flash.** ⚠️ Un flash porte une CLÉ : `addFlash('success', 'flash.x')` ou
`['flash.x', ['%p1%' => $v]]`, traduite au rendu par `|flash_text`. 🔴 **Une clé inconnue
traverse inchangée** — ne pas « corriger » ça en levant une exception. Tout nouveau flash
se catalogue.

**Pièges payés cette session, à ne pas repayer.**
🔴 **`x|default(true)` est TOUJOURS vrai si on passe `false`** : Twig déclenche `default`
sur toute valeur *vide*. Deux fois livré ici. Écrire `x is not defined or x`.
🔴 **`hidden` perd contre un `display` explicite** d'une feuille de l'auteur.
🔴 **`php -l` vert ne dit rien du conteneur** (un `use` manquant sur un paramètre de
contrôleur = 500). **Un filtre Twig neuf exige `cache:clear` AVANT `lint:twig`.**
🔴 **Supprimer un gabarit emporte ses liens** — la route répond toujours 200, donc ni les
tests ni le balayage ne le voient.
⚠️ Sondes : forger le CSRF dans LA requête qu'on envoie ; vider les mémos de
`ScheduleResolver` par réflexion entre deux écritures ; recharger la page entre deux
mesures d'interface.
### 🔴 L'état vivant du modèle de droits — à lire avant d'y toucher

**L'enforcement est ON et les quatre chokepoints sont sur grants v2**
(`usage_rights_v2_machines|places|person_booking|events`). L'écran d'ombre lit
**0 perdraient / 0 gagneraient / 12 identiques / 24 recovery admin** sur 9
membres : v1 et v2 sont d'accord partout.

✅ **Les deux tables doublonnées sont SUPPRIMÉES** (`Version20260821140000`, passée le
2026-08-21). `USAGE_GRANT` et `USAGE_PACKAGE_GROUP_ASSIGNMENT` n'existent plus : vérifié
avant le DROP que rien ne les lisait, que la seconde était vide et que les 21 lignes de
la première avaient toutes un jumeau exact dans `USAGE_PACKAGE_GRANT`.

🔴 **`USAGE_PACKAGE_GRANT` est LA table de grants** (S111, `Version20260809150000`).
S133b en avait créé une seconde faute d'avoir grepé `migrations/` ; convergée en S134b,
supprimée en 2026-08-21. ⚠️ Sa colonne est `sectionKey`, pas `section`.

🔴 **Un grant limité à un lieu n'a rien fait pendant deux sessions** : `verdict()`
n'avait pas de paramètre de lieu, tous les appelants passaient `null`, et la
branche `:venue IS NULL` matchait tout. Corrigé en S134b ; `VenueScopedGrantTest`
épingle le câblage. ⚠️ `null` reste **permissif** par décision.

**L'éditeur de grants existe** : `/admin/usage-rights/{id}/edit`, section
« Grants v2 ». ✅ **Le formulaire d'attribution à un GROUPE existe depuis S144a.**

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

### Thèmes — le chantier entier (session dédiée, non planifiée)

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

## S146 — livré (2026-08-20/21)

**Toute la phase est livrée : a→g, plus la revue de fin de phase et ses suites.** La
proposition, le constat sur les formations sans sessions et le découpage sont dans
`HISTORY.md`. Les règles à connaître sont plus haut, § « Ce qu'il faut SAVOIR après S146
et S134b ». Ce qui suit est ce qui reste OUVERT.

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

## Phase J — « boutonner » : stabiliser AVANT le commerce

**Demande opérateur, 2026-08-21**, ses mots : *« before commerce i want to smooth out
a lot of things, including a complete review of the packages and core features. Right
now there are a bunch of messy pages… Let's stabilize the current site before commerce…
We make sure everything is up to design guidelines, number of clicks, act like apple
engineers and button everything up. »*

🔴 **Phase J est BLOQUANTE avant la Phase H.** La Phase G l'était pour le modèle ; J
l'est pour la finition. Ne commencer ni paiement, ni catalogue d'offres, ni ledger tant
que les critères de sortie ci-dessous ne sont pas atteints. La raison est la même que
pour G : vendre une surface qu'on n'a pas fini de dessiner, c'est figer ses défauts dans
un contrat client.

### L'état mesuré au 2026-08-21 (chiffres, pas adjectifs)

| Ce qui est mesuré | Aujourd'hui |
|---|---|
| Règles CSS locales dans les gabarits | **783**, dans **82** gabarits qui portent un `<style>` |
| Les pires | `formation-suivi` 126 · `event-detail` 83 · `machine-historique` 65 · `admin-dashboard` 41 |
| Gabarits avec leur PROPRE `<head>` (hors shell) | **48** — et Stimulus n'y tourne pas |
| Gabarits d'administration | **73**, dont **69** sur une coquille partagée (4 hors) |
| Catalogues de traduction | 3 026 clés × 5 langues, **0 manquante** |
| a11y (9 pages mesurées) | 0 image sans `alt`, 0 bouton sans nom, 0 champ sans étiquette |

⚠️ **`admin-design` (42 règles) est légitime : il EST le guide.** Ne pas le compter
comme dette.

### Ce que « boutonné » veut dire — la liste, appliquée à CHAQUE écran

Un écran est fini quand les dix réponses sont oui. C'est la version testable de
« act like apple engineers » ; rien ici n'est une opinion.

1. **Coquille partagée.** Pas de `<head>` maison, pas de `<style>` local qui ne soit pas
   devenu une règle du guide. Si un écran a besoin d'une primitive, elle va dans
   `/admin/design` et dans la feuille partagée — c'est la règle de l'opérateur : *« if
   you need custom css it's probably worth making it a design guideline »*.
2. **Le titre vient de `NavBuilder`**, jamais recopié.
3. **Listes** : cinq colonnes maximum, actions comprises ; autant de cellules que d'
   en-têtes ; pas de `colspan` compté à la main.
4. **Chaque objet annoncé est créable, éditable, archivable** depuis son workspace —
   le critère de S134b, à re-vérifier écran par écran.
5. **Aucune affordance morte** : pas de bouton qui ne peut pas aboutir, pas de lien vers
   une page qui 404, pas de filtre qui ne filtre rien.
6. **Cinq langues, sombre, mobile, clavier.** Vérifié à l'écran, pas supposé.
7. **Le nombre de clics est COMPTÉ**, avant et après, pour le parcours principal de
   l'écran.
8. 🔴 **Un champ invalide ne fait JAMAIS ressaisir le reste du formulaire.** Testé par
   un vrai POST refusé, pas relu.
9. **Zéro champ non indispensable**, et un champ qui ne peut rien faire n'est pas
   dessiné (cf. `conditional_field`).
10. **Les primitives sont montrées dans `/admin/design`** avec le vrai composant.

### Découpage

| Étape | Livre | Qui |
|---|---|---|
| **S147 — LA REVUE** | 🔴 **Aucun code.** Terra rend et mesure chaque surface contre les dix points et produit **une liste de défauts par écran, chiffrée** ; l'opérateur la parcourt et tranche ce qui compte, ce qui attend, ce qui est faux. Sortie : la liste ordonnée qui pilote S148 et S149. | Terra mesure, **l'opérateur arbitre** |
| **S148 — le socle** | Réglages, Fonctionnalités, E-mails, Logs RFID, Thèmes, Setup/assistant, Tableau de bord. ⚠️ **Absorbe ce qui restait de S132**, qui traînait depuis la Phase G. | Luna + Terra |
| **S149 — feature par feature** | Une lettre par feature : machines, espaces, événements, formations, prêts, matériaux, badges, projets, réservations, packages/quotas. Chacune finie selon les dix points. | Luna + Terra |
| **S149z — la sortie** | Revue conjointe finale : l'opérateur et Terra reprennent la liste de S147 et vérifient qu'elle est vide ou consciemment reportée. | **Opérateur + Terra** |

⚠️ **La revue vient EN PREMIER, et elle ne code pas.** Cette base a payé cher l'inverse :
un chiffre inventé a cadré une session entière (S134j), et deux sessions ont refait du
travail déjà livré. On mesure, on montre, on décide, puis on fait.
⚠️ **La revue de fin est une fois par PHASE**, pas par étape (opérateur, 2026-08-20).

### Critères de sortie de la Phase J

- la liste de défauts de S147 est vide, ou chaque reste est **consciemment reporté et
  écrit** ;
- **aucun gabarit ne porte de `<style>` local** hors `admin-design` — ou chaque exception
  restante est une règle du guide, nommée ;
- **les 48 gabarits à `<head>` propre** sont ramenés sur la coquille, ou la liste de
  ceux qui doivent rester (kiosques plein écran) est écrite et justifiée ;
- les dix points passent sur **chaque** écran des features et du socle ;
- `/admin/design` montre chaque primitive utilisée, avec le vrai composant.

### 🅿️ Ce qui reste parqué et n'entre PAS dans J

Le sélecteur de langue (`app_switch_locale`), la suppression en masse d'événements, les
catégories comme entrées de menu, le tableau de bord « qui doit re-briller ». ⚠️ Le
tableau de bord est le seul des quatre qui touche J : il est dans S148. Les trois autres
sont des fonctionnalités, pas de la finition — les traiter dans J ferait grossir la
phase jusqu'à ce qu'elle ne finisse jamais.

## Phase H — commerce facultatif (S150–S154)

🔴 **BLOQUÉE PAR LA PHASE J** (opérateur, 2026-08-21) : stabiliser le site avant de
vendre quoi que ce soit. La Phase G était la barrière du modèle ; J est celle de la
finition.

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
