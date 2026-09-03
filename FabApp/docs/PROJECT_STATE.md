# FabOS — comment ça marche

**MAJ 2026-08-23.** Ici : ce qui est VRAI aujourd'hui. Pas de récit, pas de
« comment on en est arrivé là ». Le récit → [`HISTORY.md`](/roadmap/historique).

- Ce qui reste à faire → [`ROADMAP.md`](/roadmap)
- Où on en est cette semaine → [`WORKING_BRIEF.md`](/roadmap/brief)
- Modèle cible des droits → [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage)
- Déployer → `ARTEMIS_DEPLOYMENT.md`

---

## 1. Le produit

Plateforme de gestion d'ateliers partagés. **Symfony 8.1 / PHP 8.4 / MariaDB**,
Twig rendu serveur, **pas de SPA**. L'app est **uniquement `FabApp/`** ; les
dossiers frères du monorepo sont sans rapport ou historiques.

Live : https://fabos.dstei.fr · `APP_ENV=prod` · CT 210 sur Proxmox.

Une installation = **une** gouvernance, **plusieurs lieux** physiques. Pas de
portails (retirés en S127). Autre gouvernance = autre FabOS ; le SSO évite un mot
de passe, il ne partage ni droits ni données.

## 2. Le calendrier est la colonne vertébrale

Pas une feature : la colonne. Tout ce qui se réserve est une *couche de
ressource* projetée sur un calendrier commun.

- `RESERVATION` adresse sa cible **seulement** par `(reservableType, reservableId)`,
  les deux NOT NULL. Plus de colonnes `machineId`/`placeId`.
- `src/Reservation/` : enum `ReservableType` (`machine | place | user`), VO
  `ReservableRef`, `ReservableResolver` (**`warm()` obligatoire sur les listes** —
  il n'y a aucune jointure), `ReservationService`, `BookingResult`.
- `reservableLabel` fige le nom au moment de réserver. Pas de FK → pas de cascade,
  volontaire : l'historique d'une ressource supprimée reste lisible.
- 🔴 **Pas de cascade = nettoyage explicite.** Tout chemin de suppression /
  archivage d'une ressource réservable doit annuler lui-même ses réservations à
  venir (`cancelUpcomingForReservable()`).
- `NextFreeSlotService` répond « c'est libre quand ? » **en interrogeant
  `BookingPolicyService`**, jamais en réimplémentant les règles. Il propose ;
  `ReservationService::book()` décide.

**Ajouter un type réservable** = 1 cas de `ReservableType` + 1 branche de
`ReservableResolver` + 1 bras dans `checkAccess()` + 1 entrée dans les deux
constructeurs de calendrier.

### UN seul composant calendrier

`assets/controllers/calendar_controller.js` + `site/_calendar.html.twig` +
`site/_calendar_booking.html.twig`, alimentés par `App\Calendar\CalendarPayload`.
Vues semaine et mois, filtre lieu côté serveur.

- ⚠️ **`booking: false` dans la charge utile = composant en lecture seule.** C'est
  tout le mécanisme. C'est ce qui fait de `/calendrier` une page d'ACTIVITÉ.
- Réserver se fait sur la fiche de la ressource : `/machines/{id}` (onglet),
  `/places/{id}`.
- ⚠️ Les règles horaires du JS sont le **miroir** de `ScheduleResolver` : exception
  datée d'abord, puis les PLAGES, jamais l'enveloppe.

## 3. Réserver : quatre couches indépendantes

Elles se rejoignent dans `ReservationService::book()` et **restent séparées**.

| Couche | Question | Où | Si on fusionne |
|---|---|---|---|
| **Certification** | as-tu le droit d'y toucher ? (sécurité) | `checkAccess()` → `MachineQualificationService` | assouplir un quota assouplirait la sécurité |
| **Quotas** | combien / à quelle distance ? (équité) | `BookingPolicyService::check()` | — |
| **Accès exceptionnels** | exemption de QUOTA seulement | `AccessPassRepository` + court-circuit `$passApplies` | un pass deviendrait un contournement de sécurité |
| **Droits d'usage** | le package du membre couvre-t-il cette action, cet intervalle ? | `UsageRightsService` + `UsageCapabilityRegistry` | l'UI et l'application divergent |

- 🔴 **Pas de colonne « bypass certification » dans `ACCESS_PASS`.** Ne pas en
  ajouter.
- Refus de quota = **409**, pas 403. Ce n'est pas interdit, c'est en conflit.
- Ordre des quotas : contrainte la plus grossière d'abord (préavis/horizon avant
  l'alignement de créneau).
- Compteurs actif/jour/semaine **scopés par `ReservableType`** : une réservation
  machine ne consomme pas un quota espace.

## 4. Features (modules)

`src/Feature/` est toute l'histoire. Un `SiteFeature` porte le libellé opérateur
**et** la clé qui garde les routes — c'est la même chose. `SiteFeatureRegistry` =
le catalogue ; `SiteFeatureService` = l'état. Stockage : `SITE_MODULE`.

Trois natures :
- **resource** — un type réservable, et si les réservations de ce type sont
  acceptées : `machines`, `places`, `person_booking`.
- **activity** — un domaine avec ses pages et données : `events`, `formations`,
  `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans`,
  `maintenance`.
- **directory** — **une page et une entrée de menu, rien d'autre** : `staff`,
  `trainers`.

**Le noyau n'est jamais un interrupteur** : utilisateurs, rôles, autorisation,
desk staff, auth, profils, réglages, transport mail, moteur de réservation.

- `parent` → add-on : forcé OFF par le service dès que le parent est OFF.
- `calendarLayer` ⚠️ **plus étroit que `resource`** : `person_booking` est
  réservable et ne dessine PAS de colonne, sinon `/calendrier` revient comme une
  grille vide sur une installation « rendez-vous seulement ».
- `reservable` → lu par `ReservationService::book()` et le scanner de rappels.

🔴 **`isEnabled()` rend TRUE pour une clé inconnue** — un gate mal orthographié est
silencieusement toujours ON. Pour tout ce qui est navigable, utiliser
`allowsSurface()`. La galerie est `projects`.

🔴 **Garder une route ne protège pas une écriture.** `/api/reservations` prend la
charge polymorphe directement : le gate vit à `ReservationService::book()`, le
point de passage unique.

⚠️ `FeatureAccessSubscriber` garde **par NOM de route** : un préfixe est une
promesse. Les annuaires matchent **exactement** (`app_staff`, `app_trainers`).

⚠️ `MachineQualificationService` n'a **aucun** contrôle de feature : couper
`badges` ne rouvre PAS l'équipement.

**Ajouter une feature** : entrée dans `SiteFeatureRegistry::build()` · bras dans
`FeatureAccessSubscriber` · nav via `feature_enabled()` · i18n dans les **cinq**
catalogues · méthodes de repository **fail-safe** (try/catch → `[]`) · `reservable`
si c'est une ressource.

**Ce qui lit le registre** : `NavBuilder` (en-tête, pied, `safeDestinations()`),
`SetupHealth` (`/admin/setup`), `FirstRun` (`/admin/wizard`), `FeatureAdvice`
(avertit, ne bloque jamais), `MissingPageLog` (`/admin/missing-pages`).

⚠️ **Un listener `kernel.exception` sous la priorité −128 ne tourne jamais** :
`ErrorListener` appelle `stopPropagation()`. Le subscriber de pages manquantes est
à **−100** et doit y rester.

## 5. Droits d'usage — état vivant

**L'application est ON. Les quatre chokepoints sont sur grants v2** :
`usage_rights_v2_machines | places | person_booking | events`.

- 🔴 **DEUX VOCABULAIRES, ET C'EST DÉLIBÉRÉ (S153d).** L'écran dit **« forfait »**
  (`bundle` en anglais, `Nutzungspaket`, `plan`, `forfait`) ; le code, les tables
  et les URLs disent **`package`** — `USAGE_PACKAGE`, `UsagePackageRepository`,
  `/admin/usage-rights`. Renommer les tables aurait été un expand/contract complet
  pour zéro bénéfice visible, et casser les URLs pour du vocabulaire est le mauvais
  échange. ⚠️ **Donc : chercher « forfait » dans le code ne rend rien.** Le mot n'a
  changé que dans les cinq catalogues de traduction et dans les messages
  d'exception qui atteignent un flash.
  ✅ Le motif du renommage : la Phase H apporte offres et paiements, où « package »
  voudrait dire *une chose qu'on achète*. Le mot est libéré avant qu'il n'arrive.

- 🔴 **`USAGE_PACKAGE_GRANT` est LA table de grants.** Sa colonne est `sectionKey`,
  pas `section`. `USAGE_GRANT` et `USAGE_PACKAGE_GROUP_ASSIGNMENT` ont été
  **supprimées** (`Version20260821140000`, 2026-08-21).
- Deux actions seulement : **Use** et **Manage**. Manage inclut consultation et
  export ; **Manage ne confère JAMAIS Use**. Pas de rôle « Report ».
- **Packages cumulatifs, fermés par défaut. Aucun package ne retire un droit.**
  Les grants s'additionnent en OU.
- Portée par ressource : `reservableType / reservableId / categoryId` (l'identité
  décide, plus le libellé — J-21, 2026-08-23).
- Heure de semaine : `USAGE_GRANT_WINDOW`, N plages par grant, vérifiées par
  **couverture** de toute la réservation (`GrantWindowSet`).
- 🔴 **Une plage n'enferme personne** : « seulement le jeudi après-midi » s'obtient
  en **resserrant** le grant, jamais en ajoutant une plage à côté.
- ⚠️ Un lieu `null` sur un grant reste **permissif**, par décision.
- Éditeur : `/admin/usage-rights/{id}/edit`. Écran d'ombre :
  `/admin/usage-rights/shadow`.
- 🔴 **Un compte « identique » sur l'écran d'ombre ne prouve pas une égalité** : les
  lignes « recovery admin » contournent le modèle et sont d'accord avec elles-mêmes.

⚠️ `readiness()` et la liste des packages comptent les attributions **directes** :
un package tenu seulement par un groupe affiche 0 (todo S144e).

## 6. Horaires

`ScheduleResolver` est **le** lecteur. Il demande toujours **de quel lieu** on
parle ; les douze appelants passent le lieu qu'ils connaissent.

- Plusieurs plages par jour et par lieu (l'unicité `(venueId, dayOfWeek)` est
  tombée en S134d).
- `SCHEDULE_EXCEPTION` = fermeture datée avec sa raison. `endDate` permet
  plusieurs jours en **UNE** ligne. ⚠️ **`null` veut dire « un jour »** et chaque
  lecteur le dit avec `COALESCE`.
- `betweenFor` est un **CHEVAUCHEMENT**, pas une inclusion. `exceptionsBetween()`
  déplie la plage par date et la borne à la fenêtre demandée.
- Portée attachable : `OPENING_HOUR.scopeType/scopeId`, trois niveaux — lieu, type
  de ressource, ressource. 🔴 **Les niveaux s'INTERSECTENT** : une ressource ne peut
  que restreindre son lieu, jamais l'élargir. Un seul niveau répond ; ils ne
  s'empilent pas.
- La colonne « Effectif » de `/admin/horaires` montre la résolution.

## 7. Événements et formations

- ⚠️ **La table est `EVENEMENT`**, pas `EVENT`.
- ⚠️ `Event.category` est un **LIBELLÉ**, `Event.formation` est un **LIEN**. L'un ne
  remplace pas l'autre.
- 🔴 **Aucun code ne branche sur une catégorie précise** —
  `EventCategoryContractTest` échoue sinon.
- 🔴 **Assister ne qualifie JAMAIS.** `SessionEnrolment` écrit une progression
  *commencée* (`completed = false`, score 0) ; un formateur valide. Les deux portes
  vers une place inscrivent (inscription ET promotion depuis la liste d'attente),
  dans la même transaction que la place.
- `EventSeries` génère N séances **à la création** : **semaines uniquement**,
  chaque occurrence décalée depuis la PREMIÈRE date, affiche non recopiée, un seul
  `flush()`. Les lignes sont **indépendantes** — pas d'identifiant de série.
- **Formation = catalogue global** ; seules les sessions ont un lieu.
- La certification est **hors** du modèle de packages : c'est de la sécurité, pas
  du commerce.

## 8. Archivage

`Machine`, `Formation` et huit autres objets portent `archivedAt`.

🔴 **Masquer n'est pas refuser.** `findLive*()` retire l'objet des surfaces qui le
PROPOSENT ; `ReservationService` refuse au point de passage unique
(`MACHINE_ARCHIVED`). `/machines/{id}` répond toujours à qui a le lien.
⚠️ `findBy()` reste la question de l'**admin**. ⚠️ `countVisible()` doit porter les
mêmes conditions que `findVisible()`.
⚠️ Les packages s'archivent par leur case `active`, pas par `archivedAt` : **un
audit par nom de route ne voit pas un verbe qui est un champ.**
⚠️ **Archiver ≠ annuler** : `callOff()` prévient les inscrits ; archiver range.

## 9. Config : fail-open / fail-closed

Stores adjacents à la config = **DBAL brut, pas des entités**, et fail-safe en
lecture. La direction de l'échec est choisie et compte :

| Store | En erreur de lecture | Pourquoi |
|---|---|---|
| `BookingPolicyRepository` | **ouvert** (aucune limite) | refuser toute réservation met le labo hors ligne |
| `AccessPassRepository` | **fermé** (aucun pass) | sinon l'erreur *accorde* une exemption |
| `NotificationPreferences` | **ouvert** (on envoie) | `ReminderLog` réclame AVANT d'envoyer |
| `ReminderLog::claim()` | **fermé** (on n'envoie pas) | oublier coûte un rappel, deviner coûte une boucle |

**« Une table vide est une configuration complète et valide »** est le style de la
maison. `BOOKING_POLICY` ne sème aucune ligne ; toute colonne de limite est
nullable et `null` = pas de limite. Un enregistrement entièrement vide **supprime**
la ligne.

⚠️ `SiteSettingService::isDevelopmentMode()` ne fait qu'**afficher** la section
Développement de la nav. Jamais d'assouplissement d'accès ou d'authentification.

## 10. Mail

`App\Mail\Mailer` est le **point d'envoi unique**. Rien n'envoie autrement.

- Refuse en silence si la feature `emails` est OFF ou si aucun compte expéditeur
  n'est configuré.
- `EMAIL_LOG` est **à la fois** l'audit **et** la charge de la file — le message
  Messenger ne porte que l'id de ligne.
- Transport construit **à chaque envoi depuis le DSN en base**, pas `MAILER_DSN`.
- Locale par destinataire via `LocaleSwitcher`. Contexte stocké en JSON →
  **passer des dates ISO, jamais formatées**.
- `queueToUser()` respecte l'interrupteur maître **et** l'opt-out par catégorie.
  Le mail **transactionnel** n'a pas d'interrupteur et pas de lien de
  désabonnement.
- Rappels : `app:mail:reminders`, timer systemd horaire. Scanners sans état ;
  `MAIL_REMINDER` décide par `INSERT IGNORE`. **Réclamer avant de mettre en file.**
- ⚠️ Twig échappe le bloc `subject` → `html_entity_decode`.
- ⚠️ `strip_tags` jette le href : `MailSender` aplatit `<a>` en `label : url`
  **avant** de dépouiller.
- ⚠️ Le worker n'a **pas de requête** : les liens absolus viennent du réglage
  `public_base_url`. Non renseigné = **pas de lien**, plutôt qu'un lien cassé.
- Redémarrer `fabos-worker.service` après tout changement sous `src/Mail/`.

## 11. URLs signées

Pour ce qu'une personne doit pouvoir faire sans compte : désabonnement,
annulation invité, billets. `UriSigner` sur l'URL **absolue**, validée contre
`public_base_url` et pas contre l'en-tête `Host`.

- **GET propose, POST exécute.** Les clients mail préchargent les liens.
- **Pas d'expiration** sur le désabonnement.
- 🔴 **Une signature n'autorise pas l'action de quelqu'un d'autre** : le billet est
  signé et public, le **scan** est derrière le pare-feu `/staff`.

## 12. i18n

- **Cinq catalogues en verrou** : `fr, en, de, es, it`. `fr` est la source. Une clé
  ajoutée à un catalogue est ajoutée aux cinq.
- 🔴 **On traduit l'INTERFACE, jamais le CONTENU.** Ce que FabOS dit = interface. Ce
  que l'opérateur écrit (formation, page de labo, mentions légales, description
  d'événement) = contenu, dans une seule langue. `FormationPageContentService` est
  le patron travaillé.
- **Jamais l'organisation ni le lieu dans une chaîne** : `%org%` et `%venue%` sont
  injectés dans *toute* traduction par `VocabularyTranslator`. En Twig :
  `org_name()`, `venue_label()`. ⚠️ Ce décorateur doit rester **paresseux et
  fail-safe** (warmup, console, worker mail n'ont pas forcément de base).
- **Pluriels** : second domaine `messages+intl-icu.LOCALE.yaml`. ⚠️ Une clé ne vit
  que dans UN des deux catalogues. ⚠️ Les paramètres perdent leurs `%` :
  `{days}`, pas `%days%`. ⚠️ `#` imprime le nombre.
- **Validateurs** : `validators.LOCALE.yaml`, **la clé est la phrase française au
  caractère près**, apostrophes typographiques comprises.
- **Flashs** : `addFlash('success', 'flash.x')` ou `['flash.x', ['%p1%' => $v]]`,
  traduits au rendu par `|flash_text`. 🔴 **Une clé inconnue traverse inchangée** —
  ne pas « corriger » ça en levant une exception.
- 🔴 **`&mdash;` dans une valeur de catalogue rend le texte littéral `&mdash;`.**
  Utiliser le vrai caractère.
- ⚠️ `debug:translation` ne lit **ni** le PHP des `FormType`, **ni** les `<script>`,
  **ni** `public/js`, **ni** `{% block title %}`. Relire la page **rendue**.
- ⚠️ Un second bloc `namespace:` à la fin d'un YAML **masque** le premier
  (last-wins). Aucun lint ne le voit.

## 13. Dates et fuseaux

La boîte tourne en **UTC** ; le fuseau du labo est un **réglage**. Deux
conventions :
- **horodatage machine** → `|lab_date()` ;
- **valeur d'horloge saisie par un humain** → `|date()` simple.

Classer par **entité**, jamais par nom de champ. Se tromper est silencieux.
Épingler le fuseau à l'**écriture** (parse du formulaire) **et** à la lecture.

## 14. Front-end

- **Coquille publique** : `site/base_public.html.twig`. **Coquille admin** :
  `base.html.twig`. Ajouter une page = l'étendre, **jamais** copier un `<head>`.
  Il reste **5** gabarits à `<head>` propre : `event-ticket` + les 4 kiosques.
- **Une liste, un shell** : `_admin_list` + `_data_table` + `_cell_*`. La page
  n'apporte que ses colonnes et ses données. Format arrêté (S140/S141) : une carte,
  quatre bandes — bandeau coloré (nom de menu, compte en sous-titre, recherche, un
  bouton vert nommé), bande de filtres sur `--surface-ground`, le flash, les
  lignes. `/admin/machines` est le rendu de référence.
- **Le titre d'une liste = son entrée de menu** (`admin_nav.entry.<route>`), lu
  depuis `NavBuilder`, **jamais recopié**. ⚠️ Exception validée : `/admin/machines`
  porte sa SECTION.
- **Une seule barre latérale**, aucun bandeau pleine largeur : la bande de la carte
  porte chaque titre (S142/S143).
- **Filtres** : six filtres rapides maximum, le reste dans « Plus de filtres » et
  en chips. Tuiles à un clic pour la facette qui définit la page, listes déroulantes
  pour le reste.
- **AssetMapper + Stimulus sont LIVE. Turbo est OFF** (`enabled: false` dans
  `assets/controllers.json`). L'interaction va dans
  `assets/controllers/*_controller.js`, jamais dans un `<script>` inline.
  ⚠️ Un contrôleur ne tourne que là où `importmap('app')` est émis.
  ⚠️ Un contrôleur ne peut pas remplacer du code qui doit tourner avant le premier
  rendu (le bootstrap de thème reste inline, sinon flash de mauvais thème).
- **Formulaires** : thème de formulaire partagé + `FormType`. ⚠️ `csrf_token_id`
  **explicite**, sinon l'écran bascule sur le jeton *stateless double-submit* et
  devient invérifiable par sonde. ⚠️ Le thème rend `form_help()` — un écran qui
  portait ses hints à la main les perd si on ne les passe pas en `help`.
  ⚠️ `choice_translation_domain => false` quand les libellés sont des DONNÉES.
  🔴 Un **filtre GET** n'est pas un `FormType` : sa place est dans l'URL.
- **Couleur d'état** : `.admin-status-*` / `.admin-status-solid-*` dans `admin.css`.
  ⚠️ Le préfixe `admin-` compte : `.status-ok`/`.status-info` sont pris par
  `style.css`.
- **Jamais de couleur claire littérale.** Tokens de thème
  (`--theme-surface`, `--theme-surface-elevated`, `--theme-input`, `--color-text*`,
  `--border-color`), basculés par `html[data-theme="dark"]`.
  ⚠️ Mapper **par PROPRIÉTÉ, jamais par valeur** : `color:#fff` sur un bouton reste
  blanc, `background:#fff` sur une carte non.
- **Cibles tactiles** gardées sur `(pointer: coarse)`, jamais sur la largeur.
  44×44 minimum.
- **Affordances role-gated** : `can_reach('route')`, **jamais**
  `is_granted('ROLE_X')`. 🔴 **Il n'y a pas de hiérarchie de rôles ici** —
  `ROLE_ADMIN` n'implique pas `ROLE_STAFF`.
- `/admin/design` est la référence. Une primitive récurrente y va, et dans la
  feuille partagée. C'est la règle opérateur : *« if you need custom css it's
  probably worth making it a design guideline »*.

### Pièges CSS payés

- 🔴 **`style.css` ~3191 repeint tout `<span>` dans `.admin-panel` en gris avec
  `!important`.** Aucune spécificité ne bat ça.
- 🔴 **L'attribut `hidden` perd contre n'importe quel `display` explicite.**
- 🔴 **`admin.css` ligne 1 est `@import url("machines-list.css")`** — ce fichier
  possède `.ml-tile` et est chargé sur ~41 pages **sans apparaître dans aucun
  gabarit**. Un `?v=` ne l'atteint pas : il faut bumper les deux.
- ⚠️ **Bumper le `?v=` au DERNIER changement CSS de la session**, pas au premier.
  Vérifier par le `?v=` que la PAGE émet, jamais en curlant l'URL versionnée.
- ⚠️ Retirer une cellule d'une ligne en CSS grid oblige à corriger
  `grid-template-columns`.
- ⚠️ Une règle `.form-field*` n'est **jamais locale** : chercher qui d'autre la
  matche, et **mesurer** ces écrans-là aussi.
- ⚠️ `.admin-edit-panel` n'a pas de padding propre — il vient de `.admin-edit-form`.
  Ce qui suit `{{ form_end(form) }}` colle aux bords.
- ⚠️ `.admin-panel` est un cadre : ses enfants portent la gouttière de 24 px.
- ⚠️ `getComputedStyle` ment dans le panneau navigateur ; une feuille cross-origin
  ne se parcourt pas.

## 15. Déployer

Recette complète : `ARTEMIS_DEPLOYMENT.md`. L'essentiel :

- SSH : `ssh -i ~/.ssh/id_ovh -p 22 proxmox.lab.dryades.org`. ⚠️ **L'ancien
  `artemis.dryades.org:4002` EXPIRE au lieu de refuser** — un transfert qui pend,
  c'est ça. Compte non privilégié : `sudo pct exec 210 -- bash -lc "…"`,
  `sudo pct push`.
- App : `/opt/fabos/FabApp`. **Archives étroites seulement.** 🔴 **Jamais
  `deploy.sh`** (il fait `git pull origin main` et écrase le travail déposé à la
  main).
- **Personne ne code sur Artemis.** Comparer les hachages AVANT d'écraser : la
  boîte a déjà servi des fichiers absents du checkout local.
- 🔴 **Ordre de déploiement, pas une préférence de style :**
  - feature **DBAL fail-safe** → le code peut partir devant, il dégrade en « pas de
    lignes » ;
  - **colonne ORM mappée** → **migration D'ABORD**. Aucune dégradation possible :
    toute requête sur la table la sélectionne, la table entière fait 500.
  - Extraire **seulement** le fichier de migration, faire migrer l'opérateur,
    **puis** extraire le code.
- **Lire la sortie du lint AVANT de redémarrer.** ⚠️ Un **nouveau filtre Twig exige
  `cache:clear` AVANT `lint:twig`**. ⚠️ `php -l` vert ne dit rien du conteneur (un
  `use` manquant sur un paramètre de contrôleur = 500).
- 🔴 **`python3 tools/ctor_arity.py` — le seul contrôle qui voie un `new Foo()`
  trop court.** `php -l` vérifie la syntaxe, pas les appels : un constructeur qui
  gagne un paramètre laisse derrière lui des constructions manuelles qui ne
  planteront qu'à l'exécution. Mesuré le 2026-09-03 :
  `S158BackfillGroupsCommand` construisait `new AudienceResolver($conn)` avec UN
  argument depuis que S159g en exigeait deux — sur le chemin `--write`, donc
  **hors de portée de la sonde comme du balayage de routes**. C'est la commande
  qui VÉRIFIE le backfill : un témoin cassé, pire qu'un témoin absent.
  ⚠️ Il ignore volontairement les arguments nommés et les variadiques : un linter
  qui crie à tort n'est plus lu.
- En prod, gabarits et catalogues sont compilés : **`cache:clear` à chaque
  déploiement**. JS : `importmap:install` puis `asset-map:compile` (les dossiers
  `assets/vendor/` et `public/assets/` sont gitignorés).
- Rollback : `git archive HEAD <paths>`, pousser, extraire par-dessus, `rm -f` les
  fichiers neufs, **puis `rm -rf var/cache/prod`** (un gabarit Twig compilé survit
  à `cache:clear`).
- Redémarrer `fabos.service` ; **et `fabos-worker.service`** après du code mail.
- **Dérive de schéma assumée** : les entités sont en avance sur les migrations.
  🔴 **Jamais `doctrine:schema:update --force`** — le diff contient des DROP.
- L'agent ne peut ni migrer ni `git push` : donner la ligne de commande à
  l'opérateur.
- Cinq fichiers de CT 210 restent **volontairement modifiés localement** et ne se
  committent jamais : `.env`, `.env.local.example`, `compose.yaml`,
  `compose.override.yaml`, un SQL hérité.

## 16. Vérifier sans login

L'agent n'a pas de compte. Ce qui marche :

- Les **branches de refus** se testent anonymement (404/403/409/400).
- **`php bin/console app:render <path>`** rend n'importe quelle page depuis un
  shell, **connecté** — vraie requête, vrai noyau, vrai pare-feu. `--as=`,
  `--grep=`, `--anonymous`, `--save=`.
  🔴 **Il authentifie par construction : il ne peut JAMAIS montrer une règle
  d'accès trop permissive.** Pour prouver qu'une route refuse, la curler
  anonymement et lire le 302.
- 🔴 **`app:render` + grep prouve que le balisage existe, jamais que quelqu'un le
  voit.** Compter les **pages rendues**, pas les gabarits : une revue disait 12, le
  balayage a trouvé 27.
- **Voir les PIXELS d'une page admin sans s'authentifier** : `app:render --save`
  sur la boîte → rapatrier le HTML → réécrire les URLs d'assets **racine-relatives**
  en absolues `https://fabos.dstei.fr/…` (CSS/JS/images sont publics) → servir le
  fichier en local et l'ouvrir dans un navigateur. Thème :
  `document.documentElement.dataset.theme = 'dark'`.
  ⚠️ **Ne jamais publier ces rendus** : ils contiennent des noms et des e-mails.
  ⚠️ Retirer `main.js` et épingler `data-theme` pour la variante claire, sinon on
  teste deux fois le même thème.
- **Sonde jetable** : une commande console dans `src/Command/` qui crée ses
  fixtures, pilote la couche service, imprime un verdict, nettoie. La déployer,
  l'exécuter, **la supprimer — jamais la committer**. `app:s147:form-probe` porte
  le motif pour les POST refusés.
- **CSRF dans une sonde** : forger le jeton dans LA requête qu'on envoie ; et un
  formulaire laissé sur le jeton *stateless* par défaut est **intestable** depuis la
  console.
- **Mailpit** (`127.0.0.1:32769` sur la boîte) est l'oracle mail : lire ce qui est
  arrivé, pas le gabarit.
- ⚠️ Vider les mémos de `ScheduleResolver` par réflexion entre deux écritures ;
  recharger la page entre deux mesures d'interface.

## 17. Règles de construction

1. **Une source de métadonnées.** `FeatureWorkspaceRegistry` décrit navigation et
   scopes ; voters et services restent l'autorité métier.
2. **Une liste, un shell.**
3. **Pas de surcharge.** Lieu, tâche et filtre sont trois axes différents.
4. **URL explicable.** Défaut agrégé ; lieu, recherche, filtres et pagination
   partageables. Une préférence de profil n'est **jamais** un droit.
5. **Sécurité côté serveur.** L'UI lit le même verdict que les voters. Un lien
   caché n'autorise ni n'interdit rien.
6. **Migrations expand / backfill / contract.**
7. **Simulation avant activation.** Chaque route mutante a son voter atomique.
8. **Cinq langues, sombre, mobile, clavier.** Toute primitive est montrée avec le
   vrai composant dans `/admin/design`.
9. **Artemis est la définition de « fait ».** Doc, commit, deploy CT 210, lint,
   restart, vérification réelle.

⚠️ **Un réglage stocké que personne ne lit est pire que pas de réglage.** Ne pas
ajouter un champ avant son lecteur.
⚠️ **Les commentaires expliquent le POURQUOI**, pas le quoi.
⚠️ **Uploads** : liste blanche d'extensions, nom de fichier aléatoire, et
supprimer l'ancien fichier **seulement après** que le nouveau est enregistré.
⚠️ **Orientation d'image** : EXIF avant dimensions ; `exif_read_data()` ne lit pas
le PNG (l'orientation peut vivre dans le chunk `eXIf`) et GD ne reporte pas le tag.
Réutiliser `ImageNormalizer`.

## 18. Décisions opérateur fixées

- **Guest** = anonyme sans compte. Visibilité ≠ action. Le réglage FabOS est un
  défaut, pas un plafond : par événement `inherit`/`allow`/`deny`, séparément pour
  `view` et `register`.
- **Groupes intégrés protégés** : clés stables, non supprimables ; libellés
  configurables. User = tout compte actif, Guest = virtuel.
- **Institution** : descriptive, ou connectée après découverte sur une URL HTTPS et
  **confirmation explicite** de confiance. Changer d'origin la suspend.
- **Partage** : le personnel ne sort que si l'instance l'autorise **et** que le
  membre consent à cette donnée et cette destination.
- **Un badge n'est jamais effacé** : retrait = révocation auditée qui reste au
  journal. Un badge révoqué n'est jamais réactivé.
- **Matériaux** = catalogue partageable ; disponibilités et emplacements locaux.
- **Navigation** : « Le lieu » → « Utilisateurs » ; Horaires sous Lieux ; interface
  et accueil sous Configuration → Thèmes.
- **Admin recovery n'est pas un bypass** : ni badge requis, ni arrêt de sécurité.
- **Exposition publique par surface** : activation opérateur **et** consentement.
- **Sessions IdP** : révocation immédiate si connue, 24 h max en cas de panne.
- **« sous-lieu » s'appelle « lieu »** — *location* en anglais (S143). FR lieu, EN
  location, DE Standort, ES ubicación, IT sede. `Venue`/`VENUE`, `venue_context`,
  `?location=` n'ont pas bougé. ⚠️ La collision de mots sur `/admin/lieux` est
  **assumée** — ne pas la « corriger » sans redemander.
- **RFID physique et 2FA restent hors scope.**

## 19. Répartition

**Terra** domaine, migrations, services, voters, protocoles · **Luna**
architecture d'information, composants, listes, a11y · **Sol** revue indépendante
— et Sol ne valide jamais sa propre implémentation.

## 20. Trous de vérification connus

- 🔴 **Le chemin de réservation qui RÉUSSIT n'a jamais été vérifié par l'agent.**
  Il faut un login. Demander à l'opérateur de cliquer un vrai parcours.
- **13 formulaires** restent non vérifiés pour « un champ refusé ne fait pas
  ressaisir le reste » (J-8). Seule une soumission tranche.
- `rejectUnauthorizedDevice()` rend `null` — *autorisé* — quand
  `FABOS_RFID_API_TOKEN` est vide, et il est **non renseigné** sur la boîte.
- `TrainingEnrollment` est un **stub neutralisé**.
- Aucun compte « staff mais pas admin » pour tester les surfaces de rôle.
