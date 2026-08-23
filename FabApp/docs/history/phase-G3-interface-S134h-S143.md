## S134h–S134j — Phase G3, les listes (2026-08-11)

Le format arrêté en S130e était une maquette `dzf-` qui ne touchait aucun écran.
Cette session en fait le composant et y met toutes les listes d'administration.

**S134h — un composant, pas 41 copies.** `_admin_filters.html.twig` : une seule
rangée de tuiles pour la facette qui définit la page, puis « Affiner » en listes
déroulantes qui s'appliquent au changement, **le sous-lieu en premier**, le groupe
entier absent sur une installation à un seul sous-lieu. La recherche descend dans
l'en-tête de liste, à côté du compte, qui devient « 3 sur 11 ». Le bouton d'ajout
monte dans le bandeau, vert et nommé.

⚠️ Le vert : `--color-ok` est une couleur de **texte** — `#86efac` en sombre, pour
que « disponible » se lise sur un panneau foncé. Rempli dans un bouton, ça donne
une dalle fluo portant du blanc à 1,7:1, et c'est ce que l'opérateur a signalé.
`--color-ok-fill` / `--color-ok-on-fill` sont la paire pour le vert **surface** :
5,01:1 dans les deux thèmes, et volontairement identique d'un thème à l'autre
puisqu'il n'est jamais peint que sur le bandeau `#9E1B56`.

**S134i — six types de cellules**, chacun une classe et un partial, démontrés dans
`/admin/design#colonnes` avec les vraies machines de l'installation. `_cell_date`
**exige** sa convention (`machine` | `wall`) et n'en devine aucune : se tromper est
silencieux, plausible, et faux du décalage du lab. Sans convention, la cellule
affiche un marqueur visible plutôt qu'une date.

**S134j — douze listes remappées.** Cinq copies privées de la pastille d'état
(`.status-badge`, `.ml-state`, `.loan-status`, `.m-status`, `.pill`), deux copies
identiques de `.mat-thumb`, douze copies de la règle responsive du bandeau, deux
littéraux `#6b7280` dans des cellules.

🔴 **La prémisse chiffrée de S134j était fausse** : les « ~590 lignes de `<style>`
local » étaient les totaux de lignes des fichiers, moins quatre. Corrigé dans
`ROADMAP.md`.

**Ce qui a été trouvé en chemin, tout en production avant la session :** `.sr-only`
défini dans une feuille que deux pages chargent, donc son texte imprimé sur chaque
pastille de filtre actif de l'admin ; toutes les pastilles d'état grises en thème
sombre ; 4,09:1 et 4,08:1 en thème clair ; les comptes de tuiles calculés sur la
collection filtrée sur quatre listes ; `categoryTiles()` sans `active` ;
`machineListQuery` portant deux filtres sur cinq ; `niveau` proposé deux fois sur
les formations ; et le mot stocké interpolé en nom de classe sur les utilisateurs.

**Vérifications :** `lint:twig` (199 fichiers), `lint:yaml` (5), les 165 routes
rendues (six échecs, tous préexistants et tous des routes POST/OIDC), les hachages
des 103 fichiers comparés un à un sur CT 210, et le contraste des onze états plus
le bouton mesuré dans le navigateur : **rien sous 4,5:1 dans aucun des deux thèmes.**

## S135 — le même objet partout (2026-08-11)

Phase G3 avait mis le format et le vocabulaire en place sur douze listes. S135
les met sur **toutes**, et l'audit qui va avec a trouvé ce que « chaque page a
écrit sa propre version » coûtait vraiment.

**Sept familles de pastilles d'état** existaient en parallèle : `.status-badge`,
`.ml-state`, `.loan-status`, `.m-status`, `.status-pill`, `.pill` (deux fois,
sur deux pages, avec des valeurs différentes), `.badge-yes`/`.badge-no` (deux
fois, identiques) et `.physical-validation-status`. Toutes disent la même chose ;
aucune ne le disait pareil. Une seule reste, `_cell_state`, qui prend un
**signal** et jamais une valeur stockée.

**Six pages hors coquille** y sont entrées — les deux écrans RFID, les
inscriptions d'événement, les pages introuvables, et les lieux et
accès exceptionnels qui n'avaient jamais reçu ni panneau, ni recherche, ni
compte. `/admin/access-rfid-logs` passe de dix colonnes à cinq et
`/admin/utilisateurs/{id}` de huit à cinq : rien n'est perdu, les paires
deviennent titre + sous-titre, ce que la liste des utilisateurs faisait déjà.

🔴 **Deux bugs de cascade, dont un créé puis rattrapé dans la même session.**
`_cell_state` a été posé sur `/machines/{id}/calendrier`, une page **publique**,
qui ne charge pas `admin.css` : la pastille sortait sans aucune règle. C'est la
faute de `.sr-only` à l'identique. Le vocabulaire vit dans `components.css`
depuis, que `base.html.twig` émet partout. Et une fois stylée, elle mesurait
`rgb(212, 200, 210)` — `calendar-modern.css` porte
`html[data-theme="dark"] .calendar-stat-card span` en (0,2,1) contre (0,2,0) pour
la pastille : **une feuille de page repeignait un composant partagé.**
`:not(.status-badge)` sur ce balayage, comme dans `style.css`. Mesuré après :
7,32:1, le même objet et la même couleur qu'en administration.

🔴 **Et `machine.statut` était encore imprimé brut** sur le calendrier machine —
le mot de la base, donc `idle` en anglais sur une page française, avec la classe
tirée de ce même mot. La faute exacte que S84 avait corrigée sur
`/admin/machines`, survivante côté public.

**Vérifications :** `lint:twig` 199 fichiers, les 165 routes rendues (six échecs,
tous préexistants et tous POST/OIDC), et les hachages comparés fichier par
fichier sur CT 210.

## S134c2 / S134g / S137 — le produit arrête de mentir, et le compte appartient au membre (2026-08-12)

**S134c2 — FabOS n'invente plus le contenu d'une formation.** Une formation aux
champs vides servait à un membre un programme en quatre points, **trois sessions
à venir** rattachées à aucune donnée et impossibles à réserver, trois objectifs,
deux prérequis, trois éléments de matériel — et, trouvés en chemin, une
**description** et une **catégorie**. ⚠️ Les listes étaient *traduites*, ce qui
aggravait la chose : l'invention était courante en cinq langues. Vide veut dire
vide ; le bloc n'est plus dessiné. Les exemples restent dans
`FormationPageContentService::EXAMPLES` pour l'éditeur admin — une suggestion à
un opérateur, jamais un fait servi à un membre.

**S134g — le mot de passe oublié, de bout en bout.** 🔴 `/forgot-password` était
une **page morte** : GET seulement, aucun formulaire, aucune route POST, et la
page de connexion y renvoyait. Un membre bloqué dehors arrivait sur un écran
incapable de réinitialiser quoi que ce soit.

⚠️ **Jeton signé plutôt qu'une table**, et c'est ce qui a permis de livrer sans
migration — une migration doit être lancée à la main par l'opérateur, donc un
design qui en exige une ne peut pas partir avec les écrans qui s'en servent.
Haché (HMAC-SHA256 sur APP_SECRET), expirant (l'échéance est *dans* la charge
signée), à usage unique (la charge s'engage sur une empreinte du hash de mot de
passe courant : s'en servir change le hash, ce qui tue ce jeton **et tous les
autres** en cours pour ce compte).

Sept propriétés exercées sur CT 210 contre un compte réel : valide, correspond au
compte, expiré refusé, MAC altéré refusé, ordures refusées, mauvais secret
refusé, mort après changement de mot de passe. **Toutes PASS.**

⚠️ La réponse est la même que l'adresse existe ou non — sinon le formulaire
devient un oracle d'appartenance. Le mail part en `transactional`, pour qu'un
membre désabonné des annonces ne soit pas enfermé dehors par sa propre
préférence. Et la page « lien expiré » ne dit pas *pourquoi*.

**S137 — quatre propositions d'en-tête pour les grilles publiques**, préfixées
`dzp-`, dans `/admin/design#catalogue`. Mesurées avant la première carte :
A (aujourd'hui) 168 px, B (la bande admin) 201 px, C (une barre) 150 px,
D (recherche d'abord) 252 px. Une seule variable change d'une à l'autre.

## S139 — la recherche cherche dans tout le produit (2026-08-16)

Cinq rapports de l'opérateur, tous le même défaut : `usb` (objet prêtable),
`valentin` (projet), `D251` (espace réservable), `anniversaire` (événement
passé), les matériaux, le corps des pages personnalisées — rien ne remontait.
`/recherche` regardait **quatre** types d'objets sur dix et rendait un « aucun
résultat » propre pour les six autres. ⚠️ **Une recherche qui ne cherche pas est
indiscernable d'une recherche qui ne trouve pas** : c'est pour ça qu'elle a
tenu si longtemps, et c'est pour ça que le test qui la garde lit les
déclarations plutôt que des résultats.

**S139a — la couverture.** `SiteSearch` remplace les 75 lignes inline du
contrôleur : dix groupes au lieu de quatre, chacun derrière `allowsSurface()`.

- 🔴 **Chaque badge pointait vers `app_admin_badges`**, donc un membre qui
  cliquait tombait sur le mur de connexion. `/badges/{id}` existait depuis
  toujours.
- 🔴 **`getStatut()` était imprimé brut** dans la ligne meta d'une machine —
  `idle` en anglais sur une page française, exactement la faute que S84 puis
  S135 avaient retirée ailleurs. `getStatusKey()` désormais.
- ⚠️ Les événements passent par `findAll`, pas `findUpcoming` : « À venir » est
  une façon de **naviguer**, pas une règle de **recherche**.
- ⚠️ Les créations passent par `findPublishedForGallery` : une création non
  publiée est invisible en galerie et doit le rester en recherche.
- Les titres de groupe étaient des littéraux français servant de **clés de
  tableau** puis imprimés tels quels sur une page en cinq langues. Ce sont des
  clés `nav.*` : un résultat est classé sous l'entrée de menu qui y mène.

**S139b — les destinations.** L'opérateur a ensuite cherché `horaires`, puis
`heures`. Aucune couverture ne pouvait répondre : **aucune ligne ne s'appelle
comme ça.** Les horaires sont sept `OpeningHour` rendus dans une carte du deck
d'accueil, et personne ne tape « lundi 09:00–18:00 ». Il manquait un second
type de résultat — huit surfaces du produit, chacune avec sa liste de synonymes
traduite dans les cinq catalogues. La carte des horaires a reçu `id="horaires"`
pour qu'un résultat puisse y atterrir.

⚠️ **Correspondance par préfixe sur des synonymes entiers**, pas `str_contains`
dans les deux sens : `heure` doit atteindre `heures`, mais `re` ne doit pas
atteindre les huit destinations — une requête de deux lettres qui matche tout
enterre les vrais résultats en dessous. Un test refuse tout synonyme de moins de
trois caractères, dans les cinq langues.

🔴 **`isEnabled('bookings')` répondait toujours `true`** — le test l'a trouvé.
`bookings` n'est pas une clé du registre (les réservations sont polymorphes
depuis S8–S10 : le critère est « au moins une couche réservable »), et
`isEnabled()` **échoue en ouvert** sur une clé inconnue. La règle vivait en
privé dans `NavBuilder::featureAllows()` ; elle est maintenant
`SiteFeatureService::allowsSurface()`, que la navigation **et** la recherche
lisent. Le commentaire de `NavBuilder` disait déjà « un mot, un sens, un
endroit » — il est enfin vrai.

🔴 **Thème sombre : le diagnostic écrit en août était faux dans sa cause.** La
todo pointait vers le balayage général de `style.css`. C'était
`background: white`, écrit deux fois en clair dans le `<style>` local de
`search.html.twig` — cartes de résultat et cartes de conseil restaient des
plaques blanches sur le fond sombre. Réparé avec les tokens de surface (S83),
et **les deux thèmes vérifiés en pixels** dans le navigateur. Les trois icônes
de conseil portaient aussi le hex littéral de l'accent ; elles l'héritent.

**Renumérotation.** Phase H (commerce) S135–S139 → **S150–S154**, Phase I
(messagerie) S140–S142 → **S155–S157**. Les quatre numéros S135–S138 avaient
déjà été livrés en interface les 11 et 12 août pendant que la table les
réservait au commerce. Les numéros livrés ne bougent pas ; c'est le travail non
commencé qui se déplace. Prochain numéro libre : **S140**.

**Vérifications :** `lint:twig` 201 fichiers, `lint:yaml` 5, **38 tests / 1 607
assertions** (35/904 avant, 31/780 en S135), les 163 routes balayées (trois
échecs, tous préexistants : `/.well-known/fabos` et les deux routes legacy
`/machine/new` et `/machines/new`, signalées à l'opérateur), les hachages des 12
fichiers comparés un à un sur CT 210, et les requêtes de l'opérateur rejouées
sur le site en fr/en/de.

⚠️ **Reste de S139 :** S139c — la page porte encore 23 règles CSS locales pour
15 classes à elle. Trois de ses cinq formes ont déjà un équivalent livré
(`_cell_title`, `_cell_state`, le `frame: 'full'` de `_catalogue`) ; seules les
deux formes de « conseils » sont réellement nouvelles et méritent d'entrer dans
`/admin/design`. Documenter les cinq bénirait une copie.

## S139c / S139d — la recherche compose, les routes legacy disparaissent, un événement passé se voit (2026-08-16)

**S139c — ce qui existait déjà a été rendu, pas redessiné.** `search.html.twig`
portait 23 règles pour 15 classes à elle. Le titre et sa seconde ligne sont
`_cell_title`, la pastille de catégorie `_cell_state` en signal `muted`. Trois
règles supprimées — ⚠️ **le compte n'est pas l'intérêt** : ces trois formes ne
peuvent plus diverger du reste de FabOS.

🔴 **Le piège de cascade que ça a révélé, et qui aurait été livré sans mesure.**
`_cell_title` est un partial `_cell_*`, donc appelable de partout — mais ses
règles (`.admin-cell-user`, `.admin-cell-stack`, `.is-strong`, `.is-meta`)
vivaient dans `admin.css`, **qu'une page publique ne charge jamais**. C'est mot
pour mot la faute de S135 avec `_cell_state` sur `/machines/{id}/calendrier`.
Les quatre règles sont dans `components.css`.

🔴 **Et une deuxième, trouvée en regardant les pixels et pas le balisage.** Une
fois le composant stylé, le **titre sortait à 1,16:1** en thème sombre — presque
noir sur la carte sombre — pendant que le sous-titre à côté était correct.
`.is-meta` nomme sa couleur ; `.is-strong` la laissait à l'héritage, et ce
codebase a plusieurs balayages qui repeignent un `<span>` nu en `!important`.
**Un composant partagé ne doit pas dépendre de gagner cette course** : il nomme
sa propre couleur. Mesuré après : **13,65:1 sombre, 17,4:1 clair**.

⚠️ `getComputedStyle` a menti pendant le diagnostic — il a rendu la chaîne
entière, `body` compris, en `rgb(26,26,26)`, ce que la capture d'écran
contredisait (les titres de section et le pied étaient blancs). La capture avait
raison. Diagnostiquer sur les pixels, confirmer sur une mesure, jamais l'inverse.

Ce qui reste est documenté dans `/admin/design#recherche` : un panneau et une
carte de conseil, que rien de partagé n'exprime. ⚠️ La section dit aussi
pourquoi les trois autres formes **n'y sont pas** — un guide qui accueille
chaque variante locale devient un catalogue de dettes.

**S139c bis — toutes les routes legacy supprimées** (opérateur : « pre V1 dev
site, no need to retain legacy anything » — donc supprimer, pas rediriger).
**44 chemins**, 163 routes → 119 : tous les `.html`, tous les `_legacy`, le
`/machine` singulier, le `/calendar` anglais, les six redirections
d'administration et `/search`, doublon anglais de `/recherche`. Le formulaire de
l'en-tête pointait sur `app_search` : repointé sur `app_recherche`.
`LegacyAdminController` entier est parti, ainsi que six méthodes devenues
orphelines et trois alias morts dans `NavBuilder`.

⚠️ Déclencheur : `/machine/new` et `/machines/new` rendaient **500** — deux
routes **publiques sans attribut de sécurité** qui rendaient
`admin-machines.html.twig`, gabarit passé sur la coquille de liste en S134h/S135
et qui attend des variables qu'elles ne passaient pas. Elles n'existent plus.

**S139d — un événement passé se voit.** L'opérateur : « la mention est petite et
les inscriptions ont toujours l'air actuelles ». Mesuré avant de toucher : une
carte passée et une carte à venir étaient le **même** `<article class="ml-card">`
— affiche pleine couleur, même taille, même action — séparées par une seule
pastille grise, qu'on lit en dernier.

- `_catalogue_card` apprend `spent` : l'affiche se désature et le titre
  s'éteint. ⚠️ Pas de `filter` sur la carte entière — `annulé` doit rester rouge
  sur un événement passé, c'est un autre fait. ⚠️ La carte garde bordure, ombre
  et survol : elle reste une destination réelle ; l'éteindre dirait « cassé »
  plutôt que « fini ».
- Le pied disparaît sur une carte passée. Il disait « inscriptions fermées » —
  du **vocabulaire d'inscription**, et c'est précisément ce qui la faisait lire
  comme courante : une inscription fermée est une chose qui vient d'être
  ouverte. Un événement passé n'a pas d'histoire d'inscription. Annulé garde sa
  ligne, parce que c'est une nouvelle.
- 🔴 **Deux horloges sur la même carte.** `EventRepository` sélectionne en heure
  murale (`nowInStoredForm()`), mais le gabarit classait via
  `Event::isRegistrationOpen()`, qui compare à `new \DateTimeImmutable()` —
  l'instant **serveur**. Près de minuit, le filtre et la pastille divergeaient
  du décalage du lab. `storedNow()` est exposé, le contrôleur calcule `past`,
  le gabarit ne calcule plus rien.
- 🔴 **« 0 / 3 disponibles » au-dessus de trois cartes visibles** sur
  `?when=all`. La paire retombait sur `common.available_short` faute
  d'`available_word` — et « disponible » n'est pas un état d'événement. La clé
  `events.headline_upcoming` existait déjà et disait « à venir ».

⚠️ **Le piège Twig, une troisième fois, dans le fichier qui le documente.** Un
commentaire placé **entre deux clés d'un hash d'arguments** est une erreur de
syntaxe : « Unclosed block ». `events.html.twig` porte l'avertissement depuis
deux sessions ; la note posée à côté de `foot:` a quand même cassé la page.
`lint:twig` l'a attrapée **avant** le `cache:clear` et le restart — c'est
exactement pourquoi il passe en premier. L'avertissement couvre maintenant aussi
le hash du `{% embed %}`.

**Vérifications :** `lint:twig` 201, `lint:yaml` 5, 38 tests / 1 607 assertions,
**les 119 routes balayées — un seul non-2xx/3xx, `/.well-known/fabos` qui rend
`503 {"status":"unconfigured"}` volontairement**, les quatre chemins supprimés
confirmés en 404, contrastes mesurés dans les deux thèmes, et les hachages
comparés fichier par fichier sur CT 210.

## S139e — le fil d'Ariane n'avait aucune règle, et un événement passé le dit maintenant fort (2026-08-16)

🔴 **« Le fil d'Ariane a l'air d'être du texte brut. »** Il l'était.
`_breadcrumb.html.twig` est un partial partagé, mais ses règles vivaient dans
`details.css`, que **36 gabarits sur 201** chargent — et `/events/{id}` n'en
fait pas partie : cette page n'émet que `style.css` et `components.css`. Le
composant sortait donc **sans une seule règle**.

⚠️ **Troisième fois dans la même session, même faute :** `_cell_title` dans
`admin.css` (S139c), puis sa couleur laissée à l'héritage, puis ceci. La règle
est écrite dans `/admin/design#fil-ariane` : **les règles d'un partial partagé
vivent dans `components.css`**, la feuille que `base.html.twig` émet partout.
Une feuille que seules certaines pages chargent ne peut pas styler un composant
commun.

🔴 **Et le fil avait deux formes.** En thème sombre il portait une **pilule** —
fond, bordure, rayon, padding — que le clair n'avait pas. Un thème change des
couleurs, pas la forme d'un composant. Pilule retirée ; sombre ne redéfinit plus
que la couleur, par token au lieu du littéral `#d7c9d4`.

**Un événement passé, sur sa propre page.** S139d n'avait traité que les cartes
du catalogue ; l'opérateur regardait la **fiche**, où rien n'avait bougé. Ce
qu'elle affichait encore pour un événement terminé : un panneau « Inscription »
avec un nombre de **places restantes**, une **jauge animée**, un compteur de
liste d'attente, et un bouton d'inscription. Tout le vocabulaire de quelque
chose qu'on peut encore rejoindre.

- Le panneau devient « Participation » : le nombre de **participants**, à plat,
  sans jauge. Ce qu'une fiche d'événement passé doit au lecteur, c'est combien
  sont venus, pas combien pourraient encore venir.
- La pastille d'état de son inscription reste — c'est le fait qu'il avait une
  place — mais **le bouton « Annuler » disparaît** : on ne se retire pas d'une
  chose qui a déjà eu lieu, et un bouton qui ne peut qu'échouer est pire que pas
  de bouton.
- 🔴 **Troisième horloge.** `'registrationOpen' => $event->isRegistrationOpen()`
  comparait encore à l'instant serveur. La liste, la carte et la fiche doivent
  s'accorder sur ce qui est passé, sinon une carte dit « Terminé » et sa propre
  page propose une place.
- ⚠️ **« Événement passé » était trop petit** (opérateur). La bannière `.ev-cancelled`
  existait déjà pour les annulations : elle devient `.ev-notice` **plus un ton** —
  les noms que les pastilles de la même page utilisent déjà. **Un élément,
  plusieurs tons**, plutôt qu'une seconde classe de bannière.
- ⚠️ **Puis : « le manque de couleur la rend quelconque. »** Le ton gris était
  sémantiquement juste et visuellement muet. 🔴 **On n'a pas emprunté un feu
  tricolore pour autant** : un événement fini n'est ni une erreur (rouge), ni un
  avertissement (ambre), ni une disponibilité (vert), et se servir d'un signal
  pour attirer l'œil est exactement comment un vocabulaire de signaux cesse de
  vouloir dire quelque chose. Le système en a un quatrième qui n'est pas un
  signal : **l'accent**. `is-accent` utilise `--tone-primary-soft` (S83), donc
  une installation qui change de couleur emporte la bannière avec elle au lieu
  de garder le magenta FabOS. Mesuré : **5,51:1 sombre et 6,23:1 clair** sur le
  titre, 13:1 et 14,18:1 sur le texte.
- ⚠️ Et la phrase « cet événement a eu lieu » a quitté le panneau latéral : la
  bannière la dit déjà, et sur mobile les deux s'empilaient en une répétition.
- L'affiche est désaturée comme sur la carte (`.ev-page.is-spent`), donc un
  événement fini a la même tête dans la liste et sur sa fiche.

**Vérifications :** `lint:twig` 201, `lint:yaml` 5, 38 tests / 1 607 assertions,
rendu vérifié en pixels sur `/events/8`, et `?v=` porté à `20260816-s139e` sur
les quatre feuilles touchées — ⚠️ y compris l'`@import` de `machines-list.css`
dans `admin.css`, qu'un `?v=` sur le `<link>` n'atteint pas.

## S134g moitié 2 — le compte appartient au membre, jusqu'à sa disparition (2026-08-16)

**Décision opérateur, qui a débloqué la session :** « stats should stay,
bookings and all. If user Pierre got deleted, we should still see his activity
in stats, projects untouched, leaderboard as well. Maybe just his name gets
changed? » Donc **anonymisation, jamais suppression**. Chaque ligne survit ; la
personne s'en va.

⚠️ **Ce n'est pas un contournement du RGPD, c'est sa lecture.** L'article 17
donne un droit à l'effacement des **données personnelles** ; le considérant 26
place les informations anonymes hors du règlement. Effacer les identifiants et
garder les lignes honore la demande *et* conserve l'histoire du lab — qui n'est
plus la donnée personnelle de personne dès lors que nul ne peut dire de qui il
s'agissait.

🔴 **Tout repose donc sur l'IRRÉVERSIBILITÉ.** S'il subsiste où que ce soit une
correspondance vers la personne, c'est de la *pseudonymisation* : les lignes
restent des données personnelles et l'effacement n'a pas eu lieu. D'où :

- aucune table « comptes supprimés », aucune copie d'archive, aucune ligne
  d'audit qui les nomme ;
- l'adresse est **écrasée et non hachée** — le hachage d'une adresse connue se
  ré-identifie en testant des candidats ;
- l'avatar et la bannière sont **supprimés du disque** : une ligne qui cesse de
  nommer le fichier n'efface pas le visage qui est dedans ;
- les lignes `EXTERNAL_IDENTITY` partent, sinon la prochaine connexion OIDC
  reconstruit le compte depuis les claims du fournisseur et défait tout.

**Les satellites, chacun pour une raison qui mérite d'être dite.** `EMAIL_LOG`
garde l'adresse, le nom affiché **et** un contexte qui nomme la machine et les
horaires réservés. Une inscription faite **en invité** porte un nom et une
adresse saisis dans le formulaire, hors du compte. Et `AccessRfidLog.badgeUid`
est le numéro de la carte physique : un identifiant aussi personnel qu'une
adresse, qui survit au compte parce que le journal est une piste d'audit. Le
scan reste — c'est la statistique — le numéro non.

**Ce qui est gardé** : réservations, passages machine, emprunts, badges, points,
temps de présence, progression, votes, créations. Tout pointe vers le même id,
devenu un simple numéro de ligne qui ne désigne personne.

⚠️ **Deux entrées, un seul service.** La page du membre et l'écran d'admin
appellent le même `AccountAnonymiser`. Une seconde implémentation côté admin
serait une seconde définition d'« effacé », et celle qui dérive est celle qui
laisse des données derrière.

⚠️ **La confirmation est l'IDENTIFIANT tapé, pas le mot de passe.** Un mot de
passe exclurait quiconque se connecte par fournisseur d'identité et n'a donc pas
de mot de passe local utilisable — précisément les membres les plus susceptibles
de vouloir effacer leur copie locale. Taper son propre nom est une friction que
tout le monde peut franchir et que personne ne franchit par accident.

🔴 **L'invariant de verrouillage, testé pour de vrai.** `AccountGuard` refuse le
dernier administrateur actif, et deux cas subtils sont l'intérêt du test : un
admin **déjà anonymisé** ne compte pas comme le second (la ligne existe, un
comptage naïf dit « c'est bon », mais ce compte ne pourra plus jamais se
connecter), et un admin **suspendu** non plus. Compter des lignes plutôt que des
administrateurs utilisables *est* le verrouillage. L'effacement n'a pas d'annulation :
un opérateur qui se trompe ici n'a plus personne à qui demander.

⚠️ **Aucune migration.** Le marqueur est le domaine réservé `.invalid`
(RFC 2606), qui ne peut jamais résoudre vers une vraie boîte. Une migration doit
être lancée à la main par l'opérateur, donc un design qui en exige une ne peut
pas partir avec les écrans qui s'en servent — même raisonnement qu'au jeton
signé de la moitié 1.

⚠️ **Le nom stocké est `Anonyme #<id>`**, et c'est le seul endroit où la règle
« traduire l'interface, jamais le contenu » plie : `getDisplayName()` a **83
sites d'appel** et une entité n'a pas à porter un traducteur. Le `#id` garde deux
membres effacés distinguables dans un classement sans rien dire ni de l'un ni de
l'autre. À revoir le jour où l'affichage passera par une clé.

**Vérifications :** `lint:twig` 202, `lint:yaml` 5, **53 tests / 1 822
assertions** (46/1 813 puis 38/1 607 avant), `/profil/supprimer` rendu 200
derrière l'authentification avec ses deux listes et son champ de confirmation,
le panneau d'admin rendu sur quatre fiches, et les hachages comparés sur CT 210.

### S134g — le 500 en production, et ce qu'il a révélé (2026-08-16, même jour)

L'opérateur a essayé d'anonymiser un compte et a eu un **500**.

🔴 **`Utilisateur::setPublicFields()` prend `array`, on lui passait `null`.** La
vérification avait contrôlé que chaque setter **existait**, jamais qu'il
acceptait ce qu'on lui donnait — et les tests de contrat affirment qu'un setter
est *appelé*, ce qu'un `TypeError` traverse sans les déranger.

🔴 **La moitié la plus grave était l'ordre.** Les nettoyages en SQL brut
tournaient **avant** le point de plantage, et DBAL valide immédiatement. Une
exception à mi-parcours laissait donc `EXTERNAL_IDENTITY` supprimé et
`EMAIL_LOG` nettoyé **pendant que le compte gardait son nom** — une érasure
*partielle*, dont chaque partie est irréversible. Le compte visé n'avait aucune
ligne dans ces deux tables : rien n'a été perdu, mais c'est de la chance, pas du
design. Tout le scrub est désormais dans une transaction : **ou la personne est
effacée, ou rien ne s'est passé.**

⚠️ Un commentaire écrit le matin même défendait qu'un passage à moitié fait
était « plus facile à raisonner ». C'était justifier un ordre, pas empêcher un
problème. Il est supprimé.

⚠️ **Trouvé en corrigeant :** déplacer l'`unlink` après la transaction ne
supprime **rien**, parce que la ligne ne connaît plus le nom des fichiers. Ils
sont capturés avant, et l'`unlink` reste hors transaction — il n'a pas de
rollback, et perdre la ligne est la moins mauvaise des deux pannes.

**Trois tests ajoutés, le premier écrit après le bug qu'il aurait attrapé :**
chaque `setX(null)` de l'anonymiseur est confronté par réflexion à la vraie
signature, l'érasure doit être atomique, et les noms de fichiers doivent être lus
avant d'être effacés.

⚠️ **Vérifié autrement qu'en faisant passer des tests.** Une commande jetable a
exécuté le **vrai** scrub contre le compte réel dans une transaction, puis a
annulé : `ulpzugfv@immenseignite.info` → `anonymised-8@anonymised.invalid` /
« Anonyme #8 », sans erreur, compte intact après rollback. Commande supprimée du
conteneur ensuite. 56 tests / 1 844 assertions.

**La leçon, plus large que ce bug :** vérifier qu'une méthode existe n'est pas
vérifier qu'on peut l'appeler. Quand un service pilote des dizaines de setters
d'entité, c'est la **signature** qu'il faut confronter, et un test par réflexion
le fait sans base de données.

---

## S141 — la carte fusionnée devient LE format de liste (2026-08-16)

**Le format avait été validé sur une page et restait un paramètre.** S140 avait
fusionné les trois cartes d'`/admin/machines` en une seule, l'opérateur l'avait
retenue, et le fichier gardait quand même trois formes : le grand bandeau
pleine largeur d'origine (20 pages), `hero: 'compact'` (10 pages) et
`hero: 'merged'` (1 page). S141 supprime le drapeau, les deux autres formes et
la classe de variante `.is-merged`. Les règles s'attachent au shell lui-même,
`.admin-list-card`.

### Le titre est le nom de l'entrée de menu

Décision opérateur du 2026-08-16 : « quotas » plutôt que « gestion des
quotas ». Le mot est déjà écrit dans la barre de sous-navigation ; une page qui
le réécrit tient une deuxième copie, traduite en cinq langues et libre de
diverger. `NavBuilder::adminCurrentTitle()` renvoie
`admin_nav.entry.<route>` et le shell le lit. **`/admin/machines` est la seule
exception** — il porte le libellé de SECTION (« Équipement ») parce que c'est la
page d'atterrissage du groupe et que c'est le rendu validé à l'écran ; l'exception
est un `titleLabel` sur l'entrée, pas un `if` sur la route.

Conséquence : **41 clés mortes supprimées des cinq catalogues**, 205 lignes.
Trente et une étaient les `*.title` et `*.description` par page ; dix étaient des
`*.subtitle` publiques mortes depuis le passage aux grilles de cartes.

🔴 **`page_title`, et pas `title`.** Un `{% embed %}` sans `only` fusionne le
contexte parent, donc tant que le paramètre s'appelait `title`, n'importe quelle
variable de contrôleur du même nom devenait le titre de la page.
`/admin/machines/categories` en passe une : elle a affiché la clé brute
`machine_taxonomy.categories_title` dans le bandeau au premier essai.

### Trois affordances mortes, trouvées en regardant les pages

1. `{% block header_extra %}` n'était imprimé que dans l'ancien en-tête. Les dix
   pages passées en `compact` le perdaient en silence :
   `/admin/rfid-readers` n'affichait plus son bouton « Comment appairer un Pi ? »
   et les instructions d'appairage étaient inatteignables.
2. `sidebar_variant: 'rfid'` (puis `'user'`) rendait un `<aside>` sans aucune
   règle pour ses liens : la sidebar sortait en texte inline replié sur quatre
   pages. Les deux variantes sont supprimées ; `'edit'` reste pour S132.
3. `/admin/quotas-reservation` sans `reservableType` n'allumait aucune entrée et
   sortait donc un `<h1>` vide. Le contrôleur canonicalise l'URL.

### La revue de contenu — la moitié intéressante

🔴 **Cinq tableaux avaient plus d'en-têtes que de cellules.**
`/staff/acces-exceptionnels` (7 pour 6), `/admin/loans` (6 pour 5),
`/admin/utilisateurs/{id}` (5 pour 4), `/admin/homepage` (colonne conditionnelle
déclarée sans condition) et `/admin/access-rfid-logs`. Le motif est le même à
chaque fois : une valeur repliée dans le sous-titre de la cellule de titre, et
l'en-tête laissé derrière. **Rien ne plantait.** Le tableau dessinait chaque
colonne suivante sous le mauvais nom — les dates sous « Portée », un bouton de
révocation sous « État » — et l'en-tête en trop prenait un filet de largeur à
droite, ce qui faisait rendre « Reason » une lettre par ligne.

**`_cell_chip`** — le cas nommé par l'opérateur. `/admin/formations` imprimait le
nom du badge deux fois par ligne, dont une dans une colonne `is-tight` où il
cassait sur trois lignes. Un jeton court cliquable (`#7`), le nom complet en
`aria-label` et en infobulle ; les lignes passent de doubles à 61 px chacune.
Ses règles vont dans `components.css`, jamais `admin.css` — troisième fois que
ce piège est écrit.

🔴 **`overflow-wrap: anywhere` contre `width: 1%`.** `anywhere` laisse une
coupure douce compter dans la largeur *min-content* et `th.is-tight` vaut
`width: 1%`, donc les colonnes serrées se réduisaient à quelques caractères puis
fracassaient leurs valeurs : « 23/07/2 026 », « Salle Impres sion 3D »,
« Impri mante 3D test ». `break-word` corrige les trois d'un mot.

🔴 **`/admin/access-rfid-logs` imprimait `REQUIRED_BADGE_MISSING`**, et deux fois
— `MachineAccessService` écrit la même valeur dans `status` et dans `reason`.
`_rfid_result` porte maintenant le vocabulaire pour les deux pages qui listent
des scans, avec un repli qui **humanise** une valeur inconnue plutôt que
d'imprimer une clé : deux générations de vocabulaire cohabitent dans cette table.

### Les six squelettes d'administration deviennent un

Cinq écrans portaient un tableau sans passer par `_admin_list`, chacun avec son
propre squelette. Ils cachaient trois flashes privés à couleurs littérales, une
page entière repeinte en clair (`.admin-user-page { background: #f6f7fb; color:
#1f2937; font-family: Arial }`), deux boutons publics redéfinis localement dont
un `background: white` sur panneau sombre, et quatre `colspan` comptés à la main
et inatteignables. Le guide de style lui-même documentait un shell qu'il
n'utilisait pas ; **52 règles de maquette** y ont été supprimées avec les
propositions qu'elles imitaient.

🔴 **La mesure que `/admin/design` citait ne mesurait rien.** `paint()` cherchait
une classe que le spécimen avait cessé d'émettre, sortait tout de suite, et les
deux chiffres annoncés « mesurés dans le navigateur » affichaient un tiret
cadratin en production. Ils mesurent maintenant le haut de la carte à la première
ligne : **268 px**, et **346 px** projetés à douze catégories.

### 🔴 Onze clés supprimées par accident, et la classe entière fermée

Une regex censée retirer deux clés de `rfid_logs` a été écrite sans ancre —
`^ {4}col_status: .*$` — et a emporté **tous** les `col_status` du fichier, dans
les cinq langues. Huit listes d'administration et trois pages membres ont
affiché `admin_machines.col_status` en en-tête, entre deux exécutions vertes de
la suite de tests.

Trouvé en balayant les 139 pages rendues à la recherche d'identifiants pointés.
**Le même balayage a trouvé trois clés manquantes qui n'étaient pas de moi** :
`admin_emails.col_status` et `login.email` sur le formulaire **public** de mot de
passe oublié.

`TranslationKeyTest` ferme la classe : toute clé littérale `'x.y'|trans` d'un
gabarit doit exister dans les cinq catalogues. ⚠️ Elle ne regarde que les clés
**littérales** — une clé concaténée ne se vérifie pas sans exécuter le gabarit,
et prétendre le contraire est exactement ce qui supprime treize clés vivantes
(cf. les treize `usage_rights.verdict.*` et `notification.category.*`).

### Mesuré, pas déduit

- **31 pages du shell rendues et regardées**, clair et sombre : un bandeau, une
  carte, zéro `admin-page-header`, bords gauches alignés (carte 323, `<h1>` 348,
  facette 348), aucun débordement horizontal à 1440 px.
- `/admin/machines` : **268 px** du haut de la carte à la première ligne, contre
  430 px pour les trois cartes de S134h et 394 px pour les en-têtes à la main.
- **21 listes sondées** : zéro colonne au-delà de cinq, zéro cellule fracassée,
  zéro fait imprimé deux fois dans une même ligne.
- **139 chemins rendus** : zéro identifiant pointé hors des pages qui citent des
  clés exprès, et pour seuls non-2xx `/.well-known/fabos` 503 et
  `/desabonnement` 400, tous deux voulus.
- `?v=20260816-s141` vérifié par ce que les pages **émettent** : 106 pour
  `style.css`, 106 pour `components.css`, 61 pour `admin.css`.
- **64 tests / 2 096 assertions.**

⚠️ **Reste ouvert :** `/admin/homepage` porte six colonnes (bloc + quatre
audiences + ordre). C'est une matrice d'audiences, pas une liste
d'enregistrements, et le plafond de cinq ne lui répond pas ; le test ne le voit
pas non plus, ses colonnes venant d'une variable. À trancher si la question
revient.

---

## S142 — une seule barre latérale, et le CSS des partagés remonte (2026-08-16)

**La dernière variante privée de barre latérale.** S141 avait supprimé `'rfid'`
et `'user'` parce qu'elles étaient cassées à l'écran ; `'edit'` restait, sur les
**27 gabarits de formulaire** qui ne passent pas par `_admin_list`. Elle n'était
pas cassée — elle était *différente* : les mêmes sections, sans icônes, dans un
`<aside class="admin-edit-nav">` avec ses propres règles. Ouvrir `/admin/machines`
puis la fiche d'édition d'une machine changeait la forme de la navigation sans
qu'aucun lecteur puisse dire pourquoi.

`_admin_sidebar.html.twig` n'a plus de paramètre du tout : ni la table `shells`,
ni `admin_sidebar_variant`, ni le passe-plat `sidebar_variant` de `_admin_list`.
Les règles `.admin-edit-nav` sortent d'`admin.css`, et les sélecteurs morts
`.admin-user-nav` / `.admin-rfid-nav` — restés dans **quatorze** listes `:is()`
de `style.css` après S141 — sortent aussi.

### Les partials partagés qui portaient leur propre CSS

C'est le sous-ensemble qui casse des pages, et il en a cassé une :

🔴 **`.btn-danger` n'avait aucune règle qu'une page d'admin puisse atteindre.**
Elle est définie dans `login-register.css`, qu'aucune page d'admin ne charge, et
c'est la classe du bouton de confirmation de
`_delete_confirm_modal.html.twig` — partial partagé par `/admin/rfid-readers` et
le formulaire de lecteur. Sur la liste, le seul bouton destructeur de la boîte de
dialogue s'affichait donc en chrome nu, indiscernable d'« Annuler ». Le
formulaire s'en sortait en définissant la classe en privé. Elle est dans
`admin.css` maintenant, avec un token neuf : **`--color-stop-fill`**, parce que
`--color-stop` est une couleur de TEXTE qui vaut `#fca5a5` en sombre — remplir un
bouton avec elle est exactement la faute que le bouton vert vert avait livrée le
2026-08-11.

Trois autres partials partagés portaient des règles qu'aucune feuille ne
connaissait :

- **`_admin_edit_styles.html.twig`** (20 règles, 5 pages) — un doublon d'`admin.css`
  à ceci près qu'il repeignait l'en-tête en `#9E1B56` **plat** là où les 22 autres
  formulaires ont le dégradé. Supprimé.
- **`_delete_confirm_modal.html.twig`** — 15 règles émises dans le `<body>`, une
  fois par inclusion. Dans `admin.css`.
- **`_rfid_pairing_modal.html.twig`** — aucun style, et ses **deux** appelants
  tenaient chacun une copie identique des mêmes neuf règles `.modal*`. Un
  troisième appelant aurait obtenu une boîte de dialogue sans boîte de dialogue.

`_header.html.twig` garde le sien : c'est la couleur d'accent de l'instance,
une donnée, pas une règle.

### Ce qui remonte dans `admin.css`

`select` manquait à `.form-field input, textarea` — présent dans **quatorze**
blocs locaux, donc bordé sur les pages qui y avaient pensé et natif sur les
autres. Avec lui : `.form-help`, `.form-field.checkbox`, `.empty-state`,
`.form-warning`, `.current-asset`, `.badge-checkbox*`, `.reader-help`, le
`.choice-grid` qui remplace `.material-machines-choices` et `.batch-machines`
(mêmes déclarations, deux noms), et **ce que le thème de formulaire Symfony rend
réellement** : `fieldset`, les paires radio+label d'un `ChoiceType` étendu et le
`<small>` de `form_help()`, qui n'avaient de règles que dans les deux gabarits
Événements. Les radios prennent au passage `accent-color: #9E1B56` partout, au
lieu d'être roses sur deux pages et bleu-navigateur ailleurs.

⚠️ **Plusieurs de ces classes étaient déjà habillées pour le SOMBRE** dans les
blankets de `style.css` et pour le clair nulle part : la règle claire vivait dans
le `<style>` d'une page, recopiée entre le jumeau `-new` et le jumeau `-edit`.
Une troisième page aurait reçu une peau sombre par-dessus rien.

**Trois hauteurs de `textarea` — 110, 120, 130 — pour la même question.** Les six
copies sont parties ; la centrale (130) sert. `admin-lab-page` garde 180 : c'est
l'éditeur de corps de page, la seule hauteur qui ait été choisie.

### Mesuré

- **1 118 règles locales sur 65 gabarits → 950 sur 47.** ⚠️ Compté avec un
  script qui n'accepte que les `<style>` réels ; ne pas soustraire des chiffres
  des sessions précédentes, qui comptaient autrement.
- Sur les 27 formulaires seuls : **180 règles sur 22 gabarits → 32 sur 5**.
- Les cinq qui restent le méritent : la médiathèque de `lab-page-edit`, le QR et
  l'affiche d'`event-edit`, le bloc `.env` du formulaire de lecteur, la hauteur
  de l'éditeur de page, et un `padding-top` d'alignement.

⚠️ **Le piège de la journée, pour la prochaine session.** Le premier passage a
supprimé les blocs avec `<style>.*?</style>` en DOTALL. L'en-tête de commentaire
de ces gabarits contient la phrase « every one of these grew a `<style>` » : la
regex a donc mangé la fin du commentaire, le `{% block title %}` et les liens de
feuilles de style de **19 fichiers**. Rattrapé par une vérification structurelle
avant tout commit — chaque gabarit doit encore contenir `{% block title %}`,
`{% block stylesheets %}` et `css/admin.css`. **Ancrer sur le début de ligne**
(`^[ \t]*<style>`), jamais sur le mot.

---

## S142c/d + S138c — une seule forme de page dans tout l'admin (2026-08-16)

**S138c d'abord, décidé en une ligne par l'opérateur : « oui pour full ».**
`frame: 'full'` était opt-in le temps que `/machines` serve de terrain d'essai.
Les **neuf** appelants le passaient tous, donc la branche non-encadrée était du
code inatteignable décrivant une page que personne ne pouvait voir. Le paramètre
part, avec `{% block filters %}` (qui ne s'affichait que là, et qu'aucun appelant
ne définissait) et trois règles CSS mortes. **Aucun pixel ne change.**

### Le bandeau que l'opérateur a pris pour un bug

*« dans les pages d'admin il y a un petit bandeau de couleur avant le submenu qui
semble être un bug »*. Ce n'en était pas un : c'était l'ancien en-tête pleine
largeur, encore sur **32 pages** — 25 formulaires, Réglages, Fonctionnalités,
wizard, setup, tableau de bord, et les deux pages Créations. Les 35 listes
l'avaient perdu en S141, leur titre étant passé dans la carte. C'est le
**contraste** qui le faisait lire comme un défaut : la bande apparaissait et
disparaissait selon qu'on regardait une liste ou l'un de ses propres
enregistrements.

Mesuré à 1440 px avant/après sur `/admin/places/2/edit` : le chrome coloré avant
le sous-menu passe de **167 px à 24 px**, la bande de titre de **144 px pleine
largeur à 85 px dans la carte**.

31 pages converties. La bande vit dans **un** fichier,
`_admin_form_head.html.twig`, inclus par 26 d'entre elles. ⚠️ Ce n'est
délibérément **pas** un shell : ces pages ne s'accordent que sur la bande et
divergent sur tout ce qui est en dessous, donc chacune ouvre encore son `<main>`,
sa grille et sa carte. Elle doit donner **trois** classes à sa carte —
`.admin-panel`, `.admin-list-card`, `.admin-form-card`.

⚠️ **Pas de `{% block %}` dans un partial inclus.** Un bloc dans un
`{% include %}` ne peut pas être surchargé par l'appelant : il rendrait vide en
ayant l'air d'un point d'extension. La seule page à deux contrôles — le
formulaire de lecteur RFID — écrit sa propre bande avec les mêmes classes.

⚠️ **Le tableau de bord garde son `.admin-header`** : ce n'est pas une barre de
titre redondante mais une carte d'accueil avec avatar et raccourcis. Non tranché.

### 🔴 Et le vrai défaut, dessous : 24 px du mauvais fond, partout

L'opérateur a regardé la nouvelle forme et vu qu'il restait *« le petit défaut en
haut du sous-menu, le fond est de couleur différente »*. Il était sur **toutes**
les pages d'admin, listes comprises, depuis l'introduction des fonds.

`.admin-page` peint le fond creusé ; son premier enfant — `.admin-layout`,
`.admin-edit-layout` — porte `margin: 24px auto 48px`. Sans padding ni bordure en
haut de `.admin-page`, cette marge **s'échappe par margin collapsing**. Mesuré :
`<main>` commençait à y=172 quand l'en-tête du site finissait à y=148, et
`elementFromPoint` sur la bande intermédiaire renvoyait `BODY` — 24 px du dégradé
radial du body au-dessus du fond d'admin.

**`display: flow-root` sur `.admin-page`.** Un contexte de formatage de bloc,
c'est-à-dire exactement « contiens les marges de tes enfants », et rien d'autre.
Mesuré après : `<main>` commence à 148, le sous-menu reste à 172, aucune
géométrie ne bouge. ⚠️ Pas `padding-top: 24px` + `margin-top: 0` sur les grilles :
elles servent aussi hors `.admin-page` et il faudrait tenir la paire synchronisée
sur cinq classes. ⚠️ `.ml-page` n'en a pas besoin — `.ml-wrap` s'espace au
padding.

**La leçon :** un fond qui ne commence pas où le lecteur croit qu'il commence est
invisible à la lecture du Twig, invisible au lint, invisible aux tests, et
parfaitement visible à l'écran. C'est l'opérateur qui l'a vu, deux fois de suite,
en regardant le site tourner.

---

## S143 — « lieu », et le dernier bandeau (2026-08-16)

### Le mot

« Sous-lieu » devient **« lieu »** — *location*, *Standort*, *ubicación*, *sede*.
Le mot manquait depuis le 2026-08-11 ; l'opérateur l'a choisi le 16. Renommage de
**catalogue**, pas de schéma : `Venue`/`VENUE`, `venue_context`, `?location=` et
les noms de routes ne bougent pas — la route était d'ailleurs déjà `/admin/lieux`.
155 chaînes dans les cinq fichiers de messages, 91 occurrences dans les
commentaires de `templates/` et `src/` — un code qui appelle la chose de deux
façons est exactement la confusion qu'on corrige — et 15 phrases écrites en dur
qui disaient encore « sous-lieu » à l'écran sur les trois pages de référence.

⚠️ **Une inquiétude annoncée et démentie par la mesure.** J'avais prévenu que les
aides `venues.help.venue` en ES et IT décrivaient le lieu **parent** avec le mot
vers lequel l'enfant était renommé, et qu'il faudrait les réécrire. Faux : cette
clé décrit le champ lui-même — « la sede física a la que pertenece » **est** le
lieu. Une seule correction de cohérence en espagnol. **Vérifier une inquiétude
avant de la transmettre comme un fait.**

✅ **La collision est réelle, elle a été montrée, et l'opérateur la garde**
(*« laissons comme cela pour l'instant, je n'ai pas mieux »*). La section de menu
s'appelait déjà « Lieux », donc le mot paraît quatre fois sur `/admin/lieux` :
barre latérale, libellé du sous-menu, un de ses deux liens, titre de la carte.
⚠️ Ne pas « corriger » ça dans une session future sans le redemander : les trois
sorties — laisser, renommer la section, distinguer l'entrée — ont été posées et
aucune n'était meilleure.

### Le dernier bandeau, et les cinq familles qui le dessinaient

L'opérateur, après S142d : *« admin_header? »*. Le `.admin-header` du tableau de
bord n'était pas une barre de titre — il portait l'accueil **et les sept tuiles
de statistiques**, posées sur le dégradé magenta, au-dessus de tout le reste.
C'est une carte maintenant, avec la même bande `_admin_form_head` que les
vingt-six formulaires.

Ce que ça a emporté :

- **Le faux avatar** : `<div class="admin-avatar">AD</div>`, deux lettres en dur,
  jamais les initiales de personne.
- **Vingt couleurs littérales.** Les sept tuiles portaient leur teinte **deux
  fois** dans le markup — un `style="background: rgba(…, .1)"` en ligne et un
  `stroke="#hex"` sur le SVG. La teinte est une classe, l'icône hérite par
  `currentColor`, le fond est dérivé au `color-mix` : le sombre n'a plus besoin
  d'une seule règle.
- **`background: white`** sur la tuile, qui n'était juste que parce qu'un blanket
  `!important` la repeignait ensuite.

🔴 **Les cinq familles de bandeaux pleine largeur sont supprimées** —
`.admin-header`, `.admin-page-header`, `.admin-edit-header`, `.admin-rfid-header`,
`.admin-user-header` : le même slab magenta écrit cinq fois, unifié en S85, rendu
inutile en S142d, sans un seul appelant après le tableau de bord. Grep avant
suppression : 0 markup pour les cinq et leurs `-inner`, `h1`, `p`. ⚠️ **Ne pas
réintroduire un bandeau pleine largeur pour une page** — c'est ainsi qu'il y en a
eu cinq.

### Et la faute que le port a introduite, attrapée aux pixels

Passer les tuiles aux tokens a donné à celle d'Utilisateurs
`color: var(--color-primary)` : le magenta de marque **en premier plan** mesure
environ **1,97:1** sur un panneau sombre — exactement le piège que les liens de
logs RFID et les cellules métriques avaient déjà fallu relever.
`--color-primary-text` vaut l'accent en clair et un mélange à 50 % de blanc en
sombre, donc une déclaration répond aux deux thèmes. Mesuré après, sur
`rgb(43,35,53)` : primary **5,76** · stop 7,93 · info 8,35 · warn 10,44 · ok
10,72. **Une migration littéral→token n'est pas finie tant que le contraste n'a
pas été remesuré** : le token est correct par construction pour un fond, pas pour
un premier plan.

---

