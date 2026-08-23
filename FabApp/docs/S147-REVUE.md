# S147 — la revue de la Phase J (mesure, aucun code)

**Passée le 2026-08-22.** Méthode : `app:render` sur CT 210, **146 pages réellement
rendues** (105 sans paramètre + 41 pages de détail atteintes par un lien réel),
puis mesure du HTML rendu. ⚠️ Cette passe lit le **rendu**, pas l'écran : ce
qu'elle ne peut pas voir est listé en fin de page et reste à faire.

⚠️ Les chiffres du tableau « L'état mesuré au 2026-08-21 » de `ROADMAP.md` sont
corrigés ci-dessous — deux d'entre eux comptaient des **commentaires Twig**.

## Correction de la ligne de base

| Chiffre annoncé | Mesuré le 2026-08-22 | Pourquoi l'écart |
|---|---|---|
| 48 gabarits avec leur propre `<head>` | **5** — `event-ticket` + les 4 kiosques | Les 43 autres « `<head>` » sont **dans un `{# … #}`** qui raconte que S78 l'a retiré. La coquille est déjà partout ailleurs. |
| 783 règles CSS locales dans 82 gabarits | **897 règles dans 43 gabarits** | Idem : 39 des 82 ne mentionnent `<style>` que dans un commentaire. Le volume, lui, est un peu plus gros qu'annoncé. |
| 3 026 clés × 5 langues, 0 manquante | **3 074 clés, 0 manquante**, 1 clé en trop en ES | — |

🔴 **Conséquence directe** : le critère de sortie « ramener les 48 gabarits à `<head>`
propre sur la coquille » est **déjà atteint**, sauf pour les 4 kiosques et le billet —
et ceux-là sont exactement l'exception que le critère prévoyait d'écrire. Cette ligne
sort de la Phase J.

---

# La liste, ordonnée

## J-1 🔴 La prod est cassée sur l'upload d'images — et ce n'est pas de la finition

CT 210 est **en retard de 4 fichiers** sur le dépôt :

| Fichier | Écart |
|---|---|
| `src/Image/ImageNormalizer.php` | **197 lignes absentes de la boîte**, dont `outputExtension()` et `CREATION_EDGE` |
| `public/css/calendar-leaderboard.css` | **505 lignes absentes**, dont tout le bloc `.creation-form-page` |
| `src/Form/CreationAdminType.php` | la boîte est restée à 3 Mo, sans message d'erreur de taille |
| `src/Form/CreationUserType.php` | idem |

🔴 `SiteController.php:3091` et `AdminController.php:4410` — **identiques des deux côtés** —
appellent `$images->outputExtension()`. La méthode n'existe pas sur la boîte : **tout
envoi d'image de création fatal en 500 sur le site en ligne**, côté public comme côté
admin. Ce n'est pas un défaut de dessin, c'est un déploiement partiel jamais rattrapé.

⚠️ Accessoirement, l'arbre déployé porte **~40 fichiers AppleDouble `._*`** laissés par
un vieux `tar` macOS (`templates/site/._admin-machines.html.twig`, `public/css/._admin.css`…).
Inertes, mais ils faussent tout inventaire fait sur la boîte.

## J-2 🔴 Huit objets se suppriment encore en dur (c'est S134f, jamais fait)

`->remove()` vérifié dans le contrôleur, pas déduit d'un nom de route :

`place` · `event` · `material` · `institution` · `lab-page` · `maintenance` ·
`rfid-reader` · `creation`

⚠️ **Contre-vérification faite, et elle change le travail** : `loanable-item` a l'air
d'être le neuvième — sa route est `/admin/loanable-items/{id}/delete`. Elle **archive**
(`AdminController:3712`, méthode `archiveLoanableItem`, avec refus si l'objet est encore
sorti). Le défaut ici est le **nom de la route**, pas le verbe : à renommer, pas à
reconstruire. Même piège que les packages qui s'archivent par leur case `active`.

Entités qui portent déjà `archivedAt` : `Badge`, `EventCategory`, `Formation`,
`LoanableItem`, `Machine`, `MachineCategory`. `Venue` passe par `active`. Les huit
ci-dessus n'ont **rien** : chacun demande une migration expand.

## J-3 🔴 37 messages flash ne sont toujours pas traduits — 27 sur `/profil`

S134b en a catalogué 154 ; il en reste **37 écrits en dur**, et la page la plus
membre-visible du site en porte **27** :

> « La mise a jour de l avatar a ete refusee. » · « L image ne doit pas depasser 2 Mo. »
> · « Banniere de profil supprimee. » · « Preferences mises a jour. » · « Theme invalide. »

⚠️ Ils sont **aussi désaccentués** — donc même un francophone lit du texte abîmé.
10 autres sont dans `AdminController`. Répartition : 217 flashs sont des clés, 37 non.

## J-4 🟡 Les pluriels sont dodgés par « (s) » — 76 fois en français

| Langue | Occurrences de `(s)` |
|---|---|
| fr | **76** |
| en | 66 |
| es | 64 |
| de | 1 |
| it | 0 |

Et **2 règles ICU `plural` en tout** sur 3 074 clés. Vu à l'écran sur `/events/10` :
« Dans 11 jour(s) ». Les cinq catalogues sont complets mais ne disent pas la même
chose : DE et IT ont écrit un vrai pluriel, FR/EN/ES non. Ce n'est pas une clé
manquante, c'est une clé qui ment.

## J-5 🟡 Le CSS local, par PAGE RENDUE (pas par gabarit)

`/admin/design` (41) est le guide et ne compte pas.

| Page | Règles |
|---|---|
| `/formations/{id}/suivi` | **125** |
| `/events/{id}` | **107** |
| `/machines/{id}/historique` | **62** |
| `/admin` et `/admin/dashboard` | 40 |
| `/forgot-password` | 36 |
| `/roadmap`, `/roadmap/brief`, `/roadmap/droits-usage`, `/roadmap/historique` | 36 chacune |
| `/personnes/{id}/reserver` | 34 |
| `/kiosk/events` | 32 |
| `/mes-disponibilites` · `/kiosk/machine/{id}` | 28 |
| `/recherche` | 20 |
| `/machines/{id}` | 19 |
| `/kiosk/entries` | 17 |
| `/maintenance` | 16 |
| `/admin/utilisateurs/{id}` | 14 |
| `/admin/wizard` · `/lab` | 12 |
| `/admin/rfid-readers/*` · `/kiosk/stats` · `/badges/{id}` | 11 |
| `/admin/setup` · `/admin/events/{id}/edit` · `/admin/lab-pages/{id}/edit` | 10 |
| 10 pages restantes | 1 à 9 |

🔴 **Les trois pires sont des pages MEMBRE, pas des pages admin.** Le socle (S148) est
déjà propre à ce titre ; la dette est côté feature (S149). Les quatre pages `/roadmap`
partagent le même bloc de 36 règles — une seule correction les traite toutes.

## J-6 🟡 `/admin/utilisateurs/{id}` : 78 attributs `style=""` sur une seule page

Plus un tableau de **69 lignes** sans pagination visible. Ensuite :
`/creations/ranking` 16 · `/leaderboard` 13 · `/kiosk/entries` 12 · `/profil/supprimer` 7.

## J-7 🟡 155 emoji bruts servent d'icônes dans 26 gabarits

Alors que `_icon.html.twig` existe. 🔴 **Prouvé à l'écran** : sur `/events/10`, le `🗓`
de la puce de date rend un **tofu** (carré vide) — la police de la boîte ne couvre pas
U+1F5D3. Les pires : `admin-design` 67 (le guide, à discuter), `event-detail` 17,
`event-cancel` 8, `machine-detail` 8, `unsubscribe` 7, `staff-scan` 7.

## J-8 🟡 15 formulaires font ressaisir la saisie quand un champ est refusé

Point 8 violé **par construction** : handler POST écrit à la main, `addFlash('error')`
puis `redirectToRoute()`. Tout ce qui était tapé part avec la redirection.

| Route | Champs lus à la main |
|---|---|
| `/admin/usage-rights/{id}/edit` | **36** |
| `/profil` | 12 |
| `/admin/formations/{id}/general` | 11 |
| `/admin/settings` | 10 |
| `/admin/wizard` · `/admin/emails` | 9 |
| `/staff/acces-exceptionnels` | 7 |
| `/admin/evenements/categories` · `/admin/machines/categories` · `/personnes/{id}/demander` | 5 |
| 5 autres | 3 à 4 |

À l'inverse, **30 actions passent par un Form Symfony** et repeuplent seules. La
correction est connue et déjà appliquée ailleurs dans la base.
⚠️ Déduit du code. **À prouver par un vrai POST refusé** avant de chiffrer S148.

## J-9 🟡 Trois maquettes S103 sont encore en production

`/admin/design/workspaces` · `/admin/design/structure` · `/admin/design/droits-quotas`
portent des titres **codés en dur et en français** — « Maquette S103 — aucun enforcement »,
« Matrice cible S103 » — dans une interface traduite en cinq langues. Et
`/admin/design/workspaces` affiche des **clés de permission brutes comme texte**
(`badges.update`, `events.register`, `configuration.settings.update`…).

Même famille : `/admin/missing-pages` a son `panel_title` en dur (« Adresses demandées »),
et `/admin/horaires` laisse fuir la clé `common.apply`.

**Décision demandée** : ces trois pages ont-elles encore un rôle, ou sortent-elles ?

## J-10 🟡 Les formulaires les plus lourds (point 9 — à arbitrer, pas à trancher seul)

| Page | Champs rendus |
|---|---|
| `/admin/usage-rights/{id}/edit` | **168** |
| `/admin/formations/{id}/content` | 51 |
| `/admin/horaires` | 44 |
| `/admin/evenements/categories` · `/admin/machines/categories` · `/admin/quotas-reservation` | 42 |
| `/admin/homepage` | 37 |
| `/admin/settings` · `/profil` | 31 |
| `/personnes/{id}/reserver` | 30 |
| `/admin/machines/new` et `/edit` | 28 |

⚠️ Un gros compte n'est pas un défaut en soi — `/admin/horaires` a 7 jours × plusieurs
plages, c'est une grille, pas un questionnaire. Ce tableau sert à **choisir où regarder**,
pas à condamner.

---

# Ce qui est PROPRE — mesuré, pas supposé

- **0 lien vers une route inexistante** sur les 146 pages rendues. Le point 5 « pas de
  lien vers une page qui 404 » passe intégralement.
- **Coquille partagée** : les 141 pages hors kiosques portent toutes header, footer et
  `importmap('app')`. Stimulus tourne partout où il doit tourner.
- **Point 3** : aucune liste ne dépasse 5 colonnes, sauf `/admin/homepage` (6 — déjà
  consigné comme une matrice d'audiences, non tranché) et `/admin/design` (le guide).
  En-têtes = cellules sur **toutes** les listes ; aucun `colspan` compté à la main.
- **Aucune affordance désactivée sans explication** : les 2 de `/admin/lieux` portent
  leur `title` (« le lieu par défaut ne s'archive pas »), celle de `/machines/{id}/quiz`
  est « précédent » sur la première question.
- **Redirections** : `/admin/modules` et `/admin/capabilities` → `/admin/features`,
  `/admin/portals*` → `/admin/design/structure`, `/admin/reservations` et
  `/admin/quotas-reservation` → elles-mêmes avec `?reservableType=machine`. Toutes
  légitimes, aucune boucle.
- **Catalogues** : 3 074 clés, 0 manquante sur 5 langues.

# Ce que cette passe n'a PAS pu mesurer

Elle lit le HTML rendu. Il reste, et c'est la moitié du mandat :

1. **le nombre de clics** par parcours, avant et après (point 7) ;
2. **l'évidence du chemin** — trouve-t-on l'information sans l'avoir apprise ;
3. **sombre, mobile, clavier** vérifiés à l'écran (point 6) ;
4. **le POST refusé** qui garde la saisie — J-8 est déduit du code, il faut la sonde ;
5. **`/admin/design` montre-t-il chaque primitive utilisée** (point 10).

⚠️ Rappel de la base : `app:render` + grep prouve que le balisage existe, **jamais**
que quelqu'un le voit. Les points 1 à 3 ci-dessus demandent un vrai navigateur.

# Ordre proposé pour S148 / S149

**Hors phase, tout de suite** : J-1 (la prod est cassée).
**S148 — le socle** : J-9, J-8 (settings, wizard, emails), J-6, la part socle de J-5
(`/admin`, `/admin/wizard`, `/admin/setup`) — plus le tableau de bord « qui doit
re-briller », déjà affecté à S148.
**S149 — feature par feature** : J-2 et J-3 en premier (ils touchent des données et un
membre), puis J-5 côté feature (`formation-suivi`, `event-detail`, `machine-historique`),
J-7, J-4.

---

# Deuxième passe — au navigateur (2026-08-22)

La première passe lisait le HTML rendu. Celle-ci mesure l'écran : géométrie réelle à
375, 768 et 1280 px, cascade lue dans `document.styleSheets`, et **un vrai POST refusé**.
⚠️ Chaque alerte a été **réfutée avant d'être comptée** — une sonde naïve de contraste a
produit 39 « échecs » dont 38 étaient faux (texte blanc sur un dégradé que la sonde ne
sait pas lire). Ce qui suit est ce qui a survécu.

## J-11 🔴 `/machines/{id}` est inutilisable sur un téléphone — ✅ corrigé

Trois règles se battent dans **le même fichier** :

| Ligne | Règle | Effet voulu |
|---|---|---|
| `details.css:21` | `.machine-detail-header { grid-template-columns: 400px 1fr }` | la base |
| `details.css:1152` | `@media (max-width:1024px) { … : 1fr }` | **la correction mobile** |
| `details.css:1488` | `.machine-detail-header { … : minmax(280px,420px) minmax(0,1fr) }` | ajoutée après, **sans media query** |

🔴 Même sélecteur, même spécificité, plus bas dans le fichier : **la règle de la ligne
1488 gagne à toutes les largeurs** et la correction mobile n'a jamais rien fait.

Mesuré à 375 px : les pistes calculées sont **`280px 0px`**, le `<h1>` fait **0 px de
large**, et **13 éléments sont entièrement hors écran** — dont les trois boutons
(le principal va de `left: 337` à `right: 517`). La même règle pose `overflow: hidden`,
donc la page **ne défile même pas** jusqu'à eux : `scrollWidth === clientWidth === 375`.
À 758 px les pistes valent `420px 202px` — la photo reçoit deux fois la largeur de la
machine.

✅ **Corrigé le 2026-08-22** : la liste de pistes vit désormais dans la règle de base, en
tête de fichier, donc **en amont** des media queries ; la règle tardive ne garde que sa
peau. Vérifié à 375 px : une seule colonne de 293 px, le `<h1>` à 293 px, **les trois
boutons de 41 à 334**, 0 élément inatteignable. Vérifié à 1270 px : `420px 656px`,
identique à avant — aucune régression sur le bureau.

## J-12 🔴 La barre d'outils du calendrier déborde sur téléphone — ✅ corrigé

🔴 **Correction de ce constat, 2026-08-22.** La première rédaction disait « 168 éléments
dépassent, dont 60 `agenda-slot-cell` et 39 `slot-book-affordance` — les cibles
cliquables ». **C'était faux, et c'est exactement le faux positif que cette base
documente** : compter les éléments dont le `right` dépasse le cadre attrape tout ce qui
est *défilé hors vue* dans un conteneur qui défile très bien. `.calendar-scroll-area`
est en `overflow-x: auto` (307 px de cadre, 1120 px de grille) : **les créneaux étaient
atteignables depuis toujours**, par un défilement horizontal.

La mesure honnête compte, pour chaque élément hors cadre, s'il possède **un ancêtre
défilant**. Elle donne **5 éléments réellement inatteignables**, tous dans la barre
d'outils : la barre elle-même, deux boutons `calendar-nav-step` (semaine précédente et
suivante), la puce de semaine et son libellé.

**La cause, et c'est la jumelle de J-11** : `.calendar-nav-bar` était redéclarée en fin
de fichier, **sans media query**, avec `flex-wrap: nowrap` — ce qui battait le
`flex-wrap: wrap` de la règle de base à toutes les largeurs. Et `flex: 0 0 auto` avec le
`min-width: auto` par défaut d'un élément flex forme une paire **qui ne peut pas
rétrécir** : la barre se dimensionnait sur son contenu (731 px mesurés) dans un en-tête
de 309 px, et sortait de la carte.

✅ **Corrigé** : `nowrap` et `flex: 0 0 auto` sont passés sous `@media (min-width: 641px)`,
et la règle de base autorise le rétrécissement (`flex: 1 1 auto; min-width: 0`).
Vérifié à 375 px : barre à 309 px dans un cadre de 309, **0 élément inatteignable**.

✅ **Contre-mesure conservée** : `/machines` et `/events/{id}` donnent **0** élément hors
cadre à 375 px.

⚠️ Reste ouvert : le titre du panneau est « **Reserver** cet espace » — sans accent, et
écrit deux fois. Part avec J-3.

## J-13 🔴 « Réserver une machine » mène au seul calendrier qui ne réserve pas — ✅ corrigé

`NavBuilder.php:65` pointe `cal.book_machine` sur `app_calendar`. Or
`SiteController:314` passe **`booking: false`** à cette page, quand `/machines/{id}` et
`/places/{id}` passent `true`.

🔴 **S146 avait vu la moitié du problème** : le commentaire au-dessus explique que le
groupe a été renommé « Au programme » parce que « Calendrier » *« envoyait les gens sur
un écran qui ne peut pas »* réserver — mais **la destination de l'enfant n'a jamais été
changée**. Son voisin `cal.book_space` pointe correctement sur `app_places`.
C'est une ligne, et c'est l'entrée la plus utilisée du menu.

✅ **Corrigé le 2026-08-22** : `cal.book_machine` pointe sur `app_machines`. Vérifié en
ligne — « Réserver une machine » → `/machines`, « Réserver un espace » → `/places`, et le
groupe « Au programme » garde `/calendrier`, ce qui est juste : le groupe dit ce qui se
passe, pas « réserve quelque chose ».

## J-14 🟡 Aucun lien d'évitement, et un focus invisible

- **Pas de « aller au contenu »** : **10 éléments tabulables avant `<main>`** sur chaque
  page (44 au total sur une fiche machine). Un utilisateur au clavier retraverse tout
  l'en-tête à chaque navigation.
- **1 défaut de focus sur 10 candidats** : `.home-machine-card-link:hover, :focus
  { outline: none }` **ne remplace rien** — les cartes machine de l'accueil sont
  focalisables et ne montrent rien. ⚠️ Les 9 autres `outline: none` substituent toutes
  un `box-shadow` visible : elles ne comptent pas.

## J-15 🟡 Des plaques claires en thème sombre

**37 règles** peignent un fond littéral clair sans contrepartie sombre. Vérifié à
l'écran : `.access-status.cannot-access` (`rgb(255,235,238)`) est une carte blanc-rose
posée dans la page machine sombre ; `/formations/{id}/suivi` en porte deux autres.

⚠️ **Réfuté pendant la mesure** : la barre de nav, le bandeau d'annonce et les puces
paraissaient tous en échec pour une sonde de contraste naïve — ils sont corrects, posés
sur des dégradés que la sonde ne sait pas composer. Seules comptent les règles à
`background-color` littéral sans surcharge sombre.

## J-16 🔴 `/formations/{id}/suivi` sert un panneau de débogage au public

La carte « Informations formation » imprime, pour un visiteur non connecté :

> ID : 1 · Titre : … · Description : … · **Image : printer-3d** · Badge associé : Maker 3D

`Image : printer-3d` est le slug interne du fichier. À côté : un suivi de progression qui
affiche « 0/10 étape(s) validée(s) · 0/4 section(s) · 0/6 quiz » et « Aucune progression
enregistrée pour **votre compte** » à quelqu'un qui n'a pas de compte, plus la pondération
interne (« formation 60 % · quiz 40 % »). **10 « (s) » sur cette seule page.**

## J-17 🟡 `/machines/{id}` dit « Connexion requise » quatre fois

La pastille d'état, la phrase au-dessus, **le libellé du bouton principal** et la
bannière rouge en dessous disent la même chose.

- 🔴 Le bouton principal est étiqueté avec un **état**, pas une action, et il pointe sur
  `/login` **sans URL de retour** : le visiteur qui clique perd sa place.
- 🔴 **« Ajouter aux favoris » est une affordance morte pour un visiteur** : bouton
  pleine taille, non désactivé ; le POST répond **401**
  `{"error":"Connectez-vous pour utiliser les favoris."}` — et c'est une chaîne
  française en dur de plus, celle-là dans l'API.

## J-18 🟡 `/admin/maintenance/batch` n'est atteignable par aucun lien

« Bulk maintenance » — un formulaire Symfony qui planifie une tâche sur plusieurs
machines d'un coup — est référencé par **0 gabarit**. Il ne reste que sa route. Même
famille qu'`app_machine_ical`, resté orphelin une phase entière. Soit `/admin/maintenance`
lui donne un lien, soit il sort.

## J-19 🟡 L'entrée « Loans » de la barre latérale ouvre le catalogue d'objets

Elle pointe sur `/admin/loanable-items`, pas sur `/admin/loans`. Prêter un objet coûte
**3 clics** (Loans → Loans → New) là où toutes les autres créations en coûtent 2.

---

## Le nombre de clics — mesuré sur le graphe de liens réel

Méthode : parcours en largeur sur les liens et les `action=` des **146 pages rendues**.

| Parcours | Clics |
|---|---|
| Admin — créer machine, espace, événement, formation, badge, lieu, page, projet, package, utilisateur | **2** |
| Admin — créer institution, prêt, tâche de maintenance, matériau, lecteur RFID ; éditer l'accueil | **3** (un niveau sous une liste parente) |
| Admin — fermer le labo un jour férié (`/admin/horaires`) | 2 pour atteindre l'écran |
| Visiteur — n'importe quelle entrée du menu public | **1** (méga-menu) |
| Membre — réserver une machine : `/machines` → la fiche → un créneau ouvert → confirmer | **4** |

⚠️ Le raccourci « Réserver une machine » du menu **allonge** ce parcours au lieu de le
raccourcir (J-13). La barre latérale admin porte 14 sections, ce qui est ce qui garde
presque tout à 2 clics.

## Point 8 — prouvé, plus déduit

`php bin/console app:s147:form-probe` (`src/Command/S147FormProbeCommand.php`,
transaction annulée) : `/profil`, formulaire « profil public », **jeton CSRF valide**,
un seul champ invalide (le slug), et une biographie tapée dans le même POST.

| Mesure | Résultat |
|---|---|
| statut | **302** vers `/profil` |
| biographie tapée présente dans la réponse | **NON** |
| biographie présente après avoir suivi la redirection | **NON** |

🔴 Le garde-fou du slug rend la main **avant** que `publicBio` soit lu
(`SiteController`, branche `public_profile`). Tout ce qui a été tapé est perdu.

⚠️ **Le témoin n'a PAS pu être exécuté.** `/admin/machines/new` utilise le jeton CSRF
*stateless double-submit*, dont le cookie n'est jamais posé sur une requête construite
depuis la console. « Les 30 autres actions repeuplent toutes seules » reste donc une
affirmation **structurelle**, pas une mesure.

## Ce qui reste non mesuré après cette passe

- les écrans **admin** en sombre et en mobile : ils exigent une session navigateur que
  cette session n'a pas (et minter une session reste, à raison, bloqué) ;
- les **cinq langues** à l'écran — les catalogues sont complets, le rendu ne l'a pas été ;
- `/admin/design` contient-il chaque primitive utilisée (point 10).

---

# Addendum — « un package peut-il dire : les étudiants, les imprimantes 3D, le jeudi après-midi ? »

**Question de l'opérateur, 2026-08-22.** Réponse courte : **le moteur sait le dire, il
est migré et il est appliqué — mais l'écran ne le sait pas, et la façon de l'écrire
n'est pas celle qu'on croit.** Vérifié dans le code et sur la base de la boîte, pas
déduit de la feuille de route (qui, elle, ne mentionne pas cette capacité dans son
inventaire des manques — l'inventaire est en retard sur S144b).

## Ce qui existe déjà

S144b a livré **deux axes** que la liste « ce qu'un package ne sait toujours pas dire »
n'a jamais rattrapée :

| Axe | Où | État |
|---|---|---|
| **Ressource** — un type, une machine précise, ou une catégorie | `USAGE_PACKAGE_GRANT.reservableType / reservableId / categoryLabel` | migré, appliqué |
| **Heure de semaine** — N plages hebdomadaires par grant | table `USAGE_GRANT_WINDOW` (`dayOfWeek`, `startMinute`, `endMinute`) | migré le 2026-08-19 (`Version20260817100000`), appliqué |

L'écran existe : `/admin/usage-rights/{id}/edit` porte, par grant, un sélecteur de jour
et deux champs horaires (`14:00`–`18:00` par défaut), avec ajout et retrait de plages.

🔴 **Et c'est une COUVERTURE, pas un chevauchement.** `GrantWindowSet::covers()` exige
que *chaque minute* de la réservation tombe dans l'union des plages du jour — un package
« jeudi après-midi » n'ouvre donc pas un jeudi soir qui déborde. Le filtre est en PHP
délibérément : l'union est inexprimable dans un `WHERE` ligne à ligne, et en SQL ça
serait devenu un test de chevauchement, exactement la faute que la classe existe pour
refuser.

Le point de passage passe bien les quatre dimensions :
`ReservationService:374` → `allowsReservableDuring($user, $type, $start, $end, $venueId,
$id, categoryLabel)`.

## Donc oui — mais il faut RESTREINDRE le grant existant, pas en ajouter un

🔴 **La règle de fer du modèle : les grants s'additionnent en OU, et aucun grant ne
retire un droit.** Ajouter à côté un grant « imprimantes 3D, jeudi 14–18 » n'enferme
personne dans le jeudi : il **élargit**. « Seulement le jeudi après-midi » s'obtient en
**resserrant le grant machines de ce package**, pas en lui en ajoutant un.

⚠️ **Et l'état actuel de la base rend le piège certain** (relevé le 2026-08-22) :

| Mesure | Valeur |
|---|---|
| Packages | 2 |
| Grants | 21 |
| Grants **sans aucune portée** de ressource (`reservableType`, `reservableId`, `categoryLabel` tous NULL) | **21 sur 21** |
| Plages horaires enregistrées | **0** |

Les deux grants `machines / use` sont aujourd'hui des blancs-seings. Tant qu'ils le
restent, toute plage ajoutée ailleurs sera sans effet visible : le blanc-seing couvre
déjà toute la semaine.

## J-20 🔴 Le calendrier ne sait rien des plages — la restriction n'arrive qu'au refus

`SiteController:409-410`, dans `buildCalendarResourceAccess()` :

```php
$machineRight = $usageRights->verdict($member, 'machines');   // ni portée, ni intervalle
$placeRight   = $usageRights->verdict($member, 'places');
```

Un **seul booléen pour la ressource entière**, calculé hors du temps. Conséquence avec
un package « jeudi après-midi » : le calendrier affiche **tous** les créneaux ouverts de
la semaine comme réservables, le membre clique lundi 10:00, remplit le panneau, et se
fait refuser à la validation par `USAGE_RIGHTS_DENIED`.

Le moteur a raison, la surface ment. C'est la même famille que les affordances mortes de
J-17 : un contrôle qui ne peut pas aboutir. ⚠️ `verdict()` accepte déjà un `UsageScope`
daté — c'est l'appelant qui n'en passe pas.

## J-21 🟡 La catégorie d'un grant est comparée par LIBELLÉ exact

`UsageGrantRepository:89` : `g.categoryLabel = :categoryLabel`. Renommer une catégorie
machine **décroche silencieusement** tous les grants qui la nommaient — le grant cesse
de couvrir, personne n'est prévenu, et le symptôme est un refus de réservation sans
cause visible. Même famille que la règle déjà écrite pour les événements : *le slug est
la clé, jamais le libellé* — sauf qu'ici il n'y a pas de slug.

## Ce qu'il faudrait pour que la réponse soit un « oui » franc

1. **J-20** : passer l'intervalle et la ressource au verdict du calendrier, pour que les
   créneaux hors plage se dessinent fermés au lieu d'être refusés à la fin.
2. **J-21** : porter le grant sur l'identité de la catégorie, pas sur son libellé.
3. **Le dire à l'écran** : `/admin/usage-rights/{id}/edit` sait ajouter une plage mais
   n'explique nulle part qu'une plage **n'enferme personne** tant que le grant large
   existe à côté. C'est la phrase qui manque, et c'est elle qui fera perdre une heure au
   premier opérateur qui essaie.

---

# J-22 🔴 Les formulaires d'administration — 25 sur 52 ignorent le guide

**Demande opérateur, 2026-08-22** : *« have you done a pass on bulky, not design
guidelines following forms, i think there are a few in admin to start »*. Il y en a
plus que quelques-uns.

Le guide existe et il est écrit : `templates/form/admin_theme.html.twig`. Il raconte
lui-même son origine — vingt pages écrivaient le même enrobage de champ **116 fois**.
La forme conforme est : un `FormType` en PHP, `{% form_theme form
'form/admin_theme.html.twig' %}`, puis `{{ form_row(form.x) }}`.

| Gabarits admin/staff portant un formulaire | **52** |
|---|---|
| **Conformes** (thème appliqué, `form_row`) | **17** |
| **Construits à la main** (`<input>` bruts, aucun thème) | **25** |
| Ni l'un ni l'autre (souches de suppression, filtres de liste) | 10 |

## Les 25, par nombre de champs écrits à la main

| Gabarit | Champs bruts | Règles CSS locales | Lignes |
|---|---|---|---|
| `admin-formation-content` | **35** | 0 | 242 |
| `admin-usage-package-form` | **28** | 0 | 313 |
| `admin-settings` | **15** | 0 | 240 |
| `admin-emails` | **15** | 0 | 268 |
| `admin-opening-hours` | 12 | 6 | 206 |
| `admin-formation-quiz-form` | 10 | 0 | 215 |
| `staff-access-passes` · `admin-formation-section-form` | 7 | 0 | 135 · 78 |
| `admin-wizard` | 6 | 12 | 116 |
| `admin-homepage` · `admin-network` | 5 | 5 · 0 | 80 · 8 |
| `admin-themes` | 4 | 0 | 26 |
| `admin-utilisateur-detail` · `admin-machine-categories` · `admin-event-categories` · `_admin_filters` | 3 | 14 · 0 · 0 · 0 | — |
| `admin-features` · `admin-reporting` | 2 | 0 | — |
| `admin-booking-policies` · `admin-event-registrations` · `admin-loans` · `_admin_list` | 1 | — | — |

⚠️ **`admin-design` (10 champs bruts) ne compte pas** : c'est le guide, il montre les
primitives.

## 🔴 Ce n'est pas un défaut de plus, c'est le même que J-8

La liste des 25 recouvre presque exactement les **15 handlers POST écrits à la main**
de J-8 : `admin-settings`, `admin-emails`, `admin-wizard`, `admin-formation-content`,
`admin-usage-package-form`, `staff-access-passes`, les deux écrans de catégories,
`admin-event-registrations`. **Un formulaire construit à la main est un formulaire qui
lit `$request->request->get()`, donc qui redirige quand il refuse, donc qui fait tout
ressaisir.** Les convertir en `FormType` + thème répare les deux d'un coup :
la conformité visuelle *et* le point 8.

C'est aussi ce qui rend J-22 chiffrable comme travail : passer un écran au thème n'est
pas un travail de CSS, c'est écrire le `FormType` qui manquait.

## L'ordre à l'intérieur de J-22

1. **`admin-settings` et `admin-emails`** (15 champs chacun) — le socle, S148, et les
   deux écrans que tout opérateur touche à l'installation.
2. **`admin-usage-package-form`** (28) — c'est aussi l'écran de J-10 à 168 champs rendus
   et celui de J-8 à 36 champs lus à la main. Le pire des trois angles à la fois.
3. **`admin-formation-content`** (35) — le plus gros, mais c'est une feature (S149).
4. **`admin-opening-hours`** (12 + 6 règles locales) et **`admin-wizard`** (6 + 12).
5. Le reste, à trois champs et moins, se traite en passant avec son écran.

---

# J-23 ⚪ `/admin/usage-rights/shadow` a fini son travail — décision demandée

L'écran a été construit en S133b pour **une** raison : montrer, membre par membre, ce
que grants v2 déciderait *avant* que S134 ne l'allume, et surtout lister ceux qui
**perdraient** un accès le jour de la bascule. Son commentaire le dit : *« cette page ne
décide rien et c'est tout l'intérêt »*.

**Cette bascule est faite.** Les quatre chokepoints sont sur v2 et l'écran lit
0 perdraient / 0 gagneraient. Il reste donc deux choses dans la page, de valeur très
inégale :

- ✅ **L'audit « qui perdrait un accès »** garde une vraie valeur, et elle est
  permanente : c'est la seule surface qui répond « si je resserre ce grant, qui casse ? »
  — exactement la question que J-20/J-21 rendent urgente dès qu'un package portera une
  plage horaire.
- ⚪ **Le bouton qui déplace un chokepoint** (POST `moveChokepoint`) ne peut plus que
  **défaire** la migration : `setUsageRightsV2Active($capability, false)` renvoie une
  capacité au modèle v1. C'est un levier de migration laissé branché après la migration.

**Décision demandée** : garder la page en la recadrant comme *« simulation d'impact »*
et retirer le levier de bascule, ou la sortir entièrement ? ⚠️ Si elle sort, l'audit
sort avec elle, et il n'existe nulle part ailleurs.

---

# 🔴 J-25 — SUR LE SITE EN LIGNE, AUCUN MEMBRE NE PEUT RÉSERVER UNE MACHINE

**Trouvé le 2026-08-22 en essayant de prouver J-20.** Ce n'est pas de la finition et
ce n'est pas une décision que je peux prendre.

## Ce qui est mesuré

| Mesure (sonde en lecture, transaction annulée) | Valeur |
|---|---|
| Comptes | **9** |
| Packages actifs | 2 |
| Attributions non révoquées (`USAGE_RIGHT_ASSIGNMENT`) | **2** |
| Comptes NON-admin détenant un grant `machines / use` | **0** |
| Verdict `machines` pour un non-admin | **`denied / missing_package`** — les 3 comptes testés |
| `allowsReservableDuring()` — le point de passage lui-même | **REFUSED** |
| Enforcement | **ON** |

🔴 **La dernière ligne est celle qui compte** : ce n'est pas l'écran d'aperçu, c'est la
fonction que `ReservationService:374` appelle avant d'écrire une réservation. Un membre
qui clique un créneau aujourd'hui reçoit `USAGE_RIGHTS_DENIED`.

## Pourquoi l'écran d'ombre ne l'a pas dit

`ROADMAP.md` cite « 0 perdraient / 0 gagneraient / 12 identiques / **24 recovery
admin** » et c'est cette lecture qui a autorisé la bascule. Les 24 lignes de recovery
admin sont des comptes qui **contournent** le modèle : sur 9 comptes, la majorité sont
admins, et ils sont d'accord avec eux-mêmes quoi qu'il arrive. Les non-admins, eux, sont
refusés — et ils sont trop peu nombreux dans l'échantillon pour peser sur un total qui
mélange les deux populations.

⚠️ **La leçon est celle que cette base a déjà écrite deux fois** : un compte identique ne
prouve pas une égalité. Le chiffre rassurant portait sur la mauvaise population.

## Ce que je n'ai pas fait, et pourquoi

Je n'ai **rien attribué à personne**. Ouvrir l'accès à des membres est une décision
d'exploitation, pas une correction de revue. Trois sorties existent, elles sont
différentes, et c'est à l'opérateur de choisir :

1. **Attribuer un package** aux membres qui doivent pouvoir réserver — l'état visé,
   mais il faut décider qui et quoi.
2. **Ramener le chokepoint `machines` sur v1** le temps de le faire, avec le levier de
   `/admin/usage-rights/shadow`. 🔴 **Et c'est exactement le levier que J-23 proposait de
   retirer comme « inutile après la migration » : il ne l'est pas.** J-23 est amendé —
   la page garde son levier.
3. **Décider que c'est voulu** : un labo où l'on ne réserve pas sans package attribué.
   Alors il ne manque qu'un message honnête, et `missing_package` en est déjà un.

## Les quatre chokepoints, pas seulement les machines

Vérifié, et ils ne diffèrent pas :

| Chokepoint | Verdict d'un non-admin |
|---|---|
| `machines` | **DENIED / missing_package** — 3 comptes sur 3 |
| `places` | **DENIED / missing_package** — 3 sur 3 |
| `person_booking` | **DENIED / missing_package** — 3 sur 3 |
| `events` | **DENIED / missing_package** — 3 sur 3 |

Les quatre réglages `usage_rights_v2_*` valent 1. Donc **un membre ne peut aujourd'hui
ni réserver une machine, ni réserver un espace, ni réserver une personne, ni s'inscrire
à un événement** par le chemin qui passe par le point de contrôle.

---

# J-24 🟡 Les 69 messages de validation ne parlent que français

`debug:translation` sur la boîte, après J-3 : le domaine **`messages` est à 0 clé
manquante dans les cinq langues** — mais le domaine **`validators` en compte 69**, tous
des phrases françaises écrites en dur dans les contraintes des `FormType` :

> « Le titre ne doit pas dépasser {{ limit }} caractères. » · « Les tags ne doivent pas
> dépasser {{ limit }} caractères. » · « L’email n’est pas valide. »

Donc une interface traduite en cinq langues dont **chaque erreur de formulaire** répond
en français. C'est la moitié manquante de J-3 : les flashs sont catalogués, les
validateurs ne le sont pas.

⚠️ Deux vraies clés manquent aussi, celles-là dans `messages` :
`venues.error.timezone_required` et `venues.error.timezone_invalid`.

---

# J-4 — corrections du constat, et le mécanisme est posé

⚠️ **Deux chiffres de la première passe étaient faux.**

1. « 2 règles ICU `plural` sur 3 074 clés » : il y en avait **zéro**. Mon grep
   comptait deux clés *nommées* `unit_plural`. Il n'existait aucune machinerie de
   pluriel, et c'est pour ça que 76 clés françaises écrivaient « jour(s) ».
2. Le domaine des catalogues ne pouvait pas décliner : `messages.LOCALE.yaml`
   passe par le translator ordinaire.

## Ce qui est livré

Un **second domaine**, `messages+intl-icu.LOCALE.yaml`, dans les cinq langues. Le
suffixe fait passer le domaine par `MessageFormatter` — extension `intl` vérifiée
présente sur la boîte — qui applique les vraies règles de chaque langue.

Quatre clés, choisies parce que ce sont celles qu'un membre voit le plus, servent
d'exemple travaillé et de preuve que la chaîne fonctionne :

| Clé | Avant | Après |
|---|---|---|
| `event.hero.in_days` | « Dans 11 jour(s) » | « Dans 11 jours » / « Dans 1 jour » |
| `machine.concurrent_reservations` | « 2 réservation(s) simultanée(s) » | « 2 réservations simultanées » |
| `events.foot_seats_left` | « 3 place(s) restante(s) » | « 3 places restantes » |
| `search.results_count_for` | « 5 résultat(s) pour … » | « 5 résultats pour … » |

Vérifié sur la page rendue : `/machines/1` affiche « 2 concurrent bookings ».

## Les trois pièges, écrits dans l'en-tête du fichier

- ⚠️ **Une clé ne vit que dans UN des deux catalogues.** Celle qui reste aussi
  dans `messages.LOCALE.yaml` gagne, et le pluriel ne s'applique jamais.
- ⚠️ **Les paramètres perdent leurs `%`** : `{days}`, pas `%days%` — donc l'appel
  Twig change avec, `{'days': n}`. Un appelant non mis à jour affiche le motif ICU
  brut.
- ⚠️ **`#` dans un motif ICU imprime le nombre**, ce n'est pas un commentaire.

## Ce qui reste

**72 « (s) » en français**, et l'équivalent en anglais et espagnol. Le motif est
posé et le fichier explique comment l'appliquer ; c'est désormais un travail
répétitif sans risque d'architecture. ⚠️ Chaque clé déplacée demande **aussi** de
corriger son appelant — c'est là que se trouve le vrai coût, pas dans la
traduction.

---

# ✅ J-24 — les 69 messages de validation parlent les cinq langues

Cinq fichiers `validators.LOCALE.yaml`, 64 messages de contrainte plus les 5
vraies clés `venues.error.*`.

🔴 **La clé est la phrase française elle-même**, et ce n'est pas un raccourci :
Symfony traduit un message de contrainte via le domaine `validators` en prenant
le message pour clé. C'est la forme de migration documentée, c'est exactement
ainsi que `debug:translation` les listait déjà comme manquantes, et **aucune
ligne de PHP n'a eu à changer** — les 49 contraintes gardent leur littéral.

⚠️ **La clé doit être le message au caractère près**, apostrophes typographiques
comprises (`L’email`, pas `L'email`) : une clé qui diffère d'un signe ne
correspond pas et la phrase française traverse en l'état.

🅿️ L'étape suivante — remplacer les littéraux par de vraies clés dans les 49
contraintes — est du travail de PHP, pas de traduction, et il peut se faire
écran par écran sans toucher à ces fichiers.

## Et le résultat qui compte

```
fr missing: 0    en missing: 0    de missing: 0    es missing: 0    it missing: 0
```

`debug:translation --only-missing`, tous domaines confondus, sur la boîte.
**Les cinq langues sont complètes pour la première fois** — `messages`,
`messages+intl-icu` et `validators`.

---

# État de la Phase J au 2026-08-22, mesuré

| Défaut | Constat de la revue | Aujourd'hui |
|---|---|---|
| **J-3** flashs en dur | 37 | ✅ **0** |
| **J-24** validateurs | 69 manquants | ✅ **0** — et **0 clé manquante dans les cinq langues, tous domaines** |
| **J-15** plaques claires en sombre | 101 | ✅ **0** (+ 36 premiers plans appariés) |
| **J-6** `style=` sur `/admin/utilisateurs/{id}` | 78 | ✅ **1** (une propriété personnalisée qui porte un nombre) |
| **J-5** règles CSS locales | 897 dans 43 gabarits | **589 dans 39** — les trois pires pages membre sont sorties |
| **J-7** emoji bruts | « 155 dans 26 » → réel **74 dans 22** | **70 dans 18**, dont 29 sur `admin-design` (le guide) |
| **J-4** « (s) » en français | 76 lignes | **72 lignes**, et le mécanisme ICU existe pour les traiter |
| **J-11 · J-12 · J-13 · J-16 · J-17 · J-18 · J-19 · J-20 · J-9 · J-14** | — | ✅ livrés et vérifiés à l'écran |

## Ce qui reste, et pourquoi ça n'a pas été fait

### 🔴 J-25 — décision opérateur, en cours de son côté

### ✅ J-2 — fait le 2026-08-22 (migration passée par l'opérateur)

Les huit objets s'archivent. `findBy()` reste la question de l'admin, les surfaces
qui proposent lisent une variante `findLive*()` — le partage de
`MachineRepository::findLive()`, pas un filtre posé au hasard.

| Vérifié par sonde, transaction annulée | vu par un membre | vu par l'admin |
|---|---|---|
| Place | **2 → 1** | 2 → 2 |
| Event | **3 → 2** | 6 → 6 |
| Material | **1 → 0** | 1 → 1 |

🔴 Deux cas où archiver n'est pas ce qui existait : une **création archivée garde
ses fichiers** (l'ancienne action les effaçait du disque, une restauration
n'aurait rendu qu'une carte vide), et **archiver un événement n'est pas
l'annuler** — `callOff()` prévient les inscrits, archiver range.

⚠️ Le piège de la sonde, à ne pas repayer : le premier essai archivait
`findOneBy([])` et concluait « Event 3→3, le filtre ne marche pas ». Il marchait —
la ligne tirée était un événement PASSÉ, que la requête publique ne montrait déjà
pas. **Une sonde qui archive quelque chose d'invisible ne mesure rien.**

### J-21 — la colonne existe, le code ne l'utilise pas encore

`USAGE_PACKAGE_GRANT.categoryId` est en base depuis la migration. Ce qui reste est
du code : l'éditeur de grants doit l'écrire, et `UsageGrantRepository` doit le lire
à la place de — puis en plus de — `categoryLabel`, avant qu'un backfill vérifié ne
permette de retirer le libellé.

🔴 **L'ordre est l'inverse de celui de S144b, et c'est écrit dans le fichier.**
S144b pouvait livrer son code d'abord parce que son lecteur sonde les colonnes.
Ici il n'y a pas de sonde : dès qu'une entité déclare `archivedAt`, Doctrine la
met dans chaque SELECT, et une colonne absente est un 500 sur toutes les listes
concernées — `LOANABLE_ITEM.archivedAt` est déjà parti comme ça et a fait tomber
deux pages. **Migration d'abord, code ensuite.**

```
ssh -i ~/.ssh/id_ovh -p 22 proxmox.lab.dryades.org 'sudo pct exec 210 -- bash -lc "cd /opt/fabos/FabApp && php bin/console doctrine:migrations:migrate --no-interaction"'
```

⚠️ **Et le vrai coût de J-2 n'est pas la colonne, c'est la promesse** : S134f exige
qu'archiver une ressource réservable **annule explicitement ses réservations à
venir**. `PLACE` est réservable ; `EVENEMENT` porte des inscriptions et, depuis
S146e, des progressions. Le code qui bascule le drapeau doit s'en charger.

### J-22 / J-8 — les 25 formulaires écrits à la main : **délibérément non commencé**

C'est le plus gros reste et le plus risqué. Convertir `admin-settings` et
`admin-emails` en `FormType` veut dire réécrire deux écrans que l'opérateur touche
à chaque installation, **et je ne peux pas les tester par un POST** : ils utilisent
le jeton CSRF *stateless double-submit*, dont le cookie n'est jamais posé sur une
requête construite depuis la console (démontré en essayant, § point 8).

Commencer une refonte à deux écrans, sans pouvoir en exercer la soumission, à la
fin d'une longue session, est exactement la manière dont cette base s'est déjà
cassée. Le chemin est écrit dans J-22 ; il demande une session à lui, avec un
navigateur connecté pour vérifier chaque envoi.

---

# 🔴 J-8 — correction : le compte de 15 n'a jamais été vérifié

**2026-08-23.** J-8 disait « 15 handlers font ressaisir la saisie ». Ce chiffre
venait d'un détecteur statique — *un handler écrit à la main qui fait
`addFlash('error')` puis `redirectToRoute()`* — et **il est faux**.

## Les deux cas réellement soumis, et ils se contredisent

| Écran | POST envoyé | Résultat |
|---|---|---|
| `/profil`, formulaire « profil public » | slug invalide + une biographie tapée | 🔴 **la biographie est perdue** |
| `/admin/settings`, section « localisation » | fuseau horaire inexistant + langue changée | ✅ **la langue survit** |

`/admin/settings` **enregistre champ par champ** et ne refuse que celui qui est
invalide ; la redirection arrive *après* les écritures. Rediriger après avoir
enregistré n'est pas le défaut — rediriger *avant d'avoir lu* en est un, et c'est
ce que fait la branche « profil public », dont le garde-fou du slug rend la main
avant que `publicBio` soit lu.

## Pourquoi le détecteur se trompe, et pourquoi aucun ne marchera

Il cherche le premier `addFlash('error')` suivi d'un `return redirect` à moins de
cinq lignes. Sur `/admin/settings`, le premier flash d'erreur ne redirige pas : il
pose `$refused = true` et **continue**, et la redirection trouvée appartient à la
fin de la méthode. ⚠️ La distinction qui compte — *l'écriture a-t-elle eu lieu
avant le refus ?* — dépend du flot d'exécution, pas de la proximité textuelle.
**Aucune lecture ne tranche ; seule une soumission tranche.**

## Ce que J-8 vaut réellement

- **1 cas prouvé fautif** : `/profil`, branche « profil public ».
- **1 cas prouvé sain** : `/admin/settings`, section « localisation ».
- **13 non vérifiés.** Ils restent des candidats, pas des défauts.

⚠️ **Et c'est pour ça que la conversion en masse n'a pas eu lieu.** Réécrire
`/admin/settings` et `/admin/emails` en `FormType` se serait justifiée par un
chiffre que je viens de démentir — précisément la faute que la feuille de route
consigne : *« un chiffre inventé a cadré une session entière »*.

## Comment trancher les treize, écran par écran

`app:s147:form-probe` porte désormais le motif : une session partagée pour le GET
et le POST, un champ volontairement invalide, un champ voisin modifié, et on
regarde si le voisin a survécu. Trente lignes par écran, et la réponse est un
fait.

⚠️ Deux pièges déjà payés dans cette sonde : `/admin/settings` est **une section
par formulaire** — envoyer un nom de section inconnu tombe dans « section
inconnue », n'enregistre rien, et donne l'air que tout est perdu. Et une section
n'a de sens que si elle contient **deux** champs dont l'un peut être refusé.

## J-22 reste, et il est indépendant

**25 formulaires admin sur 52 n'utilisent pas le thème** — ça, c'est mesuré et
vrai, et ça ne dépend pas de J-8. Mais la justification « ça répare aussi le
point 8 » ne tient plus telle quelle : elle tient pour les écrans où le défaut
est prouvé, et il faut le prouver d'abord.

---

# ✅ J-22 — `/admin/settings` et `/admin/emails` sont au thème (2026-08-23)

Huit formulaires convertis : cinq cartes de réglages (`src/Form/Settings/`) et
trois d'e-mails (`src/Form/Emails/`). **19 formulaires admin conformes sur 42**,
contre 17 sur 42 au moment de la revue.

## Ce que la conversion a vraiment changé

| Écran | POST envoyé | Avant | Après |
|---|---|---|---|
| `/admin/settings`, localisation | langue changée + fuseau inexistant | message en haut de page, le champ revient à sa valeur enregistrée | **200 re-rendu**, l'erreur est sur le champ, la langue choisie est encore là |
| `/admin/emails`, rappels | case cochée + délai de 9999 h | `getInt()` acceptait, les rappels ne partaient jamais | **refusé**, la case et la valeur tapée restent à l'écran |

🔴 **Le gabarit posait des règles que personne n'appliquait.** `type="email"` sur
l'adresse d'expéditeur, `min`/`max` sur les délais : cela n'engage que le
navigateur. Le contrôleur lisait `get()` et `getInt()` — qui rend 0 pour ce qu'il
ne comprend pas. Ces trois validations n'existaient pas ; elles existent.

## Deux choses à savoir avant de convertir le suivant

⚠️ **Le thème ne rendait pas l'AIDE d'un champ.** `form_row` sortait le libellé,
le widget et les erreurs. `/admin/settings` porte une phrase d'explication sous
presque chaque champ : convertir sans ajouter `form_help()` au thème aurait
supprimé une quinzaine de phrases en silence — pire que le balisage qu'on venait
ranger. C'est fait, dans le thème partagé, donc le prochain écran en profite.

🔴 **`csrf_token_id` explicite, sinon l'écran est intestable.** Le défaut de
l'application est `submit`, présent dans `stateless_token_ids` : son jeton vit
dans un cookie posé sur la réponse, qu'une requête console ne reçoit jamais. Un
formulaire laissé sur ce défaut **ne peut pas être vérifié par sonde** — c'est
exactement ce qui avait bloqué le témoin de J-8. Un identifiant propre à la carte
retombe sur le jeton de SESSION, se teste, et il est plus étroit qu'un jeton
partagé par tout le site.

⚠️ Et les libellés sont des **clés**, pas du français en dur : les `FormType`
existants de cette base écrivent leur français directement, ce qui les rend
intraduisibles. Les huit nouveaux reprennent les clés déjà traduites en cinq
langues. Les `placeholder`, eux, se traduisent à la main — Symfony ne traduit pas
un attribut HTML.

## Deux changements visibles, assumés

- Les rôles de la carte « Exploitation » lisaient « Nom `ROLE_X` » avec
  l'identifiant en `<code>` ; un `ChoiceType` échappe ses libellés, ils lisent
  « Nom — ROLE_X ». Même information, sans la chasse fixe.
- Un champ refusé ne fait plus enregistrer les autres **du même formulaire** :
  la carte entière est re-rendue telle que soumise. C'est un pas de plus que le
  point 8, qui demandait seulement de ne pas faire ressaisir.

---

# J-22 — la classification, parce que « 25 formulaires » n'était pas 25 conversions

**2026-08-23.** Après avoir converti `/admin/settings`, `/admin/emails` et
`/admin/wizard`, j'ai voulu enchaîner sur le reste et j'ai buté sur
`/admin/features` : c'est une **grille de cartes** — une par fonctionnalité, avec
son libellé, son workspace, sa description, un interrupteur stylé et le panneau
« ce qui disparaît sans elle ». Un `ChoiceType` expanded y aurait rendu une liste
plate de cases et **détruit l'écran**.

D'où une classification, forme par forme, des 39 formulaires admin restants :

| Nature | Nombre | Faut-il convertir ? |
|---|---|---|
| ✅ **Liste de champs** | **23** | oui — c'est J-22 |
| **Matrice** (nom de champ construit : `sections[clé][champ]`) | 8 | ⚠️ pas tel quel |
| **Filtre GET** (`_admin_filters`, `_admin_list`, reporting, horaires) | 5 | 🔴 **non** — un filtre appartient à l'URL |
| **Contrôles dans une boucle** (une ligne = un contrôle) | 3 | ⚠️ pas tel quel |

⚠️ **Le chiffre « 25 gabarits sur 52 » de la revue était juste ; la conclusion
« donc 25 conversions » ne l'était pas.** Un filtre de liste n'a rien à faire dans
un `FormType` : sa place est dans la query string, et c'est déjà le cas. Une
matrice d'audiences × sections (`/admin/homepage`) ou de quotas × paliers
(`/admin/booking-policies`) est une grille, pas un questionnaire — la feuille de
route le dit déjà de la première.

## Fait

| Écran | Formulaires | Vérifié |
|---|---|---|
| `/admin/settings` | 5 | POST refusé : re-rendu, valeur conservée, erreur sur le champ |
| `/admin/emails` | 3 | idem, plus trois validations qui n'existaient pas |
| `/admin/wizard` | 1 | rendu relu à l'écran |

## Reste, par ordre de valeur

1. `admin-usage-package-form` — **6 formulaires**, et c'est l'écran le plus dense
   du produit (168 champs rendus, J-10). Le plus rentable.
2. `admin-formation-content` — 3 listes de champs (les autres formes sont des
   matrices ou des boucles).
3. `admin-opening-hours` (1), `admin-formation-section-form` (1),
   `staff-access-passes` (1), `admin-network` (add) (1),
   `admin-event-categories` / `admin-machine-categories` (créer + renommer),
   `admin-utilisateur-detail` (type de personne).

⚠️ **Trois règles apprises en convertissant, à relire avant la prochaine :**
1. **Le thème rend `form_help()` depuis S147** — mais un écran qui portait ses
   hints à la main les perd si on ne les passe pas en `help`.
2. **`csrf_token_id` explicite**, sinon l'écran bascule sur le jeton *stateless*
   et devient invérifiable par sonde.
3. 🔴 **Relire la page RENDUE.** `debug:translation` scanne les gabarits, **pas le
   PHP des `FormType`** : quatre clés de libellé inventées se sont affichées
   telles quelles sans qu'aucun outil ne les signale.
