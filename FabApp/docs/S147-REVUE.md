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

## J-11 🔴 `/machines/{id}` est inutilisable sur un téléphone

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

## J-12 🔴 Le calendrier de réservation d'un espace est coupé sur téléphone

`/places/{id}` : `.place-calendar` fait **343 px de large pour 748 px de contenu**, avec
`overflow-x: visible`. **168 éléments** dépassent le cadre, dont **60 `agenda-slot-cell`
et 39 `slot-book-affordance`** — les cibles cliquables. La page dit « Cliquez un créneau
ouvert pour ouvrir le panneau de réservation » ; la plupart des créneaux sont
inatteignables.

✅ **Contre-mesure faite** : `/machines` et `/events/{id}` donnent **0** élément hors
cadre à 375 px. Les listes et la page événement vont bien. Le dégât est sur **les deux
surfaces de réservation**, pas partout.

⚠️ Au passage, le titre du panneau est « **Reserver** cet espace » — sans accent, et
écrit deux fois.

## J-13 🔴 « Réserver une machine » mène au seul calendrier qui ne réserve pas

`NavBuilder.php:65` pointe `cal.book_machine` sur `app_calendar`. Or
`SiteController:314` passe **`booking: false`** à cette page, quand `/machines/{id}` et
`/places/{id}` passent `true`.

🔴 **S146 avait vu la moitié du problème** : le commentaire au-dessus explique que le
groupe a été renommé « Au programme » parce que « Calendrier » *« envoyait les gens sur
un écran qui ne peut pas »* réserver — mais **la destination de l'enfant n'a jamais été
changée**. Son voisin `cal.book_space` pointe correctement sur `app_places`.
C'est une ligne, et c'est l'entrée la plus utilisée du menu.

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
