# S149 — revue d'utilisabilité de fin de Phase J

**2026-08-24, conclue le 2026-08-27.** Mandat « designer d'Apple », comme S147 :
le nombre de clics avant et après, l'évidence du chemin, les frappes — surtout en
cas d'erreur — et tout champ demandé sans être indispensable.

---

## ✅ Cette revue est conclue

Elle s'est faite en **deux moitiés**, à trois jours d'écart parce que l'accès était
tombé entre les deux.

| moitié | date | ce qu'elle tranche | résultat |
|---|---|---|---|
| **statique** | 2026-08-24 | ce qu'une lecture prouve : `for`/`id`, la table des routes, `disabled`, la parité des cinq catalogues, les motifs ICU | **16 défauts**, tous corrigés |
| **navigateur** | 2026-08-27 | ce qu'aucune lecture ne prouve : géométrie, contraste, mode sombre, 375 px, focus clavier, clics réels | **6 défauts**, tous corrigés et déployés |

🔴 **Ce que la seconde moitié a démontré sur la première.** Trois des six défauts
qu'elle a trouvés vivaient dans du balisage parfaitement correct : un en-tête HTTP
manquant, trois jetons CSS jamais définis, et un libellé juste dans les cinq
catalogues mais faux à l'écran. La parité, le lint et les 227 routes étaient verts
à chaque fois. **Le verdict d'une revue statique n'est pas « c'est propre » : c'est
« rien de ce que je sais lire n'est faux ».**

⚠️ Et le pire coût n'était pas un défaut : la session du 24 a conclu « site
injoignable, 403 au proxy » et écrit une revue entière sans accès, alors que les
deux moitiés du problème étaient franchissables. Voir [[project-fabos-deployment]] :
essayer les DEUX chemins SSH, et — pour le site public — vérifier depuis quel réseau
on sort avant de conclure à une panne.

---

## Ce que la moitié statique a trouvé — 16 défauts, tous corrigés

### 🔴 `/register` : accepter deux documents illisibles

Les liens « conditions d'utilisation » et « politique de confidentialité »
pointaient sur `href="#"`, à côté d'une case **obligatoire**. Les deux pages
existent et sont routées depuis toujours (`/conditions-utilisation`,
`/confidentialite`). ⚠️ Et « et la » était du **français en dur** entre les deux :
en anglais, la phrase donnait « I accept the terms of use **et la** privacy
policy ».

### 🔴 `/profil/mot-de-passe` : trois champs de mot de passe anonymes

Le libellé visible est un `<h4>` voisin, sans lien programmatique avec le champ.
Un lecteur d'écran annonçait trois champs identiques à la suite — impossible de
distinguer l'ancien du nouveau, sur la page la plus sensible du produit.

### 🔴 `/reset-password` : une carte sans aucune règle CSS

Huit sélecteurs (`.auth-card`, `.auth-section`, `.auth-title`…) utilisés par deux
pages, définis dans le `<style>` en ligne d'**une seule**. Un `<style>` ne traverse
pas les documents : la page où atterrit un membre verrouillé dehors n'avait ni
surface, ni ombre, ni rembourrage, ni centrage.

### 🔴 Une gouttière vide de 44 px sur trois pages

`.form-input { padding-left: 44px }` réserve la place d'une icône. Trois pages
n'en ont pas : `/reset-password`, `/profil/mot-de-passe`, `/forgot-password`.

### Le reste

11 autres champs sans étiquette atteignable (l'éditeur de contenu de formation,
les téléversements d'avatar et de bannière, l'affiche d'événement, la photo de
page de lab, l'état au retour d'un prêt, l'ordre d'un bloc d'accueil, le texte
d'une réponse de quiz) · 2 contrôles désactivés qui ne faisaient rien
(`/machines/{id}` « Indisponible », `/admin/utilisateurs/{id}` « Déjà validé ») ·
1 trou de mode sombre que J-15 ne pouvait pas voir, parce qu'il vivait dans un
gabarit et non dans une feuille.

---

## Ce qui est propre, et mesuré comme tel

| Contrôle | Résultat |
|---|---|
| routes GET que rien ne référence | **0** sur 227 |
| champs sans étiquette atteignable | **0** (29 alertes → 12 vraies → 0) |
| `<img>` sans `alt` · `<button>` sans nom | **0** · **0** |
| contrôles morts | **0** hors `/admin/design` (spécimens) |
| tableaux : en-têtes ≠ cellules, ou > 5 colonnes | **0** réel (2 alertes réfutées) |
| parité des cinq langues | **3179 clés, 0 manque** |
| motifs ICU (syntaxe, types, arguments) | **395 motifs, 0 faute** |
| équilibre des blocs Twig | **209 gabarits, 0 anomalie** |
| formulaires qui font ressaisir un champ refusé | **0** sur les 13 écrans sondés |

⚠️ **Deux outils d'audit mentaient et ont été corrigés d'abord.** `parity.py`
déclarait 97 clés manquantes dont 74 vivent dans le catalogue ICU depuis J-4 ; mon
détecteur d'a11y comptait `<label><input> Texte</label>` comme un champ sans
étiquette. Un audit non vérifié invente du travail — c'est écrit dans
`feedback-fabos-verify-pixels`, et ça s'est reproduit ici deux fois.

---

## Qualité des formulaires — le test « designer d'Apple », en chiffres

⚠️ **Le barème n'est pas inventé ici.** Il existe déjà au dépôt : `docs/history/
phase-U-design-S45-S57.md` § *« Les détails à voler »*, écrit en S58 à partir des
**73 captures de Fabman** (`Stage/Drive/Images/Fabman UI/`). Cette section ne fait
que le rendre MESURABLE et l'appliquer.

Ce que ces captures font, et qu'un formulaire réussi fait :

1. **Il s'ouvre sur son cas courant.** Les champs optionnels n'existent pas tant
   qu'on ne les demande pas — *« Add a note »*, *« Set end date »* sont des liens.
2. **Une case à cocher révèle ses propres sous-champs**, indentés dessous.
   Décochée, ils ne sont pas là du tout.
3. **Le contrôle est une PHRASE avec des champs dedans** — *« They must have a
   balance of at least [€ __] to turn on any equipment with usage fees »* — pas
   une étiquette au-dessus d'une boîte.
4. **Une ligne de conséquence sous le contrôle**, calculée depuis ce qui est tapé :
   *« Creating an invoice today would set its due date to 07/30/2026 »*.
   Le formulaire dit ce qu'il s'apprête à faire.
5. **`required` est un petit mot gris à côté de l'étiquette.** Pas d'astérisque,
   et rien du tout sur les champs optionnels.
6. **La carte est ÉTROITE et centrée** (~350 pt), même pour le plus gros
   formulaire du produit — « Add member », qui a pourtant une vingtaine de champs.
   Elle ne s'étire jamais à la largeur de l'écran.
7. **La valeur courante s'affiche en texte avec un lien `Change`** quand le cas
   courant se passe de contrôle : *« Member role : Normal member — Change »*.

### Ce que ça donne chez nous

Mesuré par `tools/form_quality.py` — champs visibles **à l'arrivée**, avant tout clic :

| Écran | avant | après | replié | formulaires |
|---|---:|---:|---:|---:|
| `admin-usage-package-form` | **28** | **7** | 21 | 10 |
| `admin-formation-content` | 35 | 35 | 0 | 7 |
| `admin-emails` | 15 | 15 | 0 | 3 |
| `admin-settings` | 13 | 13 | 2 | 5 |
| `admin-event-new` | 13 | 13 | 0 | 1 |

🔴 **L'éditeur de packages ouvrait sur 28 champs vides et quatre formulaires
« ajouter » dépliés en même temps** — un grant (9 champs), une allocation (6), une
attribution à une personne (3), une au groupe (3). Quatre décisions simultanées
posées à quelqu'un qui venait peut-être renommer le package. **Les quatre passent
derrière un repli** : l'écran ouvre sur ses 4 champs de détail et trois liens.
⚠️ Le repli se **rouvre** quand la soumission est refusée — sinon l'opérateur ne
verrait ni ce qu'il a tapé ni pourquoi c'est refusé.

### Les deux chiffres qui décrivent le mieux l'écart

| | |
|---|---|
| écrans à formulaire avec **au moins un repli** | **8 sur 75** |
| champs qui portent une phrase d'aide | **56 sur 276 — 20 %** |

**Quatre champs sur cinq ne disent rien de plus que leur étiquette.** C'est la
version mesurable de « certains écrans d'admin n'ont pas l'air utiles » : un écran
qui aligne des boîtes sans dire ce qu'elles changent n'a pas l'air d'un outil, il a
l'air d'un formulaire de saisie. Les pires, tous à **zéro** aide pour 8 champs ou
plus : `UserAdminType` (15), `FormationAdminType` (13), `MaterialAdminType` (11),
`LoanAdminType` (10), `PlaceAdminType` (9), `PackageGrantType` (9),
`LoanableItemAdminType` (9), `MaintenanceTaskAdminType` (8).

⚠️ **Aucun des points 3, 4, 6 et 7 n'est appliqué nulle part** — pas une phrase à
trous, pas une ligne de conséquence, pas un `required` discret, pas un
« valeur + Change ». Ce sont quatre chantiers, pas quatre correctifs.

### 🔴 Et ce que cette section NE dit toujours pas

« Visuellement plaisant » est une question de PIXELS. Largeur de carte, rythme
vertical, graisse des étiquettes, densité : **non mesurés**, pour la même raison
que le reste de cette revue. Les chiffres ci-dessus disent qu'un écran demande
trop de choses à la fois et qu'il ne s'explique pas — ils ne disent pas qu'il est
beau ou laid.

---

## Clics : notre système de packages contre celui de Fabman

**Tâche mesurée** : créer un package qui laisse utiliser les machines, à toute
heure, et l'attribuer à un membre.

### Chez nous — compté sur le code, pas estimé

| # | Clic | Où |
|---:|---|---|
| 1 | « Droits d'usage » dans la barre latérale | → `/admin/usage-rights` |
| 2 | « Créer un package » | → `/new` |
| 3 | « Enregistrer » *(le nom se tape, ça ne compte pas comme un clic)* | → `/{id}/edit` |
| 4 | déplier « Ajouter le grant » | *(nouveau — voir plus bas)* |
| 5 | choisir la fonctionnalité | |
| 6 | choisir l'action | |
| 7 | « Ajouter le grant » | |
| 8 | déplier « Attribuer » | *(nouveau)* |
| 9 | choisir le membre | |
| 10 | « Attribuer » | |

**10 clics.** ⚠️ Et **sept des neuf champs du grant ne sont pas touchés** : lieu,
section, ressource et catégorie valent « tout » par défaut, et la fenêtre horaire a
un défaut. Le coût réel de cet écran n'a jamais été le nombre de clics.

⚠️ **Le repli d'aujourd'hui a COÛTÉ deux clics** (4 et 8) et retiré 21 champs de la
vue. C'est le bon échange, mais il faut le dire dans les deux sens.

### Chez Fabman — dérivé, pas observé

🔴 **Je n'ai pas trouvé l'écran d'édition de package.** J'ai ouvert 8 des 73
captures ; celle-là n'en fait pas partie. Le chiffre ci-dessous vient du **modèle
documenté** (`phase-U-design-S45-S57.md` ligne ~925 : un grant y est *(équipement
ou catégorie) × (24/7 · heures d'ouverture · fenêtre personnalisée) × (peut
réserver o/n)*) et du **chemin de navigation visible** dans les captures
(`Configure ▾ → Packages`). À confirmer en ouvrant les autres captures.

Environ **11 à 12 clics** : 3 pour arriver, 1 nom, 3 choix de grant, 1
enregistrement, puis l'attribution depuis la fiche du membre (3 à 4).

### 🔴 Donc : le nombre de clics n'est PAS le problème

Les deux systèmes sont au coude à coude, et le nôtre est peut-être devant. Ce qui
diffère est ailleurs, et c'est là qu'il faut travailler :

| | FabOS | Fabman |
|---|---:|---:|
| champs à l'arrivée sur l'éditeur | 28 → **7** *(corrigé ce jour)* | ~6 |
| **dimensions d'un grant** | **9** | **3** |
| choisir « à toute heure » | 3 contrôles (jour + début + fin) | **1 bouton radio** |
| le grant se lit comme | 9 boîtes de même poids | une phrase |

**Les neuf dimensions ne sont pas un défaut d'interface, c'est notre modèle** —
fonctionnalité × action × lieu × section × ressource × catégorie × jour × début ×
fin. Il est plus fin que celui de Fabman, et volontairement. Le défaut, c'est de le
**présenter à plat** : neuf contrôles de même poids alors que sept ont un défaut
sensé. Un préréglage de fenêtre (`24/7` par défaut) et un repli « Restreindre… »
sur la portée ramènent le cas courant à deux choix, sans rien retirer au modèle.

---

## Placement : plusieurs formulaires sur une page — quand, et comment les séparer

⚠️ **Compter les `<form>` ment** : un bouton « archiver » est un `<form>` avec un
jeton caché. Ce qui pèse, c'est le nombre d'**éditeurs** — un formulaire qui
demande au moins deux champs. Mesuré par `tools/form_quality.py` et le décompte
ci-dessous :

| Écran | éditeurs | champs | souches d'action | séparation actuelle |
|---|---:|---:|---:|---|
| `admin-formation-content` | **7** | 35 | 0 | carte + sommaire |
| `admin-usage-package-form` | **6** | 28 | 4 | repli |
| `admin-settings` | **5** | 15 | 0 | carte + sommaire + repli |
| `profil` · `admin-network` | 3 | 12 | — | repli |
| tout le reste | ≤ 2 | | | |

**49 écrans n'ont qu'un seul éditeur.** Un objet, une page : c'est juste, et il ne
faut pas les fusionner. La question ne se pose que pour trois écrans.

### La réponse : le seuil est à quatre

- **≤ 3 éditeurs** → des cartes empilées suffisent, chacune avec son titre, sa
  **ligne d'état** et son propre Enregistrer. `admin-settings` fait déjà exactement
  ça et c'est pour ça qu'il est le moins pénible des trois.
- **≥ 4 éditeurs** → ce n'est plus une pile, c'est la **forme 3 du contrat**
  (« object settings ») : une **sous-navigation à gauche de l'objet**, **un panneau
  à la fois**, chacun avec son Enregistrer. C'est ce que Fabman fait de son espace :
  `Location & contact info · Opening hours · Holidays & exceptions · Booking
  settings · Invoices & taxes · Payment methods · 3rd-party integrations`.

### 🔴 Et la sous-navigation doit être de VRAIES URL, pas des onglets JavaScript

`/admin/usage-rights/{id}/edit/grants`, pas `#grants`. Trois raisons, toutes
propres à ce dépôt :

1. **Le refus doit re-rendre son panneau.** Un onglet client perd le formulaire
   soumis ou oblige à réimplémenter le retour d'erreur en JS. Avec une URL par
   panneau, le comportement acquis en J-22 fonctionne sans une ligne de plus.
2. **Turbo est OFF** et Stimulus n'est chargé que là où `importmap('app')` est
   émis. Un onglet JS est une dépendance de plus sur une page qui n'en a pas besoin.
3. **Un panneau devient adressable** : « va sur l'onglet Attributions » devient un
   lien, ce qu'un `#ancre` sur une page de 35 champs n'a jamais vraiment été.

⚠️ **Ce qu'il faut garder de l'existant** : la **ligne d'état** sous le titre de
chaque carte de `admin-settings` (« ENSEA · FabLab », « Europe/Paris · 16 h 25 »).
C'est elle qui laisse lire l'état d'une installation sans rien ouvrir, et c'est
exactement l'entrée de sous-navigation de Fabman, qui porte ses compteurs. À
reprendre telle quelle dans la sous-navigation.

---

## Les quatre relecteurs — verdicts, et ce qui RESTE (2026-08-24)

Quatre paires codeur + relecteur, un écran chacune. **Trois relecteurs sur quatre
ont rendu NOT YET**, et les deux pires trouvailles étaient EN LIGNE.

### 🔴 Corrigé et déployé

| # | Ce que c'était | Qui l'a trouvé |
|---|---|---|
| 1 | **500 sur `/admin/usage-rights/{id}/edit`** — j'avais retiré `VenueRepository $venues` de la signature, deux appels y survivaient. `php -l` vert : une variable non définie n'est pas une erreur de syntaxe | relecteur A |
| 2 | **Effacement silencieux d'un programme entier** — un onglet ouvert avant le déploiement poste l'ancien format texte, le nouveau lecteur écrivait `items: []` avec un flash vert | relecteur B |
| 3 | **`[hidden]` perd contre toute règle d'auteur** — les champs de plage ne se cachaient jamais, et leur valeur était jetée en silence | relecteur A |
| 4 | **`form_help` sans `id`** — `aria-describedby` pendait dans le vide, donc l'aide n'était annoncée à personne | relecteur C |
| 5 | **Six aides affirmaient des choses fausses** sur le produit | relecteur D |
| 6 | **Deux aides demandaient l'impossible** (taper `a \| b \| c` sous un tableau) | relecteur B |
| 7 | `settings_consequence_gaps` jetait `%capabilities%` et affirmait le contraire de la vérité | relecteur C |

### 🅿️ RESTE À FAIRE — rien de tout ceci n'est cassé, tout est écrit

**Paire D — ✅ FAIT le 2026-08-27 : le motif CRUD sur ses trois écrans.**
Tout ce qui suit était écrit ici comme « reste à faire » ; c'est corrigé, déployé
et mesuré à l'écran. Voir la section « Paire D » plus bas.

**Paire C — ✅ FAIT le 2026-08-27 : la règle 4 sur `/admin/emails`.**
Les trois horizons étaient calculés, passés à la vue, et le gabarit n'en rendait
aucun — pendant que son commentaire d'en-tête décrivait la ligne de conséquence
comme si elle existait. Elles s'affichent maintenant sous chaque délai :
« La prochaine passe rappellera les réservations qui commencent d'ici le 28/08 à
15:18 », « … les prêts à rendre jusqu'au 29/08/2026 inclus ».
⚠️ **L'horizon des prêts est rendu comme une DATE**, pas une heure : il vaut
`setTime(0,0)` en UTC, donc avec une heure il aurait affiché *02:00* en été — une
heure que personne n'a choisie. Le scanner compare des dates ; la phrase aussi.
⚠️ **Et la phrase suit l'état du FORMULAIRE**, comme les délais : un rappel décoché
affiche « Ce rappel est désactivé : la prochaine passe n'enverra rien » plutôt que
de continuer à annoncer un envoi. 🅿️ Cette branche-là n'est pas exercée sur les
données de la boîte — les cinq rappels y sont actifs ; c'est la seule affirmation
de cette section qui repose sur la lecture du gabarit et pas sur un rendu.

**Paires A et B — ✅ FAIT le 2026-08-27**, sauf un point qui vous revient.
Détail, chiffres et les deux alertes RÉFUTÉES : section « Paires A et B » plus bas.
🅿️ Reste votre arbitrage : le tableau du programme a **perdu une capacité** —
coller huit étapes d'un coup et réordonner en déplaçant des lignes. Le codeur
argumente l'échange ; c'est une décision d'opérateur, pas de codeur.

### Ce que cette manche a prouvé sur la méthode

🔴 **Aucun des six outils n'a vu les deux défauts les plus graves.** Une variable
non définie n'est pas une erreur de syntaxe ; un `is_array` qui rejette l'ancien
format est du code parfaitement valide. Ce sont deux lecteurs humains — enfin,
deux agents qui LISENT — qui les ont trouvés. Les outils disent qu'un écran est
cohérent, jamais qu'il est juste.

⚠️ Et un relecteur a trouvé un défaut dans **l'outil qui le mesurait**
(`form_quality.py` comptait deux fois un `<details>` imbriqué) en refusant de le
corriger lui-même. C'est la bonne réponse : on ne répare pas sa propre balance.

---

## ✅ La seconde moitié : la passe navigateur, faite le 2026-08-27

L'accès est revenu (`proxmox.lab.dryades.org` répond ; l'IP `51.68.38.235` expire —
c'est bien deux chemins, et un seul marche par réseau). Le site public répond 200
dans le volet.

🔴 **Le « 403 au proxy » : la vraie cause, et une attribution à défaire.**
Ce n'est **pas** le défaut N1 ci-dessous, contrairement à ce que cette revue a
d'abord écrit. `fabos.dstei.fr` porte une **liste blanche NPM** — mesurée dans
`/opt/npm/data/nginx/proxy_host/5.conf` :

    allow 192.168.100.0/24;   # le LAN
    allow 100.64.0.0/10;      # le tailnet
    allow 82.64.192.225/32;
    allow 193.51.46.24;       # l'école
    deny all;

Le site est donc **privé par construction**, et un `403 Forbidden` d'openresty
signifie « ton IP de sortie n'est pas dans la liste », pas « le service est tombé ».
Vécu deux fois dans la même session : les mesures du matin passaient depuis
`193.51.46.24`, et l'après-midi, réseau changé, la sortie est devenue
`78.240.77.69` — tout est passé en 403 d'un coup, y compris le volet navigateur.
Le journal qui tranche en une ligne :
`/opt/npm/data/logs/proxy-host-5_error.log` → *access forbidden by rule, client: …*

⚠️ **Deux pannes distinctes se ressemblaient**, et c'est ce qui a fait tenir le
mauvais diagnostic : le 403 de la liste blanche, et le 302 en `http://` du défaut
N1 — qui, lui, ramenait le membre sur une URL que le filtre de l'école bloque. Les
deux se présentent comme « le site répond 403 ». Le second est corrigé et vérifié ;
le premier est un réglage voulu, et il appartient à l'opérateur.

### Méthode — ce qui a rendu la passe rapide

Deux outils valent d'être réutilisés :
1. **Un `<iframe>` de 375 px (ou 1280) piloté en JavaScript.** Il charge les pages
   l'une après l'autre sans navigation, avec son propre viewport, donc les media
   queries s'appliquent. 27 pages publiques mesurées en un appel.
2. **La recette « admin sans authentification »** de
   [[feedback-fabos-verify-pixels]], en lot : `app:render` des 59 pages admin sur
   CT 210, `tar` + `base64` pour les rapatrier, réécriture des URLs d'actifs vers
   `https://fabos.dstei.fr/`, `python3 -m http.server`, et le même iframe dessus.

⚠️ **Le volet navigateur rend des styles calculés PÉRIMÉS d'un appel de retard.**
Un lot « focus chaque contrôle et lis son contour » a rendu les valeurs décalées
d'un élément et m'a fait conclure « aucun focus visible nulle part ». La mesure
juste demande soit un vrai `Tab` au clavier, soit deux appels séparés.

---

### 🔴 N1 — toute redirection de connexion partait en `http://`, et le filtre de
l'école la bloquait  ✅ **corrigé et vérifié en ligne**

`GET https://fabos.dstei.fr/profil` répondait `302 Location:
http://fabos.dstei.fr/login` — schéma perdu. Suivi jusqu'au bout : **403**. Dans
le navigateur, sur le réseau de l'école, on obtenait la page FortiGuard
**« Web Page Blocked ! »**.

Donc **un membre déconnecté qui clique n'importe quel lien protégé, ou dont la
session expire, n'atteignait jamais le formulaire de connexion.**

⚠️ **Ce défaut est réel et indépendant de la liste blanche ci-dessus** : il a été
prouvé par l'en-tête `Location` lui-même, depuis une IP autorisée, avant et après
correction.

Cause : `config/packages/framework.yaml` ne déclarait aucun `trusted_proxies`,
donc Symfony ignorait `X-Forwarded-Proto` et fabriquait des URLs absolues en http.
Correctif : `trusted_proxies: '192.168.100.20'` — l'adresse du proxy telle que
l'app la voit dans le journal de `fabos.service` — plus les `trusted_headers`.
⚠️ **Pas `REMOTE_ADDR`** : le `:8000` est joignable sur le LAN, donc `REMOTE_ADDR`
laisserait n'importe quelle machine du réseau forger l'en-tête.

Vérifié après redémarrage : `Location: https://fabos.dstei.fr/login`.

### 🔴 N2 — trois jetons CSS utilisés partout et définis nulle part ✅ **corrigé**

`--spacing-3xl` (2 usages), `--font-size-xs` (21), `--font-size-md` (13, défini
seulement dans `calendar-fix.css` que la plupart des pages ne chargent pas). Une
`var()` sans repli et sans définition rend la déclaration **invalide**.

Conséquence mesurée à l'écran sur `/forgot-password` :
`.auth-card { padding: var(--spacing-3xl) }` valait **0 px**, le champ e-mail
touchait le bord de la carte (410 → 860 dans une carte 409 → 861), et 21 pastilles
rendaient à la taille héritée.

⚠️ **Et ce n'est PAS une régression de S149.** Le `<style>` en ligne d'origine de
`forgot-password` écrivait déjà `var(--spacing-3xl)` : la carte n'a jamais eu son
rembourrage. Le déplacement des 31 règles a fidèlement transporté le défaut. La
moitié statique avait raison de dire « la carte n'avait ni surface ni ombre » ;
elle ne pouvait pas voir qu'après correction il manquait encore le rembourrage.

Correctif dans `style.css` : `--spacing-3xl: 64px` (40 px sous 576 px),
`--font-size-xs: 12px`, `--font-size-md: var(--font-size-base)`.
Après : rembourrage **64 px**, le champ à **65 px** du bord.
Re-balayage de 27 pages : **0 jeton manquant**.

### 🔴 N3 — le bouton « Se connecter » de `/login` disait « Envoyer le lien » ✅ **corrigé**

`login.submit` valait « Envoyer le lien » / « Send the link » / « Link senden » /
« Enviar el enlace » / « Invia il link » — dans les **cinq** langues. C'est le
bouton principal de la page de connexion. La clé est aussi réutilisée comme lien
« retour à la connexion » par **sept** gabarits (`reset-password`,
`forgot-password`, `leaderboard`, `person-booking` ×2, `machine-detail`,
`_calendar_booking`), où elle rendait « Vous avez déjà un compte ? Envoyer le lien ».

⚠️ **Aucun contrôle statique ne pouvait le voir** : la clé existe, la parité des
cinq catalogues est complète, le lint passe. Il fallait regarder le bouton.
Corrigé en « Se connecter » / « Sign in » / « Anmelden » / « Iniciar sesión » /
« Accedi ». `forgot.submit` reste « Envoyer le lien » — c'est son vrai bouton.

### 🔴 N4 — `/forgot-password` se contredisait ✅ **corrigé**

Le sous-titre disait « Contactez un administrateur du FabLab : il peut
réinitialiser votre mot de passe depuis la console d'administration » — dans les
cinq langues — alors que la page porte juste en dessous le formulaire libre-service
que S134g lui a donné. Texte d'avant la fonctionnalité. Réécrit dans les cinq langues.

### 🔴 N5 — les écrans de mur : une icône sans taille, mesurée à 1665 px ✅ **corrigé**

`kiosk.css` (79 lignes) ne définissait pas `.ic`, et les gabarits kiosque sont
autonomes : ils ne chargent pas `components.css`, où vit `.ic { width: 1em }`
(S148, J-7). Un SVG sans dimension intrinsèque s'étire à la largeur de son parent.
Mesuré à 1920 × 1080 :

| écran | avant | après |
|---|---|---|
| `/kiosk/events` — épingle de lieu | **1665 × 1665 px** | 35 × 35 px |
| `/kiosk/events` — carte à la une | **1982 px de haut** pour 1080, soit **902 px coupés** | 310 px |
| `/kiosk/machine/{id}` — épingle | **289 × 289 px**, page 1115 px | 33 × 33 px, page 1080 px |

Le lieu, « 12 places » et les deux événements suivants étaient dans le DOM et
**invisibles sur le mur**. C'est exactement la faute que J-7 avait corrigée une
fois — elle rejoue partout où la feuille qui porte le correctif n'est pas chargée.

✅ `/kiosk/entries` et `/kiosk/stats` étaient propres à 1920 × 1080.

### 🔴 N6 — « Derniers passages » annonçait des passages du 10 juillet ✅ **corrigé**

`/kiosk/entries` n'affichait que `H:i`, sans date. Le dernier passage du journal
RFID date du **2026-07-10** (vérifié en base). Le mur les annonçait « 10:41 »,
donc de ce matin, pendant que l'écran voisin `/kiosk/stats` affichait
**« 0 PASSAGES AUJOURD'HUI »**. Deux murs côte à côte qui se contredisent.
Un passage qui n'est pas du jour porte maintenant son jour (« 10/07 10:41 »).

⚠️ Reste ouvert : `KioskController::stats()` enveloppe chaque compte dans
`catch(\Throwable) → return 0`. Une requête cassée s'affiche comme un zéro sur le
mur, et rien ne le dit.

---

## Ce qui a été mesuré et qui est PROPRE

| Mesure | Résultat |
|---|---|
| débordement horizontal à 375 px | **0** sur 27 pages publiques (`scrollWidth` 365 partout) |
| débordement horizontal à 1280 px | **0** sur les 59 pages admin |
| contrôles `disabled` | **0** sur les 59 pages admin |
| contraste des pastilles d'état, thème clair | Occupée **6,70** · Libre **5,02** · Hors service **6,47** |
| contraste des pastilles d'état, thème sombre | **8,35** · **10,72** · **7,93** |
| jetons CSS manquants après correction | **0** sur 27 pages |
| carte d'authentification | 452 px, centrée, surface, ombre `0 10px 20px rgba(0,0,0,.15)`, rayon 8, rembourrage 64 |
| balayage des routes après déploiement | **106 pages**, 0 échec réel (`/.well-known/fabos` rend `{"status":"unconfigured"}` en 503 volontairement, `/login/oidc/callback` exige des paramètres) |

**`/profil/password` — les trois champs ne sont pas anonymes.** Chacun porte
`aria-labelledby` + `aria-describedby` vers son `<h4>` et son `<p>`. La correction
S149 tient. ⚠️ Mon premier sondage cherchait `label[for]` et les a comptés fautifs :
**faux positif, réfuté avant d'être compté** — la règle de
[[feedback-fabos-verify-pixels]] a servi une fois de plus.

**`/register` — la phrase d'acceptation est complète dans les cinq langues**, et
les deux liens pointent bien sur `/conditions-utilisation` et `/confidentialite`,
dans une couleur distincte du texte.

**L'éditeur de packages — le repli marche.** 5 `<details>`, **tous repliés à
l'arrivée** (52 px de haut chacun), et les quatre éditeurs portent `open` quand
leur formulaire vient d'être refusé (`refusedEditor`). La question « se
rouvrent-ils sur un refus » est répondue : oui.

---

## 🅿️ Ce que la passe a trouvé et qui RESTE — décisions, pas des bugs

### 🔴 R1 — les libellés des formulaires admin sont du français en dur
Rendu de `/admin/events/new` **en anglais** : « Titre », « Début »,
« Fin (optionnelle) », « Nom du lieu », « Où se déroule l'événement ? » — **15 des
16 étiquettes en français**. `/admin/materials/new` : 20 sur 20.
Compté dans `src/Form/` : **148 libellés littéraux contre 117 clés de catalogue**,
sur **42 classes `FormType`**. `debug:translation` ne lit pas ce PHP, donc rien ne
le signale. `/admin/usage-rights/new` montre que le contraire se fait très bien.

### 🔴 R2 — la largeur des cartes de formulaire : 888 px là où Fabman tient en ~350 pt
**22 formulaires admin** ont leur champ le plus large à 888 px, c'est-à-dire toute
la largeur du panneau. Aucun n'a de largeur maximale. C'est la mesure que la
section « qualité des formulaires » attendait.

### 🔴 R3 — le repli n'existe que sur 4 pages admin sur 59
Champs visibles **à l'arrivée**, mesurés dans le navigateur :
`/admin/homepage` **35** · `/admin/horaires` **31** · `/admin/machines/new` **26**
· `/admin/materials/new` **20** · `/admin/maintenance/batch` 16 ·
`/admin/events/new` 15 · `/admin/utilisateurs/new` 14 · `/admin/features` 14.
Aucun `<details>` sur ces huit écrans.

### ⚠️ R4 — `/lab` : le point d'arrivée d'une entrée de menu ne mène nulle part
Le menu « Fablab » propose **sept** destinations (Machines, Espaces, Matériaux,
Prêts, Maintenance, Équipe, Formateurs). Sa page d'atterrissage `/lab`, celle
qu'on obtient en **cliquant** au lieu de survoler, fait 326 px de haut, montre
**trois** liens (`/lab/3`, `/lab/4`, `/lab/5`) et **ne mentionne pas Machines**.
Un visiteur qui clique perd le menu.

### ⚠️ R5 — deux mots pour le même état de machine
La liste dit « Hors service » (`machines.state_down`), la fiche dit
« En maintenance » (`machines.st_maintenance`) — même machine (id 7), deux mots.
Ce sont deux faits différents (réservabilité vs statut) ; pour un membre c'est une
machine et deux vocabulaires.

### ⚠️ R6 — la pastille « Indisponible » de S149 est inatteignable
`machine-detail.html.twig` teste `usageRight.allowed` **avant** l'indisponibilité.
Un anonyme sur une machine en maintenance lit donc « Se connecter pour réserver » ;
un membre sans package lit le verdict de son package. La branche `_cell_state`
n'apparaît que pour un membre qui a un package **et** une machine indisponible —
c'est-à-dire personne, tant que J-25 n'est pas tranché.

### ⚠️ R7 — quatre façons de dire « connectez-vous » sur un seul écran
`/machines/5` en anonyme affiche « Connectez-vous pour consulter et utiliser vos
droits d'usage », « Connexion requise », « Se connecter pour réserver » et
« Connectez-vous pour créer une réservation et vérifier vos badges ».

### ⚠️ R8 — le champ de recherche de l'en-tête fait 89 px à 375 px
Cinq caractères visibles.

### ⚠️ R9 — `/profil/password` : 500 px entre une étiquette et son champ
La ligne « réglage » met le libellé à x=32 et son champ à x=1021. Le motif vient
de Fabman (« valeur + Change »), mais Fabman y met une valeur et un bouton, pas un
champ de saisie. Et la page rendue en anglais affiche « Compte: Yanis Test ».

---

## Les parcours en clics

Marchés **dans le navigateur**, en anonyme :

1. **Anonyme → réserver une machine.** Accueil → (survol) Fablab → Machines (1) →
   la fiche (2) → « Se connecter pour réserver » (3) → `/login`. ✅ `use_referer:
   true` ramène ensuite sur la fiche, donc **3 clics jusqu'au mur d'authentification
   et rien à re-trouver après**. ⚠️ Si le visiteur **clique** « Fablab » au lieu de
   le survoler, il atterrit sur `/lab` (R4) et doit repartir par le pied de page.
2. **Membre → s'inscrire à un événement.** Accueil → Au programme → l'événement →
   inscription : **3 clics** jusqu'au bouton, mur d'authentification identique.

⚠️ **Non marchés, et il faut le dire :** les parcours 3 à 6 (annuler une
réservation, le tour complet mot de passe oublié, et les deux parcours opérateur)
demandent un compte. Ils ne sont pas comptés ici — les compter sur le code serait
exactement l'erreur que cette moitié existe pour éviter.

---

---

## ✅ Paire D — le motif CRUD, appliqué à ses trois écrans (2026-08-27)

Le reste de la paire D disait : « le motif est fini à un tiers ». Il l'était.
`MachineAdminType::SECTIONS` et `MaterialAdminType::SECTIONS` existaient, couvraient
tous leurs champs, leurs titres étaient traduits dans les cinq langues — et
**personne ne les lisait**. Seul `_loanable_item_form.html.twig` déroulait la
sienne. Un motif appliqué à un écran sur trois n'est pas un motif, c'est une
exception.

### Ce que la liste à la main coûtait, mesuré

| | avant | après |
|---|---|---|
| `manufacturer` et `model` sur la fiche machine | dans **aucun** des deux gabarits : rendus par `form_rest()`, **après le bouton Enregistrer**, sans thème | dans la section « Ce que le public voit », repliée |
| titres de section sur `/admin/machines/new` | **0** | 4, plus 2 replis |
| ce que les replis cachent à l'arrivée | — | **1046 px** sur 2037 (formulaire mesuré replis fermés puis ouverts) |
| `materials_form.help_machines` | **jamais rendue** — le champ était dessiné à la main, sans `form_help` | affichée, et référencée |
| les deux listes machine | divergeaient déjà (`machineToken` d'un seul côté) | une seule liste, en PHP |

### Un défaut trouvé en corrigeant, plus grave que celui qu'on corrigeait

🔴 **Une liste de cases à cocher n'était NI nommée NI décrite.** En vérifiant que
l'aide du champ « machines » s'affichait enfin, la page entière comptait **zéro**
`aria-describedby` : Symfony pose la référence sur un widget simple, pas sur le
conteneur d'un `expanded` + `multiple`. Et son `<label>` n'a pas de `for` — faute
d'un contrôle unique à viser — donc il ne nommait rien non plus. C'est le défaut de
la paire C (`form_help` sans `id`) une marche plus loin : l'`id` existait enfin, et
plus rien ne le visait. Le thème pose maintenant `role="group"`,
`aria-labelledby` et `aria-describedby` sur le conteneur, et `form_label` émet son
`id`. Vérifié sur 8 écrans admin rendus : **0 référence `aria-*` cassée**.

⚠️ **Et deux fautes commises en chemin, corrigées avant d'aller plus loin** — les
deux visibles seulement à l'écran :
1. L'`id` de l'étiquette était passé depuis `form_row`. `/admin/maintenance/batch`
   dessine son groupe **sans** `form_row` : sa référence `aria-labelledby` pendait
   donc dans le vide. Le défaut qu'on réparait, recréé deux écrans plus loin.
   L'`id` se pose maintenant dans le bloc `form_label`, d'où que vienne l'appel.
2. Poser `.choice-grid` sur le conteneur du groupe a **cassé la mise en page** :
   la case et son nom sont des FRÈRES, donc dans une grille ce sont deux cellules
   — la case d'« Uranus » s'est retrouvée à 200 px de son nom. L'ancien balisage y
   échappait par accident (la classe était sur un div PARENT, la grille n'avait
   qu'un enfant et ne rangeait rien). Chaque choix est maintenant une cellule,
   `.choice-item` ; mesuré : 7 px entre la case et son nom, 4 colonnes.

### L'outil qui mesure la règle 1 était sur le point de devenir aveugle

⚠️ `form_quality.py` comptait les `form_row(` **littéraux**. Le motif en écrit un
seul, `form_row(form[name])`, dans une boucle — et les trois écrans convertis
l'appellent depuis un partiel. L'outil aurait annoncé **1 champ** pour un écran qui
en montre onze, et **0 repli** pour un écran qui en a deux : la règle 1 serait
passée de mesurée à décorative au moment précis où on l'applique enfin. Il lit
maintenant la constante `SECTIONS` à la source. Chiffres après conversion :

| écran | visible | total | replié |
|---|---|---|---|
| `_machine_form` | 11 | 19 | 8 |
| `_material_form` | 8 | 10 | 2 |
| `_loanable_item_form` | 6 | 8 | 2 |

### Ce qui a été touché

- **`_form_sections.html.twig`** — la boucle, une seule fois, pour les trois types.
- **`_machine_form.html.twig`** — un formulaire machine au lieu de deux listes.
  `admin-machine-new` passe de 100 à 52 lignes, `admin-machine-edit` de 113 à 64.
- **`_material_form`**, **`_loanable_item_form`** — délèguent au partiel.
- **`admin_theme.html.twig`** — `form_label` émet son `id` ; nouveau bloc
  `choice_widget_expanded` (rôle, nom, description, une cellule par choix).
- **`MaterialAdminType`**, **`MaintenanceBatchType`** — `.choice-grid` posée par
  `attr` sur le widget, plus par le gabarit.
- **Les trois `SubmitType`** portent `common.save` au lieu du littéral
  « Enregistrer » — trois libellés de moins sur les 148 de R1.
- **`admin.css`** — une règle, `.choice-item`.

Déployé et vérifié : 115 fichiers identiques par hachage, `lint:twig` 211/0,
`lint:yaml` 39/0, `php -l` propre, **balayage de 171 pages sans échec**.

---

---

## ✅ Paires A et B (2026-08-27)

### Paire A — l'éditeur de packages

🔴 **`window_preset` et `day_of_week` portaient le MÊME libellé.** Les deux listes
sont voisines et s'appelaient toutes les deux « Créneau »
(`usage_rights.grant_window`) : la question « quel créneau ? » était posée deux
fois, et le jour ne se nommait nulle part. Le `<select>` du gabarit portait la même
chaîne en `aria-label`. Nouvelle clé `usage_rights.grant_day` dans les cinq langues.
Vérifié à l'écran : les libellés lisent maintenant **« Time slot »** puis **« Day »**.

🔴 **`array_filter()` sans callback jetait aussi les chaînes « 0 ».** `grantParts()`
rend `null` pour « pas de valeur » et une chaîne sinon ; le seul test juste est
`!== null`. Prouvé sur la boîte :

    sans callback : Atelier
    avec !== null : Atelier · 0

Une section — ou un lieu, une catégorie, une ressource — nommée « 0 » disparaissait
de la phrase de conséquence **et** du résumé de portée. Rien ne cassait : la ligne
s'affichait, simplement plus courte d'un membre. Un résumé qui omet une dimension de
la portée ment sur ce que le grant va autoriser.

🔴 **La ligne de conséquence arrivait amputée** : `feature` n'avait pas de défaut,
donc la phrase annonçait « Ce grant autorisera : Réserver · À toute heure » sans
dire de QUOI — au-dessus d'un `<select>` requis qui affichait déjà la réponse.
La phrase doit décrire ce que le navigateur va envoyer. Vérifié à l'écran :
*« This grant will allow: **Book equipment** · Use — use and book · Every location
· Any resource · Any category · At any time »*, et le `<select>` est bien sur
`machines`.

⚠️ **Les commentaires décrivaient trois préréglages ; il n'en reste que deux.**
« Horaires d'ouverture » a été retiré en S149 pour une raison qui est écrite au
complet dans `UsageRightsAdminController` au-dessus de `$presetChoices` — il ne
pouvait que COPIER la grille du lieu, et la copie ne suit pas un changement
d'horaires. Les deux commentaires du `FormType` et celui du gabarit renvoient
maintenant à ce paragraphe au lieu de le redécrire de travers.

### Paire B — l'éditeur de contenu d'une formation

⚠️ **« Le compteur n'est que sur 4 cartes sur 9 » : réfuté à moitié, et c'est deux,
pas cinq.** En regardant ce que les neuf replis contiennent, **trois n'ont rien à
compter** — `general`, `labels` et `practical` sont des groupes de champs fixes, et
un compteur y afficherait un nombre qui ne varie jamais. Il manquait sur les **deux**
autres qui bouclent sur une collection : « Introduction au parcours » (les cartes) et
« Pour aller plus loin » (les liens). Ajoutés, avec leurs pluriels ICU dans les cinq
langues. Vérifié dans l'arbre d'accessibilité : `generic "3 cards"` sous le titre du
repli.

⚠️ **« Le `<h2>` dans un `<summary>` peut sortir la page du plan de titres » :
RÉFUTÉ, mesuré.** L'arbre d'accessibilité de la page rendue expose bien
`heading "Introduction to the guided path"` à l'intérieur du `<summary>`, pour les
neuf replis. Et le `<summary>` reste un vrai contrôle : premier enfant de son
`<details>`, `tabIndex 0`, un clic bascule l'état (`display: flex` ne change rien à
tout ça). Rien à corriger — c'était un « peut », et sur ce moteur il ne le fait pas.
⚠️ La leçon est la même que pour les compteurs : **une alerte se réfute avant d'être
comptée**, sinon on invente du travail. Deux des trois points de la paire B étaient,
en tout ou en partie, des faux positifs.

🅿️ **Ce qui RESTE, et qui n'est pas un défaut : une décision d'opérateur.** Le
tableau du programme a perdu deux capacités que le `textarea` d'avant avait — coller
huit étapes d'un coup, et réordonner en déplaçant des lignes. Le codeur argumente
l'échange (une table nomme ses colonnes, un `textarea` non). Le choix est le vôtre.

---

## Verdict

**La Phase J tient sur ce qui se voit, une fois ces six défauts corrigés.**

La passe navigateur a sorti **six défauts réels, tous corrigés et déployés**, dont
deux qui cassaient l'usage pour de bon : personne ne pouvait atteindre le
formulaire de connexion depuis le réseau de l'école, et le mur des événements
cachait 902 px de son contenu. Aucun des deux n'était visible dans le balisage —
le premier était un en-tête HTTP, le second une règle CSS absente d'une feuille.

Ce qui reste (R1 → R9) est **du travail identifié, chiffré, et rien n'y est
cassé** : des formulaires trop larges et trop longs, des libellés admin qui ne se
traduisent pas, un vocabulaire à unifier. Ce sont des décisions de design, pas des
correctifs en attente.

🔴 **La seule chose qui empêche encore un membre d'utiliser le produit reste
J-25**, et c'est une décision opérateur : aucun membre ne peut réserver tant
qu'aucun package n'est attribué.
