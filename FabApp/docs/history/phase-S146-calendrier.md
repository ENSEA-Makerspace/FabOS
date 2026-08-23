# Session 2026-08-20 → 2026-08-21 — S146 (a→g) et S134b

**Le récit.** Ce qui suit a été déplacé depuis `ROADMAP.md` le 2026-08-21 : la feuille de
route ne doit contenir que le travail RESTANT, et 43 % de ses lignes étaient devenues du
récit de travail livré. Les règles qu'une session future doit connaître sont restées
là-bas, condensées, sous « Ce qu'il faut SAVOIR après S146 et S134b » ; l'histoire, avec
les mesures et les fautes, est ici.

**En un paragraphe.** La phase S146 a ramené les deux calendriers à UN composant (a),
mis le calendrier sur la fiche de la ressource (b), rendu `/calendrier` en lecture seule
comme page d'activité d'un lieu (c), lié une séance à sa formation et généré N séances
(d), fait qu'une inscription à une séance inscrit à la formation sans jamais qualifier
(e), ajouté les catégories d'événement (f) et réparé les fermetures datées, mise en page
comprise (g). En parallèle, S134b a clos la Phase G : machine et formation s'archivent,
les deux tables doublonnées sont supprimées, 154 messages flash sont traduits.

**Les migrations passées par l'opérateur** : `Version20260820100000`,
`Version20260821100000`, `Version20260821120000`, `Version20260821130000`,
`Version20260821140000`.

### 🟡 S134b — l'inventaire action-opérateur, fait (2026-08-21)

**Mesuré, pas supposé** : les 111 routes d'administration groupées par objet et par
verbe. Le critère de sortie de la Phase G — « chaque objet annoncé est créable,
éditable, **archivable** depuis son workspace » — était **faux pour deux objets** :
`Machine` et `Formation` se créaient et s'éditaient et ne se retiraient **jamais**.
Une découpeuse revendue l'an dernier restait dans tous les catalogues, tous les
sélecteurs de réservation et tous les calendriers ; la seule sortie était la base.

⚠️ **Et l'audit s'est trompé une fois : les PACKAGES n'étaient pas un manque.** Ils se
retirent déjà par la case `active` du formulaire, affichée en pastille d'état dans la
liste et écrite par le dépôt. Un audit par NOM DE ROUTE ne voit pas un verbe qui est un
CHAMP. ⚠️ La colonne `archivedAt` a donc été ajoutée à `USAGE_PACKAGE` pour rien —
`Version20260821130000` la retire.

✅ **Archivé, jamais supprimé**, et ici ce n'est pas une préférence : `RESERVATION`,
`LOG_UTILISATION` et `ACCESS_RFID_LOG` pointent sur la machine, `PROGRESSION` sur la
formation. Supprimer emporterait l'historique d'usage ou ce que les gens ont fait vers
une qualification.
🔴 **Et masquer n'est pas refuser.** `MachineRepository::findLive()` retire la machine
des surfaces qui la PROPOSENT (catalogue, calendrier, deck d'accueil, API) — mais
`/machines/{id}` répond toujours à qui a le lien, et `/api/reservations` parle le
format directement. `ReservationService` refuse donc la réservation d'une machine
archivée (`MACHINE_ARCHIVED`), au point de passage unique. C'est la règle de la maison :
un lien caché n'autorise ni n'interdit rien.
⚠️ **`findBy()` reste la question de l'ADMIN** : l'archivage doit être visible et
réversible depuis l'écran qui l'a fait.
⚠️ `countVisible()` a reçu la même condition que `findVisible()`, sinon le compteur et
la liste qu'il étiquette répondent à deux questions différentes.

**Vérifié** : 139 tests / 2379 assertions, 122 routes, et **11/11 sur la vraie base**
dans une transaction annulée — la machine quitte la liste vivante (11 → 10), reste dans
`findAll()`, reste joignable par id, **une réservation passée sait encore la nommer**,
sa réservation est refusée côté serveur, la restauration la remet ; la formation quitte
le catalogue (8 → 7) et son compteur suit.

⏭️ **Ce qui reste de S134b** : dette, routes orphelines, traductions, a11y, et la
migration de contract qui supprime `USAGE_GRANT` et `USAGE_PACKAGE_GROUP_ASSIGNMENT`.

### ✅ S134b — traductions et a11y, la fin (2026-08-21)

**a11y : mesurée propre**, et il faut le dire ainsi. Neuf pages rendues (accueil,
catalogues, calendrier, fiches) : **0 image sans `alt`**, **0 bouton sans nom
accessible**, **0 champ sans étiquette**. ⚠️ Mes trois détecteurs ont chacun produit un
faux positif qu'il a fallu réfuter — deux boutons « vides » qui portaient un
`aria-label`, huit `input` « sans étiquette » enveloppés dans un `<label>`. Un audit
d'accessibilité qui ne vérifie pas ses propres alertes invente du travail.

**Traductions : les catalogues étaient déjà à parité** (3 026 clés × 5 langues, 0
manquante). La dette était ailleurs : **154 appels `addFlash()` en français littéral**
dans sept contrôleurs — un admin en anglais recevait une interface entièrement traduite
puis « Affiche mise à jour. ». 124 clés distinctes, traduites dans les cinq langues.

⚠️ **Traduit au RENDU, pas à l'appel** : `|flash_text` (`App\Twig\FlashExtension`).
Traduire là où le message est créé demanderait un `TranslatorInterface` dans 154
actions, et ces contrôleurs injectent par action, pas par constructeur. Deux formes :
`addFlash('success', 'flash.x')` et `addFlash('success', ['flash.x', ['%p1%' => $v]])`.
🔴 **Une clé inconnue traverse inchangée** — le traducteur de Symfony rend l'id qu'on
lui donne — et c'est exactement ce qui a rendu la migration sûre : un flash non converti
se lit encore comme avant. Ne pas « corriger » ça en levant une exception.
⚠️ **Piège de méthode** : la première conversion a produit `..._2`, `..._3` pour des
textes IDENTIQUES, parce qu'elle indexait par site d'appel et non par TEXTE. 154 sites →
**120 clés**, pas 154. Une clé par phrase, pas par endroit.
⚠️ **Et `lint:twig` échouait sur 18 fichiers** tant que `cache:clear` n'avait pas tourné :
un filtre Twig neuf n'existe pas pour le linter avant que le conteneur soit reconstruit.
L'ordre habituel « lint AVANT tout » a une exception ici.

**Vérifié** : les cinq langues rendues pour quatre messages types, paramètres compris
(`Portal “Atelier” deleted, along with 3 setting(s)…`, `Portale «Atelier» eliminato,
con 3 impostazione/i…`), et le passage inchangé d'une phrase non catalguée. 139 tests /
2 379 assertions, 122 routes.

### 🟡 S134b — routes orphelines : la mesure et ses deux vraies trouvailles (2026-08-21)

**Mesuré** : chaque nom de route cherché dans tout `templates/` et `src/`, en écartant
sa propre déclaration. **9 candidates**, dont 7 explicables (une redirection délibérée,
des POST appelés depuis leur propre écran, des pages d'administration atteintes par le
menu). **Deux étaient réelles :**

🔴 **`app_machine_ical` n'était plus lié nulle part — et c'est MOI qui l'ai cassé en
S146b.** Le lien « 📅 S'abonner » vivait dans `machine-calendrier.html.twig`, que S146b
a supprimé, et il n'a pas été reporté sur l'onglet de la fiche machine. `/places/{id}`
avait gardé le sien. La route répondait toujours ; plus personne ne pouvait la trouver.
Restauré et vérifié à l'écran (le lien porte bien un jeton de flux personnel).
⚠️ **La leçon** : supprimer un gabarit emporte les liens qu'il contenait, et ni les
tests ni le balayage de routes ne le voient — la route répond toujours 200.

🟡 **`app_switch_locale` n'est lié nulle part non plus — PARQUÉ par l'opérateur le
2026-08-21 (« do translation later »).** Rien à faire pour l'instant ; ce qui suit est
l'état du dossier pour la session qui le reprendra. C'est une **question produit, pas un
bug à corriger en silence**. `/locale/{locale}` écrit `_locale` en
session et `LocaleSubscriber` le lit, mais aucun gabarit ne pointe dessus. Un membre
change de langue par `Utilisateur.langue` dans son profil ; **un visiteur non connecté
ne peut pas changer de langue du tout**, sur un produit qui en sert cinq. Deux issues,
et c'est à l'opérateur : ajouter un sélecteur de langue (en-tête ou pied de page), ou
supprimer la route et assumer que la langue vient du profil et de l'`Accept-Language`.

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

3. ✅ **La légende dit « Ouvert » sur le calendrier en lecture seule**, « Disponible »
   là où l'on réserve. Une pastille verte est une heure d'OUVERTURE ; sur une page qui
   peut réserver, une case vide est aussi une heure qu'on peut prendre, et c'est ce que
   « Disponible » dit. Sur `/calendrier` rien ne se prend, donc le mot promettait un
   contrôle absent.
   🔴 **Et le premier essai n'a rien changé : `booking|default(true)` est TOUJOURS
   vrai.** Le `default` de Twig se déclenche sur toute valeur *vide*, et `false` est
   vide — donc passer `booking: false` donnait `true`. **C'est exactement le bug que
   `_venue_context.html.twig` documente pour `allow_all`, la deuxième fois qu'il est
   livré dans ce dépôt.** `is not defined` est le test qui distingue « pas passé » de
   « passé à false ». ⚠️ Trouvé en regardant la page, pas en relisant le gabarit : le
   rendu était identique sur les deux pages, ce que seul un `curl` côte à côte montre.

⚠️ **Aucune affordance morte introduite** : le seul `disabled` de la phase est une
`<option>` de ressource verrouillée, qui porte sa raison dans son libellé.

### ✅ S146g — les fermetures datées : mise en page, et une VRAIE plage (2026-08-21)

**Demande opérateur** : *« in the exceptionally closed dates ui looks wrong and i cant
specify a range? »*, puis *« cant do multi day closings? like week off »*.

🔴 **La mise en page : deux systèmes se battaient sur un même formulaire.** Il portait
`.usage-assignment-form .usage-grant-form` — une grille à colonnes fixes écrite pour
l'attribution de packages — autour d'enfants `.afp-select`, qui appartiennent à la
rangée flex `.afp-tier` des écrans de catégories. **Mesuré sur la page** : contrôles de
46, 22, 13, 46 et 46 px, lignes de base écartées de **21 px**, bouton d'envoi seul sur
une deuxième rangée. Après : cinq pastilles **identiques** (38 px, même y), écart
ramené à **4 px**.
🔴 **La cause profonde était dans le composant partagé, pas sur cette page.** La
pastille `.afp-select` n'avait de style que pour `select` : une pastille contenant un
`input` dessinait la boîte du navigateur À L'INTÉRIEUR de la sienne. `date`, `time`,
`text` et `number` sont désormais traités comme `select`. La case à cocher garde sa
boîte — elle EST le contrôle. ⚠️ Et `closed-cell` n'avait **aucune** règle nulle part
(grepé) : le seul contrôle de la rangée sans coquille.
✅ **Ajouté au guide `/admin/design`** (section « La pastille ») avec le vrai composant
et les deux règles : ne pas envelopper une rangée de pastilles dans la grille d'un
autre écran, et si une rangée a besoin d'autre chose, c'est une règle à ajouter au
guide, pas une classe à emprunter.

✅ **La plage : `SCHEDULE_EXCEPTION.endDate`**, nullable, migration
`Version20260821100000` passée par l'opérateur. Une semaine de fermeture est **UNE
ligne**, pas sept.
🔴 **Choix inverse de celui de S146d, et délibéré.** Les séances sont des
*occurrences* — chacune se déplace, se remplit et s'annule séparément, donc lignes
séparées. Une fermeture est une *déclaration unique* : la découper ferait de « annuler
la fermeture » une suppression en masse, c'est-à-dire précisément la plainte à laquelle
ceci répond.
⚠️ **`null` veut dire « un jour », et chaque lecteur le dit avec `COALESCE`** — sinon
toutes les lignes écrites avant S146g cessent silencieusement d'être vues. Les trois
requêtes interrogent la PLAGE : `forDate` teste l'appartenance, `upcomingFor` compare
le DERNIER jour (une fermeture commencée la semaine dernière est toujours en cours), et
`betweenFor` est un **chevauchement**, pas une inclusion — une fermeture débordant des
deux côtés d'une semaine affichée serait autrement invisible.
⚠️ `exceptionsBetween()` **déplie** la plage en une entrée par date et la **borne à la
fenêtre** demandée, sinon un calendrier d'une semaine recevrait des dates qu'il ne
dessine pas.

**Vérifié** : 139 tests / 2379 assertions, et **9/9 sur la vraie base** dans une
transaction annulée — une semaine off créée par le vrai formulaire, une seule ligne
stockée, les 7 jours fermés, le motif présent **au milieu** de la plage, 7 entrées dans
la carte du calendrier, 2 pour une fenêtre de 2 jours, encore listée depuis le milieu
de la fermeture, et **une seule suppression rouvre toute la semaine**.
⚠️ Ce dernier point a d'abord échoué : `ScheduleResolver` **mémoïse par requête** et la
sonde interroge deux fois dans le même processus. Mémos vidés par réflexion — le piège
est déjà consigné, il se represente à chaque sonde.

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

---


# S146 — la proposition et le découpage, déplacés de ROADMAP le 2026-08-21

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
