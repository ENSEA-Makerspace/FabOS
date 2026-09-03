# Phases S158 & S159 — les groupes deviennent le modèle

**2026-09-01 au 2026-09-03.** Parties d'une demande simple de l'opérateur — « des
groupes qui aillent au-delà de ceux qui existent, peut-être en fusionnant les
systèmes » — et finies avec les groupes comme **seule** source des rôles et des
forfaits.

---

## 🔴 Le fait mesuré d'où tout est parti

**`USER_GROUP_MEMBER` n'était écrit par RIEN.** La table existait depuis S133b,
les sept groupes intégrés étaient semés par la migration, et aucune surface ne
permettait de créer un groupe ni d'y mettre quelqu'un. Des sept, seuls comptaient
ceux qu'un **rôle** impliquait et l'audience résolue `user` — et « Stagiaires » ou
« Bénévoles », que `USAGE_RIGHTS_VISION.md` nomme, étaient inatteignables.

C'est la famille de défaut que le dépôt avait déjà nommée un étage plus haut, pour
`USAGE_RIGHT_ASSIGNMENT.groupId` : **un demi-modèle sans surface d'écriture se lit
comme une fonctionnalité et se comporte comme une absence.**

## Ce qui a été livré, dans l'ordre

| | |
|---|---|
| **S158a** | L'écran des groupes. La liste affiche l'appartenance **effective** — sur la boîte, « Administrateur global » 6 et « Staff » 2 avec **zéro** ligne stockée ; un écran qui n'aurait compté que le stocké aurait annoncé « 0 membre » sur des groupes qui ouvrent des droits |
| **S158b** | La même appartenance depuis la fiche du membre, écrite par le même dépôt : deux vues, pas deux surfaces |
| **S158c** | Le backfill rôle → groupe, **11 lignes**, et le filtre « groupe » sur la liste |
| **S159b→f** | La fusion : garde du dernier admin d'abord, entités, `getRoles()` en union puis groupes seuls, écritures déplacées, `UTILISATEUR_ROLE` supprimée |
| **S159 (1,3)** | Un forfait ne s'attribue qu'à un groupe ; les 3 attributions personnelles converties |
| **S159g** | L'appartenance **datée** |

---

## Les leçons, celles qui coûteront cher à réapprendre

🔴 **Un argument de sûreté qui n'est qu'affirmé n'en est pas un.** Le backfill
photographie les audiences de chaque compte avant, écrit, rephotographie avec un
résolveur **neuf**, et annule tout si un seul jeu de clés a bougé. Même discipline
pour la conversion des attributions. Les deux ont conclu « neutre » — mais parce
qu'elles l'ont **mesuré**.

🔴 **Un témoin mémoïsé se prouve lui-même.** `AudienceResolver` mémoïse par
instance : réutiliser celui d'avant l'écriture rend la photo d'avant. Trois
commandes ont dû construire un résolveur neuf pour dire quelque chose.

🔴 **Un témoin mal choisi accuse la mesure.** La sonde du palier de réservation
est tombée sur un ADMIN, dont le palier reste `admin` une fois staff. L'assertion
était fausse, pas le code.

🔴 **Chaque pas de la fusion a créé le défaut du pas suivant**, et c'est le motif
de toute la phase :
- le backfill a rendu « stocké » et « venu d'un rôle » simultanés, donc le bouton
  « Retirer » supprimait la ligne sans sortir la personne du groupe →
  `roleKeysFor()` POSE la question au lieu de la déduire ;
- l'union dans `getRoles()` a laissé `hasRoleNamed()` en arrière, donc `isStaff()`
  disait non pendant que les écrans disaient oui — et le **palier de quota** en
  dépendait ;
- retirer la relation Doctrine a cassé deux requêtes qui la joignaient encore, et
  ni `php -l` ni `lint:twig` ne l'ont vu. **Seul le balayage des routes l'a vu**,
  et seulement parce qu'il compte une page sans statut comme une erreur.

⚠️ **Une garde se pose AVANT d'être nécessaire.** Celle du dernier administrateur
a été écrite alors qu'elle était sans effet — le rôle étant encore la source.
Après la fusion, elle est ce qui empêche un verrouillage.

🔴 **Et une garde ne peut rien contre l'horloge.** D'où le refus d'une date de fin
sur une appartenance `admin` : elle expirerait un jour sans que personne n'ait
rien fait.

⚠️ **`NULL` veut dire « sans limite », des deux côtés.** Les onze lignes du
backfill n'ont pas de dates ; un filtre qui les prendrait pour expirées retirerait
`staff`, `admin` et `trainers` à tout le monde. La clause est écrite **une** fois,
dans `UserGroupSchema`.

---

## Ce que l'opérateur a tranché en route

- **Le mot** : « package » devient **forfait / bundle**, avant que la Phase H ne
  fasse de « package » une chose qu'on achète. Les tables ne bougent pas — la
  correspondance des deux vocabulaires est dans `PROJECT_STATE.md`.
- **Les forfaits ne s'attribuent qu'à des groupes.** Et quand j'ai objecté que le
  commerce ne rentrait pas — un achat est individuel, et la vision interdit qu'un
  remboursement révoque « un droit venu d'un groupe » — la réponse est venue de
  lui : **on n'achète pas un forfait, on achète une APPARTENANCE datée**. Ce qui
  supprime le cas particulier au lieu de le contourner.
- **Une ligne, plus un journal**, plutôt que plusieurs lignes : moins cher, et
  sans `DROP PRIMARY KEY`. ⚠️ À la condition que la ligne soit **dérivée** du
  journal, sinon ce sont deux vérités de plus.

## La revue de DESIGN qui a suivi (2026-09-03)

L'opérateur a demandé une passe sur `/admin/usage-rights/{id}/edit` — « bien trop
complexe », puis « décousu », puis « ça ne ressemble pas au reste du site ». Elle a
trouvé plus qu'un problème d'apparence.

🔴 **Le résumé en tête de page était une chaîne de traduction FIXE.** « Donne accès
à tout, tout le temps, partout, sans limite », rendu à l'identique sur les QUATRE
forfaits de la boîte — OpenLab compris, qui ouvre deux fonctionnalités le jeudi de
13:30 à 21:00. Présenté comme la ligne de conséquence de l'éditeur, il enseignait le
contraire de ce que l'écran contenait. C'est la famille « demi-modèle » vue par un
autre bout : **un résumé qui ne lit pas ce qu'il résume est une décoration.**

🔴 **Et la première correction a créé le défaut suivant** — encore le motif de la
phase. Le résumé calculé, posé en grille au-dessus de l'éditeur, mettait les quatre
mêmes libellés DEUX FOIS à 120 px d'écart, une fois comme réponse et une fois comme
contrôle. La valeur a fini sur la LIGNE de son axe, à côté de ce qui la règle.

🔴 **« Doit-on pouvoir supprimer un droit compilé ? » — non, sauf quand c'est le
seul chemin.** Supprimer un droit que la carte sait écrire ne le retire pas : la
sauvegarde suivante le réécrit. Les suppressions n'existent donc que pour un forfait
que la carte REFUSE, où elles sont ce qui la rouvre. **L'issue de secours
n'apparaît que quand on est coincé** — et les deux branches ont été mesurées, un
grant `manage` inséré sur OpenLab puis retiré.

⚠️ **Un forfait DÉFINIT des droits ; un groupe dit QUI les a.** L'attribution a
déménagé sur la fiche du groupe, sous ses membres. ⚠️ Vérifié AVANT de retirer :
la fiche du groupe savait seulement COMPTER ses forfaits — retirer l'éditeur sans
construire l'autre côté aurait recréé le demi-modèle que cette phase a réparé.

⚠️ **`color-scheme` n'avait que sa moitié sombre**, depuis toujours. En thème clair
sur un OS réglé en sombre, le navigateur peignait ses propres contrôles — champs,
dates, heures, listes — par-dessus nos fonds blancs, sur TOUT le site. La feuille
disait vrai ; c'est le navigateur qui peignait par-dessus. Une ligne dans
`style.css`.

🔴 **Et la revue de code a trouvé ce que huit commits de vérifications avaient
manqué : l'appartenance datée n'expirait pas les RÔLES.** `AudienceResolver`
filtre la fenêtre en SQL ; `Utilisateur::getRoles()` lit la MÊME table par l'ORM
et ne la filtrait pas — l'entité ne mappait même pas les deux colonnes. Une
appartenance `staff` expirée continuait donc d'accorder `ROLE_STAFF`, donc
`isStaff()`, donc le palier de réservation. Les écrans disaient « plus membre »
pendant que la sécurité disait « toujours staff ».
⚠️ **Deux lecteurs d'une même table dont un seul filtre** — le motif de la phase,
sur le chemin où il coûte le plus cher, et posé par la phase elle-même.
🔴 **Et rien de ce qui existait ne pouvait le voir** : les quatre assertions de
S159g interrogeaient le RÉSOLVEUR, jamais l'entité, et aucune appartenance de la
boîte ne porte de dates. **Une garde datée se vérifie depuis CHACUN de ses
lecteurs, pas depuis celui qu'on vient d'écrire.**
🅿️ Reste connu, mesuré et écrit : les bornes sont l'heure MURALE du labo, et
`getRoles()` les compare à un `now` réel. Le rôle **SURVIT deux heures de trop**
en été — pas deux heures d'avance, ce que j'avais d'abord énoncé à l'envers, et
c'est le sens permissif, donc celui qu'il ne faut pas se tromper à décrire.
`AudienceResolver` a reçu `LabClock` et compare juste ; l'entité ne le peut pas.
Les deux voies sont posées dans `ROADMAP.md`.

🔴 **Et deux leçons d'outillage payées comptant :**
- **`app:render` + grep ne dit rien des pixels.** J'ai passé la moitié de la passe à
  greper du HTML pendant que la plainte portait sur l'apparence. Ce qui a débloqué :
  rapatrier le rendu, réécrire les URLs d'assets, servir en local, REGARDER.
- **`git diff --name-only` ne voit pas un fichier NEUF.** Le partiel extrait n'est
  jamais parti dans le tar et la page a rendu une `LoaderError` en prod. `lint:twig`
  ne l'a pas vu : il vérifie la syntaxe, il ne résout pas les `include`. La liste se
  fait depuis `git add -A && git diff --cached --name-only --diff-filter=d`.

---

## 🅿️ Ce qui reste

- **Le journal des appartenances** — il n'a de sens que le jour où une MACHINE
  écrit. Une table de journal sans écrivain serait un demi-modèle de plus, ce que
  cette phase a passé trois jours à réparer.
- **`ROLE`** n'accorde plus rien mais reste : le `down()` de
  `Version20260902100000` s'en sert pour reconstruire.
- **Les groupes à règle** (« tous ceux qui ont la formation Laser »), avec
  prévisualisation obligatoire. Les groupes imbriqués restent déconseillés.
