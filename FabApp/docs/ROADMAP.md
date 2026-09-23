# FabOS — ce qui reste

**MAJ 2026-08-23.** ⚠️ **Cette page ne contient QUE le travail restant.** Ce qui
est livré en sort le jour même : sinon une session le refait — c'est arrivé deux
fois.

- Comment le produit marche → [`PROJECT_STATE.md`](/roadmap)
- Ce qui est livré → [`HISTORY.md`](/roadmap/historique)
- Où on en est cette semaine → [`WORKING_BRIEF.md`](/roadmap/brief)
- Modèle cible des droits → [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage)

**Livré à ce jour** : phases A→F (S102–S128), **toute la Phase G**, la Phase G2,
l'interface S134h–S143, S144, S145a, toute la phase S146, et la revue S147.

**Ordre : J → H → I.** 🔴 **J bloque H.**

---

## L'ordre d'exécution, en une table

⚠️ **Le fichier suit désormais cet ordre.** Écrit le 2026-09-04, quand trois lots
de références et cinq phases neuves ont rendu la lecture linéaire impossible.

| Phase | Quoi | Sessions |
|---|---|---|
| **J** | ✅ **CLOSE le 2026-09-05** — les 25 défauts | S169 |
| **N** | le cleanup, et **J se ferme** | S169–S170 |
| ~~**O**~~ | ~~Machines & boîtiers~~ — ✅ **CLOSE le 2026-09-05** | S171–S174 |
| ~~**P**~~ | ~~Espaces & accès d'entrée~~ — ✅ **CLOSE le 2026-09-06** | S175–S178 |
| **Q** | Formations (absorbe la messagerie de cohorte) | S179–S183 |
| **K** ✅ | Gabarits d'e-mail modifiables | S160–S162 — **close le 2026-09-07** |
| **L** ✅ | Annoncer un événement aux membres | S163–S164 — **close le 2026-09-08** |
| **M** ✅ | Thèmes, en profondeur | S165–S168 — **close le 2026-09-19** |
| **S** | Comptes, adhésion et confiance (MFA, récupération) | S189–S192 |
| **T** | Surfaces restantes : prêts, recherche, rapports, créations | S193–S195 |
| **R** | Commerce — **la dernière**, et bloquée par J | S184–S188 |

⚠️ **R garde ses numéros bas en passant après S et T** : un numéro de session est
une étiquette, pas un rang — la note ci-dessous vaut pour elle aussi.

🔴 **K, L, M et R gardent leurs numéros bas alors qu'elles passent après O–Q** : un
numéro de session est une ÉTIQUETTE, pas un rang. Les renuméroter chaque fois que
l'ordre change ferait mentir chaque commit qui les cite.

## Cap produit

Tout fablab, école ou atelier partagé déploie **les seules fonctions dont il a
besoin**, avec une expérience cohérente.

- une installation, plusieurs **lieux** ; aucun portail ;
- SSO entre instances sans partager droits ni données ;
- sept audiences intégrées protégées + groupes locaux + packages assignables ;
- deux droits, **Use** et **Manage**, par feature / lieu / scope ;
- réservations, quotas et reporting montrés dans chaque feature, moteurs communs ;
- profils publics volontaires, échanges inter-FabOS consentis, badges fédérables ;
- plus tard : Paiements facultatif, puis messagerie Formation ;
- **Configuration → Thèmes** réunit identité visuelle, images, menus et accueil ;
- **un seul** système central de listes, filtres, workspaces, composants et CSS.

---

# Phase J — « boutonner » ✅ CLOSE le 2026-09-05

✅ **Livrée (S147–S169).** La revue S147 a rendu et mesuré 146 pages → 25 défauts
J-1…J-25 ; les 25 sont soldés. La barrière du commerce tombe.
📖 **Le récit, les dix points, le détail des 25 défauts et les mesures →
`docs/history/phase-J-boutonner-S147-S169.md`** (et `S147-REVUE.md` pour la revue
elle-même).

## ⏳ Ce qui reste ouvert de cette phase

- **J-2** — huit objets se supprimaient en dur, corrigé. ⚠️ **Reste à vérifier la
  promesse S134f** : archiver une ressource réservable doit annuler ses
  réservations à venir.
- **Documents attachés à une machine** (livré le 2026-08-28) : 🅿️ **il reste à
  l'opérateur d'y déposer les vrais documents**. Et 🅿️ le jour où un document
  devra être réservé aux membres, il faudra **le sortir de `public/`** — un
  contrôle d'accès devant un fichier que le serveur web sert directement ne
  contrôle rien.
- **Les affiches de remplacement d'événement** : les six par défaut sont livrées
  et rendues. 🅿️ **Reste** de pouvoir téléverser SES propres logos pour qu'ils
  entrent dans le tirage — une table, donc une migration, donc l'opérateur, et ça
  se décide après avoir jugé les six.
  🔴 **Et une question à trancher qui n'est pas graphique** : `/events` passe par
  `_catalogue.html.twig`, partagé avec six autres listes. Un en-tête de mois veut
  dire soit une grille PAR mois (les cartes du dernier mois ne s'alignent plus sur
  le précédent), soit un `grid-column: 1 / -1` dans une grille unique, qui garde
  l'alignement mais demande au shell une notion de « séparateur » qu'aucune autre
  liste n'a. ⚠️ Et le regroupement ne vaut que pour les objets **DATÉS** : une
  machine n'a rien à regrouper.

## 🅿️ Parqué — n'entrait PAS dans J, et reste à faire

Sélecteur de langue (`app_switch_locale` n'est lié nulle part) · suppression en
masse d'événements · catégories comme entrées de menu. Ce sont des
fonctionnalités, pas de la finition.

## ✅ CE QUE L'OPÉRATEUR VÉRIFIE — Phase J (S169)

**Demandé le 2026-09-05 : après chaque phase, la liste de ce que le RELECTEUR
teste.** Elle est délibérément faite de gestes, pas de fichiers : ce que la
machine sait mesurer est déjà mesuré, et ce qu'elle ne sait pas voir est
exactement ce qui suit.
⏳ **Cette liste n'a pas encore été parcourue** — elle reste ici tant qu'elle est
du travail qui attend.

| # | Le geste | Ce qui doit se produire |
|---|---|---|
| 1 | `/profil` → carte **Profil public** → cocher « activer », écrire une bio de deux phrases, choisir des champs, mettre une adresse **invalide** (`!!!`) → Enregistrer | 🔴 Un message d'erreur, **et la bio, les cases et l'adresse TOUJOURS À L'ÉCRAN**. C'est J-8 : avant, tout était vidé |
| 2 | Recommencer avec une adresse **déjà prise** par un autre membre | Même chose : message, et rien de perdu |
| 3 | Puis corriger l'adresse et Enregistrer | Ça passe, et la page revient sur la carte du profil public |
| 4 | `/admin/utilisateurs/{id}` → passer le statut à **Inactif** → se déconnecter → essayer de se connecter avec ce compte | 🔴 **La connexion échoue.** Avant, un compte « inactif » se connectait normalement. ⚠️ Le message est volontairement le même que pour un mauvais mot de passe — il ne doit pas révéler que le compte existe |
| 5 | Remettre le compte à **Actif**, se reconnecter | Ça remarche |
| 6 | Sur le formulaire d'un utilisateur, lire les aides sous **Rôle**, **RFID**, **Numéro interne**, **Statut** | Quatre phrases, et chacune dit une CONSÉQUENCE qu'on ne devine pas au libellé. S'il y en a une que tu trouves fausse, c'est le défaut le plus grave de la phase |
| 7 | `/machines/1` | La page s'ouvre normalement (une dépréciation PHP corrigée en passant) |

⚠️ **Ce que cette liste ne couvre pas, et que j'ai déjà mesuré** : le balayage des
118 routes, les deux sondes, les hachages de déploiement. Inutile de les refaire.

🔴 **Et l'incident à connaître** : pendant cette phase, le site est resté en **500
quelques minutes** — une signature incompatible avec une interface Symfony, que
`php -l` ne pouvait pas voir. Rétabli et vérifié. Si quelque chose te semble
bizarre au chargement, c'est le premier endroit où regarder.

---

# Phase S153 — la saisie, et les propositions qu'on solde

✅ **LIVRÉE le 2026-08-31.** Les quatre chantiers sont faits et les trois
propositions supprimées du guide de style. Le détail vit dans
`history/phase-S153-saisie.md` — il n'a plus rien à faire ici.

🅿️ **Ce qui en RESTE à faire, et c'est de l'opérateur :**

- 🔴 **confirmer, package par package, ce que `fullAccess` autorise désormais.**
  La colonne dit maintenant « aucune restriction, horaires compris », et elle est
  lue par les DEUX modèles de droits. Sur la boîte, « Accès complet » la portait
  déjà : ses porteurs gagnent donc la réservation hors grille hebdomadaire. C'est
  le cas n°1 de la liste de l'opérateur (« Staff — aucune restriction »), mais
  c'est un élargissement d'accès et il se confirme, il ne se suppose pas.
- téléverser SES propres logos pour qu'ils entrent dans le tirage des six
  affiches. Ça demande une table, donc une migration.

🅿️ **Demandé par l'opérateur le 2026-08-31, à faire APRÈS cette phase : filtrer la
liste des utilisateurs par droit d'usage (package).** `/admin/utilisateurs` a déjà
la mécanique de raffinement du shell de liste ; ce qui manque est la jointure sur
`USAGE_RIGHT_ASSIGNMENT` — et ⚠️ elle a **deux** chemins, l'attribution personnelle
et celle par groupe, comme partout ailleurs dans ce modèle. Un filtre qui n'en voit
qu'un afficherait « personne » pour un package donné à une équipe entière.

---

# Phase S158 — les groupes, et le mot 🅿️ À TRANCHER

**Demandée par l'opérateur le 2026-09-01** : « des groupes qui aillent au-delà de
ceux qui existent, peut-être en fusionnant les systèmes — ça rendra l'attribution
des packages plus facile », et « renommer les packages, *Use bundles* ou
approchant ». Ce qui suit est le plan, pas du code.

---

## 🔴 Le fait mesuré qui change la question

**`USER_GROUP_MEMBER` n'est écrit par RIEN.** Pas un contrôleur, pas un service :
la table est créée et les sept intégrés sont semés par `Version20260816130000`,
et c'est tout. Aucun écran ne crée un groupe, aucun n'y met quelqu'un.

Conséquence, aujourd'hui : des sept groupes, seuls comptent ceux qu'un **rôle**
implique (`ROLE_STAFF` → `staff`, etc.) et l'audience résolue `user`. Le
formulaire « attribuer à un groupe » d'un package est donc un contrôle dont la
portée utile se règle ailleurs, sur l'écran des rôles — et « Stagiaires » ou
« Bénévoles », que la vision nomme explicitement, sont inatteignables.

⚠️ **C'est exactement la famille de défaut que ce dépôt a déjà nommée**, un étage
plus haut : `USAGE_RIGHT_ASSIGNMENT.groupId` a vécu deux sessions sans écriture,
et le commentaire de `assignGroup()` le dit — *« un demi-modèle sans surface
d'écriture se lit comme une fonctionnalité et se comporte comme une absence »*.
Ici c'est la table des groupes elle-même.

**Donc : « aller au-delà des groupes existants » commence par pouvoir en créer
un.** Tant que non, tout le reste est décoratif.

## ✅ Et la fusion est déjà DESSINÉE, pas exécutée

`AudienceResolver` le dit dans son en-tête, depuis S133b :

> *« Les rôles amorcent les intégrés, ils ne les remplacent pas. Quand S134
> déplacera la vérité dans les groupes, la moitié rôle sort et rien d'autre n'a
> à changer. »*

Il n'y a donc pas deux systèmes à réconcilier : il y en a **un** — les audiences
— alimenté par trois sources (des lignes stockées, des rôles, l'audience résolue
`user`), et une des trois doit finir par disparaître. Le plan est d'exécuter ça,
dans l'ordre qui ne peut pas casser.

⚠️ **Les rôles Symfony ne disparaissent pas**, et confondre les deux serait le
piège de cette phase. `getRoles()` reste ce dont la sécurité, les voters et
`NavBuilder` se servent. Ce qui sort, c'est l'**amorçage** rôle → audience.

---

## L'ordre, et pourquoi il est dans cet ordre

**1. L'écran des groupes** — ✅ **LIVRÉ le 2026-09-01** (S158a). `/admin/groupes` :
créer, renommer, décrire un groupe libre ; ajouter et retirer des membres.
Aucune migration — les deux tables existaient depuis S133b et personne ne les
écrivait.
✅ **La liste affiche l'appartenance EFFECTIVE**, pas les lignes stockées : sur la
boîte, « Administrateur global » compte 6 et « Staff » 2, alors que les deux ont
**zéro** ligne en base. Un écran qui n'aurait compté que le stocké aurait annoncé
« 0 membre » sur des groupes qui ouvrent des droits.
✅ Sonde d'écriture verte : la ligne arrive en base **et** `AudienceResolver` la
voit — donc un forfait attribué au groupe suit la personne.
⚠️ Les sept intégrés ne sont pas supprimables ; `user` et `guest` sont virtuels et
n'ont jamais de ligne d'appartenance — l'écran doit le DIRE, sinon « 0 membre » se
lit comme une erreur. C'est déjà la règle de `assignmentsForPackage()`.
✅ **Cette étape seule rend réel le formulaire d'attribution par groupe**, et
c'est elle qui répond à « rendre l'attribution plus facile ».

**2. L'appartenance depuis la fiche membre** — ✅ **LIVRÉE le 2026-09-01**
(S158b). `/admin/utilisateurs/{id}` porte ses groupes, chacun disant par où il
passe, et seul le stocké s'y retire.
✅ **Une seule source d'écriture** : le même `UserGroupRepository`, les mêmes
gardes, le même jeton. Deux vues, pas deux surfaces — un second chemin aurait ses
propres refus, et celui des deux qu'on oublie de corriger est celui qui laisse
passer. La sonde vérifie que les deux vues rendent la même réponse.
🔴 La redirection est FIXE (`app_admin_user_detail`), jamais une cible venue de la
requête : un `?back=` recopié dans un `redirect()` est une redirection ouverte.

**3. Le backfill rôle → groupe.** — ✅ **FAIT le 2026-09-01** (S158c).
`app:s158:backfill-groups`, **11 lignes écrites** : admin 6, staff 2,
formateurs 3.
✅ **L'argument de sûreté est EXÉCUTÉ, pas affirmé.** La commande photographie les
audiences de chaque compte avant, écrit, rephotographie avec un résolveur NEUF, et
**annule tout si un seul jeu de clés a bougé**. Mesuré : les 9 comptes sont
identiques au jeton près. `--write` est obligatoire ; sans lui elle montre le plan.
🔴 **Et le backfill a créé un défaut qu'il fallait réparer avec lui** : « stocké »
et « venu d'un rôle » ne s'excluent plus, donc l'écran offrait un « Retirer » qui
supprimait la ligne sans sortir la personne du groupe — le rôle l'y remettait.
`AudienceResolver::roleKeysFor()` a posé la question au lieu de la déduire.
🔴 **Et cette rustine s'est retournée après le contract** (revue R1, 2026-09-03) :
`getRoles()` DÉRIVANT désormais des appartenances, la question est devenue
circulaire et répondait toujours oui — plus aucun bouton « Retirer » sur les
groupes intégrés. La méthode est SUPPRIMÉE : il n'y a plus de source à distinguer.

**4. Le contract : la moitié rôle sort de `compute()`.** 🅿️ **PROCHAINE ÉTAPE, et elle demande une DÉCISION.**
🔴 **Seulement après une passe d'ombre qui prouve, compte par compte, que les deux
moitiés disent la même chose.** C'est le protocole de `/admin/usage-rights/shadow`
et il existe déjà. Retirer l'amorçage avant, c'est retirer `staff` à tout le monde
en silence.
⚠️ Et il faut décider ce que devient le lien : un rôle donné après le contract
n'inscrit plus dans le groupe. Soit l'écran des rôles écrit les deux (couplage
explicite), soit les deux divergent volontairement. **À trancher.**

**5. Alors seulement, « au-delà des groupes existants ».** Ce qui rendrait
l'attribution vraiment facile, et ce que ça coûte :

| Idée | Ce que ça donne | Ce que ça coûte |
|---|---|---|
| **Groupes à règle** — « tous ceux qui ont la formation Laser », « tous ceux dont l'abonnement court » | L'attribution devient automatique : plus personne à ajouter à la main | 🔴 Une règle fausse ouvre un droit à une population entière, en silence. Exige une **prévisualisation obligatoire** (« cette règle vise 34 comptes, les voici ») avant enregistrement, et une relecture à chaque évaluation — jamais une copie figée, la leçon de « horaires d'ouverture » en S149 |
| **Appartenance datée** — membre du 1er sept. au 30 juin | Les cohortes scolaires s'expriment enfin | Une colonne sur `USER_GROUP_MEMBER` (migration additive). ⚠️ `AudienceResolver` devient dépendant de l'instant : sa mémoïsation par requête reste juste, une mémoïsation plus longue ne le serait plus |
| **Groupes imbriqués** | « Tous les encadrants » = staff + formateurs | 🔴 Le risque de cycle, et un calcul d'appartenance qui n'est plus une jointure. **Je le déconseille** tant que les règles ci-dessus ne sont pas livrées : elles couvrent le même besoin sans graphe |

🅿️ **Décision opérateur attendue** : lesquelles des trois, et dans quel ordre.
Ma recommandation : **appartenance datée d'abord** (petite, sûre, et les cohortes
sont le vrai besoin d'un lab scolaire), **groupes à règle ensuite**, **imbriqués
jamais**.

---

## Le mot — ✅ FAIT le 2026-09-01

Livré : **forfait / bundle / Nutzungspaket / plan / forfait**, et le menu lit
désormais « Droits d'usage → Forfaits ». Les tables, les identifiants et l'URL
`/admin/usage-rights` n'ont pas bougé — la correspondance entre les deux
vocabulaires est écrite dans `PROJECT_STATE.md`, parce que chercher « forfait »
dans le code ne rend rien. Ce qui suit est le raisonnement, gardé.

**Renommer est une affaire de VOCABULAIRE, pas de schéma.** `USAGE_PACKAGE` et
ses tables ne bougent pas : renommer une table est une manœuvre expand/contract
complète pour zéro bénéfice visible, et l'agent ne peut pas migrer. Le travail
est de **~56 chaînes françaises** et leurs quatre traductions.

✅ **Et il y a une raison de le faire MAINTENANT**, plus forte que le goût : la
Phase H apporte des offres, des commandes et des paiements. Dans ce vocabulaire,
« package » voudra dire *une chose qu'on achète*. Le libérer avant que le commerce
n'arrive coûte 56 chaînes ; après, ça coûtera une ambiguïté permanente.

⚠️ **Un défaut à corriger au passage** : la section du menu s'appelle
« Packages », le modèle s'appelle « droits d'usage », l'URL est
`/admin/usage-rights` et le document de vision `USAGE_RIGHTS_VISION.md`. **Deux
mots pour une chose** — exactement ce que la revue R5 reproche aux états de
machine. Le renommage doit refermer ça, pas en ajouter un troisième.

⚠️ **Pas via `VocabularyTranslator`.** Ce service porte les noms propres de
l'installation (`%venue%`, `%org%`) ; le nom d'un objet du produit n'est pas un
réglage par lab, sinon deux installations ne parlent plus la même langue dans la
documentation.

**Candidats, et ce que chacun dit :**

| FR | EN | Ce que ça évoque | Contre |
|---|---|---|---|
| **Forfait d'usage** | Use bundle | Ce qu'on donne à quelqu'un pour qu'il puisse faire des choses | Un peu long en tête de menu |
| Bundle d'usage | Use bundle | Fidèle au mot de l'opérateur | Anglicisme dans un admin français |
| Droits d'usage | Usage rights | Déjà l'URL, le document de vision et le nom du modèle | Ne distingue pas le MODÈLE (le forfait) de son EFFET (les droits) — et c'est justement la distinction qui manque |

🅿️ **Ma recommandation : « Forfait » en français, « Bundle » en anglais**, avec la
section du menu qui reste « Droits d'usage » et l'entrée qui devient
« Forfaits ». On lit alors : *« Droits d'usage → Forfaits »*, où le titre dit le
domaine et l'entrée dit l'objet — et « un forfait donne des droits » est une
phrase vraie, ce que « un package donne des packages » n'était pas.
⚠️ L'URL `/admin/usage-rights` **ne change pas** : elle nomme le domaine, elle est
juste, et casser des liens pour du vocabulaire est le mauvais échange.

---

# Phase S159 — les forfaits ne s'attribuent qu'à des GROUPES 🅿️ À FAIRE

**Idée de l'opérateur, 2026-09-01.** Supprimer le doublon : aujourd'hui un forfait
s'attribue **soit** à une personne, **soit** à un groupe, et tout lecteur doit
poser les deux questions. Un seul chemin — le groupe — rendrait la gestion des
droits plus simple à tenir.

---

## ✅ Ce qui rend l'idée viable MAINTENANT, et c'est mesuré

**Les quatre chokepoints sont déjà sur grants v2** (`usage_rights_v2_machines`,
`_places`, `_person_booking`, `_events` = 1). Le lecteur qui décide vraiment est
donc `UsageGrantRepository::paths()`, et **il gère les deux chemins** : une
attribution par groupe est pleinement vivante aujourd'hui.

⚠️ Ce n'était pas acquis. `UsagePackageRepository::grantingPackages()` — le lecteur
v1 — ne regarde que `a.userId` : sur une installation dont un chokepoint serait
resté en v1, un forfait donné à un groupe n'accorderait **rien**. La v1 n'est plus
consultée ici, mais elle reste dans le code et cette limite est réelle pour toute
autre installation. **À vérifier avant de porter cette phase ailleurs.**

## Ce que le doublon coûte, en une ligne

`reachOf()`, `hasUnrestrictedAccess()`, `UsageGrantRepository::grantRows()` et le
filtre « droit d'usage » posent tous la même double question — écrite quatre fois
en une session (S153c, S158c). Et « pourquoi ai-je ce droit ? » a deux réponses
possibles là où une suffirait : *parce que tu es dans ce groupe*.

## 🔴 Le seul point qui BLOQUE un modèle groupes-seuls

**Les dates de validité sont portées par l'ATTRIBUTION, pas par le groupe.**
`USAGE_RIGHT_ASSIGNMENT.validFrom / validUntil`. Mesuré sur la boîte : le forfait
*OpenLab* est attribué à Alvaro **jusqu'au 2029-09-01**.

En groupes-seuls, cette date s'applique au **groupe entier** — ce qui est juste
pour une promo (« les BUT2 jusqu'au 30 juin ») et **faux pour une personne**.
Supprimer le chemin personnel sans **appartenance datée** retirerait donc une
capacité qui est utilisée aujourd'hui. C'est la dépendance, et elle est dure.

## ⚠️ Ce qu'il faut assumer

- **Une exception à une personne devient un groupe d'une personne.** C'est plus
  honnête qu'une attribution invisible — mais la liste des groupes devient un
  fourre-tout si personne ne la range. À surveiller, pas à empêcher.
- 🔴 **Ça change le poids de la décision du contract** (§ Phase S158, étape 4).
  Si les forfaits ne passent que par les groupes ET que le rôle n'inscrit plus
  dans le groupe, alors donner `ROLE_STAFF` à quelqu'un ne lui donne **plus aucun
  forfait**. Les deux décisions ne sont plus indépendantes : **trancher le
  contract AVANT cette phase.**

---

## L'ordre, et pourquoi

**1. Retirer le formulaire « attribuer à un membre » de la fiche forfait.**
✅ **FAIT le 2026-09-02.** La fiche d'un forfait ne porte plus que trois
formulaires : son identité, ce qu'il autorise, et l'attribution à un GROUPE.
✅ **Le lecteur reste tolérant, et c'est mesuré** : `?package=1` rend toujours
Cédric et `?package=21` toujours Alvaro, par leurs attributions personnelles
existantes — qui gardent aussi leur bouton de révocation, sans quoi on aurait
supprimé le seul moyen de défaire ce que l'écran avait laissé faire.
⚠️ La colonne `userId` reste : c'est le chemin de ce qu'une MACHINE écrit.
⚠️ La liste des attributions et sa révocation RESTENT : c'est le seul endroit qui
dise ce qui est réellement en base, et l'issue de secours. Même partage qu'en
S153b.

**2. L'appartenance datée** — ✅ **PRÉPARÉE le 2026-09-02** (S159g). Code déployé
et tolérant, migration `Version20260902160000` en attente.
✅ Deux colonnes nullables, purement additives ; `UserGroupSchema` les sonde et,
sans elles, une appartenance est sans limite — le comportement d'aujourd'hui.
🔴 `NULL` vaut « sans limite » des DEUX côtés, et la clause est écrite UNE fois :
prendre les onze lignes du backfill pour expirées retirerait `staff`, `admin` et
`trainers` à tout le monde.
⚠️ `AudienceResolver` dépend désormais de l'INSTANT : sa mémoïsation par requête
reste juste, une mémoïsation plus longue ne le serait plus.
⚠️ Une appartenance `admin` ne peut pas expirer — la garde du dernier
administrateur juge une écriture, pas l'horloge.
🅿️ **Le JOURNAL n'est pas fait, et volontairement** : il n'a de sens que le jour
où une MACHINE écrit — une commande dont le remboursement doit retirer exactement
ce qu'elle a donné. Tant que seul un humain écrit, une ligne datée suffit, et une
table de journal sans écrivain serait un demi-modèle de plus.

**3. Convertir les attributions personnelles en groupes** — ✅ **FAIT le
2026-09-03** (`app:s159:convert-assignments`). Les **trois** lignes de la boîte
sont devenues trois groupes — `acces-complet-24-7-365`, `prof`, `openlab` — et
**plus une seule attribution personnelle ne subsiste**.
🔴 **La DATE a suivi la personne, pas le groupe** : *OpenLab* courait jusqu'au
2029-09-01 pour Alvaro ; cette date est désormais sur son APPARTENANCE, donc
quelqu'un ajouté au groupe demain n'en hérite pas. C'est exactement pour ça que
l'appartenance datée était une dépendance dure.
✅ Un groupe PAR FORFAIT, contenant qui le tient — réutiliser `staff` aurait donné
le forfait à des gens qui ne l'avaient pas. Vérifié après coup : `?package=1` rend
toujours Cédric, `2` Tolga, `21` Alvaro.

🅿️ **Le retrait du chemin `userId` n'est PAS fait, et ne doit pas l'être** : c'est
le chemin de ce qu'une machine écrit, et le commerce en a besoin. Voir l'effet de
bord plus haut.
⚠️ Sur la boîte : **3 lignes** écrites à la main (forfaits 1, 2 et 21). La
conversion doit être une commande avec un plan et une vérification avant/après,
comme `app:s158:backfill-groups`.

🅿️ **Décision opérateur attendue avant de commencer** : le contract de la Phase
S158 (le rôle inscrit-il encore dans le groupe ?), parce que la réponse change ce
que cette phase-ci provoque.

---

## 🔴 L'EFFET DE BORD SUR LE COMMERCE, et c'est le cas qui n'est pas géré

**Un achat est individuel par nature.** La Phase H vend une ligne « Forfait » à
UNE personne, qui a payé. En groupes-seuls, il n'existe aucun chemin pour la lui
donner — sauf à fabriquer un groupe d'une personne **par acheteur**, c'est-à-dire
une liste de groupes qui grossit à chaque vente.

Et ce n'est pas une gêne d'ergonomie, c'est une contradiction avec un contrat déjà
écrit. `USAGE_RIGHTS_VISION.md` dit, pour la compensation d'une ligne d'achat :

> *« Sa compensation ne peut révoquer que l'attribution portant la source unique
> de cette ligne, **jamais un droit équivalent venu d'un groupe** ou d'une autre
> attribution. »*

Autrement dit : un remboursement doit retirer **exactement** ce que cette
commande-là a donné. Une attribution de groupe ne sait pas faire ça — la retirer
toucherait tous les membres, et ne pas la retirer laisserait un droit payé puis
remboursé. Les deux réponses sont fausses.

⚠️ **Et la colonne qui manque le confirme** : `USAGE_RIGHT_ASSIGNMENT` porte
`id, packageId, userId, groupId, validFrom, validUntil, issuedById, createdAt,
revokedAt, revokedById` — **aucune `source`**. La « source d'attribution unique »
que S152 exige n'existe pas encore ; elle se posera sur une attribution
individuelle, pas sur un groupe.

### ✅ LA RÉPONSE, et elle vient de l'opérateur (2026-09-01)

> *« Pourquoi ne pas acheter le fait de FAIRE PARTIE d'un groupe ? Le groupe*
> *« week-end » a le droit d'utiliser le lab le week-end ; UserA paie, et entre*
> *dans le groupe pour X temps. »*

**Ça règle le problème, et mieux que la solution ci-dessus** — parce que le
commerce cesse d'être un cas particulier. Ce qui se vend n'est pas un forfait
attribué à quelqu'un, c'est **une appartenance datée**. Il reste alors **un seul
chemin pour tout le monde** : humains et machines écrivent la même chose, une
ligne d'appartenance.

Trois conséquences, toutes bonnes :

- ✅ **L'expiration cesse d'être un problème**, elle devient le sujet.
  L'abonnement EST l'appartenance datée — la dépendance dure de l'étape 2 n'est
  plus un obstacle à contourner, c'est le mécanisme.
- ✅ **La compensation redevient exprimable.** Ce que la commande a créé est une
  ligne d'appartenance, précise et nominative ; la rembourser la retire. On ne
  révoque plus « un droit venu d'un groupe » — on retire l'entrée DANS le groupe,
  ce que la vision n'interdit pas.
- ✅ **Les quotas suivent sans rien changer.** Une allocation est comptée PAR
  PERSONNE (`UsageAllowanceRepository::activeFor($user)`) : dix membres d'un
  groupe vendu « 10 h par mois » ont chacun leurs 10 h, ils ne les partagent pas.

### 🔴 Mais la difficulté se DÉPLACE, elle ne disparaît pas

✅ **D'abord, ce qui n'est PAS un problème, parce que la question s'est posée :
une personne appartient à AUTANT DE GROUPES qu'on veut.** C'est le cas depuis
S133b, la vision le dit (« Un compte peut appartenir à plusieurs groupes »), et le
lecteur le fait : `ug.groupKey IN (:keys)` compare l'attribution à **toutes** les
clés de la personne, et les grants se combinent en OU.

> *Acheter l'accès week-end ET le jeudi soir = deux groupes, deux lignes
> d'appartenance, les deux droits.* Ça marche aujourd'hui, sans rien changer.

**Ce que la clé primaire `(groupId, userId)` empêche est bien plus étroit** :
**deux raisons SIMULTANÉES d'être dans le MÊME groupe.** Vérifié en base. Le cas
n'apparaît que si un même groupe peut être à la fois donné et vendu :

> quelqu'un que l'opérateur a ajouté à la main au groupe « week-end », **et** qui
> achète ensuite ce même groupe, n'a qu'**une seule ligne**. Le remboursement la
> supprimerait — et lui retirerait du même coup l'appartenance que l'opérateur
> lui avait donnée.
>
> Même famille : un **renouvellement anticipé**, acheté avant la fin du
> précédent — deux commandes, un seul créneau de ligne.

C'est le défaut que la vision décrit (« révoquer un droit qui ne vient pas de
cette ligne »), reparu un étage plus bas.

### ✅ LA FORME RETENUE (opérateur, 2026-09-02) : une ligne, et un JOURNAL

> *« Garder une seule ligne en repoussant la date de fin fonctionne si on garde la*
> *trace de chaque modification dans des logs. User1 renouvelle avant la fin d'un*
> *abo d'un mois, on ajoute le mois suivant ; il annule, on voit sa date de*
> *commande et sa durée dans les logs. »*

**C'est la bonne forme, et elle coûte MOINS cher que ce que j'avais recommandé.**
`USER_GROUP_MEMBER` garde sa clé `(groupId, userId)` : **plus de `DROP PRIMARY
KEY`, plus d'étape de contract.** Tout devient additif —

- deux colonnes de dates sur `USER_GROUP_MEMBER` (`validFrom`, `validUntil`),
  aujourd'hui absentes : la table porte `groupId, userId, addedAt` et rien d'autre ;
- une table de journal, neuve, qui n'existe pas encore.

🔴 **MAIS À UNE CONDITION, et c'est toute la différence entre les deux lectures de
l'idée : la ligne doit être DÉRIVÉE du journal, jamais modifiée en place.**

Si le journal n'est qu'une *trace* posée à côté d'une ligne qu'on édite, on a deux
vérités — et c'est la ligne qui décide pendant que le journal a l'air juste. C'est
exactement le défaut que toute cette phase a poursuivi (`fullAccess` à 1 ET
quatorze grants). Si au contraire `validUntil` est **recalculée** depuis les
entrées à chaque écriture, alors ce n'est plus « une ligne plus un journal », c'est
**un journal avec sa réponse mise en cache** — une seule vérité, et un cache qu'on
peut toujours reconstruire.

✅ Et c'est un vrai avantage sur des lignes multiples : le lecteur ne change pas.
`AudienceResolver` continue de faire une jointure sur une ligne par (personne,
groupe).

⚠️ **Chaque entrée du journal porte SES PROPRES dates, pas une durée.** Le cas qui
le prouve : il renouvelle (+1 mois), puis se fait rembourser le **premier** mois,
déjà consommé. Retrancher « un mois » à la fin lui retirerait le mois qu'il a payé
et gardé. Recalculer depuis les entrées non remboursées donne la bonne réponse, et
elle seule.

⚠️ **Ce que le journal doit contenir** pour que le recalcul soit possible :
`(personne, groupe, du, au, source, acteur, quand, révoquée)`. La `source` est
nulle quand un opérateur écrit, et porte la clé de la ligne de commande quand une
machine écrit — c'est elle qui rend un remboursement chirurgical, et c'est elle qui
protège une appartenance donnée à la main d'être emportée par l'annulation d'un
achat.

🔴 **Le piège au moment d'ajouter les dates** : `AudienceResolver::storedKeysFor()`
lit aujourd'hui **sans aucun filtre de date**. Y ajouter un filtre doit traiter
`NULL` comme « sans limite », sinon les 11 lignes du backfill — qui n'ont pas de
dates — disparaîtraient d'un coup, et avec elles les audiences `staff`, `admin` et
`trainers` de tout le monde. Expand, toujours : le filtre est permissif sur ce
qu'il ne sait pas.

⚠️ **Et la durée appartient à l'OFFRE, pas au forfait.** « X temps » ne peut pas
vivre sur le forfait, sinon le même groupe ne peut pas se vendre au mois ET à
l'année. C'est la ligne de commande qui dit combien de temps l'appartenance dure.

🅿️ **Ce que ça change pour la Phase H** : une offre ne vend plus « un forfait »
mais « une appartenance à un groupe, pour une durée ». Le fulfillment devient une
seule opération — écrire une ligne d'appartenance datée et sourcée — au lieu de
deux chemins à tenir d'accord.

---

## 🅿️ ET SI LE RÔLE DEVENAIT UN GROUPE QU'ON NE PEUT PAS SUPPRIMER ?

**Question de l'opérateur, 2026-09-02.** Elle remplace avantageusement la décision
du « contract » de la Phase S158, qui devient sans objet : les rôles ne
*produisent* plus des groupes, **ils en sont**.

### ✅ Le cumul, d'abord : ça marche déjà, exactement comme décrit

> *« UserA est dans le groupe prof, peut-être ajouté au groupe staff et au groupe*
> *week-end. Il cumule les droits des différents groupes aux horaires concernés. »*

C'est le comportement actuel, sans une ligne à écrire : les grants se combinent en
**OU**, chaque groupe apporte les siens, et chacun porte ses propres fenêtres
hebdomadaires.

⚠️ **Une seule subtilité, et elle est réelle** : une réservation doit être couverte
**entièrement par UN grant**, pas par un assemblage de plusieurs
(`GrantWindowSet::covers()` reçoit « les fenêtres d'UN grant »). Donc si le groupe
week-end ouvre samedi 9 h–12 h et le groupe prof samedi 12 h–18 h, une réservation
de 11 h à 13 h est refusée par les deux. Ce n'est pas un défaut — c'est ce qui
empêche un patchwork de fenêtres d'ouvrir une plage que personne n'a accordée —
mais ça se dit à l'opérateur, sinon il croira à un bug.

### Ce que la fusion simplifie

- **Un seul endroit pour ranger quelqu'un.** Aujourd'hui on pose un rôle sur la
  fiche ET on gère des groupes : deux écrans pour une idée.
- **`AudienceResolver::compute()` perd sa table rôle → clé.** L'union à trois
  sources retombe à deux : les lignes, et l'audience résolue `user`. C'est le
  « contract » de S158, rendu trivial.
- **Un seul vocabulaire.** Plus de « rôle ou groupe ? » à l'écran comme dans le code.
- ✅ Et les cinq intégrés *sont déjà* les rôles (`admin`, `manager`, `staff`,
  `superuser`, `trainers`). La fusion ne crée rien : elle retire un doublon.

### 🔴 Ce qu'elle complique, et le coût est concentré là

**`Utilisateur::getRoles()` est sur le chemin de la SÉCURITÉ.** Symfony l'appelle à
chaque requête, et il lit aujourd'hui la relation Doctrine `utilisateurRoles →
Role`. Or **`USER_GROUP` n'est pas une entité** — la table est en DBAL pur
(`UserGroupRepository`, `AudienceResolver`). Faire lire les groupes à `getRoles()`
demande donc d'abord d'en faire une entité, ou de renoncer à ce que les rôles
soient dérivables du compte seul. **C'est le vrai coût, et il n'est pas
cosmétique** : un `getRoles()` qui échoue, c'est une session sans droits.

🔴 **Et un DANGER qu'il faut traiter AVANT, pas après.** `AccountGuard` protège le
dernier administrateur — mais il s'exprime en `getRoles()` et il garde
l'anonymisation, **pas le retrait d'un groupe**. Aujourd'hui c'est sans
conséquence : retirer quelqu'un du groupe `admin` ne lui retire pas `ROLE_ADMIN`,
puisque le rôle est la source. **Après la fusion, ce serait un verrouillage hors
de sa propre installation** — et l'écran des groupes livré en S158a n'a aucune
garde de ce genre.

⚠️ Autre changement de sens à assumer : la fiche d'un membre porte **un** rôle
(un `<select>`) ; une appartenance est **multiple**. « Quel rôle a cette
personne ? » cesse d'avoir une réponse unique. C'est une simplification du modèle
et un changement de l'écran.

### ✅ FAIT le 2026-09-02, dans cet ordre

1. ✅ **La garde du dernier admin sur le retrait de groupe** — posée avant d'en
   avoir besoin, donc sans effet visible au moment où elle a été écrite.
2. ✅ **`USER_GROUP` / `USER_GROUP_MEMBER` deviennent des entités**, sans
   migration, en lecture seule.
3. ✅ **`getRoles()` rend l'UNION**, puis **les groupes seuls** — chaque pas
   mesuré par une passe d'ombre compte par compte, neutre à chaque fois.
4. ✅ **Les écritures passent aux groupes** : les cases « staff »/« formateur », la
   création d'un compte, et l'inscription publique — qui écrivait une ligne
   `ROLE_USER` **déjà redondante**, `getRoles()` l'accordant sans ligne.
5. ✅ **Les lecteurs restants suivent** : le filtre « rôle » de la liste (double
   emploi avec « groupe »), les annuaires Équipe et Formateurs, la colonne staff
   de l'accueil, les rôles assignables des réglages.
6. 🅿️ **La migration `Version20260902100000` reste à jouer** — elle supprime
   `UTILISATEUR_ROLE`. Son `down()` reconstruit les lignes depuis les groupes.

⚠️ **`ROLE` n'est pas supprimée** : elle n'accorde plus rien, mais le `down()` de
la migration en a besoin pour reconstruire. La retirer est un ménage à part.

🅿️ **Mon avis** : oui, c'est la bonne destination — c'est celle que la vision
décrit déjà — et elle simplifie plus qu'elle ne complique. Mais la complication
qu'elle apporte est sur le chemin de la sécurité, donc elle ne se fait pas en
passant : elle mérite sa propre phase, après S159.

---

## Le nettoyage — ce qui reste de temporaire à l'écran

**Demandé par l'opérateur le 2026-09-01**, dans le même mouvement : ne garder que
ce qui EST. Inventaire, et ce que je propose pour chacun.

✅ **FAIT le 2026-09-02.** Les quatre écrans marqués « supprimer » ci-dessous sont
partis — routes, gabarits, entrées de menu et libellés. `/admin/design/structure`
est parti aussi : les trois redirections héritées des portails visent désormais
`/admin/lieux`, qui est ce qui a remplacé les portails et répond donc à la
question. Le service `UsageRightsShadow` et `UsageRightsService::legacyPackages()`
sont supprimés avec eux, devenus orphelins.

🔴 **Et le retour en arrière de la v2 est parti avec l'écran d'ombre, délibérément.**
`moveChokepoint()` savait remettre une capacité sur la v1 — mais le lecteur v1 ne
regarde que `a.userId` et ne voit pas les attributions par GROUPE. Rétrograder
aujourd'hui retirerait en silence les droits de tous ceux qui les tiennent d'un
groupe : une issue de secours qui casse ce qu'elle devait sauver. Le réglage
`usage_rights_v2_<capacité>` existe toujours en base, pour une écriture explicite.

| Écran | Ce que c'est | Proposition |
|---|---|---|
| `/admin/design/droits-quotas` | maquette « Accès & responsabilités ». Porte `prototype_notice` : *« Cette maquette… n'enregistre rien »* | 🔴 **supprimer** — les droits sont construits, la maquette décrit ce qui aurait pu être |
| `/admin/design/workspaces` | maquette des workspaces | 🔴 **supprimer** |
| `/admin/design/structure` | maquette de la structure | ⚠️ **à trancher** : trois redirections permanentes pointent dessus (`AdminController` ~2675, 2717, 2732). La supprimer casse ces liens ; il faut les rediriger ailleurs d'abord |
| `/admin/usage-rights/shadow` — « Aperçu grants v2 » | l'écran à lire AVANT de basculer un chokepoint | 🔴 **supprimer** : mesuré, **les quatre sont déjà basculés** (`usage_rights_v2_*` = 1). Sa raison d'être est épuisée, et il montre une comparaison qui n'a plus de second terme |
| `/admin/design` | le guide de style : ce qui EST, mesuré au navigateur | ✅ **garder** — c'est la référence du design system, pas une maquette |
| `/roadmap`, `/roadmap/historique`, `/roadmap/droits-usage` | le plan, rendu depuis le dépôt | ✅ **garder** — c'est là qu'il vit |
| `/admin/pages-manquantes` | outil de développement, derrière le drapeau | ✅ **garder**, il n'est visible qu'en mode développement |

⚠️ **La règle qui s'applique**, la même qu'en S153 : *une proposition implémentée
se SUPPRIME — page, route, lien, section*, et son raisonnement va dans
`HISTORY.md`. Ces trois maquettes sont dans ce cas ; le guide de style et la
feuille de route n'y sont pas, ils décrivent ce qui existe.

✅ **Et le vocabulaire est réglé le 2026-09-03**, la phase étant close.
`USAGE_RIGHTS_VISION.md` s'appelait « vision » et se lisait comme si tout y restait
futur, alors que les groupes, les grants v2 et l'enforcement sont en service : la
même famille de défaut que les écrans qu'on range ici — **deux vérités, dont la
plus visible est la périmée**. Il porte maintenant, en tête, ce qui est CONSTRUIT
(mesuré sur la boîte, pas déduit du plan), ce qui reste à faire, et surtout **les
trois décisions que l'opérateur a changées depuis** : un forfait ne s'attribue
plus qu'à un groupe, le rôle EST devenu un groupe indélébile, et on n'achète pas
un forfait mais une appartenance datée. ⚠️ Au passage, il citait une table
`USER_GROUP_MEMBERSHIP` qui n'existe pas — elle s'appelle `USER_GROUP_MEMBER`.

---


---

# Phase I — messagerie Formation (S155–S157) 🅿️ ABSORBÉE PAR LA PHASE FORMATIONS

⚠️ **Ne pas la planifier séparément** (opérateur, 2026-09-04) : *« attribue dans
les phases correspondantes quand tu les planifieras, ex : messagerie de formation
dans la phase formation »*. Elle attendait déjà le modèle Formation / session /
cohorte — la planifier à côté de la phase qui construit ce modèle produirait deux
plans pour un même chantier, la faute que ce dépôt range en permanence.
✅ Ce qui suit reste le contenu de référence, à reprendre **dans** la phase
Formations quand elle sera écrite.

Très loin après le workspace Formation. FabOS est la source de vérité ; l'e-mail
est une copie et une panne d'envoi ne perd jamais le message interne. Trois
visibilités : annonce formateur→cohorte sans exposer la liste, fil privé, groupe
explicite. **Aucun message privé ne bascule implicitement vers la cohorte.**

| Session | Livre |
|---|---|
| **S155** | conversations, participants, non-lus, permissions |
| **S156** | interface formateur/étudiant + duplication e-mail asynchrone |
| **S157** | modération, archivage, export, rétention |

---

# Phase K — les gabarits d'e-mail deviennent modifiables ✅ CLOSE le 2026-09-07

✅ **Livrée (S160–S162).** Un exploitant réécrit le texte d'un e-mail, par langue,
sans écrire une ligne de Twig ; `_header` et `_footer` se réécrivent une fois pour
les vingt ; et 🔴 une surcharge cassée n'empêche jamais un mot de passe oublié de
partir — mesuré, incident journalisé.
📖 **Le récit, les tensions tranchées et les mesures → `docs/history/phase-K-emails-S160-S162.md`.**

✅ **Plus rien de bloquant : la migration `Version20260907090000` a été lancée par
l'opérateur le 2026-09-07 à 18:30**, service redémarré. 65 sur 65.

## 🅿️ Ce qui reste ouvert

**L'envoi de test n'est pas livré, délibérément.** `sendNow()` existe et
marcherait. Poser un bouton qui envoie du vrai courrier depuis une session
automatisée n'est pas une décision d'agent : ça s'ajoute quand quelqu'un peut le
regarder partir.

## Ce que l'opérateur vérifie — Phase K

⏳ **Cette liste n'a pas encore été parcourue** — elle reste ici tant qu'elle est
du travail qui attend.

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S160** ✅ | nulle part | **Rien n'a changé.** C'est la mesure : le hook retiré puis remis rend 40 mails identiques, table présente et vide |
| **S161** ✅ | `/admin/emails/gabarits` (menu, à côté du compte d'envoi) | 20 gabarits × 5 langues. Une case dit « Texte livré » ou « Réécrit » — ⚠️ un vide n'est PAS un défaut, c'est le cas normal |
| **S161** ✅ | `/admin/emails/gabarits/password_reset/fr` | Les champs proposés à droite sont **ceux de ce gabarit-là** : `{{ resetUrl }}`, `{{ sender_name }}`, `{{ unsubscribe_url }}` — et pas `{{ machine }}` |
| **S161** ✅ | même écran, coller `{{ machine }}` et enregistrer | **Refusé, avec une phrase**, sur le champ concerné |
| **S161** ✅ | même écran, l'aperçu | Rendu par le moteur qui ENVOIE. Les valeurs sont en CAPITALES : c'est un exemple, et l'écran le dit |
| **S161** ✅ | écrire un texte, puis vider les deux champs | Le texte livré revient **au bit près**. Vider = « reviens au texte livré », sans bouton en plus |
| **S162** ✅ | `/admin/emails/gabarits` | Deux lignes EN TÊTE : `_header` et `_footer`. Ce ne sont pas des e-mails — c'est le chrome commun aux vingt |
| **S162** ✅ | `/admin/emails/gabarits/_footer/fr`, écrire « Écrit par {{ sender_name }}. » | Le bas de **tous** les mails français change. ⚠️ L'écran le dit AVANT qu'on écrive |
| **S162** ✅ | le même écran | **Pas de champ Objet** : une partie de chrome n'en a pas. Un seul champ proposé, `{{ sender_name }}` — pas `{{ unsubscribe_url }}` |
| **S162** ✅ | l'aperçu de ce même écran | Le pied réécrit apparaît **avec le lien de désinscription toujours dessous** : il n'est pas déplaçable |
| **S162** ✅ | `/admin/emails/gabarits/password_reset/fr`, coller un objet sur deux lignes | **Refusé, avec une phrase**, sur le champ Objet |
| **S162** ✅ | `/admin/emails`, colonne **Texte** | La colonne existe et s'écrit (migration passée). ⚠️ Les 126 mails déjà partis disent « avant le suivi » : le prochain envoi sera le premier renseigné |
| **S162** ✅ | la sonde, pour ce qui ne se voit pas à l'écran | `php bin/console app:s162:layout-probe` — 26 assertions (29 avec `--log-write`), dont le mot de passe oublié qui part malgré une surcharge cassée |

---

# Phase L — annoncer un événement aux membres ✅ CLOSE le 2026-09-08

**Demandé par l'opérateur le 2026-09-04.** C'est *« Notify members on event
creation »*, **7 votes** chez Fabmanager.

## Ce qui existe déjà, mesuré

- ✅ **`NotificationCategory::NEWS` existe et est DÉSABONNABLE** (`OPTOUTABLE`),
  et `UnsubscribeLinker` sait fabriquer le lien de désabonnement.
- ✅ `Mailer::queueToUser(..., $transactional: false)` respecte déjà le refus.
- `EventMailer` ne sait aujourd'hui qu'accompagner une inscription : inscrit,
  liste d'attente, promu, annulé, événement décommandé. **Aucune diffusion.**

## 🔴 Deux faits du code qui décident de toute la phase

**1. L'annonce doit passer par `NEWS`, jamais par `EVENT`.** `EVENT` n'est pas
dans `OPTOUTABLE` — et c'est juste, puisqu'il porte les mails d'inscription qu'on
ne peut pas refuser. Annoncer sous `EVENT` rendrait l'annonce non refusable ;
rendre `EVENT` refusable ferait perdre à un inscrit la confirmation de sa propre
inscription. La catégorie existe déjà : il n'y a rien à inventer.

**2. 🔴 Il n'y a AUCUN état « brouillon » sur un événement.** Les seuls champs de
cycle de vie sont `cancelledAt` et `archivedAt` : un événement enregistré est en
ligne. Une notification automatique « à la création » annoncerait donc à tout le
labo les événements à moitié saisis, les titres provisoires et les erreurs de
date — **et un e-mail parti ne se rattrape pas.**

✅ **D'où la forme retenue, et elle diffère de la demande d'origine : l'annonce
est un GESTE EXPLICITE, pas un effet de bord de l'enregistrement.** Un bouton
« Annoncer aux membres » sur la fiche de l'événement, qui dit d'abord *combien de
personnes* il va toucher et demande confirmation. C'est plus sûr, et beaucoup
moins cher qu'ajouter un état de publication à tout le modèle.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S163** ✅ | **Livré le 2026-09-08** : la diffusion sous `NEWS`, une file par destinataire dans SA langue, et la trace par événement | ✅ Sonde `app:s163:announce-probe`, 17 assertions, **aucun courrier mis en file** |
| **S164** ✅ | **Livré le 2026-09-08**, dans la même passe : le compte avant envoi, la confirmation, et l'état sur la fiche | ✅ Le compte du bouton (**7**) et celui de la sonde coïncident, mesurés par deux chemins |

### ✅ S163/S164 — l'idempotence est tranchée par la BASE, pas par un `if`

🔴 **La marque est posée par un `UPDATE … WHERE announcedAt IS NULL`, AVANT le
premier envoi.** « Lire `announcedAt`, puis écrire » laisse entre les deux une
fenêtre où deux clics simultanés — ou le double POST d'un navigateur impatient —
passent tous les deux, et le labo reçoit l'annonce en double.
⚠️ **Et la marque reste posée même si les envois échouent ensuite.** C'est
délibéré : entre « quelques membres n'ont rien reçu » et « tout le labo a reçu
deux fois », le second est le défaut le plus difficile à réparer.
✅ Mesuré : deux `claim()` sur un événement jetable, un seul gagne, et le compte
du PREMIER est conservé — un second appel qui ne renvoie pas mais réécrirait le
compte ferait mentir l'écran.

✅ **Sous `NEWS`, jamais sous `EVENT` — et les DEUX moitiés sont mesurées.** Le
cobaye sort de la liste quand il coupe les annonces, **et ses mails d'inscription
passent toujours**. C'est l'invariant qui compte : `EVENT` n'est pas
désabonnable, sinon un inscrit perdrait la confirmation de sa propre inscription
en refusant la publicité.

🔴 **Le compte du bouton est celui RÉELLEMENT mis en file** — mêmes trois filtres
que l'envoi (compte actif, courrier accepté, catégorie non refusée). Annoncer
« 120 personnes » puis n'écrire qu'à 87 ferait chercher une panne d'envoi là où
il n'y a que des membres qui ont dit non.
🅿️ Les comptes anonymisés sortent **sans clause spéciale** : `AccountAnonymiser`
les passe en `inactif`. Une règle « exclure les adresses sentinelles » serait une
seconde vérité à tenir d'accord.

⚠️ **Une fois annoncé, le bouton DISPARAÎT — il ne devient pas gris.** Un bouton
désactivé invite à chercher comment le réactiver ; la phrase qui le remplace dit
ce qui s'est passé et quand. Le geste n'est pas répétable, et c'est ce qui rend
l'idempotence lisible plutôt que subie.
✅ Vérifié à l'écran avant redémarrage : l'événement futur porte « Announce to 7
members », le passé porte « Not announceable… » et **zéro** occurrence de
`event_announce` — pas de formulaire caché, pas d'affordance morte.

🅿️ **La sonde n'appelle PAS `announce()`, et c'est ce qui structure le fichier.**
Un appel écrirait pour de vrai à tous les membres ; une sonde qui déclenche
l'effet qu'elle mesure n'est pas une sonde. Elle mesure donc les trois pièces
isolément — les filtres basculés puis remis, la course sur un événement jetable
créé **archivé et annulé** puis supprimé, les quatre refus en mémoire.
🅿️ **Ce qui n'est donc pas mesuré, dit franchement** : la boucle d'envoi. Trois
lignes, aucune branche.

🅿️ **Aucune annonce réelle n'a été envoyée.** Le premier vrai envoi écrira à 7
personnes ; c'est un geste sortant, il revient à l'opérateur.

## Ce que l'opérateur vérifie — Phase L

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S164** | `/admin/events/11/edit`, section « Annoncer aux membres » | Un bouton **« Annoncer à 7 membres »** — le compte, pas un verbe seul |
| **S164** | `/admin/events/1/edit` (événement passé) | **Aucun bouton.** Une phrase dit pourquoi : annulé, archivé, ou déjà commencé |
| **S163** | cliquer, confirmer | 🅿️ **Envoie 7 vrais e-mails.** Ensuite le bouton disparaît et la ligne dit « Annoncé le … à 7 membres » |
| **S163** | recharger la page après l'annonce | Le bouton ne revient pas. Un second POST est refusé par la base, pas par l'écran |
| **S163** | un membre, préférences → couper les annonces | Il sort du compte du bouton, **et garde** ses confirmations d'inscription |
| **S163** | la sonde, pour ce qui ne se voit pas | `php bin/console app:s163:announce-probe` — 17 assertions, aucun courrier mis en file |

🅿️ **Ce qui n'est PAS dans cette phase, volontairement** : le *digest* périodique
(« un résumé hebdomadaire des événements à venir »), que la demande d'origine
mentionne aussi. Il est moins intrusif et sans doute meilleur — mais c'est une
autre mécanique (planification, fenêtre, regroupement) et il n'a de sens qu'une
fois la diffusion unitaire éprouvée.

⚠️ **Indépendante de la Phase K.** Si K est livrée d'abord, le texte de l'annonce
est modifiable sans travail supplémentaire ; sinon il vit en clés de traduction
comme les 23 autres.

---

# Phase M — les thèmes, en profondeur ✅ CLOSE le 2026-09-19

**Demandé par l'opérateur le 2026-09-04.** Reprend le chantier « Thèmes » qui
traînait sans plan (voir plus bas, section conservée pour le détail des
intentions).

## Ce qui existe déjà, mesuré

- ✅ **Le cycle brouillon → aperçu → publication → abandon EXISTE**
  (`ThemeManager`, `/admin/themes`), sur **quatre** valeurs : `orgName`,
  `venueLabel`, `primaryColor`, `logoPath`.
- 🔴 **Mais `logoPath` est une CHAÎNE : un nom de fichier**, validé par une regex,
  qui doit déjà se trouver dans `public/images/`. Rien ne téléverse. Mettre un
  logo demande donc un accès au serveur — ce qui n'est pas un thème, c'est un
  déploiement.
- ⚠️ **Mesuré le 2026-09-04 : `site_logo_path` n'a AUCUNE ligne en base**, donc le
  site sert le `Logo_ENSEA.png` codé en dur. Le mécanisme marche, il n'a
  simplement jamais servi.
- ⚠️ Le nom est périmé : la fonction Twig s'appelle `portal_logo_path()` et le
  commentaire de `_logo.html.twig` renvoie à l'« écran Portails », supprimé.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S165** ✅ | **Livré le 2026-09-10** : la médiathèque d'identité. Téléversement depuis l'écran, nommage serveur, identifiant stable, suppression refusée tant qu'un thème référence le fichier. `portal_logo_path` → `site_logo` | ✅ Sonde `app:s165:media-probe`, 18 assertions, **médiathèque rendue vide et thème non touché** |
| **S166** ✅ | **CLOSE le 2026-09-18** : 68 littéraux tombés, le contraste REFUSE, les trois préréglages, le logo sombre et l'icône dérivée. 🅿️ **Reste hors phase** : l'image de partage — voir ci-dessous | ✅ `tools/brand_literals.py` (68 → 0), `app:s166:contrast-probe` (27), `app:s166:presets-probe` (21) |
| **S167** ✅ | **Livré le 2026-09-18** : l'aperçu rend de VRAIES pages (accueil + catalogue, desktop et mobile, clair et sombre) ; la publication devient atomique et refuse un logo disparu | ✅ Sonde `app:s167:theme-probe` (14 assertions, thème remis en place) + trois mesures `app:render` |
| **S168** ✅ | **CLOSE le 2026-09-19** : l'icône d'onglet thémable, et 🔴 **« dépublier » dépublie enfin**. 🅿️ **Hors phase** : un menu dont l'opérateur ordonne les entrées — voir la mesure ci-dessous | ✅ `app:s168:favicon-probe` (13) et `app:s168:unpublished-probe` (11, table rendue intacte) |

### ✅ S168b — « dépublier » ne dépubliait pas

🔴 **Le défaut, sur TROIS pages en production.** Archiver une page du lab la
retirait du MENU — `findTopLevelWithChildrenLive()` filtre — et la laissait
**entièrement lisible à son URL**. Un signet, un lien dans un mail, un partage,
un moteur de recherche : le contenu restait servi à tout le monde. Le contrôleur
faisait quatre lignes et ne regardait pas `archivedAt`.

🔴 **« Dépubliée » n'était vrai que dans UNE requête sur quatre.** Le même fait
avait quatre lecteurs et un seul le connaissait :
- le menu filtrait ✅ ;
- `/lab` listait les sous-pages archivées sous leur parent vivant ✗ ;
- la page de détail les liait ✗ ;
- et la route de détail les rendait ✗.
C'est le motif « deux vérités pour un fait », et la réponse est **une** définition
que tout le monde traverse : `LabPage::getLiveChildren()`, **sur l'entité** — un
dépôt ne sert que les appelants qui pensent à lui, alors qu'un gabarit écrit
`page.liveChildren` sans rien savoir de la règle.

✅ **L'accueil, avec trace, et sans boucle possible** — le critère de sortie, mot
pour mot. Mesuré : `302 → /`, l'accueil répond **200**, et un message explique le
renvoi. La cible est écrite en dur : viser le référent est exactement la façon
dont on fabrique une boucle.
⚠️ **Ni 404 ni page blanche**, délibérément : un 404 sur un lien qui marchait hier
ressemble à une panne du site.

🔴 **Sauf pour qui peut la rééditer.** Sans ça, archiver devient irréversible en
pratique — il faudrait restaurer à l'aveugle pour relire. La page s'ouvre pour
l'opérateur avec un bandeau qui dit qu'elle est dépubliée. Mesuré par
`app:render` : **200 + « This page is UNPUBLISHED »**.
⚠️ Le droit est demandé à `canReach()` sur l'écran d'édition, pas à un rôle écrit
en dur : il n'y a pas de hiérarchie de rôles ici, et une seconde définition de
« qui administre les pages » divergerait.

⚠️ **Un quatrième défaut trouvé en passant** : dans la liste d'administration, la
ligne de tête portait « Archivée » et **celle des sous-pages ne portait rien**.
Sur un écran fait POUR restaurer, c'est la seule information qui manque.

🅿️ **Ce que S168 ne livre PAS, et la mesure qui le justifie** : « l'ordre et la
visibilité des entrées de menu, les entrées système protégées ». Mesuré — les
pages du lab ont DÉJÀ ordre (`position`) et visibilité (archivage) ; le reste du
menu est une liste littérale dans `NavBuilder::header()`, gâtée par feature et par
`canReach()`. Rendre ces entrées-là ordonnables par l'opérateur, c'est construire
un éditeur de menu — une fonctionnalité, pas une finition. Et « destinations
limitées aux routes autorisées » **existe déjà** : `safeDestinations()` dérive du
pied de page, lui-même gâté.

### ✅ S168a — l'icône que personne ne pouvait changer

🔴 **`asset('images/favicon.png')` était écrit à la main dans HUIT gabarits**,
dont les quatre kiosques. Un labo qui posait son logo dans `/admin/themes`
gardait donc l'icône de FabOS dans l'onglet **et sur le mur de son atelier** — la
moitié la plus visible d'une identité, et la seule que personne ne pensait à
changer parce qu'elle n'était proposée nulle part.

✅ **Un seul gabarit l'écrit désormais**, comme pour le logo : c'est ce qui rend le
réglage tenable. Un second `<link rel="icon">` ailleurs serait une icône qui ne
change pas quand on change l'icône.
🅿️ **Une exception, attendue explicitement par la sonde** : `event-ticket`, qui ne
lit ni la feuille du site ni la médiathèque, parce qu'un billet s'ouvre sur un
téléphone avec un mauvais réseau et s'imprime sur ce qui traîne.

⚠️ **La sonde compte les émissions dans les SOURCES, pas dans un rendu** — rendre
les huit pages demanderait huit décors, dont un kiosque qui n'existe que pour un
lieu donné. Ce qui est vérifiable partout et sans décor : qu'il ne reste qu'un
seul endroit qui écrive cette balise.

✅ **L'icône est protégée comme le logo** : la médiathèque refuse de supprimer une
image que le thème publié **ou** le brouillon référence — les deux valeurs
comptent maintenant pour quatre.

🅿️ **Aucune variante n'est GÉNÉRÉE.** Une image de 2400 px servie comme icône
marche — les navigateurs la réduisent — mais elle coûte un téléchargement inutile
sur chaque page. Une vraie variante 32 px est le travail des « variantes de logo »
que S166 garde en réserve, et le dire vaut mieux que de laisser croire que c'est
fait.

### ✅ S165 — un réglage qu'on ne pouvait pas régler depuis l'écran qui le proposait

🔴 **`site_logo_path` était une CHAÎNE** : le nom d'un fichier qui devait DÉJÀ se
trouver dans `public/images/`. Rien ne téléversait, donc poser un logo demandait
un accès SSH — ce qui n'est pas un thème, c'est un déploiement.

🔴 **Le nom du fichier téléversé n'atteint JAMAIS le disque.** Le fichier
s'appelle `<mediaId>.<ext>`, tiré au sort ; le nom d'origine ne sert qu'à
l'affichage. Deux personnes qui envoient `logo.png` ne s'écrasent pas.
✅ **Et Symfony réduit déjà ce nom d'affichage à son `basename` avant qu'on le
voie** — « logo maison ../../.env.png » revient « .env.png ». La sonde le MESURE
plutôt que de le supposer : le jour où cette garde amont change, ce nom est écrit
tel quel dans une page d'administration.

🔴 **Le SVG est REFUSÉ, et ce n'est pas un oubli** — l'ancienne expression
régulière l'acceptait. Un SVG est un document XML qui peut porter `<script>`,
servi depuis NOTRE origine : du code exécuté dans la session de chaque visiteur.
⚠️ **Le type est décidé par le CONTENU**, pas par l'extension : la sonde envoie le
même SVG renommé `.png`, et il est refusé aussi.
⚠️ La contrepartie est réelle et l'écran l'annonce : un logo vectoriel devient un
PNG. Le format redeviendra acceptable le jour où les fichiers seront servis
depuis un domaine séparé.

🔴 **CORRECTION de la mesure de sortie ci-contre** : « l'orientation EXIF est lue
AVANT les dimensions, et `exif_read_data()` ne lit pas le PNG » décrivait une
capacité qui **existe déjà**, dans `ImageNormalizer`, avec son propre historique
de pannes. S165 la RÉUTILISE au lieu d'en écrire une seconde — deux
implémentations divergent, et le défaut est une photo couchée que personne ne
remarque pendant un mois.
✅ Ce qui en découle et qui se mesure : les dimensions sont enregistrées **après**
normalisation, donc ce sont celles qu'on verra ; et un PNG opaque est rangé en
`.jpg`, ce qui prouve que la normalisation s'est appliquée au fichier RANGÉ et
pas à une copie.

🅿️ **Aucune branche de compatibilité, et c'est mesuré** : le 2026-09-08,
`SITE_SETTING` ne contenait aucune ligne `site_logo_path` et le brouillon portait
`logoPath: ""`. Pas une seule valeur héritée à convertir — l'écrire aurait été
écrire du code sans cas d'usage, puis le maintenir.

⚠️ **`portal_logo_path()` est SUPPRIMÉ, pas aliasé.** Il renvoyait à un écran
« Portails » qui n'existe plus, et rendait un nom de fichier que l'appelant devait
préfixer — donc un chemin construit dans un gabarit. Garder un alias aurait laissé
les deux vocabulaires cohabiter sans que rien ne tranche.
✅ Vérifié à l'écran : la page d'accueil sert toujours
`Logo_ENSEA.png?v=20260706-3`, à l'identique.

⚠️ **La sonde ne touche pas au thème** : elle passe une liste de références
SYNTHÉTIQUE à `delete()`, ce que l'API permet justement parce que la médiathèque
ne connaît pas les thèmes. Le brouillon de l'opérateur n'est ni lu ni écrit.
✅ Résidu vérifié après coup : **0 ligne** en base, **0 fichier** dans
`public/uploads/identity/`.

### ✅ S167 — l'aperçu rend les vraies pages, et la publication ne se coupe plus en deux

🔴 **Ce qui était là : une bande DESSINÉE À LA MAIN.** Un `<strong>`, un `<span>`
et un `<i>` teintés de la couleur du brouillon. Elle prouvait qu'on savait
dessiner une bande. Elle ne disait rien de ce qu'on veut savoir — si le badigeon
`!important` de `style.css` repeint l'accent, si un jeton est lu là où on croit,
si le logo tient dans l'en-tête. C'est la leçon de
[[feedback-fabos-verify-pixels]], appliquée à l'écran qui en avait le plus besoin.

✅ **Quatre cadres : l'accueil et le catalogue, deux largeurs, deux thèmes.**
L'accueil porte l'en-tête, le logo et les blocs ; la liste porte les cartes, les
pastilles et les filtres — l'autre moitié du thème. Deux fois l'accueil n'aurait
montré qu'une moitié.

🔴 **Le thème sombre de l'aperçu est IMPOSÉ par l'URL, et VERROUILLÉ.** Le thème
du site est appliqué par `main.js` depuis `localStorage` : quatre cadres dans une
même page partagent ce stockage, donc afficheraient tous le même thème — et la
moitié sombre de l'aperçu aurait été un mensonge. `data-theme-locked` dit au
script de ne pas réécrire ce que le serveur vient de poser.

🔴 **`?theme=draft` est réservé aux administrateurs, et vérifié par la page
APERÇUE.** Ce sont l'accueil et le catalogue qui liraient le brouillon, et aucun
des deux n'appartient à l'administration : mettre la garde dans le contrôleur de
la grille n'aurait protégé que la grille.
⚠️ Rien n'est mis en session : le mode ne vit que dans l'URL de la requête en
cours. Un drapeau en session survivrait à la fermeture de l'aperçu et montrerait
un thème non publié pendant des heures, sans rien qui l'explique.

✅ **Les DEUX moitiés de la garde sont mesurées, par deux outils différents** — la
sonde n'a pas de session, `app:render` en a une :
- sans droits : `?theme=draft` ne suffit pas, le mode forcé est nul ;
- avec droits : `/?theme=draft&theme_mode=dark` rend
  `<html lang="en" data-theme="dark" data-theme-locked="1">` ;
- et `/` ordinaire rend `<html lang="en">` — **aucune fuite dans les pages
  ordinaires**.

🔴 **La publication était QUATRE publications.** Quatre `set()` à la suite : une
panne entre le deuxième et le troisième laissait le site avec le nouveau nom et
l'ancienne couleur, à moitié rhabillé, sans moyen de savoir où ça s'était arrêté.
Une transaction rend l'ensemble atomique.
🔴 **Et le FICHIER compte autant que les réglages** : publier un logo supprimé
entre la saisie et la publication poserait une image cassée sur chaque page. On
refuse AVANT d'écrire — mesuré, avec les quatre réglages publiés vérifiés
INTACTS après le refus.
⚠️ La médiathèque interdit déjà de supprimer une image que le brouillon
référence ; cette garde couvre ce que l'autre ne peut pas voir — une suppression
en base à la main, une restauration, un fichier parti du disque.

🔴 **Une première version de `transactional()` RÉESSAYAIT hors transaction** quand
celle-ci échouait — ce qui aurait réécrit par-dessus une transaction partiellement
appliquée, c'est-à-dire exactement le demi-thème qu'on prétend empêcher. Elle ne
rattrape plus rien : l'échec remonte, et l'écran dit « rien n'a été publié », ce
qui est alors la vérité.

✅ **Et la limite notée en S166b tombe** : `--color-primary-text` est maintenant
émis depuis PHP avec `--color-primary`. Un repli statique ne pouvait pas suivre un
thème ; celui-ci le suit partout, aperçu compris.

### ✅ S166c — des préréglages, pas des curseurs ; et deux logos plutôt qu'un pari

🔴 **Trois axes, trois listes FERMÉES.** Un champ « rayon » laisse taper 40 px et
transforme chaque carte en gélule ; un champ « taille du texte » laisse taper
24 px et fait déborder chaque composant à hauteur fixe. Trois choix par axe — neuf
combinaisons — ça se REGARDE avant de livrer, et l'aperçu de S167 les montre.

🔴 **Le piège central, trouvé en LISANT `style.css`, pas en livrant.** Sous
576 px, `--spacing-lg`, `--xl`, `--2xl` et `--3xl` descendent d'un cran dans un
`@media`. Le `<style>` du thème est émis APRÈS la feuille, à spécificité égale :
un `:root` de thème aurait donc GAGNÉ partout, mobile compris, et supprimé cette
réduction **en silence**. La densité réémet le palier dans le même `@media`.
✅ **Et la sonde vérifie les DEUX moitiés** : que `style.css` déclare bien ce
palier — sinon la précaution serait du bruit et personne ne le saurait — et que
le thème le réémet.

✅ **Au préréglage livré, RIEN n'est émis.** Vérifié à l'écran : la page d'accueil
ne porte aucune balise `<style>` de plus qu'avant, et aucun jeton de barème. La
même garantie qu'à S160 pour les e-mails — « je n'ai rien changé » se vérifie au
lieu de se promettre.
⚠️ **`--font-size-md` n'est PAS réémis** : `style.css` le définit comme
`var(--font-size-base)`, donc il suit tout seul. Le figer en pixels le ferait
cesser de suivre au premier changement de barème.

🔴 **Un logo SOMBRE, parce qu'un seul fichier pour les deux thèmes était un
pari.** Un logo foncé sur un en-tête sombre est invisible — et personne ne s'en
aperçoit tant qu'il ne bascule pas.
⚠️ **Deux `<img>` et une bascule CSS, pas un `src` échangé en JavaScript** : le
thème est posé sur `<html>` par un script, donc échanger le `src` après coup
ferait clignoter l'ancien logo à chaque chargement.
⚠️ **`display`, jamais l'attribut `hidden`** — la couverture de `style.css` pose
des `display` explicites qui gagnent contre `hidden` ([[feedback-fabos-css-cascade]]).
✅ Sans logo sombre choisi, ni la classe ni la seconde image ne sont émises :
balisage inchangé.

✅ **L'icône d'onglet est une VRAIE petite variante** — le 🅿️ laissé ouvert en
S168a. Une image de 2400 px servie comme icône marche, mais coûte un
téléchargement inutile sur chaque page, et les kiosques tournent toute la journée.
🔴 **Elle ne pouvait PAS réutiliser le redimensionneur existant** : celui-là
aplatit sur du BLANC (ses conteneurs de sortie sont destructifs), ce qui collerait
un carré blanc dans un onglet sombre. Deux besoins opposés, deux méthodes — et la
sonde lit l'octet 25 du PNG pour vérifier que le canal alpha est bien là.
⚠️ Générée à l'entrée plutôt qu'au moment du choix : un cache différé demanderait
une invalidation, donc un second état à tenir d'accord.

🔴 **Une assertion de la sonde passait POUR LA MAUVAISE RAISON, et l'échec l'a
montrée.** Elle soumettait `$brouillon + ['radius' => 'gelule']` : l'union de
tableaux PHP garde la valeur de gauche quand la clé existe des deux côtés — le
brouillon a déjà `radius`, donc la valeur hostile n'entrait jamais, et
« le brouillon n'a pas bougé » était vrai trivialement. `array_merge`, et le refus
est maintenant mesuré pour de bon.

🅿️ **L'image de partage n'est PAS livrée, et la raison est mesurée** : le dépôt ne
contient **aucune** balise `og:` — zéro, sur 232 gabarits. Une image de partage
demanderait donc d'introduire tout Open Graph (titre, description, par page ou
pour le site), ce qui est une fonctionnalité à part entière — et le site est
derrière une liste blanche NPM, donc rien ne peut aller chercher cette image
aujourd'hui. À reprendre le jour où le site s'ouvre.

### ✅ S166b — le contraste est mesuré, et il REFUSE

🔴 **Refusé, pas signalé** — c'est la mesure de sortie, et la différence est
tout : un avertissement qu'on peut ignorer se fait ignorer, et le texte illisible
part en production avec l'aval apparent de l'écran qui l'a laissé passer. Le
refus vit dans `ThemeManager::saveDraft()`, le point de passage, pour qu'un
import ou une commande bute sur la même règle que le formulaire.
⚠️ **Et le message porte les NOMBRES** : « 2,78:1 alors qu'il en faut 4,5:1 » dit
de combien assombrir. « Contraste insuffisant » n'aide personne à choisir la
couleur suivante.

⚠️ **DEUX contrôles, et un piège qui aurait fait croire à trois.** Le contraste
WCAG est SYMÉTRIQUE : « blanc sur la couleur » et « la couleur sur blanc »
donnent exactement le même nombre. Les lister séparément aurait affiché deux
lignes toujours identiques — l'illusion d'une vérification de plus.

🔴 **Le second contrôle n'est PAS redondant, et un exemple le prouve** : du NOIR
pur passe le premier à **21:1** et échoue le second à **3,39:1** — sa variante
éclaircie devient un gris moyen, illisible sur le panneau sombre. Sans lui,
« noir » serait accepté comme accent et casserait le thème sombre de tout le
site.
⚠️ Mesuré sur `#342b41`, le panneau ÉLEVÉ : il est plus clair que `#2b2335`, donc
plus dur pour un accent clair. La marque y fait 5,13 contre 5,75.

✅ **Les nombres de la sonde sont calculés À PART, à la main.** Comparer la sortie
du code à elle-même ne prouverait rien ; ces valeurs viennent de la formule WCAG
appliquée séparément, et c'est ce qui permet de détecter une erreur de luminance.
✅ Corrobore le relevé du 2026-09-05 : la marque fait bien **7,65:1**.

🔴 **Une trouvaille en chemin : `--color-primary-text` avait DEUX valeurs.** Le
repli statique valait `#f3a8c8`, le `color-mix()` rend `#ce8daa` — deux accents
différents selon le moteur, **deux points de contraste d'écart** (7,17 contre
5,11), sous un commentaire qui affirmait leur équivalence. Le repli porte
désormais ce que `color-mix()` produit réellement.
🅿️ **Et la limite de la forme est dite** : un repli STATIQUE ne peut pas suivre un
thème. Sur un moteur sans `color-mix()`, un labo qui change sa couleur garde cet
accent-ci. La corriger demande d'émettre le jeton calculé depuis PHP — c'est le
travail de S167, avec la publication atomique.

⚠️ **La sonde capture le brouillon AVANT et le remet APRÈS, quoi qu'il arrive.**
Elle compte sur le refus pour ne rien écrire — mais si la garde venait à laisser
passer, l'appel écraserait le thème de l'opérateur. Une sonde ne doit pas
dépendre de ce qu'elle mesure pour être inoffensive.

### ✅ S166a — le compte de la feuille de route était faux DANS LES DEUX SENS

🔴 **« 66 couleurs en dur », mesuré le 2026-09-05. Le vrai chiffre est 68**, et
l'écart est instructif — c'est exactement pourquoi un outil valait mieux qu'un
comptage :
- **trop bas** : il ne cherchait que `#9E1B56`, pas `rgba(158, 27, 86, …)`, qui
  est la MÊME couleur écrite autrement — **quinze occurrences de plus**, dont les
  puces de `/machines/{id}` et de `badge-detail` ;
- **trop haut** : il comptait des littéraux CITÉS DANS DES COMMENTAIRES, qui
  documentent une correction passée, et ceux d'`event-ticket.html.twig`.
⚠️ Et il avait déjà dérivé : 70 hex le 2026-09-15 contre 66 dix jours plus tôt.
Sans garde, ils reviennent.

🔴 **Le CONTEXTE décide du jeton, et c'est là qu'est le vrai défaut.** Une couleur
de TEXTE devient `--color-primary-text`, qui s'éclaircit en thème sombre ; un
fond, une bordure, un contour ou un `fill` SVG devient `--color-primary`, qui ne
bouge pas. Les confondre rend du bordeaux sur du bordeaux — c'est le trou de mode
sombre que `admin-loans` documentait déjà pour un seul `style="color:#6b7280"`.

⚠️ **`#7a1542` n'était pas dans le compte et méritait d'y être** : c'est la fin du
dégradé du primaire. Mesuré plutôt que deviné — 158·0,78 = 123 (0x7B ≈ 0x7A),
27·0,78 = 21 (0x15), 86·0,78 = 67 (0x43 ≈ 0x42) — donc
`color-mix(in srgb, var(--color-primary) 78%, black)`, à un point près.

🔴 **TROIS exclusions, et chacune tient :**
- `templates/emails/` — un client de messagerie ne sait pas lire `var()`. Le
  littéral y est la BONNE réponse, pas une dette ;
- `event-ticket.html.twig` — délibérément autonome (« no site stylesheet »),
  parce qu'un billet s'ouvre sur un téléphone avec un mauvais réseau et s'imprime
  sur ce qui traîne. Sans feuille du site, il n'y a pas de jeton à lire ;
- `admin-design.html.twig` — c'est la page qui DOCUMENTE le système : les hex y
  sont le SUJET, dans des `<code>`.
🔴 **Cette dernière a été apprise en la cassant.** La première passe a réécrit une
phrase de cette page et l'a rendue absurde : « les trois icônes portaient
`stroke="var(--color-primary)"`, le hex littéral de l'accent ». Repérée à la
relecture du diff, annulée, et l'exclusion est maintenant dans l'outil avec cette
raison écrite.

🅿️ **Ce qui n'est PAS vérifié : les pixels.** `app:render` prouve que le rendu ne
contient plus un seul littéral et porte bien les jetons — il ne prouve pas qu'on
VOIE la bonne couleur. `color-mix()` en particulier est écarté en bloc par un
moteur qui ne le connaît pas. Le dépôt s'en sert déjà dans `style.css` et
`components.css`, donc le pari est le même qu'avant ; il reste à le regarder.

## 🔴 Les pièges que cette phase va rencontrer, nommés d'avance

- **Le badigeon `!important` de `style.css`** repeint tout `<span>` d'un
  `.admin-panel` en gris. Tout jeton de couleur neuf se vérifie **au rendu**, pas
  dans la feuille — trois écrans s'y sont déjà fait prendre, le dernier le
  2026-09-03.
- **`color-scheme`** vient d'être réglé (2026-09-03) : un thème qui repose sur les
  contrôles natifs du navigateur doit le poser, sinon l'OS gagne.
- ✅ **Le cache-buster est centralisé** depuis le 2026-09-03 : une publication de
  thème doit bumper `css_version`, et c'est désormais **une ligne**.
- ⚠️ **Publication atomique** veut dire réglages **et** fichiers : publier la
  couleur avant que le logo ne soit en place laisse un site à moitié rhabillé.

---

## 🔴 Trois constats de SÉCURITÉ sur les boîtiers, VÉRIFIÉS le 2026-09-04

La revue apportée avec les planches Équipement (`docs/references/equipement/REVIEW-SOL.md`)
énonce trois P0. **Je les ai vérifiés dans le code plutôt que de les recopier —
les trois sont exacts.**

**1. ✅ CORRIGÉ le 2026-09-05 (S171) — l'API des boîtiers échouait en position OUVERTE.**
Elle rend maintenant **503 `device_api_not_configured`** quand le jeton manque —
503 et non 401, parce que le problème n'est pas l'appelant mais l'installation :
un boîtier qui reçoit 401 fait tourner son jeton pour rien.
🔴 **Et je dois corriger MON compte rendu du 2026-09-04** : j'avais écrit « un
POST sans en-tête rend 404 “machine inconnue”, donc la requête a franchi la
garde ». C'était faux — l'URL que j'avais testée, `/access`, **n'existe pas** ;
la vraie route est `/authorization`, et ce 404 était un 404 de ROUTAGE en HTML.
Le trou était bien réel, mais par lecture du code (`return null` quand le jeton
manque), pas par cette mesure-là. ⚠️ Une mesure qui tombe sur la mauvaise URL
ressemble à s'y méprendre à une mesure.

**1 bis. Le constat d'origine :**
`RfidMachineController::rejectUnauthorizedDevice()` fait
`if ($expectedToken === '') { return null; }` : sans `FABOS_RFID_API_TOKEN`, la
garde laisse passer. Mesuré sur la boîte : la variable n'est ni dans `.env` ni
dans `.env.local`, et un POST sans en-tête `X-FABOS-DEVICE-TOKEN` rend **404
« machine inconnue », pas 401** — donc la requête a franchi la garde.
✅ **Et le correctif ne casserait rien aujourd'hui** : un seul lecteur existe, vu
pour la dernière fois le **2026-07-10**. Aucun boîtier n'appelle.
🅿️ Deux moitiés, et il faut les deux : poser le jeton **et** rendre la garde
fail-closed, sinon la prochaine installation retombe dans le même trou.
⚠️ **À trancher par l'opérateur** : fermer une API d'accès physique se fait en
connaissance de cause, pas au détour d'une copie de fichiers.

**2. 🅿️ TOUJOURS OUVERT — et c'est une DÉCISION, pas un oubli.** Ce que le mur du
labo affiche publiquement appartient à l'opérateur, pas à moi : changer ce que les
membres voient sur leur propre écran ne se fait pas au détour d'une correction de
sécurité. Fait en attendant, gratuitement : les **quatre** kiosques portent
désormais `noindex, nofollow, noarchive` — aucun n'en avait. ⚠️ Ce n'est PAS une
garde : un robot poli obéit, un aspirateur non. Les deux voies restent celles de
la revue — signalétique anonymisée, ou kiosque authentifié par le boîtier.
Constat d'origine : Aucun `IsGranted`,
répond 200 sans session, et rend 49 références d'avatar — noms et passages RFID.
⚠️ Le site entier est derrière une liste blanche NPM, donc ce n'est pas exposé à
Internet aujourd'hui : c'est une protection d'INFRASTRUCTURE, pas une garde de
l'application. À décider explicitement — signalétique anonymisée, ou kiosk
authentifié par le boîtier.

**3. ✅ CORRIGÉ le 2026-09-05 (S171) — le formulaire Lecteur apprenait à donner la base à un boîtier.**
Les cinq lignes `FABOS_DB_*` sont parties de l'exemple `.env`, remplacées par ce
dont un boîtier a réellement besoin — son jeton, celui de sa machine, l'URL de
l'API et son courtier MQTT — plus une phrase qui dit **pourquoi** il n'aura jamais
la base. Constat d'origine :
`admin-rfid-reader-form.html.twig` affiche un exemple `.env` contenant
`FABOS_DB_HOST`, `FABOS_DB_USER` et `FABOS_DB_PASSWORD`. Un boîtier mural ne doit
jamais recevoir d'accès SQL. Ce n'est « que » de la documentation — c'est-à-dire
une consigne de fabriquer le trou soi-même.

⚠️ Ces trois-là sont des CONSTATS, pas la phase : la phase Équipement se planifiera
avec les planches. Mais le n°1 et le n°3 n'ont pas à l'attendre.

---

## 🅿️ Une planche de références, avec une DATE DE PÉREMPTION

**Ajoutée le 2026-09-04 à la demande de l'opérateur**, en DEUX lots sur
`/admin/references`, derrière le drapeau de développement :
- **Formations** — onze maquettes LMS (catalogue, parcours, quiz, exercice
  pratique, validation par l'équipe, badge, constructeur) ;
- **Machines & boîtiers** — huit maquettes plus DEUX documents (README et revue
  UX/sécurité), rendus depuis `docs/references/equipement/` par le même service
  que la feuille de route, jamais recopiés dans un gabarit.

- **Espaces & accès d'entrée** — huit maquettes plus DEUX documents. ⚠️ Cette
  revue-ci introduit `AccessPoint`, **distinct de `Machine`** : c'est la réponse
  au todo « contrôle d'accès aux lieux » du 2026-09-03, qui rejoint donc la phase
  Espaces.

⚠️ **Un seul écran pour les trois**, et une seule entrée de menu : trois planches
temporaires feraient trois choses à retirer. Chaque section part avec SA phase.

🔴 **Elle enfreint à moitié une règle de la maison, et il faut le dire.** Le menu
Développement s'est fait retirer trois maquettes en S159, parce qu'une
proposition non implémentée n'a rien à faire à côté d'écrans réels. Celle-ci ne
tient que pour deux raisons : elle **s'annonce** comme une planche dès sa
première ligne, et elle n'a **aucun formulaire** — on ne peut pas la confondre
avec le produit.

⚠️ **Donc elle part.** Le jour où la phase Formations est écrite, ces images
rejoignent son dossier de phase et l'écran disparaît — page, route, entrée de
menu, images. Si cette ligne est encore là dans trois mois, c'est que la règle a
été perdue de vue.

⚠️ Non traduite, comme `/admin/pages-manquantes` : un écran d'outillage caché
derrière le drapeau n'entre pas dans les cinq catalogues. Seule l'entrée de menu
l'est.

## Ce que l'opérateur vérifie — Phase M

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S165** ✅ | `/admin/themes`, section « Médiathèque d'identité » | Un champ de fichier et un bouton Téléverser. ⚠️ Vide au départ : le site affiche le logo livré, et l'écran le dit |
| **S165** ✅ | y déposer un PNG | La vignette apparaît, sur un damier — un logo transparent sur fond blanc a l'air d'un logo blanc |
| **S165** ✅ | essayer d'y déposer un SVG | 🔴 **Refusé, avec la raison** : un SVG peut contenir du code |
| **S165** ✅ | le champ « Logo du site » | Une **liste** de ce qui est dans la médiathèque, plus « — logo livré — ». Plus de nom de fichier à taper |
| **S165** ✅ | choisir le logo, enregistrer le brouillon, puis revenir à la médiathèque | L'image porte « Utilisée par le thème » et **n'a plus de bouton Supprimer** — le brouillon compte autant que le publié |
| **S165** ✅ | publier, puis regarder l'en-tête du site | Le logo a changé partout. ⚠️ Un seul gabarit rend le logo, c'est ce qui rend le réglage tenable |
| **S165** ✅ | la sonde | `php bin/console app:s165:media-probe` — 18 assertions, médiathèque rendue vide, thème non touché |
| **S166** ⏳ | `/machines/{id}`, `/login`, `/register`, `/`, en thème **SOMBRE** | 🅿️ **La ligne que je n'ai pas pu mesurer.** Les puces de matière, les icônes et les libellés d'accent doivent être LISIBLES — plus de bordeaux sur fond sombre. Le rendu ne porte plus aucun littéral, mais seul un œil voit une couleur |
| **S166** ⏳ | n'importe quelle page publique | Les icônes et les pastilles suivent la couleur principale du thème. Changer `primaryColor` dans `/admin/themes` doit les faire bouger TOUTES |
| **S166** ✅ | `/admin/themes`, sous les champs | Deux lignes de contraste **avec leurs nombres** : « Texte blanc sur la couleur 7,65:1 », « Éclaircie, en thème sombre 5,13:1 » |
| **S166** ✅ | y taper `#4caf50` (un vert clair) et enregistrer | 🔴 **REFUSÉ**, avec le nombre mesuré et le seuil — pas un avertissement qu'on peut ignorer |
| **S166** ✅ | y taper `#000000` | 🔴 **Refusé aussi**, et c'est le point : il passe le premier contrôle à 21:1 et échoue le second à 3,39:1 |
| **S167** ✅ | `/admin/themes`, bouton « Prévisualiser » | **Quatre cadres avec de VRAIES pages** : accueil et catalogue, desktop et mobile, clair et sombre. Plus aucune vignette dessinée |
| **S167** ✅ | le cadre « sombre » | Il est **vraiment** sombre, même si ta préférence est claire — le thème est imposé par l'URL et verrouillé |
| **S167** ✅ | changer la couleur du brouillon **sans publier**, puis prévisualiser | Les quatre cadres bougent. ⚠️ Le site public, lui, ne bouge pas : le brouillon n'est pas publié |
| **S167** ✅ | ouvrir `/?theme=draft` **déconnecté** | Le site normal. Le brouillon ne fuit pas |
| **S167** ✅ | la sonde | `php bin/console app:s167:theme-probe` — 14 assertions, thème remis à son état de départ |
| **S168** ✅ | `/admin/themes`, champ « Icône d'onglet » | Une **liste** de la médiathèque, comme le logo. Vide = celle du produit |
| **S168** ✅ | choisir une icône, publier, puis recharger n'importe quelle page | L'onglet du navigateur change. ⚠️ Un navigateur met une icône en cache plus longtemps que tout le reste : forcer le rechargement |
| **S168** ✅ | `/kiosk/entries` sur le mur | **La même icône.** C'était le vrai trou : un kiosque en plein écran montre son onglet à tout l'atelier |
| **S168** ✅ | la sonde | `php bin/console app:s168:favicon-probe` — 13 assertions |
| **S168** ✅ | archiver une page du lab, puis ouvrir son URL **déconnecté** | 🔴 Renvoyé à l'accueil, **avec un message**. Avant : la page s'affichait entièrement |
| **S168** ✅ | la même URL, **connecté en admin** | La page s'ouvre, avec un bandeau « DÉPUBLIÉE » — sinon on ne peut plus relire ce qu'on vient d'archiver |
| **S168** ✅ | archiver une SOUS-page, puis `/lab` et la page du parent | Elle disparaît des deux. Avant : listée et cliquable |
| **S168** ✅ | `/admin/lab-pages` | La sous-page archivée porte « Archivée », comme les pages de tête |
| **S168** ✅ | la sonde | `php bin/console app:s168:unpublished-probe` — 11 assertions, table rendue intacte |
| **S166** ✅ | `/admin/themes` → « Densité : Aérée », enregistrer, prévisualiser | Les quatre cadres respirent. ⚠️ Le cadre **mobile** doit garder des écarts plus serrés que le desktop — c'est le palier sous 576 px, qui aurait sauté en silence |
| **S166** ✅ | « Arrondis : Nets » puis « Doux » | Les cartes, boutons et champs suivent — 118 usages de `--border-radius` d'un coup |
| **S166** ✅ | tout remettre sur « Standard » | **Aucune balise `<style>` en plus** sur les pages : le préréglage livré n'émet rien |
| **S166** ✅ | téléverser un logo clair, le choisir en « Logo sombre », basculer en thème sombre | L'en-tête change de logo. ⚠️ Vide = le même dans les deux thèmes |
| **S166** ✅ | la sonde | `php bin/console app:s166:presets-probe` — 21 assertions, brouillon et médiathèque remis en place |

# Phase N — le cleanup (S169–S170)

**Demandée par l'opérateur le 2026-09-04** : *« le reste, fais une phase
cleanup »*. Elle ramasse ce qui n'appartient à aucune autre phase — et rien
d'autre. ⚠️ **Ce qui a une phase va dans SA phase** : la messagerie Formation est
absorbée par la phase Formations, le contrôle d'accès aux lieux par la phase
Espaces, les trois P0 des boîtiers par la phase Machines.

## Ce qu'elle contient

| | Ce qui reste, et pourquoi ça traîne |
|---|---|
| **J-5** | ✅ **CLOS PAR LA MESURE le 2026-09-05.** 38 gabarits, 643 règles, **544 sélecteurs locaux distincts — et 9 seulement apparaissent dans plus d'un gabarit**, dont 6 sont des artefacts de comptage (`0%`, `100%`, `">`). Il reste **une** duplication réelle : `.form-field textarea { min-height: 180px }` dans la paire `admin-lab-page-new` / `-edit`. ⚠️ **Laissée là, et c'est un choix** : la remonter dans `details.css` élargirait la hauteur de TOUS les textareas des pages qui la chargent. Un risque de régression visuelle pour une règle. Le reste est bien du CSS spécifique à sa page, ce que J-5 supposait sans l'avoir prouvé |
| **J-8** | ✅ **RÉGLÉ le 2026-09-05.** Les deux refus de la branche « profil public » redirigeaient, jetant l'adresse, la bio et les cases. Ils rendent désormais la page avec la saisie. 🔴 Prouvé par `app:s147:form-probe` : **statut 200, aucune redirection, la bio saisie revient à l'écran** — la sonde disait NON |
| **J-10**, moitié « taux d'aide » | ✅ **CLOS le 2026-09-05, et le critère a été REFORMULÉ.** Le taux n'est pas l'objectif : le compléter à l'aveugle produit du bruit. Le vrai critère de S149 est « des écrans à ZÉRO aide pour 8 champs ou plus ». Mesuré : il en restait **quatre**, il en reste **trois**, et chacun est déjà dans une phase — `LoanAdminType` (10 champs) → Phase T, `PlaceAdminType` (9) → Phase P, `MaintenanceTaskAdminType` (8) → ✅ **fait le 2026-09-19**, après la clôture de O : la phase s'est fermée sans lui. Les y traiter coûte zéro travail supplémentaire ; les traiter ici serait refaire demain un formulaire qu'on retouche aujourd'hui. ⚠️ Et `PackageSpecType`, que la mesure accusait à 0/14, explique dans son GABARIT (5 aides) : **ce n'était pas un défaut**. Le taux global est passé de 20 % à 32 %. Historique du barème dans `S149-REVUE.md`. 🔴 **CORRECTION du 2026-09-06 : « `admin-formation-content` et ses 35 champs » n'existe plus.** Mesuré : **1 champ visible à l'arrivée**, la recherche du site. La cible était atteinte depuis S149 ; ce document a continué à citer le chiffre d'avant. Un plan qui traîne une mesure périmée envoie travailler là où il n'y a plus rien à faire. Deux phases revendiquaient J-10 dans la première version de ce plan ; c'est corrigé. Barème dans `S149-REVUE.md` |
| **Suppression en masse** | 12 événements créés d'un envoi, retirés un par un. 🔴 **Une décision d'abord** : sélection multiple (aucune notion de série, réutilisable) ou identifiant de série (moins de clics, mais « que devient une séance déplacée ? »). ⚠️ Supprimer et annuler ne sont pas la même action quand des gens sont inscrits |
| **Catégorie → entrée de menu** | Une entrée de menu EST un filtre enregistré ; `/events?category=<slug>` existe déjà. Donc un réglage de navigation, pas une page |
| **Tableau de bord** | Quatre propositions comparables posées dans `/admin/design#tableau-de-bord` le 2026-08-27. 🔴 **Bloqué sur un choix, pas sur du travail** |
| **Fuseau côté entité** | `getRoles()` ne peut pas atteindre `LabClock` : un rôle survit jusqu'à l'offset du labo de trop. Borné, permissif, **sans exposition réelle aujourd'hui** — aucune appartenance datée ne porte un groupe à rôle. Les deux voies sont écrites plus bas |
| **`ROLE`** | La table n'accorde plus rien mais reste : le `down()` de `Version20260902100000` s'en sert pour reconstruire. À retirer quand ce retour arrière n'aura plus de sens |

## L'ordre, et pourquoi

| Session | Livre |
|---|---|
| **S169** | Les trois restes de J — **et J est close**, ce qui débloque la Phase H. J-8 puis J-10 (des corrections), J-5 en dernier parce qu'il commence par une mesure qui peut conclure « rien à faire » |
| **S170** | Les décisions de l'opérateur, une fois prises : suppression en masse, catégorie → menu, tableau de bord. ⚠️ **Ne pas commencer S170 avant que les trois soient tranchées** — construire l'une des deux voies de la suppression en masse avant le choix, c'est jeter la moitié du travail |

## Critères de sortie

- 🔴 **Phase J close pour de bon**, donc la barrière du commerce tombe.
- ⚠️ Chaque ligne fermée l'est **par une mesure**, pas par une relecture : deux
  défauts J étaient déjà caducs depuis S159 et personne ne l'avait vu.
- 🅿️ Le fuseau et `ROLE` peuvent rester ouverts en sortant : ils sont datés,
  bornés, et sans exposition. Les fermer demande une décision d'architecture.

---

# Phase O — Machines & boîtiers (S171–S174) — ✅ **CLOSE le 2026-09-05**

✅ **Livrée.** Les trois P0 de sécurité, les états réels d'un boîtier, la fiche
machine séparée en deux publics, et la matière ramenée à une seule vérité.
📖 **Le récit, les décisions et les mesures → `docs/history/phase-O-machines-boitiers-S171-S174.md`.**

## 🔴 La règle des trois phases issues des planches — lire AVANT de les ouvrir

⚠️ **Cette règle reste ICI, et pas dans l'historique** : la Phase P et la Phase Q
la citent en pointant « en tête de la Phase O ».

**On AMÉLIORE, on ne refait pas.** L'opérateur, 2026-09-04 : *« les screenshots
ont de bonnes idées, dérives-en des upgrades, pas des refontes entières »*.

🔴 **Et une précision qui restreint encore, donnée le 2026-09-05 :
« les screenshots sont pour de la PRÉSENTATION DE CONTENU, on ne change pas le
thème ou autre ».** Donc une planche ne dit RIEN sur les couleurs, la typographie,
le chrome ni l'identité — seulement sur **ce qu'on montre, dans quel ordre, et
regroupé comment**. Une couleur ou un composant vus sur une planche ne sont pas
une consigne : le thème et le système de design du produit gagnent toujours.

⚠️ **Et ce n'est pas une liste de tâches** : *« c'est de la clarification,
n'implémente pas tout »*. On en tire les idées qui servent une session, pas les
huit écrans d'un coup.

Concrètement : on garde les shells, les composants, le vocabulaire de colonnes,
les droits et le thème. Une planche apporte une IDÉE de présentation — « la
disponibilité lisible dès la carte », « une zone Exploitation séparée du contenu
public ». ⚠️ Toute planche qui semblerait exiger un shell neuf, une couleur neuve
ou un composant neuf est le signe qu'on l'a mal lue.

## Ce que l'opérateur vérifie — Phase O

🔴 **L'opérateur est le relecteur** (sa demande, 2026-09-05). Donc chaque session
finit par une chose CONSTATABLE À L'ÉCRAN, pas par un rapport. Une ligne qui dit
« vérifié en interne » n'est pas une ligne de cette liste.
⏳ **Cette liste n'a pas encore été parcourue** — elle reste ici tant qu'elle est
du travail qui attend.

⚠️ **Un test qui échoue ici n'est pas un détail de finition** : c'est la preuve que
la session a livré autre chose que ce qu'elle annonce.

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S171** | `/admin/rfid-readers/{id}/edit` | Le mode d'emploi du boîtier ne contient **aucun** `FABOS_DB_*` — ⚠️ ni dans le bloc affiché, **ni dans ce que copie le bouton Copier** (c'était deux endroits, un seul avait été nettoyé) |
| **S171** | n'importe quel navigateur | `POST https://fabos.dstei.fr/api/rfid/machines/1/authorization` **sans** en-tête de jeton rend **503 `device_api_not_configured`**, pas une autorisation. ⚠️ La route est `/authorization` — j'ai d'abord testé `/access`, qui n'existe pas, et pris son 404 pour une preuve |
| **S172** | `/admin/rfid-readers` | La colonne Statut ne dit **jamais « Actif »** sur un boîtier muet depuis plus d'une heure. Le lecteur de la boîte (silencieux depuis le 2026-07-10) doit lire **« Hors ligne »**, pas « Actif » |
| **S172** | même page | ⚠️ Le seuil d'une heure est un **choix**, pas une mesure — aucun boîtier ne tourne, personne ne connaît leur cadence. S'il te paraît trop court ou trop long, c'est un réglage, dis-le |
| **S173** ✅ | `/machines/{id}` en membre | La carte **« Puis-je l'utiliser ? »** est la PREMIÈRE de l'onglet, avant la description. Elle s'appelait « Badges requis » et empruntait le libellé du KIOSQUE, écrit pour un mur |
| **S173** ✅ | `/machines/{id}` en admin | Une zone **Exploitation** en bas de l'onglet : compteurs, liens staff, informations techniques. ⚠️ En anonyme, **zéro balise** de cette zone — vérifié au rendu, pas seulement à la lecture |
| **S173** ✅ | `/machines/{id}` et `/formations/{id}` en **anglais** | Les libellés « Bookings: », « Enrolled: », « Completed: » sont bien là. 🔴 Ils **disparaissaient** : un `%count%` numérique fait lire le message comme une forme plurielle, et Symfony jette ce qui précède les deux-points quand c'est un seul mot. Le français y échappait par sa typographie — donc les quatre AUTRES langues étaient seules cassées. `tools/i18n/count_colon.py` l'interdit maintenant |
| **S174** ✅ | `/machines/{id}` d'une machine sans matériaux saisis | Elle n'annonce **rien** — plus « PLA, PETG, TPU, Support ». ⚠️ **Aucune machine d'ici n'a le champ vide** : le défaut était une mine pour une installation neuve. Prouvé en vidant puis en remettant la machine 10 au bit près |
| **S174** ✅ | `/machines/{id}` | **Une seule** section matériaux. Il y en avait deux, côte à côte, l'une sur le texte libre et l'autre sur `MACHINE_MATERIAL` |
| **S174** ✅ | `/materiaux`, puis un clic sur un matériau | On arrive sur **sa fiche**, avec les machines qui l'acceptent, cliquables. Avant, chaque carte renvoyait à la liste d'où l'on venait de cliquer |
| **S174** ✅ | `GET /api/machines/5` **sans être connecté** | `machineToken` vaut **null**. 🔴 Il valait `"prusa-mk3s-01"` — le segment qui adresse la machine sur l'API des boîtiers, publié à qui passait. ✅ Pas un contournement : S171 a rendu cette API `fail-closed`. Une divulgation inutile, pas une porte ouverte |

## ⏳ Ce que la passe de fond a laissé ouvert

⚠️ **Le kiosque garde favicon, CSS et styles locaux.** Soit il rejoint le shell et
le thème publié — ce que la **Phase M** demande aussi — soit on écrit pourquoi il
reste à part. **Pas de troisième option silencieuse.**

---

# Phase P — Espaces & accès d'entrée (S175–S178) — ✅ **CLOSE le 2026-09-06**

✅ **Livrée.** `AccessPoint` existe : un boîtier commande une **PORTE**, plus
seulement une machine. Mise en service et incidents actionnables, parcours membre,
accès temporaire lié à une réservation.
📖 **Le récit, les décisions et les mesures → `docs/history/phase-P-espaces-acces-S175-S178.md`.**

## Ce que l'opérateur vérifie — Phase P

🔴 **L'opérateur est le relecteur.** Une ligne par constat visible à l'écran ;
jamais « vérifié en interne ». ⏳ **Cette liste n'a pas encore été parcourue** —
elle reste ici tant qu'elle est du travail qui attend.

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S175** ✅ | `/admin/access-points` (menu **Espaces**) | L'écran existe et il est VIDE. ⚠️ C'est le bon résultat : la migration crée la table, elle n'invente aucune porte |
| **S175** ✅ | `/admin/access-points/new` | Le champ **Espace ouvert** peut rester vide — un portail d'entrée n'ouvre aucune salle en particulier. Vide est une réponse, pas un oubli |
| **S175** ✅ | `/admin/rfid-readers/1/edit` | Les DEUX cibles sont là, l'une sous l'autre, **avant** le bouton Enregistrer. Choisir une machine ET un point d'accès doit être REFUSÉ, avec l'erreur sur le champ |
| **S175** ✅ | `/admin/rfid-readers` | La colonne s'appelle **Commande** (plus « Machine »), et « LECTEUR ZÉRO » y affiche toujours exactement ce qu'il affichait : `Imprimante 3D test`, `Inactif` |
| **S175** ✅ | rien du tout | 🔴 **C'est la mesure de la session** : le rendu de la liste des boîtiers diffère d'exactement **deux lignes** avant/après — le compteur du menu Espaces (4→5) et l'en-tête de colonne. Rien d'autre n'a bougé |
| **S176** ✅ | `/admin/rfid-readers/1/edit` | Un bloc **Mise en service** en bas : 4 étapes faites, **1 bloquante en rouge** — « Allumer l'API des boîtiers ». 🔴 Cette étape était INVISIBLE : depuis S171 la garde refuse tout appel sans `FABOS_RFID_API_TOKEN`, et aucun écran ne le disait |
| **S176** ✅ | `/admin/rfid-readers` | Un bandeau rouge en haut le dit une fois pour tous les boîtiers. ⚠️ La colonne Statut ne le répète PAS : une cause commune ne doit pas se lire comme plusieurs pannes |
| **S176** ✅ | `/admin/access-rfid-logs?days=0&result=no` | **72 refus sur 72** portent une colonne « À faire » avec un VERBE, et le clic mène au bon endroit — 42 vers la fiche du membre, 19 vers LE lecteur fautif, 11 vers la liste quand la cible n'existe pas par construction. Zéro lien mort |
| **S176** ✅ | `/admin/design` | Les deux composants neufs y sont, avec leurs trois états et le défaut qui les a fait naître |
| **S177** ✅ | `/places/{id}` | Une section **« Comment on y entre »**. Aujourd'hui elle dit qu'aucun accès n'est déclaré — c'est vrai, et le dire vaut mieux qu'une page qui a l'air complète |
| **S178** ✅ | `/admin/access-points` avec une porte sans boîtier | Elle lit **« Aucun boîtier »** en ambre, pas « Actif » en vert. Une porte annoncée au membre et que rien n'ouvre est une affordance morte |
| **S177** 🅿️ | `/places` **un jour ouvré, aux heures d'ouverture** | 🔴 **LE SEUL POINT QUE JE N'AI PAS PU MESURER.** La pastille d'une salle occupée doit dire **« Libre à 14:00 »**, plus « Occupé ». À 01h30 le labo est fermé et la branche « fermé » gagne — correctement. La prouver demandait d'insérer une réservation en production : le classificateur a refusé, et il a raison. Vérifié qu'aucune écriture n'a eu lieu |
| **S178** ✅ | `php bin/console app:s178:door-probe` | Huit sections vertes, dont **« refusé immédiatement après l'annulation »**. Base rendue à l'identique. ⚠️ Rien n'est révoqué parce que rien n'est accordé : la question est reposée à chaque badge |

⚠️ **Ce que je ne peux PAS mesurer sur cette boîte, et qui reste donc à ta
main** : l'API des boîtiers rend `503 device_api_not_configured` faute de
`FABOS_RFID_API_TOKEN` (S171 fait son travail). Aucune autorisation de bout en
bout n'est vérifiable ici — seulement la base et l'écran.

---

# Phase Q — Formations (S179–S183)

**Planifiée le 2026-09-04**, d'après les onze planches LMS. **Absorbe l'ancienne
Phase I** (messagerie Formation) : elle attendait le modèle session / cohorte que
cette phase construit.
⚠️ **La règle de lecture des planches est en tête de la Phase O** : présentation
de contenu uniquement, jamais le thème, et pas une liste de tâches.

## Ce qui existe déjà, mesuré

`Formation`, `Section`, `Quiz`, `Progression`, `Badge`, `UtilisateurBadge`,
`MachineBadge` (le lien badge → machine, donc l'accès), `/formations/{id}/suivi`,
et l'écran d'édition de contenu `admin-formation-content` — **35 champs visibles**,
le plus lourd du produit, et l'un des trois restes de la Phase J.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S179** | **Le parcours de l'apprenant, sans nouveau modèle** : « votre prochaine étape » sur la fiche, progression lisible, et ce qu'on obtient à la fin (le badge, et la machine qu'il ouvre) | Un apprenant sait quoi faire ensuite **sans lire toute la page** |
| **S180** ⏳ | **L'étape PRATIQUE.** ✅ **Livré le 2026-09-06** : la FILE des validations (`/admin/validations-pratiques`, menu Formations). 🅿️ **Le bouton « demander une évaluation » est REFUSÉ, pas oublié** — avoir fini la théorie EST la demande, et la file se déduit ; un enregistrement de demande créerait une seconde vérité sur qui est prêt, et quiconque finit sans cliquer n'existerait pour personne. Ce que ce choix perd : signaler qu'on est dispo à un MOMENT donné — c'est du rendez-vous, pas de la qualification. 🔴 **Reste à faire, et ça demande une migration** : voir la ligne ci-dessous | Un badge ne s'obtient plus que par un chemin complet et tracé |

### 🔴 S180 — le défaut trouvé en construisant, qui demande une migration

**Ce qui décide qu'une formation exige une validation pratique est une liste de
MOTS-CLÉS FRANÇAIS codée en dur** : `laser`, `soudure`, `fraiseuse`, `cnc`,
`brodeuse` (`TrainingPolicyService::PHYSICAL_FORMATION_KEYWORDS`), cherchés dans
le titre et la catégorie.

⚠️ **C'est une garde de SÉCURITÉ décidée par une correspondance de chaîne.** Un
labo qui nomme son cours « Découpe au CO2 », « Plasma », « Tour à métaux » ou qui
travaille en anglais n'obtient **aucune** exigence pratique — silencieusement, et
sur un écran qui a l'air correct. C'est la même famille que le repli
`['PLA','PETG','TPU','Support']` retiré en S174 : une liste en dur qui tient lieu
de donnée.

**Le correctif** : un champ explicite sur `Formation` (« exige une validation
pratique »), la liste de mots-clés rétrogradée en valeur PAR DÉFAUT à la création,
et une migration additive qui coche le champ pour les formations qui matchent
aujourd'hui — pour que rien ne change au moment où elle passe.
⚠️ Migration = étape de l'opérateur, donc à séquencer explicitement.

✅ **CORRIGÉ le 2026-09-06 (S180b).** `FORMATION.requiresPractical` existe, la
migration a rempli chaque ligne depuis les mots-clés eux-mêmes (4 à `1`, 4 à `0`,
60 formations internes laissées à `NULL`), et la case est sur l'écran d'édition,
dans la section du badge — parce que les deux répondent à la même question.
`TrainingPolicyService` lit le champ d'abord ; `null` seul retombe sur les
mots-clés, `false` est un avis et il gagne.

🔴 **Prouvé à l'écran, pas déduit** : déclarer l'exigence sur « Formation découpe
vinyle » — qu'AUCUN mot-clé n'attrape — la fait apparaître dans la liste des
validations physiques de la fiche membre ; la remettre à `0` l'en retire. C'est
exactement ce qui était impossible avant. Base rendue à l'identique.

✅ **Mesuré le 2026-09-06** : sur cette installation, 4 formations déclenchent les
mots-clés et ont bien leur validation physique ; la file est vide, et **elle a
raison** — une seule personne dépasse 80 % de théorie (laser, 100 %) et sa
pratique est déjà validée. `EN_ATTENTE = 0` sur les quatre. Vérifié par une
commande temporaire, supprimée du Mac ET de la boîte.

🅿️ **Reste la CONTRACTION** : passer `requiresPractical` en `NOT NULL` et
supprimer la lecture des mots-clés dans `TrainingPolicyService`. À faire une fois
que le code qui écrit toujours la colonne aura tourné un moment — expand, soak,
contract. ⚠️ Tant que le repli existe, une formation créée par un code plus
ancien reste jugée sur son intitulé.
| **S181** ✅ | **Le constructeur.** ✅ La **checklist de mise en ligne** (2026-09-06). 🔴 **Et la cible « 35 → sous 12 » était PÉRIMÉE** — voir la mesure ci-dessous. L'aperçu apprenant existait déjà (bouton « Voir la page »). ✅ **Les étapes se DÉPLACENT** (S181b, 2026-09-23) | ✅ `app:s181:journey-order-probe`, la route comprise, base rendue à l'identique |

### 🔴 S181 — la cible de J-10 était déjà atteinte, et le plan ne le savait pas

**Mesuré le 2026-09-06 sur `/admin/formations/2/content` :** **1 champ visible à
l'arrivée** — et ce champ est la **recherche de l'en-tête du site**, qui
n'appartient pas à cet écran. Les neuf replis sont tous fermés au chargement.

⚠️ **La cible « 35 → sous 12 » décrivait l'écran d'AVANT S149**, qui a converti
les sept formulaires en cartes repliées. Le plan a continué à citer le chiffre
d'avant pendant quatre phases. **Un plan qui traîne une mesure périmée envoie
travailler là où il n'y a plus rien à faire** — c'est le même défaut que la
Phase S, qui listait la vérification d'e-mail parmi ses acquis alors qu'elle
n'existe pas.

✅ **Ce que S181 livre à la place, et qui manquait vraiment** : la **checklist de
mise en ligne**. Neuf replis, neuf formulaires, et rien qui réponde à « est-ce
que ça tient debout ? » — l'auteur devait ouvrir les neuf cartes pour découvrir
qu'il manquait un quiz, ou ne pas le découvrir et publier un parcours qui ne mène
à rien.

🔴 **Une seule étape BLOQUE, et ce n'est pas celle qu'on croit** : le niveau de
risque laissé indécis. Tant que `requiresPractical` vaut `NULL`, la question
« faut-il une évaluation sur la machine ? » est encore tranchée par un mot-clé
français dans le titre (S180b). Publier dans cet état, c'est publier une
formation dont personne n'a validé le niveau de risque.
✅ Prouvé à l'écran : remis à `NULL`, l'étape passe `is-blocking` ; remis à `1`,
elle repasse `is-done`. Base rendue à l'identique.

✅ **Troisième réemploi de `_commissioning`** — la mise en service d'un boîtier
(S176), le parcours d'un apprenant (S179), la mise en ligne d'une formation
(S181). Trois sujets sans rapport : c'est la définition d'un composant.
🔴 **Et un défaut du composant corrigé au passage** : il écrivait
`rfid_form.commissioning_title` EN DUR, donc le parcours d'un apprenant
s'annonçait « Mise en service » sur une page de formation. Un composant qui
impose le vocabulaire de son premier appelant n'est pas un composant, c'est une
copie qui s'ignore. `title` est maintenant obligatoire.
### ✅ S181b — l'ordre d'un parcours se déplace, il ne se tape plus

**Mesuré avant** : l'ordre d'une étape était un NUMÉRO saisi dans son formulaire.
Passer la 4ᵉ en tête = rouvrir quatre formulaires et renuméroter à la main ; deux
étapes au même numéro s'ordonnaient par leur id, un ordre que l'auteur ne voyait
nulle part. (La base est propre aujourd'hui : 1..n partout — c'est le geste qui
était cher, pas les données qui étaient fausses.)

✅ **Deux flèches ↑ ↓ par étape** dans « Sections du parcours ». Un formulaire par
flèche, sans JavaScript ; chaque bouton dit ce qu'il déplace ; au retour la page
rouvre la liste, s'ancre sur l'étape et **rend le focus à la flèche** (à l'autre
si celle-là vient de s'éteindre en bout de liste) : on enchaîne au clavier.
`JourneyOrder` renumérote tout le parcours 1..n puis échange deux voisines, dans
une transaction ; les blocs de contenu de la page, rangés dans la même table,
ne bougent jamais. Le numéro affiché est la POSITION.
✅ **Le champ « Ordre » quitte le formulaire d'une section** : il était la source
des doublons. Une nouvelle étape se place en fin de parcours.
⚠️ **Aucune progression n'est réécrite** : l'accès à une étape se calcule à
l'affichage (la précédente est-elle réussie ?). Réordonner un parcours déjà
entamé peut donc demander à un apprenant l'étape passée devant lui — c'est le
même choix que S182 : on l'exige, on ne révoque rien.

✅ **Sonde** : déplacements réels dans une transaction annulée (échange avec la
seule voisine, 1..n sans trou, bornes refusées, aller-retour neutre, blocs de
page intacts, section d'une autre formation refusée), puis **la route comme un
navigateur** — page, jeton, POST : jeton faux → 403 sans rien bouger, bon jeton →
303 ancré sur l'étape. Relecture de `SECTION` : identique à la ligne près.
Mobile : flèches de 29×34 à 40×40 px (ici et dans l'éditeur de quiz).

| **S182** ✅ | **Le quiz.** ✅ L'invariant (2026-09-06) ; ✅ **l'écran de résultat et la correction côté serveur** (S182c, 2026-09-23) ; ✅ **les types « remettre dans l'ordre » et « réponse courte »**, et un constructeur qui ne corrompt plus les bonnes réponses (S182d, 2026-09-23) | ✅ `app:s182:retake-probe` ; `app:s182:quiz-integrity-probe` (53 quiz, 2 960 combinaisons, 185 questions rouvertes) |

### 🔴 S182 — le défaut n'était pas là où la feuille de route le cherchait

**Une reprise de quiz ne réinitialisait rien** : `QuizProgressService` garde déjà
`max($ancien, $nouveau)`. La mesure était donc déjà tenue de ce côté-là.

🔴 **Ce qui cassait était un geste d'ADMIN.**
`GuidedTrainingService::synchronizeParentProgress()` recalculait les trois valeurs
sans plancher, et `$requiredQuizTotal` est le nombre de quiz obligatoires
**aujourd'hui**. **Ajouter un quiz à une formation « dé-diplômait » d'un coup tous
ceux qui l'avaient terminée** — et `dateEnd` était remis à `null`, c'est-à-dire
que la date à laquelle quelqu'un a fini son parcours était effacée.

🔴 **Et ça produisait deux vérités pour un fait.** Un badge ne se retire JAMAIS :
`ProgressionBadgeSubscriber` accorde et n'a aucun chemin de révocation. On se
retrouvait donc avec quelqu'un qui POSSÈDE le badge d'une formation que sa
progression déclare non terminée.

✅ **La règle, alignée sur celle du badge** : `completed` et `dateEnd` sont des
PLANCHERS, le score suit le même `max()` qu'un quiz. Un labo qui ajoute un quiz
l'exige des NOUVEAUX apprenants ; il ne révoque pas rétroactivement.
🅿️ Retirer une validation reste possible — mais comme geste d'administration
explicite, pas comme effet de bord d'une recompilation.

✅ **Vérifié DANS LES DEUX SENS sur la boîte** : avec l'ancien code, les trois
assertions tombent (complétion perdue, date effacée, score reculé) ; avec le
correctif, les trois tiennent. Le fichier a été remis au hash près.
⚠️ **Et la sonde choisit exprès une formation au parcours INCOMPLET** — sur une
formation terminée, l'ancien code passait aussi, et la sonde n'aurait rien
mesuré.
### ✅ S182c — les bonnes réponses ne quittent plus le serveur

🔴 **Mesuré avant : la page d'un quiz envoyait `correct: true` au navigateur**,
pour chaque bonne réponse — lisible en deux clics dans « Afficher le code
source », par un visiteur anonyme (3 drapeaux sur `/formations/9/quiz/1`). Et la
note affichée à la fin était calculée par le navigateur, le serveur recalculant
la sienne de son côté : deux vérités.
✅ **Après : 0 drapeau.** `QuizScorer` est le seul à corriger ; la page n'a plus
que les énoncés et les choix (l'ordre attendu d'une remise en ordre part
mélangé). `POST /api/quizzes/{id}/check` corrige sans rien enregistrer pour un
visiteur. La sonde a rejoué **2 960 combinaisons de réponses** sur les 53 quiz :
ancien correcteur et nouveau, **zéro désaccord**.

✅ **L'écran de résultat dit QUOI relire, sans donner la solution** (planche
`lms-quiz-result-retry`) : la liste des questions à revoir, « Voir toutes mes
réponses » (les siennes, jamais les attendues), « Repasser le quiz » seulement
en cas d'échec. 🅿️ Limite assumée : les reprises étant illimitées, on peut
encore trouver par élimination. La fermer, c'est limiter les reprises — une
décision de labo, pas de code.

### ✅ S182d — deux types de questions, et un constructeur qui perdait des bonnes réponses

🔴 **Mesuré avant : SUPPRIMER un choix dans l'éditeur décalait les cases « bonne
réponse ».** Quiz 1, question 2, deux bonnes réponses ; on retire la 2ᵉ ligne
(fausse), on enregistre : **une seule bonne réponse sauvée, aucune erreur**. La
carte d'une question existante et celle d'une question ajoutée étaient deux
copies écrites à la main, et la première n'avait pas les attributs que la
renumérotation cherchait. ✅ **Une seule macro** (`_quiz_question_card`) rend
les deux ; même geste après : **deux bonnes réponses envoyées**, noms alignés.
Et la case avait pour tout libellé une infobulle : elle porte maintenant
« Juste », visible.

✅ **Un type par question** : choix (unique ou multiple selon le nombre de cases
cochées, comme avant), **remettre dans l'ordre** (on saisit les étapes DANS
l'ordre, rangs et flèches ↑ ↓ ; plus de case « Juste »), **réponse courte** (une
ligne par réponse acceptée ; casse, accents et ponctuation finale ne comptent
pas). `QuizDraft` lit, valide et type le brouillon : un ordre à une étape ou une
réponse courte sans réponse est refusé.
🔴 **Rouvrir puis réenregistrer un quiz ne change rien** : la sonde refait ce
trajet pour les **185 questions** de la base — aucune ne change de type ni de
bonne réponse.

✅ **Côté apprenant** : une liste à flèches, dont chaque bouton dit ce qu'il
déplace (« Monter « Charger le filament » ») ; le focus clavier suit l'étape
déplacée, y compris quand elle arrive en tête et que sa flèche ↑ s'éteint.
Réponse courte : un champ texte.

🔴 **Mesuré en passant, en thème sombre, et corrigé** (défauts antérieurs) :
le bandeau des trois écrans d'édition d'une formation restait **blanc, titre
illisible** ; les boutons ↑ ↓ « Retirer » et « Retour à l'éditeur » étaient à
**2,03:1** (→ 7,87:1) ; sur la page du quiz, la pastille du type était à
**~1,4:1** (→ 9,2:1) et l'astuce sous 3:1 (→ 10,4:1). En mobile, une étape
passait de 136 à 92 px : flèches et « Retirer » sur une seule ligne.

| **S183** ✅ | **La messagerie de cohorte.** ✅ L'annonce (2026-09-06) ; ✅ **le fil privé (2026-09-23)** — un fil par (formation, apprenant), lu par le groupe `trainers` | ✅ `app:s183:cohort-probe` et `app:s183:thread-probe` (22 assertions, **aucun courrier**, rien laissé derrière) |

### ✅ S183b — le fil privé, et un modèle tranché par une mesure

🔴 **Mesuré avant d'écrire : toutes les formations portent « Équipe FabLab »
comme formateur** — un libellé, pas une personne. `Formation::$formateur` est une
chaîne libre, et rien ne relie une formation à ceux qui l'encadrent. Le seul
modèle fidèle est une boîte d'ÉQUIPE, et l'équipe qui existe est le groupe
`trainers` (→ `ROLE_TRAINER`), que l'opérateur gère déjà.
🅿️ Le jour où une formation nomme ses formateurs, le schéma ne change pas : on
restreint QUI VOIT les fils, sans déplacer une ligne.

🔴 **Un fil par (formation, apprenant), un seul apprenant par fil.** L'invariant
« aucun message privé ne bascule implicitement vers la cohorte » devient une
propriété du SCHÉMA : il n'existe aucun fil à plusieurs apprenants, donc aucune
requête qui pourrait en élargir un. `UNIQUE(formationId, learnerId)` + `INSERT
IGNORE` : c'est la base qui tranche quand deux onglets ouvrent le fil.

🔴 **Administrer n'est pas un droit de lecture.** Mesuré : un autre apprenant NE
lit PAS le fil, un administrateur non formateur NON PLUS — il reçoit
« Access Denied … ROLE_TRAINER » sur la boîte. D'où une boîte HORS de `/admin`.

✅ **FabOS est la source, l'e-mail une COPIE** — la règle de l'ancienne Phase I.
Message écrit d'abord, copies ensuite dans un `try` ; une par destinataire, jamais
de liste ; catégorie `MESSAGE` désabonnable. Le mail dit de répondre DEPUIS
FabOS : une réponse par retour de courrier n'arriverait nulle part.

✅ **Une seule définition de la cohorte** : `isMember()` est la règle de
`recipients()`, restreinte à la personne. Et un fil privé est effacé à
l'anonymisation — pas de clé étrangère, parce qu'un compte n'est jamais
supprimé : une cascade ne se déclencherait pas.

🔴 **Le contrôle VISUEL a trouvé ce que le balisage ne montrait pas** : le
formulaire sortait nu sur la page publique, parce que les règles de champ ne
vivaient que dans `admin.css`. Ma première réponse — un `.public-form` à part —
était une seconde copie ; **l'opérateur a demandé de DÉPLACER**, et c'était juste.
Trente règles vont dans `components.css`. Mesuré avant/après : l'admin est
identique en clair ; en sombre les erreurs deviennent lisibles ; et 🔴 **la page
profil avait déjà le même défaut** — trois champs en style natif du navigateur,
corrigés au passage.

### 🔴 S183 — l'invariant n'était pas à écrire, il était à ne pas casser

**« Une annonce n'expose aucune adresse » est une propriété du `Mailer`, pas une
fonctionnalité de cet écran.** `queueToUser()` prend UN utilisateur et écrit UNE
adresse ; il n'existe aucun chemin qui en accepte plusieurs. Pas de `CC`, pas de
`BCC`, donc **pas de liste à oublier de masquer**.
⚠️ Le prix, assumé : une annonce à trente personnes est trente envois. Grouper
pour aller vite est exactement la façon dont ce genre de fuite arrive.

✅ **`NotificationCategory::NEWS` existait SANS émetteur** — son commentaire le
disait : « Nothing emits this yet; the switch exists first ». S183 est son
premier émetteur, donc la case de préférence que les membres voyaient déjà se met
enfin à servir. L'annonce est **non transactionnelle** : elle respecte l'opt-out.

✅ **La cohorte se DÉDUIT des progressions**, y compris celles portées par les
formations internes (sections, quiz) qui remontent à leur parent. Une table
d'inscription serait une seconde vérité sur « qui suit ce cours », et quiconque
commence sans y figurer ne recevrait rien.

🔴 **La sonde N'ENVOIE RIEN, et c'est délibéré.** Le mailer de cette installation
est configuré et non suspendu : déclencher une annonce écrirait à de vrais
membres. Elle vérifie à la place que l'API **ne peut pas** prendre plusieurs
destinataires — ce qui est plus fort qu'un envoi réussi : un envoi prouve qu'une
fois ça s'est bien passé, la signature prouve qu'aucun chemin n'existe pour que
ça se passe mal. ✅ Vérifié : `EMAIL_LOG` compte **0** ligne
`formation_announcement`.

## Ce que l'opérateur vérifie — Phase Q

🔴 **L'opérateur est le relecteur.** Une ligne par constat visible à l'écran.

| Session | Où | Ce qui doit être vrai |
|---|---|---|
| **S179** ✅ | `/formations/2` connecté | Le bouton principal dit **ce qu'on fait maintenant** (« Continuer le cours »), plus « Voir ma progression » — qui est un endroit, pas une action |
| **S179** ✅ | même page | Un **parcours** en étapes : contenu, quiz, validation pratique, badge. ⚠️ C'est le composant `_commissioning` de S176, celui de la mise en service d'un boîtier — aucun dessin neuf |
| **S179** ✅ | `/formations/1` (imprimante 3D) | **Trois** étapes, pas quatre : pas de validation pratique. C'est S180b qui le décide, et ça se voit |
| **S179** ✅ | bas de la fiche | **« Ce que le badge ouvre »** — les machines réellement déverrouillées. La relation décidait de l'accès à chaque scan et n'était jamais montrée à l'apprenant |
| **S179** ✅ | `/formations/2` **déconnecté** | **Aucune étape.** Cinq étapes toutes « non faites » annonceraient à un visiteur qu'il a échoué à des épreuves qu'il n'a pas passées |
| **S180** ✅ | `/admin/validations-pratiques` (menu Formations) | La file existe, et elle est **vide** — c'est le bon résultat : une seule personne dépasse 80 % de théorie et sa pratique est déjà validée |
| **S180b** ✅ | `/admin/formations/2/edit` | Une case **« Exige une validation pratique »**, cochée pour la découpe laser, décochée pour l'imprimante 3D. 🔴 Avant, ça se DEVINAIT à partir du titre : « Découpe au CO2 » ou tout intitulé anglais n'exigeait rien |
| **S181** ✅ | `/admin/formations/2/content` | Une carte **« Prête à être publiée ? »** en haut, cinq étapes. 🔴 La cible « 35 champs → sous 12 » était PÉRIMÉE : mesuré, **1 seul champ est visible à l'arrivée**, et c'est la recherche de l'en-tête du site |
| **S182** ✅ | `php bin/console app:s182:retake-probe` | Verte. 🔴 Le défaut n'était pas la reprise d'un quiz — c'était **ajouter un quiz obligatoire**, qui « dé-diplômait » tous ceux qui avaient fini et effaçait leur date de fin. Vérifié dans les deux sens |
| **S181** ✅ | `/admin/formations/2/content`, « Sections du parcours » | Deux flèches ↑ ↓ devant chaque étape ; ↑ éteinte sur la première, ↓ sur la dernière |
| **S181** ✅ | cliquer ↓ sur la 1ʳᵉ étape | La page revient sur la liste ouverte, l'étape est 2ᵉ, **le focus est sur sa flèche ↓** — Entrée la redescend. Puis la remonter : on revient à l'ordre de départ |
| **S181** ✅ | « Modifier » une étape | Plus de champ « Ordre » |
| **S181** ✅ | `php bin/console app:s181:journey-order-probe` | Verte, base rendue à l'identique |
| **S182** ✅ | `/formations/9/quiz/1`, « Afficher le code source » | Aucun `"correct"` dans la page. Avant : les bonnes réponses y étaient, lisibles par un visiteur |
| **S182** ✅ | finir un quiz en se trompant | Le résultat liste **les questions à relire**, sans donner les bonnes réponses ; « Repasser le quiz » n'apparaît qu'en cas d'échec |
| **S182** ✅ | `/admin/formations/1/quizzes/1/edit` | Chaque question a un **type**. En « Remettre dans l'ordre », les cases « Juste » disparaissent, des rangs et des flèches apparaissent |
| **S182** ✅ | même page, retirer une réponse fausse AU-DESSUS d'une bonne, enregistrer | Les bonnes réponses restent cochées. 🔴 Avant : l'une d'elles sautait, sans message |
| **S182** ✅ | la même page en thème sombre | Le bandeau du haut est sombre et son titre se lit. Avant : bandeau blanc, titre blanc |
| **S182** ✅ | un quiz avec une question « ordre » (à créer : aucune n'existe encore) | Des flèches ↑ ↓ ; au clavier, on garde sa place en déplaçant |
| **S182** ✅ | `php bin/console app:s182:quiz-integrity-probe` | Verte, rien écrit |
| **S183** ✅ | `/admin/formations/2/annonce` (bouton « Écrire à la cohorte ») | La page dit **combien** de personnes elle touche — et n'affiche **aucune adresse**. Deux champs, objet et message ; pas de destinataires à cocher |
| **S183** ✅ | `php bin/console app:s183:cohort-probe` | Verte, et **elle n'envoie aucun courrier** : le mailer de la boîte est actif, une sonde qui écrit à de vrais membres pour se prouver quelque chose ne se lance pas toute seule |
| **S183** ✅ | une formation que tu SUIS, page « Suivi » | Un 3ᵉ onglet **« Messages »**. ⚠️ Absent pour qui n'a pas commencé la formation |
| **S183** ✅ | y écrire un message | Il apparaît dans le fil ; les formateurs reçoivent une copie par e-mail, **chacun la sienne** |
| **S183** ✅ | menu *Apprendre*, connecté en **formateur** | « Messages des apprenants » : la boîte de l'équipe, les non-lus sont les tiens. ⚠️ Invisible pour un admin non formateur |
| **S183** ✅ | répondre depuis la boîte | La réponse part à CET apprenant seul. Son onglet affiche un compteur |
| **S183** ✅ | `/profil`, les champs « adresse publique » et « bio » | Ils ont le style du site. Avant : style natif du navigateur, 13 px, bordure grise |
| **S183** ✅ | `php bin/console app:s183:thread-probe` | 22 assertions, **aucun courrier**, comptes et fils rendus à leur compte de départ |

## La passe de fond de cette phase

- **Réemploi** : les badges, les quotas et les droits d'usage existent. Cette
  phase les BRANCHE, elle ne les double pas. ⚠️ La certification reste hors du
  modèle de forfaits : sécurité, pas commerce.
- **Conformité** : mêmes outils, début et fin — et l'écran de contenu est le pire
  du produit pour `form_placement.py`, donc c'est la mesure qui dira si S181 a
  réussi.

---

# Les sept lots de références, et où va chaque planche

**Le plan maître (`docs/references/MASTER.md`, 2026-09-05) apporte quatre lots de
plus** — Utilisateurs, transverses, coordination, surfaces finales — soit **52
maquettes et 13 documents** au total sur `/admin/references`.

⚠️ **La règle de lecture ne change pas** : présentation de CONTENU, jamais le
thème, et ce n'est pas une liste de tâches (voir en tête de la Phase O).

## 🔴 Deux phases neuves seulement — le reste se RANGE

Quatre lots ne veulent pas dire quatre phases. La plupart des planches complètent
un chantier déjà planifié ; leur donner une phase à elles produirait deux plans
pour un même écran.

| Planche | Va dans |
|---|---|
| `01-evenements` | **Phase L** — elle porte déjà l'annonce aux membres |
| `03-materiaux-equipement` | **Phase O, S174** — la matière ramenée à une seule vérité |
| `04-maintenance` | **Phase O** — la file d'intervention, avec le lien machine |
| `05-configuration`, `07-kiosque`, `01-accueil-configurable` | **Phase M** — le kiosque doit consommer le thème publié, c'est déjà son S168 |
| `01-calendrier`, `02-rendez-vous-personne` | **Phase P** — réservation et créneaux |
| `02-acces-exceptionnels` | **Phase P** — l'accès temporaire, motivé et audité |
| `06-preferences-email` | **Phase K** — les préférences vivent avec les gabarits |
| `04-mon-badge`, `04-recuperation-compte` | **Phase S** (neuve) |
| `02-prets`, `06-creations`, `05-recherche`, `03-rapports` | **Phase T** (neuve) |
| `03-groupes` | ✅ **Rien à faire : construit en S158/S159.** À COMPARER, pas à refaire — et si la planche montre mieux, c'est une amélioration de présentation, pas un modèle |

## ⚠️ L'ordre du plan maître, et pourquoi je garde le mien

Le maître propose : socle partagé → sécurité et droits → parcours principaux →
exploitation → éditorial. **C'est un bon ordre, et il recoupe le nôtre à un
décalage près** : son « socle partagé » est en grande partie la **Phase J**, qui
se termine, et son « sécurité et droits » est le début de la **Phase O** (les
trois P0) plus ce que S158/S159 ont déjà livré.
🔴 **Ce que je garde du nôtre** : il est ancré sur ce qui existe VRAIMENT dans ce
dépôt — les entités, les écrans, les défauts mesurés — là où le maître décrit un
produit cible. Un plan qui ignore l'état du code se paie à la première session.

---

# Phase S — comptes, adhésion et confiance (S189–S192)

D'après le lot `users` et deux planches de coordination.

## Ce qui existe déjà, mesuré

`Utilisateur`, l'inscription, `/profil`, les groupes et leurs droits (S158/S159),
l'annuaire `/admin/utilisateurs` avec ses filtres.

🔴 **CORRECTION DU 2026-09-06 — « la vérification d'e-mail » était listée ici
comme EXISTANTE. Elle n'existe pas.** `SiteController::register()` fait
`->setIsVerified(true)->setStatut('actif')` à la création
(`src/Controller/SiteController.php:2162`, vérifié) : n'importe quelle adresse,
même inventée, ouvre un compte actif immédiatement. Un plan qui compte une garde
absente parmi ses acquis est pire qu'un plan qui l'oublie — il ferme la question.

🔴 **Et l'inscription DIVULGUE l'existence d'un compte.**
`src/Controller/SiteController.php:2141` rend « Un compte existe déjà avec cette
adresse email », donc `/register` est un oracle d'appartenance : on teste une
adresse, on sait si elle est membre du labo. ⚠️ **Dix mètres plus loin, le même
produit applique l'invariant INVERSE** : `SecurityController::forgotPasswordSubmit()`
rend toujours `forgot.sent_if_exists`, que l'adresse existe ou non, et le
commentaire dit pourquoi. Deux vérités pour un fait, encore.

🅿️ **Et ça ne se corrige PAS en changeant la phrase.** Sans vérification
d'e-mail, l'inscription n'a que deux issues : refuser (donc divulguer) ou
accepter (donc laisser créer des comptes sur l'adresse d'autrui). La
non-divulgation à l'inscription est une CONSÉQUENCE de S189, pas un correctif
séparé — les deux se font ensemble ou aucune ne tient.

🔴 **Ce qui n'existe pas** : la vérification d'e-mail, le MFA, la gestion des
sessions, une récupération de compte non divulguante à l'inscription, et un
parcours d'adhésion.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S189** | **L'entrée** : inscription courte qui annonce ses prochaines étapes, activation par e-mail avec renvoi et correction d'adresse — **sans impasse**. 🔴 **Et c'est là que `/register` cesse d'être un oracle d'appartenance** : la réponse devient la même que l'adresse existe ou non, ce qui n'est possible QUE parce que l'activation par e-mail arrive dans la même session | Une adresse mal tapée se corrige sans recréer un compte. 🔴 Et une sonde : deux adresses, l'une connue l'autre non, **réponses identiques** — la même mesure que S191 |
| **S190** | **L'adhésion** : ne demander que ce qui est nécessaire, au moment où ça l'est. Et la **validation par l'équipe**, progressive et justifiable | Un compte en attente sait ce qui lui manque, et qui l'a validé |
| **S191** | **Sécurité du profil** : sessions visibles et révocables, MFA. ⚠️ Et une **récupération de compte NON DIVULGUANTE** — la réponse est la même que l'adresse existe ou non | 🔴 Prouvé par une sonde : deux adresses, l'une connue l'autre non, réponses identiques |
| **S192** | **Les droits EXPLIQUÉS** côté admin — par rôle, lieu, formation et durée — et « mon badge » sans identifiant sensible | Un admin répond à « pourquoi cette personne a-t-elle ce droit ? » **depuis l'écran** |

## La passe de fond

- **Réemploi** : `AudienceResolver` répond déjà « d'où vient ce droit » ; S192
  l'AFFICHE, elle ne le recalcule pas.
- ⚠️ **Aucun écran de ce lot ne doit exposer un UID de badge** — c'est le même
  invariant que les kiosques.

---

# Phase T — les surfaces restantes (S193–S195)

Ce qui n'appartient à aucune autre phase : prêts, créations, recherche, rapports.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S193** | **Prêts** : la circulation de l'objet, la fiche cliquable, le retour. ⚠️ `LoanableItem` et l'archivage existent — c'est de la présentation, pas un modèle | Rendre un objet se fait depuis la fiche, pas depuis une liste |
| **S194** | **Recherche globale** : résultats par type, navigation rapide. ⚠️ `/search` existe déjà | Un résultat ouvre toujours la fiche de son objet |
| **S195** | **Rapports qui conduisent à une action**, et **créations** : une communauté sobre | Un rapport propose l'action qu'il suggère, au lieu de la décrire |

## La passe de fond

- 🔴 **C'est la phase où l'on vérifie que TOUT objet ouvre sa fiche** — le critère
  de sortie que le plan maître pose pour chaque lot, et qui se mesure d'un seul
  balayage.

---

# Phase R — commerce facultatif (S184–S188) — LA DERNIÈRE

🔴 **BLOQUÉE PAR LA PHASE J** (opérateur, 2026-08-21), et **replanifiée en
dernier** le 2026-09-04, à sa demande. Renumérotée S184–S188 : les anciens
numéros S150–S154 chevauchaient des sessions déjà livrées, ce qui rendait le plan
illisible.

## ✅ Ce que l'opérateur a DÉJÀ tranché, et qui change tout

🔴 **On n'achète pas un forfait : on achète une APPARTENANCE DATÉE à un groupe**
(décision du 2026-09-01, voir `history/phase-S158-S159-groupes.md`). C'est ce qui
supprime le cas particulier du commerce au lieu de le contourner : un seul chemin
pour les humains et pour les machines, et l'expiration devient le mécanisme au
lieu d'un obstacle.
⚠️ **Conséquence directe** : le commerce n'écrit **pas** dans les forfaits. Il
écrit une ligne d'appartenance, avec des dates, dans `USER_GROUP_MEMBER` — la
table que S159g a rendue datable exprès. Le reste du produit ne change pas.
⚠️ Et **la durée appartient à l'OFFRE, pas au forfait** : sinon le même groupe ne
peut pas se vendre au mois ET à l'année.

## Ce qui existe déjà, mesuré

Presque rien, et c'est sain : `wallet` n'apparaît **nulle part** dans le dépôt,
aucune entité de facture, aucun fournisseur de paiement. La phase part d'une page
blanche — sauf l'appartenance datée, qui est déjà là et qui est le cœur.
🅿️ **Signal du marché voisin** : sur les 22 demandes les mieux votées de
Fabmanager, **huit** sont du commerce, et la notion de **portefeuille** y revient
quatre fois — crédit libre, prépayé, facturation manuelle, facture après
consommation (`FABMANAGER-ECARTS.md`). C'est un signal faible, mais il pointe tous
dans la même direction.

## Les invariants, inchangés

- 🔴 **Le retour navigateur ne confirme JAMAIS un paiement** — seul un webhook
  vérifié, ou sa réconciliation.
- Clé unique par événement fournisseur ; outbox persistante par ligne → effet
  **exactement une fois** malgré reprises et pannes.
- La livraison passe par le service métier normal, sans toucher voter, badge,
  quota ni réservation.
- **Ni carte ni credentials fournisseur en base FabOS.**
- ⚠️ Entièrement désactivable : une installation qui ne vend rien ne doit pas voir
  un seul écran de plus.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S184** | **Le catalogue d'offres**, sans aucune transaction. Une offre = un groupe + une durée + un prix | Le module éteint ne change **rien** à l'écran, prouvé par le balayage des routes |
| **S185** | **Commandes, paiement, webhooks, réconciliation, remboursements, audit** | 🔴 Un webhook rejoué deux fois ne crée qu'un effet ; un retour navigateur seul n'accorde rien |
| **S186** | **La livraison** : l'achat écrit une **appartenance datée**, et le remboursement retire exactement ce que cette commande-là a donné | 🔴 Prouvé par une sonde avant/après, comme le backfill de S158c |
| **S187** | **Le portefeuille**, si l'opérateur le veut : crédit, débit, ledger append-only. ⚠️ **À trancher** — c'est la demande la plus fréquente du marché voisin, et c'est aussi le plus gros morceau | Un solde ne se recalcule jamais : il se dérive du ledger |
| **S188** | **Reporting, rapprochement, audit UX** | Les totaux se réconcilient avec le fournisseur, exports scopés |

## La passe de fond de cette phase

- **Réemploi** : l'appartenance datée, les groupes, les forfaits et le journal des
  mails existent. Le commerce les BRANCHE.
- 🔴 **Et il ferme une dette de S159** : le **journal des appartenances**, laissé
  de côté parce qu'« il n'a de sens que le jour où une MACHINE écrit ». Ce jour
  est celui-ci. ⚠️ À la condition écrite alors : la ligne doit être **dérivée** du
  journal, sinon ce sont deux vérités de plus.

---

# Restes ouverts, hors phase

## Packages — ce qu'ils ne savent toujours pas dire

**Liste de choix, pas un plan. Rien n'est construit.**

- 🟡 **1. Assouplir un quota de palier — le plus vendable.** « 30 jours d'avance au
  lieu de 7 », « 8 h au lieu de 4 ». `BookingPolicy` porte déjà `maxHorizonDays`,
  `maxDurationMinutes`, `maxActiveReservations` — mais **par palier**, pas par
  package. Forme : un package portant des **surcharges** appliquées **vers le haut
  seulement**.
- 🟡 **2. Validité relative à l'attribution.** « Trois mois à partir de
  l'activation » n'existe pas : `validFrom`/`validUntil` sont deux dates absolues.
- 🟡 **3. Report des heures non consommées.** Une allocation hebdomadaire non
  utilisée est perdue le lundi.
- 🟡 **4. Allocation par catégorie de machines.** Les grants savent le dire, les
  allocations non, faute d'un comptage qui l'honore.
- ⚪ **5. Priorité / préemption** — pas demandé, **contraire à « aucun package ne
  retire un droit »**. Ne pas construire sans décision explicite.
- ⚪ **6/7. Prix, panier, paiement, matériaux inclus** → Phase H.
- ⚪ **8. Formations incluses** — la certification est **délibérément hors** du
  modèle de packages : sécurité, pas commerce. Vendre « la formation laser » est
  une commande Phase H qui **inscrit à une session**, elle ne fabrique pas un badge.

## ✅ S144e — « ce package touche N personnes » — FAIT le 2026-09-03

🔴 **Et le défaut était devenu total, pas partiel.** L'aperçu d'activation de
`/admin/settings` faisait `COUNT(DISTINCT a.userId)` ; or S159 a fait de
l'attribution à un GROUPE la seule surface humaine et sa conversion a déplacé les
trois dernières lignes personnelles, donc **toutes** les attributions vivantes ont
`userId = NULL`. Mesuré à l'écran : « 4 forfaits actifs couvrent **0** membres »,
et 0 pour les quatre capacités, pendant que les quatre chokepoints décidaient
réellement. ⚠️ L'écran portait l'avertissement « les attributions de GROUPE ne
sont pas comptées » — il ne mentait donc pas, il était **mort** : un aperçu qui
annonce toujours zéro n'invite pas à la prudence, il invite à conclure de travers.

✅ `AudienceResolver::memberIdsFor()` est écrit, et la liste comme l'aperçu
comptent des PERSONNES. Mesuré après : « 4 forfaits atteignent 3 personnes »,
2/2/3/3 selon la capacité, et la colonne de la liste s'appelle « Personnes ».
🔴 **L'inverse est PROUVÉ, pas affirmé** — sonde section 8, les deux sens sur tous
les comptes et toutes les clés : `k ∈ keysFor(p)` ⟺ `p ∈ memberIdsFor(k)`. Les
deux sens, parce que ne tester qu'un seul laisserait passer un membre OUBLIÉ, qui
est le défaut coûteux.
⚠️ Requêtes bornées : une clé de groupe n'est résolue qu'une fois par page.

## Thèmes — le détail des intentions (le PLAN est la Phase M)

⚠️ **Cette section n'est plus le plan** : la Phase M (S165–S168) le porte, avec
ses sessions, ce qu'on mesure et les pièges nommés. Ce qui suit reste utile pour
l'intention de chaque morceau — ne pas la traiter comme une liste de tâches
parallèle, sous peine d'avoir deux plans pour un chantier.

- **Médiathèque d'identité** au lieu du champ texte `logoPath` : logo
  clair/sombre/compact, favicon, image de partage. Validés, renommés serveur,
  référencés par ID stable, supprimables seulement après contrôle des références.
  **Aucun chemin `public/images/…` libre.**
- **Éditeur guidé** : identité, variantes de logo, palette avec contrastes,
  rayon/typo/densité en presets.
- **Workflow** brouillon → aperçu → publication → retour arrière. L'aperçu rend de
  **vraies** surfaces (accueil, catalogue, détail, admin, un kiosk),
  desktop/mobile, clair/sombre. Publication atomique réglages **et** assets.
- **Kiosks** consomment le thème publié. Aucun favicon, logo ou couleur statique
  ne survit dans un kiosk.
- **Navigation & accueil** : ordre et visibilité par drag-and-drop accessible,
  destinations limitées aux routes autorisées, entrées système protégées. Une page
  dépubliée rétablit l'accueil FabOS avec audit, sans page blanche ni boucle.
  ⚠️ `_logo.html.twig` retombe encore sur `Logo_ENSEA.png` et `site_logo_path`
  n'est éditable nulle part — dé-marquer avant la médiathèque laisserait le site
  sans logo et sans moyen d'en remettre un.

## 🟡 Supprimer en masse ce qu'on a créé en masse

**Opérateur, 2026-08-21** : *« if we can create X events, we have to have a way to
mass delete them »*. S146d crée jusqu'à 12 événements d'un envoi ;
`/admin/events` ne les retire qu'un par un.

🔴 **La tension à trancher d'abord** : S146d a fait des lignes **indépendantes**
(pas d'identifiant de série) exprès. Deux voies :
- **sélection multiple** sur `/admin/events` — ne suppose aucune série, réutilisable
  ailleurs, plus de travail d'interface ;
- **identifiant de série** nullable et informatif — moins de clics, mais réintroduit
  la notion de série et la question « que devient une séance déplacée ? ».

⚠️ **Une séance à laquelle des gens sont inscrits ne se supprime pas en silence.**
Supprimer et annuler (`callOff`) ne sont pas la même action.

## 🟡 Une catégorie peut devenir une entrée de menu

**Opérateur, 2026-08-20.** Une entrée de menu **EST un filtre enregistré** :
`/events?category=<slug>` existe déjà. Donc c'est un **réglage de navigation**, pas
une page.

À vérifier avant : ⚠️ **où vit le réglage** (le menu public n'est pas
`NavBuilder::admin()` ; Thèmes est censé être l'endroit des menus) · 🔴 **un menu ne
doit nommer ni une catégorie archivée ni une catégorie vide** · ⚠️ **le slug est la
clé, jamais le libellé** · ⚠️ le libellé est un **contenu**, donc non traduit à côté
d'entrées traduites · ⚠️ plafond d'entrées (le menu principal en a déjà cinq).

## Petits restes datés

- **Le tableau de bord a perdu son caractère** (opérateur, 2026-08-16) : *« the old
  homepage looked more "special", we'll find a way to make it pop again later »*.
  ⚠️ **Pas une régression à annuler** : le rendre distinctif **sans** réintroduire
  un bandeau pleine largeur ni une couleur en dur. Pistes : une bande d'accueil qui
  reste dans la carte mais respire ; les sept chiffres traités comme la figure de la
  page ; une seule surface accentuée réservée à cet écran. **À montrer en
  propositions comparables dans `/admin/design` avant de construire** — c'est le
  protocole qui a marché pour le format de liste (quatre tours).
  ✅ **Premier tour posé le 2026-08-27 : `/admin/design#tableau-de-bord`.** Quatre
  cadres, la même donnée dans tous — la référence d'aujourd'hui, puis A (la bande
  respire et porte un FAIT, « Ouvert jusqu'à 17:30 »), B (les trois premiers
  chiffres deviennent la figure de la page, sans aucune surface colorée), C (une
  seule surface teintée sur tout l'écran, réservée aux chiffres).
  ⚠️ **Aucune ne réintroduit ce qui avait été retiré** : rien ne sort de la carte, et
  pas une couleur littérale — `color-mix` sur `--color-primary` pour A,
  `--tone-primary-soft` pour C, `--color-primary-text` pour B. Vérifié en clair : le
  gros chiffre de B mesure **7,65:1** sur sa carte.
  🅿️ **Il manque l'avis de l'opérateur pour trancher** — c'est tout ce qui manque.
- ✅ **`/events` sans paramètre rend 0 carte** — corrigé le 2026-08-27, et dans le
  shell partagé plutôt que sur la page : `_catalogue.html.twig` offrait toujours
  « Réinitialiser » vers `path(route)`, or pour `/events` la page sans paramètre EST
  « à venir ». La sortie ramenait au même vide. Elle pointe maintenant sur une tuile
  non vide et l'appelle par son nom (« Passés »), et garde la réinitialisation quand
  c'est une RECHERCHE qui ne donne rien. Vérifié sur `?category=Atelier` (0 à venir,
  2 au total) et sur `?q=zzzzqqq`.
- **`/admin/homepage` porte six colonnes** (bloc + quatre audiences + ordre). C'est
  une matrice d'audiences, pas une liste. Le plafond de cinq ne lui répond peut-être
  pas. Non tranché.
- ✅ **Logs RFID** — clos le 2026-08-27, et deux des trois l'étaient déjà :
  `status` a été traité en S141f (`_rfid_result`), la couleur de LED en J-6. Il
  restait `reason`, qui imprimait `BADGE_MATCH`, `REQUIRED_BADGE_MISSING`,
  `TRAINING_OK`, `TRAINING_REQUIRED`, `RFID_NOT_FOUND` — des mots de firmware.
  ⚠️ Et deux valeurs héritées manquaient à la table de `_rfid_result` : `AUTHORIZED`
  (16 lignes) et `NO_TRAINING` (12) tombaient dans le repli qui humanise, donc
  s'affichaient « authorized » à côté de « Autorisé » sur la ligne voisine.
  ⚠️ `reason` est aussi MASQUÉ quand il redit le statut : le service écrit la même
  valeur dans les deux pour un tiers des lignes. Vérifié sur les 137 lignes de
  l'historique complet (`?days=0`) : **0 énumération brute**.
- ⚠️ **« Quiz et validations physiques n'ont aucune UI de création » — À MOITIÉ
  RÉFUTÉ, mesuré le 2026-08-27.** Les quiz en ont une :
  `/admin/formations/{id}/quizzes/new` et `/edit` rendent un formulaire complet
  (titre, type, note de passage, section rattachée, texte des questions). Les
  validations physiques ne sont pas un objet à part : c'est une `FORMATION` dont la
  `categorie` vaut exactement `Validation physique` — 7 lignes en base, plus 53 en
  `Quiz interne`.
  🔴 **Le vrai défaut n'était pas l'absence d'écran, c'était le SILENCE d'une faute
  de frappe** : le champ est du texte libre, et rien ne disait que ces deux chaînes
  retirent la formation du catalogue public. Une coquille créait une formation
  ORDINAIRE et publique là où on croyait poser un échafaudage. ✅ Corrigé : le champ
  porte maintenant une liste des catégories déjà employées (les deux spéciales
  incluses, volontairement) et une aide qui dit ce qu'elles font.
  🅿️ Ce qui resterait, si on le veut : un écran dédié pour créer une étape du
  parcours guidé. Ce n'est plus un manque bloquant, c'est du confort.
- ✅ **« Les deux écrans RFID montrent les mêmes deux boutons deux fois » — RÉFUTÉ**
  le 2026-08-27, et c'était déjà à moitié réfuté (`feedback-fabos-verify-pixels` :
  « un seul le faisait »). Compté sur les deux pages rendues :
  `/admin/rfid-readers` porte « Cancel », « Close » et « Delete permanently », tous
  distincts et venant de sa modale ; `/admin/access-rfid-logs` porte un seul
  « Confirm ». **Aucun doublon.** Rien à corriger — l'item est clos par la mesure.

## 🅿️ TODO (opérateur, 2026-09-03) — contrôle d'accès aux LIEUX → PHASE ESPACES

✅ **Ce todo a trouvé sa phase le 2026-09-04.** Les planches « Espaces & accès »
(`/admin/references`) et leur revue y répondent directement : elles introduisent
un **`AccessPoint` distinct de `Machine`** — porte, portail, casier, zone — ce qui
est exactement ce qui manquait, puisqu'un lecteur RFID est aujourd'hui rattaché
OBLIGATOIREMENT à une machine et ne peut pas représenter une porte sans machine
fictive. Le raisonnement ci-dessous reste valable ; il se planifiera **dans** la
phase Espaces, pas à côté.

> « Dans le module lieux, on pourrait rajouter de l'access control. Avec des
> boîtiers identiques à ceux des machines mais connectés à des gâches
> électriques. Idem, les droits d'accès au lieu seraient déduits comme ceux des
> machines. »

**Consigné, pas construit.** Ce qui suit est ce que le dépôt dit déjà de l'idée.

### ✅ Ce qui existe et se réutilise tel quel

Le **boîtier** : `RfidReader` porte `readerToken` (unique), `isActive`,
`lastSeenAt`. Le firmware appelle `MachineAccessService::authorize($machineToken,
$rfid)`, qui rend un verdict + une raison + un journal. Rien de tout ça n'est
spécifique à une machine, sauf la cible.

### 🔴 « Déduits comme ceux des machines » serait un RECUL, et c'est le point

L'accès **machine** se décide sur les **badges** (formations) :
`findRequiredForMachine` ∩ badges de la personne, booléen. Il ne sait rien des
jours ni des heures — « aucun badge requis » ouvre à 3 h du matin.

L'accès **lieu**, lui, est déjà entièrement décrit par le forfait : `PackageSpec`
porte `venuesAll/venues`, `daysAll/days`, `startTime/endTime`, `hoursExempt`. Un
verdict de porte est donc **déjà calculable** — appartenance aux groupes (S158/9)
→ forfaits → axes lieu + jours + horaires. C'est plus riche que le chemin machine,
et c'est exactement ce que le compilateur de S153 a été écrit pour dire.
🅿️ La vraie question n'est pas « comment copier les machines » mais **si les
machines doivent rejoindre ce chemin-là**. À trancher avant d'écrire une ligne.

### ⚠️ Ce qu'une PORTE a de plus qu'une machine

- 🔴 **Fail-safe / fail-secure est une question de sécurité incendie, pas de
  logiciel.** Une gâche doit libérer sur alarme, quoi que dise le serveur. Cette
  décision se prend avec l'installateur, et le code ne doit jamais pouvoir la
  contredire.
- 🔴 **Hors ligne, une machine reste éteinte ; une porte enferme.** Le modèle
  actuel est un appel HTTP en direct. Il faut dire ce que fait le boîtier quand
  le réseau tombe — avant de poser le premier.
- ⚠️ **La sortie n'est pas l'entrée.** Rien dans le modèle ne distingue les deux
  sens.
- ⚠️ **Les bornes horaires sont dans le fuseau du labo** (`LabClock`) : une porte
  qui ferme à 22 h se trompe silencieusement d'une heure deux fois par an.
- ⚠️ `RfidReader.machine` est la SEULE cible. Un lecteur sans cible, ou avec deux,
  doit refuser — pas ouvrir.

### ⚠️ Et ça change une ligne de ce fichier

« RFID physique et 2FA restent hors scope » (§ Travaux transversaux). Le boîtier
machine existe déjà côté logiciel ; c'est la **gâche** qui est neuve.

## 🟡 Une entité ne sait pas quelle heure il est au labo

**Trouvé le 2026-09-03, borné, écrit — pas corrigé.**

Les bornes d'appartenance (`USER_GROUP_MEMBER.validFrom/validUntil`) et celles des
attributions sont de « convention B » au sens de `LabClock` : l'heure **murale**
du labo, stockée telle quelle. `AudienceResolver` a reçu l'horloge et compare
juste. `Utilisateur::getRoles()`, lui, est appelée par la sécurité sans argument
et **ne peut pas atteindre `LabClock`** — une entité est hydratée par Doctrine,
pas construite par le conteneur.

🔴 **Conséquence mesurée : le rôle SURVIT jusqu'à l'offset du labo de trop** —
deux heures à Paris en été, et jamais dans l'autre sens. Une appartenance `staff`
qui finit le 30 juin à minuit accorde encore son rôle à 01:59. L'écran, lui, dit
« expirée ». Deux heures de divergence, permissives.

⚠️ **Ce n'est pas une régression** : c'est le balayage que `LabClock` déclare
« consigné, pas fait » depuis S38b, dont `Event`, `OpeningHours` et la validité
des passes font aussi partie. L'appartenance datée l'a simplement amené sur le
chemin de la sécurité.

🅿️ **Deux voies, et c'est une décision d'architecture :**
- **donner l'heure du labo à l'entité** — horloge posée au démarrage et lue
  statiquement. Marche pour tous les cas d'un coup, mais introduit un état global
  que ce dépôt n'a nulle part ;
- **stocker en UTC** ces colonnes-là et convertir à l'affichage. Exact et sans
  état global, mais c'est une migration de données **et** un changement de
  convention à propager aux attributions, sous peine d'une TROISIÈME convention.

⚠️ Tant que rien n'est tranché, ne pas « améliorer » un seul des deux lecteurs :
les faire diverger davantage est pire que l'écart actuel.

## ✅ Le cache-buster est centralisé — FAIT le 2026-09-03

**Mesuré le 2026-09-03.** Chaque écran admin écrit son propre
`asset('css/admin.css') ~ '?v=…'`. Toucher `admin.css` oblige donc à un `sed` sur
**72 fichiers**, et le diff de la session en devient illisible : la revue de code
de cette nuit a dû écarter « ~120 changements de gabarit qui ne sont que des
bumps » pour trouver les huit vrais. ⚠️ **C'est ainsi qu'un vrai changement se
cache** — et une modification oubliée dans le lot ne se voit plus.

✅ **Ce qui marche déjà et qu'il ne faut PAS casser** : les feuilles propres à une
page portent un buster daté qui correspond exactement au dernier changement du
fichier — vérifié sur les quatre (`machine-historique`, `formation-suivi`,
`home-deck`, `event-detail`). Ce n'est pas de la dérive, c'est la discipline qui
fonctionne. `style.css` n'est lié que dans 3 gabarits, dont `base.html.twig` :
là non plus, rien à changer.

✅ **Fait, et étendu au JS dans la foulée** : deux globales Twig dans
`config/packages/twig.yaml`, `css_version` (158 références, 15 feuilles) et
`js_version` (21 références). Séparées, pour qu'un changement de feuille
n'invalide pas le JS et réciproquement.
⚠️ **Le comportement ne change PAS** : les 179 références partageaient déjà la
même valeur, posée au `sed`. La centralisation rend explicite ce qui était déjà
vrai, et supprime les diffs de 120 fichiers qui cachaient les vrais changements.
✅ **Le risque signalé a été MESURÉ** : sans `strict_variables` en prod, une
globale absente rendrait un `?v=` vide en silence. Vérifié sur quatre pages plus
la **page d'erreur 404** — toutes portent une vraie version, zéro `?v=` vide.
🅿️ Les six busters restants sont volontairement littéraux : quatre feuilles
propres à une page, `badges.js` et le logo.

## 🅿️ Trois écarts venus du marché voisin (2026-09-03)

Dépouillement des 99 demandes de <https://feedback.fab-manager.com> → détail et
vérifications dans `FABMANAGER-ECARTS.md`. Sur les 22 mieux votées : **7 existent
déjà chez nous**, **8 sont du commerce** (Phase H), **7 sont de vrais écarts**.

🔴 **Leur demande n°1 est notre acquis** : le RFID (18 votes, *Planned* chez eux)
tourne ici depuis longtemps — et l'opérateur en demande l'extension aux lieux.

Les trois qui méritent une décision :
- **Personnaliser les gabarits d'e-mail** (10 votes) — `/admin/emails` règle
  l'ÉTAT du mail, pas son TEXTE. Petit, bien voté, **absent de notre plan** ;
- **Prévenir les membres à la création d'un événement** (7) — `EventMailer` ne
  sait qu'accompagner une inscription. Plus petit encore ;
- **OpenBadge pour valider une formation** (14) — c'est notre S125 (badges
  fédérés), très loin dans le plan. À faire remonter, ou pas : c'est une décision.

⚠️ **Signal faible, à traiter comme tel** : le plus haut score du tableau est 18.
Ça corrige des angles morts, ça ne réordonne pas une feuille de route.

## Travaux transversaux conservés

Sécurité restante de Phase H (**test réel du booking**, requêtes groupées) ·
verrou d'annulation et no-show sur ressources qui ont un signal · files d'attente,
stockage/retrait, motif d'utilisation · audit et notes sur toute action Manage
exercée sur autrui.

**RFID physique et 2FA restent hors scope.** La réservation d'un pool de machines
n'est pas impliquée par les catégories.
