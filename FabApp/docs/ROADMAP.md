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

# Phase J — « boutonner » ⬅️ EN COURS

**Demande opérateur, 2026-08-21** : *« before commerce i want to smooth out a lot
of things… act like apple engineers and button everything up. »*

🔴 **BLOQUANTE avant la Phase H.** G était la barrière du MODÈLE ; J est celle de
la FINITION. Vendre une surface non finie fige ses défauts dans un contrat client.

## Les dix points — un écran est fini quand les dix réponses sont oui

1. **Coquille partagée.** Pas de `<head>` maison, pas de `<style>` local qui ne
   soit pas devenu une règle du guide.
2. **Le titre vient de `NavBuilder`**, jamais recopié.
3. **Listes** : cinq colonnes max, actions comprises ; autant de cellules que
   d'en-têtes ; pas de `colspan` compté à la main.
4. **Chaque objet annoncé est créable, éditable, archivable** depuis son workspace.
5. **Aucune affordance morte** : pas de bouton qui n'aboutit pas, pas de lien qui
   404, pas de filtre qui ne filtre rien.
6. **Cinq langues, sombre, mobile, clavier.** Vérifié à l'écran, pas supposé.
7. **Le nombre de clics est COMPTÉ**, avant et après.
8. 🔴 **Un champ invalide ne fait JAMAIS ressaisir le reste du formulaire.** Prouvé
   par un vrai POST refusé, pas relu.
9. **Zéro champ non indispensable.**
10. **Les primitives sont dans `/admin/design`** avec le vrai composant.

## Découpage

| Étape | Livre | Qui |
|---|---|---|
| ✅ **S147 — LA REVUE** | passée 2026-08-22, aucun code hors la sonde. 146 pages rendues + passe navigateur (375/768/1280, cascade, clavier, sombre, un vrai POST refusé) → **25 défauts J-1…J-25**. Détail : `S147-REVUE.md` | Terra mesure, opérateur arbitre |
| **S148 — le socle** | Réglages, Fonctionnalités, E-mails, Logs RFID, Thèmes, Setup/assistant, Tableau de bord. ⚠️ Absorbe ce qui restait de S132 | Luna + Terra |
| **S149 — feature par feature** | machines, espaces, événements, formations, prêts, matériaux, badges, projets, réservations, packages/quotas. Chacune finie selon les dix points | Luna + Terra |
| **S149z — la sortie** | revue conjointe finale : la liste S147 est vide ou consciemment reportée | Opérateur + Terra |

⚠️ **La revue vient EN PREMIER et elle ne code pas.** Un chiffre inventé a cadré
une session entière (S134j). On mesure, on montre, on décide, puis on fait.
⚠️ **La revue de fin est UNE FOIS PAR PHASE**, pas par étape (opérateur,
2026-08-20). Mandat « designer d'Apple ». Lui donner les **URLs et les parcours**,
pas le diff.

## Les 25 défauts — état au 2026-08-23

| # | Défaut | État | Étape |
|---|---|---|---|
| **J-25** | 🔴🔴 **aucun membre ne peut réserver en ligne** — les 4 chokepoints répondent `DENIED / missing_package` à tout non-admin | ⏭️ **décision opérateur, tout de suite** | — |
| **J-8** | un champ refusé fait ressaisir le reste | 🔶 **le chiffre de 15 était FAUX.** Au 2026-08-23 : **9 écrans prouvés sains** par un POST refusé (`app:s147:form-probe`, 13 sondes), 4 de plus convertis par le même mécanisme mais non sondés un par un. 🔴 **Reste `/profil`, branche « profil public » — le seul défaut prouvé, et il n'est pas admin** | S149 |
| **J-9** | trois maquettes S103 en prod, titres en dur, clés brutes à l'écran | 🔶 partiel | S148 — ⏭️ décision opérateur |
| **J-10** | formulaires les plus lourds | 🔶 **l'éditeur de packages : 28 champs visibles à l'arrivée → 7** (les 4 éditeurs « ajouter » repliés, 2026-08-24). Barème et chiffres dans `S149-REVUE.md` § qualité des formulaires. Restent `admin-formation-content` (35) et le taux d'aide de **20 %** | S149+ |
| **J-23** | `/admin/usage-rights/shadow` : bascule finie, audit encore utile | ⚠️ **amendé par J-25 — le levier RESTE** | S148 |
| **J-4** | « (s) » au lieu de pluriels ICU | ✅ 2026-08-24 — **77 clés** migrées, 5 langues, 0 « (s) » restant. Validateur statique : `tools/i18n/icu_audit.py` (395 motifs, 0 faute) | — |
| **J-5** | CSS local par page rendue | 🔶 708 → **653 règles dans 37 gabarits**. Les deux familles à duplication PROUVÉE sont rassemblées (kiosque, authentification) et ont révélé 2 défauts visibles. Le reste est du CSS réellement spécifique à sa page | S149+ |
| **J-7** | emoji bruts comme icônes | ✅ 2026-08-24 — **0 emoji d'interface** sur 16 pages rendues. Les 33 signes typographiques restants sont une décision écrite en tête de `_icon.html.twig` | — |
| **J-22** | formulaires admin hors thème | ✅ 2026-08-23 — **27 conversions**, 13 écrans, 13 sondes vertes. Restent, écrits : la matrice de fonctionnalités (partial partagé), 5 filtres GET, la semaine d'horaires, les contrôles en boucle des tableaux | — |
| **J-1** | déploiement partiel : l'upload d'images fatalait en prod | ✅ 2026-08-22 | — |
| **J-2** | huit objets se supprimaient en dur | ✅ 2026-08-22. ⚠️ **Reste à vérifier la promesse S134f** : archiver une ressource réservable doit annuler ses réservations à venir | — |
| **J-3** | flashs en dur | ✅ 37 → **0** | — |
| **J-6** | `style=""` sur `/admin/utilisateurs/{id}` | ✅ 78 → **1** | — |
| **J-11** | `/machines/{id}` cassé sur téléphone | ✅ 2026-08-22 | — |
| **J-12** | barre d'outils du calendrier | ✅ 2026-08-22 — ⚠️ le constat initial était faux, 5 contrôles réellement inatteignables | — |
| **J-13** | « Réserver une machine » menait au calendrier lecture seule | ✅ 2026-08-22 | — |
| **J-14** | pas de lien d'évitement, focus invisible | ✅ | — |
| **J-15** | fonds clairs sans variante sombre | ✅ 101 → **0** | — |
| **J-16** | `/formations/{id}/suivi` imprimait ID/titre/slug au public | ✅ | — |
| **J-17** | `/machines/{id}` « Connexion requise » ×4, favoris mort | ✅ | — |
| **J-18** | `/admin/maintenance/batch` sans lien | ✅ | — |
| **J-19** | « Loans » ouvrait le catalogue d'objets | ✅ | — |
| **J-20** | le calendrier ignorait les plages horaires | ✅ 2026-08-22 | — |
| **J-21** | catégorie d'un grant comparée par libellé exact | ✅ 2026-08-23 — l'identifiant décide | — |
| **J-24** | messages de validation en français en dur | ✅ 69 → **0**, cinq langues complètes | — |

⚠️ **Rien de cette liste n'est reporté hors de la Phase J.**

## Critères de sortie

- la liste S147 est vide, ou chaque reste est **consciemment reporté et écrit** ;
- **aucun gabarit ne porte de `<style>` local** hors `admin-design`, ou chaque
  exception est une règle nommée du guide ;
- ✅ **gabarits à `<head>` propre : ATTEINT** — ils sont **5** (`event-ticket` +
  4 kiosques), et c'est l'exception que ce critère prévoyait ;
- les dix points passent sur **chaque** écran du socle et des features ;
- `/admin/design` montre chaque primitive utilisée, avec le vrai composant.

## 🅿️ Parqué — n'entre PAS dans J

Sélecteur de langue (`app_switch_locale` n'est lié nulle part) · suppression en
masse d'événements · catégories comme entrées de menu. Ce sont des
fonctionnalités, pas de la finition. ⚠️ Le tableau de bord « qui doit re-briller »
est le seul des quatre qui touche J : il est dans **S148**.

## ✅ `/prets/{id}` n'avait pas de navigation — corrigé le 2026-08-27

**Signalé** : *« the whole menu disappears in that page! »* — https://fabos.dstei.fr/prets/1

`loan-item.html.twig` étendait **`base.html.twig`**, la coquille nue : elle n'a ni
bloc `header` ni `include` de `_header`, seulement un `{% block body %}` et un pied
de page. Les 39 autres gabarits qui l'étendent sont des écrans admin ou staff, et
eux reçoivent l'en-tête par `_admin_list.html.twig` ; celui-ci n'incluait rien.
⚠️ Le pied de page était bien là — d'où « le menu disparaît » et non « la page est
nue ». Balayage de 29 pages publiques : c'était **la seule**.

Corrigé en une ligne — `{% extends 'site/base_public.html.twig' %}`, comme `/prets`.
Les trois blocs utilisés (`title`, `stylesheets`, `body`) existent des deux côtés.
Au passage, son cache-buster était resté sur `?v=20260816-s134`.

**Vérifié sur la page en ligne** : en-tête, navigation, recherche et bouton de
connexion sont revenus, « Fablab » s'allume comme section active. Nouveau balayage,
**31 pages publiques cette fois** (avec `/prets/2` et `/lab/3`) : **0 sans coquille**.

---

## 🅿️ Une proposition d'écran « événements », d'après Fabmanager (opérateur, 2026-08-27)

✅ **La première moitié est faite et regardable : `/admin/design#evenements`.**
Les **six affiches de remplacement** y sont rendues, chacune avec sa géométrie —
pas seulement sa couleur, parce qu'à luminosité égale six teintes seraient la même
image. Elles vivent dans `templates/site/_event_placeholder.html.twig`.
- 🔴 **Le tirage est stable** : `id % 6`, jamais `random()`. Un tirage par rendu
  ferait changer l'affiche à chaque rechargement et deux membres ne verraient pas
  la même page.
- ✅ **La question « dark/light » s'annule** : en DESSINANT au lieu de téléverser,
  `var(--color-primary)` et `var(--tone-primary-soft)` suivent le thème du membre.
  Mesuré sur la page rendue — fond `srgb 0.223 0.133 0.223` en sombre,
  `srgb 0.954 0.893 0.920` en clair, **un seul fichier**. Pas douze PNG, pas de
  préférence à lire.
🅿️ **Ce qui reste de cette moitié** : téléverser SES propres logos pour qu'ils
entrent dans le tirage. Ça demande une table, donc une migration, donc l'opérateur
— et ça se décide après avoir jugé les six.
✅ **La seconde moitié aussi** — le regroupement par mois est un spécimen dans la
même section, rendu avec les VRAIES classes du catalogue. « AOÛT · 1 événement »
puis « SEPTEMBRE · 4 » : la hauteur des blocs dit le volume avant le compte.
🔴 **La question à trancher n'est pas graphique** : `/events` passe par
`_catalogue.html.twig`, partagé avec six autres listes. Un en-tête de mois veut
dire soit une grille PAR mois (le spécimen — et les cartes du dernier mois ne
s'alignent plus sur le précédent), soit un `grid-column: 1 / -1` dans une grille
unique, qui garde l'alignement mais demande au shell une notion de « séparateur »
qu'aucune autre liste n'a. ⚠️ Et le regroupement ne vaut que pour les objets
DATÉS : une machine n'a rien à regrouper.



**Source** : trois captures de Fabmanager (instance Technistub) décrites dans
`Stage/Drive/Images/Fabmanager UI/README.md` — événements, formations, machines.
⚠️ **Fabmanager, pas Fabman** : c'est une seconde source, distincte des 73 captures
qui ont donné le barème de qualité de formulaire.

**La demande** : une **page d'exemple** d'une version améliorée de nos événements.
Donc une proposition à regarder, pas un remplacement à déployer — elle passe par
`/admin/design` en propositions comparables, comme le format de liste
([[feedback-fabos-design-review-loop]]), et la revue designer est **une fois par
phase**.

### Ce que l'opérateur retient de la référence

1. ✅ **Les dates sont visibles**, et fortes : « Le 28/08/2026 » en rouge et en gras
   est l'élément le plus lourd de la carte, l'horaire juste dessous en plus petit.
2. ✅ **Le regroupement par mois donne le VOLUME d'un coup d'œil** — « AOÛT, 2026 »
   avec une carte, « SEPTEMBRE, 2026 » avec neuf : on voit que la rentrée est
   chargée sans lire une seule ligne. Une liste à plat ne le dit pas.
3. ✅ **Le logo en remplacement d'affiche est malin** — la moitié image d'une carte
   sans photo est remplie par le logo du lab, pas par un vide ni une icône générique.

### Le point neuf : plusieurs logos de remplacement, tirés au sort

⚠️ **Le défaut de la référence est justement là** : toutes les cartes portent le
même logo, donc quinze cartes identiques. Le remède devient le symptôme.

**Ce qui est demandé :**
- pouvoir **enregistrer un OU PLUSIEURS logos** de remplacement, affectés
  **aléatoirement** aux événements sans affiche ;
- **six images par défaut livrées avec FabOS**, un pseudo-logo décliné en
  **variations de couleurs proches du thème par défaut** ;
- **une variante claire et une variante sombre**, pour suivre la préférence
  d'affichage du membre.

**Ce qu'il faut trancher avant de dessiner :**
- 🔴 **« Aléatoire » doit être STABLE.** Un tirage à chaque rendu fait changer
  l'image d'un événement à chaque rechargement, et deux membres ne voient pas la
  même page. Le tirage doit être une fonction de l'id de l'événement
  (`id % nombre_de_logos`), pas de `rand()`.
- ⚠️ **Clair/sombre : deux fichiers, ou un SVG qui suit `currentColor` ?** Le second
  est la façon dont le jeu d'icônes est déjà fait (`_icon.html.twig`) et il n'a
  besoin d'aucune préférence à lire. À comparer avant de produire douze PNG.
  ⚠️ Rappel : `--color-text-inverse` vaut `#FFFFFF` et **n'est jamais redéfini en
  sombre** — une image qui s'appuie dessus reste blanche sur fond sombre.
- ⚠️ **Où vivent les logos téléversés** : `public/uploads/<famille>/`, motif de
  `AdminController` ~3282, et la même question que pour les documents machine —
  supprimer la ligne n'efface pas le fichier.
- ⚠️ **Nos cartes disent déjà des choses que la référence ne dit pas** : garder
  l'état, le prochain créneau et les compteurs. La comparaison des trois captures
  (fin du README de référence) montre que Fabmanager ne les a pas.

### Ce qu'on ne copie PAS

- Le pied de carte à deux verbes (« Réserver · Consulter ») vient des écrans
  **formations et machines** de Fabmanager, pas de celui des événements. Il vaut
  d'être discuté pour NOS cartes machine — l'opérateur avait justement signalé
  « je n'ai que le bouton Voir » — mais c'est un autre sujet, à ne pas glisser dans
  celui-ci.

---

## ✅ Documents attachés à une machine — FAIT le 2026-08-28

**Demandé** : *« add on the machine pages files to download related to each
machine, example: usage guide, safety sheet, etc. »*

Un bloc « Documents attachés » sur `/admin/machines/{id}/edit` — téléverser,
retirer — et un onglet « Documents » sur la fiche publique, dans la barre qui
existait déjà. Migration `Version20260828100000` passée par l'opérateur le
2026-08-28 à 08:32, **avant** le déploiement du code qui lit la table.

🔴 **Ces fichiers sont PUBLICS.** Ils vivent sous `public/uploads/`, donc leur
adresse suffit. C'est le bon défaut pour une fiche de sécurité — elle se lit AVANT
d'être formé — et l'écran d'admin prévient en toutes lettres de ne rien y mettre
d'interne. 🅿️ **Le jour où un document devra être réservé aux membres, il faudra
le sortir de `public/`** : un contrôle d'accès devant un fichier que le serveur web
sert directement ne contrôle rien.

**Les décisions prises, pour ne pas les re-litiger :**
- le type est **constaté** (`getMimeType()`, finfo), jamais annoncé — liste blanche
  de douze types ;
- le nom sur le disque est construit à partir du type constaté, jamais du nom
  envoyé (qui peut contenir des `../`), mais `originalName` est rendu au
  téléchargement ;
- **supprimer efface aussi le fichier**, contrairement aux avatars : l'octet est
  public, le laisser laisserait une fiche retirée toujours lisible ;
- l'onglet public n'apparaît que s'il y a des documents ;
- le formulaire d'ajout est un `<form>` à part, APRÈS `_machine_form` — imbriquer
  des formulaires est interdit en HTML.

⚠️ **Le piège qui a mordu au premier déploiement** : `naming_strategy: underscore`
dans `doctrine.yaml` faisait chercher `stored_name` là où la table dit
`storedName`. L'entité déclare ses noms de colonnes explicitement.

✅ Sonde d'écriture `app:s152:document-probe` verte. ⚠️ Elle ne couvre pas le
téléversement HTTP lui-même (validation de type, déplacement du fichier), qui
demande un vrai POST authentifié.

🅿️ **Reste à faire, et c'est à l'opérateur** : y déposer les vrais documents.

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

# Phase H — commerce facultatif (S150–S154)

🔴 **BLOQUÉE PAR LA PHASE J** (opérateur, 2026-08-21).

Entièrement désactivable. Offres dans leur workspace métier, moteur commun pour
commandes / paiements / rapprochement.

- 🔴 **Le retour navigateur ne confirme JAMAIS un paiement** — seul un webhook
  vérifié ou sa réconciliation.
- Clé unique par événement fournisseur ; fulfillment persistant / outbox par ligne
  → effet **exactement une fois** malgré retries et crashs.
- La livraison passe par le service métier normal : sans toucher voter, badge,
  quota ni réservation.
- Ni carte ni credentials fournisseur en base FabOS.

| Session | Livre |
|---|---|
| **S150** | catalogue d'offres et prix ; aucune transaction |
| **S151** | commandes, paiements, webhooks, réconciliation, remboursements, audit |
| **S152** | livraison packages et matériaux ; hold stock atomique ou backorder explicite |
| **S153** | ledger append-only des crédits de temps et achats de formation |
| **S154** | reporting commerce, rapprochement, audit UX |

---

# Phase I — messagerie Formation (S155–S157)

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

## S144e — « ce package touche N personnes »

`readiness()` et la liste comptent les attributions **directes** ; un package tenu
seulement par un groupe annonce 0. Il faut un
`AudienceResolver::memberIdsFor($groupKey)` **inverse exact** de `keysFor()` :
rôles + lignes `USER_GROUP_MEMBER` + l'audience virtuelle `user`.
⚠️ **Ne pas réécrire cette logique à côté.**

## Thèmes — le chantier entier (session dédiée, non planifiée)

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

## 🅿️ TODO (opérateur, 2026-09-03) — contrôle d'accès aux LIEUX

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

## Travaux transversaux conservés

Sécurité restante de Phase H (**test réel du booking**, requêtes groupées) ·
verrou d'annulation et no-show sur ressources qui ont un signal · files d'attente,
stockage/retrait, motif d'utilisation · audit et notes sur toute action Manage
exercée sur autrui.

**RFID physique et 2FA restent hors scope.** La réservation d'un pool de machines
n'est pas impliquée par les catégories.
