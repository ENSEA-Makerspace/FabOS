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
| **O** | Machines & boîtiers (dont les 3 P0 de sécurité) | S171–S174 |
| **P** | Espaces & accès d'entrée (dont `AccessPoint`) | S175–S178 |
| **Q** | Formations (absorbe la messagerie de cohorte) | S179–S183 |
| **K** | Gabarits d'e-mail modifiables | S160–S162 |
| **L** | Annoncer un événement aux membres | S163–S164 |
| **M** | Thèmes, en profondeur | S165–S168 |
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
| **J-25** | ✅ **RÉGLÉ le 2026-09-04.** « Accès complet » (#20, les 4 capacités, SANS exemption d'horaires) est attribué à l'audience `user`. Mesuré : la portée passe de **3 personnes à 9**, et `machines` de **2 à 9**. ⚠️ S158/S159 avaient construit la route ; **personne n'était dessus** — un modèle complet dont aucune donnée n'emprunte le chemin se lit comme une panne | opérateur, 2026-09-04 | `app:j25:open-booking` |
| **J-8** | un champ refusé fait ressaisir le reste | ✅ **CLOS le 2026-09-05** — le dernier écran défaillant (`/profil`, branche profil public) rend désormais la page avec la saisie, prouvé par la sonde. Historique : **le chiffre de 15 était FAUX.** Au 2026-08-23 : **9 écrans prouvés sains** par un POST refusé (`app:s147:form-probe`, 13 sondes), 4 de plus convertis par le même mécanisme mais non sondés un par un. 🔴 **Reste `/profil`, branche « profil public » — le seul défaut prouvé, et il n'est pas admin** | S149 |
| **J-9** | trois maquettes S103 en prod, titres en dur, clés brutes à l'écran | ✅ **CADUC, mesuré le 2026-09-04** : `debug:router` ne connaît plus `design/droits-quotas`, `design/workspaces` ni `design/structure` — le nettoyage de S159 les a supprimées, page, route et lien. Le défaut n'a plus de sujet | S159 |
| **J-10** | formulaires les plus lourds | ✅ **CLOS le 2026-09-05** : la moitié « taux d'aide » est réglée et reformulée (voir Phase N), la moitié « écran de contenu, 35 champs » appartient à la Phase Q (S181). Historique : **l'éditeur de packages : 28 champs visibles à l'arrivée → 7** (les 4 éditeurs « ajouter » repliés, 2026-08-24). Barème et chiffres dans `S149-REVUE.md` § qualité des formulaires. Restent `admin-formation-content` (35) et le taux d'aide de **20 %** | S149+ |
| **J-23** | `/admin/usage-rights/shadow` : bascule finie, audit encore utile | ✅ **CADUC, mesuré le 2026-09-04** : la route n'existe plus (S159 l'a retirée, avec le retour arrière qui était devenu un piège). ⚠️ Le réglage `usage_rights_v2_*` reste en base pour une écriture explicite | S159 |
| **J-4** | « (s) » au lieu de pluriels ICU | ✅ 2026-08-24 — **77 clés** migrées, 5 langues, 0 « (s) » restant. Validateur statique : `tools/i18n/icu_audit.py` (395 motifs, 0 faute) | — |
| **J-5** | CSS local par page rendue | ✅ **CLOS PAR LA MESURE le 2026-09-05** : 544 sélecteurs locaux, **9 dupliqués dont 6 artefacts de comptage**, une seule duplication réelle laissée sciemment (voir Phase N). Historique : 708 → **653 règles dans 37 gabarits**. Les deux familles à duplication PROUVÉE sont rassemblées (kiosque, authentification) et ont révélé 2 défauts visibles. Le reste est du CSS réellement spécifique à sa page | S149+ |
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

## ✅ CE QUE L'OPÉRATEUR VÉRIFIE — Phase J (S169)

**Demandé le 2026-09-05 : après chaque phase, la liste de ce que le RELECTEUR
teste.** Elle est délibérément faite de gestes, pas de fichiers : ce que la
machine sait mesurer est déjà mesuré, et ce qu'elle ne sait pas voir est
exactement ce qui suit.

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

# Phase K — les gabarits d'e-mail deviennent modifiables (S160–S162)

**Demandé par l'opérateur le 2026-09-04**, sur la trouvaille du dépouillement
Fabmanager : *« Customize email templates »*, **10 votes**, et le seul écart à la
fois bien voté, petit, et absent de notre plan. Détail dans
`FABMANAGER-ECARTS.md`.

## Ce qui existe déjà, mesuré

- **23 gabarits Twig** dans `templates/emails/`, tous héritant de
  `_layout.html.twig`. Le sujet est un `{% block subject %}`.
- 🔴 **Le texte n'est PAS dans les gabarits : il est en CLÉS DE TRADUCTION**
  (`mail.event.registered.subject`), donc en cinq langues.
- `Mailer::queue()` enregistre `template` + `context` + `locale` dans le journal ;
  **le rendu a lieu plus tard, à l'envoi**, par le worker.
- ✅ `sendNow()` avec `NotificationCategory::TEST` existe déjà — l'aperçu et le
  « m'envoyer un test » sont donc à moitié construits.

## 🔴 La tension à trancher AVANT d'écrire une ligne

**Un texte modifié par l'opérateur est du CONTENU, pas de l'interface.** La règle
de la maison est explicite : *on traduit l'UI, jamais le contenu*. Donc une
surcharge est **par langue**, et les traductions livrées restent le repli. Il n'y
a pas de version « une seule langue » qui tienne : un lab bilingue qui ne
surcharge que le français casserait ses mails anglais s'il remplaçait la clé.

🔴 **Et l'opérateur n'écrira JAMAIS de Twig.** Laisser saisir du Twig, c'est
offrir l'exécution de code arbitraire dans un gabarit. Deux issues seulement :
le bac à sable Twig, ou une syntaxe de champs restreinte (`{{ event }}`) validée
à l'enregistrement. **La seconde est recommandée** : elle est vérifiable, elle
n'a pas de surface d'évasion, et personne n'a demandé de boucles dans un e-mail.

⚠️ **Le rendu a lieu à l'ENVOI, pas à la mise en file.** Un gabarit modifié entre
les deux change le mail déjà en attente. À trancher : figer le rendu à la mise en
file, ou rendre à l'envoi. Rendre à l'envoi est plus simple et cohérent avec
l'existant — mais alors **une surcharge cassée ne doit jamais empêcher un mail
transactionnel de partir**.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S160** | Le modèle et le REPLI, sans éditeur. Une surcharge `(templateKey, locale, subject, body)`, lue à l'envoi, qui retombe sur le gabarit livré dès qu'elle manque, est vide ou lève | 🔴 **Sans aucune surcharge, les 23 mails rendent exactement ce qu'ils rendent aujourd'hui** — comparaison octet à octet, sinon la phase a déjà cassé quelque chose |
| **S161** | L'éditeur : un écran par gabarit et par langue, la liste des champs disponibles **pour ce gabarit-là**, refus d'un champ inconnu, aperçu et envoi de test | Un champ inconnu est refusé avec une phrase ; l'aperçu rend le vrai gabarit, pas une approximation |
| **S162** | L'en-tête et le pied (`_layout`) surchargeables séparément ; « revenir au texte livré » par gabarit ; la garde du transactionnel | 🔴 Une surcharge volontairement cassée sur `password_reset` : le mail part quand même, avec le texte livré, et l'incident est journalisé |

## Critères de sortie

- Aucune surcharge en base ⇒ aucun changement visible nulle part.
- 🔴 **Un mot de passe oublié part toujours**, quelle que soit la bêtise saisie.
- Une surcharge s'applique dans la langue du destinataire, et seulement là.
- ⚠️ Le journal des mails dit **quelle version** a servi — livrée ou surchargée —
  sans quoi « pourquoi ce mail dit ça ? » est insoluble.

---

# Phase L — annoncer un événement aux membres (S163–S164)

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
| **S163** | La diffusion : catégorie `NEWS`, une file par destinataire **dans SA langue**, lien de désabonnement, et une **trace par événement** qui rend l'envoi idempotent | 🔴 Deux clics sur « Annoncer » n'envoient qu'une fois ; un membre désabonné de `NEWS` ne reçoit rien et **garde** ses mails d'inscription |
| **S164** | Le geste : compte avant envoi (« ceci écrira à N personnes »), confirmation, et l'état sur la fiche (« annoncé le … à N personnes ») | Le compte annoncé est celui réellement mis en file ; l'écran dit quand l'annonce a déjà eu lieu |

🅿️ **Ce qui n'est PAS dans cette phase, volontairement** : le *digest* périodique
(« un résumé hebdomadaire des événements à venir »), que la demande d'origine
mentionne aussi. Il est moins intrusif et sans doute meilleur — mais c'est une
autre mécanique (planification, fenêtre, regroupement) et il n'a de sens qu'une
fois la diffusion unitaire éprouvée.

⚠️ **Indépendante de la Phase K.** Si K est livrée d'abord, le texte de l'annonce
est modifiable sans travail supplémentaire ; sinon il vit en clés de traduction
comme les 23 autres.

---

# Phase M — les thèmes, en profondeur (S165–S168)

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
| **S165** | La **médiathèque d'identité** : téléversement, validation, renommage serveur, identifiant stable, suppression refusée tant qu'un thème référence le fichier. Fin du chemin libre. Renommage `portal_logo_path` → `site_logo` | 🔴 On pose un logo **sans toucher au serveur** ; un fichier référencé ne se supprime pas ; ⚠️ l'orientation EXIF est lue AVANT les dimensions, et `exif_read_data()` ne lit pas le PNG |
| **S166** | L'**éditeur guidé** : palette avec contrastes, rayon / typo / densité en préréglages, variantes de logo (clair, sombre, compact, favicon, image de partage). 🔴 **ET LES 66 COULEURS DE MARQUE ÉCRITES EN DUR**, mesurées le 2026-09-05 : `#9E1B56` et `#6b7280` apparaissent **66 fois dans les gabarits du SITE** — `register` 11, `_formation_visual` 10, `person-booking`, `login`, `machine-detail` 6 chacun. Un éditeur de palette qui laisse 66 endroits ignorer la palette ne change pas le thème, il le contredit. ⚠️ **Les 41 occurrences des E-MAILS ne comptent pas** : un client de messagerie ne sait pas lire `var()`, la couleur littérale y est la bonne réponse | 🔴 **Le contraste est MESURÉ, pas affirmé** — c'est déjà la pratique du dépôt (7,65:1 relevé sur une proposition de tableau de bord). Une palette qui échoue est refusée, pas signalée |
| **S167** | L'**aperçu sur de VRAIES surfaces** : accueil, catalogue, détail, un écran admin, un kiosk — desktop et mobile, clair et sombre. Publication **atomique** des réglages ET des fichiers | 🔴 L'aperçu rend les vraies pages, pas des vignettes dessinées à la main : c'est la leçon de `feedback-fabos-verify-pixels`, où un balisage présent ne prouvait pas qu'on le voyait |
| **S168** | **Kiosks et navigation** : aucun favicon, logo ou couleur en dur ne survit dans un kiosk ; ordre et visibilité des entrées de menu, destinations limitées aux routes autorisées, entrées système protégées | 🔴 Une page dépubliée rétablit l'accueil FabOS **avec trace**, sans page blanche ni boucle de redirection |

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

**1. 🔴 L'API des boîtiers échoue en position OUVERTE.**
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

**2. ⚠️ `/kiosk/entries` est PUBLIC et montre qui est passé.** Aucun `IsGranted`,
répond 200 sans session, et rend 49 références d'avatar — noms et passages RFID.
⚠️ Le site entier est derrière une liste blanche NPM, donc ce n'est pas exposé à
Internet aujourd'hui : c'est une protection d'INFRASTRUCTURE, pas une garde de
l'application. À décider explicitement — signalétique anonymisée, ou kiosk
authentifié par le boîtier.

**3. 🔴 Le formulaire Lecteur apprend à donner la base de données à un boîtier.**
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
| **J-10**, moitié « taux d'aide » | ✅ **CLOS le 2026-09-05, et le critère a été REFORMULÉ.** Le taux n'est pas l'objectif : le compléter à l'aveugle produit du bruit. Le vrai critère de S149 est « des écrans à ZÉRO aide pour 8 champs ou plus ». Mesuré : il en restait **quatre**, il en reste **trois**, et chacun est déjà dans une phase — `LoanAdminType` (10 champs) → Phase T, `PlaceAdminType` (9) → Phase P, `MaintenanceTaskAdminType` (8) → Phase O. Les y traiter coûte zéro travail supplémentaire ; les traiter ici serait refaire demain un formulaire qu'on retouche aujourd'hui. ⚠️ Et `PackageSpecType`, que la mesure accusait à 0/14, explique dans son GABARIT (5 aides) : **ce n'était pas un défaut**. Le taux global est passé de 20 % à 32 %. Historique du barème dans `S149-REVUE.md`. ⚠️ **L'autre moitié — `admin-formation-content` et ses 35 champs — appartient à la Phase Q (S181)** : c'est l'écran de contenu d'une formation, il se refait avec le constructeur, pas à côté. Deux phases revendiquaient J-10 dans la première version de ce plan ; c'est corrigé. Barème dans `S149-REVUE.md` |
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

# Phase O — Machines & boîtiers (S171–S174)

**Planifiée le 2026-09-04**, d'après les huit planches et la revue Sol
(`/admin/references`). Absorbe les **trois P0 de sécurité** vérifiés le même jour.

## 🔴 La règle des trois phases issues des planches — lire AVANT de les ouvrir

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

## Ce qui existe déjà, mesuré

`Machine`, `MachineDocument` (S152), `MachineFavorite`, `MaintenanceTask`,
`RfidReader`, `AccessRfidLog`, `MachineAccessService`, `Material` +
`MACHINE_MATERIAL`, les kiosques, `/admin/machines` et `/machines/{id}`.
🔴 **Et `Machine::materials`, un tableau texte, EN PLUS de la relation** — deux
sources de vérité, dont l'une retombe sur une liste codée en dur
(`['PLA','PETG','TPU','Support']`) quand elle est vide. Une découpeuse sans
matériaux annoncerait donc du PLA.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S171** | 🔴 **La sécurité des boîtiers, d'abord.** Garde `fail-closed` quand `FABOS_RFID_API_TOKEN` manque ; retrait de l'exemple `.env` qui donne `FABOS_DB_*` à un boîtier ; décision explicite sur `/kiosk/entries` | Un POST sans en-tête rend **401**, pas 404. ✅ Sans risque mesuré : un seul lecteur, vu la dernière fois le 2026-07-10 |
| **S172** | **Identité et santé d'un boîtier** : secret propre au device, révélé UNE fois, rotation et révocation, dernière connexion, état réel — prêt / hors ligne / non configuré / erreur / association invalide — au lieu d'un booléen plus `lastSeenAt` | Chacun des cinq états est atteignable et distinguable à l'écran |
| **S173** | **La fiche machine se sépare en deux publics** : membre (statut utilisable, prochaine action, prérequis exacts, matériaux compatibles, réserver) et une zone **Exploitation** staff/admin. ⚠️ La page RESTE une page — pas deux routes, pas un shell neuf | Le membre atteint « puis-je l'utiliser ? » sans quitter la fiche ; le staff ne voit plus ses outils mélangés au contenu public |
| **S174** | **La matière devient une seule vérité** : `MACHINE_MATERIAL` canonique, `Machine::materials` rétrogradé en note de transition, et une fiche `/materiaux/{id}` avec les machines réellement compatibles. **Plus de liste codée en dur** | 🔴 Aucune machine n'annonce un matériau qu'elle ne prend pas ; le repli en dur n'existe plus |

## La passe de fond de cette phase

⚠️ Une phase qui ne fait que sa fonctionnalité laisse le socle où il était.
- **Réemploi** : les patterns locaux de la fiche machine (matériaux, maintenance)
  remontent dans le système de design s'ils servent ailleurs — sinon ils restent,
  et on l'écrit.
- **Conformité** : `tools/dead_affordances.py`, `tools/a11y_static.py`,
  `tools/form_placement.py` et `tools/ctor_arity.py` passés en début ET en fin de
  phase, l'écart commenté.
- ⚠️ **Le kiosque garde favicon, CSS et styles locaux** : soit il rejoint le shell
  et le thème publié (ce que la phase Thèmes demande aussi), soit on écrit
  pourquoi il reste à part. Pas de troisième option silencieuse.

---

# Phase P — Espaces & accès d'entrée (S175–S178)

**Planifiée le 2026-09-04**, d'après les huit planches et la revue Sol. Absorbe le
todo « contrôle d'accès aux LIEUX » du 2026-09-03.
⚠️ **La règle de lecture des planches est en tête de la Phase O** : présentation
de contenu uniquement, jamais le thème, et pas une liste de tâches.

## 🔴 Le fait de modèle qui commande toute la phase

**Un lecteur RFID est aujourd'hui rattaché OBLIGATOIREMENT à une machine.** Une
porte ne peut donc être représentée qu'en inventant une machine fictive — ce qui
est exactement le genre de contournement qui se paie deux ans plus tard. La revue
propose `AccessPoint`, distinct de `Machine` : porte, portail, casier, zone.
✅ C'est la réponse au todo de l'opérateur sur les gâches électriques.

⚠️ **Et le verdict d'une porte est plus riche que celui d'une machine** : les axes
lieu / jours / horaires d'un forfait le décrivent DÉJÀ (`PackageSpec`), là où
l'accès machine est un booléen sur les badges. On ne réinvente rien ; on branche.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S175** | `AccessPoint`, et le lecteur s'y rattache aussi bien qu'à une machine. **Migration additive**, aucun lecteur existant déplacé | Un lecteur existant continue de répondre exactement comme avant — comparaison avant/après, annulée sinon |
| **S176** | **La mise en service** : créer → associer porte/lieu → révéler le secret UNE fois → tester la connexion. Et les **incidents** d'accès actionnables : une cause mène vers le membre, le badge, la formation, le lecteur | Un refus se corrige depuis l'incident, sans chercher dans un journal |
| **S177** | **Le parcours membre** : disponibilité lisible dès la carte (« Disponible à 14:00 » plutôt que « Occupé »), fiche d'espace qui répond « puis-je réserver, quand, qu'est-ce qui est inclus, comment j'entre », et « Mes réservations » avec la prochaine et sa fenêtre d'accès | 🔴 **Compté en clics**, cibles de la revue : trouver un espace libre 1–2, réserver 2–3, retrouver sa réservation 1 |
| **S178** | **L'accès temporaire lié à une réservation** : une marge courte avant/après, révoqué à l'annulation. Et `Espaces > Exploitation` : réservations proches, fermetures, points hors ligne, refus | 🔴 Annuler une réservation retire l'accès **immédiatement**, prouvé par une sonde |

## La passe de fond de cette phase

- **Réemploi** : le calendrier, les créneaux et les politiques de réservation
  existent — cette phase ne doit pas en écrire une seconde version.
- ⚠️ **Le kiosque d'entrée est PUBLIC** : ni identité, ni UID de badge, ni journal,
  ni secret. C'est un critère de sortie, pas une intention.
- **Conformité** : mêmes outils, début et fin.

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
| **S180** | **L'étape PRATIQUE** : demander une évaluation, la file des validations pour l'équipe, la validation elle-même. C'est le chaînon qui manque entre « quiz réussi » et « badge » | Un badge ne s'obtient plus que par un chemin complet et tracé |
| **S181** | **Le constructeur** : le parcours en étapes ordonnables, la checklist de mise en ligne, l'aperçu apprenant. ⚠️ **Et c'est là qu'on solde J-10** — les 35 champs deviennent des étapes, pas un formulaire | 🔴 Champs visibles à l'arrivée : 35 → cible **sous 12**, barème de `S149-REVUE.md` |
| **S182** | **Le quiz** : types de questions, résultat et reprise. Sur l'existant, pas un moteur neuf | Une reprise ne réinitialise pas ce qui était acquis |
| **S183** | **La messagerie de cohorte** (ex-Phase I) : annonce formateur → cohorte sans exposer la liste, fil privé, groupe explicite. 🔴 **Aucun message privé ne bascule implicitement vers la cohorte** | Une annonce n'expose aucune adresse ; un fil privé le reste |

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

`Utilisateur`, l'inscription, `/profil`, la vérification d'e-mail, les groupes et
leurs droits (S158/S159), l'annuaire `/admin/utilisateurs` avec ses filtres.
🔴 **Ce qui n'existe pas** : le MFA, la gestion des sessions, une récupération de
compte non divulguante, et un parcours d'adhésion.

| Session | Livre | Ce qu'on mesure |
|---|---|---|
| **S189** | **L'entrée** : inscription courte qui annonce ses prochaines étapes, activation par e-mail avec renvoi et correction d'adresse — **sans impasse** | Une adresse mal tapée se corrige sans recréer un compte |
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
