# Phase J — « boutonner » (S147 → S169)

**2026-08-21 au 2026-09-05, CLOSE le 2026-09-05.** Partie d'une phrase de
l'opérateur : *« before commerce i want to smooth out a lot of things… act like
apple engineers and button everything up. »*

🔴 **Elle était BLOQUANTE avant la Phase H (commerce).** G était la barrière du
MODÈLE ; J était celle de la FINITION. Vendre une surface non finie fige ses
défauts dans un contrat client.

---

## Les dix points — un écran est fini quand les dix réponses sont oui

C'est le barème de la phase, et il reste le barème de tout écran neuf.

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

## Le découpage, et pourquoi la revue passe en premier

| Étape | Livré | Qui |
|---|---|---|
| ✅ **S147 — LA REVUE** | passée 2026-08-22, aucun code hors la sonde. 146 pages rendues + passe navigateur (375/768/1280, cascade, clavier, sombre, un vrai POST refusé) → **25 défauts J-1…J-25**. Détail : `S147-REVUE.md` | Terra mesure, opérateur arbitre |
| **S148 — le socle** | Réglages, Fonctionnalités, E-mails, Logs RFID, Thèmes, Setup/assistant, Tableau de bord. ⚠️ Absorbait ce qui restait de S132 | Luna + Terra |
| **S149 — feature par feature** | machines, espaces, événements, formations, prêts, matériaux, badges, projets, réservations, packages/quotas. Chacune finie selon les dix points | Luna + Terra |
| **S149z — la sortie** | revue conjointe finale : la liste S147 vide ou consciemment reportée | Opérateur + Terra |

⚠️ **La revue vient EN PREMIER et elle ne code pas.** Un chiffre inventé avait
cadré une session entière (S134j). On mesure, on montre, on décide, puis on fait.
⚠️ **La revue de fin est UNE FOIS PAR PHASE**, pas par étape (opérateur,
2026-08-20). Mandat « designer d'Apple ». Lui donner les **URLs et les parcours**,
pas le diff.

---

## Les 25 défauts — l'état à la clôture

| # | Défaut | Issue |
|---|---|---|
| **J-25** | ✅ **RÉGLÉ le 2026-09-04.** « Accès complet » (#20, les 4 capacités, SANS exemption d'horaires) attribué à l'audience `user`. Mesuré : la portée passe de **3 personnes à 9**, et `machines` de **2 à 9**. ⚠️ S158/S159 avaient construit la route ; **personne n'était dessus** — un modèle complet dont aucune donnée n'emprunte le chemin se lit comme une panne. Commande : `app:j25:open-booking` |
| **J-8** | un champ refusé fait ressaisir le reste — ✅ **CLOS le 2026-09-05**, le dernier écran défaillant (`/profil`, branche profil public) rend désormais la page avec la saisie, prouvé par la sonde. Historique : **le chiffre de 15 était FAUX.** Au 2026-08-23 : **9 écrans prouvés sains** par un POST refusé (`app:s147:form-probe`, 13 sondes), 4 de plus convertis par le même mécanisme mais non sondés un par un. 🔴 Le seul défaut restant prouvé n'était même pas admin |
| **J-9** | trois maquettes S103 en prod, titres en dur, clés brutes à l'écran — ✅ **CADUC, mesuré le 2026-09-04** : `debug:router` ne connaît plus `design/droits-quotas`, `design/workspaces` ni `design/structure` ; le nettoyage de S159 les a supprimées, page, route et lien. Le défaut n'a plus de sujet |
| **J-10** | formulaires les plus lourds — ✅ **CLOS le 2026-09-05** : la moitié « taux d'aide » réglée et reformulée (Phase N), la moitié « écran de contenu » renvoyée à la Phase Q (S181). Historique : **l'éditeur de packages, 28 champs visibles à l'arrivée → 7** (les 4 éditeurs « ajouter » repliés, 2026-08-24). 🔴 **CORRECTION du 2026-09-06 : « `admin-formation-content` (35) » était PÉRIMÉ.** Mesuré ce jour-là : **1 champ visible à l'arrivée**, et c'est la recherche de l'en-tête du site. S149 avait replié les neuf cartes ; le chiffre d'avant a survécu **quatre phases** dans le document |
| **J-23** | `/admin/usage-rights/shadow` — ✅ **CADUC, mesuré le 2026-09-04** : la route n'existe plus, S159 l'a retirée avec le retour arrière devenu un piège. ⚠️ Le réglage `usage_rights_v2_*` reste en base pour une écriture explicite |
| **J-4** | « (s) » au lieu de pluriels ICU — ✅ 2026-08-24, **77 clés** migrées, 5 langues, 0 « (s) » restant. Validateur statique `tools/i18n/icu_audit.py` (395 motifs, 0 faute) |
| **J-5** | CSS local par page — ✅ **CLOS PAR LA MESURE le 2026-09-05** : 544 sélecteurs locaux, **9 dupliqués dont 6 artefacts de comptage**, une seule duplication réelle laissée sciemment (Phase N). Historique : 708 → **653 règles dans 37 gabarits**. Les deux familles à duplication PROUVÉE (kiosque, authentification) ont été rassemblées et ont révélé 2 défauts visibles. Le reste est du CSS réellement spécifique à sa page |
| **J-7** | emoji bruts comme icônes — ✅ 2026-08-24, **0 emoji d'interface** sur 16 pages rendues. Les 33 signes typographiques restants sont une décision écrite en tête de `_icon.html.twig` |
| **J-22** | formulaires admin hors thème — ✅ 2026-08-23, **27 conversions**, 13 écrans, 13 sondes vertes. Restent, écrits : la matrice de fonctionnalités (partial partagé), 5 filtres GET, la semaine d'horaires, les contrôles en boucle des tableaux |
| **J-1** | déploiement partiel : l'upload d'images fatalait en prod — ✅ 2026-08-22 |
| **J-2** | huit objets se supprimaient en dur — ✅ 2026-08-22 |
| **J-3** | flashs en dur — ✅ 37 → **0** |
| **J-6** | `style=""` sur `/admin/utilisateurs/{id}` — ✅ 78 → **1** |
| **J-11** | `/machines/{id}` cassé sur téléphone — ✅ 2026-08-22 |
| **J-12** | barre d'outils du calendrier — ✅ 2026-08-22. ⚠️ **Le constat initial était faux** : 5 contrôles réellement inatteignables, pas ceux annoncés |
| **J-13** | « Réserver une machine » menait au calendrier lecture seule — ✅ 2026-08-22 |
| **J-14** | pas de lien d'évitement, focus invisible — ✅ |
| **J-15** | fonds clairs sans variante sombre — ✅ 101 → **0** |
| **J-16** | `/formations/{id}/suivi` imprimait ID/titre/slug au public — ✅ |
| **J-17** | `/machines/{id}` « Connexion requise » ×4, favoris mort — ✅ |
| **J-18** | `/admin/maintenance/batch` sans lien — ✅ |
| **J-19** | « Loans » ouvrait le catalogue d'objets — ✅ |
| **J-20** | le calendrier ignorait les plages horaires — ✅ 2026-08-22 |
| **J-21** | catégorie d'un grant comparée par libellé exact — ✅ 2026-08-23, l'identifiant décide |
| **J-24** | messages de validation en français en dur — ✅ 69 → **0**, cinq langues complètes |

⚠️ **Rien de cette liste n'a été reporté hors de la Phase J.** Trois défauts se
sont révélés **caducs** (J-9, J-23) ou **périmés dans leur énoncé** (J-10) — pas
parce qu'on les a fermés, mais parce qu'une autre phase avait supprimé leur sujet
et que le plan continuait de les décrire. C'est la leçon d'entretien de la phase :
**un plan qui traîne une mesure périmée envoie travailler là où il n'y a plus rien
à faire.**

## Les critères de sortie, et comment ils sont tombés

- la liste S147 est vide, ou chaque reste est **consciemment reporté et écrit** ;
- **aucun gabarit ne porte de `<style>` local** hors `admin-design`, ou chaque
  exception est une règle nommée du guide ;
- ✅ **gabarits à `<head>` propre : ATTEINT** — ils sont **5** (`event-ticket` +
  4 kiosques), et c'est exactement l'exception que ce critère prévoyait ;
- les dix points passent sur **chaque** écran du socle et des features ;
- `/admin/design` montre chaque primitive utilisée, avec le vrai composant.

---

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

⚠️ La leçon tient en une phrase : **un balayage qui ne compte que les gabarits
n'aurait rien vu** — 40 gabarits étendaient la coquille nue, 39 légitimement.
Ce sont les PAGES qu'il faut compter.

---

## La proposition d'écran « événements », d'après Fabmanager (opérateur, 2026-08-27)

**Source** : trois captures de Fabmanager (instance Technistub) décrites dans
`Stage/Drive/Images/Fabmanager UI/README.md` — événements, formations, machines.
⚠️ **Fabmanager, pas Fabman** : c'est une seconde source, distincte des 73 captures
qui ont donné le barème de qualité de formulaire.

**La demande** : une **page d'exemple** d'une version améliorée de nos événements.
Donc une proposition à regarder, pas un remplacement à déployer — elle passe par
`/admin/design` en propositions comparables, et la revue designer est **une fois
par phase**.

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
même logo, donc quinze cartes identiques. **Le remède devient le symptôme.**

Ce qui a été demandé : enregistrer un OU PLUSIEURS logos de remplacement, affectés
**aléatoirement** aux événements sans affiche ; **six images par défaut** livrées
avec FabOS, un pseudo-logo décliné en variations de couleurs proches du thème ;
une variante claire et une variante sombre.

### ✅ Ce qui a été livré, et les deux décisions qui ont tenu

**La première moitié est faite et regardable : `/admin/design#evenements`.** Les
**six affiches de remplacement** y sont rendues, chacune avec sa géométrie — pas
seulement sa couleur, **parce qu'à luminosité égale six teintes seraient la même
image**. Elles vivent dans `templates/site/_event_placeholder.html.twig`.

- 🔴 **Le tirage est stable** : `id % 6`, jamais `random()`. Un tirage par rendu
  ferait changer l'affiche à chaque rechargement et deux membres ne verraient pas
  la même page.
- ✅ **La question « dark/light » s'ANNULE** : en DESSINANT au lieu de téléverser,
  `var(--color-primary)` et `var(--tone-primary-soft)` suivent le thème du membre.
  Mesuré sur la page rendue — fond `srgb 0.223 0.133 0.223` en sombre,
  `srgb 0.954 0.893 0.920` en clair, **un seul fichier**. Pas douze PNG, pas de
  préférence à lire.
  ⚠️ Le piège qu'on évitait au passage : `--color-text-inverse` vaut `#FFFFFF` et
  **n'est jamais redéfini en sombre** — une image qui s'appuie dessus reste blanche
  sur fond sombre.

✅ **La seconde moitié aussi** — le regroupement par mois est un spécimen dans la
même section, rendu avec les VRAIES classes du catalogue. « AOÛT · 1 événement »
puis « SEPTEMBRE · 4 » : la hauteur des blocs dit le volume avant le compte.
Livré sur les vraies cartes en **S153**.

### Ce qu'on ne copie PAS

- Le pied de carte à deux verbes (« Réserver · Consulter ») vient des écrans
  **formations et machines** de Fabmanager, pas de celui des événements. Il vaut
  d'être discuté pour NOS cartes machine — l'opérateur avait justement signalé
  « je n'ai que le bouton Voir » — mais c'est un autre sujet, à ne pas glisser dans
  celui-ci.
- ⚠️ **Nos cartes disent déjà des choses que la référence ne dit pas** : l'état, le
  prochain créneau et les compteurs. La comparaison des trois captures montre que
  Fabmanager ne les a pas.
- ⚠️ **Où vivraient les logos téléversés** : `public/uploads/<famille>/`, motif de
  `AdminController` ~3282 — et la même question que pour les documents machine :
  supprimer la ligne n'efface pas le fichier.

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
d'interne.

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
`storedName`. L'entité déclare désormais ses noms de colonnes explicitement.

✅ Sonde d'écriture `app:s152:document-probe` verte. ⚠️ Elle ne couvre pas le
téléversement HTTP lui-même (validation de type, déplacement du fichier), qui
demande un vrai POST authentifié.

---

## L'incident de la phase, à connaître

🔴 Pendant S169, le site est resté en **500 quelques minutes** — une signature
incompatible avec une interface Symfony, que `php -l` ne pouvait pas voir.
Rétabli et vérifié. C'est le premier endroit où regarder si un déploiement de
docs ou de code rend une erreur au chargement.

## 🅿️ Ce qui est sorti de la phase sans y entrer

Sélecteur de langue (`app_switch_locale` n'est lié nulle part) · suppression en
masse d'événements · catégories comme entrées de menu. Ce sont des
fonctionnalités, pas de la finition — elles restent dans `ROADMAP.md`.
⚠️ Le tableau de bord « qui doit re-briller » était le seul des quatre qui touchait
J : il est parti dans S148.
