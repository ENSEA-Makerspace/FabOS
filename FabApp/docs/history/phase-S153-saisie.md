# Phase S153 — la saisie, et les propositions qu'on solde

**Ouverte le 2026-08-28, livrée le 2026-08-31.** Quatre chantiers : trois soldent
une proposition déjà dessinée et validée par l'opérateur, le quatrième est neuf.

🔴 **Règle de la phase, donnée par l'opérateur : une proposition implémentée se
SUPPRIME.** Page, route, lien, section. Le raisonnement vit ici et dans le
commit, jamais dans un écran. Une maquette laissée en place devient, six mois
plus tard, une chose qu'on prend pour un composant.

---

## 1. La saisie des packages — le chantier neuf

**Ce que la mesure disait.** Les deux packages existants portent 21 grants, et
les 21 ont lieu, section, ressource et catégorie **vides**. Zéro fenêtre, zéro
quota. Les six dimensions que l'écran faisait remplir n'avaient jamais servi, et
« tout autoriser » avait coûté quatorze créations de grant sur un package dont
`fullAccess` valait **déjà 1** — deux vérités pour un seul fait.

**Ce qui est livré.**

- `PackageSpec` — quatre axes de restriction (*à quoi / quand / où / combien*)
  plus une ligne d'extension (*horaires*), chacun étant une case « aucune
  restriction » **plus** sa liste. Les deux, parce que « toutes les catégories »
  et « les cinq qui existent aujourd'hui » sont la même liste et deux intentions.
- `PackageSpecCompiler` — la traduction vers les cinq tables, et **la
  normalisation** : aucune restriction sur les quatre axes + la cinquième ligne
  cochée ⇒ `fullAccess = 1` et rien d'autre. Pas de ligne de feature, pas de
  grant, pas de fenêtre, pas de quota.
- `decompile()`, et **son refus est la partie importante** : un package que ces
  cinq lignes ne savent pas exprimer — un grant `Manage`, une section, une
  machine précise, deux semaines différentes — n'ouvre pas la carte. La rouvrir
  écraserait, à la première soumission, ce qu'elle n'a pas su afficher.
- La révélation du détail est en **CSS pur** (`:has()`) : le formulaire est
  utilisable sans qu'un contrôleur ait tourné. ⚠️ Et le CSS **cache**, il ne
  désactive rien — c'est la case qui décide, côté serveur, comme le préréglage de
  plage de S149.
- `USAGE_PACKAGE.fullAccess` porte la case « sans limite d'horaires » : aucune
  migration. Son effet est un `if` dans `ReservationService`, à l'endroit où la
  réservation refuse `FABLAB_CLOSED`, et il ne lève que la **grille
  hebdomadaire** — `ScheduleResolver::hasDatedRulesOn()` protège les fermetures
  datées, qui sont les jours où l'opérateur a écrit une ligne exprès.
- La matrice de fonctionnalités et la case « accès complet » sont **parties** de
  la carte d'identité du package : deux surfaces d'écriture pour un même fait,
  c'était exactement les deux vérités trouvées en base.
- Sonde d'écriture `app:s153:package-probe` — écrit, relit, et **réserve le même
  créneau hors horaires sans le package puis avec**, dans une transaction
  annulée.

🔴 **Un changement d'AUTORISATION est passé avec, et il faut le dire.**
`UsageGrantRepository::paths()` ignorait `fullAccess` : un package « accès
complet » sans grants autorisait **tout** sous la v1 et **rien** sous la v2. La
normalisation ci-dessus produit précisément des packages `fullAccess` sans
grants, donc elle aurait été un piège sans cette correction. Conséquence à
mesurer sur la boîte : les porteurs d'un package `fullAccess` gagnent la
réservation hors grille hebdomadaire — ce qui est le cas n°1 de l'opérateur
(« Staff — aucune restriction »), mais qui reste une décision à confirmer
package par package.

**Ce que le formulaire ne dit toujours pas, et pourquoi.** À qui (l'attribution
est une liste sous le package) ; la section et la ressource précise (zéro fois
sur 21 grants) ; la priorité (écartée par l'opérateur, et absente du modèle).

---

## 2. Le regroupement par mois sur `/events`

`{% block cards %}` est **à l'intérieur** de `<div class="ml-grid">`, et `/events`
surcharge déjà ce bloc. Les en-têtes de mois sont donc des éléments de grille en
`grid-column: 1 / -1` (`.ml-month`) : la grille reste **une** grille, donc les
cartes de septembre s'alignent encore sur celles d'août, et le shell partagé par
six autres listes n'est pas touché d'une ligne.

⚠️ Le compte par mois demande une passe préalable — un en-tête annonce
« 4 événements » avant ses quatre cartes, et on ne peut pas compter en avançant.
⚠️ `dateDebut` est nullable : un événement sans date n'ouvre pas de groupe.

---

## 3. Les six affiches de remplacement, posées sur les vraies cartes

`_catalogue_card.html.twig` gagne `placeholder_seed`. Passé, une carte sans photo
dessine `_event_placeholder.html.twig` au lieu de l'icône de catégorie ; absent,
la carte est **pixel-identique** à avant, pour les six autres catalogues.

🔴 Le tirage est **stable** — `id % 6`, jamais `random()` : sinon l'affiche
changerait à chaque rechargement et deux membres ne verraient pas la même page.
⚠️ `placeholder_seed is defined and is not null`, jamais `|default()` : `0` est un
modulo légitime, et `0|default(x)` vaut `x`.
⚠️ Le SVG n'a ni `width` ni `height` : posé nu dans la grille il retombait sur
300×150. Trois déclarations dans `machines-list.css` lui donnent la boîte, et la
même montée au survol que la photo.

🅿️ **Reste dehors** : téléverser ses propres logos. Ça demande une table.

---

## 4. Le tableau de bord — proposition A

La bande respire et porte un **fait**. `_admin_form_head.html.twig` gagne `fact`
et `fact_note` : sans eux, les vingt-six pages de formulaire sont au pixel près
celles d'avant.

⚠️ **Le fait vient du résolveur d'horaires**, jamais d'une chaîne écrite dans le
gabarit — une bande qui affirme « Ouvert » un jour de fermeture est pire que pas
de bande. `AdminController::openState()` parcourt les **intervalles** et non
l'enveloppe : à 12:30 dans un lab qui ferme le midi, l'enveloppe dit ouvert et la
porte est fermée.
⚠️ `.admin-hero-fact` a fallu l'exclure des **deux** couvertures `!important` de
`style.css`, par la même voie que `.admin-hero-note` — sinon gris sur magenta,
donc lisible, donc invisible à la relecture, donc livré.

---

## Ce qui a été RETIRÉ

- `/admin/design/packages`, sa route `app_admin_package_form_vision`, son gabarit
  et son lien depuis le guide ;
- la section `#packages` du guide de style ;
- la section `#evenements` entière — la planche des six et le spécimen de
  regroupement ;
- la section `#tableau-de-bord` entière — la référence, le cadre A et les
  variantes B et C non retenues.
