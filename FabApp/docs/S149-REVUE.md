# S149 — revue d'utilisabilité de fin de Phase J

**2026-08-24.** Mandat « designer d'Apple », comme S147 : le nombre de clics avant
et après, l'évidence du chemin, les frappes — surtout en cas d'erreur — et tout
champ demandé sans être indispensable.

---

## 🔴 Ce que cette revue NE peut pas conclure

**Elle ne dit pas que le produit est de première classe, et elle ne peut pas le
dire.** L'accès à CT 210 est tombé en cours de session (le tailnet ne résout plus)
et le site public répond **403 au proxy** depuis ce réseau. Aucune page n'a donc
été rendue ni mesurée.

Ce document est la **moitié statique** d'une revue en deux moitiés. Elle ne
contient que ce qu'une lecture TRANCHE :

| Question | Tranchée par |
|---|---|
| une étiquette est-elle associée à son champ ? | les règles HTML — `for`/`id`, enveloppe, `aria-*` |
| ce lien mène-t-il quelque part ? | la table des routes |
| ce bouton fait-il quelque chose ? | l'attribut `disabled`, sans condition |
| cette page est-elle atteignable ? | les références `path()` sur les 227 routes |
| les cinq langues sont-elles complètes ? | la parité des catalogues |
| ce motif de pluriel tient-il ? | le validateur ICU |

🔴 **Rien de ce qui suit ne remplace la passe navigateur.** La leçon est écrite
dans `feedback-fabos-verify-pixels` et elle a coûté deux sessions : le balisage
correct ne prouve pas qu'on voit quelque chose. Contraste, géométrie, mode sombre,
mobile, focus clavier, nombre de clics réel : **non mesurés**.

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

## 🅿️ La seconde moitié : ce qu'il faut mesurer au retour de l'accès

À faire **dans un navigateur**, pas par `app:render` + grep.

### Les quatre corrections de cette session, à regarder

| URL | Ce qu'on vérifie |
|---|---|
| `/reset-password` | la carte a bien une surface, une ombre, un centrage |
| `/forgot-password` | rien n'a bougé (les 31 règles ont changé de fichier) |
| `/profil/mot-de-passe` | plus de gouttière vide à gauche des trois champs |
| `/kiosk/events`, `/kiosk/machine/{id}`, `/kiosk/entries`, `/kiosk/stats` | les 4 écrans de mur après le passage aux jetons `--k-*` |
| `/register` | la phrase d'acceptation dans les cinq langues |
| `/machines/{id}` (machine indisponible) | la pastille remplace bien le bouton mort |

### Les parcours à chronométrer en clics

1. Anonyme → réserver une machine : `/` → `/machines` → `/machines/{id}` → créneau
2. Membre → s'inscrire à un événement : `/` → `/events` → `/events/{id}` → inscription
3. Membre → retrouver ses réservations et en annuler une
4. Membre verrouillé dehors → `/login` → `/forgot-password` → e-mail → `/reset-password`
5. Opérateur → créer une machine et la rendre réservable
6. Opérateur → ouvrir un accès exceptionnel à un membre

### Les mesures qui ne se lisent pas

- contraste réel des pastilles d'état sur les deux thèmes ;
- 375 px : rien ne déborde, aucun défilement horizontal ;
- focus clavier visible sur **chaque** contrôle du parcours 1 ;
- les quatre écrans kiosque à leur taille réelle (tout est en `vw`/`vh`) ;
- J-25 : un membre non-admin peut-il enfin réserver ? *(décision opérateur, pas du code)*
- **la largeur des cartes de formulaire** : Fabman tient son plus gros formulaire
  dans ~350 pt ; les nôtres s'étirent-ils à la largeur de l'écran ? (mesurer
  `admin-event-new`, `_material_form`, `admin-usage-package-form`)
- **le repli des packages** : les quatre `<details>` s'ouvrent-ils bien, et se
  rouvrent-ils sur un refus ?

---

## Verdict

**Non prononçable en l'état.** La moitié lisible est propre et 16 défauts réels en
sont sortis — dont trois qui touchaient des pages qu'un membre rencontre au pire
moment : l'inscription, le mot de passe oublié, le changement de mot de passe.
La moitié visible n'a pas été regardée une seule fois.
