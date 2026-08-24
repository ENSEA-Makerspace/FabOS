# Le contrat de forme — comment un formulaire est fait ici

**S149, 2026-08-24.** Ce fichier est **normatif**. Un formulaire qui ne le respecte
pas est un défaut, pas un choix.

⚠️ **Il ne remplace pas `/admin/design`** — le guide montre les primitives, celui-ci
dit comment on les assemble. Le guide est la référence pour *à quoi ça ressemble* ;
ce fichier est la référence pour *ce qu'on demande, dans quel ordre, et à quel
moment*.

Le barème vient des **73 captures de Fabman** (`Stage/Drive/Images/Fabman UI/`),
déjà lues en S58 (`docs/history/phase-U-design-S45-S57.md` § « Les détails à
voler »). On adopte les **formes**, pas les fonctionnalités.

---

## Les sept règles

### 1 · L'écran s'ouvre sur son cas courant

Un formulaire montre ce qu'il faut pour le cas le plus fréquent, et **rien de
plus**. Tout le reste est derrière un repli.

```twig
<details class="settings-danger settings-danger--neutral"{{ refusedEditor|default(null) == 'x' ? ' open' : '' }}>
    <summary>{{ 'x.add'|trans }}</summary>
    <div>…</div>
</details>
```

🔴 **Le repli DOIT se rouvrir sur un refus**, sinon l'opérateur ne voit ni ce qu'il
a tapé ni pourquoi c'est refusé. Le drapeau vient du **contrôleur**, jamais de
`form.vars.submitted` lu dans le gabarit : `prod` n'a pas `strict_variables`, une
variable absente y est silencieusement `null`, et le repli resterait fermé sans que
rien ne le dise.

**Mesure** : `python3 tools/form_quality.py` — colonne « visible ». Un écran de
configuration au-dessus de **12 champs visibles à l'arrivée** est à revoir.

### 2 · Une case à cocher révèle ses propres sous-champs

Décochée, ils ne sont **pas là**. Pas grisés — absents, ou repliés.

### 3 · Le contrôle est une PHRASE avec des champs dedans

> ☐ **Limiter les réservations par membre**
> Max. `[180]` minutes par membre, par équipement et par **jour**.

Et non « Minutes max par jour : `[180]` ». Le libellé au-dessus d'une boîte est le
défaut du thème ; la phrase est ce qu'on vise quand le réglage est une **règle**.

⚠️ Un champ d'identité (un nom, un e-mail) reste étiquette-au-dessus. La phrase est
pour les **règles**, pas pour les données.

### 4 · Une ligne de conséquence, calculée depuis ce qui est tapé

> Fuseau : `(UTC+02:00) Paris` — *Heure locale : 16 h 25*

Le formulaire dit ce qu'il **s'apprête à faire**. Une phrase sous le contrôle,
`.form-help`, calculée par le contrôleur.

### 5 · `required` est un petit mot gris à côté de l'étiquette

Pas d'astérisque. **Rien du tout** sur les champs optionnels — l'absence de mention
est la mention.

### 6 · La carte est ÉTROITE

Fabman tient son plus gros formulaire — « Add member », une vingtaine de champs —
dans **~350 pt** de large, centré, le reste de l'écran vide. Un formulaire ne
s'étire pas à la largeur de l'écran : une ligne de saisie de 1 200 px est plus
difficile à lire, pas plus confortable.

⚠️ Deux champs côte à côte **seulement** quand ils vont ensemble : prénom/nom,
ville/code postal, début/fin. Sinon pleine largeur de la carte.

### 7 · La valeur courante en texte, avec un lien `Changer`

> **Rôle du membre**
> Membre normal — *Changer*

Quand le cas courant se passe de contrôle, on affiche la **valeur**, et le contrôle
n'apparaît qu'à la demande.

---

## Ce qu'on ne fait pas

- ❌ **Pas de septième forme.** Les six shapes sont dans `phase-U-design-S45-S57.md`.
- ❌ **Pas de CSS local** qui ne soit pas devenu une règle du guide (point 1 des dix).
- ❌ **Pas de clé de traduction inventée.** `debug:translation` ne lit pas le PHP des
  `FormType` : une clé inventée s'affiche telle quelle sans qu'aucun outil ne le
  signale. Réutiliser la clé que le gabarit emploie déjà.
- ❌ **Pas de contrainte sans son message dans les cinq catalogues**
  (`translations/validators.*.yaml`).
- ❌ **Jamais un contrôle qui ne fait rien** : un `<button disabled>` est un état, pas
  une action — c'est une pastille `_cell_state`.

---

## La liste de contrôle, avant de dire « fini »

```bash
python3 tools/form_quality.py         # champs visibles à l'arrivée
python3 tools/a11y_static.py          # étiquette atteignable, alt, nom accessible
python3 tools/dead_affordances.py     # href="#", disabled en dur
python3 tools/twig_balance.py         # équilibre des blocs
python3 tools/i18n/parity.py translations templates
python3 tools/i18n/icu_audit.py
```

⚠️ **Aucun de ces outils ne remplace le rendu.** Ils disent qu'un écran demande trop
de choses, pas qu'il est beau. La géométrie, le contraste, le mode sombre et le
focus clavier se mesurent dans un navigateur — voir `S149-REVUE.md`.
