# S78 — one code, many uses

**Written 2026-08-02**, after S77, at the operator's request: *"main goal is consistency and one code, many uses in the UI."*

S59 proved the pattern on the public side — **seven catalogues on two shared templates**, and an eighth (`/mes-reservations`) that fitted in ~60 lines by adding to the shell instead of copying it. This is the same exercise applied to everything S59 did not touch.

Everything below is **counted, not estimated**. The counts are what makes the case; re-run the greps before trusting them if this document has aged.

---

## What is actually wrong — measured

| | count | why it matters |
|---|---|---|
| Templates carrying their own `<!DOCTYPE html>` and `<head>` | **54 of 126** | There is no admin layout. Every stylesheet, every `?v=` cache-buster, every `<meta viewport>` is pasted 54 times. Bumping a cache-buster is a 54-file edit, which is why they have drifted apart already. |
| Templates with an inline `<style>` block | **82** | ~5 700 lines of CSS that no stylesheet knows about (S53's number). |
| Templates containing a literal hex colour | **70** | ⚠️ **Each one is a dark-mode hole.** `machines-list.css` has zero literal colours and that is exactly why dark mode was free there. `admin-place-edit` hardcodes `#f9fafb`, `#e5e7eb`, `#6b7280`, `#111827` — four invisible-on-dark values in one 79-line file. |
| Hand-rolled tables | **25, across 13 different class names** | The linear-list twin of S59's grid. `admin-table` (9), then `admin-user-table`, `admin-rfid-table`, `reg-table`, `reader-table`, `quota-table`, `portal-table`, `pass-table`, `opening-hours-table`, `miss-table`, `mail-table`, `homepage-table`, `dz-type` — one each, every one with its own stylesheet block. |
| Hand-rolled `<nav class="breadcrumb">` | **11** (+1 divergent, +1 that re-declares the CSS inline) | Same markup, same separator, pasted. And the pages that *should* have one and don't are invisible in this count. |
| Admin `new`/`edit` pages repeating the `form_label`/`form_widget`/`form_errors` triplet per field | **~20 pages** | The triplet is Symfony's `form_row()` with extra steps. A form theme replaces all of it. |

**And the thing none of the counts show: the admin side is not translated at all.** `Nom`, `Localisation`, `Actions`, `Voir`, `Modifier`, `Supprimer`, `Aucun espace enregistré.` are literals in the templates. The i18n constraint says five locales with parity; the admin screens are one locale with no parity to check. Every shell below takes translated strings as parameters, so converting a page is also when it gets its five rows.

---

## The four shells

Same rules as S59's, because they are what made S59 hold:

- ⚠️ **No shell may branch on entity type.** Everything variable is a parameter; anything that cannot be a parameter gets a **block**, not an `if`.
- ⚠️ **The caller owns the words**, already translated. The shell prints what it is handed. (S59 learned this the hard way: `_catalogue` prints `tile.label` raw, and passing a message key put `resv.f_current` on screen.)
- ⚠️ **Tokens only, zero literal colours.** That is the whole dark-mode story.

### 1. `site/_data_table.html.twig` — the linear list

The twin of `_catalogue.html.twig`. Serves the 25 admin tables, and any future "list of rows with actions".

```twig
{% embed 'site/_data_table.html.twig' with {
    columns: [{label: 'admin.col_name'|trans}, {label: 'admin.col_place'|trans}, {label: '', actions: true}],
    rows: places,
    empty: 'admin.places_empty'|trans,
} %}
    {% block row %}
        <td>{{ row.nom }}</td>
        <td>{{ row.localisation ?: '—' }}</td>
        {% block row_actions %}…{% endblock %}
    {% endblock %}
{% endembed %}
```

**What the shell buys, beyond the line count:**

- ⚠️ **`colspan` is computed, never written.** Today it is hand-counted in all 25 (`<td colspan="5">`) and it is wrong the moment a column is added — a silently broken empty state nobody looks at, because you only see it when the table is empty.
- **One empty state**, with the same shape as `_catalogue`'s: a sentence that offers the way out, not the word "Aucun".
- A **responsive rule in one place**. A `<table>` is the one element that cannot be made narrow, and S52 (mobile) is still open on all 25.
- **Row actions become one idiom**, so the delete confirmation can stop being `onsubmit="return confirm(…)"` — inline JS, present in most of the 25, and against the standing Stimulus rule while `_delete_confirm_modal.html.twig` already exists and is used by nobody in these files.

### 2. `site/_form_layout.html.twig` + a **form theme**

The form fix is two things, and the theme is the important half.

**`form/admin_theme.html.twig`** — a Symfony form theme overriding `form_row` to emit the existing `.form-field` wrapper. After it, twenty pages stop writing

```twig
<div class="form-field">{{ form_label(form.nom) }}{{ form_widget(form.nom) }}<div class="form-errors">{{ form_errors(form.nom) }}</div></div>
```

per field, and write `{{ form_row(form.nom) }}` — or just `{{ form_widget(form) }}` for the whole thing. Field order and which field is full-width move to the **form type in PHP**, where they belong and where they can be tested.

⚠️ **Applied per-form with `{% form_theme %}`, not globally in `twig.yaml`.** A global theme changes every form on the site — public booking, registration, the wizard — in one commit that cannot be verified without running the app. Per-form is incremental and each page is its own rollback.

**`_form_layout.html.twig`** wraps the page around it: header with title + "back to X" link, the sidebar, the panel, the read-only metadata grid (`ID`, `Créé le` — currently pasted with its own hardcoded greys into every `edit` page), and the cancel/submit footer.

### 3. `site/_breadcrumb.html.twig`

```twig
{% include 'site/_breadcrumb.html.twig' with {trail: [
    {label: 'nav.home'|trans, route: 'app_home'},
    {label: 'nav.machines'|trans, route: 'app_machines'},
    {label: machine.nom},
]} only %}
```

Last entry with no route renders as `aria-current="page"`. ⚠️ `only` matters here: a breadcrumb that can see the page context is a breadcrumb that will eventually branch on it.

**The real deliverable is not the eleven conversions — it is the pages that have no breadcrumb at all.** A shell makes adding one a two-line job, which is the only way the missing ones ever get added. That list has to be built by walking the routes, not by grepping for the ones that already have it.

### 4. `site/_admin_page.html.twig` — the missing layout

The 54 templates with their own `<!DOCTYPE>` are the root cause of the other three problems: with no layout there is nowhere to put a stylesheet link, so each page grew a `<style>`, and each `<style>` grew literal colours.

⚠️ **This is the one with real deploy risk** and it goes last. It touches the `<head>` of every admin screen at once, and the S38b postmortem's lesson applies exactly: *"the pass changed private helper signatures, and their callers are not in the diff you are looking at"* — the 500 on `/leaderboard` was found by sweeping every page, not the touched ones.

---

## The nav, separately — two real bugs, not just inconsistency

`NavBuilder` (250 lines) already centralises *which entries exist*, so the menu is not the copy-paste problem the rest of this document is. What it has instead:

1. 🔴 **No active state anywhere.** No `aria-current`, no `.is-active` on `.navbar-link` — grepped, the only `is-active` in `style.css` is on the language switcher. The menu never tells you which section you are in, on any page, which is most of what "the menu feels inconsistent" means. Fix: compare `app.request.attributes.get('_route')` against the entry route in `_header.html.twig`, set `aria-current="page"`, style it. One file, one CSS rule.

2. 🔴 **The header search is a dead affordance without JS.** `_header.html.twig` emits a bare `<input>` and `<button>` with **no `<form>`**; `main.js` attaches a click handler that sets `window.location.href = '/search?q=…'` — a hardcoded path, not `path('app_search')`. With JS off, or before `main.js` loads, typing and pressing Enter does nothing. Fix: make it a real `<form action="{{ path('app_search') }}" method="get">` and delete the JS. It then works without JS *and* keeps working with it.

3. **Admin navigates by a 391-line `_admin_sidebar.html.twig`, a second nav model** that does not go through `NavBuilder`. Not a bug, but it is why admin and public feel like two products. Worth reconciling *after* the shells land, not before — the sidebar is also the thing every converted page includes, so it should stop moving first.

---

## Order of work, by risk

Each step is independently reviewable and independently revertible. **Nothing below can be verified from an agent session** — no PHP on the machine, and `git push` does not work from here — so each step is sized to be readable in a diff.

| # | step | risk | why this order |
|---|---|---|---|
| 1 | `_breadcrumb.html.twig` + convert the 11 | low | Purely additive, no behaviour, smallest possible proof the shell approach reads well. |
| 2 | Nav active state + header search `<form>` | low | Two isolated bugs with visible fixes. Independent of the shells. |
| 3 | `_data_table.html.twig` + convert 3–5 tables | medium | Proves the shape against real callers before the other 20. ⚠️ Convert a few, then stop and look — S59's shell needed three additions before it fitted the eighth caller. |
| 4 | remaining ~20 tables | medium | Mechanical once step 3 has settled. Each page also gains its five translation rows. |
| 5 | `admin_theme.html.twig` + convert 3 form pages | medium | Per-form, so blast radius is exactly the pages listed in the diff. |
| 6 | remaining form pages | medium | |
| 7 | `_admin_page.html.twig` layout | **high** | 54 `<head>`s at once. Do it when nothing else is in flight, and sweep **every** page afterwards, not the touched ones. |

**Deploy note:** steps 1–6 touch only templates and one CSS file, so the tar carries them. Any stylesheet touched needs its `?v=` bumped or nobody who has visited gets it.

---

## What shipped 2026-08-02 — steps 1–5, **deployed and verified**

### Step 1 — `site/_breadcrumb.html.twig`, all 12 callers converted

Emits the same markup and class names as the eleven originals, so **no stylesheet moved for the conversion itself** and nothing looks different today. What changed underneath:

- 🔴 **A real bug fixed:** `person-booking` built its middle link inside an `{% if %}` and left the separator outside it, so anyone who is neither a trainer nor staff — or whose feature is switched off — saw **"Accueil / / Léa"**. The shell drops falsy entries, so callers pass `null` and the trail closes up. `lab-page-detail` had the same conditional shape and now uses the same idiom.
- **Three duplicate CSS definitions folded into one.** `formation-suivi` and `machine-historique` each re-declared `.breadcrumb` inline *while already loading `details.css`*, which defines it. Both removed.
- ✅ **One of the three had a genuine improvement worth keeping: `flex-wrap: wrap`,** which `machine-historique` needed because its trail is four steps. Promoted into `details.css`, so the **other ten trails now wrap on a narrow screen** — a mobile fix (S52) the rest were missing purely because it had been written in the wrong file.
- `event-detail`'s `style="padding-top:18px"` became `.breadcrumb-inset`. The shell emits one markup shape on purpose; a partial that accepts a style attribute has twelve variants again.
- The last step is never a link and carries `aria-current="page"`; separators are `aria-hidden`, so a three-step trail is no longer read aloud as "slash slash".

### Step 2 — the two nav bugs

- 🔴 **`.navbar-link.active` was already styled, twice (light and dark), and nothing had ever emitted the class.** The menu had no active state on any page of the site. `nav_is_current()` (new, in `NavExtension`) now decides it, and those two existing rules came to life untouched. ⚠️ **Matched by path prefix, not route name** — route equality lights "Machines" on `/machines` and goes dark on `/machines/12`, the page where knowing your section matters *more*. Groups light up when any child is current. Dropdown links had no rule at all; one was added.
- 🔴 **The header search was a dead affordance.** A bare `<input>` and `<button>` with **no `<form>`**, made to work by `main.js` setting `window.location.href = '/search?q=…'` — a hardcoded path ignoring `path()`. It did nothing before that script loaded and nothing at all without JS. Now an ordinary GET form. ⚠️ The JS is **deleted, not left dormant**: it called `button.setAttribute('type', 'button')`, which disarms the submit button — reintroducing that helper breaks the form it was meant to help.

### Step 3 — `site/_data_table.html.twig` + `site/_admin_delete_form.html.twig`

Four tables converted (`admin-places`, `admin-institutions`, `admin-materials`, `admin-events`) as proof of shape before the rest.

- ⚠️ **The caller keeps its own `{% for %}`**, exactly as `_catalogue` hands its `cards` block the whole grid. The tidier shape — the shell looping and calling a per-`row` block — depends on the loop variable reaching a block defined in another template. It probably works; there is **no instance of it anywhere in this repo** to prove it, and under `strict_variables: true` being wrong is a 500 on every admin list at once, on a change no agent session can render-test. Written the proven way and left there.
- **`colspan` is computed from `columns|length`** and can no longer be hand-counted wrong. All 25 originals wrote it by hand, and a wrong one is invisible until the table is empty.
- **`.admin-table-wrap` is emitted by the shell**, so the one element that cannot reflow is scrollable everywhere by construction rather than in 25 separate decisions.
- **The delete form is one partial**, and its confirmation is `confirm_controller.js` instead of `onsubmit="return confirm('Supprimer cet espace ?')"` — inline JS with the sentence buried in an HTML attribute where no catalogue could reach it. It is now a translated value.
- 🔴 **Checked before converting, and it nearly bit:** the Stimulus controller only runs where `importmap('app')` is emitted, and **54 templates carry their own `<head>`**. All four callers extend `base.html.twig`, which emits it — but dropping this partial on a standalone page would delete **without asking**. Recorded at the top of the partial.
- The admin side gained its **first translated strings** (`adm.*`, five locales, parity checked at 897 keys).

### Step 5 — the form theme + the metadata grid

**`form/admin_theme.html.twig`.** Twenty pages wrote the same four-line wrapper once per field — **116 times**. With the theme applied a page writes `{{ form_row(form.nom) }}`. Which field spans the grid moved to `row_attr` on the FormType, where it is a fact about the form rather than something re-decided in both the `new` and the `edit` template that render it. ⚠️ Applied per form with `{% form_theme %}`, never globally.

**`site/_admin_meta_grid.html.twig`** — the "ID / Créé le" strip, converted on **all six** edit pages. Each carried its own copy declaring four literal hex values, so **all six were dark-mode holes**; now tokens in `admin.css`. ⚠️ The partial deliberately does not format dates: `createdAt` needs `|lab_date`, a booking date needs plain `|date`, and a partial that formatted would have to guess.

### 🔴 Fixed on the way: `/admin/settings` was returning 500

Found by sweeping **all 147 routes** rather than the touched ones — exactly the S38b lesson. Pre-existing and **proved so against the pre-deploy rollback archive**: CT 210 carried a hand-deployed `AdminController.php` two lines ahead of the Mac, calling `SiteSettingService::getReservationHistoryMonths()`, which was never written on either side. The full diff between the two files was those two lines, so deploying the Mac's copy fixed it without losing anything container-only.

⚠️ **The general hazard this exposes is worth more than the fix:** CT 210's filesystem can be ahead of the Mac in ways no local grep will show. A route sweep after every deploy is the only thing that catches it.

### Where it stands now — counted after the work

| | before | now |
|---|---|---|
| Hand-rolled tables | 25 | **16** (7 on the shell) |
| `form_label(form.…)` triplets | 116 | **108** (2 pages converted as proof) |
| Pages with hardcoded `.readonly-*` hex | 6 | **0** |
| `onsubmit="return confirm(…)"` | 11 | **7** |
| Hand-rolled breadcrumbs | 11 (+2 divergent) | **0** |

### Step 6 shipped 2026-08-02 (S79's session) — the form pages

**89 of the 108 triplets are gone.** Sixteen admin templates now write
`{{ form_row(form.x) }}`; 105 insertions against 298 deletions. `{% form_theme %}`
is added per page, never globally — the property that made it safe to do the
sixteen in one pass is that a page *without* the tag renders exactly as before,
so an unconverted page is never collateral.

⚠️ **19 triplets are deliberately left, in three shapes the theme does not
reproduce.** Converting any of them would have silently changed the page:

| shape | pages | why it stays |
|---|---|---|
| rows that render `form_help` | `creation-new` (7), `admin-creation-new`/`-edit` (2 each), `admin-rfid-reader-form` (2) | The theme emits label/widget/errors and **no help**. Conversion would drop the hint text — invisibly, since the field still renders. |
| checkbox rows written widget-before-label with class `check` | `admin-utilisateur-new` (2), `admin-event-new`/`-edit` (1 each) | The theme's `checkbox_row` emits `form-field-check`; these use `check`. Different class, and the stylesheet has not been read for it. |
| a widget wrapped in its own div | `_material_form`'s `machines` row (1) | `.material-machines-choices` is load-bearing markup between widget and wrapper. |

Each is a reason to **extend the theme deliberately later** — a `form_help`
block would clear thirteen of the nineteen at once — not a page to force
through it now.

⚠️ **Found while verifying: four of the converted pages had never been rendered
by the route sweep at all.** The sweep probes `{id}` as `2`, and institutions
start at 5, lab-pages at 3. Every `*/2/edit` line in its output was a 404 being
read as "swept". Render edit pages by an id that exists — pull one out of the
list page — or the sweep is quietly checking nothing.

Verified on the box: all 20 form pages 200. `/admin/loans/new` emits 7
`form-field` + 2 `form-field full` + 9 `<label>` + 11 `form-errors`, which is
element-for-element what the hand-written version emitted.

### Still to do — the exact remaining list

**Tables (16).** Already on `.admin-table` and mechanical: `admin-creations`, `admin-loans`, `admin-usage-logs`, `admin-access-rfid-logs`, `admin-utilisateur-detail` (4 tables in that one). Then the 12 bespoke-class tables — ⚠️ **each needs its own stylesheet block read before conversion**, because the class name is load-bearing there and `admin-table` may not be a visual no-op.

**Forms — ✅ step 6 shipped 2026-08-02.** 108 triplets → **19**, all three remaining shapes documented above. The next move on forms is a `form_help` block in the theme, which clears thirteen of the nineteen.

**Step 4, the remaining 16 tables, is untouched.** Listed above.

**Step 7, the missing admin layout, is untouched and still the high-risk one.** 54 templates carry their own `<head>`; 82 have an inline `<style>`; **68 still contain a literal hex colour**. Do it alone, and sweep every route afterwards — ⚠️ and sweep it with **ids that exist**, per the note above.
