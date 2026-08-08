# FabOS — plan

**Updated 2026-08-05.** ✅ **S84 shipped 2026-08-05 — the last five div grids become tables, and the surface rule finally lands.** `badges · formations · machines · reservations · utilisateurs` were rectangular grids of cells with a header row, a hand-written `grid-template-columns` and a hand-computed `min-width` — tables in a div costume. All five are `_data_table` now. **−2 543 lines, +586.**

🔴 **They were not outside S83's rule for want of a token.** Each redefined `.admin-panel { background: white; box-shadow: var(--shadow) }` in an inline `<style>`, and the shell emits `{% block stylesheets %}` **after** the `admin.css` link — the ground and lift were applied, loaded, then overwritten, on exactly the five pages that looked wrong. 🔴 **And the lift never landed on ANY dark admin panel:** `style.css` forces `.admin-panel`'s dark surface with `!important` at (0,2,2) carrying `0 14px 34px rgba(0,0,0,.28)` against `--shadow-card`'s `0 16px 40px rgba(0,0,0,.38)`. Its other literals are byte-identical to the dark tokens, so only the shadow was visibly wrong — the same drift that hid `#f6f7fb` against `#f6f8fb`. Tokens now; the `!important` stays, because it is there to beat something.

🔴 **The status chip lied on two pages.** `/admin/machines` emitted `class="status-badge disponible"` **hardcoded**, so a machine in `panne` printed its real word inside a green chip; `/admin/utilisateurs` did the same with `active`. Measured on the box after the fix: machines emit `disponible` ×7, `idle` ×3, `maintenance` ×1 where all eleven previously said `disponible`. ⚠️ **Why they had to hardcode it:** `Utilisateur::$statut` is `'actif'` and `Machine::$statut` is `'idle'` — **French values against English class names.** Both spellings are listed in `admin.css` rather than mapped in Twig, and the base chip now carries a neutral fill, so an unknown state is still a chip; before, it rendered with no fill and read as plain text. ⚠️ `.status-badge` had **no base rule anywhere central** — two of the five declared it inline, three inherited a *different* base from `login-register.css`, so the same chip was two sizes depending on which list you were on.

⚠️ Also gone: a complete **modal** on two of them for a dialog not in their markup, and a hand-copied `<footer>` whose Documentation · Support · API · Statut · Mentions légales · Politique · CGU links were **all `href="#"`** while the real routes exist. ⚠️ `_data_table` gained one parameter — a per-column `class` string, because the shell writes the header and the caller writes the cells, so a right-aligned actions column had no way to make its two halves agree.

✅ **And the last two surfaces, same day — the rule is now applied everywhere it was named.** Three private copies of one idea, each broken differently. 🔴 **`.stat-card` in `style.css` was `background: var(--color-text-inverse)` — a TEXT token used as a surface.** It is `#FFFFFF` and deliberately never overridden in dark (inverse text stays white by definition), so every stat card on the site was a white box on a dark page, and **no dark rule anywhere covered `.stat-card`** to catch it. The same `--color-bg` / `--surface-ground` confusion one layer over: the token resolved to the right pixel in light mode, so nothing looked wrong until the theme moved. `index.html.twig` then kept `background: white` in its `page_styles` block — emitted after `style.css`, so the homepage would have stayed white however often the shared rule was fixed. What is left in the page is the **shape** (a compact 160px column tile against the shared wide row with an icon); the ground, fill, border and lift are not different. A **third** copy sat in `machine-historique.html.twig` — not a dark hole, the other failure: `--color-bg-light` is a *recessed* fill, so cards read as inset panels and elevation only arrived on hover. `.ev-card` was the only one already theme-correct and simply had **no elevation at all**; it gets `--shadow-card`, and the ground goes on a new full-bleed `.ev-page` — ⚠️ never on `.ev-wrap`, capped at 1040px, which would paint a stripe down the middle. Same trap `.ml-wrap` set in S83.

⚠️ **Unverified on the homepage:** the `fablab_stats` section is switched **off** in the homepage-section config, so `.stat-card` markup does not render at `/`. The rule is right and proven in the served sheet; the pixels wait for that section being switched on.

⬅️ Judgement calls worth objecting to: the two *"Edition admin active"* header chips are dropped, and `panel_note: 'Données chargées depuis MariaDB'` is now the row count on all five lists.

---

## ✅ S85 — one shape for the admin lists — **VALIDATED AND SHIPPED 2026-08-05 (presentation layer)**

Rendered live at `/admin/design`, which keeps **no private copy** of any rule — the mockup uses the real classes, so it cannot become prettier than reality.

**Where it actually stands — read this before claiming anything about the lists.**

| | |
|---|---|
| Table rules (width, `.is-grow`/`.is-tight`, pinned action, one row verb, `.admin-meter`, hover, stagger) | ✅ **all 25 lists** — they hang off `.admin-table`, so the skeleton does not matter |
| Page skeleton on `_admin_list` | ⬅️ **16 of 25** |
| Tiled header (`.ml-cats` + search) | ⬅️ **0** — needs per-list counts computed in each controller |
| Five-column reduction | ⬅️ **0** — waits on S58 and S41 |

⬅️ **The nine still on another skeleton, and they are two different jobs.** **Four convert mechanically** (single list, `.admin-header`/`.admin-main-content`): `missing-pages` · `access-rfid-logs` · `rfid-readers` · `staff-access-passes`. **Five are not pure lists** and want a decision each about what the shell holds: `emails` (settings form + list) · `homepage` and `opening-hours` (forms) · `booking-policies` (one table per scope) · `event-registrations` (list + cancel-event panel).

🔴 **Two errors made in this session, both worth not repeating.** (1) *"All 27 lists"* was claimed twice before counting: S82's *"15 of 15"* only counted the pages it had scoped, and **eleven more list pages were never in that number** — including a FOURTH skeleton family, `.admin-rfid-page`/`.admin-rfid-layout`/`.admin-rfid-panel`, that nothing had recorded. (2) The row verb was first scoped `.admin-page …`, which those eleven do not carry, so their buttons kept the old look while the sixteen changed. Re-anchored to `.admin-table` — a row verb lives in a table on every skeleton, and edit pages keep their full-size `.btn-action` because they have no `.admin-table`.

🔴 **And the whole rollout was invisible for an hour.** `admin.css` was edited four times and `?v=` was bumped once, early — so every browser kept the pre-S85 file while the markup was provably correct. ⚠️ **`curl`ing the versioned URL does not catch this**: the server serves the new bytes at the old query string, so the check passes while every real browser is on the cached copy. Compare the `?v=` **the page emits** against the file's last edit instead. One key across all 60 templates now: `20260805-s85-lists`.

1. **The admin list header becomes the public catalogue header** — same `.ml-head` / `.ml-cats` / `.ml-filters`, one-click category tiles with counts, always-visible search. No new component. This is most of "constant identity" and costs nothing to write.
2. **Five columns maximum**, everything else to the detail page. The first column becomes an *identity* cell (avatar/icon + name + one quiet line) instead of three columns — `.admin-cell-user` / `.admin-cell-stack` already shipped in S84. Plans: utilisateurs 15→5 · machines 12→5 · formations 11→5 · réservations 7→5 · badges already 5.
3. **The action column is pinned right *and* fixed at 9.5rem.** Pinned answers *"the buttons are after a scroll"* — on `/admin/utilisateurs` that scroll was **1 780 px**. Fixed answers *"buttons at a consistent location"*: a column sized by its content puts "Modifier" and "Voir détail" at two different x positions and the eye re-finds the target on every page. Measured: **152 px at both a 1024 and a 1600 px viewport**, flush to the wrap's right edge.
4. 🔴 **`.admin-table` has no width rule at all** — not in `admin.css`, not in `style.css`. A `<table>` without one shrinks to its content, so all fifteen lists float left of an empty gutter on a wide screen: **660 px of table in a 982 px panel**. ⚠️ `width: 100%` alone is not the fix — the browser splits the surplus evenly and *Statut* ends up as wide as *Membre*. The split is stated: one column takes the slack (`.is-grow`, the identity), the rest shrink to content (`.is-tight`). Measured 1024 → 1600 px: Membre **196 → 473**, the three tight columns **unchanged**, table = wrap, no overflow.

5. **One verb per row, one shape, one word** — the constant rule, with a decidable criterion: `Ouvrir` (arrow) when the row has a detail page, `Modifier` (pencil) when the form *is* the page. **Never both.** *Voir* / *Voir détail* / *Éditer* disappear — three words for two ideas. ⚠️ Today that one column is `.action-btn`, `.btn.btn-secondary.btn-small` **and `.pagination-link`** — a *pagination* button doing edit duty on `/admin/formations`. New shared partial `site/_icon.html.twig` (24×24, `currentColor`, `stroke-2`, same grammar as the sidebar's private macro). ⚠️ **No icon library**: the CSP forbids third-party hosts, `public/assets/` is gitignored so a package would need reinstalling every deploy, and four icons weigh less than one request. ⚠️ Icon **and** word, never icon alone — guessing whether the pencil means edit is a bet, and the column is 9.5rem wide.

✅ **Decided 2026-08-05.** **Tiles replace the filters** — two controls for one filter is the question *"which one wins?"* asked of the user; what remains (période, rôle) goes in the search bar. **Dropped columns go to the detail page, later.** **Grouped queries are in (S41)** — *"prochaine réservation"* depends on it directly, so **S41 comes before that column, not after**.

⚠️ **RECORDED FOR S58 — the data that will be visible nowhere.** Between "the lists drop to five columns" and "the detail pages are unified", these fields have **no surface at all**: `Machine.machineToken`, `granulariteMinutes`, `limiteReservations`, `createdAt`, `updated`, `lastAuthorizationTime`; `Utilisateur.identifiantRfid`, `numeroId`, `verified`, `tempsPresenceTotal`, `theme`, `langue`, `username`; `Formation.duree`, `categorie`, `placesTotales`, `badge`. **S58 owns putting them back** — that is now the concrete acceptance test for the detail-page work, not a vague "unify the pages". *(The operator filed this as S68; S68 is the booking lock-window / no-show session, so it is recorded here against S58, which is the detail pages. Say if a third session was meant.)*

**Deployment order, which falls out of those answers:** width (one line, and it helps the ten already-converted lists immediately) → pinned action → tiled header → **S41** → five columns → **S58**.

⚠️ **The proposed rules deliberately live in `admin-design.html.twig`'s own `<style>`, not in `admin.css`.** A proposal that already applies everywhere is not a proposal. Promoting them is step one.

✅ **S83 shipped 2026-08-04 — the surface system.** The calendar's look (cool ground, cards lifted on a soft wide shadow) lived in a private palette of nineteen `--cal-*` tokens with hardcoded light values — good-looking and unreachable by the theme. Three ideas promoted into `style.css` in both themes: **`--surface-ground`** (what cards sit on — deliberately NOT `--color-bg`), **`--shadow-card`** (16px/40px at .08 — wide and faint reads as elevation; `--shadow` at 4/6 is a button's shadow), and **`--tone-*-soft`** (derived with `color-mix`, so no dark variant needed). `--cal-*` survive as 18 aliases, 0 literals. 🔴 Also fixed: a blanket `background: transparent !important` on `:is(.main-content, …)` at (0,2,2) was eating the calendar's dark ground, which its own rule at (0,2,1) could never beat.

✅ **Applied the same day.** Ground on `.admin-page` and `.ml-page` (a new full-bleed wrapper — `.ml-wrap` is width-capped and would have painted a stripe); lift on `.admin-panel`, `.admin-edit-panel`, `.admin-user-panel`, `.admin-subnav`, `.deck-panel`, `.ml-card`. 🔴 `.admin-page` had already invented the ground, hardcoded as `#f6f7fb` — a shade off the calendar's `#f6f8fb`; two people built the same idea twice and neither could reach the theme. ⚠️ `.ml-card` carries the lift **at rest**, hover moves to `--shadow-lg`: a grid where nothing lifts until you point at it reads as a table with rounded corners. ⚠️ The dark blanket `background: transparent !important` now carries `var(--surface-ground) !important` — the rule that was the obstacle became the mechanism.

⬅️ *(The div grids named here were S84's job — done. See the top of this file.)*

✅ **S82 shipped 2026-08-04 — the admin sidebar joins NavBuilder, and regroups by feature.** The entries were a 240-line array inside the template with its own gating and its own "current" — a second nav model, which is most of why admin and public felt like two products. `nav_admin()` builds it now, sections are **features** (Équipement · Espaces · Réservations · Événements · Prêts · Matériaux · Formations · Badges · Créations · Pages du Lab, then Le lieu and Configuration), and a section disappears with its feature rather than leaving a heading behind. Four render variants → one loop; 391 lines → 85. `RouteAccessChecker` extracted so PHP and Twig ask the firewall the same question. Also: the admin lists' **add button had four different class combinations** (one of them the public button, rendering at twice the size) — now one; and `.filters-bar` was defined inline four times — now once, in `admin.css`.

✅ **The list pages have a shell too** — `site/_admin_list.html.twig`, nine pages on it. There were two skeletons and nothing said which was current: eleven pages used `admin-page` + `admin-layout`, five used four nested wrappers to do the same job. The shell also renders the flashes, which one page had been styling with an inline hardcoded green.

✅ **The admin navigation is two levels now.** Feature grouping made the sidebar honest and also forty rows long. Sections live on the side (one row each, with a count); the pages of the section you are in run across the top of the content. 42 rows → 14. The strip is emitted by `_admin_sidebar` and spans the layout grid, so it reaches all forty admin pages without editing any.

✅ **All fifteen list pages are on the shell** — the last five landed in S82 and their bespoke grids became real tables in S84.

✅ **S81 shipped 2026-08-04 — one menu, everywhere.** Eight admin pages hand-rolled their own header (flat hardcoded links, no NavBuilder, an inert search box); 607 lines of header CSS in five layered "fix" sections, two of them scoped to `body.header-calendar-*` and one of those admin-only, so the bar genuinely differed by page *and* by role. One system now: one breakpoint, `min-width: 0` so a long menu cannot shove the search out, nav scrolls rather than wrapping. ⚠️ `base_public.html.twig` emits `main.js` by default now — 21 of 38 public pages never loaded it, so their burger did nothing.

✅ **S78 is COMPLETE** — steps 1–7 all shipped, deployed and verified (2026-08-04). Standalone admin `<head>`s 46 → 0, hand-rolled tables 25 → 0, form triplets 108 → 19 (the rest deliberate), 221 hex literals → tokens. ⬅️ **Next session: pick from Phase H (S38–S44) — S41's batched query is what `/machines` and `/badges` are waiting on.**  ✅ **S77 and S79 are shipped, deployed and verified**; S78 steps 1–5 and **step 6** likewise (108 form triplets → 19, the rest documented as deliberate). Both are written up in `docs/HISTORY.md`. Only what is **not done** lives here. Shipped sessions, their postmortems and every "why" live in `docs/HISTORY.md` (2 000 lines) — read it *only* when you touch the thing it describes. Cold start: `docs/PROJECT_STATE.md`.

---

## Hard constraints — read before writing any code

| | |
|---|---|
| **Stimulus yes, Turbo no** | AssetMapper is live since 2026-08-01 — `importmap('app')` sits in both shells and in 8 standalone pages. **Turbo is `enabled: false` in `assets/controllers.json` and `window.Turbo` is undefined in prod; navigation is untouched.** Write Stimulus controllers, not inline `<script>`. ⚠️ Turning Turbo *on* is still its own watched session: 19 inline blocks would break (some hang off `DOMContentLoaded`, which never fires on a Turbo visit; others add `document` listeners they never remove). ⚠️ Deploy now needs `importmap:install` + `asset-map:compile` — `assets/vendor/` and `public/assets/` are gitignored, so the tar does not carry them. ⚠️ A controller cannot replace anything that must run **before first paint** — the theme bootstrap in `_header_auth`/`_admin_sidebar` stays inline for that reason. |
| **`strict_variables: true`** | every template read needs `\|default()`. |
| **Timezone** | ✅ **Unified 2026-08-01. The zone is an operator setting** (`timezone`, asked in the install wizard, editable in Site settings; box stays UTC). **Two conventions, and the filter name tells you which one a call site is in.** **A = machine timestamps** (`createdAt`, logs, `Progression`, `CURRENT_TIMESTAMP` defaults) — stored UTC, rendered with **`\|lab_date('…')`**. **B = human-entered wall-clock** (`Reservation`, `Event`, `OpeningHours`, access passes) — stored in lab time, rendered with plain **`\|date('…')`**. ⚠️ **Using the wrong one is silent and shifts by hours**: `\|lab_date` on a booking shows a 10:00 slot as 12:00; plain `\|date` on a log shows a 10:41 scan as 08:41. ⚠️ **Classify by entity, never by field name** — `dateDebut` belongs to both. ⚠️ **Date-only fields stay `\|date`** even when they are convention A: a due date is a calendar day, not an instant. ⚠️ **Never `date_default_timezone_set()`** — tried and reverted the same day: it moves the read, Doctrine's hydration *and* the Symfony form model zone together, so nothing appears to change while stored dates change meaning. Never hardcode a zone in PHP either; take `SiteSettingService::getTimezone()`. |
| **Cache-buster** | bump `?v=` on any stylesheet you touch or nobody who has visited gets it. |
| **prod** | `cache:clear` mandatory on every deploy. No admin loopback bypass. |
| **i18n** | 5 locales, parity checked. One new string = 5 rows. |
| **Migrations** | expand = migration first, contract = code first. Agent sessions **cannot** run them; hand the operator the one-liner. |
| **Vocabulary** | operator-configurable (S31) and per portal. Never hardcode "machine"/"member". |
| **Feature gates** | every surface gates on the registry. Visibility ≠ permission; the firewall decides. |
| **Affordances** | use `can_reach('route')` (S49), never a hand-written role. `ROLE_ADMIN` does **not** imply `ROLE_STAFF`. |
| **No dead affordances** | grep the `disabled` attribute, not "bientôt disponible". Never ship a control that does nothing. |
| **Entity drift** | entities are pre-existingly ahead of migrations. Never `schema:update --force`. |

**Deploy + verify:** Artemis / CT 210 is the operator's review environment. A fix or feature is not finished until its documentation is updated, it is committed, it is deployed there and verified there. Deploy manual tar to CT 210, never `deploy.sh`. Docs deploys are pre-authorised. `git push` does not work from a session. Verify by rendering (`app:render`) and **measuring**. ⚠️ `app:render` + grep proves the markup exists, **never that a member can see it** — for any "can they see or click this" claim, measure geometry in a browser; that gap hid every booking control behind a CSS clip for two sessions. ⚠️ Read the `lint:twig` / `lint:yaml` output **before** `cache:clear` and the restart, not after. Details in `PROJECT_STATE.md` §9.

---

## Open decisions — blocking

1. **`FABOS_RFID_API_TOKEN` is unset and the device check fails open.** Operator must set it. **Blocks S67** — a permission model enforced only in the web flow is not enforced.
2. **Pool booking resolves when?** At booking, or at start? Different calendars. Recommend at-booking + re-assign. Blocks S74.
3. **Package capabilities vs `can_reach()`.** A package-granted capability is invisible to `security.access_map`. Pick: package grants a role / a voter / capabilities stay out. Blocks S67.

**Decided 2026-08-01: AssetMapper on, Stimulus only, Turbo off.** See the constraint row above and S51 in `HISTORY.md`.

---

## Decided 2026-08-01

- **Packages: yes** — and badges stay. Package = *what + when*; badge = *am I qualified*. Both must pass. No price field.
- **Billing, credits, 2FA: no.**
- **Booking lock window + no-show release: yes** (S68).
- **Archive not delete: yes** (S69).
- **Favourites: removed** (S75).
- **Ticket printer: later.** ESC/POS over TCP 9100 is the path. S70 must work mail-only.
- **Categories:** inside the `machines` feature, operator-editable. Bundled defaults deferred.
- **Queue is additive** — it does not replace booking. Both coexist per resource.
- **Kiosk stays public** — `/kiosk/*` publishes who is in the building right now, unauthenticated. Accepted for now; **restricting the kiosk routes as a group is a later job**, not a per-page patch.
- **History pages are scoped to the reader** (S38c): anonymous sees nothing, a member sees their own rows, staff/admin see everything. Rows are filtered, not columns masked.
- **Booking verbs (S77), decided 2026-08-02:** *Terminer maintenant* is its own action and is never locked · *Déplacer* is an atomic swap, never a mutation · the lock window blocks cancel + reschedule only · staff-on-behalf stays in S62.
- **S68 splits, decided 2026-08-02.** ⚠️ **There is 1 RFID reader for 11 machines** (counted, not assumed). The lock window needs no hardware and can ship. The **no-show release is per-resource and off by default**, offered only where a reader exists — applied globally it would release every booking on the 10 machines that have no "showed up" signal at all.
- **Principle:** anything the member can do, staff will not have to.

---

## Pending sessions

### Phase H — hardening (do first)
**S38–S44.** ✅ **S38 shipped 2026-08-01, both sides.** API: 8 leaking endpoints, not the 3 recorded — 5 gated, 3 narrowed. Pages: the three calendars no longer emit a name, a `motif` or even a `user_id` to a viewer who is not entitled to it. **Who is entitled is an operator setting** (`booking_identity_roles`, ticked per role from the `ROLE` table in Site settings), defaulting to staff + admin. ⚠️ **Your own booking always stays identified to you** — `mine` is computed server-side, so hiding identities never hides your own slot from you. S41 = the batched "upcoming reservations grouped by resource" query — **`/machines` and `/badges` both do one query per card today and are waiting on it**. S44 = verify the booking happy path end to end (needs real rows; operator).

### ✅ S77 — cancelling and changing a booking — **shipped, deployed, verified 2026-08-02**

Four verbs (`cancel` / `endNow` / `reschedule` / `restore`) behind `BookingVerbService`, a page per booking at `/reservations/{id}`, and calendar slots linking to it. **Full write-up in `docs/HISTORY.md`.** What it left behind for others:

- **S68** fills `BookingPolicyService::changeDeadlineFor()` — it returns null and does no query today. One line, one place; every verb, note and deadline sentence follows from it.
- **S62/S63** own staff-on-behalf and the audit trail. `VerbContext::Staff` exists and is reachable only via `POST /api/staff/reservations/{id}/cancel`; it **confines** a previously ambient power but is **not audited**.
- ⚠️ **`Event`, `OpeningHours` and access-pass validity still compare a hydrated wall-clock date against a real instant** and are out by the lab's UTC offset, always permissively. `LabClock` exists to fix them; the sweep was not done.
- ⚠️ **`Formation.image` holds icon slugs, not paths** — misnamed column. Nothing reads it as an image any more, but real training photos have nowhere to go.

### ✅ S78 · one code, many uses — **COMPLETE, all 7 steps, shipped and verified 2026-08-04**

| | before | after |
|---|---|---|
| Hand-rolled tables | 25 across 13 class names | **0** |
| Standalone admin `<head>`s | 46 | **0** |
| `form_label` triplets | 108 | **19** (three shapes the theme cannot reproduce — documented) |
| Hand-rolled breadcrumbs | 11 | **0** |
| Hex literals in template `<style>` | — | **221 tokenised**, 412 deliberate |

Full write-up and every ⚠️ worth keeping: **`docs/UI-CONSISTENCY.md`**.

⚠️ **The one method to carry forward:** for any change that rewrites a page's
shell, render every route to a file *before*, again after, and diff the bodies
with CSRF tokens masked. A route sweep says the page still answers 200; it does
not say the page still contains what it used to. That diff is what made a
44-template `<head>` conversion safe to ship in one go.

### ✅ S80 · uploads that fit the page they land on — **shipped, deployed and verified 2026-08-04**

**The problem, measured on the live box:** the two event posters are **5712×4284 PNGs of 23 MB each**. `/events` was pulling **46 MB to fill two 464 px cards** behind a single-threaded `php -S`; the cards stayed blank for seconds and read as broken.

S79 fixed the *display* half — `EventArtwork` caches a 1600 px JPEG in `uploads/events/cache/` and handles EXIF orientation — but the **stored original is untouched**, so 23 MB still sits on disk per poster and still gets decoded (~98 MB in memory) whenever the cache is rebuilt.

**What shipped.** `ImageNormalizer::capUploaded()` rewrites an upload in place before it is stored — upright, capped at **2400 px** long edge, re-encoded. Measured on the real file: **22.7 MB → 0.73 MB**, and 5712×4284 landscape → 1800×2400 portrait, because that file's EXIF orientation is 6. Wired on **all five** image paths: event posters, lab-page photos, creation images (admin and public), avatars. Full write-up in `docs/HISTORY.md`.

⚠️ **It can change the container, so callers build their filename from what it RETURNS.** A PNG with no alpha is a photograph in a format that cannot compress photographs; anything with real transparency keeps PNG.

**One thing left, and it needs you:** the two files already on disk were re-encoded in place (22.7 MB → 6.30 MB each, upright) but **kept as PNG**, because `EVENEMENT.posterFilename` ends in `.png` and correcting that is a DB write an agent session cannot make. To claim the other 5.5 MB each, rename them to `.jpg` and run:

```sql
UPDATE EVENEMENT SET posterFilename = REPLACE(posterFilename, '.png', '.jpg') WHERE posterFilename LIKE '%.png';
```

**Original brief, for the record:**

- Cap the stored original at upload time in `AdminController::eventPoster`. Suggested **2400 px long edge**, re-encoded as JPEG when the source is a photograph.
- ⚠️ **Reuse S79's orientation handling, do not write a second copy.** `EventArtwork::orientation()` / `::upright()` already parse the PNG `eXIf` chunk by hand and bake the rotation into GD's output. The right shape is to lift that pair into something both the uploader and the thumbnailer call — two copies of EXIF logic will drift, and the failure is a sideways picture nobody notices for a month.
- **Sweep the other upload paths for the same fault**: lab-page photos (same shape, same allow-list), avatars, creation images. Each stores whatever the camera produced.
- Decide what happens to the originals already on disk (two event posters today): a one-off console command that re-encodes in place is fine, but it is **destructive** — it needs the operator's word and a copy taken first.

⚠️ **Read the header of `src/Event/EventArtwork.php` before writing any of this.** `getimagesize()` reports how pixels are *stored*; a browser draws them EXIF-corrected; `exif_read_data()` does not read PNG at all. Getting that order wrong is what classified a portrait poster as a landscape banner and cropped it to a letterbox — see S79 in `docs/HISTORY.md`.


⚠️ **Adding `_admin_delete_form.html.twig` to a page requires `importmap('app')`.** `base.html.twig` and `base_public.html.twig` emit it; the 54 standalone-`<head>` templates do not, and there the delete form would **submit without asking**. Checked per caller so far.

### Phase U — remaining
- **S47** booking flow — leftovers: one-click confirm, cancel-with-undo, smart-defaults inventory, `motif` optional per site.
- **S48** disclosure — leftovers: `machine-historique` (pagination), `formation-suivi` (1 199 lines), `profil` restructure.
- **S49** role surfaces — mechanism shipped, **no surfaces built**. Needs a staff-but-not-admin account; none exists.
- **S51** components — **unblocked 2026-08-01** (Stimulus live, Turbo off). Leftovers: inline edit-in-place, drawers, date pickers, toasts-without-reload — all now buildable as controllers, but **without Turbo Frames** the inline edit is fetch-and-swap written by hand (~40 lines, once). First adoption shipped: `clock_controller` replaced the 3 hand-rolled kiosk clocks. **Next earned adoption: the two confirm dialogs** (`_delete_confirm_modal` + `mes-reservations`) — same behaviour, different markup, one controller. Both guard destructive actions, so that one gets its own verification.
- **S52** mobile — calendar one-handed, forms at narrow width. Not verified at presets.
- **S53** retire stylesheets — ~460 KB, 3 overlapping calendar files, ~5 700 lines inline `<style>`.
- **S56** password reset (no backend exists) · **S57** account deletion (erase-vs-anonymise per table, undecided).

### Proposed, unapproved
- **S58** one detail page — **7 unlike `*-detail` templates**. The list shape is done; this is its twin.
- **S59** ✅ **shipped** — see below.
- **S60** sentence-shaped settings + reveal links + consequence lines.
- **S61** 3-click booking, closed hours drawn on the grid.
- **S62** staff surfaces: book-on-behalf, audited override, out-of-service, track-activity, invite/lock/export member.
- **S63** notes + metadata + change log. **S62's override is unsafe without it.**
- **S64** guided post-create chains + onboarding checklist.
- **S65** member area (`/mon-compte`) — the gap S54's nav removal created.
- **S66** holidays & exceptions on opening hours.
- **S70** queue (device **or group**) · **S71** storage bins + pickup. **One workflow; neither ships alone. S70 is probably phase-sized.**
- **S72** member-reported faults → folds into `MaintenanceTask`, not a second inbox.
- **S73** purpose on a usage session.
- **S74** categories you can act on. ⚠️ **`MachineCategory` exists as an entity but `MACHINE_CATEGORY` does not exist in the DB** — starts with a migration. Also: icon belongs to the category, and `Machine` has **no brand/model column** (the make is smuggled into `nom`).
- **S76** access modes: libre-service / sur réservation / sur rendez-vous. **Mode ≠ state**; maintenance is a state. *Sur rendez-vous* needs a staffing rota we don't have — scope separately or drop.

### Approved, unscheduled
**S67** packages — first safe web layer shipped as S97: reusable feature packages, member assignment and opt-in enforcement for reservations / signed-in event registrations. Next scope is resource/category schedules and a loan-service audit; physical-card enforcement remains explicitly deferred. · **S68** lock + no-show (needs RFID reader coverage count; no signal = releases everything) · **S69** archive · **S75** remove favourites (closes the audit's missing-CSRF item — grep routes, not the star).

---

## S59 — the catalogue shape (shipped 2026-08-01)

**Seven public catalogues on two shared templates.** `site/_catalogue.html.twig` (header, category tiles, filters, count, empty state, grid) + `site/_catalogue_card.html.twig` (head, media, body, foot). Stylesheet `public/css/machines-list.css`, tokens only.

On it: `machines · places · prets · materiaux · equipe/formateurs · badges · formations`.

**The rules that keep it working — do not break these:**

- ⚠️ **Neither shell may branch on entity type.** Everything variable is a parameter; anything that can't be a parameter gets a **block**, not an `if`. State vocabulary stays in the caller.
- ⚠️ **Availability and permission are separated by POSITION, not colour.** Head = fact about the *thing* (libre/occupée/complet/hors service/**labo fermé**). Foot = fact about *you* (libre-service/formation requise/obtenu/en cours).
- ⚠️ **"Labo fermé" is checked before "occupée."** A shut venue is not a busy machine — without it every card reads *occupée* on a Saturday.
- ⚠️ **"Libre" means free NOW**, not "a slot exists this fortnight".
- ⚠️ **Never show a bookable time to someone unqualified** — that is a promise `book()` refuses. State word yes, time no.
- ⚠️ **`authorized == false` for anonymous even when nothing is required.** Ask whether the *thing* has a requirement separately from whether *this person* meets it. This shipped wrong once.
- ⚠️ **No category sections.** A group of one is a row of one. Categories are a persistent tile bar.
- ⚠️ **No progress bar** — it would serve 2 pages of 7. Text only.
- Media box keeps one fixed aspect ratio with or without a photo.

**Adding an eighth catalogue:** ~60 lines. Controller builds `cards` + `tiles`; template embeds the shell and decides only the words.

**The eighth arrived 2026-08-02: `/mes-reservations`.** It is not a catalogue of things but a list of *your* bookings, and it fitted with three additions to the **shared** files rather than a copy of them:

- **`_catalogue_card.html.twig` gained an `actions` block** (after the anchor, inside the article) and **`url` may now be empty**. ⚠️ The block is not a convenience: a reservation carries a cancel `<form>`, and a form nested in the card's `<a>` is invalid HTML. It is the only thing a catalogue card could not already express.
- **`_catalogue.html.twig` gained `reveal_after: N`** — first N cards, one "Voir plus", handled by `reveal_controller.js`. A shell feature so the seven other catalogues inherit it instead of each growing its own toggle. ⚠️ Extra cards stay in the DOM (hidden by CSS) so search, print and screen readers reach them, and no-JS degrades to *everything visible*.
- **The four states are tiles, not sections** — same rule as categories.

⚠️ **Tile labels must be handed to the shell already translated.** It prints `tile.label` raw, because every other caller passes a finished word; passing a message key puts `resv.f_current` on screen. It did, once.

**Not verified: dark mode and 320 px.** One stylesheet covers all seven — a single check.

---

## Known gaps recorded, not faked

- `Machine`: no `brand`/`model` — the make is smuggled into `nom`. `Material`: no quantity, so a materials page cannot show stock. `Formation.score` is an `int` of unknown scale.
- The `/proposition/*` prototype was **deleted 2026-08-01** once all seven catalogues were live. ⚠️ It carried the only preview of **S58's detail-page shape** — that is in git history (`b87c9bd`, `templates/site/proposition/_detail_page.html.twig`), not on the site.
