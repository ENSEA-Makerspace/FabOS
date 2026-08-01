# FabOS — plan

**Updated 2026-08-01.** Only what is **not done**. Shipped sessions, their postmortems and every "why" live in `docs/HISTORY.md` (2 000 lines) — read it *only* when you touch the thing it describes. Cold start: `docs/PROJECT_STATE.md`.

---

## Hard constraints — read before writing any code

| | |
|---|---|
| **Stimulus yes, Turbo no** | AssetMapper is live since 2026-08-01 — `importmap('app')` sits in both shells and in 8 standalone pages. **Turbo is `enabled: false` in `assets/controllers.json` and `window.Turbo` is undefined in prod; navigation is untouched.** Write Stimulus controllers, not inline `<script>`. ⚠️ Turning Turbo *on* is still its own watched session: 19 inline blocks would break (some hang off `DOMContentLoaded`, which never fires on a Turbo visit; others add `document` listeners they never remove). ⚠️ Deploy now needs `importmap:install` + `asset-map:compile` — `assets/vendor/` and `public/assets/` are gitignored, so the tar does not carry them. ⚠️ A controller cannot replace anything that must run **before first paint** — the theme bootstrap in `_header_auth`/`_admin_sidebar` stays inline for that reason. |
| **`strict_variables: true`** | every template read needs `\|default()`. |
| **Timezone** | box is UTC, app is `Europe/Paris`. Pin the zone on the **read** as well as the write. ⚠️ **Do not "fix" this with a global `date_default_timezone_set()`** — tried and reverted 2026-08-01. `DATETIME` columns carry no zone and Doctrine hydrates them in PHP's default zone, so changing it moves the read *and* the hydration: display is unchanged and every stored date silently changes meaning. ⚠️ **The DB mixes two conventions** — `new \DateTimeImmutable()` (no zone) wrote UTC wall-clock, `new \DateTimeImmutable(…, new \DateTimeZone('Europe/Paris'))` wrote Paris wall-clock. No single default is correct for both. Making the zone a setting means auditing which rows are which first; it is a session, not a line. |
| **Cache-buster** | bump `?v=` on any stylesheet you touch or nobody who has visited gets it. |
| **prod** | `cache:clear` mandatory on every deploy. No admin loopback bypass. |
| **i18n** | 5 locales, parity checked. One new string = 5 rows. |
| **Migrations** | expand = migration first, contract = code first. Agent sessions **cannot** run them; hand the operator the one-liner. |
| **Vocabulary** | operator-configurable (S31) and per portal. Never hardcode "machine"/"member". |
| **Feature gates** | every surface gates on the registry. Visibility ≠ permission; the firewall decides. |
| **Affordances** | use `can_reach('route')` (S49), never a hand-written role. `ROLE_ADMIN` does **not** imply `ROLE_STAFF`. |
| **No dead affordances** | grep the `disabled` attribute, not "bientôt disponible". Never ship a control that does nothing. |
| **Entity drift** | entities are pre-existingly ahead of migrations. Never `schema:update --force`. |

**Deploy + verify:** manual tar to CT 210, never `deploy.sh`. Docs deploys are pre-authorised. `git push` does not work from a session. Verify by rendering (`app:render`) and **measuring**, never by eyeballing. Details in `PROJECT_STATE.md` §9.

---

## Open decisions — blocking

1. **Does the calendar show who booked?** `/calendrier` publishes real names + free-text `motif` to anonymous visitors today. Three options: names for nobody, names for signed-in members only, names for staff only. **Blocks the close of S38** — the API side is fixed, the page is not.
2. **`FABOS_RFID_API_TOKEN` is unset and the device check fails open.** Operator must set it. **Blocks S67** — a permission model enforced only in the web flow is not enforced.
3. **Pool booking resolves when?** At booking, or at start? Different calendars. Recommend at-booking + re-assign. Blocks S74.
4. **Package capabilities vs `can_reach()`.** A package-granted capability is invisible to `security.access_map`. Pick: package grants a role / a voter / capabilities stay out. Blocks S67.

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
- **Principle:** anything the member can do, staff will not have to.

---

## Pending sessions

### Phase H — hardening (do first)
**S38–S44.** S38 = badge UIDs on the public API — **the API side shipped 2026-08-01** (8 leaking endpoints, not the 3 recorded; 5 gated, 3 narrowed; verified by anonymous sweep). 🔴 **S38 is not closed:** `/calendrier`, `/calendar` and `/machines/{id}/calendrier` still server-render every booking's real name and free-text `motif` to anonymous visitors. Gating the API while the page publishes the same rows is theatre — but what the calendar shows is a product decision, so it needs an answer, not a patch. S41 = the batched "upcoming reservations grouped by resource" query — **`/machines` and `/badges` both do one query per card today and are waiting on it**. S44 = verify the booking happy path end to end (needs real rows; operator).

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
**S67** packages · **S68** lock + no-show (needs RFID reader coverage count; no signal = releases everything) · **S69** archive · **S75** remove favourites (closes the audit's missing-CSRF item — grep routes, not the star).

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

**Not verified: dark mode and 320 px.** One stylesheet covers all seven — a single check.

---

## Known gaps recorded, not faked

- `Machine`: no `brand`/`model` — the make is smuggled into `nom`. `Material`: no quantity, so a materials page cannot show stock. `Formation.score` is an `int` of unknown scale.
- The `/proposition/*` prototype was **deleted 2026-08-01** once all seven catalogues were live. ⚠️ It carried the only preview of **S58's detail-page shape** — that is in git history (`b87c9bd`, `templates/site/proposition/_detail_page.html.twig`), not on the site.
