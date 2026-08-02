# FabOS — plan

**Updated 2026-08-02.** ⬅️ **Next session: S78 — the linear list + the form shell + the nav audit** (see below). S77 shipped in code and is **awaiting deploy and verification**. Only what is **not done**. Shipped sessions, their postmortems and every "why" live in `docs/HISTORY.md` (2 000 lines) — read it *only* when you touch the thing it describes. Cold start: `docs/PROJECT_STATE.md`.

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

**Deploy + verify:** manual tar to CT 210, never `deploy.sh`. Docs deploys are pre-authorised. `git push` does not work from a session. Verify by rendering (`app:render`) and **measuring**, never by eyeballing. Details in `PROJECT_STATE.md` §9.

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

### ✅ S77 — cancelling and changing a booking (shipped 2026-08-02, **not yet deployed or verified on the live site**)

**What shipped.** Four verbs on an existing booking, decided in one place and asked by everyone: `BookingVerbService` answers *may this, and if not why not, in words*; `ReservationService` gained `cancel/endNow/reschedule/restore`; four `POST /api/reservations/{id}/…` routes carry them. The page no longer decides anything — it reads verdicts.

- **`Terminer maintenant`** shrinks `dateFin` to now. Never touches the lock window, sends no mail, no confirmation dialog.
- **`Déplacer`** re-runs the *entire* booking sequence with `ignoreId` set, then applies. Nothing is written unless every rule passes, so a failed move leaves the old slot standing. Needed `?int $ignoreId` on `countActiveUpcomingForUser` + `countForUserStartingBetween` (`hasOverlap` already had one) — without it a booking blocks its own move and anyone on their cap can never move anything.
- **Undo** absorbed from S47: cancel redirects with `?undo=<id>`, re-checked server-side, and `restore` re-validates the whole booking path rather than flipping a status.
- **The refusal sentences are the deliverable**, not a detail — `.ml-why` sits where the button would be, deduplicated, naming either the deadline or the verb that does work. Not a disabled button.
- `BookingPolicyService::changeDeadlineFor()` is the **S68 seam** — returns null, does no query, and is the only line S68 has to change.

🔴 **The card was clipping its own controls — every one of them, since the `actions` block was added.** Reported by the operator as *"I only get the View button"*. `.ml-card` is a grid item with `overflow: hidden` and `.ml-card-link` took `height: 100%`, so the `actions` block — a sibling **after** the link — fell past the card's height and was cut off. Measured: button bottom **892** vs card bottom **849**, 43 px past the clip. So the cancel form that shipped 2026-08-02 was **never visible either**, and S77's three new verbs inherited it. ⚠️ **This is also the real cause of the "affordance is invisible" symptom** the S77 brief blamed on cancel only rendering for future bookings — no amount of reading the Twig would have found it. ⚠️ **And the reason it survived two sessions: verification rendered the page and grepped the markup, which was correct every time. `app:render` + grep proves the HTML exists; it never proves a member can see it.** Fixed: the card is a flex column, the link uses `flex: 1 1 auto`. Equal-height rows and the pinned foot re-measured on `/machines`.

🔴 **Found and fixed on the way: every `booking vs now` comparison was skewed by the lab's UTC offset.** S38b classified the two storage conventions and fixed *display*; it never looked at *comparison*. A `Reservation` is stored as lab wall-clock and hydrated with no zone, so it carries lab digits under UTC — and `$reservation->getDateDebut() <= new \DateTimeImmutable('now', $labZone)` compares a fake instant to a real one. Always permissive: a finished booking sat in *À venir* for two more hours and stayed cancellable after it had started, which is why nobody saw it. New `LabClock` (`now()` / `instantOf()` / `storedFormOf()`) fixes the verbs and the `/mes-reservations` grouping. ⚠️ **The same skew exists wherever `Event`, `OpeningHours` and access-pass validity are compared to `now` — not swept, recorded here.**

⚠️ **Also closed on the way:** the cancel redirect handed a client-supplied `Referer` straight to `redirect()` — an open redirect. It is now reduced to path + query.

**Left for S68/S62:** the lock window itself, and staff acting on someone else's booking (the admin override the endpoint already had is preserved, not widened).

### ⬅️ SUPERSEDED — the S77 brief this was built from

**Why.** `/mes-reservations` was rebuilt on the catalogue shell 2026-08-02 and the page now looks right — which made it obvious that **there is barely anything you can do from it**. Diagnosed before writing this, so the next session starts from facts rather than from the impression:

- ✅ **Cancelling a *future* booking works.** The form is on the card, `POST /api/reservations/{id}/cancel` checks CSRF (`cancel_reservation_<id>`), ownership, and already-cancelled. Verified rendering for a real account.
- 🔴 **A booking that is *running right now* cannot be cancelled** — the endpoint refuses anything not strictly in the future (`RESERVATION_NOT_FUTURE`). "I am here, I am done early, release the machine" has no path.
- 🔴 **Nothing can be edited.** No reschedule, no change of `motif`, no shortening. There is no route and no UI. Today the answer is cancel-and-rebook, which loses the slot to whoever books it first.
- 🟡 **The affordance is invisible when it does not apply.** Cancel only renders on future bookings, so a member whose list is mostly past sees a page with no controls at all and concludes there are none. Whatever ships must say *why* an action is unavailable, per the no-dead-affordances rule — an absent button and an impossible action look identical.

**✅ All four product questions are DECIDED (2026-08-02) — build to these:**

1. **"Terminer maintenant" is its own action**, separate from "Annuler". The booking stays attributed and counts as honoured; only its end moves. ⚠️ **It is never blocked by the lock window** — someone standing at the machine who has finished is doing the lab a favour.
2. **Rescheduling is an atomic swap, not a mutation.** The old slot is held until the new one is confirmed; if the new one is taken, nothing moves. Reuses the existing booking path instead of a second one where the quota, opening-hours, conflict and qualification rules can silently diverge.
3. **The lock window blocks cancel and reschedule, never "terminer".** Matches S68's already-decided sentence ("annuler ou modifier"). Staff keep an override — S62, audited by S63.
4. **Staff acting on someone else's booking stays in S62.** Not in scope here.

⚠️ **What a member sees inside the window is part of the work, not a detail.** Per the no-dead-affordances rule, the control stays visible and explains itself with the deadline named — an absent button and an impossible action look identical, which is exactly how this page came to look like it had no controls at all.

⚠️ **Sequencing:** build S77's verbs so they *ask* `BookingPolicyService`, then let S68 fill the policy in. A window restricting verbs that do not exist yet cannot be tested.

⚠️ **Absorb S47's leftover *cancel-with-undo* here** rather than inventing a third model for the same action.

**Where the code is:** `SiteController::myReservations`, `templates/site/mes-reservations.html.twig` (the cancel dialog is still an inline `<script>` there — folding it into a shared confirm controller is the S51 adoption already queued), `ApiController::cancelReservation` (~line 561), `ReservationService`.

### ⬅️ S78 · one code, many uses — **steps 1–5 of 7 shipped and deployed 2026-08-02**

Full plan, counts and the exact remaining list: **`docs/UI-CONSISTENCY.md`**. Asked for directly by the operator — *"main goal is consistency and one code, many uses in the UI"* — and it is S59's exercise applied to everything S59 did not touch.

**The measured case:** 54 of 126 templates carry their own `<!DOCTYPE>` and `<head>` (there is no admin layout) · 82 have an inline `<style>` · **70 contain a literal hex colour, and each one is a dark-mode hole** · 25 hand-rolled tables across **13 different class names** · 11 hand-rolled breadcrumbs · ~20 form pages repeating the `form_label`/`form_widget`/`form_errors` triplet per field. **And the admin side had no translations at all** — every column header and empty state was a French literal.

**Shipped:** `_breadcrumb.html.twig` (12 callers, 0 hand-rolled left) · nav active state + the header search as a real form · `_data_table.html.twig` + `_admin_delete_form.html.twig` + `confirm_controller.js` (**7 of 25 tables**) · `form/admin_theme.html.twig` (116 field triplets → `form_row`) · `_admin_meta_grid.html.twig` (**all 6 dark-mode holes closed**) · `adm.*` in five locales, 907 keys, parity checked.

🔴 **A live 500 fixed on the way: `/admin/settings`.** Found by sweeping **all 147 routes**, not the touched ones — the S38b lesson. Pre-existing, proved against the pre-deploy rollback archive: **CT 210 carried an `AdminController.php` two lines ahead of the Mac**, calling a `SiteSettingService` method nobody ever wrote. ⚠️ The general hazard matters more than the fix — the container's filesystem can be ahead of the Mac in ways no local grep shows, so **sweep the routes after every deploy**.

**Three bugs found while measuring, all fixed:** `person-booking` rendered *"Accueil / / Léa"* for anyone who was neither trainer nor staff · `.navbar-link.active` was styled twice, light and dark, and **nothing had ever emitted the class**, so the menu had no active state on any page · the header search was a bare input with **no form**, working only via a JS handler with a hardcoded `/search`.

⚠️ **Remaining steps are ordered by risk in the plan doc.** Step 7 (the missing admin layout, 54 `<head>`s at once) is the high-risk one and goes last, alone.

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
