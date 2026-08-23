## Phase U — Design system and UI upgrade

*Added 2026-07-31. **Sequencing: S45 and S46 landed 2026-07-31; the rest of Phase U can follow Phase D.** The LMS adds roughly ten new screens, and building them against today's chrome means building them twice. The letter is out of alphabetical order for the same reason as Phase H — read the order from this sentence.*

🔴 **A second block, S58–S66, was added 2026-08-01 from a read of 73 Fabman screenshots — see *Phase U (continued) — read against Fabman* at the end of this phase. It is PROPOSED ONLY and nothing in it is authorised to be built.** It also carries the **feature-gap table** (packages, billing, notes, change log, holidays, overrides…), each row with a recommendation, none of them decided. ⚠️ **Its argument for going before Phase D is the same one S45/S46 won**: S58 and S59 fix the page shapes, and the LMS is about to add ten more pages to whichever pile exists at the time.

**The goal in one sentence.** The app should be simple, modern and pleasant enough that a member books a machine without being taught how, and an operator recognises their own place in it.

### What is already true, so nobody redoes it

S29 gave the **admin** one chrome, one panel model and a checked dark theme. S30 put contextual "edit this" actions on the public detail pages. S31 made the vocabulary the operator's. Portals already carry a **per-portal accent colour** (`portal_primary_color`) and a logo. None of that needs redoing — this phase builds on it.

### What was measured on 2026-07-31, because the numbers set the agenda

| | |
|---|---|
| CSS shipped | **460 KB across 14 stylesheets** — `style.css` alone is 113 KB |
| …of which calendar | **135 KB in four overlapping files**: `calendar.css`, `calendar-fix.css`, `calendar-modern.css`, `calendar-leaderboard.css` |
| Inline `<style>` blocks | **87 templates, 5 782 lines** |
| Public templates extending a shared layout | **0 of 45** — every public page is a standalone HTML document |
| Admin sidebar | **29 entries, 3 generic groups, zero feature gating** |
| Longest public templates | `formation-suivi` 1 199 · `profil` 714 · `machine-historique` 663 · `calendrier` 589 (395 of them inline JS) |

Two of those are the real story. **The public site — the part members actually use — never got the treatment the admin got in S29**: no shared layout, so every page re-declares its own head, its own stylesheet set and its own spacing. And **the admin sidebar ignores the feature model entirely**, so an events-only deployment still sees Machines, Espaces, Matériaux, Prêts and Maintenance in its own navigation, which is precisely the promise Phase A made and this screen breaks.

---

### How this phase works — read before starting any of it

#### The stack is already there and unused

`symfony/ux-turbo`, `symfony/stimulus-bundle` and `symfony/asset-mapper` are **all in `composer.json` today**, and:

- **Turbo is used in exactly 0 templates.**
- The only Stimulus controllers are the two scaffolding defaults, `hello_controller.js` and `csrf_protection_controller.js`.
- Meanwhile `calendrier.html.twig` hand-writes **395 lines of inline JS**.

So the modern interaction layer this phase needs is installed, paid for and idle. **Turbo Frames give inline editing and partial page updates** (S49, S51) with no client-side framework and no duplicated rendering — the server keeps rendering Twig, which is the architecture this app deliberately has. **Stimulus is for the sprinkles**: a date picker, a disclosure toggle, a segmented control.

⚠️ **Do not add a front-end framework.** "Twig server-rendered, no SPA" is a stated architectural decision, not an accident, and it is what makes the feature-gating model work — a route that 404s takes its UI with it. A React island would need the gate reimplemented client-side, which is how "visibility is not permission" gets violated by accident.

⚠️ **Do not write more inline JS either.** Every line added to a `<style>` or `<script>` block in a template is a line the next audit cannot find. The 395 in the calendar were moved into that page's `javascripts` block by S46 and are S47's to rewrite, not a pattern to follow.

#### Anti-goals

- **Not a rebrand.** S31 made the words the operator's and S27 made the accent colour theirs. A redesign that hardcodes new ones takes that back.
- **Not a rewrite.** Every session here should be revertible on its own and leave the app running, exactly like every session before it.
- **Not "modern" at the cost of legible.** A vibrant palette that fails AA is not modern, it is broken — see the measured table in S45.
- **Not a components-for-their-own-sake exercise.** A component earns its place by replacing at least two hand-rolled copies.

#### The verification recipe, once, for all of these

Every session in this phase is visual, and this project already learned how to check visual work without guessing (`PROJECT_STATE.md` §9):

1. `php bin/console app:render <path> --save=…` for each affected page.
2. Serve the saved HTML locally against the **public** stylesheets; drive it with the browser.
3. **Measure, don't eyeball** — computed contrast and geometry in the DOM. S29's whole yield came from printing computed styles; none of it from looking.
4. ⚠️ **Strip `main.js` and the inline theme bootstrap, and pin `data-theme`** — the signed-in user's stored theme is baked into every rendered page, so an unmodified "light" render is dark on a dark machine. A whole audit pass was silently run against one theme twice before this was noticed.
5. ⚠️ **Bump the `?v=` cache-buster** on any stylesheet touched, or the change reaches nobody who has visited before.

#### Order and dependencies

```
S45 tokens ✅ ┬─> S46 public layout ✅ ┬─> S47 booking flow
             │                        ├─> S48 disclosure
             │                        └─> S52 mobile
             ├─> S51 components ──────┴─> S49 role surfaces
             └─> S53 retire stylesheets   (last — it is the clean-up the rest earns)

independent, do any time: S50 admin nav ✅ · S54 dead buttons ✅ · S55 single-feature look ✅
spun out of S54:          S56 password reset · S57 account deletion
```

**S45 and S46 went before Phase D and are done (2026-07-31).** The LMS adds around ten screens; built against the old chrome they would have been built twice. Everything else in Phase U can wait for the LMS to land.

**S54 was the cheapest win in the phase and is done (2026-07-31)** — it came out at −748 lines for +25, and it turned up two features that were advertised but had no backend at all (**S56** password reset, **S57** account deletion). ⚠️ **The "19 buttons" estimate was low by half**: grep for the `disabled` attribute, not for "bientôt disponible" — a third of the dead controls never apologised.

#### A type scale for the "big bold numbers"

The principle is "large, bold numbers for key information", which needs actual sizes or every page invents its own:

| Token | Size / weight | Used for |
|---|---|---|
| `--fs-display` | 48–64px / 800 | the one number a page is about — minutes used, rank, places left |
| `--fs-stat` | 32px / 700 | stat tiles in a row |
| `--fs-h1` | 30px / 800 | page title *(already the admin's size — keep)* |
| `--fs-h2` | 22px / 700 | panel titles *(already in use)* |
| `--fs-body` | 15–16px / 400 | prose |
| `--fs-meta` | 13px / 500 | labels, timestamps, help text |
| `--fs-micro` | 12px / 600 | chips, tags, table headers |

⚠️ **One display number per screen.** If everything is 56px, nothing is. The dashboard's six stat tiles are `--fs-stat`, not six displays.

---

### S45 · Design tokens: one palette, one type scale, one rhythm — ✅ shipped 2026-07-31

**Why.** Everything else in this phase is "make page X nicer", and without a token layer each of those sessions ends by adding a fifteenth stylesheet. This is the session that makes the others cheap.

**Scope.** One token sheet — colour, type scale (including **display sizes for big bold numbers**), spacing, radius, elevation, motion durations — expressed as CSS custom properties layered *over* the existing `--theme-*` variables rather than replacing them. A vibrant accent to move away from the current muted palette.

⚠️ **The accent must stay the operator's, not ours.** `portal_primary_color` already exists per portal and S31 made the words configurable; hardcoding a fashionable brand colour would walk that back. Ship a good *default* and keep the override path.

⚠️ **Contrast is a gate, not a preference.** Brand pink `#9E1B56` is **1.97:1** as text on the dark panel — that is why the admin needed lifted tints. Every token pair goes through the S29 harness (`PROJECT_STATE.md` §9) in both themes before adoption. A vibrant palette that fails AA is not a modern palette.

**Verify.** A swatch page rendering every token pair in both themes with its measured ratio, and one page migrated as proof the tokens are usable.

**✅ What shipped.** The token layer extends the existing `:root` vocabulary rather than starting a parallel one — spacing, radius, shadow and transition were already there, so only the gaps were filled: semantic pairs promoted out of `admin.css` (the public side could not reach them), `--font-size-display: 56px` (the ramp stopped at 40px, a heading rather than a number you can read across a room), weight tokens, and the accent's text form.

⚠️ **The structural fix: `--color-primary-text`.** The dark block overrode fifteen variables and *not* `--color-primary`, which is why every use of the brand colour as text failed on dark and why S29 had to hand-patch classes one at a time. The dark value is **derived, not fixed** — `color-mix(in srgb, var(--color-primary) 50%, white)` — so a portal that sets its own accent gets a legible dark variant for free. 50% was chosen by measurement: it keeps every candidate accent between 5.75:1 and 7.08:1 on `--theme-surface`.

**`/admin/design` is the reference the rest of Phase U checks against.** It reads computed values off `:root` and measures contrast in the browser, so it cannot claim a ratio the stylesheet does not deliver, and it reflects a portal's own accent.

⚠️ **That page shipped wrong twice before it shipped right**, caught by reading its own output rather than trusting it: `color-mix()` computes to `color(srgb 0.81 0.55 0.67)`, three numbers on a 0–1 scale. Reading them as bytes reported the accent at 1.39:1; assuming `srgb` contributed a digit returned NaN. **If a colour token ever reads as absurd, suspect the parser before the palette.**

**Verified** in both themes, all eight text roles ≥ 4.5:1 — dark 5.77→13.65, light 5.02→17.40. Cache-buster bumped on both stylesheets across 81 templates.

#### A validated starting palette, so S45 does not begin by guessing

⚠️ **The finding that shapes the whole token layer: no single accent hex can be text in both themes.** Every candidate was measured against white and against the dark panel `#2b2335`:

| candidate | on white | on dark panel |
|---|---|---|
| `#9E1B56` *(today's brand)* | 7.65 ✅ | **1.97 ❌** |
| `#D6246E` vivid rose | 4.84 ✅ | **3.11 ❌** |
| `#6D28D9` violet | 7.10 ✅ | **2.12 ❌** |
| `#0D9488` teal | **3.74 ❌** | **4.02 ❌** |

Anything saturated enough to be vibrant and dark enough to read on white is too dark to read on `#2b2335`; lighten it until it works there and it fails on white. **So every colour role is a pair — a light-theme value and a dark-theme value — never one hex.** This is exactly the shape `.admin-status-*` already has from S29; the token layer generalises it rather than inventing something new.

**A starting set, all measured, all passing 4.5:1 in their own theme:**

| Role | Light | on white | Dark | on panel | on elevated |
|---|---|---|---|---|---|
| primary (rose — keeps the brand family, more saturated) | `#C2185B` | 5.87 | `#FF8FB8` | 7.08 | 6.30 |
| primary (violet — if a cleaner break is wanted) | `#6D28D9` | 7.10 | `#C4B5FD` | 8.16 | 7.25 |
| accent (cyan — for secondary actions and data viz) | `#0E7490` | 5.36 | `#67E8F9` | 10.39 | 9.24 |
| danger | `#b91c1c` | 6.47 | `#fca5a5` | 7.93 | 7.06 |
| warn | `#b45309` | 5.02 | `#fcd34d` | 10.44 | 9.29 |
| ok | `#15803d` | 5.02 | `#86efac` | 10.72 | 9.54 |
| info | `#1d4ed8` | 6.70 | `#93c5fd` | 8.35 | 7.43 |

The four semantic rows are **already shipped and in use** — S29 measured and deployed them as `.admin-status-*`. Only the primary and accent rows are new, which makes S45 much smaller than it looks.

⚠️ **`#C2185B` doubles as a button fill with white text at 5.87:1**, so one value covers "primary text" and "primary button" in light mode. Do not assume that holds for whatever replaces it — check `white on X` as well as `X on white`.

⚠️ **This is a *default*, not a brand decision.** `portal_primary_color` already lets each portal override the accent, and S28 validates it as a hex on read *and* on save. Whatever S45 picks must flow through that same setting, or the palette work quietly un-does S27.

---

### S46 · One public layout — ✅ **shipped 2026-07-31**

**Why.** **0 of 45 public templates extended `base.html.twig`** (14 lines, doing nothing). Every public page was a standalone document re-declaring its head and stylesheet set. Same disease as the admin's six skeletons, on the side that matters more, and the reason the public pages drifted apart.

**What shipped.** `site/base_public.html.twig`, extended by **all 37 pages that carry the header and footer**. It owns the doctype, the head, the favicon, `style.css` and its cache-bust, and the header/footer includes. Children fill `title`, `head`, `stylesheets`, `page_styles`, `body`, `javascripts`, and set `body_class` / `html_attrs` as **variables** rather than blocks.

The other 8 public templates (kiosk ×4, `event-ticket`, `staff-scan`, `recherche`, `creation-new`) include **neither header nor footer** — a different medium, deliberately out of scope, not pages that were missed.

**How it was verified.** All 37 rendered with `app:render --save` before and after, then compared. Each batch came back identical once five differences were normalised away, and every one of the five is deliberate:

| Difference | Why it is expected |
|---|---|
| head whitespace collapsing | the base's `{%-` controls |
| `<html lang="fr">` → the request locale | **the i18n bug the base fixes as a side effect** — 20 public pages hardcoded `fr` on a site shipping five languages |
| `style.css` gaining `?v=20260731-tokens` | six pages were still asking for the un-busted file |
| CSRF tokens | per-request |
| `noindex` meta moving up inside `<head>` | the two mail-reached pages, now in the `head` block |

**Anything else in that diff is a regression.** Normalise the five, then demand equality — do not read diffs by eye. Two findings came out of exactly that and would not have survived eyeballing:

⚠️ **`profil` server-renders `data-theme-preference` on `<html>`**, and the first pass dropped it silently — the base's `<html>` tag is fixed. Hence **`html_attrs`**, printed raw because it is markup, so **values are escaped at the call site** (`~ user.theme|e('html_attr') ~`). It is a one-page hook, not an attribute bag.

⚠️ **Four pages carried a `BANDEAU D'ALERTE` comment banner above the header include.** On `index.html.twig` there was a real second alert bar behind it (fixed in 9613eb2 — the homepage was emitting two); on `login`, `profil` and `register` only the dead label. Both are the same fossil from when the alert bar was per-page.

⚠️ **The base loads no `main.js`, and `body_class` is a `set` variable.** Both from measuring: main.js is on 29 public pages and absent from the rest, so a default would add a script where none existed; and a `body_class` *block* would leave `class=""` on forty pages that never had one. ⚠️ Two pages interpolate `is_granted` into their body class — that needs a Twig **concatenation**, not a quoted copy of the attribute, or the `{{ }}` renders literally.

⚠️ **A separate base from `base.html.twig` on purpose** — that one is extended by 23 admin templates, and widening it would have meant changing the admin's shell inside a public-side session. **S53 merges them.**

⚠️ **The calendar's 395 lines of inline JS were moved, not rewritten** — they are S47's, and they now sit in that page's `javascripts` block.

**Verify.** ✅ 37 of 37 identical under the five normalisations above. Rendered HTML being unchanged is also what carries "in both themes": the theme is CSS- and `data-theme`-driven, and the one page that server-renders a theme attribute keeps it.

---

### S47 · The booking flow, in as few steps as it can honestly take — 🟡 **partly shipped 2026-08-01**

**Why.** Booking is what the app is *for*, and it is the one flow the plan admits has never been verified end to end. The calendar is a 589-line page with 395 lines of inline JS.

**Scope.** Pick resource → see availability → confirm. Smart defaults throughout: the nearest bookable slot pre-selected, the member's usual duration remembered, and **slots that cannot be booked never offered** rather than offered and refused. Inline explanation at the point of choice — "you need the laser badge for this", "you already hold your two bookings this week" — instead of after submit.

⚠️ **Client-side hints are a convenience; they are never the rule.** The three permission layers (certification, quotas, access passes) meet server-side in `ReservationService::book()` and stay there. Mirroring a quota in JS is a UX improvement and a security no-op.

⚠️ **Quota refusals are 409, not 403** — "this conflicts with what you already hold", not "you are not allowed". The wording must offer the fix (cancel something), because that is what makes it true.

⚠️ **Refusal order is deliberate: min-notice and horizon before slot alignment.** Get it wrong and a member fixes the alignment of a slot they were never allowed to book, then gets refused again.

**Verify.** Count clicks for one real booking before and after. Every refusal branch still reachable and still explained. **This is also the session to finally verify the happy path** (Phase H S44).

#### What shipped 2026-08-01 — and what did not

⚠️ **The click-count baseline below is partly stale, so read it with this.** Both calendars *already* meet the "book a specific slot" target: clicking a slot opens the panel with the date and both times filled from the click (`setBookingSelection` → `syncBookingHiddenFields`), the start/end are hidden inputs driven by two selects rather than four empty fields, and `motif` is already labelled optional on both. Re-measure before "fixing" that row again.

**Shipped: the answer, up front.** `machine-detail` now computes the next slot *this person could actually book* and puts it on the primary action — "Réserver — libre le 03/08 à 08:00" — instead of a bare *Réserver* that opens a grid to be read by eye. New `src/Reservation/NextFreeSlotService.php`.

- **It validates candidates *through* `BookingPolicyService`**, it does not reimplement min-notice, horizon, alignment or quotas. A second copy would drift, and the copy is what members would see. It walks the slot grid and asks the real checker about each, taking the first not refused.
- **It suggests, it never permits.** `ReservationService::book()` still runs every layer; a slot suggested and then taken is still refused there.
- **Anonymous visitors get opening-hours-and-overlap only** — quotas are per person, so with no user the honest claim is "the resource is free then", not "you may book it".
- **No slot is offered on a machine the member is not certified for**, which would be a promise `book()` refuses.

⚠️ **The timezone warning below is real and it fired immediately.** The service pins `Europe/Paris` and produced 08:00; the button advertised **06:00**. Twig's `|date` renders in the app default, which is **UTC**, so a correct value was converted on the way out. **Pin the zone on the read as well as the write.** Caught only by diffing the rendered button against `/api/opening-hours` — the render looked entirely plausible on its own.

**Also shipped 2026-08-01:** `place-detail`'s three empty inputs now open on the next bookable slot, from the same service. ⚠️ Guarded with `|default(null)` — `twig.yaml` sets `strict_variables: true` and `renderPlaceBookingError()` re-renders that template without the variable, so an unguarded reference would have 500'd **every failed booking submit**, which is exactly the path where the member has already made one mistake.

⚠️ **Availability on the `/machines` card face was deliberately NOT built.** There are 11 machines and the list is *not* server-paginated, so a per-card next-free-slot is 11+ extra queries growing linearly with the lab. It needs one batched query for all upcoming reservations grouped by resource, and a session that can measure the result — Phase H's S41 is about exactly this class of query. Doing it naively would trade a UI win for the performance problem the plan already knows about.

**Not done, still S47's:** "book next free" as a true one-click confirm (it links to the calendar, so it is 2 interactions, not 1); availability on the card face in `/machines` (see above); cancel-with-undo; the smart-defaults inventory (`place-detail`'s three empty inputs and the six admin form defaults); making `motif` optional *per site*. **And the happy path is still unverified end to end** — that needs a booking to actually be created, which means real rows in production, so it stays with the operator (Phase H S44).

#### The click-count baseline S47 has to beat

Traced through the templates on 2026-07-31. These are the numbers to measure against afterwards — "it feels faster" is not a result.

**Booking a machine, as a member, from the home page — 3 page loads, 5 interactions, 5 fields:**

1. click **Machines** in the nav → `/machines`
2. click a machine card → `/machines/{id}` (`machine-detail`)
3. click **Réserver** → `/machines/{id}/calendrier` (`machine-calendar`)
4. click a slot → modal opens
5. fill `booking-start` + `booking-start-time` + `booking-end` + `booking-end-time` + `booking-motif`, then submit → `api_reservation_create`

The shared `/calendrier` grid is the same shape with a resource picker (`booking-machine`, `resourceKey`) instead of step 2–3.

**What is wrong with it, specifically:**

- **Three page loads before the member sees a single free slot.** Availability — the only thing they came for — is four clicks deep.
- **The modal asks for a start date, a start time, an end date and an end time as four separate inputs**, all empty (see the defaults inventory above), when the member has just clicked the exact slot they want. The click already contains the answer.
- **`motif` is required-looking on every booking.** For a member booking a printer for an hour, "reason" is a tax on the common case.
- The date/time inputs re-ask for the day the member is already looking at.

**Targets for S47** — pick from these, do not do all of them:

| Flow | Now | Target |
|---|---|---|
| Book the next free slot on a known machine | 5 interactions | **1** — a "book next free: 14:00–15:00" primary action on `machine-detail`, confirmed inline |
| Book a specific slot | 5 + 5 fields | **2** — click the slot, confirm; times come from the slot, `motif` optional and remembered |
| See whether a machine is free at all | 2 page loads | **0** — availability on the card face in `/machines` (see the catalogue) |
| Cancel a booking | list → confirm dialog | **1 + undo** — cancel immediately, offer undo for a few seconds |

⚠️ **Fewer clicks must not mean fewer refusals.** All three permission layers still run server-side on every one of these; a one-click path that skips `ReservationService::book()` is not a shortcut, it is a hole. What one-click removes is *typing*, not *checking*.

⚠️ **`motif` may be load-bearing for some operators** — a shared workshop may genuinely want to know why a machine is held for three hours. Make it optional per site rather than deleting it, and default it to empty rather than to a placeholder that becomes noise in the admin list.

#### The smart-defaults inventory

Every date and time input in the app was checked on 2026-07-31. **Not one of them opens with a useful value.** The three on `place-detail` look pre-filled — `value="{{ submitted.date ?? '' }}"` — but that only echoes back what the member already typed after a failed submit; on first load all three are empty. `EventAdminType`, `LoanAdminType` and `MaintenanceTaskAdminType` set no `'data' =>` default either.

So booking a space today means typing a date and two times from nothing, on a page that already knows the opening hours, the existing bookings and the minimum notice.

| Input | Today | Should open at | The app already knows this from |
|---|---|---|---|
| `place-detail` date / start / end | empty ×3 | next open slot that satisfies min-notice, duration = this member's usual | `OpeningHoursProvider`, `ReservationRepository`, `BookingPolicyService` |
| `person-booking` (2) | empty | the bookable person's next published availability | `UserAvailabilityRepository` |
| `my-availability` (2) | empty | next week, mirroring last week's pattern | the owner's existing rows |
| `staff-access-passes` (2) | empty | today → +7 days, the common case | — |
| `EventAdminType` start/end | empty | next round hour, end = start + last event's median duration | `EventRepository` |
| `LoanAdminType` due date | empty | today + the site's default loan period | `loans` settings |
| `MaintenanceTaskAdminType` due | empty | today + the task's recurrence interval | the task being cloned |
| `admin-reservations`, `admin-usage-logs` filters | 2 of 4 / 2 of 2 filled | ✅ leave alone — these are the pattern to copy |

⚠️ **A default is a suggestion, never a constraint.** The point is that the common case needs no typing, not that the uncommon one becomes impossible. Every one of these stays editable, and the *server* still validates — a pre-filled slot that has since been taken must still be refused by `ReservationService::book()`.

⚠️ **Defaults must respect the quota rules, or they teach the wrong thing.** Offering "now + 10 minutes" on a site with a two-hour minimum notice produces a form that is wrong the instant it loads. Compute the default *through* `BookingPolicyService`, not around it.

⚠️ **Timezone.** The box runs UTC and the booking flow uses `Europe/Paris`. A default computed server-side and rendered into a `datetime-local` will drift by the offset unless the zone is pinned on the write *and* the read — this has bitten before (an access pass tested as valid after expiry).

---

### S48 · Progressive disclosure on the pages that are walls of text — 🟡 **partly shipped 2026-08-01**

**Why.** `formation-suivi` is 1 199 lines, `profil` 714, `machine-historique` 663, `machines` 503. These are not complicated ideas; they are everything shown at once.

**Scope.** Essential-first on each, with the rest behind disclosure — summary card, then "show more". Per page: **profil** → identity + next booking + badges held, history collapsed. ⚠️ Its "tabs" are anchor links (`#info`, `#badges`) and every section renders at once — this is a real restructure, not a CSS change. **machine-detail** → can I book it, when, and what do I need; specs and accepted materials collapsed. **machine-historique** → paginated, filtered by default to the recent window. **formation-suivi** → current step foregrounded, completed steps collapsed. **machines** → card grid with availability as the primary signal.

⚠️ **Never collapse safety information.** Certification requirements, hazards and the reason a booking was refused stay visible by default. Progressive disclosure is for volume, not for consequence.

#### What shipped 2026-08-01

**Native `<details>`, no JavaScript.** ⚠️ **Stimulus cannot be used for this yet: `importmap()` is called in exactly zero templates, so AssetMapper never loads and neither Turbo nor Stimulus runs on any page.** The bundles are installed and inert. Wiring them up is **S51's** job and is not a thing to slip into a disclosure session — Turbo eager-loads and would start intercepting every navigation. Native disclosure also degrades to "always open" and keeps content in the DOM for in-page search and print.

- **`machine-detail`** — technical bookkeeping behind disclosure; certification requirements untouched and open.
- **`profil`** — reservation history collapsed, and **open by default when there are five or fewer**, because collapsing three rows hides nothing and costs a click.

**Still S48's:** `machine-historique` (pagination + a default recent window), `formation-suivi` (current step foregrounded, completed collapsed), `machines` (availability as the primary card signal), and `profil`'s deeper restructure.

---

#### 🔴 Found during S48: machine device tokens were public — **half-fixed, needs the operator**

**`machine.machineToken` was rendered to anonymous visitors** on `machine-detail`, `machine-historique` (twice) and the homepage machine cards. It reads like a label — `printer-01` — and is not one: it is the path segment that addresses the machine on the device API.

```
POST  /api/rfid/machines/{machineToken}/authorization
POST  /api/rfid/machines/{machineToken}/work-sessions
PATCH /api/rfid/machines/{machineToken}/work-sessions/current
```

**And `RfidMachineController::rejectUnauthorizedDevice()` fails open**: when `FABOS_RFID_API_TOKEN` is empty it returns `null` — allowed. **That variable is not set on the live box**, so those endpoints accept anyone, and the public pages were publishing the identifier needed to address them. This is the same finding Phase H recorded as "the RFID device endpoints have no authentication at all", now with the other half of it: the machine identifier was public too.

✅ **Fixed 2026-08-01:** the token is gone from all three public templates (admin screens still show it, which is where it belongs). Verified: zero hits on `/machines/1`, `/machines/1/historique`, `/` and `/machines`.

❌ **Not fixed, and not mine to fix:**
1. **Set `FABOS_RFID_API_TOKEN`** in `/opt/fabos/FabApp/.env.local` and on the devices — it is secret material, so the operator generates it.
2. **Make the check fail closed** (S38/S39). An unset token must refuse, not allow. Until then the endpoints are open to anyone who can guess a machine token, and `printer-01` is a guess.

⚠️ The endpoints were **not** exercised against production to demonstrate this — the reasoning is from the code and the env, deliberately.

---

### S49 · Where each role changes data — 🟡 **partly shipped 2026-08-01**

**Why.** S30 answered "edit this thing" for admins on six detail pages. The unanswered question is the shape of the whole app: a member checking availability and a staff member moving somebody's booking are doing different jobs on the same screen, and today both are sent to the admin panel.

**Scope.** Decide, per surface, who edits what and where — then build the affordances:

| Surface | Member | Staff | Admin |
|---|---|---|---|
| Calendar / a slot | book, cancel **their own** | move, reassign, cancel **anyone's**, inline on the grid | same as staff |
| Machine page | see availability + what they need | mark out of service, add a maintenance note inline | edit the record |
| Event page | register, cancel their place | check-in, manage the roster inline | edit, cancel the event |
| A person | edit their own profile | issue an access pass, see held badges | roles, authorisation |
| Configuration | — | — | admin panel only |

The pattern: **members act on their own records in place; staff act on other people's records in place; admins configure the site in the panel.** Global configuration stays in the panel — that boundary is what keeps the public pages readable.

⚠️ **There is no role hierarchy in this app: `ROLE_ADMIN` does not imply `ROLE_STAFF`.** S30 gated a chip on `ROLE_STAFF` whose route was under `/admin` and it vanished for the only people who could use it. Every affordance's role must mirror its target route's own `access_control` line.

⚠️ **Visibility is never permission.** Every one of these is a shortcut; the firewall decides on each request. Verify both halves — affordance absent for the wrong role *and* the endpoint refusing.

#### What shipped 2026-08-01 — the mechanism, not yet all the surfaces

**Affordances no longer name a role; they ask the route's own rule.** New `can_reach('route', params)` Twig function (`src/Twig/RouteAccessExtension.php`) consults **`security.access_map` — the same `access_control` map the firewall itself uses**. There is no second copy to drift: edit `security.yaml` and every affordance follows. Multiple roles on one line mean "any of these", matching the firewall's affirmative decision.

That is the direct answer to the no-role-hierarchy warning above. Hand-writing a role is guesswork about a rule that lives somewhere else, and S30 already lost a chip to it.

- **`_admin_inline.html.twig`** — callers stop passing `role:`. An explicit `role:` still works, but now as an *extra* restriction on top of the route's rule rather than a replacement.
- **`_admin_sidebar.html.twig`** — entries render only if reachable, on top of S50's feature gate. **This fixed a real bug**: `staff-access-passes` is the one non-admin template that includes the admin sidebar, so a staff member who is not an admin reached it legitimately and was shown **thirty `/admin` entries, every one of which bounces them to `/login`**.

**Verified both directions on the real stack**, by rendering the same pages as two accounts: an admin gets the chips (2 on `/formations/1`, 1 on `/machines/1`, plus 21 inline-edit links) and a plain member gets **zero**. The admin sidebar is unchanged at 30 entries across 6 groups.

⚠️ **This install has no staff-but-not-admin account**, so the exact case the sidebar bug affected could not be reproduced end to end — the fix is verified by the mechanism and by the member/admin split, not by that persona. Worth creating one before trusting the staff surfaces.

⚠️ **`can_reach` answers "may you open this URL", not "may you do this to this record."** Per-row questions — is this *your* booking, is this event yours to cancel — belong to controllers and voters, and must not migrate into it.

**Not done, still S49's:** the surface table above is a design brief, and only its *access plumbing* exists now. Nobody has yet built inline staff editing on the calendar grid, mark-out-of-service on the machine page, roster check-in on the event page, or member edit-in-place on the profile (the last is also what S54 removed a dead button for).

---

### S50 · Admin navigation that follows the feature model — ✅ **shipped 2026-07-31**

**Why.** The sidebar has **29 entries in three generic groups** (Contenu, Réglages, Stats & Live) and **gates none of them by feature**. An events-only portal — the exact deployment Phase A exists to make possible — still sees Machines, Espaces, Matériaux, Prêts and Maintenance in its admin. The public nav solved this in S24; the admin never did.

**Scope.** Gate every entry by its feature, and regroup along the registry's own vocabulary (**resource / activity / directory**, plus kernel) instead of "Contenu". Collapse the rarely-touched settings behind a single entry.

⚠️ **Reuse `NavBuilder`'s rules rather than inventing a second nav system**: a group with no visible children is never rendered, a group's own link follows its children, and visibility is presentation and never permission.

⚠️ **Kernel screens are never gated** — users, roles, auth, settings, portals, mail. An admin who switched everything off must still be able to switch something back on.

**Verify.** Toggle a feature off and the entry disappears from the admin as well as the public nav; with everything off, the sidebar still offers the kernel screens.

#### What actually shipped

30 entries (not 29 — `/admin/design` arrived with S45), each carrying its gate, regrouped exactly as the table below says. **The gating alone would have been a bug**: all four sidebar variants printed `<span class="admin-nav-group-label">` unconditionally, so hiding every child of a group would have left a bare heading over nothing. Emptiness is now checked per group, in all four.

⚠️ **The warning below about `Accès exceptionnels` was stale and is now resolved.** `security.yaml` grants `^/staff` to **`[ROLE_STAFF, ROLE_ADMIN]`**, so an admin can open it; the entry stays in the sidebar. (`NavBuilder` shows it in the *public* header only for staff who are not admins — the two are consistent, not contradictory.)

⚠️ **Verifying this needed more than the live site: all 14 features are switched on there, so nothing exercised a gate.** Rendering `/admin` proved the six groups appear, and no more. The gating itself was tested by running the template's own `|filter` expressions through Twig with `feature_enabled` and `has_calendar_layer` stubbed, over four deployment shapes — everything on, events-only, everything off, spaces-only. Results: events-only drops the whole *Réserver* group with no stray heading; everything-off still lists the kernel; **spaces-only keeps *Réservations*, which is the polymorphic rule the table demands and the one a naive `machines` gate would have broken.** *(An install where a feature is genuinely off would have caught this for free — this one had nothing to offer, so the check had to be built.)*

#### The concrete regrouping S50 should land

All 29 entries as they stand, with the feature that should gate each. **Fourteen of them belong to a switchable feature and none is gated today** — which is what an events-only operator currently sees in their sidebar.

| Proposed group | Entry | Gate by feature |
|---|---|---|
| *(always, no group)* | Tableau de bord · État de l'installation | kernel — never gated |
| **Réserver** *(resource)* | Machines · Espaces · Réservations · Quotas de réservation · Accès exceptionnels | `machines` · `places` · any resource on · any resource on · any resource on |
| **Activités** *(activity)* | Événements · Formations · Badges · Créations · Matériaux · Objets prêtables · Prêts · Maintenance · Pages du Lab | `events` · `formations` · `badges` · `projects` · `materials` · `loans` · `loans` · `maintenance` · `lab_pages` |
| **Annuaires** *(directory)* | *(none today — the staff/trainer directories have no admin screen)* | `staff` · `trainers` |
| **Les gens** *(kernel)* | Utilisateurs · Institutions | never gated |
| **Le lieu** | Horaires · Interface accueil · Portails | kernel · kernel · kernel |
| **Suivi** | Utilisations · Logs RFID · Lecteurs RFID · Pages introuvables | `machines` · `machines` · `machines` · kernel |
| **Configuration** *(collapsed by default)* | Fonctionnalités · Réglages du site · E-mails · Configuration initiale | never gated |

**What this buys.** An events-only portal's sidebar becomes: Tableau de bord · **Activités** (Événements) · Les gens · Le lieu · Configuration — five groups instead of twenty-nine entries, and not one of them mentions a machine. That is the Phase A promise finally reaching the screen the operator spends their time on.

⚠️ **Three traps in this table specifically.**

- **`Réservations`, `Quotas` and `Accès exceptionnels` are gated on "any resource feature", not on `machines`.** Bookings are polymorphic since S8–S10; gating the booking screens on equipment would hide them from a spaces-only or appointments-only deployment that books perfectly well.
- **`Accès exceptionnels` is a `/staff` route living in an admin sidebar.** With no role hierarchy, an admin who is not staff cannot open it — check the `access_control` line before deciding whether it belongs here at all (S49's rule).
- **`Prêts` and `Objets prêtables` share one feature** (`loans`); two entries, one switch. Group them or the operator will look for two toggles.

⚠️ **`Institutions` and `Utilisateurs` are kernel and must never be gated** — an operator who switched everything off still has to be able to manage people and switch something back on.

---

### S51 · Feedback, motion and the components to build it from — 🟡 **partly shipped 2026-08-01**

**Why.** Actions today mostly succeed by reloading the page. Nothing acknowledges a click.

**Scope.** A small component set, used everywhere rather than reinvented per page:

- **Cards** as the default container for a thing (machine, event, booking, member).
- **Toasts** replacing full-page flash reloads for anything that does not change the page.
- **Skeleton loaders** for the calendar and lists, instead of a blank gap.
- **Inline edit-in-place** for staff (see S49) — click the value, not a link to a form.
- **Segmented controls and toggles** replacing single-choice selects.
- **Date/time pickers** replacing the paired text inputs in the booking and event forms.
- **Sheets/drawers** for detail-on-demand instead of a page navigation.
- **A floating action button** on mobile for the one primary action of each screen (book, register, add).
- **Stat tiles** with large bold numbers for the dashboard, leaderboard and profile.

⚠️ **Respect `prefers-reduced-motion`** — every animation needs a no-motion path.

⚠️ **Optimistic feedback must not claim success the server has not given.** The booking chokepoint can still refuse; a green tick that gets retracted is worse than a half-second wait.

#### What shipped 2026-08-01

`public/css/components.css`, loaded once from the public layout, built only from S45 tokens. **Two components, because two are used:** toasts (the server-rendered flashes on `/` and `/profil`) and a mobile FAB (the machine page's primary action).

⚠️ **Cards, skeleton loaders, segmented controls and stat tiles were written and then deleted before commit.** Nothing referenced them, and shipping CSS nothing uses is the dead weight this phase keeps removing. They go in when a surface adopts them — not before.

#### The front-end decision — settled 2026-08-01: **AssetMapper on, Stimulus only, Turbo off**

The decision was recorded as a binary ("AssetMapper on or off"). It is not one — there are **three** levers, and only one is site-wide:

- **AssetMapper** — where the JS lives and how it is built. Local.
- **Stimulus** — behaviour attached to markup by `data-controller`. Local, opt-in per element.
- **Turbo Drive** — intercepts every link and form. **Site-wide, and it switches itself on**: `assets/controllers.json` shipped `turbo-core` as `enabled: true, fetch: eager`, so the first call to `importmap()` would have started Turbo without a line of Turbo being written. That is the trap the original framing hid.

**What shipped.** `importmap('app')` in **10 templates** — both shells (`base.html.twig` 23 pages, `site/base_public.html.twig` 37) and the 8 standalone pages that carry inline JS. `turbo-core` set to `enabled: false`.

⚠️ **The "one admin shell" assumption was wrong.** `base.html.twig` covers only 23 templates (14 admin + 9 static); **53 admin/kiosk templates are standalone documents with their own `<head>`**, and *none* of the admin pages carrying real inline JS extend it. Scoping this to "the admin shell" would have reached 23 pages with nothing to gain and missed every page that needed it.

**Three scaffolding files deleted, and they were not inert:**
- `assets/styles/app.css` was `body { background-color: skyblue }`, imported by `app.js`. Calling `importmap()` with it in place would have turned **60 pages sky blue**.
- `csrf_protection_controller.js` is not a Stimulus controller at all — it is a global `submit` listener that rewrites `_csrf_token` fields for Turbo's stateless CSRF manager. Left in place it would have been live on every page.
- ⚠️ **A tarball cannot delete.** All three still existed on CT 210 after extraction and needed an explicit `rm`; skipping it would have shipped exactly what deleting them locally prevented.

**First adoption — `clock_controller.js`**, replacing the 3 hand-rolled kiosk clocks (`kiosk-entries` and `kiosk-stats` were identical to the character; `kiosk-events` was the two-element variant). −26 lines of inline JS, and it clears its `setInterval` on `disconnect()`, which none of the three did.

⚠️ **Found while replacing them: the kiosk clock rendered UTC.** `{{ 'now'|date('H:i') }}` with no zone, so the wall display showed **two hours early** for the first 10 seconds after every reload — and permanently if JS failed. The controller now pins `Europe/Paris` rather than trusting the Pi's own timezone. **This is systemic: 142 `|date()` filters across `templates/` pin no zone, against 9 that do.** Logged as its own task; only the two clocks rewritten here were fixed.

⚠️ **What Stimulus cannot take over: anything that must run before first paint.** `_header_auth` and `_admin_sidebar` carry byte-identical theme-bootstrap blocks — a textbook duplication, and exactly the wrong thing to convert. A Stimulus module is deferred, so it would run *after* first paint and trade a duplication for a flash of the wrong theme. They stay inline.

**Verified in prod, measured not eyeballed:** the importmap tag and 6 modulepreloads are emitted; every module returns 200; `window.Turbo` is `undefined`; `main.js` still works alongside it and the theme still applies; no console errors. The decisive check: the clock element was overwritten with a sentinel string and **Stimulus rewrote it 10 s later** — proof the controller is connected and ticking, not merely downloaded.

**Left deliberately:** `@hotwired/turbo` stays declared in `importmap.php`. Nothing imports it, so it is never preloaded or fetched — it costs one unused file in `public/assets/` and is the ready-made path the day Turbo is turned on.

🟡 **What the rest of S51 costs now.** Drawers and segmented controls need no JS at all (`<dialog>`, radios). Date pickers are native or a small controller. **Inline edit-in-place and toasts-without-reload were the two items that wanted Turbo Frames/Streams** — without them each is roughly 30–40 lines of fetch-and-swap, written once. That is the price of keeping navigation untouched, and it was paid knowingly.

Two rules encoded rather than assumed:
- **Success toasts fade after 6s; errors and warnings never do.** An error that removes itself is one the reader can miss, and it exists precisely because something did not happen.
- **`prefers-reduced-motion` gets no motion, not less** — and content stays visible, so the success toast that would have faded simply stays.

---

### S52 · Mobile and touch — 🟡 **partly shipped 2026-08-01**

**Why.** The half-wired mobile nav is a long-standing debt, and the calendar — the primary surface — has never been designed for a phone, which is where somebody standing next to a machine will actually book it.

**Scope.** Finish the mobile nav; make the calendar usable one-handed; audit touch target sizes; check every form on a narrow viewport.

**Verify.** The S29 harness at mobile and tablet presets, both themes.

#### What shipped 2026-08-01

**Measured first.** The burger was ~40×32 and nav links ~35px tall — both under the 44px minimum, on the surface someone uses while standing next to a machine.

- 44×44 minimum for the burger, nav and dropdown links, profile/detail/formation tabs, buttons, and inline admin actions.
- ⚠️ **Gated on `(pointer: coarse)`, not on width.** A narrow desktop window is still a mouse and a large tablet is still a finger; a width query would have changed desktop layouts for no reason.
- The mobile toggle flipped a class and **announced nothing** — a screen reader got the same thing open or shut. It now carries `aria-controls` and a kept-in-sync `aria-expanded`, and **Escape** closes it and returns focus to the button instead of stranding focus in a menu that has gone.
- Nav dropdowns already had `:focus-within` beside `:hover`, so touch and keyboard opening were fine — **checked before changing anything.**

**Still S52's:** the calendar one-handed (the actual headline), and every form on a narrow viewport. ⚠️ **Neither was verified at the mobile/tablet presets** — that needs the S29 browser harness, which this session did not run.

---

### S53 · Retire the fourteen stylesheets — 🟡 **barely started 2026-08-01**

**Why.** **460 KB of CSS, 135 KB of it four overlapping calendar files** (`calendar.css`, `calendar-fix.css`, `calendar-modern.css`, `calendar-leaderboard.css`), plus 5 782 lines of inline `<style>` across 87 templates. A file called `-fix` next to the file it fixes is a description of the problem.

**Scope.** Fold everything into the token system and the component set from S45/S51; delete what the migration makes dead; move what remains page-specific into the components it belongs to.

⚠️ **Last, not first.** This is the clean-up that the rest of the phase earns; done before them it is just churn.

⚠️ **The cache-buster is load-bearing.** Every stylesheet is referenced with a `?v=` string; changing a file without bumping it ships nothing to anyone who has visited before. S29 had to bump all 58 admin templates for exactly this reason.

**Verify.** Total CSS materially smaller, every page still renders identically, and the class audit finds nothing newly undefined.

#### What shipped 2026-08-01 — the provably-dead subset only

- **`calendar.css` deleted: 28 KB referenced by zero templates**, and imported by nothing (checked `templates/`, `src/`, `public/`, `assets/`, `config/`, and `@import` across the other sheets). The three calendar sheets that *are* loaded are untouched.
- **`index.html.twig` and `search.html.twig` each carried the same `<style>` block twice, byte for byte** — once in `page_styles`, again inside `javascripts`, left over from S46. Second copy removed after asserting the two were identical.

⚠️ **Deleting a file from the repo does not remove it from the box** — the deploy tarball can only add. It needs an explicit `rm` on CT 210 in the same step, which is what happened here.

**S53 remains almost entirely undone**, and deliberately: ~460 KB across thirteen sheets and ~5 700 lines of inline `<style>` in 87 templates. Folding the three overlapping calendar files together is the real work and needs the pages *driven in a browser*, not grepped — which is exactly why this session stopped at what could be proven dead by measurement.

---

### S54 · Remove the buttons that do nothing — ✅ **shipped 2026-07-31**

**Why.** The app currently advertises features it does not have. **19 "bientôt disponible" affordances across six templates** — five on `formation-detail`, ten on `admin-badges`, one each on `profil`, `register`, `forgot-password` and `admin-reservations` — plus a `disabled` *Edit* button sitting next to the profile's personal-details card. Nothing in this phase does more damage to how finished the app feels: a greyed-out button is a promise the software breaks every time somebody hovers it.

**Scope.** For each one, exactly one of three outcomes, decided per case and written down: **build it** (if it is a session's worth of work, it becomes a session), **remove it** (if nobody has missed it), or **replace it with a sentence** saying what to do instead. No fourth option, and specifically not "leave it, it explains a roadmap" — the roadmap is not on the page.

⚠️ **A disabled control is exempt from contrast rules, which is why these hide from audits.** WCAG 1.4.3 does not apply to inactive components, so S29's contrast harness passed straight over the *Gestion bientôt disponible* buttons on `/admin/badges`. Dead affordances have to be found by grep, not by measurement.

**Verify.** `grep -r "bientôt disponible" templates/` returns nothing, and every previously-disabled control either works or is gone.

#### What actually shipped

⚠️ **The count was wrong: it was 19 by one grep, but 40 static `disabled` attributes across 16 templates once you look for the attribute rather than the phrase.** The phrase-grep misses every dead control that never said "bientôt" — the ten filter and bulk-select widgets on `/admin/badges`, the three reservation-history filter tabs on `/profil`, the two "coming soon" tabs on `formation-suivi`. **Grep for `disabled`, not for the apology.**

Net: **−748 lines, +25.** Every decision, by page:

| Page | Dead affordance | Outcome |
|---|---|---|
| `admin-badges` | 2 bulk buttons, a **duplicate dead filter bar sitting directly under a working one**, select-all + per-row checkboxes, and a **69-line mock "create badge" modal that nothing ever opened** (`openAddBadgeModal` had no callers) | **removed** — 302 lines. `app_admin_badge_new` / `app_admin_badge_edit` and the `q` filter already worked; the page was a mockup layered on top of a working page |
| `admin-reservations` | Export button, "valider/annuler la sélection", checkbox column | **removed** — the JS behind them was three `alert()` stubs, and no CSV/export pattern exists anywhere in the admin |
| `profil` | `disabled` Edit, 3 reservation filter tabs, a one-option `disabled` `<select>`, 2FA, active sessions, delete account | **removed**; the `<select>` became the static label it always was. Edit-in-place is **S49's**, already scoped there |
| `formation-detail` | Enrol, add-to-calendar, per-session enrol, 3 share buttons | **removed** — "Voir ma progression" was already there and works, so it became the primary action |
| `formation-suivi` | Discussions + Notes tabs | **removed** |
| `formation-suivi` | `no_question` ×2, `practical_to_configure` | **replaced** — these were never "coming soon", they were **real states rendered as disabled buttons**. Now the note/badge spans their siblings already used |
| `login`, `register` | CAS buttons, `studentId` field | **removed** — no CAS config and no `studentId` exist anywhere in `src/`; pure mockup residue |
| `forgot-password` | Submit button, under a subtitle promising a reset email | **replaced with a sentence** → **S56** |

Also deleted because nothing referenced them any more: ~92 lines of `login-register.css` (`.btn-cas`, the divider group, `.field-disabled`), 24 lines of dead `.quick-action-card.disabled` rules, and **19 orphaned translation keys × 5 locales**. Locale key parity re-verified at 627 keys each, and every `|trans` key in every template resolves.

⚠️ **Removing a cell from a CSS-grid row means fixing `grid-template-columns` too** — both list tables declared a `40px`/`42px` checkbox column that outlived its checkboxes, which would have left every row indented by a phantom column.

⚠️ **Two of the CSS rules deleted were selector *lists* mixing dead and live selectors** — `html[data-theme="dark"] .field-note, … .divider-text` and a `.login-divider::before, … , .checkmark` group. Deleting the whole rule would have broken `.checkmark` in dark mode; blindly deleting the two dead lines from the first would have left a **bare `html[data-theme="dark"] {}` selector recolouring the entire page**. Prune selectors, don't delete rules.

**Found in passing, fixed:** `kiosk-events` asked for `kiosk.no_events`, which does not exist — a wall-mounted kiosk with no upcoming events was rendering the raw key string. `kiosk.events_empty` already existed with exactly the right text; the template referenced the wrong key.

#### Landed in the same commit, but not S54: *Mes réservations* and *Mes disponibilités* left the main nav

**Operator decision, 2026-07-31.** Both were top-level `header()` entries in `NavBuilder`. They are personal pages — one person's own bookings, one person's own availability — sitting in a menu whose every other entry is a thing the *site* has: the machines, the rooms, the training catalogue, the events. They are now reached from `/profil` only, where the reservations section had **already linked to both** long before this change; the destination needed no work.

Also removed from `footer()`, which the class documents as "a flat list of the same entries" — leaving it there would have made that sentence false.

⚠️ **This overrides a deliberate earlier judgement, and the comment explaining it is gone with it.** The old code gave *Mes disponibilités* a nav link specifically for people with `isBookable()`, reasoning they "manage their slots and answer requests often enough to deserve" one. If a bookable person later complains the page got harder to reach, that is this change, and the fix is a link in the account area — not putting a personal page back in the site menu.

### S56 · Password reset, end to end

**Why.** S54 found `/forgot-password` shipping a form whose submit button was `disabled`, under a subtitle promising "nous allons vous envoyer un lien de réinitialisation par email". The controller has a **`GET`-only route and no handler at all** — there is no reset feature, and never was. The page now says to ask an administrator, which is true but is a person doing a computer's job.

**Scope.** A reset-token entity (hash, expiry, single use), a POST handler that always answers the same whether or not the address exists, the mail itself, and the set-new-password form. The mail worker and `getPublicBaseUrl()` from S31 are already in place, so the link in the inbox will resolve.

⚠️ **This is a new ORM entity, so the migration ships and runs *before* the code** — the expand rule.

⚠️ **Do not reuse the `q`-style GET flow for the token.** A reset token in a query string lands in server logs, browser history and any `Referer` header the next page sends.

**Verify.** A reset mail arrives, its link works exactly once, a second use is refused, and an expired token is refused. Requesting a reset for an address that does not exist returns the same page and the same timing as one that does.

---

### S57 · Account deletion someone can actually reach

**Why.** S54 removed a `disabled` *Supprimer mon compte* button from the profile's danger zone — the affordance was dead, so it had to go, but the underlying right is real and the app now offers no route to it at all.

**Scope.** Decide erase-vs-anonymise **per table first, written down** — reservations, usage logs, RFID access logs and badge awards each have a different answer, and some are records the lab needs to keep. Then the confirmation flow, and the admin's side of it.

⚠️ **This is the one place where "delete" must not mean `ON DELETE CASCADE`.** S10 already proved that lesson on booking policies: a cascade here would silently delete lab history that has nothing to do with the person's identity.

---

### S55 · What a single-feature deployment actually looks like — ✅ **shipped 2026-08-01**

**Why.** Phase A made an events-only or lending-only install *possible*; this session makes one look like it was built that way rather than like a fablab with most of the lights off. S24 sorted the nav, S28 gave a portal its own front door, S50 fixes the admin sidebar — the front page itself was never checked.

**Of the six homepage blocks, two are feature-gated and four are not:**

| Block | Gated on | Verdict |
|---|---|---|
| `featured_machines` | `machines` ✅ | correct |
| `upcoming_events` | `events` ✅ | correct |
| `mini_leaderboard` | **nothing** | ⚠️ **bug** — `leaderboard` *is* a registry feature, so switching it off hides the nav entry and 404s `/leaderboard`, while the homepage keeps rendering a leaderboard block that links there |
| `fablab_stats` | **nothing** | machine and usage statistics on a site that may have neither — and the key name is itself a leftover of the vocabulary S31 removed |
| `how_it_works` | **nothing** | describes the equipment/badge/training journey to people who may have none of it |
| `opening_hours` | **nothing** | fine — a venue has hours whatever it does |

⚠️ **`mini_leaderboard` is the same bug S37 fixed on the error page**: an affordance pointing at a route the gate 404s. It is worth fixing on its own merits before the rest of this session.

**Scope.**
- Gate every homepage block on its feature, and rename `fablab_stats` now that the vocabulary is configurable.
- **Empty states worth landing on.** An events-only site with no events currently shows `events.none_upcoming` and nothing else; it should say what this place is and — for an admin — offer to create one. Same for a fresh lending library, a training catalogue with no courses.
- **Check the whole public surface against three personas**, not one: events-only, equipment-only, lending-only. Each should read as a coherent product.

⚠️ **This is a presentation session and must not become an authorisation one.** Blocks disappearing is the same rule as the nav: hiding a block hides a block. The route gate and the firewall decide what exists.

**Verify.** Switch every feature off but one, three times over, and walk the public site as an anonymous visitor: no block, link or empty state mentions a feature that is off, and nothing offered leads to a 404.

#### What actually shipped

**Seven blocks, not six** — `latest_rfid_logs` is a seventh, staff/admin-only, and it is machine data, so it gates on `machines` too.

**The gate went into `HomepageVisibilityService::getVisibilityMap()`, not into the call sites.** That map is what the controller loads data from, what the template renders from, *and* what `HomepagePersonalizationService` builds the section order out of. Those three could disagree and did: the controller checked `isEnabled('machines')` before loading featured machines while the template rendered the section from visibility alone. One rule, one place, three consumers.

| Block | Gate |
|---|---|
| `opening_hours`, `how_it_works` | kernel — a venue has hours whatever it does |
| `upcoming_events` | `events` |
| `fablab_stats`, `featured_machines`, `latest_rfid_logs` | `machines` |
| `mini_leaderboard` | `leaderboard` — **the live bug** |

**Also fixed:** `how_it_works` step 2 linked to `/formations` unconditionally, which 404s with `formations` off — now gated exactly as step 3's calendar link already was (keep the text, drop the link). And an events-only site with no events rendered *nothing at all* here; it now says so, and offers an admin the one action that fixes it.

⚠️ **`fablab_stats` keeps its key on purpose.** It is persisted in `HOMEPAGE_SECTION_VISIBILITY.sectionKey`, so renaming it is a data migration that changes nothing anyone sees. Only the **label** was the S31 vocabulary leftover, and only the label changed.

⚠️ **Which exposed a real bug: `buildRow()` preferred the stored `label` over the code default, so the rename appeared to work and did nothing.** The label is code-owned — the admin screen renders it as a caption, never an input, and `AdminController` writes it back from `DEFAULT_SECTIONS` on every save — so the stored value was a snapshot shadowing the code. Fixed; code now wins.

⚠️ **Verifying gates on this install is not possible: all 14 features are on.** What the live site *did* prove, for free: the new events empty state renders (there are no upcoming events), and `featured_machines` is absent because the operator has switched it off for all four audiences — not because anything broke. The check that mattered was mechanical: **every `feature` key in `DEFAULT_SECTIONS` validated against the registry's 14 real keys**, since a typo there hides a block forever and silently. *(Do that check whenever a table maps to feature keys — S50 needed the same one.)*

---

### The per-page catalogue

Read against the templates on 2026-07-31 — page sizes are real, and so are the specific problems. This is the working list for S46–S48 and S51; it is not a separate session.

| Page | Today | Change |
|---|---|---|
| **`index`** (354) | Hero + blocks, homepage visibility already role-gated and orderable | Lead with **one big number and one action** for the signed-in member — their next booking, or "book something". A logged-in member should not land on marketing. |
| **`machines`** (503) | **Already a card grid** (`machines-grid` / `machine-card`), with category filters and search | The cards exist; **availability is not the primary signal on them**. Put "free now / free at 14:00 / needs a badge you don't have" on the card face, and make the filter row a segmented control rather than a select. |
| **`machine-detail`** (327) | Has the right states already — reserve, login-to-reserve, training-required, unavailable, go-to-quiz, favourite | Keep the states, raise the hierarchy: the **answer** ("you can book this, next free at 14:00") above the specification. Collapse specs and accepted materials. Certification requirements stay visible (S48's rule). |
| **`machine-historique`** (663) | Full history, unpaginated | Paginate; default to the recent window; summary counts as stat tiles at the top. |
| **`calendrier`** (589, **395 inline JS**) | The booking surface | S46 moved the JS into the page's `javascripts` block unchanged; S47 owns rewriting the interaction. |
| **`events`** (**73**) | Thin list, `events.none_upcoming` | **The most underbuilt page in the app** — an events-only deployment's front door is 73 lines. Card grid with date, capacity remaining, registration state; empty state that offers the admin a "create one" path. |
| **`event-detail`** (246) | Hero image, registration panel | Registration as a one-click primary action with immediate feedback; guest flow visible without an account. |
| **`profil`** (714) | ⚠️ **Anchor links styled as tabs** (`profile-nav` → `#info`, `#badges`, `#reservations`) — every section is rendered at once, so it is one long page pretending to be four. Plus a **`disabled` Edit button**. | Real tabs or real sections with disclosure; inline edit-in-place replacing the dead button (S49, S54). |
| **`mes-reservations`** (187) | Filter tabs (current/upcoming/past) + tables | Cards on mobile, table on desktop; cancel as a one-click action with undo rather than a confirm dialog. |
| **`formation-suivi`** (1 199) | Longest template in the app | Current step foregrounded, completed steps collapsed. Progress as a single bold number. |
| **`formation-detail`** (301) | **Five "bientôt disponible" controls** | S54 decides each. |
| **`leaderboard`** | Ranked list | The one page where big bold numbers are obviously right. |
| **`badges`** | Grid | Held vs available as the primary split; "what unlocks this" on the card. |
| **Kiosk pages** | Standalone screens | Genuinely different medium — large type, no interaction, high contrast at distance. Do not fold them into the member design system; give them their own tokens from S45. |

### Component inventory (what S51 builds, used everywhere after)

**Layout** — card, stat tile (large bold number + label + delta), section with disclosure, sheet/drawer, empty state (illustration + one action), skeleton loader.
**Input** — date/time picker, segmented control, toggle, slider for durations, combobox with search, inline edit-in-place.
**Feedback** — toast, inline validation, optimistic state with rollback, progress ring.
**Navigation** — sticky page header with the primary action, breadcrumb, mobile bottom bar, floating action button (one per screen, the screen's single primary verb).
**Status** — the `admin-status-*` set from S29, extended to the public side with the same measured contrast discipline.

⚠️ **Build these against the tokens from S45, not against `style.css`.** A component that hardcodes a hex is a component that will need the S29 dark-mode treatment all over again.

---

## Phase U (continued) — read against Fabman

🔴 **S58–S66 are PROPOSED and awaiting the operator's approval. Nothing in them is authorised to be built.** The operator asked for it to be written up, not started. Approve session by session; each one below is independently revertible and independently skippable.

✅ **S67, S68 and S69 are APPROVED in principle (2026-08-01) — packages, the booking lock and no-show release, and archive-not-delete — but none is scheduled and none is authorised to start.** They exist because the feature-gap table below got four answers; see *Decisions taken 2026-08-01*. **Billing and credits are decided NO.**

🔴 **A second comparison, against [UTA-FabLab/fabapp](https://github.com/UTA-FabLab/fabapp), added S70–S73 — also proposed, also unapproved.** It is at the end of this phase and it is the more important of the two reads: **it found a model gap, not a UI gap.** We can book a machine and we cannot queue for one, and a 3D printer is queued, not booked. See *read against UTA FabApp*.

*Added 2026-08-01, from **73 screenshots of Fabman** (`Stage/Drive/Images/Fabman UI/`) — a commercial fablab-management SaaS covering roughly the same ground: equipment, members, bookings, training, RFID door/machine control. It is a fair comparison for the shapes and an unfair one for the scope: Fabman is a paid product with billing at its centre, and this app is not. **Read the two apart: adopt the shapes, decide the features.***

### The one structural difference, from which most of the others follow

**Fabman ships two applications, not one site with an admin.**

| | staff app | member portal |
|---|---|---|
| URL | `fabman.io/manage/{space}/…` | `fabman.io/members/{space}/…` |
| Nav | **6 tabs** — Overview · Members · Activity Log · Bookings · Billing · Analytics | **3 tabs** — Equipment · My bookings · Billing |
| Configuration | one **Configure ▾** menu, 7 entries | none |
| Anonymous visitors | — | — (both need a login) |

Against FabOS today: a public header of 1 link + 4 groups (~16 destinations), an **admin sidebar of 30 entries in 6 groups**, and **123 templates in `templates/site/`, 60 of them `admin-*`**. The member's own things — bookings, availability, progress — are scattered across the public site, and S54 has just removed two of them from the main nav on the correct reasoning that a personal page does not belong in a menu of things the *site* has. **That removal created the gap S65 fills.**

⚠️ **Fabman's member portal is not equivalent to our public site and must not be copy-pasted onto it.** Ours is *also* the venue's public face — anonymous visitors, opening hours, the events an events-only deployment exists to advertise, lab pages. Fabman has no such audience. The lesson to take is the **member area**, not the deletion of the public site.

### The consistency contract — six shapes, and nothing else

The reason Fabman reads as one product with a fraction of our chrome is that **every screen is one of six shapes.** This table is the deliverable of S58–S60 and the thing every later session is checked against. **A seventh shape is a bug.**

| Shape | Where it is used in Fabman | Our equivalent today |
|---|---|---|
| **1 · List** | members, equipment, packages, courses, bookings, invoices, activity log | 20-odd hand-rolled tables and grids |
| **2 · Detail** | a member, a machine, a course, an invoice, a package | **7 `*-detail` templates, no two alike** |
| **3 · Object settings** | package, space, equipment — left sub-nav inside the object, counts on the entries | none — settings are top-level admin screens |
| **4 · Form card** | add equipment, add member, book, create invoice | ~25 forms, each its own layout |
| **5 · Wizard step** | set up courses, connect a bridge, add packages — always with *"Skip this step for now"* | `admin-wizard` / `admin-setup` only |
| **6 · Calendar** | overview day-timeline, 7-day equipment grid | `calendrier` (589 lines) + 3 overlapping stylesheets |

⚠️ **This contract only holds if it is enforced when the next feature lands.** Phase D adds around ten LMS screens. If they are built before S58/S59, they become ten more hand-rolled pages and this section is dead on arrival. **That is the argument for taking S58 and S59 before Phase D, the same argument S45/S46 won.**

### The details worth stealing, itemised

Small, cheap, and the actual source of the "feels finished" difference:

- **Every settings control is a sentence with inputs in it.** *"Allow bookings from `[2]` hours up to `[7]` days in advance."* · *"It must be cancelled `[0]` `[days]` in advance."* · *"Don't charge anything if the session is less than `[0]` seconds long."* Not a label above a box.
- **A consequence line under the control, computed from what is typed.** *"Booking this package today would cost €9 today and €9 on every 30th of the month."* · *"The next invoice will get the number 1001."* · *"Creating an invoice today would set its due date to 07/30/2026."* The form tells you what it is about to do.
- **Optional fields do not exist until asked for.** *"Add a note"* · *"Add metadata"* · *"Add a comment"* · *"Set end date"* are links; the field appears on click and gains a *"Hide"* affordance. A form opens at its common case.
- **A checkbox reveals its own sub-fields, indented underneath it.** Unchecked, they are not there at all.
- **Repetition collapses.** Opening hours show seven weekday rows; tick *"Different per weekday?"* off and they become one **"Monday – Friday"** row plus Saturday and Sunday. Same data, one third of the screen.
- **Status is a small uppercase strip under the title**, not a badge floating in the corner: `LASER CUTTER – AVAILABLE`, `ACTIVE MEMBER – joined a minute ago`, `ACCOUNT OWNER`.
- **`Edit` is a quiet link in the card's top-right**, never a button competing with the page's real action.
- **A provenance footer on every record**: *"Edited a few seconds ago by CCk CcK (view change log)"*.
- **Empty states offer the widening action**: *"No activity found for this date range. **Show results for all dates**"* — never a bare "no results".
- **Result counts are sentences**: *"1 result: 1 member, €20.00 (excl. taxes)"*.
- **Applied filters become removable chips** (`Owners ✕`) sitting under the filter row.
- **Dates are relative where relative is clearer**: *"Tomorrow at 9:00 AM"*, *"due today"*.
- **`?` tooltips explain the rule, not the field.** The one on *"Ignore member booking restrictions"* enumerates every layer it bypasses, in a paragraph.
- **Duplication is a first-class action**: *"Create a copy of this equipment"*, *"Copy all settings from…"* on create.
- **Archive, not delete**, as the default destructive action — with a split-button caret hiding the real delete.

⚠️ **Not everything there is good, and two things are actively worse than ours.** Fabman's booking grid renders **all 24 hours** including the dead ones, so a member scrolls past midnight-to-8am to find the morning; and its equipment "list" is an unsorted flat table with no card face and no availability until you open the item. **Our `machines` card grid is already better than theirs.** Copy the shapes, not the page.

---

### The feature gap — what Fabman has that we do not

Comparing entity for entity. `src/Entity/` has 39 entities and **none** of them is a plan, a charge, a note or an audit row. **Each line below needs a yes/no from the operator before it can become a session** — several are deliberate absences, not oversights.

| Missing | What Fabman does with it | Recommendation |
|---|---|---|
| **Packages / membership plans** | A named bundle of *equipment permissions* (what, when — 24/7, opening hours, or custom) + *credits* + a price, assigned per member with a start date. | ✅ **DECIDED 2026-08-01 — build it, and badges stay.** See S67 and the decision note below. |
| **Billing** — charges, invoices, payments, taxes, dunning, Stripe/SEPA | An entire tab, plus per-member charges and an invoice PDF | ❌ **DECIDED 2026-08-01 — not now.** "Not our goal, we'll do that later." It is a phase, not a session, and S67 must be built so it does not block one. |
| **Credits** | Prepaid balance, scoped to equipment or category, with an expiry | ❌ **DECIDED — deferred with billing.** ⚠️ A package therefore has **no price field** for now; see S67. |
| **Notes + metadata on every object** | A staff-private rich-text note and a free JSON blob on member, equipment, course, package, booking, charge | 🟢 **Recommend building** — S63. Cheap, and the thing every operator improvises in a spreadsheet otherwise. |
| **Change log** | *"Edited … by … (view change log)"* on every record | 🟢 **Recommend building** — S63, same session. |
| **CSV export on every list** | One cloud icon in the table header | 🟢 **Recommend building** — S59. ⚠️ S54 deleted a fake Export button from `admin-reservations`; this is what earns it back. **Do not re-add the icon before the download works.** |
| **Manually log a machine session** (*"Track activity"*) | Staff record a usage session by hand: start, optional end, member, note | 🟢 **Recommend building** — S62. We have `LogUtilisation` and an RFID box that will miss sessions; today there is no way to correct that. |
| **Invite a member to create an account** | *"X has not yet been invited to create a user — Send invitation email"*, and *"Send password reset email"* from the admin side | 🟢 **Recommend building** — S62, and it partly **pre-empts S56**: admin-triggered reset is the same mail with a different entry point. |
| **Lock a member account** | One link on the member card | 🟢 Cheap, same session. |
| **Export all member data** | One link on the member card | 🟢 Same session — and it is **half of S57**, done for free. |
| **Holidays & exceptions** on opening hours | Dated closures with a title, and a choice: *"affects only opening hours"* vs *"affects everyone — only admins can use equipment"* | 🟢 **Recommend building** — S66. `OpeningHour` exists; this is a second table and a check in `OpeningHoursProvider`. A lab that never closes for Christmas does not exist. |
| **Take equipment out of service** | *"Disable equipment (maintenance, repair, …)"* on the detail page | 🟢 **Already scoped as S49's** unbuilt surface — S62 builds it. |
| **Booking granularity, lock window, no-show release** | *"Prevent members from cancelling … `[24]` hours before"* · *"Allow others to use booked equipment if the member hasn't shown up for `[30]` minutes"* | ✅ **DECIDED 2026-08-01 — copy both, as settings.** See S68. Granularity was not asked for and is not included. |
| **Equipment categories as a first-class managed list** | A tiny CRUD screen; categories then carry permissions and discounts wholesale | 🟢 We have `MachineCategory` already — it just has no admin screen. Fold into S59 as a list-pattern proof. |
| **Archive vs delete** | Archive is the button; delete hides behind a caret | ✅ **DECIDED 2026-08-01 — build it.** See S69. Still wants S58/S59 first, because it is a change to ~15 list and detail screens. |
| **Copy an object** | *"Create a copy of this equipment / package"* | 🟢 Cheap per entity; add opportunistically, not as a session. |
| **Two-factor authentication** | Member-facing, in the portal's security screen | 🔴 S54 deleted a dead 2FA control from `profil` for good reason. Leave deleted until someone asks. |

#### Decisions taken 2026-08-01 — four of them

**1 · Packages: yes. Badges stay. They are two questions, not one.** The operator's sentence is the specification: *"a user might be allowed 24/7 access but still need training on a machine before using it if the machine requires it."*

| | answers | scope | set by |
|---|---|---|---|
| **Package** | *what may I reach, and when* | a set of equipment or categories × a time window (24/7 · opening hours · custom) | the operator, assigned per member |
| **Badge** | *am I qualified for this machine* | one machine, independent of any package | the machine's own requirement |

**Both must pass, plus the existing quotas.** A 24/7 package on a laser cutter that requires the laser badge still refuses an untrained member — and refuses them for the *training* reason, not the access reason. Fabman draws the identical line and says so in its own override tooltip: *"the member still needs to be allowed to turn on the machine, eg., have training and a valid package for the time period."*

⚠️ **This makes four permission layers, and the honest question is whether it should stay at three.** Today: badges (certification) · booking policies (quotas) · access passes (`Accès exceptionnels`, the `/staff` pass desk). **An access pass is arguably just a package with an end date** — same shape, temporary. Decide in S67 whether it becomes one, because merging them later means migrating live rows.

**2 · Billing: no, not now.** Therefore **a package has no price and no billing cycle** — it is a permission grant, nothing else. ⚠️ **Do not add a nullable `price` "for later".** An unused money column is the thing that makes the eventual billing phase harder, not easier: it will have accumulated nulls, no currency, no tax rule and no history, and the first real invoice will have to migrate all of it.

**3 · Booking lock window and no-show release: copy both.** As settings, in the same sentence form S60 builds.

**4 · Archive rather than delete.** Archive is the visible action; real deletion moves behind a caret.

---

⚠️ **Three things we have that Fabman does not, and they must survive all of this**: the **feature registry** (their app is one fixed shape; ours becomes an events-only or lending-only install), **portals** with their own accent and front door, and the **public, anonymous-readable venue site**. Any pattern imported here is imported *through* those three, not over them.

---

### S58 · One detail page, seven times over

**Why.** There are **seven `*-detail` templates** — `machine`, `place`, `event`, `formation`, `badge`, `lab-page`, `admin-utilisateur` — and no two are laid out alike, plus `machine-historique` (663) and `profil` (714) which are detail pages that grew. Fabman renders a member, a machine, a course and an invoice through visibly the same card. This is the S29 "six skeletons" finding again, on the pages a member actually reads.

**Scope.** One include — working name `site/_detail_page.html.twig` — owning: breadcrumb (`Parent › Thing › Action`); an identity card carrying title, the small uppercase status strip, a quiet `Edit` link top-right, label-above-value micro-rows, verb links at the card foot, and the provenance footer; a right rail for related objects, each with its own action button; and related-list sections below with the action in the section header and a **sentence** empty state inside.

Migrate **three** pages as proof — recommend `machine-detail`, `admin-utilisateur-detail` and `event-detail`, because they are the three most different from each other today.

⚠️ **Status strips are `.admin-status-*` from S29, promoted to the public side by S45 — not new colours.** Every one goes through the contrast harness in both themes before it ships.
⚠️ **The card's verb links are affordances, so they go through `can_reach()` (S49), never a hand-written role.**
⚠️ **`strict_variables: true`** — every slot the include reads needs a `|default()`, or the first page that omits one 500s. S47 has already been bitten by exactly this on `place-detail`.
⚠️ **Titles and status words are operator vocabulary (S31) and translated ×5 locales.** Key parity is 627 and verified; a new key is five new rows, not one.

**Verify.** The three pages rendered with `app:render` before and after, diffed with the S46 normalisations; contrast measured in both themes; and a class audit finding nothing newly undefined.

---

### S59 · One list page, and the CSV that was promised

**Why.** Every list in the app was built on its own — filters sometimes above, sometimes in a bar, sometimes absent; empty states that say nothing; and S54 found a fake *Export* button whose JS was three `alert()` stubs. Fabman's lists are one component: search + two or three filters, always visible, never behind a disclosure.

**Scope.** One include — working name `site/_list_page.html.twig` — with: an always-visible search-and-filter row; **applied filters as removable chips**; a **result-count sentence** ("2 résultats · 1 membre"); a **CSV download** control; and an empty state that **offers the widening action** ("aucune activité sur cette période — *voir toutes les dates*"). Filter selects become type-to-search comboboxes with grouped options where the list is long.

Adopt on `admin-reservations`, `admin-utilisateurs`, `admin-usage-logs` and `mes-reservations`. Ship `MachineCategory`'s missing admin screen on the same include as the proof that it is reusable.

⚠️ **Build the CSV before showing the icon.** The whole point of S54 was that an affordance that does nothing is worse than an absent one. And a CSV of `admin-usage-logs` is **personal data leaving the building** — it must be behind the same `access_control` line as the page, and it must not include badge UIDs (S38's finding).
⚠️ **Comboboxes need JS, which does not load.** Native `<select>` with `<optgroup>` until the AssetMapper decision (see S51); the combobox is a follow-up, not a blocker.
⚠️ **`admin-usage-logs` and `admin-reservations` filters are the two that are already right** (S47's inventory) — copy their defaults into the component, do not overwrite them.

**Verify.** Same list, same rows, before and after. Every filter combination still reachable by URL. A CSV that opens in a spreadsheet with correct accents (BOM) and `Europe/Paris` timestamps.

---

### S60 · Settings that read as sentences, and forms that open small

**Why.** Our settings screens are label-over-input grids where Fabman's are prose with inputs embedded, and ours show every optional field at once where Fabman's reveal them on request. The difference is not decoration: *"Allow bookings from `[2]` hours up to `[7]` days in advance"* is self-documenting, and `Min. notice (h)` + `Horizon (j)` is not.

**Scope.** Three patterns, applied to `admin-booking-policies`, `admin-opening-hours`, `admin-settings` and the booking forms:

1. **Sentence controls** — the rule written out with the inputs inline.
2. **Reveal links** for optional fields — *"Ajouter une note"*, *"Définir une date de fin"* — with a *"Masquer"* once open. Native `<details>`, no JS (S48's constraint).
3. **Consequence lines** computed under the control — *"Une réservation demain à 08:00 serait refusée : le préavis minimum est de 2 h"*. This is the S47 refusal-order warning turned into something the operator can see **while setting the rule** instead of after a member hits it.

Also: collapse the seven weekday rows in `admin-opening-hours` behind a *"Horaires différents selon les jours ?"* toggle, defaulting to the collapsed "Lundi – Vendredi" form.

⚠️ **A consequence line is computed by the real service or it is a lie.** `BookingPolicyService` and `OpeningHoursProvider`, never a second copy of the arithmetic — the S47 `NextFreeSlotService` rule.
⚠️ **Sentence layouts break first at 320px and in the longest locale.** Check both; a sentence that wraps mid-input is worse than a label.
⚠️ **Timezone on the read as well as the write** — S47 shipped a button advertising 06:00 for an 08:00 slot because `|date` rendered in the app default (UTC). A consequence line is exactly the same hazard.

**Verify.** Every rule still saves and loads identically — the forms change shape, not schema. Screenshot both themes at mobile and desktop presets.

---

### S61 · Book in three clicks, with the closed hours drawn

**Why.** S47 got `machine-detail` to advertise the next free slot and got both calendars to fill the times from the clicked slot. What is left is the shape of the surface itself, and Fabman's member path is: equipment row shows `AVAILABLE` + a **Book** button → a 7-day column grid → click a slot → a confirm card reading *"Epilog 48 / Friday, 7/31/2026 / 3:00 PM – 4:00 PM (1 hour)"* with an `Edit` link and an optional comment → **Confirm**. **Zero fields typed.**

**Scope.** The confirm step as a page of its own that states the booking as a sentence and offers exactly two actions (Edit, Confirm); `Book` promoted onto the `machines` card face; and — the part that matters most — **closed and already-taken hours drawn on the grid** rather than merely refused on submit.

⚠️ **The 24-hour grid is the thing Fabman gets wrong.** Render the opening window plus a margin, with an affordance to expand — a member should not scroll past 3am.
⚠️ **Availability on the card face is the query problem S47 deliberately refused** — 11 machines, unpaginated, one query each. It needs the single batched "upcoming reservations grouped by resource" query, and that is Phase H's **S41**. **S61 is blocked on S41 for the card face**; the rest of it is not.
⚠️ **Fewer clicks, same refusals.** Every layer still runs in `ReservationService::book()`. A confirm page that posts to anything else is a hole, not a shortcut.
⚠️ **A slot suggested is not a slot held.** Between the grid and Confirm someone else can take it; the refusal must name that, not blame the member.

**Verify.** Re-count clicks against the S47 baseline table for all four flows. Every refusal branch still reachable and still explained.

---

### S62 · The staff surfaces, and the honest override

**Why.** S49 shipped the *mechanism* (`can_reach()`) and none of the *surfaces*: nobody has built inline staff editing, out-of-service, or roster check-in. Fabman shows what those look like — and shows the one thing we have no equivalent of at all: an **explicit, labelled override**. Their booking form has *"Ignore member booking restrictions"* whose tooltip enumerates every layer it bypasses and then says what it still cannot do ("the member still needs to be allowed to turn on the machine").

**Scope.** Build S49's table, plus the member-record actions Fabman has and we lack:

- **Book on behalf of a member**, and a `Staff only` reservation that holds a machine with no member attached.
- **"Ignore member booking restrictions"** — staff-only, logged, with the enumerating tooltip.
- **Take equipment out of service** from `machine-detail` — with a reason and an expected return, shown to members as a reason and not as a silence.
- **Track activity** — record a machine session by hand (start, optional end, member, note) for the sessions the RFID box misses. `LogUtilisation` already exists.
- On the member record: **send an invitation** to create an account, **send a password reset**, **lock the account**, **export all their data**.

⚠️ **An override is a security decision and must be audited, not just gated.** It writes who overrode what and why; without that it is a back door with a nice label. This is the strongest single argument for taking **S63 first**.
⚠️ **It must not be able to override the things that are physical safety.** Certification for a laser cutter is not a scheduling preference. Fabman draws exactly this line and says so in the tooltip — copy the line and the sentence.
⚠️ **`ROLE_ADMIN` does not imply `ROLE_STAFF`.** Every one of these goes through `can_reach()` against its own route's `access_control` (S49), and this install still has **no staff-but-not-admin account** to test with. **Create one before this session, not during it.**
⚠️ **"Send password reset" from the admin side is S56's mail with a different trigger.** Build S56 first or build the token here and let S56 add the public entry point — but do not build the token twice.
⚠️ **"Export all member data" is half of S57** and touches the same erase-vs-anonymise question. Read S57 before writing the export.

**Verify.** Both directions, as S49 did: the affordance absent for a member *and* the endpoint refusing. Plus: an override leaves an audit row, and a certification refusal survives the override.

---

### S63 · Notes, metadata, and a change log

**Why.** Fabman puts a staff-private note and a free-form JSON blob on every object, and a *"Edited … by … (view change log)"* footer on every record. We have neither, so every operator improvisation — why this machine is really out of service, which locker number this person has — lives in a spreadsheet or in someone's head. And S62's override is not safe to ship without the log.

**Scope.** A polymorphic note (author, timestamp, body, target) and an optional metadata blob, exposed on member, machine, place, event, reservation and loanable item; plus a change log written on the mutations that matter, and a provenance footer rendered by S58's detail card.

⚠️ **New ORM entities — the migration ships and runs *before* the code.** The expand rule; see `feedback_fabos_migration_hazard`.
⚠️ **Notes are staff-private and will contain things about people.** They are personal data: gated on the route's own rule, excluded from the public API, included in S57's export and deletion decision, and **never** rendered on a public detail page. The S38 lesson — a field that looks like a label is not necessarily one.
⚠️ **A change log grows without limit.** Decide retention now, not when the table is 40M rows: what is kept, for how long, and whether it is trimmed. Phase H's S41 is the session that will otherwise have to.
⚠️ **Do not log the values of secret fields.** `machineToken`, password hashes, anything from `.env` — a diff-based logger will happily record all three.

**Verify.** A note round-trips and is invisible to a member. A change to a machine appears in its log with the right author. The public API and every public template are re-grepped for both new fields, the way S48 re-grepped for `machineToken`.

---

### S64 · Finish the thing you just created

**Why.** In Fabman, saving a machine with *"requires training"* ticked does not return you to a list — it routes you to *"OK, so members need a training course for Epilog 48. Do you want to create one right away?"*, then to *"Connect a bridge"*, then to the finished detail page. Every step offers *"Skip this step for now — you can always return here later."* Our `admin-machine-new` saves and drops you on `admin-machines`, with the badge and the RFID reader left as things you have to know to go and do.

**Scope.** A `?new=1` step chain after creation: machine → the badge it requires (create or attach) → the RFID reader → the detail page. Same for event → registrations settings, and formation → sections. Plus the dismissible **onboarding checklist** on `/admin` that ticks its own items off (Fabman's *"Ready to dive in?"* panel), reusing `FirstRun` / `SetupHealth`, which already know most of the answers.

⚠️ **Every step is skippable and the object is already saved before the chain starts.** A wizard that can lose work is worse than no wizard. Fabman's steps are post-save navigation, not a multi-step form.
⚠️ **The chain has to respect the feature gates.** Offering "create the badge" on an install with `badges` off is offering a 404 — the exact class of bug S37 and S55 each fixed once.
⚠️ **`admin-wizard` and `admin-setup` already exist.** Read them first; this is an extension of that idea to per-object creation, and a third setup mechanism would be its own problem.
⚠️ **Dismissal must persist per operator**, or the panel comes back every login and becomes noise. `HomepageUserPreference` is the nearest existing pattern.

**Verify.** Create a machine requiring a badge on a fresh install and reach a fully configured machine without visiting a list. Do it again with `badges` off and reach the detail page with no dead step. Skip every step and land somewhere sensible.

---

### S65 · A member area, not a member page

**Why.** S54 removed *Mes réservations* and *Mes disponibilités* from the main nav — correctly, because a menu of what the *site* has is the wrong home for one person's own things — and left them reachable only from a link inside `/profil`. Fabman's answer to the same problem is a **portal**: three entries, one of which is "my bookings". `profil` is 714 lines with anchor links styled as tabs, which is not an area; it is a long page pretending to be four.

**Scope.** A small account area — recommend `/mon-compte` — with a handful of entries: *Mes réservations*, *Mes disponibilités* (only for people who are bookable), *Ma progression*, *Mon compte*. `profil` becomes its landing screen and sheds the sections that move out. The identity card, next booking and badges held stay foregrounded; history goes behind S48's disclosure.

⚠️ **This is not Fabman's member portal and must not swallow the public site.** Anonymous visitors, opening hours, events and lab pages stay exactly where they are. This is an area *inside* the public site for the signed-in member's own records.
⚠️ **The bookable-person case is the one S54 knowingly made worse** and wrote down: the old code gave *Mes disponibilités* a nav link specifically for `isBookable()` people, and that reasoning is now only in the roadmap. **This session is where it comes back** — as an entry in the account area, not in the site menu.
⚠️ **`profil` server-renders `data-theme-preference` on `<html>`** via S46's `html_attrs` hook. Whatever becomes the landing screen inherits that, or the signed-in theme silently stops applying.
⚠️ **Every route moved is a URL someone has bookmarked or mailed.** Redirect the old ones; do not just move them.

**Verify.** Every destination reachable in ≤2 clicks from anywhere signed in. A bookable person finds their availability page without being told where it is. Old URLs still resolve.

---

### S66 · Holidays, exceptions, and the closures nobody modelled

**Why.** `OpeningHour` describes a normal week and nothing else. Every real venue closes for holidays, and the current answer is to edit the weekly hours and remember to change them back. Fabman models it as dated exceptions with a title and — the part that is actually thoughtful — a **scope**: *"affects only opening hours: 24/7 packages and packages with custom times work as usual"* versus *"affects everyone! Only admins can use equipment during this time."*

**Scope.** A dated exception (title, from, until, all-day or timed, scope) read by `OpeningHoursProvider`, so that it flows for free into `BookingPolicyService`, the calendars, `NextFreeSlotService` and the homepage's opening-hours block. Past entries hidden behind *"Voir les entrées passées"*.

⚠️ **New ORM entity — migration first**, and the read path is the one that matters: a closure that the booking checker does not see is a closure that lets members book Christmas Day.
⚠️ **The two scopes are two different questions, and the honest one for us is different from Fabman's.** They distinguish by package; we have no packages (see the gap table). Our equivalent is almost certainly "closed to members, open to staff" — **decide the wording before the schema**, because it becomes a column.
⚠️ **`Europe/Paris`, pinned on the read and the write.** An all-day closure stored as UTC midnight closes the wrong day twice a year.
⚠️ **An exception must not silently cancel bookings that already exist inside it.** Decide: refuse the exception, or list the affected bookings and make the operator act. Do not cascade — the S10 lesson.

**Verify.** Set a closure; `/api/opening-hours`, the calendar grid, the next-free-slot button and the homepage block all agree. Booking inside it is refused with a reason naming the closure. Staff can still book if the scope says so.

---

### S67 · Packages — what you may reach, and when

**Why.** Decided 2026-08-01. Access is currently expressed three ways and none of them answers *"this person may use the workshop 24/7, that one only during opening hours."* Badges answer qualification, quotas answer volume, access passes answer exceptions — nothing answers the ordinary standing grant.

**Scope.** A `package`: a name, a description, and a set of grants. Each grant is *(equipment or category) × (24/7 · opening hours · custom window) × (may book, yes/no)*. Assigned to a member with a start date and an optional end date. Read by `BookingPolicyService` on booking and by the RFID authorization endpoint on machine start.

**No price. No billing cycle. No credits.** Decision 2 above.

⚠️ **The check is an AND with the badge, and the refusals must be distinguishable.** "You may not be here at this hour" and "you are not trained on this machine" are different problems with different fixes, and a member told the wrong one goes to the wrong person. This is S47's 403-vs-409 rule again: the wording has to offer the fix.

⚠️ **Refusal order matters, as it did in S47.** Package window before badge, or an untrained member booking at 3am is told to go and get trained for a slot they could never have had.

⚠️ **Decide access-passes-versus-packages before the migration, not after.** If a pass is a package with an end date, say so now and migrate the rows once. See the decision note above.

⚠️ **The RFID endpoint is the half everyone forgets, and it is the half that opens doors.** `ReservationService::book()` is not the only gate — `RfidMachineController` authorises a physical machine start. A package enforced only in the web booking flow is not enforced. ⚠️ **And that endpoint currently fails open** when `FABOS_RFID_API_TOKEN` is unset, which it still is on the live box (S48). **Fix that first or S67 ships a permission system anyone can bypass with curl.**

⚠️ **New ORM entities — migration first, then code.** Expand rule.

⚠️ **A member with no package.** Decide explicitly: does that mean no access, or unrestricted access? Getting it wrong in either direction is a live incident — one locks everyone out on deploy day, the other makes the whole feature decorative. **Recommend: the existing behaviour is preserved until an operator creates the first package,** so the migration is inert until used.

⚠️ **Feature-gate it.** A lending-only or events-only install has no use for equipment access windows; this is a registry feature like the rest.

**Verify.** A member with an opening-hours package is refused at 3am by the web form *and* by the RFID endpoint. A member with 24/7 and no laser badge is refused on the laser and told it is the training. A member with 24/7 and the badge is allowed. An install with no packages behaves exactly as it does today.

---

### S68 · Lock window and no-show release

**Why.** Decided 2026-08-01, copied from Fabman's booking settings. Two rules we have no equivalent of: a member cancelling a slot ten minutes before it starts wastes it, and a member who books and never turns up blocks a machine for everyone else.

**Scope.** Two settings, written as sentences (S60's form):

- *"Empêcher les membres d'annuler ou de modifier une réservation `[24]` heures avant son début."*
- *"Autoriser d'autres personnes à utiliser un équipement réservé si la personne qui l'a réservé ne s'est pas présentée depuis `[30]` minutes."*

Both live in `BookingPolicyService` beside the existing min-notice, horizon and quota rules.

✅ **COUNTED 2026-08-02: there is 1 `RFID_READER` row for 11 machines.** So the decision is settled — **the no-show release is per-resource and off by default**, offered only where a reader exists, and the lock window (which needs no hardware) ships independently of it. Do not re-open this on an assumption; re-count.

⚠️ **The no-show release needs a "showed up" signal, and we only have one on machines with a reader.** The signal is the RFID work-session start (`LogUtilisation` / `MachineUsageHistory`). **On a machine with no reader there is no signal at all, so the rule would release every single booking after 30 minutes.** Therefore: the setting is *per resource and only offered where a reader exists*, or it is globally off by default and the operator is told which machines it can apply to. **Check the reader coverage across the 11 machines before choosing** — do not assume it is total.

⚠️ **Releasing is not cancelling.** The original booking must stay visible and attributed, or the member is told they never booked and the lab loses the no-show record it wanted. Release the *slot*; keep the *row*, marked.

⚠️ **The lock window and staff are different questions.** Staff must still be able to move or cancel a locked booking — that is S62's override, and it must be audited (S63).

⚠️ **`Europe/Paris` on read and write.** Both rules are clock arithmetic near a boundary; this has bitten twice already.

⚠️ **No cancellation-fee logic.** That is billing, and billing is decided no.

**Verify.** A member cancelling inside the lock window is refused with a message naming the deadline; outside it, allowed. A booking on a reader-equipped machine with no session start releases after the configured minutes and the row survives, marked. A machine with no reader never releases anything. Staff can still move a locked booking, and it lands in the change log.

---

### S69 · Archive, not delete

**Why.** Decided 2026-08-01. Deleting a machine, a course or a member destroys history that has nothing to do with wanting the thing off the list. Fabman's answer is that **Archive is the button** and real deletion hides behind a caret.

**Scope.** An archived flag on the entities that are listed and retired rather than genuinely removed — machine, place, loanable item, material, course, badge, package, event. Archive as the primary action; delete demoted behind a disclosure and kept only where it is genuinely safe. An *"Afficher les archivés"* control on the affected lists.

⚠️ **Every list query needs the filter, and the one that is forgotten is the bug.** An archived machine reappearing on `/machines` is the obvious failure; an archived machine still counted in a dashboard statistic is the one nobody notices. **Grep the repositories, not the templates.**

⚠️ **Uniqueness still applies to archived rows.** Archiving `printer-01` does not free its `machineToken` — the next machine created with that name will collide. Decide per field whether archiving releases the constraint.

⚠️ **Archived must not mean invisible to the things that reference it.** A past reservation on an archived machine still has to render the machine's name, and a badge awarded by an archived course is still held. Archive hides it from *choosing*, never from *reading*.

⚠️ **This is a change to ~15 admin screens, which is why it comes after S58 and S59.** Done before them it is fifteen hand-edits; done after, it is one change to two includes.

⚠️ **Not the same thing as S57's account deletion.** A person is not an inventory item, and the erase-vs-anonymise decision there is per table and still unmade. Keep them apart.

**Verify.** An archived machine leaves every list, keeps rendering in existing reservations, and stops being offered anywhere new. Toggle the "show archived" control and it comes back. Statistics agree with the lists.

---

---

## Phase U (continued) — read against UTA FabApp

🔴 **PROPOSED. S70–S73 are not approved and not authorised to be built.**

*Added 2026-08-01 from **[UTA-FabLab/fabapp](https://github.com/UTA-FabLab/fabapp)** — the University of Texas at Arlington FabLab's own operational system, open source, PHP, v1.0 in production, 22 stars.*

**Operator context, and it explains the design: it is a full fablab like ours, but the bulk of the usage is the printers — and the operator has been there.** So this is not a narrow single-purpose tool whose ideas need translating; it is the same kind of place as ours, further along, with the parts that a heavy-traffic queue actually needs already built. ⚠️ **They also run physical hardware we do not: numbered storage boxes, and small thermal printers that print a paper queue ticket.** That is why `wait_ticket.php` exists, and it is a real dependency in S70 below — not a UI detail.

⚠️ **This is not a codebase to copy from and this section does not propose copying any of it.** Raw PHP with an `admin/sub/*.php` and `pages/sub/*.php` AJAX layer, no framework, no tests visible. **Our Symfony/Twig architecture is the better one and stays.** The value here is entirely the **domain model** — what a busy university makerspace actually turned out to need after running for real — and on that, they are ahead of both us and Fabman.

### The finding that matters more than anything in the Fabman section

**Booking is the wrong model for a 3D printer, and booking is all FabOS has.**

| | the interaction | the right model |
|---|---|---|
| Laser cutter, CNC, sewing machine, a room | you stand at it, or occupy it, for a known time | **book a slot** — ours and Fabman's model |
| 3D printer, and any long unattended job | you submit an 8-hour job and *leave*, then come back for the part | **join a queue** — UTA's model |

**Both are needed in the same lab, and UTA is a full fablab that needs both too.** The point is not that printers are a different kind of building; it is that **the interaction is different and only one of the two is modelled here.** `Reservation` is polymorphic across machines, places and people, and every single one of them is a time slot. **Nobody reserves a printer for 8 hours in a lab with three printers and forty students**, so what happens instead is a paper list, a whiteboard, or a queue in someone's head — and the app does not know about any of them.

UTA models it properly: `Wait_queue` with `Operator`, a **device *or* a device group** (queue for "any FDM printer", not printer #3), `estTime`, `last_contact`, and a `valid` flag; `calculateWaitTimes()` estimating from the jobs actually running; contact details captured **per ticket** (email, phone, carrier) so the member can leave the building; a `now_serving` screen for staff; and `transferFromWaitQueue()` when they are called.

⚠️ **And it does not stop at the queue — the loop only closes because of the next two pieces.** The job finishes while the member is elsewhere, so the part has to go *somewhere*: `StorageBox` / `ObjBox`, a numbered physical bin, with `pickup.php`. **Queue → print → bin → notify → pickup is one workflow, and building only the first third of it leaves parts on a shelf with names on masking tape.** That is the honest scope warning for S70.

### What they have that we do not

`class/` is the whole domain: `Accounts · AuthRecipients · Devices · IndividualsCertificates · Materials · Notifications · ObjBox · OfflineTrans · Purpose · Service · Status · StorageBox · TrainingModule · Transactions · Users · Wait_queue · wait`.

| Missing | What it does | Recommendation |
|---|---|---|
| **Wait queue** | Queue for a device *or a device group*, estimated wait, notify by email/SMS, staff "now serving" | 🟢 **Strongly recommend — S70.** The one genuine model gap in this app. ⚠️ Probably phase-sized, not session-sized. |
| **Finished-object storage** | Numbered bins, object → bin, pickup, notification | 🟢 **Recommend — S71**, and it is **not optional if S70 ships**; see above. |
| **Member-reported faults** | `Service` — a member opens a ticket on a device, with history and a log; plus offline tickets | 🟢 **Recommend — S72.** We have `MaintenanceTask`, authored by admins. **The person who finds a broken machine is the member standing at it**, and today they have no way to say so. |
| **Purpose on a usage session** | A short list — why is this person using this machine (a course, research, personal) | 🟢 **Recommend — S73.** Cheap, and it is the reporting axis a *university* lab is actually asked for: how much laser time went to which course. We have `Institution`; we have nothing per session. |
| **Certificate revocation + expiry** | `training_revoke.php`, `IndividualsCertificates`; Fabman independently has *"training is only valid for `[12]` months"* | 🟡 **Check first, then decide.** We award `UserBadge`/`UtilisateurBadge` — verify whether anything can take one back or expire it before scoping. **Two independent products both have this; that is a signal.** |
| **Live "in use" state** | `isInUse.php`, `getDot.php` — a dot showing whether a machine is running right now | 🟢 Fold into **S61**'s card face. We already have the signal (`LogUtilisation` / RFID work sessions) and do not show it. |
| **Onboarding / offboarding** | Explicit member lifecycle flows, not just create/delete | 🟡 Partly **S62** (invite, lock) and **S57** (deletion). Worth naming the two ends explicitly. |
| **Consumables: cart, stock decrement, sheet goods with variants** | `add_cart` / `Materials` / `inventory_quantity` / `sheet_goods` + `si_getVariants` | 🟡 **Split it from money.** ⚠️ Their version is wired to `Transactions` and `Accounts`, which is **billing, decided no**. But *"this member took 400mm of 3mm ply, decrement the stock"* is stock control, not accounting, and we already have `Material`. Recommend the stock half, explicitly without the ledger. |
| **`Transactions` / `Accounts` / `OfflineTrans`** | A member money ledger | ❌ **Billing. Decided no 2026-08-01.** Consistent with the Fabman answer. |
| **OctoPuppet / JuiceBox integrations** | OctoPrint control, physical device power | 🟡 Out of scope for Phase U, but note it: **their printers are software-controlled, ours are RFID-gated.** A queue that can *see the job finish* is a much better queue than one that cannot. |

---

### Direction taken 2026-08-01, second round — read before scoping S61, S67 or S70

Seven decisions from the operator, in their own framing. **Several change sessions that were already written; where they do, the session below says so.**

#### 0 · The principle: anything the member can do, the staff will not have to

*"User interactions are important… Anything the user can do the staff won't have to!"*

**This is now the tie-breaker for every affordance question in the phase.** S49's table asked "who edits what, where"; this answers the ambiguous rows in favour of the member. Concretely it decides: members report faults (S72), members claim their own place in a queue (below), members pick up their own parts (S71), and — see decision 6 — members may do maintenance.

⚠️ **It does not weaken any server-side check.** "The member can do it" means the affordance is theirs; the three permission layers still run. Self-service and unchecked are different words.

#### 1 · Categories become a first-class, app-wide concept — new session **S74**, and it is a prerequisite for S70

*"I do like their categories so you queue for a laser or a specific laser (ex: the only one big enough). It would be useful app wide this whole category thing."*

`MachineCategory` exists today and is used for **filtering a list, and nothing else**. It becomes the unit you can *act on*: queue for "a laser", browse by category, and — decision 2 — book one.

#### 2 · You book the category, not the machine, unless you need a specific one

*"If two machines only have limited space you book that instead of the specific machine."*

The common case is that any machine in the category will do; picking a specific one is the **exception**, for when only that one is big enough. So booking a category means booking *a* member of it, and the specific-machine path stays for the exception.

⚠️ **The hard question is when the pool resolves to a machine, and it must be answered before any code.** Two options, and they produce different calendars:

| | resolve **at booking** | resolve **at start** |
|---|---|---|
| The member is told | "you have printer #2 at 14:00" | "you have *a* printer at 14:00" |
| Utilisation | worse — a gap can't be refilled | better — whoever is free wins |
| The calendar shows | a booking on a machine row | a booking on a *category* row |
| Failure mode | fragmentation | the member arrives and is sent to a machine they didn't expect |

**Recommend resolve-at-booking with a re-assign on the day**, because it is the only one where the calendar stays readable and the member knows where to stand — but this is the operator's call and it is load-bearing.

⚠️ **`Reservation` currently points at a resource.** Pointing at a category is a schema change, and it is the same polymorphism question S8–S10 already answered once for machines/places/people. **Re-read those before adding a fourth shape.**

#### 3 · The calendar stays a calendar — this constrains S70

*"I do believe our calendar should only have that, the calendar with current infos and handles reservation as it does."*

`/calendrier` keeps its job: current information and reservations. **The queue does not go on the calendar grid.** A queue has no start time, so drawing it there would be inventing one.

⚠️ **This kills the most obvious S70 design** — "show the queue as pending blocks on the grid". Queue lives on the machine page, on a category page, and on a staff "now serving" screen. Good: it also means S70 does not have to touch the 589-line calendar or its three stylesheets.

#### 4 · The machines page leads with categories, and status is unmissable — this specifies S61 and S48's `machines` row

*"Machines could showcase the categories first, then the machines with their status obvious (free / used / you don't have the badge…)"*

So: category first as the primary grouping, machines within it, and **on every machine a single obvious state**. The three the operator named are three different kinds of thing and must not be one badge colour:

| state | kind | what the member does next |
|---|---|---|
| **Free** | availability | use it, book it |
| **In use** | availability | queue for it, or see when it frees |
| **You don't have the badge** | *permission* | go and get trained — a different verb entirely |

⚠️ **Add a fourth: "out of service"** (S62), and a fifth once packages land: **"not in your package / outside your hours"** (S67). ⚠️ **Permission states must not be rendered as availability states** — a member who reads "unavailable" when they mean "untrained" goes and waits instead of going and training.

⚠️ **This is the S47 query problem again, now larger.** Per-card live status across every machine, grouped by category, is the batched-query work parked in Phase H **S41**. The *layout* can ship first with status on the detail page only.

#### 5 · The badge swipe is the claim action — this joins S68 and S70

*"If someone swipes their card after the no-show limit, it moves to them in the queue."*

The RFID swipe is already the "I am physically here" signal. It becomes the arbitration between a released booking and the queue: the no-show window expires, the slot is released, and **the swipe is how the next person takes it** — no staff involvement, which is decision 0 exactly.

⚠️ **The operator's sentence has two readings and the difference matters**: does the machine go to *whoever swipes*, or to *the person at the head of the queue, once they swipe*? The second is fair; the first is a race won by whoever loiters nearest. **Recommend: only the head of the queue can claim, and the ticket is offered to them for a bounded time before it passes down.** Confirm before building.

⚠️ **This makes the RFID endpoint a decision point, not just a gate** — it now reads the queue and mutates it. That endpoint **still fails open** with `FABOS_RFID_API_TOKEN` unset (S38/S48). It was already true for S67; it is now true for S70 as well.

#### 6 · Packages grant *capabilities*, not just access — this extends S67

*"Maintenance in some labs would be done by members and no staff, but packages could handle this (ability to do maintenance becomes a thing part of the package or not)."*

So a package's grants are not only *(equipment × window)* but also named capabilities — the first being **may perform maintenance**.

⚠️ **This is the biggest architectural consequence in this whole message, and it needs deciding, not assuming.** It makes packages a **third authorisation source** beside roles and badges — and S49's whole mechanism, `can_reach()`, consults `security.access_map`, which **cannot see a package**. A maintenance action gated by a package is invisible to the one mechanism built to keep affordances honest.

Three ways out, and the operator or the next session must pick one:

1. **A package grants a role.** Reuses the firewall and `can_reach()` unchanged. Crude — roles are global, packages are scoped.
2. **A voter.** Correct in Symfony terms; means `can_reach()` gains a companion for capability questions, and S49's "one map, no second copy" property weakens.
3. **Capabilities stay out of packages** and member-maintenance is a role an admin grants. Simplest, and loses the operator's point.

⚠️ **Recommend 1 or 2, and recommend deciding it in S67 rather than S70** — but ⚠️ **do not let packages become a general-purpose RBAC.** A small closed list of capabilities, defined in code, is a feature; an operator-editable permission matrix is a second security system.

⚠️ **Member-performed maintenance touches safety.** "May log that I cleaned the bed" and "may sign off that the laser is safe again" are not the same permission. **Split them before building**, or the out-of-service flow (S62) can be cleared by whoever feels like it.

#### 7 · Ticket printer: yes, but a future release

*"We could add a printer if you know how to handle the print in a future release."*

**Yes, this is tractable.** Thermal receipt printers speak **ESC/POS**, and the clean path for us is a **network printer on the lab LAN, addressed from the server over raw TCP 9100** — the app renders the ticket as ESC/POS bytes and writes them to a socket. No driver, no browser print dialog, no kiosk PC. CT 210 already sits on `192.168.100.x` with the RFID readers, so this is the same shape of device we already talk to.

Fallback if the printer must be USB: a **small polling agent** next to it, exactly mirroring the existing RFID device pattern.

⚠️ **The printed ticket must never be the only record of a place in the queue.** Printers jam, run out of paper and get unplugged. The queue position lives in the database; **the paper is a receipt, not the ticket itself** — otherwise a paper jam loses someone's turn.
⚠️ **Print asynchronously.** A dead printer must not block or fail joining the queue; queue the print job the way mail is queued (`messenger`, and the `fabos-worker` unit already runs).
⚠️ **Deferred to a future release by decision.** Not in S70's scope; **S70 must work with mail-only** and treat paper as an enhancement.

---

### S74 · Categories you can act on

**Why.** Decisions 1 and 2. `MachineCategory` is a filter today; it becomes the thing a member queues for and books. It is a **prerequisite for S70** — queueing for "any FDM printer" is meaningless without it — and it is what the machines page leads with.

**Scope.** Promote the category: an operator-managed list (UTA and Fabman both have this tiny CRUD, we do not), category pages, category-first grouping on `/machines`, and the pool-booking model from decision 2 once its resolution timing is settled.

⚠️ **Resolve the at-booking-vs-at-start question first.** It is in decision 2 and everything else depends on it.
⚠️ **Categories are operator vocabulary (S31) and per portal**, like everything else nameable.
⚠️ **Not every category is a pool.** Two lasers of different bed sizes are one category and *not* interchangeable. **A category needs a "these are interchangeable" flag**, or pool-booking will hand someone a machine their job does not fit — which is the exact failure the operator named when they said "the only one big enough".
⚠️ **`places` are polymorphic with machines in `Reservation` already.** If categories become bookable, decide whether places get them too, or the two halves of the calendar diverge.

**Verify.** A member books a category and gets a usable machine. A member who needs the big laser can still pick it. A non-interchangeable category refuses to be pool-booked. Every machine shows one unambiguous state.

#### What the prototype settled, 2026-08-01

Built at `/proposition/machines` and iterated with the operator. **Approved by inspection; these are now S74's brief, not open questions.**

⚠️ **Categories are a quick-filter bar, NOT section headings — and this was learned the hard way.** The first build grouped the grid into one section per category. Eleven machines across six categories renders **one card and three empty cells, six times down the page**: *a group of one is a row of one*, and no amount of styling fills it. The operator's words were "lots of emptiness, I don't like it." Moving categories up into a persistent tile bar made them **more** prominent than a heading was — always on screen, carrying the icon and the free-count — while the grid below stays one continuous flow. ⚠️ **Do not re-propose category sections.**

⚠️ **The category tile carries three things and the third is the point**: icon, total, and **how many are free right now**. That last number is what decides whether the member walks over there.

⚠️ **The icon belongs to the category, not the machine.** `iconSlug` is a column on `Machine` today, so eleven rows each carry their own copy of "what a laser looks like". It moves to the category. **Repeat it on every card too** — in one continuous grid there is no heading saying what kind of thing you are looking at, and the icon lets the eye sort by shape without reading a word.

⚠️ **`MachineCategory` exists as an entity and `MACHINE_CATEGORY` does not exist in the database** — 1146, checked on the live box. Categories are denormalised `categorySlug` / `categoryLabel` strings on `Machine` and nothing else. **S74 cannot "just use `MachineCategory`"; it has to create the table.** This is the known entity-ahead-of-migrations drift, not a broken deploy — but it means S74 starts with a migration.

✅ **Categories live inside the `machines` feature and are operator-editable**, like every other piece of vocabulary (S31), and per portal.

🟡 **Shipping a default set of categories with the app was raised and explicitly deferred** — "we'll deal with that later." Worth revisiting: both Fabman and UTA ship one, and an empty category list is a bad first run.

⚠️ **`Machine.photo` is editable in the admin and rendered on the public list by nothing.** So the grid has never met a real photograph, and the normal case is a half-populated lab — three machines shot, eight not. **Both states need the same fixed aspect ratio** or the grid goes ragged the day someone uploads one picture. The prototype cover-crops a photo and falls back to the category icon on a tinted tile; neither reads as a missing image.

⚠️ **The machine icons hardcode `#9E1B56`** in `machines.html.twig`. The prototype switched them to `var(--color-primary)` so they follow the portal's own accent (S27) and dark mode. **A page full of hardcoded brand hexes is what turns a rebrand into a rewrite** — fix it in S58/S74, not later.

---

### S75 · Remove favourites

**Why.** Operator decision, 2026-08-01: *"We can remove favourites altogether, not sure the value it adds."* Taken while comparing the machines list against the prototype — with the star gone the card header rebalances, and the category icon takes the vacated side and does real work.

**Scope.** `MachineFavorite`, `MachineFavoriteRepository`, the favourite controller and its routes, the star on the card and the list row, the `data-favorite` attributes, the "Mes favoris" filter button, the favourites branch of the list JS, the sort-favourites-first pass in `SiteController::machines()`, the CSS, and the translation keys across five locales.

⚠️ **This closes an open audit item rather than leaving it.** The 2026-07-10 audit flagged **missing CSRF on the favourite add/remove endpoints**. Deleting the feature removes the vulnerability — but only if the *endpoints* go, not just the button. **Grep for the routes, not for the star.**

⚠️ **It is a table with member data in it.** Dropping it is a contract migration: code first, then the migration, and it belongs in the same conversation as S57's erase-vs-anonymise decisions.

⚠️ **S54's rule applies to the CSS.** Two of the rules it deleted were selector *lists* mixing dead and live selectors — prune selectors, never delete whole rules — and removing a cell from a grid row means fixing `grid-template-columns` too. The machine list row is a nine-column grid; check it.

⚠️ **Locale parity is verified at 627 keys × 5.** Every `machines.fav_*` key goes, in all five, and the parity check runs afterwards.

**Verify.** `grep -ri favorite src/ templates/ public/css/ translations/` returns nothing. The nine-column list row still lines up. The endpoints 404. Locale parity holds.

---

### S70 · A queue for the machines nobody can book

**Why.** See above. This is the model gap, not a UI gap.

**Scope.** A queue ticket: person, a device **or a device group**, joined-at, an estimated wait computed from what is actually running, a contact channel, and a state. A member joins from the machine page; staff see a "now serving" screen; being called transfers the ticket into a real usage session.

⚠️ **Per resource, not per lab.** A laser cutter is booked and a printer is queued **in the same lab, on the same day**, and both models must coexist. The choice belongs on the resource — a flag on `Machine`, decided by the operator. **Do not replace booking; add a second mode beside it.**

⚠️ **"Any FDM printer" is the whole point.** Queueing for a *specific* printer recreates the problem — the member waits behind a 9-hour job while an identical machine sits idle. This needs device *groups*, and we have `MachineCategory` already, unused for anything but filtering.

⚠️ **An estimate that is wrong is worse than no estimate.** It has to come from real running-session data (`LogUtilisation` / `MachineUsageHistory`), it has to say it is an estimate, and it must degrade honestly to "we can't tell yet" on a machine with no history.

⚠️ **Notification is the feature.** A queue whose member has to stand and watch the screen is a whiteboard with extra steps. Mail exists (S13/S15, the worker and the reminder timer). **SMS does not, and UTA carries `phone` + `carrier` per ticket for exactly that reason** — decide whether we need it, and if so it is a new external dependency and a cost, not a session detail.

⚠️ **UTA prints a paper ticket, and we have no printer.** `wait_ticket.php` drives a small thermal printer: the member takes a physical ticket and walks away, which is what makes the deli-counter model work for someone with no phone to hand. **Decide the equivalent before designing the flow** — a printed ticket, a QR the member scans to hold their place, or mail-only. Each gives a different screen. ⚠️ **Do not assume the hardware**: if a ticket printer is wanted it is a purchase and a driver, and the kiosk screens (`kiosk-*`) are the nearest thing we have to that medium today.

⚠️ **Feature-gate it** like everything else in the registry, and ⚠️ **it interacts with quotas**: is a queue ticket subject to the booking quota? Decide before building, or a member holds two bookings and six queue tickets.

⚠️ **This is very likely a phase, not a session** — an entity, a member surface, a staff surface, notification, and the storage loop below. **Scope it honestly before approving it.**

**Verify.** Two machines in one lab, one bookable and one queued, both working. A queue for a group hands the member the first free machine of that group. The estimate is derived from real sessions and says so. Leaving the queue works and does not notify the wrong person.

---

### S71 · Where the finished part waits

**Why.** S70 only makes sense if it closes. The member left the building; the print finished; the part exists and needs an address.

**Scope.** Numbered storage bins, an object assigned to a bin when a job completes, a notification to its owner, and a pickup action that frees the bin. UTA has `StorageBox`, `ObjBox`, a bin creator and a pickup page.

⚠️ **Do not build this before S70 is approved, and do not ship S70 without it.** Separately they are both half a workflow.

⚠️ **Bins are physical and finite.** The interesting states are "all bins full" and "this part has been here three weeks" — an unclaimed-object policy is an operator decision, and it is the one that decides whether the shelf works. Ask before designing.

⚠️ **UTA has the boxes; we would be buying shelving as well as writing software.** The operator has seen theirs. **Confirm the physical setup exists or is planned before this is scheduled** — a bin-numbering screen for bins that do not exist is the purest form of the dead affordance S54 spent a session deleting.

---

### S72 · The member who found the broken machine

**Why.** `MaintenanceTask` is authored by admins on a maintenance screen. The person who discovers a jammed extruder is a member standing in front of it with a phone. UTA gives them `Service` — open a ticket against a device, with a history and a log.

**Scope.** A fault report from the machine page: what, optionally a photo, and it lands where maintenance already lives. Status back to the reporter. Staff triage it into the existing `MaintenanceTask` rather than into a parallel system.

⚠️ **One concept, not two.** A ticket that does not become a maintenance task is a second inbox nobody reads. **Reuse `MaintenanceTask`**; a member report is an origin, not a new entity.

⚠️ **Reporting a fault should be able to take the machine out of service** — that is S62's out-of-service action, triggered by staff, not by the reporter. **A member must not be able to disable a machine by filing a ticket.**

⚠️ **Photos are uploads**, and this repo already has an upload path with its own history (`creation-upload`, the branch name). Reuse it; do not add a second.

---

### S73 · Why is this person using this machine

**Why.** `Purpose` is a two-column table — id and title — attached to usage. For a university lab it is the reporting axis: how much of the laser's time was a course, research, or personal work. We have `Institution` on a person and nothing at all on a session.

**Scope.** An operator-editable list of purposes, an optional purpose on a usage session and/or a booking, and a breakdown in the existing statistics.

⚠️ **Optional, and it must stay optional.** A required dropdown between a member and a machine is a tax on every single use, and it will be answered at random within a week — which is worse than no data, because it looks like data.

⚠️ **It is operator vocabulary (S31) and it is per portal.** A course list is not the same in two deployments.

⚠️ **"Which course" is about a person, and aggregate reporting on people has a privacy question** the leaderboard already raised and never settled (see the 2026-07-10 audit). Answer it here or inherit it.

---

### S76 · Access modes — how you get on a machine

🔴 **Proposed 2026-08-01.** Written at the operator's request, out of the prototype: *"we should have machine statuses, maintenance, first come first served, by appointment, by reservation (appointment means a staff must be there)."*

**Why.** Every machine in this app is booked, because `Reservation` is the only way in. Real labs have at least three different answers to *"how do I get on this thing"*, and today the app can express one.

⚠️ **The operator's list mixes two different concepts, and separating them is the whole session.**

| | | set by | changes |
|---|---|---|---|
| **Mode** | *libre-service* · *sur réservation* · *sur rendez-vous* | the operator, on the machine | rarely |
| **State** | *libre* · *occupée* · *complet* · *hors service* · *labo fermé* | computed | minute to minute |

**`maintenance` is a state, not a mode.** Putting it in the mode list means an operator sets it once and it never clears itself — a machine stuck out of service until someone remembers the dropdown. Out-of-service is S62's action, with a reason and an expected return.

**Scope.** A mode on the resource, and the three consequences it has:

| Mode | The card's action | What the booking layer must do |
|---|---|---|
| **Libre-service** | none — "allez-y" | refuse a booking attempt; there is nothing to book |
| **Sur réservation** | *Réserver* | today's behaviour, unchanged |
| **Sur rendez-vous** | *Demander un créneau* | refuse unless a staff member is present for that slot |

⚠️ **"Sur rendez-vous" is the expensive one and it should be scoped separately or dropped.** It means the app has to know **when staff are on site**, and there is no staffing calendar — `UserAvailability` exists for bookable *people* (the `person_booking` feature), which is a related but not identical idea. **Decide whether appointment mode reuses `person_booking` or needs a staffing rota, before committing to it.** The other two modes are cheap; this one is a feature.

⚠️ **Mode changes what "available" means, so it changes the list card.** A libre-service machine is never "complet" — it has no bookings — and showing it a next-free-slot is nonsense. The four-state display built in the prototype assumes reservation mode throughout; **each mode needs its own state vocabulary**, or the card lies about two thirds of the lab.

⚠️ **It interacts with S70's queue, and the pairing is the interesting part.** A libre-service machine with a queue is exactly UTA's model — walk up, take a ticket, wait your turn. **Mode and queue are orthogonal**: mode says how you get on, queue says what happens when it is taken. Do not fold them into one enum.

⚠️ **`ReservationService::book()` must enforce the mode**, not just the UI. A libre-service machine that still accepts a POST to the booking endpoint is a machine with two conflicting truths.

⚠️ **Per resource, and it applies to places too** — a meeting room is far more likely to be *sur réservation* than a soldering iron. `Reservation` is already polymorphic; the mode column should be too, or the two halves of the calendar diverge.

⚠️ **Default must be today's behaviour.** Every existing machine becomes *sur réservation* on migration, or the deploy silently changes how the whole lab works.

**Verify.** All three modes on three machines in one lab. A libre-service machine offers no booking affordance *and* refuses a hand-made POST. An appointment machine refuses a slot with no staff. Existing machines behave exactly as they did before the migration.

---

### Where these go in the order

```
already shipped: S45 tokens ✅ · S46 public layout ✅ · S50 admin nav ✅ · S54 dead buttons ✅ · S55 single-feature ✅

    S58 detail pattern ─┬─> S62 staff surfaces ──> needs S63 first (audit)
    S59 list pattern ───┤                          needs a staff-only account
    S60 sentence forms ─┘
                        └─> everything Phase D builds, checked against the contract

    S63 notes + changelog ──> unblocks S62's override
    S61 three-click booking ──> card face blocked on Phase H S41 (the batched query)
    S64 guided chains · S65 member area · S66 holidays — independent, any time

approved 2026-08-01, still unscheduled:
    S67 packages ──> BLOCKED on S38/S48: the RFID endpoint fails open today,
                     so a permission model enforced only in the web flow is not enforced
    S68 lock + no-show ──> needs the reader-coverage count across the 11 machines
    S69 archive ──> after S58/S59, or it is fifteen hand-edits instead of two

from UTA, proposed only — and these are MODEL work, not UI work:
    S70 queue ──┬─> S71 storage + pickup   ⚠️ one workflow; neither ships alone
                └─> ⚠️ probably a PHASE, not a session. Scope before approving.
    S72 member-reported faults ──> folds into the existing MaintenanceTask
    S73 purpose on a session ──> independent, cheap, any time
```

⚠️ **S70–S73 are the odd ones out in Phase U and it is worth saying so.** Phase U is a design-system phase; these are **domain model** changes that happen to have screens. If they are approved they should probably become their own phase rather than inflating this one — the queue in particular. **They are written here because that is where the comparison landed, not because that is where they belong.**

**If only three of these are ever approved, take S58, S59 and S63** — the two shape sessions before Phase D adds ten screens to the pile, and the audit trail that S62 and every future override depend on.

⚠️ **All nine still sit behind the same open decision as the rest of Phase U: `importmap()` is called in zero templates, so Turbo and Stimulus run nowhere.** Every session above is written to be buildable *without* them — native `<details>`, `<select>`, full page loads. **If AssetMapper is turned on first, S58–S61 get materially better and materially cheaper** (inline edit, drawers, a real combobox, a confirm step without a page load). That decision is worth making before S58, not after.

---

