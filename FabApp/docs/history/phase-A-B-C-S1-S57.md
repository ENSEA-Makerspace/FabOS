## Phase A — Modularity

*This is the precondition for the whole goal. Do it in order.*

### S21 · Equipment becomes a resource module — ✅ shipped 2026-07-28

**Why.** Equipment was kernel. That single fact made an events-only or training-only deployment impossible: the equipment pages and an equipment-shaped calendar were unremovable. This session was deliberately first and deliberately narrow — no new concepts, just parity with how places already work.

**What landed.**
- `machines` is a module: `SiteFeatureRegistry`, admin label *Équipement (réservation)*, route gate, nav gating. The key stays `machines`; the operator-facing word is *equipment*.
- The route gate matches `app_machine…` as a prefix — every equipment route is named that way and nothing else is (`app_maintenance` diverges at the fourth letter) — plus `app_kiosk_machine` exactly.
- The calendar's equipment layer is conditional, exactly as the place layer was.
- **The calendar's visibility is derived.** `ModuleService::RESOURCE_MODULES` names the layers; `hasResourceLayer()` / Twig `has_resource_layer()` is the one question the route gate, header and footer all ask. With no layer on, `app_calendar*` 404s. Adding a third layer means one entry in that constant.
- `BookingReminderScanner` filters **per booking** rather than being switched off wholesale — it is the one scanner serving every layer at once, so equipment off must still leave room reminders going.
- Collateral swept: homepage featured-equipment block, the "book a slot" step card, search results, `/mes-reservations` calls to action, the calendar page's own equipment button, footer and header.

**Two shells found by the four-way boot, and fixed.**
- The **"Le Lab" nav group** rendered an empty dropdown under a heading that 404'd once all its children were gated off. Pre-existing, but only reachable now that equipment can leave that group. Its heading now follows whichever child page still exists, and the group drops entirely when none does.
- **`ModuleService::all()` was adopting rows for retired keys.** The `emails` row left behind when mail became kernel was still drawing a switch on the admin screen that controlled nothing. Unknown keys are now ignored.

**Deliberately left alone.** The RFID box endpoints (`/api/rfid/machines/…`) stay ungated — they are hardware, and *Physical access control* is its own capability later; gating them here would lock people out of a door as a side effect of a catalogue decision. The homepage stats tiles and the `how_it_works` cards are ungated for **every** module, not just this one; that is S23/S24's to fix uniformly rather than one module at a time. API routes have never been module-gated (`api_leaderboard`, `api_badges` likewise) — unchanged.

**Verified** on the live container by booting four ways — everything on, equipment-only, events-only, training-only — over HTTP: no 500s, every gated route **404s** rather than merely hiding, no empty menu shells, and the home page links to `/machines` and `/calendrier` exactly when it should. `/calendar/machine/{id}.ics` answers 403 without the feed token when enabled and 404 when not, which is the right precedence.

**Deployed** code-first with no migration, as planned.

---

### S22 · Untangle what is conflated — ✅ shipped 2026-07-28

**Why.** Rule two of the model. Until this was done, capabilities would have inherited today's conflation and the wrong things would have become switchable.

**The answer, and it is the thing to remember: `SiteFeatureRegistry`.** The word "module" was answering three questions at once, so the three are now named in code and rendered as groups on the admin screen:

| Layer | What a module of this kind owns | Modules |
|---|---|---|
| **resource** | a bookable kind — and whether bookings of it are accepted at all | `machines`, `places`, `person_booking` |
| **activity** | a feature domain, with its own pages and data | `events`, `formations`, `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans`, `maintenance` |
| **directory** | **a page and a menu entry, and nothing else** | `staff`, `trainers` |

**What landed.**
- **People and roles are kernel and are not switchable.** `ROLE_STAFF` / `ROLE_TRAINER` authorisation, role membership and the staff desk (pass issuing, ticket scanning) depend on no module. The directory group on the admin screen says so in words, because the operator is the person most likely to assume otherwise.
- `staff` and `trainers` are **directory surfaces**, matched **exactly** (`app_staff`, `app_trainers`) rather than by prefix.
- **`person_booking`** is a resource module of its own, independent of either directory. It gates every `app_person*` route, the "book" button in the directories, the availability link in the profile and the nav.
- **Enforcement moved to the chokepoint.** `ReservationService::book()` refuses a booking whose resource layer is off (`RESOURCE_KIND_UNAVAILABLE`, 404). Route gating alone was not enough: `/api/reservations` speaks the polymorphic payload directly and would have kept accepting bookings for a kind the deployment does not offer. the feature's `reservable` is the one map, read by both the chokepoint and the reminder scanner.
- **The gallery left the leaderboard's namespace** — `/creations*`, `app_creation*`, with permanent redirects from the three old public GET paths. Redirects are named under the gallery's own gate, so with it off they 404 rather than redirecting onto a page that then 404s. POST endpoints moved without redirects: they are form targets, never a URL anyone holds, and a 301 is not reliably re-POSTed.
- The `app_leaderboard_creation…`-before-`app_leaderboard` **ordering dependency is gone**.
- The leaderboard's `stats` array — a user count, a machine count and a published-creation count — **was never rendered by the template**. Deleted along with the two repository injections feeding it, so a page that ranks people no longer queries the equipment and project tables on every request.
- `has_resource_layer()` → **`has_calendar_layer()`**. With a resource layer that deliberately is not on the grid, the old name had become a lie about which question it answers.

**`emails` was already settled** before this session (`fe36c67`): mail is kernel with a settings-level pause switch. S21 finished the job by making `ModuleService::all()` ignore its leftover row.

**Deliberately not done.** `person_booking` is **not** in `CALENDAR_LAYERS`. People are booked from their own pages and no column is drawn for them, so listing it there would bring `/calendrier` back as an empty grid for a deployment that books only people — the failure S21 removed. Add it the day people appear on the grid.

**Verified** across five boots on the live container — everything on, directories off, person-booking off, gallery-only, leaderboard-only. With both directories off the staff desk still answers and people can still be booked; gallery and leaderboard each work with the other disabled; old gallery URLs 301 while the gallery is on and 404 when it is off; and the booking chokepoint refuses each kind exactly when its module is off, proven with a past-dated probe that cannot write a row.

---

### S23 · Introduce capabilities — ✅ shipped 2026-07-28, ⚠️ **superseded the same day**

> **Read this first.** The two-layer model below was **collapsed into site features** hours after it shipped (`b294326`). `src/Capability/` no longer exists; `SiteFeature` carries both the operator-facing name and the route-gating key. The record is kept because the *reasoning* still applies — especially why module state stayed authoritative — and because it explains why there are inert `capability_*` rows in `SITE_SETTING`. Everything below is history.

**Why.** The heart of the phase — what turns fourteen implementation checkboxes into a question a newcomer can answer.

**What landed.** `src/Capability/` — a `Capability` value object, a `CapabilityRegistry` holding the catalogue, and a `CapabilityService` owning state and derivation. `/admin/capabilities` is the screen (`/admin/modules` 301s to it); the sidebar entry is now *Fonctionnalités*.

#### Two departures from the plan, both deliberate

**1. Module state stays authoritative in `SITE_MODULE`.** The plan had capability state authoritative with module rows kept *only* as deviations. Rejected for two reasons:

- Every existing install already has an explicit row for **every** module — the old admin form wrote them all on save — so on the day this shipped, every module would have read as a deviation.
- Deriving module state on read leaves an Advanced-panel untick nowhere to live. That *is* the "I unticked it and it came back" bug the plan was guarding against.

So capabilities are the operator's **intent**, persisted alongside (as `SITE_SETTING` rows, hence no migration); toggling one **writes** module rows. A deviation is a *diff* between intent and reality — informative, not authoritative. There is exactly one source of truth for "is this module on". What the operator sees is identical either way: capabilities, an Advanced panel showing what they imply with the differences marked, and a reset action.

**2. Capability saves apply as a delta, not a wholesale overwrite.** Only the modules whose *implication* moved are rewritten. That is what makes "a deviation survives a capability change" true, while a capability that actually owns the module still wins — the operator's newer, explicit choice should.

#### Other decisions worth knowing

- **One switch per module concept.** The catalogue offered *Materials & stock* both as an add-on of *Book equipment* and as a capability of its own. Union semantics make that work, but the screen would show two switches mirroring each other. It is one capability; *Book equipment* says in words that it also feeds "what this machine accepts". (The machine detail page already degrades on `materialsEnabled`, so the separation was correct in the code.)
- **Team directories are one capability over two modules**, as the catalogue had it. Publishing only one of the two lists is real but rare — and it is exactly what the Advanced override is for, which gives the deviation machinery a genuine use case rather than a hypothetical.
- **Add-ons are capabilities with a `parent`.** Stored, derived and applied identically; nested and dimmed in the UI, and still submitted while dimmed, so a parent switched off and back on restores its add-ons as the operator left them. Keeping them one concept is what stops progressive disclosure leaking into the model. Only one exists today: *Suivi de maintenance* under *Book equipment*, default off.
- **`recommends` was not built.** Its only reader is S26's warning UI. A stored field nothing consults lies to the operator (principle 10) — S26 adds it together with the warnings.
- **No `reservable` field either.** The `resource` group already says a capability books something and the feature's `reservable` already maps modules to kinds; a third copy of that fact is how the three drift apart.
- the registry surfaces any module no capability claims — it would otherwise be unreachable from this screen. Shown in the Advanced panel rather than thrown, so a half-wired new module cannot take the admin screen down.
- **Empty still means "as before".** Until the screen is saved once, capability state is read *back* from the current module state: it opens describing the install rather than proposing to change it, and an install that never visits it behaves exactly as it did.

**The catalogue is currently 1:1 between capability and module, bar team directories.** The union is therefore real code with no overlapping case to exercise yet; the first genuine overlaps arrive with the LMS (S32+), where *Train people* and *Credentials & badges* both touch badge award. Worth knowing before assuming that criterion was tested against overlap — it was tested against the real registry, which has none.

**Verified** on the live container: each capability alone produces a coherent app (equipment, equipment+maintenance, events, training, training+credentials, directories, appointments, gallery); turning any one capability off leaves every module its neighbours still need running; and a deviation survives an unrelated change, yields to its owning capability, and clears on reset. The form was exercised over HTTP — all three actions plus a rejected CSRF token.

**Out of scope, as planned.** Per-capability settings sprawl. A capability is on or off; its internals stay in their own admin screens.

---

### S24 · Menus assemble themselves — ✅ shipped 2026-07-28

**What landed.** `src/Nav/NavBuilder.php` returns the header as an ordered list of links and groups, already filtered; `_header` and `_footer` render that list. Fifteen hand-written `feature_enabled()` checks and a nested-ternary landing-page chain are gone.

The three rules it now states **once** rather than fifteen times:

1. **A group with no visible children is not rendered at all** — not as an empty dropdown, and not as a heading linking to a 404.
2. **A group's own link follows its children.** Where the natural landing page is gone, the heading points at the first child still there.
3. **Visibility is presentation, never permission.** Hiding an entry hides a link; the route gate and the firewall decide what exists. Nothing in the nav may be the only thing between a user and a page.

⚠️ **The calendar group keeps its own landing rule** rather than inheriting rule 2: it goes to `/calendrier` whenever anything is drawn on the grid, even when the first child listed (equipment) is what is off. That was the one place the old chain was *not* first-visible-child, and a naive rewrite would have lost it silently.

**Verified as the plan asks.** The rendered nav was snapshotted **before** the change across four feature combinations — everything on, events-only, equipment-only, training-only — and the header and footer link lists come back **byte-identical** after. The refactor changes no output, which was the whole claim.

**Still out of scope, deliberately.** Admin-editable menus, custom labels, reordering, external links. The derived version may well be enough permanently.

**Original scope, for reference.**

**Why.** Nav is hardcoded in `_header.html.twig` as two curated dropdowns with ~15 scattered `module_enabled()` checks. Every new module means hand-editing the header, and an all-disabled group still renders its wrapper.

**Scope.**
- Grow module entries to carry **nav metadata**: group, order, route, i18n label key, required role.
- Header and footer render from the registry: an entry shows if its module is enabled and the viewer's role allows; **a group with no visible children is not rendered at all.**
- Keep today's grouping and labels. Items stay in the submenus they are in now.

**Out of scope — deliberately.** Admin-editable menus, custom labels, reordering, external links. Explicitly deferred by the operator; the derived version may be enough permanently.

**Verify.** With everything enabled, snapshot the rendered nav before and after — it should be **byte-identical**, proving the refactor changed no output. Then each capability off in turn: no empty dropdowns, no dead links.

---

### S25 · First-run setup — ✅ **complete** (health panel + wizard; sample data dropped)

**✅ Done — the setup health panel** (`/admin/setup`, sidebar *État de l'installation*). `SetupHealth` lists what is not configured **and what each gap costs**, because the whole point is that these gaps are invisible: mail that is never sent and never errors, a message with no link rather than a broken one, a ticket with no QR. Severity is by consequence, not rarity — **blocking** (people will try something that does not work) · **degraded** (works, quietly does less) · **info** (a deliberate choice worth confirming). Mail *paused* is a separate check from mail *never configured*. The panel is **read-only and links out**: every fix belongs on the screen that owns the setting, and a second place to edit the same thing is how they drift. Unresolved items sort above resolved ones, worst first. Verified healthy / all-features-off / events-only on the live container.

**✅ The wizard shipped 2026-07-31**, at `/admin/wizard`, sidebar *Configuration initiale*. Both of the questions this entry said to settle first are settled, and both the conservative way.

**"Unconfigured" is an explicit flag, never an inference.** `setup_completed_at` is written once — by finishing or by skipping — and nothing else moves it. Deciding from whether settings happen to be empty is wrong in both directions: an operator deliberately running without a public URL would be nagged forever, and one who filled a single field would be declared finished.

⚠️ **A missing flag alone is not enough to call an install fresh, and this is the part that protects you.** Every install predating this code has no flag, so the flag by itself would show a first-run wizard to a lab that has run for a year. `FirstRun::isFresh()` also requires the install to *look* empty, counting operator-created content (machines, events, training, spaces, lab pages) rather than users — SSO or an import can create accounts on a site nobody has configured. It fails safe toward "already set up": if the tables cannot be read, it assumes there is content.

**Nothing redirects.** This entry warned that a global redirect-to-wizard interceptor is a real hazard on a live install; it is, because every route that forwards is a route that can strand someone. The wizard is reached from a dashboard card (shown only when fresh) and the sidebar — so the worst a bug in it can do is render a page badly.

It asks **only for settings that already have readers**: organisation and venue (S31), public URL, address, locale. Features get a *link*, not a copy of their screen — two places to switch the same thing on is how they drift apart. Skipping is a real answer and also sets the flag.

**Verified on the live container:** reports *already set up*, the dashboard card does not render, and setting then clearing the flag flips `completedAt` while `isFresh()` stays false throughout. ⚠️ The genuinely-empty case is **not** exercised — proving it would mean emptying five tables on a production database.

**❌ Sample data: dropped 2026-07-31, at the operator's request.** It was the last item here and it is not coming back unless asked for. Worth recording *why* it is a fair thing to drop rather than a gap: the health panel already teaches what a populated install would have demonstrated, and the feature is unusually expensive for what it gives — it writes production rows, and a believable demo set has to be maintained in step with every schema change forever. **If it is ever revived, the wipe is the hard half, not the seeding.**

**Original scope, for reference.**

**Why.** Now nearly free, because the wizard just asks for capabilities — the same concept the admin manages forever after, not a separate preset system.

**Scope.**
- A first-run wizard while the install is unconfigured: organisation name, public URL, address, timezone/locale, then **capabilities**.
- A **setup health panel** listing what is not yet configured and why it matters ("no sender account → no mail goes out", "no public URL → tickets carry no QR"). Several existing features degrade silently *by design*; this is where that becomes discoverable.
- ~~Optional **clearly-labelled sample data**, idempotent, with a one-click wipe.~~ *(dropped 2026-07-31)*

**Verify.** Each capability combination produces a coherent app. Wipe returns the install to clean.

---

### S26 · Feature dependencies, stated honestly — ✅ shipped 2026-07-28

**What landed.** `SiteFeature::$recommends` (the field S23 deliberately deferred until it had a reader) plus `FeatureAdvice`, which turns it into warnings on the feature screen: a feature that is **on**, leaning on a companion that is **off**. A recommendation from a feature that is itself off raises nothing — nobody is relying on it.

**Warn, never block.** Every combination here is a legitimate way to run a place, so the card says what is being given up and leaves the choice alone.

**⚠️ One relationship contradicts what this plan assumed, and the plan was wrong.** The worked example below read *"Machine workshop uses Training & badges for certification gating — without it, anyone can book any machine."* That is **not** what happens. `MachineQualificationService` has **no feature check at all** — it reads badge records directly — so switching badges off does not re-open equipment to anyone. The real consequence is harder to diagnose: **the gate keeps refusing while the pages that would explain the refusal disappear**, so a member denied a slot has no way to see what they are missing. The shipped warning says that.

The other three, each verified in the source:

| Feature on | Companion off | What is actually lost |
|---|---|---|
| `machines` | `materials` | Every equipment page loses its "accepted materials" section |
| `person_booking` | `staff` | The "réserver" button exists **only** on the directory pages, so booking still works but nothing links to it |
| `badges` | `formations` | Nothing awards credentials automatically; each is issued by hand from the admin |

**Verified** on the live container: each declared relationship produces its warning and only when it should; no warnings with everything on; none with everything off; nothing hard-fails.

**Original scope, for reference.**

**Why.** Capabilities lean on each other. Cert-gating needs badges. Training awards badges. Events want mail.

**Scope.** Capabilities declare *recommended* companions; the capability screen warns clearly when a choice will lame something ("Machine workshop uses Training & badges for certification gating — without it, anyone can book any machine"). **Warn, don't block** — an ungated community workshop is a legitimate choice.

**Verify.** Each declared relationship produces its warning; nothing hard-fails.

---

## Phase B — Portals

*Multi-tenant, one install. The operator wants this working before revisiting event polish.*

### S27 · Portal admin CRUD + branding — ✅ shipped 2026-07-30

**What landed.** `/admin/portals` (list + create + delete) and `/admin/portals/{id}` (identity, features, overrides), sidebar *Portails*. **No migration** — `PORTAL` has existed since S12 and both config tables have carried a `portalId` just as long; the whole mechanism was simply unreachable because nothing could create a portal.

**Per-portal features are tri-state, and that is the load-bearing decision.** A portal says **on**, **off**, or **nothing at all**, and nothing-at-all is the default. Plain checkboxes would have written an explicit row for every feature the first time anyone pressed Save — silently freezing that portal against every later change to the site-wide switches, while looking identical on screen. `null` means inherit, and choosing inherit *deletes* the row rather than storing a value.

**The default portal has no override editor, by construction.** It is not the important portal, it **is** the global scope: `portalId = 0` means "the default portal's value", which is what every other portal falls back to. Its page says so and links to the ordinary settings screens. `isDefault` is not editable anywhere — moving the flag would re-point every existing global row at a different front door without moving the rows, and that needs a migration, not a checkbox.

⚠️ **Only settings read *during a request* can be overridden per portal, and this is structural.** A portal is resolved from the request's hostname. Anything read outside a request resolves at the global scope whatever a portal's row says — so **mail sender identity and `public_base_url` are deliberately absent from the catalogue**: they are read by the queue worker, which handles no request and has no hostname, so a per-portal value would be stored, shown in the admin, and never once used. The screen says why rather than offering a setting the app would ignore. **Check this before adding a field to `PortalOverrides::FIELDS`.**

Two smaller traps, both encoded in `PortalRepository`:

- **A blank hostname stores as `NULL`, never `''`.** The column is UNIQUE, and MySQL treats NULLs as all distinct but empty strings as equal — so a second portal saved without a hostname would be refused as a duplicate of the first, over a field nobody touched.
- **Deleting a portal deletes its scoped config rows.** No FK by design, so no cascade — the same rule reservations live under. `id` is AUTO_INCREMENT, so orphaned rows would eventually be handed to a new portal, which would look like a haunting.

The accent colour is **validated on read as well as on save**, because it is interpolated into a `<style>` block where HTML escaping does nothing — inside CSS a `}` ends the rule. The logo setting refuses path separators for the same class of reason.

**Verified on the live container** against a throwaway portal, since deleted: 15 service-level checks (slug and hostname normalisation, both clash refusals, the NULL-hostname trap, all three feature states, inherit removing its row, blanking removing an override, the default portal refusing deletion, delete taking its 3 scoped rows with it) — and end to end, a request on the portal's hostname **404s `/machines` while the main site serves it 200**, drops it from the nav, swaps the logo and emits the accent colour, with the main site's markup unchanged.

**⬜ Left for later, deliberately.** Add-ons are not per-portal settable (they follow their parent, and the screen only offers root features). Logo is a filename in `public/images/`, not an upload — an upload path belongs with the lab-page photo convention. And a portal that overrides nothing still renders the site's own hero, which is where **S28** picks up.

---

### S28 · Per-portal home page — ✅ redirect shipped 2026-07-30, blocks still to do

**Why.** The driving case: a tenant who only wants events should have the events page as their front door.

**Scope.**
- One setting: home is **either** the block homepage (default) **or** one enabled capability's landing page, from an allow-list of route names. **Never a free-text path** — that is an open-redirect and a 500 generator.
- **302 redirect**, not an internal forward: no controller duplication, and the URL bar stays honest.
- Portal-scope the existing `HomepageSectionVisibility` blocks, which are *already* ordered and role-gated — they just aren't per-portal yet.

**✅ What landed.** `SiteFeature::$landingRoute` — each feature's canonical **public** page — plus `PortalHome`, and a `portal_home_route` field in `PortalOverrides`. A portal opens on one of its own features; blank means the ordinary block homepage. **No migration.**

- **A route name chosen from a list, never a path.** Free text would be an open redirect, and a 500 on the front page the first time somebody typed one wrong. The list comes from the enabled features, so **`app_home` is not in it** and a redirect loop cannot be configured. `person_booking` has no landing route on purpose: its page needs a login.
- ⚠️ **The list is rebuilt on every request, and that is the load-bearing part.** A portal pointing at events whose `events` feature is later switched off would otherwise 302 its own front page onto a 404 — the gate doing its job would make the site unreachable at the root. Re-checking means the worst case is the ordinary homepage coming back. **Verified live:** with events off for that portal, `/events` 404s and `/` answers 200.
- **302, and it must stay 302.** This redirect is changeable from a form. A 301 would be cached by every browser that ever saw it and would keep sending people to the old page long after the setting changed, with nothing left server-side to explain why.

**Prompted by a real portal created through the UI mid-session: overrides are now validated on save**, all-or-nothing, before anything is written. The accent colour was already validated on read — it lands in a `<style>` block where HTML escaping buys nothing — but accepting on save and refusing on read is the worst combination available: stored, shown back as though it applied, silently doing nothing. That is precisely what `#1D4ED8.` (trailing dot) did. Refusals now carry a sentence naming the format. Verified against a trailing-dot colour, a CSS injection (`#fff} body{display:none`), a `../../.env` logo, an unknown route and a bad locale — each refused, nothing written.

**⬜ Still to do — and it needs a migration, so read §7 first.** Portal-scoping the `HomepageSectionVisibility` blocks. ⚠️ **That is an ORM entity**, so adding a `portalId` column is the dangerous shape: entity mapping has no fail-safe degradation, and the code must ship *after* the migration or every page touching the table 500s. It also serves the weaker half of the case — the driving one is answered by the redirect above. Worth doing only if a portal genuinely needs its own *hero content* rather than its own front door.

---

## Phase C — Usability & consistency

### S29 · One admin layout — ✅ **complete (extraction, visual pass, chrome, panel padding)**

**✅ Done.** `public/css/admin.css` owns the admin chrome. **49 rules were duplicated across the 55 admin templates — 653 lines** — and that duplication is the mechanism behind every bug listed below: a fix landed on whichever page someone was editing and nowhere else.

- Rules moved **byte-for-byte**, and the stylesheet is linked **before** each page's inline block, so light mode renders exactly as before and a page that genuinely needs to override something still can. What remains inline is only what is page-specific.
- **Corrections are at the end of the file, not folded into the move**, so they read as deliberate: radios/checkboxes opt out of `.form-field input { width: 100% }` · `.btn-action` (12 templates), `.btn-small` (14) and `.form-field.check` were used everywhere and defined nowhere · content after `form_end()` gets the gutter `.admin-edit-panel` never had · a dark-theme block, overriding rather than variabilising so light mode stays byte-identical.
- **Class audit re-run:** 6 undefined classes across all 55 pages, down from the set above. The remaining five are single-page and cosmetic, and are named in a comment in `admin.css` so the next audit knows they were judged, not missed.
- Checked: all **62 admin paths still answer**, and every page links the stylesheet.

**✅ The visual pass is done (2026-07-31), and the agent could do it after all.** The plan said this one needed eyes on screens. `app:render` plus the fact that the stylesheets are *public* turned out to be enough: render each admin page to HTML, serve it locally against the real CSS, drive a browser over it in both themes, and measure contrast in the DOM rather than judging it by eye. Re-checked after fixing — **7 pages × 2 themes, all clean**.

What it found, none of which was visible in the markup:

- **108 hardcoded light backgrounds across 40 of the 58 admin templates.** The shared chrome was already safe — `html[data-theme="dark"] .admin-panel` outranks a page's inline `.admin-panel` on specificity, whatever the source order — but each page's *own* classes were never covered. `/admin/emails` rendered its counters at **1.06:1**. Fixed in `admin.css`, grouped by what the surface *is* (panel / recessed fill / form control / meaningful tint), so a new page picks a line instead of inventing another hex.
- **Every status colour failed in dark**: 5.0–6.7:1 on white, 2.2–3.0:1 on the dark panel. The colours carrying the meaning were the unreadable ones, on the screens whose job is to flag problems. Three pages had each invented the same four hues inline; they now share `.admin-status-*`, dark values measured at 7.1–10.7:1.
- **Two failures in *light* mode too**, found while checking dark: the solid status marks put white text on `#16a34a` (3.3:1) and `#d97706` (3.2:1).

⚠️ **Two traps worth keeping.** (1) The status classes were first called `.status-ok` / `.status-info` — names **style.css already uses** for the public `/status` page. It loads first, so the new colour won while their near-white background stayed, and the dark rule then put pale green on near-white. In a 113 KB global stylesheet, assume every generic utility name is taken; this is invisible in the markup and in either file alone. (2) **`main.js` applies the OS theme at runtime**, so a "light" test render is only light if you strip it and pin `data-theme` — the first audit pass reported light-mode results that were actually dark.

**✅ The skeletons are converged too (2026-07-31).** The plan said three; measuring found **six** — the two documented extras `.admin-rfid-*` / `.admin-user-*` (3 pages) plus `.admin-content-grid`, its own two-column shell on 6 more, which had never been named anywhere. Banner, shell and rhythm are now defined once in `admin.css` and every existing class name resolves to it, rather than rewriting 55 templates.

**Verified by measuring, not by eye**: across one page of every family, header background, header padding, sidebar column and gap all agree, and the **content column is 938px on every page** — the number that actually matters. Two families keep different margin/padding because their width and gutter come from an outer `.admin-main`; identical content width is the proof that this is structure rather than drift.

⚠️ **One deletion was wrong, and only the measurement caught it.** Six pages nest `.admin-main-content` *inside* `.admin-content-grid`, where it is the content column and not the shell. Each carried an inline `display: flex` saying so; removing those as "duplicate chrome" gave them a second, empty 260px sidebar. Now one descendant rule states the structure instead of six copies. **Treat an inline override as deliberate until measured otherwise.**

**✅ And the panel padding is unified too (2026-07-31) — S29 is done.** This was logged as a markup job needing a `.admin-panel-body`; measuring 21 panels across 14 pages showed the premise was wrong. `.admin-panel` was not two semantics — it was **three values with no rule behind them** (0, 24px, 32px). Pages whose panel opens with `.admin-panel-header` had 32px *and* 0px, and the header brings its own 20px/24px regardless, so the 32px was padding wrapped around already-padded content. Drift, not design.

The panel is now a frame everywhere and its children take the gutter — the identical idiom `.admin-edit-panel` already used, needing no wrapper element. `.admin-panel-header` and the table wrappers are exempt: they pad themselves, or want to bleed to the edge. It was checked first that every other child had **zero** horizontal padding of its own, so nothing doubles up.

Fallout, both found by measuring: nine pages carried a hand-written `<div style="padding: 0 24px">` compensating for an unpadded panel — redundant now, removed; and two bordered cards (`.check`, `.reader-help`) had their *internal* 16px overridden to 24px by the gutter, restored at page level where a card's own metrics belong.

**Verified before and after:** all 21 panels report 0px, every non-exempt child reports the 24px gutter, tables still bleed full width, both themes correct.

⚠️ **The reusable lesson from all three S29 passes: read the templates to find candidates, but decide from measurement.** Reading said three skeletons (there were six), said `.admin-panel` had two semantics (it had three arbitrary values), and made six deliberate `display: flex` overrides look like duplicate chrome. Every one of those was caught by printing computed styles, and none of them by looking at the source.

**Original scope, for reference.**

**Why.** Three incompatible admin skeletons exist, each with copy-pasted inline CSS across ~53 templates. Direct cause of several visible bugs: panels with no padding, buttons styled nowhere, radios inflated to full width, 37 pages rendering the sidebar as a bare link list.

**Scope.** One shared admin base template owning chrome, panel padding, form controls, buttons and flash styling. Migrate every admin page to it and **delete the per-page copies**. Re-run the class audit (cross-reference every `class="…"` against defined selectors).

**Verify.** Visual pass over every admin page in **both themes** — the one session that genuinely needs eyes on screens, not status codes.

---

### S30 · Admin actions where the content is — ✅ shipped 2026-07-31

**What landed.** One macro, `site/_admin_inline.html.twig`, and one affordance, on six detail pages: events (edit + registrations), equipment, spaces, training (edit + content), lab pages, badges. Global configuration stays in the panel; this is only ever *this record, here*. No migration.

**The button is a shortcut, never the permission.** Every target sits behind `/admin`, gated by `access_control` — so what makes it work is the firewall on each request, not the chip's visibility. **Both halves were verified rather than assumed:** signed in, the six pages render 8 chips; anonymous, they render **zero**; and every real href scraped out of the rendered pages returns **302** when fetched without a session.

⚠️ **`role` must mirror the route's actual access rule, and this app has no role hierarchy — `ROLE_ADMIN` does not imply `ROLE_STAFF`.** The registrations chip was gated `ROLE_STAFF` on the reasoning that it is a staff-desk job; its route lives under `/admin`, so the effect was to hide it from the only people who can use it. Not a hole — the firewall was never in question — but the wrong kind of wrong, and it surfaced only because the chip count came back 1 instead of 2. Check the path prefix and the matching `access_control` line.

⚠️ **The chip carries its own backdrop.** It sits beside a title, and a title can be overlaid on a photo hero (events, equipment). Transparent looked right on a white panel and was invisible on the event page — visible only in a screenshot, not in any markup check.

**⬜ Not done.** List pages have no "add" chip yet (the panel's own new-buttons already cover it), and there is no inline check-in — the ticket scanner is its own `/staff` screen and moving it inline needs the staff-vs-admin split thought through, given the missing role hierarchy above.

---

### S31 · Neutral vocabulary + organisation identity — ✅ shipped 2026-07-31

**What landed.** Two settings on `/admin/settings` — `org_name` (the school, association or company) and `venue_label` (what the place is called) — kept **separate on purpose**, because "the ENSEA FabLab" is two different nouns and a deployment can need to change one without the other. No migration.

**158 occurrences across five catalogs** became `%venue%` (133) and `%org%` (25), plus page titles and kiosk headers via `venue_label()` / `org_name()` in Twig.

**`VocabularyTranslator` decorates the translator, so no call site passes anything.** Doing it per-`|trans` would have meant editing every call in the app and, worse, would have failed *silently* the first time someone forgot — printing a raw `%venue%` onto a public page. Decoration makes it structural: the parameters are always present for templates, controllers, mail and the console alike, and Symfony ignores parameters a string has no placeholder for, so the other ~800 keys are untouched.

⚠️ **Resolution is lazy and fail-safe, and it must stay that way.** This service sits in front of *all* text, so it is reachable during cache warmup, from the console and from the mail worker — none of which are guaranteed a database. Reading settings in the constructor would make a config store a hard dependency of rendering any string at all. On failure it returns the old hardcoded words.

**Defaults are this install's existing wording, not neutral words.** The point is to make the vocabulary editable, not to rename somebody's site out from under them — an operator who never opens the screen sees no change.

**Verified:** with defaults, "FabLab" appears exactly where it did and **no page leaks a raw placeholder**; setting the pair to "Makerspace Lyon" / "atelier" changes the wording across the public site; clearing them restores it.

⚠️ **Concatenating into a `??` fallback needs parentheses.** `description ?? 'text' ~ venue_label()` can bind as `(description ?? 'text') ~ venue_label()`, appending boilerplate to a record that *has* a description. Twig 3.15 deprecates the ambiguity; here it was a bug in waiting.

**Known remaining:** five `@ensea.fr` email placeholders. Those are a domain, not a name — inventing a setting to hold one example string is worse than leaving it.

---

