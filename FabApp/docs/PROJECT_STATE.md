# FabOS — project state & handover

**Last updated:** 2026-07-30 (S21–S26, S29 extraction, S37) · **Branch:** `fix/creation-upload-duration-and-image` · **Live:** https://fabos.dstei.fr, running `APP_ENV=prod` since 2026-07-30

This file exists so that a person — or an AI agent — can pick up this codebase cold and be productive without re-deriving the architecture or re-discovering the traps. Read it before touching anything. It is deliberately opinionated about *why* things are the way they are, because most of the mistakes available here are ones that look reasonable until they cost you a production outage.

---

## 1. What FabOS is

A management platform for shared workshops, originally a fablab tool. **Symfony 8.1 / PHP 8.4 / MariaDB**, Twig server-rendered, no SPA. Repo layout is a monorepo: the app lives in `FabApp/`; siblings `Boitier/`, `Stage/`, `Template-html/` are unrelated or historical.

### The central architectural idea: it is a calendar-based app

The calendar is the **spine**, not a feature. Anything bookable is a *resource layer* that projects onto one shared calendar. This is not aspirational — it is what the code already does, via a deliberately polymorphic reservation model (sessions S8–S10):

- `RESERVATION` addresses its target **only** as `(reservableType, reservableId)`, both NOT NULL. There are no `machineId` / `placeId` columns any more.
- `src/Reservation/` owns the model: `ReservableType` enum (`machine | place | user`), `ReservableRef` VO, `ReservableResolver` (batch `warm()` — **mandatory on listing pages**, there is no join to lean on), `ReservationService`, `BookingResult`.
- `reservableLabel` snapshots the resource's name at booking time. No FK means no cascade, which is intentional: deleted-resource history stays readable and admin search is a plain `LIKE`.
- **No cascade means you must clean up explicitly.** `deletePlace()` calls `cancelUpcomingForReservable()` itself. There is still **no machine-delete route**; whoever adds one must do the same or silently orphan bookings.

**To add a bookable resource kind:** one `ReservableType` case, one `ReservableResolver` branch, one arm in `checkAccess()` if it needs gating, and one entry in the two calendar builders (§4). That is the whole point of those three sessions.

---

## 2. Booking permission: three independent layers

They meet in `ReservationService::book()` and they are **kept apart on purpose**. Do not merge them.

| Layer | Question | Where | Failure mode if merged |
|---|---|---|---|
| **Certification** | *May you touch this at all?* (safety) | `checkAccess()` → `MachineQualificationService` | A lab loosening quotas would loosen safety |
| **Quotas** | *How much / how far ahead?* (fairness) | `BookingPolicyService::check()` | — |
| **Access passes** | Staff-issued exemption **from quotas only** | `AccessPassRepository` + `$passApplies` short-circuit | A pass would become a safety bypass |

Consequences worth internalising:

- **Cert-gating was already built** long before it appeared as a roadmap "todo". `MachineQualificationService::getStatus()` reads the training mode (free / theory / theory+physical), required badges via `MachineBadge`, held badges via `UtilisateurBadge`. Admins bypass it. The calendar surfaces it through `SiteController::buildCalendarBookingAccess()`. The RFID door path is the separate `MachineAccessService::authorize()`. **Don't rebuild any of this.**
- **There is deliberately no cert-bypass column** in `ACCESS_PASS`, and a comment in the migration says so. Don't add one. A pass is a convenience object that gets handed around and extended; safety bypass needs its own explicitly-issued, supervision-scoped record.
- **Quota refusals are 409, not 403.** The booking isn't forbidden in principle, it conflicts with what you already hold — cancelling something makes it succeed.
- **Quota checks are ordered coarsest-constraint-first.** Min-notice/horizon before slot alignment: "you can't book this soon at all" must beat "round it to the half hour", or fixing the alignment of an unbookable slot just earns a second refusal.

---

## 3. Config, and the fail-open / fail-closed rule

Config-adjacent stores are **raw DBAL, not entities**, and fail-safe on reads. The direction of failure is chosen per store and it matters:

| Store | On read error | Why |
|---|---|---|
| `BookingPolicyRepository` | **open** (= no limit) | Refusing every booking over a config problem takes the lab offline |
| `AccessPassRepository` | **closed** (= no pass) | The failure would otherwise *grant* an exemption nobody issued |
| `NotificationPreferences` | **open** (= send it) | `ReminderLog` claims *before* sending, so failing closed would burn the claim and drop the reminder permanently and silently |
| `ReminderLog::claim()` | **closed** (= don't send) | Forgetting costs one reminder; guessing costs a mail loop |

**"Empty table is a complete, valid configuration"** is the house style. `BOOKING_POLICY` seeds no rows; every limit column is nullable and null means no limit. Same for access passes and reminder toggles (all ship **off**). An all-blank save *deletes* the row rather than storing nulls, so "configured" and "actually constrains something" can never drift apart.

**Portal scoping.** `PortalContext::scopeId()` returns the current portal's id or `0` for global. Settings/modules read most-specific-first and fall back to global. ⚠️ The scope column is `portalId INT NOT NULL DEFAULT 0` with **0 = global**, *not* a nullable `portal_id` — it had to join the PRIMARY KEY of `SITE_SETTING`/`SITE_MODULE` to keep the `ON DUPLICATE KEY UPDATE` upserts working, and PK columns can't be NULL. The seeded `default` portal **owns no rows — it *is* the global scope**.

---

## 4. Modules

`SiteFeatureRegistry` is the list of keys; `SITE_MODULE` holds the on/off state per portal.

### A feature is one of three things (S22)

Read this before adding one. The word "module" used to answer three questions at once, and the conflation is what let "hide the team page" look like "turn off the team".

| Layer | Owns | Modules |
|---|---|---|
| **resource** | a bookable kind, **and whether bookings of it are accepted at all** | `machines`, `places`, `person_booking` |
| **activity** | a feature domain, with its own pages and data | `events`, `formations`, `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans`, `maintenance` |
| **directory** | **a page and a menu entry, and nothing else** | `staff`, `trainers` |

**Kernel is not on this list and never becomes a toggle:** users, roles and authorisation, the staff desk, auth, profiles, settings, portals, mail transport, the booking and calendar engine.

The admin screen renders these groups, so the operator is told what kind of switch they are looking at. Anything in `MODULES` but missing from `LAYERS` lands in an "other" group rather than disappearing — an invisible module would be stuck at whatever it currently is.

### Site features — one concept (S23, collapsed 2026-07-28)

`src/Feature/` is the whole story. A **`SiteFeature`** carries the operator-facing label and description *and* the key that gates routes — they are the same thing. `SiteFeatureRegistry` is the catalogue and the single source of the metadata; `SiteFeatureService` owns state.

- `group` — `resource` (owns a `ReservableType`; decides whether bookings of that kind are accepted at all) · `activity` (a feature domain with pages and data) · `directory` (**a page and a menu entry, nothing else** — the people, their roles and their authorisation are kernel).
- `parent` — makes it an **add-on**: nested in the UI, and **forced off by the service whenever its parent is off**, so that is true everywhere, not just on the screen that asked.
- `calendarLayer` — draws a column on the shared grid. ⚠️ Narrower than the resource group: `person_booking` is a bookable kind that deliberately is *not* a calendar layer, or `/calendrier` would return as an empty grid for an appointments-only deployment.
- `reservable` — the `ReservableType` it owns; read by `ReservationService::book()` and the booking reminder scanner.

**Storage is still `SITE_MODULE` with the old keys** — renaming would need a migration for no gain. Twig: `feature_enabled('x')` and `has_calendar_layer()`. Screen: `/admin/features`; `/admin/modules` and `/admin/capabilities` 301 to it.

⚠️ **S23 briefly had a second layer** ("capabilities" over "modules", with derivation and deviations). It was removed the same day — the catalogue was one-to-one, so it was a second vocabulary for the same choices. Leftover `capability_*` rows in `SITE_SETTING` are inert. Don't reintroduce the split unless the catalogue genuinely stops being one-to-one.

**Adding a feature:** one entry in `SiteFeatureRegistry::build()`, one arm in `FeatureAccessSubscriber`, nav gating with `feature_enabled()`, i18n in all five catalogs, fail-safe repository methods.

### What else reads the feature registry

Four things sit on top of it. All were added 2026-07-28; none needs a migration.

| Where | What it does | Watch |
|---|---|---|
| `src/Nav/NavBuilder.php` | Builds the header and footer from what is switched on. `_header`/`_footer` just render the list. | A group with no visible children is never emitted, so an empty dropdown cannot render. The **calendar group keeps its own landing rule** (`/calendrier` whenever anything is on the grid) rather than "first visible child" — don't "simplify" that away. |
| `src/Feature/SetupHealth.php` | `/admin/setup` — what is not configured **and what each gap costs**. Severity by consequence: blocking / degraded / info. | Read-only by design. Every fix links out to the screen that owns the setting; a second place to edit the same thing is how they drift. |
| `src/Feature/FeatureAdvice.php` | Warnings on the feature screen: a feature that is **on** leaning on a companion that is **off**. Declared as `SiteFeature::$recommends`. | **Warn, never block.** ⚠️ Verify any claimed coupling against the source before declaring it — the plan's own example was wrong (see below). |
| `public/css/admin.css` | The admin chrome, once. 49 rules were duplicated across the 55 admin templates (653 lines). | Linked **before** each page's inline block, so page-specific rules still win. Light mode is byte-identical to before; the dark-theme block at the end **has never been looked at**. |
| `src/Nav/NavBuilder::safeDestinations()` | Somewhere to go from an error page. Reads the footer, so nothing switched off is ever offered. | Added S37, after the 404 page was found offering equipment and training **by name** — on an install with those off, every escape route from the 404 was itself a 404. |
| `src/Http/MissingPageLog` + `MissingPageSubscriber` | `/admin/missing-pages` — the URLs people ask for and don't get, tagged with **why**. | A switched-off feature 404ing is not a bug, it is demand; that is the distinction the screen exists to draw. `FeatureAccessSubscriber::refuse()` writes the feature key onto the request so the reason survives the exception. ⚠️ **Priority −100, and it must stay above −128** — see below. |

⚠️ **A `kernel.exception` listener below −128 never runs.** `ExceptionEvent` extends `RequestEvent`, and `RequestEvent::setResponse()` calls `stopPropagation()`, so the instant Symfony's `ErrorListener` (−128) attaches the error page every lower-priority listener is skipped. The missing-page subscriber was written at −256 precisely to stay out of the response's way, and it recorded nothing at all — while `debug:event-dispatcher` listed it as registered and correct. **Anything that only observes an exception still has to run *before* the error page exists.**

⚠️ **`MachineQualificationService` has no feature check.** Turning `badges` off does **not** re-open equipment — the gate keeps reading badge records and refusing, while the pages that would explain the refusal disappear. The roadmap originally claimed the opposite. Check couplings in the code, not in the plan.

✅ **The live site ran `APP_ENV=dev` until 2026-07-30** — a public 404 returned Symfony's debug exception page with routing internals in an HTML comment, and the profiler answered to anyone. `APP_ENV=prod` now sits in `.env.local`, overriding `.env` (left untouched; backup at `.env.local.bak-20260730`). **A 404 is a designed outcome here, not an incident** — the gating model uses it — so the error page was the thing worth getting right, which is S37.

### The module scaffold (copy it)

1. add a `SiteFeature` to `SiteFeatureRegistry::build()` (label, description, group)
2. — its label lives on the feature itself; there is no separate labels map any more
3. gate routes with a name arm in `FeatureAccessSubscriber`
4. gate nav in `_header` + `_footer` with `feature_enabled('x')`
5. i18n `nav.x` + `x.*` in **all five** catalogs
6. repository methods **fail-safe** (try/catch → `[]`) so a new-table module can deploy safely *before* its migration
7. **a resource feature also needs** its `reservable` set — that is what makes `ReservationService::book()` refuse the kind, and what keeps the booking reminder scanner quiet about it

⚠️ **`FeatureAccessSubscriber` gates by route *name*, so the map is only as good as the naming.** A prefix is a promise that no unrelated feature will ever be named under it. It bit us once: `app_staff*` matched both the staff *directory* and the staff *desk* (`app_staff_access_passes`, `app_staff_event_scan`), so turning the directory off would have 404'd the ticket scanner. Directories now match **exactly** (`app_staff`, `app_trainers`). **When you add routes, check they don't accidentally inherit another module's gate.**

⚠️ **Route gating alone is not enforcement for a write path.** `/api/reservations` takes the polymorphic payload directly, so a disabled resource layer would have kept accepting bookings. The gate lives at `ReservationService::book()` — the chokepoint every path already goes through — and refuses with `RESOURCE_KIND_UNAVAILABLE` / 404.

**Machines are a module** as of S21 (`machines`), on the same footing as `places`. The key stays `machines` in code; the operator-facing word is **equipment**.

**The calendar is a surface, not a module.** `SiteFeatureRegistry::calendarLayers()` lists the modules that draw a column on the grid (`machines`, `places`). `hasCalendarLayer()` and the Twig `has_calendar_layer()` are the single question asked by the route gate, the header and the footer: with no layer on, `app_calendar*` **404s** instead of rendering an empty grid.

⚠️ **The calendar-layer flag is narrower than the resource group, on purpose.** `person_booking` is a full resource layer but is booked from its own pages and draws no column, so setting `calendarLayer` on it would bring the calendar back as an empty grid for an appointments-only deployment. Add it the day people appear on the grid.

The calendar takes both machines and places today, each conditional on its module. Two builders in `SiteController` produce them: `buildCalendarResources()` and `buildCalendarResourceAccess()`, both keyed by a composite **`"kind:id"`** string. That key exists because machine 2 and place 2 are different resources with the same id — a bare id collided in the filter set, the grid columns and the access map.

**`SiteFeatureService::all()` ignores rows whose key is no longer in `MODULES`.** A retired key (`emails`) had been leaving a live-looking switch on the admin screen that controlled nothing.

**The project gallery is `/creations*` / `app_creation*`** (S22), not `/leaderboard/creations*`. The three old public GET paths answer with permanent redirects, named `app_creation_legacy_*` so they sit under the gallery's own gate — with the gallery off they 404 rather than redirecting onto a page that then 404s. The vote and delete POST endpoints moved without redirects on purpose.

---

## 5. Mail

`App\Mail\Mailer` is the **single send chokepoint**. Nothing sends mail any other way.

- Refuses silently when the `emails` module is off or no sender account is configured. A lab without SMTP sends nothing rather than throwing from the middle of a booking.
- `EMAIL_LOG` is **both** the audit trail **and** the queue payload — the async Messenger message carries only the row id, so the queue stays tiny, a worker restart mid-send survives, and a retry re-reads current state.
- Transport is built **per send from the DB-configured DSN**, not `MAILER_DSN`.
- Each recipient's own locale via `LocaleSwitcher`. Context is stored as JSON, so **pass ISO date strings, never formatted ones** — the worker re-renders later, possibly in another language.
- `queueToUser()` honours the master `notificationEmail` switch **and** the per-category opt-out for non-transactional mail. Transactional mail (booking confirmations, event registration) has no off switch and carries no unsubscribe link — silently dropping a confirmation is worse than sending one.
- **Scheduled** mail is `app:mail:reminders` on an hourly systemd timer. Scanners are stateless and re-report everything; `MAIL_REMINDER`'s unique key decides what already went out via an `INSERT IGNORE` that is allowed to collide. **Claim before queue.** Adding a reminder = one class implementing `ReminderScanner` (auto-tagged).

### Mail gotchas that cost real debugging time

- **Twig auto-escapes the `subject` block**, so `MailSender` `html_entity_decode`s it. Do the same for any new plain-text header.
- **`strip_tags` keeps a link's label and throws the href away.** The text alternative said "Cancel my registration" with no URL — for a guest that link is their only way out. `MailSender` now flattens `<a>` to `label : url` *before* stripping.
- **The text part is built from the `body` block only**, so it never picks up the layout footer. The unsubscribe link is appended explicitly.
- **`show_cancel|default(true)` is a trap:** Twig's `default` fires on any *falsy* value, so passing `false` to suppress something returns `true`. Use `is not defined or`.
- Mail rendered by the worker has **no request**, so the router falls back to `DEFAULT_URI` = `http://localhost`. Absolute links come from the admin-set **`public_base_url`** setting; unset means **no link at all** rather than a broken one.

---

## 6. Signed URLs (the pattern used four times now)

For things a person must be able to do without an account: unsubscribe, guest event cancellation, event tickets. `UriSigner` over the **absolute** URL (so the signature covers the host), validated against `public_base_url` rather than the incoming `Host` header.

- **GET offers, POST performs.** Mail clients and security scanners fetch every link in a message before a human sees it; a mutating GET would unsubscribe or cancel for them.
- **No expiry** on unsubscribe: a link found in a two-year-old mail should still work.
- **A signature is not authorisation for someone else's action.** The event *ticket* is signed and public; the *scan* endpoint it points at is behind the `/staff` firewall. Two different URLs on purpose — one URL doing both jobs would hand every ticket-holder a self-service check-in button.

---

## 7. Deploying — read this before any schema change

Live app is **CT 210** on the Proxmox host "Artemis". Full recipe and SSH details are in the operator's private notes; the essentials:

- Code is placed **by hand** (tar + `pct push` + extract). **Never run `deploy.sh`** — it does `git pull origin main` and CT 210's own checkout is still `main`, so it would revert all hand-deployed work.
- **The agent cannot run migrations or `git push`** (classifier blocks production DB writes; the git remote needs interactive credentials). Hand the operator the one-liner.

### ⚠️ Deploy ordering is not a style preference

- **DBAL-repo feature** (booking policies, access passes, reminders): code may ship first — it degrades to "no rows". Migration can follow.
- **ORM-entity feature** (any new `#[ORM\Column]`): **migration MUST run first.** Entity mapping has **no fail-safe degradation** — adding a mapped column makes every query on that table select it, so the whole table 500s on the old schema. This took the live site down once (S20: `/`, `/events`, `/calendrier` all 500'd, because the homepage queries events).
- The safe sequence: extract **only** the migration file (`tar xzf bundle FabApp/migrations/VersionXXX.php` — Doctrine can't list a migration whose file isn't on the box), have the operator migrate, *then* extract the code.
- **Rollback recipe:** `git archive HEAD <paths> -o /tmp/rollback.tar` from the Mac, push and extract over the top, then `rm -f` the newly-added files (a tarball can't delete). **Then `rm -rf var/cache/prod`** — `cache:clear` was not enough; a stale *compiled Twig template* survived it and kept throwing an error citing **line 55 of a 41-line file**, which is the signature of exactly that. ⚠️ **It is `var/cache/prod` since 2026-07-30**; the box was on `dev` when that recipe was written, and clearing the wrong directory looks exactly like the bug it is meant to fix.
- **Templates and translations need `cache:clear` now, where dev picked them up on its own.** In prod every Twig template and message catalog is compiled once and cached, so a file push alone changes nothing on the page. This is the most likely way a future deploy will appear to have silently failed.
- Restart `fabos.service` **and `fabos-worker.service`** after touching anything under `src/Mail/` — the worker is a long-running process holding old code.

### Schema drift

The entities are **pre-existingly ahead of the migrations**. `doctrine:schema:update --dump-sql` shows a large diff that is *not* your fault and **must never be applied with `--force`** — the diff contains DROPs.

---

## 8. Verifying without a login

The operator's account is the only real login, and the agent has no credentials. Techniques that work:

- **Refusal branches** are testable anonymously (404/403/409/400 come back before auth in many cases).
- **A throwaway console command** is the workhorse: it creates its own fixtures, drives the service layer, prints a verdict table, and cleans up after itself. Deploy it to `src/Command/`, run it, then **delete it — never commit it**. Several bugs were only found this way (timezone drift, a false buffer pass, mail text-part link loss).
- **Mailpit** (`127.0.0.1:32769` on the box) is the mail oracle: read the delivered message, including the raw source for headers and the text part. Don't trust the template — read what arrived.
- **`php bin/console app:render <path>` renders any page from a shell, signed in** — the real request, through the real kernel, firewall and templates. `--as=` picks an account (default: the first admin), `--grep=` filters the body, `--anonymous` shows what a visitor sees, `--save=` writes the HTML out. This is how admin and staff screens get inspected now. ⚠️ **Committed in `507f3a8` but not deployed** — deploying it edits production authentication config, so it is the operator's call; until then admin screens are not inspectable at all.
- ⚠️ **What it replaced, and why the replacement is a different shape.** `LOCAL_ADMIN_BYPASS` was a password-less sign-in **reachable over HTTP**, held off by a flag, an IP range and debug mode — three conditions that a *request* travels through. Moving to `prod` switched it off by accident, which tells you how load-bearing they were. `ConsoleRenderAuthenticator` cannot be armed by a request at all: a method call on the service instance arms it, `PHP_SAPI === 'cli'` backstops it (a web request is never `cli` — the built-in server is `cli-server`), so it grants nothing a shell already lacked.
- ⚠️ **`app:render` authenticates by construction, so it can never show you an access rule that is too permissive.** It answers "does this page render, and what is on it" — the class of bug that hides behind a login. To prove a route actually *refuses*, curl it anonymously and read the 302.
- ⚠️ **What that bypass used to hide, kept here because the lesson outlives it:** with it on, a `curl` returned **200 whether or not the route was protected**, and a POST really executed. The ticket-scan route looked wide open and was not. Any future debug aid of this shape needs the same treatment — prove the refusal with the aid switched *off*.

---

## 9. Front-end conventions (and why the admin looks inconsistent)

There is **no shared admin base template**. Every admin page carries its own copy of the layout CSS in an inline `<style>`. This is the single biggest source of "the page renders badly" bugs, and there are **three** incompatible skeletons in the wild:

- **Style A** — standalone HTML: `.admin-header` → `.admin-main-content` (grid `280px 1fr`) → `.admin-panel`
- **Style B** — `{% extends 'base.html.twig' %}`: `.admin-page` → `.admin-page-header` → `.admin-layout` (grid `260px 1fr`) → `.admin-panel`
- **Edit/new pages** — `.admin-edit-header` → `.admin-edit-layout` → `.admin-edit-panel` + `.admin-edit-form`

Specific traps, each of which has already caused a visible bug:

- **`.admin-edit-panel` has no padding of its own** — it comes from `.admin-edit-form` on the `<form>`. Anything placed **after `{{ form_end(form) }}` sits flush against the panel edges.**
- **Classes used but defined nowhere** (`.btn-action`, `.form-field.check`) render as bare unstyled elements. An audit script that cross-references `class="..."` against defined selectors catches these instantly — worth re-creating.
- **`.form-field input { width: 100% }` also matches radios and checkboxes**, inflating them into full-width boxes. Exclude both types.
- **Never write literal light colours.** A full theme system exists (`--theme-surface`, `--theme-surface-elevated`, `--theme-input`, `--color-text*`, `--border-color`), flipped by `html[data-theme="dark"]` from `UTILISATEUR.theme`. An `<input>` with no `background` of its own is the loudest failure: a white box on a dark panel. Brand pink `#9E1B56` fails contrast on dark — add a dark override lifting to a light tint of the same hue.
- The **shared sidebar's** base look now lives in `style.css` (37 of 53 admin pages were rendering it as a bare link list). It sits **before** the media queries and dark-theme overrides on purpose so those still win. **Don't "tidy" it to the end of the file.**

---

## 10. Conventions to follow

- **i18n: five catalogs**, always in lockstep — `fr, en, de, es, it`. `fr` is the source language. A key added to one must be added to all five; they should stay identical in line count and structure.
- **Comments explain *why*, not *what*.** The codebase is written so the next reader understands the trade-off, especially where a simpler-looking choice would be wrong.
- **A stored setting that nothing reads is worse than no setting.** `UTILISATEUR.rappelReservation` was writable from two UIs and read by nothing — people who switched booking reminders off were reminded anyway. Retired in S16 by migrating those values into the real opt-out. Don't add a field before its reader.
- **Uploads:** extension allow-list, random filename, and delete the old file only **after** the new one is recorded, so a failed move can't lose the existing art. Convention lives in the lab-page photo upload.
- **Timezone:** the box runs **UTC**; the booking flow uses `Europe/Paris`. Naive datetime strings written and read without pinning a zone drift by the offset — an already-expired access pass tested as valid. Pin the zone on **both** the write (form parse) and the read (`fromRow`).

---

## 11. Where the bodies are buried

- ~~`public/js/calendar.js` (967 lines) is referenced by no template~~ — deleted 2026-07-30. The calendar page has its own inline JS.
- `TrainingEnrollment` is a **neutralised stub** (`Formation` has no date either), which is why training-session reminders and pass auto-issue to attendees are blocked, not merely unscheduled.
- `.flash` / `.notice` had **no dark-mode rules anywhere** until recently — they existed only as light-coloured copies in ~50 inline style blocks. Fixed globally with one `html[data-theme="dark"]` block that out-specifies them all.
- Five tracked files on CT 210 are **intentionally kept locally modified** and must never be committed from there: `.env`, `.env.local.example`, `compose.yaml`, `compose.override.yaml`, and a legacy SQL file.

---

## 12. Standing verification gaps (be honest about these)

- **The booking success path has never been verified by the agent.** Creating a real reservation needs a login; the firewall 302s anonymous POSTs to `/login` before the controller runs, so `POST /api/reservations` can't be curl-tested at all. Refusal branches were verified another way. **Ask the operator to click through one real booking** rather than assuming.
- Authenticated Twig pages (`/mes-reservations`, `/profil`, `/admin/*`) are **not agent-reachable at all any more** — the loopback bypass died with debug mode on 2026-07-30 (§8). Anything admin-shaped now needs either the operator's eyes or a throwaway console command that drives the service layer instead of the page.
- **`MISSING_PAGE` has never had a row written to it.** The table does not exist until the operator runs `Version20260804100000`; everything either side of the write was verified, including that a miss with nowhere to record still returns a clean 404. Once migrated, the first check is that `/admin/missing-pages` lists a deliberately wrong URL followed from a real page.
