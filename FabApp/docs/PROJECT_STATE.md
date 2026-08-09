# FabOS — project state & handover

**Last updated:** 2026-08-09 (through S104) · **Branch:** `main` · **Live:** https://fabos.dstei.fr, running `APP_ENV=prod`

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

⚠️ **`NextFreeSlotService` (S47) answers "when is this free?" by asking `BookingPolicyService` about candidate slots, never by reimplementing the rules.** Anything that suggests a time must go through the real checker, or it becomes a second set of booking rules that drifts from the enforced ones. It suggests only — `ReservationService::book()` still decides. ⚠️ **Pin `Europe/Paris` on the read as well as the write**: PHP's default zone here is UTC, so an unpinned Twig `|date` advertised an 08:00 opening as 06:00.

**To add a bookable resource kind:** one `ReservableType` case, one `ReservableResolver` branch, one arm in `checkAccess()` if it needs gating, and one entry in the two calendar builders (§4). That is the whole point of those three sessions.

---

## 2. Booking permission: four independent layers

They meet in `ReservationService::book()` and they are **kept apart on purpose**. Do not merge them.

| Layer | Question | Where | Failure mode if merged |
|---|---|---|---|
| **Certification** | *May you touch this at all?* (safety) | `checkAccess()` → `MachineQualificationService` | A lab loosening quotas would loosen safety |
| **Quotas** | *How much / how far ahead?* (fairness) | `BookingPolicyService::check()` | — |
| **Access passes** | Staff-issued exemption **from quotas only** | `AccessPassRepository` + `$passApplies` short-circuit | A pass would become a safety bypass |
| **Usage Rights** | *Does this member's package cover this action and use interval?* | `UsageRightsService` + `UsageCapabilityRegistry` | UI and enforcement disagree, or a package becomes a safety/role bypass |

Consequences worth internalising:

- **Cert-gating was already built** long before it appeared as a roadmap "todo". `MachineQualificationService::getStatus()` reads the training mode (free / theory / theory+physical), required badges via `MachineBadge`, held badges via `UtilisateurBadge`. Admins bypass it. The calendar surfaces it through `SiteController::buildCalendarBookingAccess()`. The RFID door path is the separate `MachineAccessService::authorize()`. **Don't rebuild any of this.**
- **There is deliberately no cert-bypass column** in `ACCESS_PASS`, and a comment in the migration says so. Don't add one. A pass is a convenience object that gets handed around and extended; safety bypass needs its own explicitly-issued, supervision-scoped record.
- **Quota refusals are 409, not 403.** The booking isn't forbidden in principle, it conflicts with what you already hold — cancelling something makes it succeed.
- **Quota checks are ordered coarsest-constraint-first.** Min-notice/horizon before slot alignment: "you can't book this soon at all" must beat "round it to the half hour", or fixing the alignment of an unbookable slot just earns a second refusal.
- **S104 quota repair:** all active/day/week counters are scoped by `ReservableType`; a machine booking cannot consume a place/person quota. An access pass skips only the soft quota checks; slot alignment and buffer conflict run first and remain non-bypassable.
- **Live S97–S99 Usage Rights is an AND gate, never an override.** It is still opt-in and portal-local **in the running code only**; S102 supersedes that target with FabOS-wide packages and sub-location-scoped grants. Administrators retain recovery access, and the live gate cannot bypass a disabled feature, certification, opening hours or quotas. Only capabilities with a central enforced write path belong in `UsageCapabilityRegistry`; today those are machines, places, person booking and member-only events. Public/administrative UI consumes the same verdict as the services through the shared `rights-*` components.

---

## 3. Config, and the fail-open / fail-closed rule

### Temporary development workspace — remove before production

`SiteSettingService::isDevelopmentMode()` controls **only** whether the admin navigation shows the Development section (Design maquette, read-only product prototypes and diagnostics). It defaults to off, is writable only through the already-admin-only Site settings form, and must never be used to relax route access or authentication. S103 adds the read-only `FeatureWorkspaceRegistry` matrix and corrected Rights/Structure maquettes; it persists nothing and does not enforce access. Before promoting this installation beyond Artemis development, turn the setting off and remove the development menu/setting if it is no longer needed. The only CLI-only authenticated renderer remains `app:render`; do not reintroduce a request-reachable admin bypass.

Config-adjacent stores are **raw DBAL, not entities**, and fail-safe on reads. The direction of failure is chosen per store and it matters:

| Store | On read error | Why |
|---|---|---|
| `BookingPolicyRepository` | **open** (= no limit) | Refusing every booking over a config problem takes the lab offline |
| `AccessPassRepository` | **closed** (= no pass) | The failure would otherwise *grant* an exemption nobody issued |
| `NotificationPreferences` | **open** (= send it) | `ReminderLog` claims *before* sending, so failing closed would burn the claim and drop the reminder permanently and silently |
| `ReminderLog::claim()` | **closed** (= don't send) | Forgetting costs one reminder; guessing costs a mail loop |

**"Empty table is a complete, valid configuration"** is the house style. `BOOKING_POLICY` seeds no rows; every limit column is nullable and null means no limit. Same for access passes and reminder toggles (all ship **off**). An all-blank save *deletes* the row rather than storing nulls, so "configured" and "actually constrains something" can never drift apart.

**Legacy live state — portals remain reachable until S105–S127 remove them safely.** Since S27, `/admin/portals` creates/configures them, `PortalOverrides::FIELDS` lists overrides, and per-portal features are tri-state. S102 says no new product work may depend on this model. S105 must freeze creation and report every divergent hostname/setting/feature/package before consolidation; S127 removes storage only after a complete compatibility cycle and tested restoration.

**Target boundary after S102:** a service with independent admins, theme, features or data runs its own FabOS; a Sub-location is only a physical subdivision under shared governance/data and lists aggregate all authorized sub-locations by default. SSO/LDAP may reuse authentication but never imports local roles, groups, packages or recovery authority. `ExternalIdentityLink` will key links by immutable `(issuer, subject)`, never e-mail. Selected portable data crosses instances through the signed FabOS network; reservations/events remain owned by one instance and redirect there unless a future distributed-booking protocol is built explicitly.

**S102 decisions refined, target only — running authorization is unchanged.** Guest is the anonymous audience and visibility is separate from action/registration; per-event tri-state overrides inherit a FabOS default. Admin, Manager, Staff, Super user, User, Guest and Trainer/Formateurs are protected built-ins; User means every active authenticated account and Guest means anonymous, both without memberships. Package grants from the person and every group accumulate by union and default to deny; quota profiles remain complete alternative paths rather than fieldwise merges. An Institution has one canonical unique HTTPS origin and remains descriptive unless secure FabOS discovery succeeds and an admin confirms trust. Personal exports require instance allowlisting plus member consent, while non-personal catalogues follow publication and peer-trust policy. Badge awards are never deleted, only revoked with provenance, including across QR/import. Materials are an instance catalogue, while availability/location/stock are sub-location data.

**Future Commerce target — no live payment code exists.** After the S103–S128 core is stable, an optional Commerce module may sell package assignments, materials and credits for machine/person/training time. A shared offer/order/payment/refund/fulfillment engine serves feature-specific storefronts. Only verified webhooks/provider reconciliation confirm payment; unique provider events plus per-line fulfillment/outbox produce effectively-once delivery. Package fulfillment uses normal Usage Rights assignment, materials reserve stock atomically (or declare backorder), and time uses an append-only hold/consume/release ledger. Refund money and domain compensation are separate/idempotent. Commerce never writes voters directly, auto-books, or bypasses badges, qualification, shutdowns, capacity, schedules or unrelated quotas. Provider credentials and card data do not belong in FabOS storage.

**Much-later Training communications target — no messaging code exists.** S134–S136 will add immutable visibility types: trainer→cohort announcements, trainer↔one-learner private threads, and explicitly composed groups. The persisted plain-text, bounded/rate-limited FabOS message is canonical; one asynchronous e-mail copy per recipient is notification only. Every read/write and the mail worker revalidate active account, enrollment/assignment and participation so queued mail cannot leak after revocation. Mail failure cannot lose the internal message, retries are deduplicated, recipient identities are isolated, and inbound e-mail replies/real-time chat/attachments are not implied.

**Legacy portal scoping to migrate, not copy.** `PortalContext::scopeId()` returns the portal id or `0`; `SITE_SETTING`, `SITE_MODULE`, usage packages, mails and logs consume it. The default portal owns no rows and maps to global. Preserve these exact semantics in the S105 consolidation report so differing rows are never dropped silently; future scope is explicit Sub-location, not a renamed `portalId`.

---

## 4. Modules

`SiteFeatureRegistry` is the live list of keys; `SITE_MODULE` currently holds on/off state per portal. S103 defines the future workspace metadata, while S105 consolidates feature state to the FabOS instance.

### A feature is one of three things (S22)

Read this before adding one. The word "module" used to answer three questions at once, and the conflation is what let "hide the team page" look like "turn off the team".

| Layer | Owns | Modules |
|---|---|---|
| **resource** | a bookable kind, **and whether bookings of it are accepted at all** | `machines`, `places`, `person_booking` |
| **activity** | a feature domain, with its own pages and data | `events`, `formations`, `badges`, `projects`, `leaderboard`, `lab_pages`, `materials`, `loans`, `maintenance` |
| **directory** | **a page and a menu entry, and nothing else** | `staff`, `trainers` |

**Kernel is not on this list and never becomes a toggle:** users, roles and authorisation, the staff desk, auth, profiles, settings, mail transport, and the booking/calendar engine. Portals are currently kernel infrastructure but are scheduled for retirement by S127; Sub-location becomes a first-class domain model, not a feature toggle.

✅ **Both navs now obey this (S50, 2026-07-31).** `_admin_sidebar.html.twig` carries a `feature:` per entry — `null` = kernel, never gated; `'*resource'` = **any** bookable layer via `has_calendar_layer()`. ⚠️ **Booking screens (Réservations, Quotas, Accès exceptionnels) must gate on `'*resource'`, never on `machines`** — bookings are polymorphic, so a `machines` gate hides them from a spaces-only install that books perfectly well. ⚠️ **Gating a nav means checking groups for emptiness in the same change**: all four sidebar variants printed their group heading unconditionally, so gating alone leaves bare headings over nothing. ⚠️ **This install has all 14 features on, so it cannot test a gate** — exercise the `|filter` expressions with stubbed `feature_enabled`/`has_calendar_layer` instead of assuming the live render proves anything.

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
| `src/Nav/NavBuilder.php` | Builds the header and footer from what is switched on. `_header`/`_footer` just render the list. | A group with no visible children is never emitted, so an empty dropdown cannot render. The **calendar group keeps its own landing rule** (`/calendrier` whenever anything is on the grid) rather than "first visible child" — don't "simplify" that away. ⚠️ **The menu lists what the *site* has, not what *you* have** (2026-07-31): *Mes réservations* and *Mes disponibilités* were pulled from both `header()` and `footer()` and now live only on `/profil`. Personal pages do not go back in here. |
| `src/Feature/SetupHealth.php` | `/admin/setup` — what is not configured **and what each gap costs**. Severity by consequence: blocking / degraded / info. | Read-only by design. Every fix links out to the screen that owns the setting; a second place to edit the same thing is how they drift. |
| `src/Feature/FirstRun.php` | Whether this install has ever been set up; drives the `/admin/wizard` card. | ⚠️ **"Fresh" needs the flag absent *and* the install to look empty.** A missing `setup_completed_at` alone would show a first-run wizard to every install that predates the flag. Fails safe toward "already set up". **Nothing redirects to the wizard** — a card and a sidebar entry are the only ways in, on purpose. |
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

Live app is **CT 210** on the Proxmox host "Artemis". Connect with `ssh -i ~/.ssh/id_ovh -p 4002 artemis.dryades.org`; the key stays outside the repository. The account is unprivileged, so container commands use `sudo pct exec 210 -- bash -lc "…"` and transfers use `sudo pct push`. The essentials:

- The canonical macOS-safe tar/push/extract tutorial is `docs/ARTEMIS_DEPLOYMENT.md`. Follow it instead of improvising: macOS xattrs, AppleDouble entries and ownership metadata otherwise break or pollute every archive.
- **Nobody develops or edits code on Artemis.** Only Codex sessions deploy and verify there. A pre-deploy hash/diff still protects against an earlier Codex deployment that has not yet been reflected in the local checkout; it is not coordination with another developer.
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

There is **no shared admin base template**, but as of S29 the chrome is defined once in `public/css/admin.css` and every skeleton's class names resolve to it — banner, two-column shell and rhythm are identical across all of them (verified by measuring computed geometry: the content column is 938px on every admin page). What is left is one markup job, described at the end of this section.

⚠️ **There were never three skeletons — there are six.** The three below, plus `.admin-rfid-*` (2 pages), `.admin-user-*` (1) and `.admin-content-grid` (6), the last of which is its own two-column shell and had never been written down. Counting them by reading templates missed half; counting them by measuring found them all.

- **Style A** — standalone HTML: `.admin-header` → `.admin-main-content` (grid `280px 1fr`) → `.admin-panel`
- **Style B** — `{% extends 'base.html.twig' %}`: `.admin-page` → `.admin-page-header` → `.admin-layout` (grid `260px 1fr`) → `.admin-panel`
- **Edit/new pages** — `.admin-edit-header` → `.admin-edit-layout` → `.admin-edit-panel` + `.admin-edit-form`

Specific traps, each of which has already caused a visible bug:

- **`.admin-edit-panel` has no padding of its own** — it comes from `.admin-edit-form` on the `<form>`. Anything placed **after `{{ form_end(form) }}` sits flush against the panel edges.**
- **Classes used but defined nowhere** (`.btn-action`, `.form-field.check`) render as bare unstyled elements. An audit script that cross-references `class="..."` against defined selectors catches these instantly — worth re-creating.
- **`.form-field input { width: 100% }` also matches radios and checkboxes**, inflating them into full-width boxes. Exclude both types.
- ✅ **The public side has a shared layout: `site/base_public.html.twig`, extended by all 37 pages that carry the header and footer** (S46, 2026-07-31). It owns the head, the favicon, `style.css` and its cache-bust, and the header/footer includes; children fill `title`, `head`, `stylesheets`, `page_styles`, `body`, `javascripts`, and set `body_class` / `html_attrs` as variables. **Add a public page by extending it — never by copying another page's `<head>`.** The eight templates that still stand alone (kiosk ×4, `event-ticket`, `staff-scan`, `recherche`, `creation-new`) include neither header nor footer and are a different medium, not an oversight. ⚠️ It is deliberately separate from `base.html.twig`, which 23 admin templates extend; **S53 merges them.**
- ⚠️ **AssetMapper + Stimulus are live since 2026-08-01; `symfony/ux-turbo` is installed but switched off.** Interaction work goes in a **Stimulus controller** (`assets/controllers/*_controller.js`), never in a new inline `<script>`. `calendrier.html.twig` still hand-writes 395 lines of inline JS — that is S47's to rewrite, not a pattern to copy. "Twig server-rendered, no SPA" is a deliberate decision — client-side rendering would need the feature gate reimplemented, which is how "visibility is not permission" gets broken by accident.
- **~430 KB of CSS across 13 stylesheets (`calendar.css`, 28 KB and referenced by nothing, deleted in S53), 135 KB of it four overlapping calendar files** (`calendar.css`, `calendar-fix.css`, `calendar-modern.css`, `calendar-leaderboard.css`), plus 5 782 lines of inline `<style>` in 87 templates. A file named `-fix` beside the file it fixes is the shape of the problem.
- ✅ **Every homepage block is feature-gated (S55, 2026-08-01)** — the gate lives in `HomepageVisibilityService::getVisibilityMap()`, the single map that data-loading, rendering *and* section-ordering all read. ⚠️ **Don't gate at the call site**: doing so is what let the controller and the template disagree about `featured_machines`. ⚠️ **Block labels are code-owned** — `buildRow()` used to prefer the stored one, so renaming a label silently did nothing on any install that had saved the form.
- ✅ **Dead affordances are gone (S54, 2026-07-31)** — 40 static `disabled` controls across 16 templates, for −748 lines. ⚠️ **The audit that found them grepped `"bientôt disponible"` and undercounted by half**: a dead filter bar, a bulk-select column and a 69-line mock modal never apologise. **Grep the `disabled` attribute.** A disabled control is exempt from WCAG 1.4.3 *and* from markup audits, so nothing but grep will find these. The ~10 `disabled` attributes left are all genuine state (feature gates, `unlocked`, JS-driven pagination) — before adding another, it must be a state the page can leave.
- ⚠️ **A dead affordance is usually sitting on top of a working feature.** `/admin/badges` had a `disabled` search box directly *below* a search form that worked, and a mock "create badge" modal beside a real `app_admin_badge_new` link. Check for the working version before building anything.
- ⚠️ **Removing a cell from a CSS-grid row means fixing `grid-template-columns`.** Two list tables kept a `40px`/`42px` column after their checkboxes went, indenting every row by a phantom column. And when deleting now-dead CSS, **prune selectors out of lists rather than deleting whole rules** — one such rule also styled `.checkmark`, and pruning another carelessly would have left a bare `html[data-theme="dark"] {}` recolouring the whole page.
- ✅ **Role-gated affordances call `can_reach('route')`, never `is_granted('ROLE_X')`** (S49, 2026-08-01). It asks `security.access_map`, the same `access_control` map the firewall consults, so there is no copy of the rules to drift. ⚠️ **There is no role hierarchy here** — `ROLE_ADMIN` does not imply `ROLE_STAFF` — which is why naming a role by hand keeps going wrong in both directions. ⚠️ It answers "may you open this URL", not "may you do this to this record": per-row rules stay in controllers and voters. ⚠️ Still presentation — the firewall is what stops anyone.
- 🔴 **`machine.machineToken` must never appear on a page anonymous users can read** (found 2026-08-01). It looks like a label and is the path segment addressing the machine on `/api/rfid/machines/{machineToken}/…`. It was public on `machine-detail`, `machine-historique` and the homepage cards; removed. Admin screens may show it. ⚠️ The other half is still open: `rejectUnauthorizedDevice()` returns `null` — *allowed* — when `FABOS_RFID_API_TOKEN` is empty, and it is unset on the live box.
- ✅ **`public/css/components.css` is the component layer (S51)** — token-only, loaded once from `base_public.html.twig`. ⚠️ **Only components that a surface actually uses live in it**; cards/skeletons/segmented/stat-tiles were written and deleted before commit rather than shipped as dead CSS. ⚠️ **Success toasts fade, errors never do.**
- ⚠️ **Touch targets are gated on `(pointer: coarse)`, never on viewport width** (S52) — a narrow desktop window is a mouse, a big tablet is a finger. 44×44 minimum.
- ⚠️ **`importmap('app')` is in `base.html.twig`, `site/base_public.html.twig` and 8 standalone pages.** A page that is none of those (there are ~45 standalone admin templates left) needs the line added before a controller will run on it. **Turbo is `enabled: false` in `assets/controllers.json`** — `window.Turbo` is undefined in prod and navigation is untouched; turning it on is its own watched session. Native HTML (`<details>`, `<dialog>`) is still the first choice when it suffices.
- ⚠️ **Deploying a JS or template change now needs two extra commands on the box:** `php bin/console importmap:install` then `php bin/console asset-map:compile`, before `cache:clear`. `assets/vendor/` and `public/assets/` are gitignored, so the tar never carries them. Skipping `asset-map:compile` means the browser gets 404s for every module.
- ⚠️ **A Stimulus controller cannot replace code that must run before first paint.** Modules are deferred. The theme bootstrap duplicated in `_header_auth` and `_admin_sidebar` stays inline for exactly this reason — converting it would trade a duplication for a flash of the wrong theme.
- **Status colour has one home: `.admin-status-*` / `.admin-status-solid-*` in `admin.css`**, with measured dark values. Don't write a status hex into a page's inline block — three pages did, and all three were unreadable on dark (2.2–3.0:1). ⚠️ The `admin-` prefix matters: `.status-ok` and `.status-info` are **already taken by style.css** for the public `/status` page, and since it loads first you inherit its near-white background while overriding only the text.
- **Verifying CSS is possible now**, and this is how S29's visual pass was done: `app:render --save` each admin page, serve the HTML locally against the *public* stylesheets, drive the browser over it, and measure contrast in the DOM instead of judging by eye. ⚠️ **Strip `main.js` and pin `data-theme` for the light variant** — main.js applies the OS theme at runtime, so an unmodified "light" render is dark on a dark machine, and a whole audit pass can silently be testing one theme twice.
- **Never write literal light colours.** A full theme system exists (`--theme-surface`, `--theme-surface-elevated`, `--theme-input`, `--color-text*`, `--border-color`), flipped by `html[data-theme="dark"]` from `UTILISATEUR.theme`. An `<input>` with no `background` of its own is the loudest failure: a white box on a dark panel. Brand pink `#9E1B56` fails contrast on dark — add a dark override lifting to a light tint of the same hue.
- ⚠️ **An inline override is deliberate until measured otherwise.** Six pages nest `.admin-main-content` *inside* `.admin-content-grid`, where it is the content column rather than the shell, and each said so with an inline `display: flex`. Deleting those as duplicate chrome gave every one of them a second, empty 260px sidebar. That rule now lives once in `admin.css` as a descendant selector.
- **`.admin-panel` is a frame, everywhere. Its children take the 24px gutter** — same idiom as `.admin-edit-panel`. Exempt: `.admin-panel-header` (pads itself) and the table wrappers (bleed to the edge). Don't put padding back on the panel; don't add a padded direct child without exempting it, or it doubles. *(It looked like two semantics and was actually three arbitrary values — 0, 24px and 32px — which is why the `.admin-panel-body` wrapper the plan called for turned out to be unnecessary.)*
- ⚠️ **Read the templates to find candidates; decide from measurement.** Across S29's three passes, reading the source said there were three skeletons (six), said `.admin-panel` had two semantics (three values), and made six deliberate `display: flex` overrides look like duplicate chrome. Every one of those was caught by printing computed styles in a browser, and none by reading. The harness: `app:render --save`, serve locally against the public stylesheets, iterate in same-origin iframes, print computed geometry.
- The **shared sidebar's** base look now lives in `style.css` (37 of 53 admin pages were rendering it as a bare link list). It sits **before** the media queries and dark-theme overrides on purpose so those still win. **Don't "tidy" it to the end of the file.**

---

### Inline admin actions (S30)

`site/_admin_inline.html.twig` is the one affordance for "edit the thing you are looking at", used on the six public detail pages. Adding one is a `{{ admin_actions([...]) }}` call next to the title.

- ⚠️ **`role` must mirror the target route's real `access_control` line, and there is NO role hierarchy in this app** — `ROLE_ADMIN` does not imply `ROLE_STAFF`. Getting it wrong is a usability bug in both directions: too strict hides the shortcut from the people entitled to it, too loose offers a button that bounces to `/login`.
- **Visibility is not authorisation.** The firewall decides on every request; the chip only decides whether a shortcut is offered. Verify both: chips absent for anonymous *and* the href 302s.
- The chip **carries its own backdrop** because titles here are sometimes overlaid on a photo hero. Don't make it transparent again — it looks correct on a white panel and disappears on `/events/{id}`.

---

## 10. Conventions to follow

- **i18n: five catalogs**, always in lockstep — `fr, en, de, es, it`. `fr` is the source language. A key added to one must be added to all five; they should stay identical in line count and structure.
- **Never write the organisation or the venue into a string.** `%org%` and `%venue%` are injected into *every* translation by `VocabularyTranslator` (a decorator on `translator`), so a new catalog string just uses them — no call site passes anything. Outside translations, Twig has `org_name()` and `venue_label()`. ⚠️ Keep that decorator **lazy and fail-safe**: it sits in front of all text and is reached during cache warmup, from the console and from the mail worker, none of which are guaranteed a database.
- ⚠️ **Concatenating into a `??` fallback needs parentheses.** `x.description ?? 'text' ~ venue_label()` can bind as `(x.description ?? 'text') ~ venue_label()` and append boilerplate to a record that has a description. Twig 3.15 deprecates the ambiguity — treat that warning as a bug report.
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
