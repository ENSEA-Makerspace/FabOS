## Phase H — Hardening, correctness and coverage

*Added 2026-07-31. **This phase runs before Phase D.** The letter is out of alphabetical order on purpose: A–E were already assigned and several of those sessions have shipped, so renumbering them would invalidate every commit message and cross-reference that cites them. Read the order from this sentence, not from the letter.*

**Why now.** Most of this is not new work — it is a codebase audit from 2026-07-10 that was parked before anything was implemented, plus the verification debts this project has been honest about accumulating. It was re-checked against the live site on 2026-07-31 and **the critical findings are still true today**, so the parking was the only thing keeping them open.

**Sizing note.** These are deliberately small next to a Phase A session. That is the point: each one should be startable and finishable without a decision from anybody, because the reason they went unfixed for three weeks was never difficulty.

### Before anything else — three chores, not sessions

1. **Push.** Twenty commits sit only on this Mac and CT 210's filesystem. The branch on GitHub is the only second copy.
2. **Clear two probe rows** from `MISSING_PAGE` (`/probe-raw`, `/probe-service`), left by an S37 verification harness — the *Vider le journal* button on `/admin/missing-pages`.
3. **Re-save the `events` portal.** Its accent colour is stored as `#1D4ED8.` with a trailing dot, so no colour is emitted. Since S28 the form refuses that instead of accepting it silently.

---

### S38 · Stop publishing badge UIDs and names — ⚠️ do this first

**Why.** `GET https://fabos.dstei.fr/api/leaderboard` returns, unauthenticated and from the public internet, a list of real people paired with their **physical badge UID**:

```
{"rank":1,"username":"yanis","displayName":"Yanis Test","rfid":"8C52B359"}
```

That UID is the credential the door and machine readers trust. Publishing it next to a name is a badge-cloning kit. Two neighbours are the same class: `/api/access-rfid-logs` exposes badge UIDs plus who used which machine when, and `/api/reservations` exposes real names with what they booked and when.

**Scope.** Drop `rfid` from the leaderboard payload outright — nothing needs it client-side. Decide per endpoint whether the fix is *narrowing the serialiser* or *requiring authentication*; prefer narrowing, because an endpoint that stops returning a field cannot leak it again by a permissions mistake later.

⚠️ **Watch — find the consumers before changing the shape.** These are public JSON endpoints on a live site; a kiosk screen, a script or somebody's dashboard may be reading them. Removing a field is a breaking change for whoever that is. Grep the repo, then ask the operator, before deciding between narrowing and gating.

⚠️ **A field-name grep is not enough to check this.** The entity property is `identifiantRfid` and the JSON key is `rfid`; grepping the former returns zero hits and reads like "already fixed". **Verify by fetching the endpoint and reading the payload.**

**Verify.** Anonymous fetch of each endpoint contains no badge UID and no personal data beyond what a public leaderboard needs; the signed-in paths still return what their pages require.

#### What shipped 2026-08-01 — the API side

**The scope was more than double what this section listed.** Fetching every endpoint anonymously and reading the payloads — the check this section demands, and the only one that works — found **8 leaking endpoints, not 3**:

| endpoint | leaked | fix |
|---|---|---|
| `/api/leaderboard` | `rfid` + real name | **narrowed** — field dropped, stays public |
| `/api/access-rfid-logs` | `badgeUid` + name + machine + time | gated `ROLE_ADMIN` |
| `/api/machines/{id}/historique` | `badgeUid` + names, enumerable | gated `ROLE_ADMIN` |
| `/api/progressions` | name + training **score** | gated `ROLE_ADMIN` |
| `/api/reservations` | name + what + when + `motif` | gated `ROLE_ADMIN` |
| `/api/reservations/{id}` | same, enumerable by id | gated `ROLE_ADMIN` |
| `/api/calendar` | name + `motif`, every active booking | **narrowed** to occupancy |
| `/api/machines/{id}/reservations` | name + `motif` | **narrowed** to occupancy |

⚠️ **Three of the eight were in no list anywhere** — `machines/{id}/historique`, `progressions` and `calendar`. They were found by the sweep, not by reading the finding.

⚠️ **The grep trap is worse than recorded.** The property is `identifiantRfid`, the leaderboard key is `rfid`, and the RFID-log key is **`badgeUid`**. No single grep covers all three; only fetching the payloads does.

**`/api-docs` decided narrow-vs-gate more cleanly than guesswork could.** `/api/leaderboard` is *documented public* — so the contract was right and only the field was wrong, which is the textbook narrowing case. `/api/reservations` was *already documented* under "Réservations connectées" and simply never enforced, so gating it **restored** the documented contract instead of breaking one. The other three were undocumented, so nothing public could depend on them.

⚠️ **The shared serialisers could not be narrowed in place.** `reservationToArray()`, `rfidLogToArray()` and `progressionToArray()` are used by the `ROLE_ADMIN` endpoint `/api/users/{id}`, where the name and the motif are exactly what the caller is entitled to. Stripping them would have silently emptied the admin views. Hence a **second** serialiser, `reservationToOccupancyArray()` — and deliberately a separate function rather than an `$includeIdentity = false` flag, because a flag puts the safe outcome one forgotten argument away.

**Verified:** all 18 GET endpoints fetched anonymously; every one either returns no identity field at all or redirects to login. `/api/leaderboard` still ranks, `/api/calendar` and `/api/machines/{id}/reservations` still return their slots.

#### What shipped 2026-08-01 — the page side, as an operator setting

**`booking_identity_roles`** in `SITE_SETTING` (no migration — the table predates this), ticked per role in Site settings from the **`ROLE` table the operator already edits**. This install offers `ROLE_ADMIN`, `ROLE_STAFF`, `ROLE_TRAINER`, `ROLE_USER`; the default is **staff + admin**, deliberately the restrictive end, so an install that never opens the screen does not inherit the leak.

`BookingIdentityPolicy` owns the decision so the two calendar templates cannot drift, and so the next surface that shows bookings asks rather than invents.

⚠️ **Each configured role is tested separately and OR'd** — this app has no role hierarchy, so testing only the "highest" one would hide names from admins on an install that ticked staff alone.

⚠️ **`user_id` is no longer sent to the browser at all.** The calendars badge your own booking as "Ma réservation", which needed the viewer's id client-side; shipping every booking's id would let anyone group one person's slots and follow their week — most of what a name gives you. A server-computed `mine` flag replaced it, so **hiding identities never hides your own booking from you**.

⚠️ **`method_exists(null, 'getId')` is a TypeError in PHP 8** — it took all three calendars to 500 on first deploy, for the anonymous visitor the change exists to serve. Fixed by typing against the entity. The lesson is the older one: `curl` the page, do not trust that it compiled.

**Verified:** all three calendars fetched anonymously emit `user: null`, `motif: null`, no `user_id`, and still carry every slot's start and end; `app:render /admin/settings` shows the four role checkboxes with staff + admin pre-ticked.

#### S38c — the surfaces the setting did not reach (2026-08-01)

Closing the calendars was not the end of it. A sweep of **every public page** for real badge UIDs and real names — rather than of the pages just touched — found two more:

- 🔴 **`/machines/{id}/historique` was published in full to anonymous visitors**: badge UIDs in clear (`8C52B359`, `39F1C387`, …), the name attached to each scan, each work session and each booking. The JSON endpoint of the same name had been gated an hour earlier; **the page rendering the same rows was missed.** Names now follow `BookingIdentityPolicy`; the badge UID column is `ROLE_ADMIN` only, because a UID is a credential and no display need justifies it.
- 🔴 **`/api-docs` hardcoded a real badge UID as its example.** `8C52B359` is an actual row in the database — the public documentation was handing out a working credential labelled "example". Replaced with `A1B2C3D4` and a comment telling the next author never to paste a live one back in.

⚠️ **The lesson is the sweep, not the fix.** Both were found by grepping every public page for known-real UIDs and names, not by reasoning about which pages "should" have them. This is the second time the API was closed while a page kept publishing the same rows.

**Left deliberately public, because names are the point:** `/equipe` (a team directory), `/creations` (a gallery of members' own published work), `/leaderboard` (documented public, and a leaderboard without names is a list of numbers).

**Decided the same day — the history pages are scoped to the reader, not merely masked.** Masking the columns still told an anonymous visitor *how many* people used a machine and exactly when, which is most of the information. So the **rows** are filtered:

| viewer | rows |
|---|---|
| anonymous | none, with an invitation to sign in |
| signed-in member | their own scans, sessions and bookings, nobody else's |
| staff / admin | everything |

⚠️ **The entitlement reuses `BookingIdentityPolicy` rather than a hardcoded `ROLE_STAFF`.** An operator who ticks *formateur* for the calendars means it here too, and two rules answering the same question drift apart.

⚠️ **The badge UID column stays `ROLE_ADMIN` even in "own" scope.** A member's own UID is not a leak, but a credential printed on a page has no display need behind it.

⚠️ **The scope has to be *said*, not just applied.** The stat cards count the visible rows, so a member shown "3 passages" with no explanation reads it as the machine's whole history — a wrong number stated confidently, which is worse than the missing rows. Hence `.fab-scope-notice`, a real component in `components.css` (⚠️ `.notice` looked reusable but is defined only *inline* in `admin-emails.html.twig`, plus an orphaned dark-theme rule in `style.css` — using it here would have styled nothing). Cache-buster bumped on `components.css`.

**Verified with `app:render --as`**, all three scopes on the same page: anonymous 0/0/0; Sofia 36/0/6 with her own name and **zero** occurrences of Yanis, Cedric or Alvaro; admin 129/2/14 with UIDs.

**Decided: `/kiosk/*` stays public.** It publishes who is in the building right now, unauthenticated — accepted for now, with **restricting the kiosk routes as a group** left as a later job rather than a per-page patch.

**Superseded — the original text of this finding:**

🔴 **Not fixed, and it needs a decision: the calendar *page* leaks the same data by another route.** `/calendrier`, `/calendar` and `/machines/{id}/calendrier` all return **200 anonymously** and server-render every booking's real name and free-text `motif` into the page's inline JS (`calendrier.html.twig:204`). Closing the API while the page publishes the same rows is theatre. It was left open on purpose: what the calendar should show is a **product decision, not a security one** — some labs show who booked so you can go and ask them, some do not — and it changes what signed-in members see, not just anonymous visitors. **S38 is not done until that is answered.**

---

### S38b · The timezone audit — what the two conventions actually are (2026-08-01)

The blocked step from the reverted `date_default_timezone_set()` attempt. **The conventions are not arbitrary; they are two coherent rules, and only one of them is broken.**

**Convention A — machine timestamps. Stored UTC wall-clock, displayed WITHOUT conversion. 🔴 This is the bug: shown 2 h early.**
Written by `new \DateTimeImmutable()` with PHP's default zone (UTC on the box), or by MariaDB's `CURRENT_TIMESTAMP` default — **MariaDB is on UTC too** (`NOW()` == `UTC_TIMESTAMP()`, checked), so the 21 columns carrying that default agree with the PHP ones.
Fields: `AccessRfidLog.createdAt`, `LogUtilisation.dateDebut/dateFin` (`WorkSessionService`), `Progression.dateDebut/dateEnd`, every `createdAt`/`created`/`updated`, `derniereConnexion`, `lastSeenAt`, `lastAuthorizationTime`, `queuedAt`.

**Convention B — human-entered wall-clock. Stored in the lab's wall-clock, displayed WITHOUT conversion. ✅ Correct today.**
The rule is documented in `StaffController` around line 126: parse the `datetime-local` string in the lab zone so that, formatted back to a naive string for storage, it stays the time the human typed.
Fields: `Reservation.dateDebut/dateFin` (`SiteController` ~1572, `ApiController` ~720, `PersonBookingController`), access-pass validity, `OpeningHours.openTime/closeTime`, and `Event.dateDebut/dateFin` — the last via `EventAdminType`, which sets no `model_timezone`/`view_timezone`, so the submitted wall-clock round-trips verbatim.

⚠️ **The fix is therefore the opposite of "pin `Europe/Paris` everywhere".** Pinning a Convention B field **double-shifts it**: opening hours render `08:00` correctly today and a pin would make them `10:00`. The spawned session that was mass-pinning 142 call sites was stopped for exactly this reason — it would have broken the fields that were already right while fixing the ones that were wrong.

⚠️ **And it is why no global default can work.** A global default moves the read *and* the hydration (they cancel, so nothing appears to change) *and* the Symfony form model timezone — so newly created events would start being stored under a different convention from the existing ones. A silent split, on top of a fix that fixes nothing.

**What the real fix looks like:** convert on display **for Convention A only**, via the operator's configured zone. The 142 unpinned calls split roughly by field — `createdAt` (22), `dateEnd`, `checkedInAt`, `derniereConnexion`, `lastAuthorizationTime`, `updated` and friends are A; the 48 `dateDebut` and 20 `dateFin` are **split by entity**, `Reservation`/`Event` being B and `LogUtilisation`/`Progression` being A. ⚠️ **Classify by entity, never by field name** — `dateDebut` alone belongs to both conventions.

#### What shipped 2026-08-01 — the unification

**The zone is an operator setting.** `timezone` in `SITE_SETTING` (no migration), asked as part of wizard step 3 ("Où est-ce, dans quel fuseau, et dans quelle langue ?") and editable in Site settings — 418 zones, validated against `DateTimeZone::listIdentifiers()`, rejected with a flash rather than silently ignored. The settings screen shows *"Il est 23:31 chez vous d'après ce réglage"* so the operator can check it against their own watch instead of trusting a zone name.

**`|lab_date()` — a named filter rather than a timezone argument.** `LabTimeExtension` adds one filter and one function (`lab_timezone()`). The name is the point: **the call site now states which convention it belongs to**, instead of leaving the next reader to trace the value back to the code that wrote it. `|lab_date()` = machine timestamp, convert. `|date()` = human-entered wall-clock, already lab time.

**The pass:** 48 machine-timestamp calls across 23 templates converted to `|lab_date()`, plus the 11 hardcoded `|date(…, 'Europe/Paris')` pins — **zero `Europe/Paris` left in `templates/`**. PHP: 15 hardcoded zones replaced by the setting across `AccessPass`, `AccessPassRepository`, `PersonAvailabilityService`, `NextFreeSlotService`, `ReservationService`, `IcalFeedService` and four controllers (a `labZone(SiteSettingService)` helper).

⚠️ **Date-only fields were deliberately left on `|date()`** even when they are convention A — `dueDate`, `expectedReturnDate`, `dateObtention`. A due date is a calendar day, not an instant; converting it would flip the day around midnight on any zone west of UTC.

⚠️ **The iCal feed had a trap that only appeared once the zone became configurable.** It emitted wall-clock under `TZID` with a hand-written `VTIMEZONE` carrying CET/CEST offsets and the EU daylight-saving rules. Swapping the TZID alone would have advertised, say, `America/New_York` against Paris's offsets. It now emits **UTC** — no VTIMEZONE, unambiguous for every client, and immune to a country changing its DST rules.

⚠️ **`clock_controller.js` no longer hardcodes the zone either** — it reads `data-clock-timezone-value`, rendered from the setting. Empty falls back to the browser's own zone: on a wall display, the machine's local time beats the server's UTC.

**Verified by measurement, before and after.** The witness was `/kiosk/entries`, which shows a machine timestamp next to a clock: entry times read **08:41 before and 10:41 after**, while the clock, opening hours (`08:00`) and booking slots (`10:00`) did not move — convention B untouched, which was the whole risk of the pass. All 18 public pages 200; `app:render` confirms the field on both admin screens.

**One 500 in the middle of it:** `resolveLeaderboardPeriod()` gained a parameter its caller did not pass, so `/leaderboard` threw `ArgumentCountError` until the call site caught up. Found by sweeping every page rather than the ones that were touched — the pass changed private helper signatures, and their callers are not in the diff you are looking at.

---

### S39 · RFID device auth that fails closed — ⚠️ sequencing matters more than the code

**Why.** `RfidMachineController::rejectUnauthorizedDevice()` reads `FABOS_RFID_API_TOKEN` and, when it is empty, **returns `null` — which means "allowed"**:

```php
if ($expectedToken === '') {
    return null;
}
```

The variable is unset in both `.env` and `.env.local` on CT 210, so **the RFID device endpoints currently have no authentication at all**. They are not read-only: `POST /api/rfid/machines/{token}/authorization` decides whether a badge may use a machine, and the work-session routes start and stop machine time.

**Scope.** Invert the default so an unset token refuses instead of allows, provision a real token, and write down how a reader gets one.

⚠️ **This is the fail-open/fail-closed rule from §3 of the handover, applied to a place that got it backwards.** Config stores fail *open* so a config problem cannot take the lab offline; a credential check must fail *closed*, because the failure mode of guessing is that anybody is a device.

⚠️ **Deploy order, and it is the migration hazard in a different costume: provision the readers FIRST, flip the code SECOND.** Landing fail-closed before every reader carries the token locks every door and machine in the building at once. If the readers cannot yet be provisioned, ship the token check as *warn-and-log* first and flip it after — but do not leave it warning forever, which is where this finding came from.

**Verify.** With the token set: correct header → 200, wrong or missing → 401. **Then the operator badges in on a real reader** — this is not verifiable from a shell.

---

### S40 · The two remaining ways in

**Why.** `security.yaml` has no `login_throttling`, so `/login` accepts unlimited guesses at whatever rate an attacker can manage. And `LOCAL_ADMIN_BYPASS=1` is still sitting in `.env.local` — inert today only because the site runs `prod` and the class it belonged to is deleted, but it is a live-looking switch that means nothing, which is exactly the kind of thing someone re-enables.

**Scope.** Add `login_throttling` to the main firewall. Delete the `LOCAL_ADMIN_BYPASS` line from `.env.local`.

**Verify.** Repeated failed logins start being refused; a correct login still works first time. `app:render` is unaffected — it never used that flag.

---

### S41 · The queries that read whole tables

**Why.** Several hot paths load a full table into PHP and aggregate by hand instead of asking SQL: `/formations` (`SiteController:1860`), `/search` and `/api/search` (the same logic written twice, `SiteController:1466-1515` + `ApiController:337-389`), the leaderboard's all-time tab (`LogUtilisationRepository::computePresenceMinutesByUser`), and admin usage-logs, which has no pagination at all. `LOG_UTILISATION` — the fastest-growing table in the app — **has no index on `dateDebut`** despite being filtered and sorted on it everywhere.

**Why now rather than later.** It currently holds 2 rows. Every one of these is cheap to fix today and a production incident to fix at 500 000.

⚠️ **The index is a migration, so it is an expand: migration first, code second** (§7). Adding an index is safe to run ahead of the code that benefits from it.

**Scope.** Push the aggregation into SQL, paginate usage-logs, add the index, and collapse the duplicated search logic into one place.

**Verify.** Same output before and after — snapshot the rendered lists and diff them, exactly as S24 did for the nav. Not "it looks right".

---

### S42 · A test suite that exists

**Why.** `tests/` contains one file: `bootstrap.php`. There is **no automated coverage of anything**, on a codebase that now carries a booking engine with three independent permission layers, a polymorphic reservation model, portal scoping with tri-state inheritance, and fail-safe repositories whose whole contract is what they do when the database is unavailable. Every regression this project has caught, it caught by hand.

**Scope.** Start where tests are cheapest and most valuable — pure logic with no database:

- quota ordering in `BookingPolicyService` (coarsest-constraint-first is a *deliberate* order and nothing enforces it)
- `ReservableRef` / `ReservableResolver`
- `PortalOverrides::save()` validation and the all-or-nothing rule
- `SiteFeatureService::stateForScope()` tri-state, and add-ons forced off by parent
- `FirstRun::isFresh()` — including the "flag absent but install has content" case, which is the one path that could not be exercised against production
- `MissingPageSubscriber`'s filters
- `VocabularyTranslator` fallback when settings are unavailable

**Watch.** Do not chase a coverage number. The list above is chosen because each item is a rule someone could plausibly "simplify" away, and the test is what says no.

**Verify.** `vendor/bin/phpunit` green, and it runs without a database.

---

### S43 · Delete what is dead, and clear the deprecations

**Why.** Dead code is read by the next person as if it matters. `src/Entity/TlseUser.php` and `src/Entity/UserBadge.php` are both orphaned — `UtilisateurBadge` is the real one — and both are still present. Two deprecation warnings appear on every cache rebuild: `UtilisateurBadge::$utilisateur` declares `nullable` on a join column that is part of the identifier (a no-op that becomes an error in Doctrine 4), and every DDL migration warns about committing an already-committed transaction (MariaDB commits DDL implicitly; the fix is `transactional: false`).

⚠️ **Watch — `resources/database/` is stale but not unused.** It is still read by `app:fabos:seed` and `ImportGuidedSectionsCommand`, and it has drifted from `migrations/sql/`. Reconcile or retire those commands deliberately; do not delete the directory because it *looks* obsolete.

**Verify.** App boots, `cache:clear` is silent, and whatever the seed commands are supposed to do, they still do or are gone on purpose.

---

### S44 · Pay off the verification debts

**Why.** These are the things this project has been honest about not knowing, and they have outlived several sessions.

- ⚠️ **The booking success path has still never been verified end to end.** Every refusal branch is tested; the happy path is assumed. It needs a login, so it needs the operator — **this is the oldest open item in the whole plan.**
- **50 of the 58 admin pages have never been looked at** in either theme. S29 audited 8. The harness exists and is written down in `PROJECT_STATE.md` §9.
- **Public pages have never been contrast-audited at all** — S29 only ever pointed the tool at `/admin`.

**Scope.** Run the existing harness over the remaining admin pages and over the public site; have the operator click one real booking.

**Verify.** It is the verification. Record what was checked, so the next session inherits a smaller list rather than the same one.

---

### Also outstanding, tracked in their own entries

Deliberately not duplicated here — each is written up where it belongs, with its reasoning:

- **S27** — add-ons are not per-portal settable; the portal logo is a filename rather than an upload.
- **S28** — portal-scoped homepage blocks (an ORM entity, so migration-first).
- **S30** — no "add" chips on list pages; no inline check-in (needs the staff/admin split settled, given there is no role hierarchy).
- **S31** — five `@ensea.fr` email placeholders, left because a domain is not a name.
- **From the 2026-07-10 audit, smaller items:** CSRF on `MachineFavoriteController` favourite add/remove; `.env` committed to git (placeholders today, a bad habit to keep); the "who voted" panel not wired to the mini-cards or the ranking page, and the open privacy question of whether individual scores should be public at all.

---

