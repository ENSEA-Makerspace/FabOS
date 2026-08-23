## Phase D — Training / LMS, built for beginners

*The operator's note: "training looks complicated — as many simple pages as possible while hiding the complexity."*

### The key structural insight

**A training session is an event with a curriculum attached.** Do not build a parallel registration system. The event engine already provides — and has been verified to provide — enrolment, capacity, waitlists with automatic promotion, per-attendee tickets and QR codes, door check-in with attendance records, reminder mail, and organiser cancellation with a reason. Reusing it means the LMS inherits all of that on day one, and there is exactly one concept of "signing up for a thing at a time".

**Attendance already equals completion evidence.** The check-in timestamp built for events is precisely what a trainer needs to mark someone as having attended.

### Progressive disclosure — four levels, three toggles

Default is level 1. Each level is one switch, and nothing above level 1 appears until asked for.

| Level | What the admin does | What they get |
|---|---|---|
| **1 · A page** *(default)* | Title, description, image | A readable catalogue entry. Genuinely useful alone. |
| **2 · Sign-ups** | One toggle | A roster. Reuses event registration wholesale. |
| **3 · Scheduled sessions** | One toggle | Dated instances with capacity, waitlist, reminders, check-in. |
| **4 · Advanced** *(collapsed)* | Open "Advanced" | Awards a badge on completion · prerequisites · quiz · assigned trainer |

### S32 · Training content (level 1)

Revive training as a plain content type: title, description, image, category, duration, objectives, prerequisites-as-text, materials provided. A catalogue and a detail page. `TrainingEnrollment` is a **neutralised stub** and `Formation` has no date — treat this as a fresh, careful model, not a resurrection.

### S33 · Enrolment and sessions (levels 2–3)

A `TrainingSession` = a scheduled instance. Reuse the event registration engine for signing up.

#### One class, three calendars — settled 2026-07-24

The requirement: glance at a class and see **where** and **with whom**; look at the room's calendar and see the class sitting in it; look at the trainer's calendar and see it too.

**The answer is: duplicate the rows, but not the truth.** The session is the single source of truth for when/where/who; it **projects** an occupancy reservation onto each resource it consumes.

*Why not merge the model.* `RESERVATION` addresses exactly one target, `(reservableType, reservableId)`. Making that many-to-many would reach into overlap detection, both calendar builders, the quota engine, access passes and the booking mail — essentially everything built across S8–S20 — to serve one use case. The blast radius is not worth it.

*Why not "just display it".* If the room is not genuinely reserved, somebody books it out from under the class. A real reservation gets **collision detection for free**, which is the entire point of putting it on the calendar in the first place.

*So: one owner, N projections.* Creating a session with a room and a trainer writes two reservations — `place:{room}` and `user:{trainer}` — both stamped as **derived from that session**.

Rules that keep projections from rotting, each one preventing a specific failure:

- **Not individually cancellable.** Cancelling the room booking directly would leave a class with nowhere to happen. The cancel path must refuse and point at the session instead.
- **Re-synced on edit, cancelled with the session.** Moving the session moves its projections; calling it off releases them.
- **Skip quotas and passes, keep overlap.** A projection is not personal consumption, so it must not eat the trainer's weekly booking allowance — but it absolutely must still collide with anything else in that room. `ReservationService::book()` is the chokepoint, so this is a sibling `occupy()` path that runs the collision check and skips the quota layer, not a bypass of the chokepoint itself.
- **Label them.** `reservableLabel` already snapshots a name; the projection's `motif` should say which class, so the room's calendar reads "Formation: Laser 101" rather than an anonymous block.

*There is no third calendar.* The training view reads sessions directly — it needs no reservation at all. So this is one calendar with resource layers, plus a training listing, not three calendars to reconcile.

*Build it generic.* An event occupying the main hall is the same shape. Implement **resource occupancy** once, keyed by an owning record, and let both `TrainingSession` and `Event` use it — otherwise this gets written twice and drifts.

### S34 · Completion, badges and snapshots (level 4a)

Attendance or trainer confirmation completes an enrolment; completion may award a badge. **Credentials are immutable: snapshot on award** — freeze title, description, badge image, date and awarding trainer into the award record, so later edits to the live training never rewrite what somebody earned. This closes the loop with cert-gating: an earned badge is what unlocks a machine.

### S35 · Quizzes and prerequisites (level 4b)

Optional, hidden by default. Quiz model, pass/fail gating a badge, prerequisite chains. Build only once levels 1–3 are in real use.

---

## Phase E — Membership & billing

### S36+ · Memberships, then payments

Tiered dues, renewal, expiry, and access gated on active membership. Payments as **separate modules** per billable thing (memberships, event tickets, material purchases, machine time), designed against a provider but **never handling live payment credentials in this workflow** — the operator wires their own keys.

Also the gate for the event **price / paid-attendance** work deliberately left out of S20, and for material and consumable billing.

---

### S37 · 404s that help, and a record of what people are looking for — ✅ shipped 2026-07-30

*Logged 2026-07-28 at the operator's request, after hitting a disabled page. Migration `Version20260804100000` applied by the operator the same day.*

**✅ The cause, first: the live install now runs `prod`.** It had been on `APP_ENV=dev`, so "it throws an error" was literally Symfony's development exception page, in English, with `No route found for "GET http://fabos.dstei.fr/…"` sitting in an HTML comment — and the profiler answering to anyone. That was the whole of the reported bug; everything below is what makes the page actually good. See *What to do next* for the change and for the `LOCAL_ADMIN_BYPASS` consequence it dragged along.

**✅ A real 404 page.** The templates already existed and had never once been *seen*, because dev mode meant they never rendered. Two things were wrong with them and both mattered:

- **They were hardcoded French, and hardcoded to features.** The way out of the 404 was "Voir les machines" / "Voir les formations" — so on an install with equipment and training switched off, **every escape route from the 404 was itself a 404.** They now come from `NavBuilder::safeDestinations()`, which reads the footer: the suggestions inherit the menu's own rule that nothing switched off is ever offered. Wrapped in a `try/catch` down to the home link alone, because an error page that throws while explaining an error has nowhere left to go.
- **The wording said "something broke".** A disabled feature 404s *by design* — that is the gating model, not a fault — so the page now reads **"this page isn't part of this site"**, translated across all five catalogs (`error.*`). It deliberately stops there: *which* feature is off goes to the log for the admin, not onto a page anyone can probe.

**✅ The misses are logged** — `MISSING_PAGE`, raw DBAL, fail-safe, aggregated one row per distinct path with a counter, plus first/last seen and the last referrer. Screen at `/admin/missing-pages` (sidebar *Pages introuvables*), which sorts most-asked-for first and tags every row with **why**: a switched-off feature, a broken internal link, or a wrong URL out in the world. That distinction is the entire point — the first is not a bug, it is demand, and it is answered on the feature screen.

**Most of the work was deciding what *not* to record**, because a raw 404 log is bots. Three filters, each killing a whole class of noise:

1. **No route matched, or the feature gate refused.** A 404 from a controller that *did* match its route — reservation 4211 does not exist — is a missing *row*, not a missing page, and it would bury everything else.
2. **A referrer, or a known reason.** Probes for `/wp-admin` arrive without one; a person following a broken link brings the page they came from. The deliberate exception is a switched-off feature, where an old bookmark is exactly how somebody arrives and nothing links there any more — so `FeatureAccessSubscriber` now writes the feature key onto the request before throwing, and the reason survives into the log.
3. **Pages, not assets.** A dead stylesheet is a broken asset; mixing those in makes a list of missing *pages* unreadable.

**No scheduled prune, on purpose.** Aggregation already bounds the table by the number of *distinct* wrong URLs rather than by traffic, so size is not the problem relevance is: the screen has a "forget what has been quiet for 90 days" button and a clear-all, both POSTs behind a CSRF token.

**⚠️ One bug, found only because the table was there to check: the subscriber never ran.** It was registered at priority **−256**, deliberately below Symfony's `ErrorListener` (−128) so as not to interfere with the response being built. But `ExceptionEvent` extends `RequestEvent`, and `RequestEvent::setResponse()` calls **`stopPropagation()`** — so attaching the error page ends the event, and everything below −128 is skipped. `debug:event-dispatcher` listed the listener as present and correct the whole time; the only symptom was an empty table. Now −100. **The general rule: a `kernel.exception` listener that merely observes still has to run before the error page exists.**

**Verified on the live container**, migration applied: public 404 renders the real page with working links and no debug markers · `/_profiler` 404s · every public page still 200s · a miss with an internal referrer is recorded and its counter increments on a second hit · an external referrer is tagged as such · **a bot probe with no referrer is dropped** · a dead asset is dropped · a 404 from a controller whose route *did* match (machine 99999999) is dropped, because that is a missing row, not a missing page. Earlier, before the migration, the screen also rendered its empty state with the table absent — the fail-safe doing its job.

*(Two rows, `/probe-raw` and `/probe-service`, are left over from the throwaway command used to prove the store worked. Clear them with the screen's own **Vider le journal** button, which is also the only untested control on the page.)*

---

