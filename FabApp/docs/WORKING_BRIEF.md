# FabOS — working brief

**Read this first.** It is the short operational memory for a new work session. Use the linked documents when a task needs their detail; do not re-derive their decisions.

## General rules

- **One source of change.** Reuse shared services, templates, components, styles and configuration. If several pages need the same behaviour or design, extend the shared source instead of copying it.
- **One design system.** `/admin/design` is the reference for recurring UI patterns. A new recurring pattern belongs there and in the shared design layer, not in one page's inline style.
- **The shortest honest path wins.** Minimise clicks and choices; keep advanced complexity out of the normal path. Never show a control that does nothing.
- **A session is not done locally.** Update docs, commit, deploy the selected files to Artemis CT 210, and verify the running result there.
- Development mode is a temporary Artemis workspace switch, off by default. It only reveals admin navigation; it never changes authentication. Turn it off — and remove its menu if this install is promoted — before production.

## Product and architecture

- FabOS is a Symfony 8.1 / PHP 8.4 / MariaDB application, rendered by Twig. The app is **only** `FabApp/`; the monorepo siblings are unrelated or historical.
- The calendar is the spine. Reservations refer to `(reservableType, reservableId)` and currently cover machine, place and user. Add a kind through `ReservableType`, `ReservableResolver`, booking access and both calendar builders.
- Booking has three deliberately separate layers: **certification** (safety), **quotas** (fairness), and **access passes** (quota exemptions only). Do not turn a pass into a safety bypass.
- Site features are the one registry for operator-facing modules and route gates. A hidden route is not enforcement: writes must be protected at the service chokepoint too.
- Five translation catalogues stay in lockstep: `fr`, `en`, `de`, `es`, `it`. Use operator-configurable vocabulary, never hardcode the organisation, venue, “machine” or “member”.

## Time, data and safety

- The server runs UTC; the operator configures the lab timezone. There are two date conventions: **machine timestamps** use `|lab_date`; **human-entered wall-clock values** use plain `|date`. Classify by entity, never by a field name.
- Configuration repositories have intentional fail-open or fail-closed behaviour. Preserve the existing direction; a database problem must not accidentally grant access or take the lab offline.
- There is known entity/migration drift. **Never run `doctrine:schema:update --force`.** For a mapped ORM column, migration first; for a fail-safe DBAL feature, code can ship first.
- Reservations have no cascade. Any delete path for a reservable resource must explicitly cancel its upcoming bookings.

## Design and front end

- `site/base_public.html.twig` is the shared public layout. Extend it for normal public pages; do not copy a `<head>`.
- Admin lists and tables use the shared list/table shells. A recurring admin pattern belongs in `admin.css`, shared partials and `/admin/design`.
- AssetMapper and Stimulus are live. **Turbo is off.** New interaction goes in a Stimulus controller unless native HTML suffices; do not add new inline scripts.
- Any stylesheet change needs its emitted `?v=` cache buster updated. Template/translation changes need cache clear; JS changes also need `importmap:install` then `asset-map:compile`.
- Rendered markup is not visual verification. For an affordance, responsive layout or contrast claim, inspect the running page and measure it in a browser.

## Artemis deployment and verification

- Follow the complete macOS-safe procedure in [ARTEMIS_DEPLOYMENT.md](ARTEMIS_DEPLOYMENT.md); do not reconstruct the tar/push/extract commands from memory.
- Artemis CT 210 is the operator's review environment; app root is `/opt/fabos/FabApp`.
- Connect to the Proxmox host with `ssh -i ~/.ssh/id_ovh -p 4002 artemis.dryades.org`. The private key remains outside the repository; never copy it into FabOS. The SSH account is unprivileged: run container commands as `sudo pct exec 210 -- bash -lc "…"` and use `sudo pct push` for narrow archives.
- Artemis is a deployment and verification target, not a coding workspace: only Codex sessions deploy there and nobody edits application code directly on CT 210. Keep the pre-deploy comparison because it detects an earlier Codex deployment that was not carried back to the local checkout, not concurrent operator development.
- Deploy **narrow archives only**: tar the intentional paths, `pct push` them, then extract at `/opt/fabos`. **Never use `deploy.sh`** and never sync/pull the whole checkout: CT 210 has hand-deployed divergence.
- Lint before cache clear/restart and read the output. Clear `var/cache/prod` / run `cache:clear` for every deployment. Restart `fabos.service`; restart `fabos-worker.service` too after mail-code changes.
- For a migration with mapped entities: push/extract just the migration, have the operator migrate, then deploy code. A rollback overlays an archive and removes genuinely new files, then clears `var/cache/prod`.
- Verify anonymously where possible. For privileged pages use `app:render` when available, and ask the operator to test a real signed-in flow when that is the only honest test.

## Current position — 2026-08-09

- Latest recorded product decision is **S102**, refined by the operator decisions recorded for S103. It supersedes the portal-shaped parts of S100–S101 before schema work: a service needing independent administration/theme/data gets its own FabOS; sub-locations exist only inside one shared governance/data set and aggregate by default. LDAP/OIDC/SAML may share authentication, never rights or data; local accounts/groups/packages/audit remain authoritative and selected data crosses instances only through the FabOS network. Packages carry only Use/Manage grants scoped by sub-location and feature; Manage includes reporting on its scope and never grants Use. Configuration will expose one versioned Themes workspace for colours, images, public name, menu labels/order and homepage blocks/content/order. The live S97–S99 model is unchanged and enforcement remains off. Physical cards/readers remain explicitly deferred.
- Full-access packages are durable: they include future audited capabilities automatically. Ordinary edits preserve grants for temporarily disabled site features instead of deleting them.
- Latest delivery is **Phase B (S109–S111)**: protected groups plus packages/grants v2 et attributions directes/groupe sont persistés en shadow mode. Next phase: **S112–S117**, shared workspaces. All authorization changes stay in shadow mode before activation.
- Guest is now the anonymous audience; visibility and registration/action are separate tri-state event policies inheriting FabOS defaults. The seven protected built-ins include Formateurs; User is every active account and Guest is anonymous, without explicit memberships. Packages default-deny and accumulate grants from the individual and all groups. Institutions use one canonical unique HTTPS origin and remain descriptive until secure FabOS discovery plus explicit trust. Personal sharing requires instance allowlisting and member consent; non-personal catalogues use publication/trust policy. Badge awards are revocable, never deleted, and imports propagate expiry/revocation. Materials are a shareable FabOS catalogue with local sub-location availability/stock. These are target decisions only; live S97–S99 behaviour is unchanged.
- The five former operator questions are decided: Admin recovery never bypasses qualification or physical shutdown; only Use/Manage exist; Formation definitions are global and physical sessions are sub-location scoped; public identity surfaces require per-surface operator policy plus member consent while necessary internal views follow authorization; known IdP disablement revokes immediately and an outage grants existing sessions at most 24 hours.
- Future S129–S133 adds optional Commerce after the core audit: one offer/order/payment/refund/fulfillment engine for package assignments, materials and machine/person/training-time credits. Verified provider events plus per-line outbox/fulfillment make delivery effectively-once; stock is held atomically and time uses an append-only ledger. Refund and domain compensation remain separate. Payment is never itself a permission or booking and cannot bypass the safety/access layers. Pool booking and physical-card enforcement remain deferred.
- Much later, S134–S136 adds Training communications only after formations, sessions/cohorts, enrollment and permissions are stable. Announcement, one-learner private thread and explicit group are separate immutable visibilities. FabOS stores bounded/rate-limited plain text canonically and sends a per-recipient asynchronous e-mail copy through the existing mail chokepoint. The worker revalidates access immediately before delivery; mail failure never loses the internal message. No inbound e-mail reply, real-time chat or attachment support is implied.
- Real verification gaps remain: booking happy path needs an operator account/real rows; role surfaces need a staff-but-not-admin account; some responsive/dark states remain unmeasured.

## History worth carrying forward

- The site is now `APP_ENV=prod`; the old HTTP local-admin bypass is inert. Do not reintroduce request-reachable passwordless access.
- `Mail\Mailer` is the send chokepoint. The worker is long-running and must be restarted after mail changes.
- `machineToken` is device-facing information: never expose it publicly. RFID device auth must fail closed once its token is configured.
- PNG orientation can live in its `eXIf` chunk; `getimagesize()` and `exif_read_data()` alone are insufficient. Reuse `ImageNormalizer` / existing orientation logic for image work.
- Feature gates affect navigation and pages, but service-layer enforcement protects writes. Role hierarchy does not exist: use `can_reach()` for affordances and the firewall for access.
- Compiled Twig, translations, AssetMapper assets and browser caches can make a correct deploy appear unchanged. Verify the emitted cache-buster and the running page, not just the source.

## Reading order and end-of-session checklist

1. Read this brief, then the current [Roadmap](ROADMAP.md).
2. Read [Project State](PROJECT_STATE.md) before architectural or deployment work.
3. Search [History](HISTORY.md) for the session/subsystem being changed; it contains the traps and rationale.
4. Before handoff: update the relevant docs, lint/validate, commit, deploy narrowly to Artemis, verify the live result, and record what was actually verified.
