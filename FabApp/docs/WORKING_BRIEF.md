## Position actuelle — 2026-08-17

✅ **Déployé et vérifié le 2026-08-19** : 109 fichiers identiques par hachage sur
CT 210, services redémarrés, site 200, **92 tests / 2 173 assertions**, 121 routes
GET balayées sans un seul 500. `?v=20260817-s144` confirmé par ce que les pages
émettent.

🔴 **DEUX MIGRATIONS ATTENDENT D'ÊTRE LANCÉES** — `Version20260817100000` et
`Version20260817110000` (S144b et S144c). Additives, ne suppriment rien. Le code
déployé fonctionne sans elles : il sonde le schéma avant de nommer une colonne et
échoue vers l'ancien comportement, donc les nouvelles dimensions n'existent
simplement pas tant qu'elles n'ont pas tourné.

```
ssh -i ~/.ssh/id_ovh -p 22 proxmox.lab.dryades.org 'sudo pct exec 210 -- bash -lc "cd /opt/fabos/FabApp && php bin/console doctrine:migrations:migrate --no-interaction"'
```

✅ **S144 — le système de packages est fini** (demande opérateur du 2026-08-17).
Un package sait maintenant dire **à qui** (un membre ou un groupe), **sur quoi**
(un type de ressource, une machine précise, une catégorie), **quand** (créneaux
hebdomadaires, « lundi 14:00–18:00 ») et **combien** (X heures ou X réservations
par jour / semaine / mois / total). Détail dans `HISTORY.md` § S144, état vivant
dans `ROADMAP.md` § S144.

⚠️ Les paiements ne sont **pas** dans S144 : la Phase H (S150–S154) reste
facultative et non commencée. S144 livre l'entitlement qui rend un package
vendable, pas le commerce.

## Position précédente — 2026-08-16

Branche `s129/venues-workspace` · arbre propre · déployé et vérifié par hachage
sur CT 210 · **rien poussé sur GitHub**.

⚠️ SSH : `proxmox.lab.dryades.org:22` (tailnet). `artemis:4002` **expire**.
✅ **Numérotation réparée le 2026-08-16.** Le commerce (Phase H) est passé de
S135–S139 à **S150–S154**, la messagerie (Phase I) de S140–S142 à **S155–S157**.
Les numéros livrés n'ont pas bougé. **Prochain numéro libre : S140.**

Livré : G3 (S134h/i/j) · S135 · S136 · S134c2 · S137 · S138a/b · **S139 en entier
(a/b/c/c bis/d)** · **S134g complet (les deux moitiés)**. Détail dans
`HISTORY.md`.

✅ **44 routes legacy supprimées** le 2026-08-16 : 163 routes → 119. Un seul
non-2xx sur tout le site, `/.well-known/fabos`, qui rend `503 unconfigured`
volontairement.

✅ **S134g est complet** (moitié 2 livrée le 2026-08-16) : anonymisation
irréversible, statistiques et créations intactes, dernier administrateur refusé.
Décision opérateur : « stats should stay, bookings and all… maybe just his name
gets changed? »  🔴 **Un 500 en production le même jour** — `setPublicFields()`
prend `array`, on lui passait `null` — a révélé que l'érasure pouvait se faire à
moitié (les scrubs SQL bruts valident immédiatement). Corrigé, transaction
ajoutée, vérifié par un vrai scrub annulé sur CT 210. Détail dans `HISTORY.md`.

⚠️ **Leçon à retenir avant de toucher à `AccountAnonymiser`** : vérifier qu'un
setter *existe* n'est pas vérifier qu'il *accepte* ce qu'on lui passe. Un test
par réflexion confronte chaque `setX(null)` à la vraie signature — il est là,
gardez-le.

Ouvert, par ordre :
1. Les 21 autres gabarits d'e-mail (⚠️ vérifié le 2026-08-16 : les 23 gabarits
   passent déjà par les cinq catalogues et les **97 clés `mail.*` existent dans
   les cinq langues** ; le rendu par destinataire est déjà correct via
   `LocaleSwitcher`. Ce qui reste à revoir n'est donc PAS l'i18n).
2. S138c — passer `frame: 'full'` et `hero: 'compact'` en défaut, supprimer les
   drapeaux. Décision opérateur.
3. Renommage « lieu ». Mot non choisi.
4. S134d/S134e (⚠️ migration opérateur) · S134f · S134b.

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
- ⚠️ **Translate the UI. Never translate content.** FabOS's own words — labels,
  headings, buttons, explanations of how the product works — are UI and belong in
  the catalogues. Anything an operator writes about *their* thing is content: a
  training, a lab page, a legal notice, an event description. **A training exists
  in one language and that is it** (operator, 2026-08-11). The test is not where
  the string lives but who is speaking. Where FabOS ships a default that an
  operator will overwrite, translate the default and never the override — see
  `FormationPageContentService` for the worked pattern.

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
- Connect to the Proxmox host with `ssh -i ~/.ssh/id_ovh -p 22 proxmox.lab.dryades.org`. ⚠️ **The old `artemis.dryades.org:4002` is dead and TIMES OUT rather than refusing** — a transfer that hangs is this, not a slow copy. SSH moved to port 22 behind a tailnet on 2026-08-12; the public site was unaffected, so if SSH hangs while `https://fabos.dstei.fr` answers 200, suspect the access path and not the app. The private key remains outside the repository; never copy it into FabOS. The SSH account is unprivileged: run container commands as `sudo pct exec 210 -- bash -lc "…"` and use `sudo pct push` for narrow archives.
- Artemis is a deployment and verification target, not a coding workspace: only Codex sessions deploy there and nobody edits application code directly on CT 210. Keep the pre-deploy comparison because it detects an earlier Codex deployment that was not carried back to the local checkout, not concurrent operator development.
- Deploy **narrow archives only**: tar the intentional paths, `pct push` them, then extract at `/opt/fabos`. **Never use `deploy.sh`** and never sync/pull the whole checkout: CT 210 has hand-deployed divergence.
- Lint before cache clear/restart and read the output. Clear `var/cache/prod` / run `cache:clear` for every deployment. Restart `fabos.service`; restart `fabos-worker.service` too after mail-code changes.
- For a migration with mapped entities: push/extract just the migration, have the operator migrate, then deploy code. A rollback overlays an archive and removes genuinely new files, then clears `var/cache/prod`.
- Verify anonymously where possible. For privileged pages use `app:render` when available, and ask the operator to test a real signed-in flow when that is the only honest test.

## Current position — 2026-08-16

**Branch `s129/venues-workspace`, 54 commits, nothing pushed to GitHub.**
Everything is deployed to CT 210 and hash-verified. **64 tests / 2 096
assertions.** 139 paths render; the only non-2xx are `/.well-known/fabos` 503
(instance identity unset, deliberate) and `/desabonnement` 400 (no token).

**Shipped since the 08-11 section below:** S132b · **S134c** (five languages) ·
**S134g** (password reset, then irreversible anonymisation) · **S135–S138b**
(one object vocabulary, public card grids) · **S139a–e** (search covers ten
catalogues + destinations; **44 legacy routes deleted**) · **S140–S141** (the
merged list card becomes the only admin list format, on all 36 list pages).

⚠️ **`ROADMAP.md` was emptied of everything shipped on 2026-08-16.** It is now
pending work only, ~290 lines; the one-line-per-session record is the index at
the end of `HISTORY.md`. **Start there, not here** — this section is a running
log and the roadmap is the plan.

⚠️ **Two things this file claimed as owed that are in fact done** (checked in
the code 2026-08-16): the **sub-venue field on the machine/place/event/loanable
forms** — Phase G's exit criterion — and both dated regressions
(`admin-events`' UTC "À venir", the raw `admin_list.all`). What is still owed
from S132 is Réglages / E-mails / Fonctionnalités, and from S133 the machine
category CRUD, the Événements "Ailleurs / externe" option and the Packages
surfaces.

## Position au 2026-08-11 (log)

**Phase G is in progress on branch `s129/venues-workspace`. Everything below is
committed, deployed to CT 210 and verified there.** Read `ROADMAP.md` Phase G and
Phase G2 before starting.

- **S129 ✅** Lieux workspace operable — venue list/create/edit/archive, `VenueGuard`
  owns the archive verdict, two invariants (default venue and last active venue can
  never be archived). No migration: `VENUE` already had every column.
- **S130 ✅** admin navigation de-duplicated — "Le lieu" grab-bag split into Lieux,
  Utilisateurs, Packages, Réseau; Matériaux moved under Équipement; 73 admin routes
  before and after.
- **S131 ✅** sub-venue context on the screens that *store* one (Horaires, Objets
  prêtables). `VenueContext::single()` is the contract for venue-scoped **editing** —
  it never answers "all", because "all" names no row to write.
- **S130b ✅ — 2026-08-11.** **One sub-navigation.** All twenty admin pages drew
  **two**: `NavBuilder`'s strip and `FeatureWorkspaceRegistry`'s tabs. They
  disagreed about what Équipement contains (6 entries vs 8), about which parent
  owns Institutions (Badges vs Réseau), and — only one of the two read the
  catalogues — **rendered in two different languages on the same screen**. Six
  live screens were reachable *only* from the tabs. The tabs are gone; those six
  are now in `NavBuilder`; the labels are catalogue keys in all five locales; a
  section with one entry draws no strip. Shipped alongside: the footer moved into
  `base.html.twig` (27 of 50 pages ended in whitespace), the users-list "Modifier"
  that opens a read-only detail page, the loan object made clickable, and the two
  duplicated buttons on Lecteurs RFID.
- **S130d ✅ — 2026-08-11.** **Réseau FabOS moved into Configuration** (operator's
  call, and the right one). It was a top-level section holding exactly one screen,
  so it took a whole sidebar row — level with Équipement's eleven — to say one
  thing, and under S130b's one-entry rule it drew no strip at all. What it
  configures is this instance's identity, its OIDC providers and its trusted
  peers: settings, not a workspace anyone works in. It sits after E-mails and
  before Configuration initiale, which stays last because it is the entry you use
  once. Side effect worth having: `/admin/network` now *has* a sub-navigation.
- **S130c ✅ — 2026-08-11.** **The sub-venue filter is a one-click tile row.** It
  was a `<label>` + `<select>` + right-aligned sentence in a full-width bar *above*
  the panel holding every other filter — so the one control that scopes all the
  others looked like it belonged to another screen, and cost two clicks and a menu
  where categories and status cost one. Same `ml-filter-group-label` / `ml-cats` /
  `ml-tile` markup as the rest now, first group inside the filters panel. **It
  renders nothing at all on a single-venue install**, and that test lives in the
  partial so neither call site can forget it — `admin-opening-hours` had it from
  S131 and the admin list shell never did.
  - 🔴 **`allow_all|default(true)` was always true.** Twig's `default` fires on any
    *empty* value, and `false` is empty. `/admin/horaires` passes `allow_all: false`
    and had offered "all sub-venues" since S131 — the one option its own comment
    says must not exist there. Choosing it does not fail: `VenueContext::single()`
    reads `all` as "not specified" and falls back to the default venue, so the
    operator edits the default venue's week while the URL says `location=all`.
    Use `allow_all is not defined or allow_all`.
  - ⚠️ **`admin.css` line 1 is `@import url("machines-list.css")`.** That file owns
    `.ml-tile` — every one-click filter on ~41 admin pages — and is loaded on all of
    them **without appearing in any template**. Grepping rendered HTML for its name
    finds nothing and is *not* evidence it is absent; I spent a detour concluding
    the tiles were unstyled because my local mirror of the stylesheets omitted the
    imported file. The import now carries its own `?v=`: it is fetched at a URL the
    page's buster never touches, so editing it needs **both** busters bumped.
- **S132 ⬅️ partial.** Measured: 59 admin templates held 1 322 lines of local
  `<style>` and **zero** were purely redundant. Shipped: the copy-pasted rules,
  the sidebar/flash/table/creations component reclaim, the RFID pairing modal
  extraction. **Still owed:** the Réglages / E-mails / Fonctionnalités rebuilds.
- **S133 ⬅️ partial.** Shipped: the sub-venue field on machine/place/loanable/event
  forms (Phase G's exit criterion), plus the two dated regressions. **Still owed:**
  the rest of S133's parity list.
- **S134c ✅ — done, 2026-08-11.** Twelve batches. **~1 015 hardcoded strings at
  the start; what remains is noise plus what was deliberately left as content.**
  `debug:translation` reports 0 missing in `messages` for all five locales;
  `parity.py` reports 0 MISSING; 30 tests / 218 assertions pass on CT 210.

### The four things S134c actually taught, none of them about translation

1. 🔴 **`scan_hardcoded.py` reads text nodes and a few attributes of
   `templates/*.twig`, and nothing else.** Not inline `<script>`, not
   `public/js/*`, not `{% block title %}`, not PHP, not Twig expressions. **Three
   separate times a screen was recorded here as "done" and was not** — including
   the dashboard, whose most prominent panel was French in five languages for two
   batches after it was declared clean. `tools/i18n/README.md` now carries the
   greps for every blind spot. **A page is done when those greps are empty too.**
2. 🔴 **A literal is invisible to every measurement this project takes.**
   `debug:translation` only sees keys. That is why five complete catalogues
   coexisted with: 40 French strings across the two calendars' `<script>` blocks,
   35 in `public/js/`, 41 French browser tab titles, 14 in the dashboard
   controller's `sprintf` calls, and 101 inside Twig `|default()` expressions.
3. 🔴 **Storage vocabulary reached the screen in three places.** `disponible` and
   `idle` were two filter tiles for one machine state; `user.statut` printed raw on
   three pages. `Machine::getStatusKey()` / `statusFilterForKey()` is the pattern —
   an entity method returning the catalogue key, plus a query counterpart so a
   tile's count and its link cannot drift. `Utilisateur::getStatusKey()` copies it.
   **The remaining raw-vocabulary cells (`user.theme`, `log.category`,
   `log.template`, `log.color`) should be solved that way and not a second way.**
4. ⚠️ **The pre-deploy hash comparison earned its place twice.** CT 210 was serving
   `creation-new.html.twig` with its header deleted and nothing in its place, and
   an older `_creation_image.html.twig` from before thumbnails. Neither was in the
   local checkout. **Do not skip that step.**

### The rule that closed it

**Translate the UI. Never translate content.** See the product section above. It is
the operator's decision of 2026-08-11 and it settles what looked like 460 remaining
literals: `static/*` (mentions légales, conditions, confidentialité, documentation,
support, statut, roadmap) is one install's own public copy, and the
development-mode reference pages are internal documentation. **Neither is work.**

`FormationPageContentService` is the worked pattern for a shipped default an
operator overwrites: `DEFAULTS` holds keys where FabOS is speaking, literals where
the training is; `getContent()` translates **before** merging the stored block, so
their prose never passes through a catalogue.

### Next

**⚠️ Phase G3 — les listes (S134h–S134j) is the next phase.** Decided 2026-08-11,
specified in `ROADMAP.md`. The format is settled and visible in
`/admin/design#filtres`; what remains is applying it and fixing what the filter
rework made visible.

- **S134h** — apply the retained format to all 41 lists, from `_admin_list` and one
  filter component. Not 41 copies.
- **S134i** — a **column vocabulary** in `_data_table`, demonstrated in
  `/admin/design`: media, title+subtitle, state chip, meter/metric, date, actions.
  Each is a class and a partial, not a recipe re-typed per page.
- **S134j** — remap every list onto it and delete the local CSS. ⚠️ Measured
  2026-08-11: seven lists carry 70–91 lines of local `<style>` — `admin-badges` 91,
  `admin-utilisateurs` 90, `admin-reservations` 90, `admin-venues` 88,
  `admin-machines` 82, `admin-usage-logs` 76, `admin-formations` 73 (~590 lines).
  The rest are already near zero.

⚠️ **"Lieu" is a bad word** (operator, 2026-08-11) and needs replacing. It is
in all five catalogues, in filter labels, in `VenueContext` and in column headers.
**It is a catalogue rename, not a schema one** — `Venue`/`VENUE` stay. Do it in one
pass during S134i, with the chosen word, rather than drifting into it.

**Then, still open and unscheduled:**

- **S134c2** — FabOS invents a training's programme, sessions, objectives,
  prerequisites and materials when the fields are empty. Wrong in any language,
  which is why S134c left it alone. It has now been deferred twice, both times for
  visible admin work the operator asked for.
- The rest of Phase G2: **S134d/S134e** (one schedule truth), **S134f** (archive
  instead of hard delete), **S134g** (password reset).
- S132's and S133's unfinished items above.

**Still-true traps from earlier in Phase G:**

- ⚠️ **`strict_variables: true` is set only under `when@test`, not in prod.** A
  template reading a variable the controller does not pass resolves to null in
  silence. That shipped a data-corruption bug in S131 (fixed in S132b). Twig lint
  does not catch it; the test environment would.
- ⚠️ **A count that matches your expectation is not evidence** when the failure mode
  produces the same count. Assert the positive case: render with two venues and
  require the control.
- ✅ **Resolved by S130b: `NavBuilder` is the whole admin navigation.** The
  registry is metadata and nothing else now — its `TABS` map, `forRoute()` and the
  `feature_workspace()` Twig function are deleted, so there is no longer a second
  place to update. It still feeds `/admin/design/workspaces`. Add a destination in
  `NavBuilder::admin()` / `adminByFeature()` and give it an
  `admin_nav.entry.<route>` key in all five catalogues; `AdminNavCatalogueTest`
  fails if you forget either.
- ⚠️ **A navigation entry's parameters are part of its address.**
  `/admin/reservations` is the machine, space and user list depending on
  `reservableType`; `/admin/reporting/{workspace}` cannot even be generated
  without one. `adminItem(..., params: [...])` feeds both `isCurrent()` and
  `canReach()` — omitting them from the second silently deleted Reporting from
  two strips, because that catch exists to make dead routes vanish quietly.
- ⚠️ In the `edit` sidebar variant `shell.icons` is false, so the lit class is
  `active`, never `admin-nav-link active` — check `aria-current`.
- ⚠️ `_logo.html.twig` still falls back to `Logo_ENSEA.png` and `site_logo_path` is
  **not editable anywhere in the UI**. De-branding that fallback before the Themes
  media library exists would leave the site with no logo and no way to restore one.
  Operator has confirmed it belongs in Themes.
- 🔴 **`&mdash;` inside a catalogue value renders as the literal text `&mdash;`.**
  `|trans` output is escaped, so the entity is escaped a second time. Use the real
  character; a value that genuinely carries markup must be rendered `|trans|raw`.
- ⚠️ `catalogue.add_keys` inserts into an **existing** namespace block on purpose:
  appending a second `admin_emails:` at the end of a file silently shadows the
  first under YAML last-wins, and no lint catches that.

**Findings from the independent whole-site review (2026-08-11).** S130b closed
four of them: the sidebar/tabs disagreement about "Équipement", the users-list
"Edit" that opens a detail page, loan rows not linking to the object, and the
footer-less pages. ⚠️ **Two of the review's counts were wrong and the method is
why:** it said *12* pages lack the footer — measuring the running pages found
**27**; and it said *both* RFID screens duplicate their buttons — only
`admin-rfid-readers` did. Counting templates is not counting pages.

**Still open from that review:** Logs RFID dumps 100 rows and its `status` /
`reason` / `color` cells still print the stored words — the headers translate now,
which made the gap more visible, not smaller. Solve it with
`Machine::getStatusKey()`'s pattern, not a second way.

**Also logged during S134c, unscheduled:** the Pi pairing runbook bakes
`/home/subhen/…` into text operators read (now one `device_root` default in
`_rfid_pairing_modal.html.twig` rather than fourteen literals, but the paths are
real device paths — a device question); both RFID screens show the same two
buttons twice, three lines apart; quizzes and physical validations still have no
UI to create them; four `validators`-domain messages are French sentences used as
their own message id, so `debug:translation` reports them missing in all five
locales including French.

## Superseded — 2026-08-10

- Latest recorded product decision is **S102**, refined by the operator decisions recorded for S103. It supersedes the portal-shaped parts of S100–S101 before schema work: a service needing independent administration/theme/data gets its own FabOS; sub-locations exist only inside one shared governance/data set and aggregate by default. LDAP/OIDC/SAML may share authentication, never rights or data; local accounts/groups/packages/audit remain authoritative and selected data crosses instances only through the FabOS network. Packages carry only Use/Manage grants scoped by sub-location and feature; Manage includes reporting on its scope and never grants Use. Configuration will expose one versioned Themes workspace for colours, images, public name, menu labels/order and homepage blocks/content/order. The live S97–S99 model is unchanged and enforcement remains off. Physical cards/readers remain explicitly deferred.
- Full-access packages are durable: they include future audited capabilities automatically. Ordinary edits preserve grants for temporarily disabled site features instead of deleting them.
- **Phase F is complete (S127–S128).** S127 removed the retired portal model from code and Artemis: hostname resolution, portal-scoped settings/features/logs, portal templates and the `PORTAL` table are gone. Artemis was explicitly treated as disposable development data: one portal, two scoped settings and thirteen feature overrides were purged; packages would have been folded into the single site. Legacy `/admin/portals*` permanently redirects to `/admin/design/structure`. The irreversible migration is `Version20260810130000.php`; never reuse it for a production installation without an independently tested backup/restore plan.
- ⚠️ **Phase G (S129–S134b) is the current work and is blocking before Commerce.** Earlier bullets in this file numbered Commerce as S129–S133 and Training comms as S134–S136; `ROADMAP.md` renumbered them to S135–S139 and S140–S142 when Phase G was inserted, then to **S150–S154 and S155–S157** on 2026-08-16 after S135–S138 were spent on UI work. The roadmap is authoritative. A session that follows the old numbers starts Commerce, which the roadmap forbids until an operator can administer a sub-venue from the canonical interface and S134b has validated the cross-cutting cleanup.
- S128 then audited the shared core on Artemis: **30 PHPUnit tests / 208 assertions**, **188 Twig templates**, **29 YAML** translation/configuration files, and all **14 canonical shared-workspace routes** rendered HTTP 200 through `app:render`. The completed commits are `fd519c5` (S127) and `132aad2` (S128 docs/audit record). Next planned work is optional Commerce, **S129–S133**.
- Guest is now the anonymous audience; visibility and registration/action are separate tri-state event policies inheriting FabOS defaults. The seven protected built-ins include Formateurs; User is every active account and Guest is anonymous, without explicit memberships. Packages default-deny and accumulate grants from the individual and all groups. Institutions use one canonical unique HTTPS origin and remain descriptive until secure FabOS discovery plus explicit trust. Personal sharing requires instance allowlisting and member consent; non-personal catalogues use publication/trust policy. Badge awards are revocable, never deleted, and imports propagate expiry/revocation. Materials are a shareable FabOS catalogue with local sub-location availability/stock. These are target decisions only; live S97–S99 behaviour is unchanged.
- The five former operator questions are decided: Admin recovery never bypasses qualification or physical shutdown; only Use/Manage exist; Formation definitions are global and physical sessions are sub-location scoped; public identity surfaces require per-surface operator policy plus member consent while necessary internal views follow authorization; known IdP disablement revokes immediately and an outage grants existing sessions at most 24 hours.
- Future S129–S133 adds optional Commerce after the core audit: one offer/order/payment/refund/fulfillment engine for package assignments, materials and machine/person/training-time credits. Verified provider events plus per-line outbox/fulfillment make delivery effectively-once; stock is held atomically and time uses an append-only ledger. Refund and domain compensation remain separate. Payment is never itself a permission or booking and cannot bypass the safety/access layers. Pool booking and physical-card enforcement remain deferred.
- Much later, S155–S157 adds Training communications only after formations, sessions/cohorts, enrollment and permissions are stable. Announcement, one-learner private thread and explicit group are separate immutable visibilities. FabOS stores bounded/rate-limited plain text canonically and sends a per-recipient asynchronous e-mail copy through the existing mail chokepoint. The worker revalidates access immediately before delivery; mail failure never loses the internal message. No inbound e-mail reply, real-time chat or attachment support is implied.
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
