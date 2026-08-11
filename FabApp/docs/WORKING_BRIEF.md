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
- Connect to the Proxmox host with `ssh -i ~/.ssh/id_ovh -p 4002 artemis.dryades.org`. The private key remains outside the repository; never copy it into FabOS. The SSH account is unprivileged: run container commands as `sudo pct exec 210 -- bash -lc "…"` and use `sudo pct push` for narrow archives.
- Artemis is a deployment and verification target, not a coding workspace: only Codex sessions deploy there and nobody edits application code directly on CT 210. Keep the pre-deploy comparison because it detects an earlier Codex deployment that was not carried back to the local checkout, not concurrent operator development.
- Deploy **narrow archives only**: tar the intentional paths, `pct push` them, then extract at `/opt/fabos`. **Never use `deploy.sh`** and never sync/pull the whole checkout: CT 210 has hand-deployed divergence.
- Lint before cache clear/restart and read the output. Clear `var/cache/prod` / run `cache:clear` for every deployment. Restart `fabos.service`; restart `fabos-worker.service` too after mail-code changes.
- For a migration with mapped entities: push/extract just the migration, have the operator migrate, then deploy code. A rollback overlays an archive and removes genuinely new files, then clears `var/cache/prod`.
- Verify anonymously where possible. For privileged pages use `app:render` when available, and ask the operator to test a real signed-in flow when that is the only honest test.

## Current position — 2026-08-11

**Phase G is in progress on branch `s129/venues-workspace` (11 commits, all deployed
and hash-verified on CT 210).** Read `ROADMAP.md` Phase G and Phase G2 before
starting; they were both rewritten this session.

- **S129 ✅** Lieux workspace operable — venue list/create/edit/archive, `VenueGuard`
  owns the archive verdict, two invariants (default venue and last active venue can
  never be archived). No migration: `VENUE` already had every column.
- **S130 ✅** admin navigation de-duplicated — "Le lieu" grab-bag split into Lieux,
  Utilisateurs, Packages, Réseau; Matériaux moved under Équipement; 73 admin routes
  before and after.
- **S131 ✅** sub-venue context on the screens that *store* one (Horaires, Objets
  prêtables). `VenueContext::single()` is the contract for venue-scoped **editing** —
  it never answers "all", because "all" names no row to write.
- **S132 ⬅️ partial.** Measured: 59 admin templates held 1 322 lines of local
  `<style>` and **zero** were purely redundant. Shipped: the copy-pasted rules (54
  copies over 36 templates), the sidebar/flash/table/creations component reclaim, one
  footer in `_admin_list`. **Still owed:** RFID pattern extraction into
  `/admin/design`, and the Réglages / E-mails / Fonctionnalités rebuilds.
- **S133 ⬅️ partial.** Shipped: the sub-venue field on machine/place/loanable/event
  forms (Phase G's exit criterion — nothing could be filed into a second venue before
  it), plus the two dated regressions. **Still owed:** the rest of S133's parity list.
- **S134c ⬅️ in progress, second batch shipped 2026-08-11.** Seven admin screens are
  now at zero hardcoded strings and verified on Artemis: dashboard, Réglages,
  E-mails, détail utilisateur, formulaire lecteur RFID, badges, inscriptions à un
  événement. **222 keys × 5 languages added; `debug:translation` reports 0 missing
  in the `messages` domain for all five locales.**
- **S134c third batch shipped 2026-08-11.** Six more screens at zero: liste des
  lecteurs RFID, assistant de configuration initiale, dérogations de quota, parcours
  de formation (`_formation_journey`, the public training page), éditeur de quiz.
  **138 more keys × 5.** The pairing runbook was **extracted into
  `_rfid_pairing_modal.html.twig`** — the two RFID screens each carried the whole
  nine-step modal and its open/close script, ~45 lines duplicated byte for byte.
  That is the RFID extraction S132 still owed. The `/home/subhen/…` device paths are
  now one `device_root` variable instead of fourteen literals in two files.
- **S134c fourth batch shipped 2026-08-11.** Four more screens at zero: éditeur de
  contenu de formation (the 92-literal one), profil, édition d'un événement,
  calendrier machine. **153 more keys × 5.**
  - 🔴 **FabOS ships five languages; a member could only ever pick two.** The profile
    language selector and its save path in `SiteController` both carried a retyped
    `['fr', 'en']`, as did the admin's create-user form — while
    `config/packages/translation.yaml` enables five and every catalogue has five. A
    German, Spanish or Italian member could not choose their own language, and a POST
    that tried was rejected with "Langue invalide." Four copies of the list existed;
    there is now one, `App\Service\LocaleCatalog`, fed from
    `%kernel.enabled_locales%` — the same value the framework itself reads. Verified
    on the running site: five options on `/profil`, `/admin/utilisateurs/new` and
    `/admin/settings`.
  - `UserAdminType`'s `locale_choices` is **required**, not defaulted: a default
    would let a future caller silently re-create the two-language bug.
  - `FormationContentAdminController` returned French labels for quiz types; it
    returns `typeLabelKey` now, because a literal in PHP is a string no catalogue
    reaches.
- **S134c fifth batch shipped 2026-08-11.** Eleven more screens at zero, deployed and
  verified on Artemis with `app:render`: Logs RFID, section de formation
  (création/édition), Interface accueil, Thèmes, Gestion des réservations, détail
  d'un badge, détail d'une machine, suivi de formation, Gestion des formations,
  Prêts, Utilisations machines. **206 keys × 5.** `debug:translation` still reports
  0 missing in `messages` for all five locales.
  - 🔴 **The machine detail page printed French status constants next to a
    catalogue that already had them.** `machines.st_available` /
    `st_maintenance` / `st_broken` existed in five languages and the machines
    *list* used them; the *detail* page computed `'En maintenance'` /
    `'En panne'` / `'Disponible'` inline from `machine.statut`. Same page, two
    vocabularies. It reads the catalogue now.
  - Two `fsui` values still told the reader to import "le fichier SQL fourni avec le
    patch" — the patch-instruction family logged in the fourth batch, in the
    catalogue rather than in a template, which is why the earlier sweep missed them.
    They now say what is true of an installed FabOS.
  - Twig plurals (`{{ n }} log{{ n > 1 ? 's' : '' }}`) became `%count%` parameters on
    six screens. The Twig form cannot be adapted by a translator at all.
- **S134c sixth batch shipped 2026-08-11.** Five more at zero, deployed and verified:
  publier une création, modifier une machine, horaires d'ouverture, détail d'une
  formation, calendrier. **79 keys × 5.**
  - 🔴 **The calendar's booking flow was French in all five languages.** Twenty
    strings lived inside `calendrier.html.twig`'s inline `<script>` — the slot
    states, the four validation messages, "Réservation refusée", "Semaine ${n}".
    A comment in that block already named the gap and asked that no new string
    join them. They come through one `T` object built from the catalogue now,
    the same way `MANAGE_LABEL` already did. **A string a translator cannot reach
    does not show up in `debug:translation`** — five complete catalogues and a
    French-only page are not contradictory. Grep the `<script>` blocks.
  - 🔴 **A public course page was headed "Synthèse MariaDB"**, with rows captioned
    "Depuis PROGRESSION", "completed = 1", "completed = 0". Both keys are dropped
    from all five catalogues, not renamed.
  - ⚠️ **CT 210's `creation-new.html.twig` had diverged from the local checkout**
    — the pre-deploy hash comparison caught it. Artemis was serving that page with
    the copied header *deleted* and nothing in its place: no logo, no navigation,
    no sign-in block. The local copy still had the copy. Neither survives: the page
    extends `base_public` now. This is the case the comparison step exists for.
- **S134c seventh batch shipped 2026-08-11.** Six more at zero, deployed and
  verified: modération créations, Fonctionnalités, créer une machine, gestion des
  machines, modifier une page, calendrier machine. **56 keys × 5.** 30 PHPUnit
  tests / 218 assertions pass on CT 210 after the entity change below.
  - 🔴 **`disponible` and `idle` were two tiles for one state.** `MACHINE.statut` is
    free text: the column default is `idle`, the seeded rows say `disponible`, and
    the admin list grouped its status tiles by that raw string *and* printed it in
    the cells — two chips meaning the same thing, in two languages, on a French
    page, next to a detail page translating its own private copy of the mapping.
    **`Machine::getStatusKey()` is now the single place that decides how the column
    is shown**, and `Machine::statusFilterForKey()` is its query counterpart so a
    tile's link selects exactly the rows the tile counted. Verified on the live
    data: three tiles became two (Disponible 10, En maintenance 1), each link
    returns its own count, and `?statut=idle` still returns 2 — a raw value that is
    not a display key still matches exactly, so old links keep working.
    ⚠️ An unrecognised word falls through to *available* on purpose. A machine
    nobody has classified is bookable today; failing closed here would take it
    offline on a spelling.
  - **The machine calendar held the same twenty French JS strings the week calendar
    did** — the previous batch fixed one of two copies. Both have a `T` object now.
  - **Five `onsubmit="return confirm('…')"` handlers were still in the templates.**
    Three carried French sentences inside an HTML attribute (photo, affiche,
    création); two were translated but still inline JS. All five are
    `confirm_controller` now. Every page involved emits `importmap('app')` —
    checked, because a confirm that does not load is a delete that never asks.
    ⚠️ `grep onsubmit templates/` is the check; only `admin-design`'s two
    `onsubmit="return false"` demo forms remain, and they carry no text.
- **S134c eighth batch shipped 2026-08-11.** Seven more screens at zero, deployed
  and verified: gestion des utilisateurs, créer/modifier un badge, créer une
  formation, état de l'installation, historique machine, quiz. **119 keys × 5.**
  30 tests / 218 assertions still pass.
  - 🔴 **35 French sentences lived in `public/js/`** — `quiz.js` (22),
    `section-journey.js` (6), `main.js` theme switch (6). The quiz-taking flow and
    the guided-course validation, member-facing, on a five-language site.
    **Nothing in this project could see them**: not `debug:translation` (a literal
    is not a key), not `scan_hardcoded.py` (it strips `<script>` *and never looked
    outside `templates/` at all*). The two grep commands that do find them are now
    in `tools/i18n/README.md` — run them before calling a screen done.
    The fix shape: the template emits a `<script type="application/json">` label
    node (or `data-` attributes when there are only a few) and the JS reads it,
    falling back to the key so a gap shows rather than hides.
    ⚠️ `public/js/*` is **not** AssetMapper — bump the `?v=` on every template that
    references a changed file, which for `main.js` is 21 of them.
  - **Three screens printed `user.statut` raw** (users list, user detail, profil).
    `Utilisateur::getStatusKey()` now, following `Machine::getStatusKey()`. It
    needs no filter counterpart and the docblock says why.
  - **`profil` was the third page carrying `status-badge active` hardcoded**, after
    the two S84 found. Verified live: the chip reads "Active", not `actif`.
- **S134c ninth batch shipped 2026-08-11.** Sixteen small forms at zero, plus two
  sources of French outside any template. **~200 keys × 5.** 30 tests / 218
  assertions still pass.
  - 🔴 **A third blind spot: `{% block title %}` is not scanned either.**
    Forty-one templates carried a French browser tab title — every admin
    create/edit form, the dashboard, login, the profile. 28 translated (the rest
    are the deferred dev-mode and `static/*` pages); the grep is in the tooling
    README. What is left in that list is a bare variable or a `|trans` call —
    **a French word there is a bug.**
  - 🔴 **The dashboard's activity feed was built in PHP with `sprintf`** — 14
    French strings, "Utilisateur inconnu" three times over. The dashboard has been
    recorded here as "at zero hardcoded strings" since the second batch while its
    most prominent panel was French in five languages.
    `buildRecentActivities()` returns keys and raw values; the template composes.
    ⚠️ **That is now three times a page was called done and was not.** The scan
    covers text nodes and a few attributes of `templates/*.twig` and nothing else:
    not `<script>`, not `public/js`, not `{% block title %}`, not PHP.
  - **`admin_form` holds what the small forms share** — the "Champs modifiables"
    heading was one edit away from existing in six namespaces.
  - `register` offered `jean.dupont@ensea.fr` as the e-mail example on the page
    every new member reads; the creation form explained `public/uploads/creations/`
    and the user form said the password "est hashé par Symfony". Same
    implementation-detail-as-help-text family as the SQL-patch instructions.
- **S134c tenth and eleventh batches shipped 2026-08-11.** Every remaining admin
  list and small form is at zero, and the fourth blind spot is closed.
  ~120 keys × 5 in the tenth, ~90 in the eleventh.
  - 🔴 **A fourth blind spot: French inside Twig expressions.** `|default('…')`,
    ternaries, `{% set %}` lists — the scan reads text nodes and attributes, so it
    never saw them. **101 strings**, every one a fallback: what a page shows
    exactly when the data is missing. "Utilisateur inconnu" and "Sans description"
    each existed in eight templates. One `fallback` namespace now.
    ⚠️ Accent-free French (`'Oui'`, `'Fait'`, `'Toutes'`) also slips past an
    accent-based grep — the README says to grep those words by name.
  - `formation-detail` held **its entire default content set** in expressions —
    21 strings on the page a member reads before signing up, one offering "le
    catalogue connecté à MariaDB".
  - **The scanner now reports only noise outside the deferred pages**: form
    `value=` attributes, URL placeholders, Twig fragments.
- **S134c twelfth batch shipped 2026-08-11 — the content/UI line, drawn.**
  `FormationPageContentService::DEFAULTS` was ~40 French strings the service
  always supplied, so `formation-detail`'s `|default()` fallbacks never fired. It
  holds keys now, **but only where FabOS is the one speaking**: the four section
  headings, the three "how the guided path works" cards, the practical-sign-off
  wording, the two navigation cards. `program` and `sessions` — this course's
  timetable and its dates — stay literal, because they are content.
  `getContent()` translates the defaults **before** merging an operator's stored
  block over them, so their prose never passes through a catalogue; only values
  shaped `namespace.key` are resolved, which keeps `'00:00'` and `'available'`
  literal without a second list. Verified live: a French visitor sees "Description
  détaillée", an English admin sees "Detailed description", and both see "Accueil
  et sécurité" and "Mardi prochain" untouched. The content editor shows the
  translated defaults, not keys.
  **Decision applied, and it closes the open questions:**
  - `static/*` (~85 literals — mentions légales, conditions, confidentialité,
    documentation, support, statut, roadmap) is **content. Leave it.** It is what
    a given install publishes about itself; it belongs to the operator and to the
    Themes content model, not to five catalogues.
  - The development-mode reference pages (~376) are internal documentation.
    Not translated.
  🔴 **Logged, not fixed — FabOS invents a training's content when it is empty.**
  A fabricated four-part timetable, three fake sessions ("Mardi prochain",
  "Places disponibles"), three objectives, two prerequisites and three materials,
  all shipped as defaults every install serves until someone overrides them. It
  is wrong in any language, which is exactly why it is not a translation fix.
  Nothing invented should be presented as this training's.
  ⚠️ `admin-design` (235),
  `usage-rights-vision` (55), `structure-vision` (35), `workspace-vision` (25) and
  `admin-missing-pages` (26) top the raw count but are **development-mode or
  reference documentation** — do them last. The `static/*` pages (api-docs 32,
  documentation 17, legal-notice 15, terms 15, roadmap 14, support 13) are
  operator-authored public copy and want a decision before a catalogue: they may
  belong in the Themes content model rather than in `messages`.
  **~598 literals across 64 templates remain** (heuristic upper bound from the
  scan; it still counts some Twig expressions and format examples). ⚠️ **The scan reads text nodes and a
  few attributes of `templates/*.twig` and nothing else — not `<script>`, not
  `public/js`, not `{% block title %}`, not PHP** — the two calendars' forty and `public/js/`'s thirty-five
  never appeared in any number this document has ever quoted. All are closed now;
  the greps that would have found them are in the tooling README.
- **The tooling is in the repo: `FabApp/tools/i18n/`** (`scan_hardcoded.py`,
  `parity.py`, `catalogue.py`, plus a README with the workflow and the two rules the
  scripts cannot enforce). Python 3 only, so it runs on the Mac where there is no
  PHP. Do not rebuild it. ⚠️ `catalogue.add_keys` inserts into an **existing**
  namespace block on purpose: appending a second `admin_emails:` at the end of a file
  silently shadows the first under YAML last-wins, and no lint catches that.
  - ⚠️ **The previous entry claimed dashboard and Réglages were already done. They
    were not** — 7 and 24 literals were still there, including every stat-card label
    on the dashboard. A page is done when a scan of it comes back empty, not when the
    session that touched it says so.
  - 🔴 **`&mdash;` inside a catalogue value renders as the literal text `&mdash;`.**
    `|trans` output is escaped, so the entity is escaped a second time. `/admin/settings`
    had been shipping `Administration &mdash; Réglages du site` on screen since the
    previous batch. Every entity in all five catalogues is now a real character
    (`—`, ` `, `…`), which is correct in both escaped and `|raw` contexts.
  - Three things that were not translation faults, fixed on the way: the détail
    utilisateur page called itself *"Consultation lecture seule"* while carrying three
    forms; the event registration pill printed the stored enum (`registered`,
    `waitlisted`) verbatim on a French page; and the RFID reader form wrote its two
    copy-outcome messages as literals inside `<script>`, where no translator can reach
    them — they are read off `data-` attributes now.

**Traps confirmed this session, worth not re-learning:**

- ⚠️ **`strict_variables: true` is set only under `when@test`, not in prod.** A
  template reading a variable the controller does not pass resolves to null in
  silence. That shipped a data-corruption bug in S131 (the hours screen wrote one
  venue's week onto another's, fixed in S132b). Twig lint does not catch it; the test
  environment would.
- ⚠️ **A count that matches your expectation is not evidence** when the failure mode
  produces the same count. S131 was "verified" by counting a rendered element, getting
  0, and reading it as "correctly hidden on a single-venue install" — it was actually
  "variable undefined". Assert the positive case: render with two venues and require
  the control.
- ⚠️ **The sidebar is built by `NavBuilder`, not `FeatureWorkspaceRegistry`.** The
  registry is metadata; `nav_admin()` is what `_admin_sidebar` reads. Registering a
  route in the registry alone leaves it unreachable. Both must be updated until S130's
  successor collapses them.
- ⚠️ In the `edit` sidebar variant `shell.icons` is false, so the lit class is
  `active`, never `admin-nav-link active`. Grepping the latter reports every
  create/edit page as unlit and it is not — check `aria-current`.
- ⚠️ `_logo.html.twig` still falls back to `Logo_ENSEA.png` and `site_logo_path` is
  **not editable anywhere in the UI**. De-branding that fallback before the Themes
  media library exists would leave the site with no logo and no way to restore one.
  Operator has confirmed it belongs in Themes.

**An independent whole-site review (2026-08-11) produced findings not yet acted on:**
sidebar and workspace tabs disagree about what "Équipement" contains; ~~the users list
action says "Edit" but opens a page titled "Consultation lecture seule" that contains
three forms~~ (subtitle fixed 2026-08-11; the misleading list *action label* is not);
loan rows do not link to the object's page; machine status is raw storage vocabulary
mixing French and English (`idle` vs `disponible`); 12 admin pages still end without
the shared footer (the standalone scaffolds, not the `_admin_list` ones); Logs RFID
dumps 100 rows with a "Color" column printing the words `purple`/`green`. Phase G2 in
`ROADMAP.md` schedules the rest.

**Found while doing S134c, logged not built:**

- The Pi pairing runbook bakes one developer's home directory (`/home/subhen/…`)
  into text every operator reads. Now a single `device_root` default in
  `_rfid_pairing_modal.html.twig` rather than fourteen literals across two files, so
  changing it is one edit — but the paths are real paths on the device image, so
  what they should become is a device question, not a template one.
- Both RFID screens still show the same two buttons twice, three lines apart: the
  page header and the panel header each carry "Comment appairer une Pi ?" and
  "+ Nouveau lecteur RFID".
- The event-registration and formation screens hint at importing "le script SQL
  fourni dans ce patch" / "le script de données fourni avec le patch" — instructions
  addressed to whoever was applying a patch, not to an operator of an installed
  FabOS. The wording is neutral now, but the underlying gap (quizzes and physical
  validations have no UI to create them) is real and unscheduled.
- Four `validators`-domain messages are French sentences used as their own message id,
  so `debug:translation` reports them missing in all five locales including French.
  They are the creation-upload constraints in the entity attributes.
- `admin-utilisateur-detail` and `admin-emails` still print raw storage vocabulary in
  data cells: `user.statut`, `user.theme`, `log.category`, `log.template`, `log.color`.
  **`Machine::getStatusKey()` / `statusFilterForKey()` is the pattern to copy** — an
  entity method returning the catalogue key, plus its query counterpart so counts and
  rows cannot drift apart. Do not solve it a second way. Logs RFID joins the list: its `Statut`, `Motif` and
  `Couleur` **column headers** now translate, but the cells under them still print
  the stored `status`, `reason` and `color` strings verbatim — `purple`/`green` on a
  French page. Translating the header made that gap more visible, not smaller.
- The two RFID screens still carry the same two buttons twice, three lines apart
  (page header and panel header) — unchanged by this batch, still worth one edit.

## Superseded — 2026-08-10

- Latest recorded product decision is **S102**, refined by the operator decisions recorded for S103. It supersedes the portal-shaped parts of S100–S101 before schema work: a service needing independent administration/theme/data gets its own FabOS; sub-locations exist only inside one shared governance/data set and aggregate by default. LDAP/OIDC/SAML may share authentication, never rights or data; local accounts/groups/packages/audit remain authoritative and selected data crosses instances only through the FabOS network. Packages carry only Use/Manage grants scoped by sub-location and feature; Manage includes reporting on its scope and never grants Use. Configuration will expose one versioned Themes workspace for colours, images, public name, menu labels/order and homepage blocks/content/order. The live S97–S99 model is unchanged and enforcement remains off. Physical cards/readers remain explicitly deferred.
- Full-access packages are durable: they include future audited capabilities automatically. Ordinary edits preserve grants for temporarily disabled site features instead of deleting them.
- **Phase F is complete (S127–S128).** S127 removed the retired portal model from code and Artemis: hostname resolution, portal-scoped settings/features/logs, portal templates and the `PORTAL` table are gone. Artemis was explicitly treated as disposable development data: one portal, two scoped settings and thirteen feature overrides were purged; packages would have been folded into the single site. Legacy `/admin/portals*` permanently redirects to `/admin/design/structure`. The irreversible migration is `Version20260810130000.php`; never reuse it for a production installation without an independently tested backup/restore plan.
- ⚠️ **Phase G (S129–S134b) is the current work and is blocking before Commerce.** Earlier bullets in this file numbered Commerce as S129–S133 and Training comms as S134–S136; `ROADMAP.md` renumbered them to S135–S139 and S140–S142 when Phase G was inserted. The roadmap is authoritative. A session that follows the old numbers starts Commerce, which the roadmap forbids until an operator can administer a sub-venue from the canonical interface and S134b has validated the cross-cutting cleanup.
- S128 then audited the shared core on Artemis: **30 PHPUnit tests / 208 assertions**, **188 Twig templates**, **29 YAML** translation/configuration files, and all **14 canonical shared-workspace routes** rendered HTTP 200 through `app:render`. The completed commits are `fd519c5` (S127) and `132aad2` (S128 docs/audit record). Next planned work is optional Commerce, **S129–S133**.
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
