## Phase C — S112–S117 · shared workspaces — ✅ shipped 2026-08-09

The descriptive S103 registry now drives the real operator shell. Every migrated list gets the same feature tabs, one quick axis, server-side search, progressively disclosed advanced filters and at most six removable URL chips. The five translation catalogues carry the shell copy and `/admin/design` renders the actual components. Quotas and Reporting are deliberately not rendered before S118/S119.

Equipment gained category and model/brand catalogue tabs plus nullable manufacturer/model metadata; Materials, Maintenance and typed machine reservations stay in the same workspace. Events and Loans use their existing domain adapters. Spaces gained nullable category, responsible person and department facets without weakening the required venue relationship. Existing Gallery, Pages, Users, Locations, Packages, Network and Configuration lists were connected to the registry rather than copied.

Badges are now non-destructive: the delete route and danger zone are gone, archive/restore preserves the definition, and all award/equipment/institution badge FKs prevent cascading deletion. The existing local award path was already insert-only. Configuration gained `/admin/themes`, with a scoped JSON draft, validated logo/colour values, preview, explicit publish and discard back to the published version; publication feeds the existing organization, venue, logo and accent readers.

Deployment used three additive migrations: space facets, equipment manufacturer/model, and badge archival/FK hardening. The migration was preflighted against the actual Artemis FK names before execution.
## 2026-08-10 — Phase D: policies, reporting, workspace parity (S118–S120)

Booking policies now cover the complete shared path: notice, horizon, granularity, duration, active/day/week caps, buffer and cancellation/reschedule notice. The new field is read by the existing central booking-verb service, so UI and API actions receive the same verdict and physical availability remains separate.

Reporting is a tagged adapter registry rather than page-specific queries. Its first reservation adapter serves the Equipment and Spaces workspaces with venue/date scopes, aggregate metrics and identity-free CSV exports; the stable capability names are `analytics.view` and `analytics.export`. The screen composes the existing workspace tabs, venue selector, admin list shell, summary cards and data tables.

Historical unscoped reservation links now redirect to the Equipment workspace with their filters preserved. The duplicate global Reservations navigation group is removed after Quotas and Reporting reached their feature workspaces; packages and usage rights remain reachable from the shared venue administration area.
## 2026-08-10 — Phase E: identity, public profiles and FabOS Network (S121–S126)

FabOS now separates sign-in federation from data federation. OIDC uses authorization code with PKCE, configured HTTPS issuers and secrets read only from named environment variables. External accounts are keyed exclusively by `(issuer, subject)`; matching e-mail never merges accounts, external claims never create roles, and local suspension remains authoritative.

Member pages are private by default. A member explicitly enables `/m/{slug}` and separately selects name, avatar, bio, badges and completed training; e-mail, RFID, identifiers, reservations, groups and packages are not representable in the public-field allow-list. Public pages are non-indexable by default.

The Network workspace centralizes the instance Ed25519 identity, OIDC providers and trusted peers using the shared admin shell. The target schema includes bounded one-use exchanges, replay storage, audit, credentials with source/expiry/revocation provenance, and machine catalogue projections. Imported machine metadata is filtered so local tokens, RFID, state, safety, location, availability and stock can never be overwritten.

Because FabOS has not shipped V1 and has no real installations, Phase E implements the target schema directly. No historical compatibility layer or transition route was added.

## 2026-08-10 — Phase F: S127 portal retirement — ✅ shipped on Artemis

The single-instance retirement removes hostname portal resolution and per-portal configuration from the runtime. Artemis is a disposable development dataset, so its one portal, two scoped settings and thirteen feature overrides were intentionally purged; packages would have been folded into the single site. Legacy `/admin/portals*` routes transition permanently to the structure screen. The migration's rollback deliberately refuses to fabricate discarded portal data. Artemis verification: rebuilt production cache, 188 Twig templates valid, service active, public homepage 200, structure screen 200 and portal transition 301.

### S128 · audit transversal du socle — ✅ shipped 2026-08-10

The workspace contract was rechecked rather than inferred from the UI: the focused workspace/reporting tests and full suite pass (**30 tests, 208 assertions**); all **188 Twig** templates and **29 YAML** translation/configuration files validate; and the live kernel renders all fourteen canonical shared-workspace routes with HTTP 200. The audit resolved route paths from Symfony before rendering, avoiding false negatives from guessed URLs. The scope remains descriptive where designed to be descriptive; the shell still does not grant access, and route/services remain the enforcement boundary.

### S130b · one admin sub-navigation — ✅ shipped 2026-08-11

The admin had **two** sub-navigations and drew both on every page. `NavBuilder`
emitted a labelled strip from the feature sections; `FeatureWorkspaceRegistry`
emitted a second, unlabelled one from a `TABS` map that `_workspace_tabs.html.twig`
rendered inside the content column. Measured on CT 210 before the change: **20 of
20 admin pages carried both.**

They were not two views of the same list. Équipement had six entries on one strip
and eight on the other. Institutions sat under Badges in one and under Réseau in
the other. Five sections rendered a bar whose only link was the page already open —
twice over. And because only the tabs read the catalogues, an English account saw a
French strip stacked on an English one.

**Six live screens were reachable only from the tabs** — `app_admin_reservations`,
`app_admin_booking_policies`, `app_admin_reporting`, `app_admin_machine_categories`,
`app_admin_machine_models` and `app_admin_homepage`. Deleting the tabs without
folding them in would have orphaned all six, which is why the coverage table was
built before anything was removed.

`NavBuilder` is now the whole navigation. The registry keeps `all()` and
`themeContract()` for `/admin/design/workspaces` and lost `TABS`, `forRoute()`,
`markActiveTab()` and the `feature_workspace()` Twig function.

**Two things the merge forced, both of them real.** Réservations, Quotas and
Reporting are *one route each serving several sections*, told apart by
`reservableType` or `workspace`. Matching on the route alone lit all three at once
and sent an operator looking at space reservations into Équipement, so entries
carry `params` and `isCurrent()` compares them. Those same params also go to
`canReach()` — it answers by generating the URL, and `/admin/reporting/{workspace}`
cannot be generated without one, so the first deploy silently dropped Reporting
from both strips and rendered `/admin/reporting/equipment` with no navigation at
all. The catch that swallowed it exists to make dead routes vanish quietly; it
does the same to a live route asked about incorrectly.

**The labels moved to the catalogues.** `workspace.tab.*` was already translated in
all five locales and keyed by route, so those 27 values were re-homed as
`admin_nav.entry.<route>` rather than rewritten; 11 entries and 14 section names
that had lived as French literals in `NavBuilder.php` were added. That file was
invisible to `scan_hardcoded.py`, which reads Twig text nodes — the same blind spot
that hid the dashboard for two batches of S134c.

**Shipped in the same session, from the whole-site review:**

- The **footer moved into `base.html.twig`**. S129 had established "the footer
  belongs to the shell" for the 25 pages on the list shell; the same split existed
  one shell up, where 82 templates extend a base that emitted no footer at all.
  Fifteen templates were compensating with a hand-written include. ⚠️ **The review
  said 12 pages lacked a footer; measuring the running pages found 27.**
- The users list said **Modifier** and opened a read-only detail page. Now *Voir*.
- A loan's object is a link to that object.
- Lecteurs RFID drew its pairing and create buttons **twice, three lines apart**;
  the panel copy is gone. ⚠️ The review said both RFID screens did this. Only one
  did.

**Tests.** The two `FeatureWorkspaceRegistryTest` cases that asserted the tab map
were removed — they described deleted machinery — and replaced by
`AdminNavCatalogueTest`, which fails if a navigation label is a literal, if an
entry key does not name its route, or if any key is missing from any of the five
catalogues. **31 tests / 780 assertions**, up from 30 / 167.

**Verified on CT 210, not inferred.** 34 admin pages re-rendered: every one HTTP
200, `workspace-tabs` count **0**, exactly one strip, exactly one lit entry, and
the parameterised routes lighting their own section. 50 pages checked for the
footer: **0 without**. Public and static pages checked for a doubled footer: all
exactly 1. Twig 191/191 and YAML 5/5 valid. Hash comparison over all 94 changed
files: identical. ⚠️ Twig lint refused this session's work **twice**, both times
over an explanatory comment and not the change — once for a comment between two
keys of an argument hash, once for spelling the comment-closing sequence inside a
comment. Both are the traps `ARTEMIS_DEPLOYMENT.md` already names, and both were
caught before the restart because the lint runs first.

### S130c · the sub-venue filter becomes a one-click filter — ✅ shipped 2026-08-11

Every other filter on an admin list is one click on a labelled chip row:
categories, status, the tile groups. The sub-venue filter — the one that scopes
what all the others operate on — was a `<label>`, a `<select>` and a
right-aligned sentence in a full-width bar of its own, sitting *above* the panel
that holds the rest. It cost two clicks and a menu, and its placement said it
belonged to a different screen.

It is now the first group inside that panel, in the same
`ml-filter-group-label` / `ml-cats` / `ml-tile` markup as everything else. Read
top to bottom the list narrows from *where*, to *what kind*, to *which words*.

**It renders nothing on a single-venue install**, which is the operator's actual
request and was already true on one page and false on forty:
`admin-opening-hours` had wrapped its include in `venues|length > 1` since S131
and `_admin_list` never did. The test moved into the partial so neither call site
can forget it, and the redundant guard came out of the hours page.

Two details the tiles carry that the select did not: `page` is dropped from every
link — narrowing to a sub-venue with fewer rows while staying on page 4 shows an
empty list that reads as "this sub-venue is empty" — and the "does not change
your permissions" line appears only while a sub-venue is actually selected,
rather than permanently reassuring about a filter that is not filtering.

🔴 **`allow_all|default(true)` was always true.** Twig's `default` fires on any
*empty* value, not only an undefined one, and `false` is empty. `/admin/horaires`
passes `allow_all: false` precisely because opening hours are stored per venue,
and it had been offering "all sub-venues" since S131 — the one option its own
include comment says must not exist there. Choosing it does not error:
`VenueContext::single()` reads `all` as "not specified" and falls back to the
default venue, so the operator edits the default venue's week while the URL says
`location=all`. That is the silent-wrong-row failure S131 was written to prevent,
surviving inside the fix for it. `allow_all is not defined or allow_all` is the
test that distinguishes undefined from false.

⚠️ **`admin.css` line 1 is `@import url("machines-list.css")`,** which is how
`.ml-tile` reaches ~41 admin pages that never name that file. Two consequences.
Grepping rendered HTML for `machines-list.css` finds nothing and does not mean it
is absent — a same-origin mirror of the four linked stylesheets, built to inspect
the cascade, omitted the imported file and rendered the tiles unstyled, which
briefly looked like a discovery and was an artifact of the mirror. And the
imported file is fetched at a URL the page's `?v=` never touches, so editing it
requires bumping **both** the import's own version and `admin.css`'s, or a cached
`admin.css` keeps importing the old URL.

**Verified on CT 210.** 13 pages rendered: the old `<select>` absent on all of
them; three tiles with exactly one lit on machines, places, events and loanable
items; `?location=fabshop` lighting FabShop and the counts below it dropping to
that venue's single machine; `/admin/horaires` down to two tiles with no "all";
and no picker at all on the pages that carry no venue context. The install has
two active venues, so this is the positive case measured directly rather than a
count that happens to match. 31 tests / 780 assertions; 34 admin pages still 200;
50 pages still footered; hash comparison over 75 files identical.

### S130d · Réseau FabOS becomes a Configuration tab — ✅ shipped 2026-08-11

Operator's call, and the right one. Réseau FabOS was a top-level admin section
holding exactly one screen. That cost it a whole sidebar row — level with
Équipement's eleven screens and Formations' catalogue — to say one thing, and
under S130b's rule a one-entry section draws no strip, so `/admin/network` had no
sub-navigation at all.

What that screen configures is the instance's own Ed25519 identity, its OIDC
providers and its trusted peers. That is configuration, not a workspace an
operator works inside. It now sits in the Configuration section between E-mails
and Configuration initiale — which stays last, because it is the one entry you
use once and never again.

Two consequences worth noting. `/admin/network` gained a sub-navigation it never
had. And the sidebar lost a row, which is the point: the sidebar lists workspaces,
and a single settings screen was never one.

Institutions did **not** move with it. `Institution` is referenced by exactly one
entity — `Badge`, many-to-many — so it is a badge issuer, not a federated peer,
and it stays under Badges. The retired workspace registry had claimed it for
Réseau; that disagreement is what S130b resolved.

`admin_nav.section.network` was removed from all five catalogues with the section.
The entry key `admin_nav.entry.app_admin_network` stays and is now read from
Configuration.

**Verified on CT 210.** Ten pages rendered: no Réseau row in the sidebar; every
Configuration screen showing the same seven-entry strip with exactly one lit;
`/admin/network` lit inside it; Badges and Institutions untouched. 31 tests / 769
assertions. 34 admin pages still 200, 50 still footered. ⚠️ The sub-venue sweep
returned four tiles instead of three mid-session — not a regression: a third
sub-venue, "Batiment D", was created on the install while this was being built,
and the tile row picked it up on its own. That is the data-driven behaviour S130c
was after, confirmed by accident.

### S130e · the list format, decided — ✅ 2026-08-11

Four rounds with the operator, in `/admin/design`. The proposals have been removed
from that page; only the retained format remains. What follows is the reasoning, so
removing them costs nothing.

**Round 1 — three panels.** The diagnosis mattered more than any of them: the
panel stacked **three different kinds of control as if they were peers** — a scope
(sub-venue), facets (category, status) and a query (search) — and two of the three
grow on their own, one with the organisation and one with the catalogue. Measured:
394 px of controls before the first row, 446 px projected.

**Round 2 — the page shape.** The operator took A and changed it: title left,
sub-venue **right** in the hero. That forces the create action out of the hero, and
it landed in the list header as a green `+`.

**Round 3 — search and the button.** Search moved out of the filter panel into the
list header, after the count: it finds *one* row, so it belongs to the list. The
add button went back into the hero, tested as an icon-only green circle and as a
named green pill. The pill won — an icon beside a sub-venue menu can read as "add
a sub-venue", and a `title` does not fix a first visit.

**Round 4 — the final shape.** Sub-venue left the hero entirely and became the
**first dropdown under "Affiner"**: it is a filter like the others, and it earned
neither its own bar nor a place in the hero. The hero keeps the title and the one
green named button. **124 px**, down from 394.

**What it cost to build, and is worth knowing.**

🔴 **`style.css` line 3191 repaints every `<span>` inside `.admin-panel`.**
`html[data-theme="dark"] body :is(… .admin-panel …) :is(p, span, small, li, …) {
color: var(--color-text-light) !important }`. The design page *is* an
`.admin-panel`, so the white `+` on the green pill rendered grey-on-green —
measured at `rgb(212, 200, 210)`. No specificity beats an `!important`; the rare
`!important` in that section exist for this. Invisible to lint, invisible in the
Twig, visible only by interrogating the cascade.

⚠️ **Twice in one session I sliced the file into head/tail, then edited the
original, then reassembled from the stale head — discarding the edit.** Both times
the symptom was a mockup that rendered subtly wrong rather than an error. When
splitting a file to swap a block, re-slice *after* every other edit, or apply the
edits to the reassembled file.

⚠️ **The first version of the comparison table asserted "3 rangées" for one
proposal from reasoning.** On a page whose stated purpose is measuring rather than
asserting, that was the wrong kind of claim. Every projection is now produced by
cloning the real panel, adding the missing tiles and measuring the clone.

**Next:** Phase G3 (S134h–S134j) applies the format to all 41 lists, defines a
column vocabulary and deletes ~590 lines of local list CSS. ⚠️ "Sous-lieu" is a bad
word and is to be renamed in one catalogue pass during S134i — `Venue`/`VENUE` stay.


---

# Index des sessions livrées — une ligne chacune

Écrit le 2026-08-16, en vidant `ROADMAP.md` de tout ce qui était fait. La
roadmap ne contient plus que le travail restant ; ce qui suit est le registre
de ce qui existe. Les récits détaillés sont plus haut dans ce fichier.

**Phase A — fondations (S102–S108, 2026-08).** S102 décisions + roadmap
nettoyée · S103 registre Feature Workspace v2 + contrat Thèmes · S104 quotas
réparés (compteurs par type, contraintes dures) · S105 gel des portails +
rapport de consolidation · S106 entité Sous-lieu + horaires migrés · S107
machines/espaces/objets/événements rattachés · S108 préférence de sous-lieu,
`?location=`, composant partagé.

**Phase B — groupes et packages (S109–S111).** S109 sept groupes intégrés
protégés · S110 grants Use/Manage + scopes en simulation · S111 packages v2 en
shadow.

**Phase C — le shell et les workspaces (S112–S117).** S112 shell central
listes/filtres/facettes · S113 Équipement · S114 Événements et Prêts · S115
Espaces · S116 Formations et Badges (archivage, pas de suppression) · S117
Galerie, Pages, Utilisateurs, Lieux, Packages, Réseau, Configuration, Thèmes.

**Phase D — réservations et reporting (S118–S120).** S118 politiques par
feature · S119 socle Reporting + `analytics.view/export` · S120 retrait de
Réservations globale.

**Phase E — identité et réseau (S121–S126).** S121 fédération OIDC · S122
`/m/{slug}` opt-in · S123 identité d'instance + API versionnée · S124 import QR
signé et consenti · S125 badges/formations fédérés · S126 marques/modèles
fédérés.

**Phase F — retrait et audit (S127–S128).** S127 portails retirés · S128 audit
transversal (30 tests / 208 assertions, 14 workspaces en 200).

---

