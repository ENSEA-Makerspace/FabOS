# i18n tooling — S134c

Three small scripts written during S134c to move hardcoded strings into the five
catalogues without breaking them. They need only Python 3 (no PyYAML), so they run on
the Mac where there is no PHP. Run them from `FabApp/`.

They are **development aids, not tests**. Nothing in the app calls them and nothing
deploys them; they exist so the next session does not rebuild them.

## `scan_hardcoded.py` — what is left to translate

```bash
python3 tools/i18n/scan_hardcoded.py templates          # ranked by count
DETAIL=1 python3 tools/i18n/scan_hardcoded.py templates/site/admin-emails.html.twig
```

Strips Twig comments, `<style>`, `<script>`, `<pre>` and `<code>`, then reports text
nodes and human-facing attributes that still contain prose.

⚠️ **It is a heuristic upper bound, not a count of bugs.** It still flags Twig
expressions split across lines, form `value="save"` attributes, and format examples
like `smtp://user:pass@host`. Read the `DETAIL=1` output before believing a number.
A template is done when what remains is only that noise.

## `parity.py` — are the five catalogues in step

```bash
python3 tools/i18n/parity.py translations templates
```

`MISSING` lines are keys absent from at least one locale — the thing to drive to
zero. It parses the nesting, anchors/aliases and inline flow mappings these files
actually use; a naive parser reports every `reporting.*` key as absent from Spanish
because that block is written `reporting: &reporting_es`.

The `UNDEFINED` section is noisy by design: it also scans `src/**/*.php`, where
capability names like `badges.update` look exactly like translation keys. Treat it as
a lead, not a finding.

⚠️ **The authoritative check runs on Artemis**, because it is the one that knows what
the app actually loads:

```bash
php bin/console debug:translation <locale> --domain=messages --only-missing
```

## `catalogue.py` — add keys to all five files

Import it; it has no CLI. The point is `add_keys`, which inserts into an **existing**
namespace block rather than appending a second one — a second `admin_emails:` block
at the end of the file silently shadows the first under YAML's last-wins rule, and
nothing lints that.

```python
import sys, os
sys.path.insert(0, 'tools/i18n')
import catalogue as C
C.CAT = os.path.abspath('translations')

KEYS = {'fr': [('save', 'Enregistrer')], 'en': [('save', 'Save')], ...}
assert len({tuple(k for k, _ in v) for v in KEYS.values()}) == 1   # same key set everywhere
for loc in C.LOCALES:
    C.add_keys(loc, 'admin_emails', KEYS[loc], comment='E-mails admin — S134c.')
```

Always assert the key sets match before writing. Adding a key to four catalogues out
of five is the failure this workflow exists to prevent, and `parity.py` will catch it
afterwards — but only if you run it.

## What the scan cannot see (S134c, learned the hard way)

`scan_hardcoded.py` strips `<script>` before it counts, and it only ever walks
`templates/`. Two whole classes of user-visible string are therefore invisible to
it — **and to `debug:translation`, because a literal is not a key**. A page can be
reported clean by every measurement this repo takes and still be French in five
languages.

Both were found and closed in S134c. Check them by hand before calling a screen done:

```bash
# French sentences inside inline <script> blocks
grep -rnoE "'[^']{4,90}'" templates/ | grep -E "[éèêàûôçÀ-Ý]" | grep -v '|trans'

# ... and in the standalone files under public/js/
grep -noE "['\\`\"][^'\\`\"]{3,90}['\\`\"]" public/js/*.js | grep -E "[éèêàûôçÀ-Ý]"
```

It has a fourth: **French inside Twig expressions is invisible too** — the scan
reads text nodes and a few attributes, so a string in `|default('…')`, in a ternary
or in a `{% set %}` list is never counted. There were 101, all of them fallbacks —
what a page shows exactly when the data is missing. The check:

```bash
grep -rnoE "'[^']{3,110}'" templates/ --include=*.twig | grep -E "[éèêàûôçÀ-Ý]" \
  | grep -v "|trans" | grep -vE "'[a-z_]+\.[a-z_]+'"
```

Accent-free French (`'Oui'`, `'Non'`, `'En cours'`, `'Fait'`, `'Toutes'`) slips past
that one — grep for those words by name as well.

It has a third: **`{% block title %}` is not scanned either.** Browser tab titles
are user-visible, and 41 of them were French. The check:

```bash
grep -rn "{% block title %}" templates/ | grep -vE "'[a-z_]+\.[a-z_]+'\|trans"
```

Anything left in that list should be a bare variable (`{{ machine.nom }} - FabOS`)
or already a `|trans` call. A French word there is a bug.

The fix for the JS cases is always the same shape: the template emits the translated labels
(a `<script type="application/json">` node, or `data-` attributes when there are
only a few), and the JS reads them. `quiz.html.twig` → `public/js/quiz.js` is the
worked example; `profil.html.twig` → the theme switch in `main.js` is the small one.

⚠️ Those files live in `public/`, not AssetMapper. **Bump the `?v=` on every
template that references them**, or browsers keep the old strings.

## The rule that decides whether a string belongs here at all

**Translate the UI. Never translate content.** FabOS's own words are UI: labels,
headings, buttons, and its explanations of how the product works. Anything an
operator writes about their own thing is content: a training, a lab page, a legal
notice, an event. *A training exists in one language and that is it* — operator,
2026-08-11.

The test is not where the string lives, it is who is speaking. A section heading
above a course description is UI; the description is not. Where FabOS ships a
default an operator will overwrite, translate the default and never the override:
`FormationPageContentService` resolves its keys **before** merging the stored
block, which is the pattern to copy.

This is why `templates/site/static/*` is not on the to-do list. It looks like 85
untranslated literals; it is one install's own public copy.

## Two rules these scripts cannot enforce

- **No HTML entities in catalogue values.** `&mdash;` in a value rendered by a plain
  `|trans` comes out as the literal text `&mdash;`, because Twig escapes it. Use the
  real character (`—`, `…`, U+00A0). A value that genuinely carries markup —
  `<strong>`, `<code>`, an `<a href="%url%">` — must be rendered `|trans|raw`.
- **A complete catalogue does not mean the language is reachable.** All five were
  complete and in parity for four batches while the profile's language selector
  offered only two. Check the selector, not just the strings behind it.
