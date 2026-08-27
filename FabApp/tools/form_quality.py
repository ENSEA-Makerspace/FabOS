# -*- coding: utf-8 -*-
"""Qualité des formulaires — le test « designer d'Apple », en chiffres lisibles.

Ce qu'on mesure, et pourquoi (repris de la lecture de Fabman déjà au dépôt,
`docs/history/phase-U-design-S45-S57.md` § « Les détails à voler ») :

  · combien de champs l'écran montre À L'ARRIVÉE — un formulaire s'ouvre sur son
    cas courant, pas sur toutes ses options ;
  · combien sont dans un repli (`<details>`) — les champs optionnels n'existent
    pas tant qu'on ne les demande pas ;
  · combien de formulaires distincts sur la page — quatre boîtes ouvertes en même
    temps, c'est quatre décisions simultanées ;
  · combien de champs obligatoires — le point 9 des dix.
"""
import re, glob, os, sys

def blank(m):
    return ''.join('\n' if c == '\n' else ' ' for c in m.group(0))
COMMENT = re.compile(r'\{#.*?#\}|<!--.*?-->', re.S)
STYLESCRIPT = re.compile(r'<(style|script)\b.*?</\1>', re.S)

# les champs d'un FormType, pour compter ce que `form_row` déploie
def type_fields():
    out = {}
    for p in glob.glob('src/Form/**/*.php', recursive=True):
        src = open(p, encoding='utf-8').read()
        cls = os.path.basename(p)[:-4]
        out[cls] = len(re.findall(r'->add\(', src))
    return out

FIELDS = type_fields()

# 🔴 **S151 — le motif `SECTIONS` rendait cet outil AVEUGLE.** Trois gabarits
# (machine, matériau, objet prêtable) ne listent plus leurs champs : ils incluent
# `_form_sections.html.twig` avec la constante `SECTIONS` de leur `FormType`, qui
# déroule un seul `form_row(form[name])` en boucle. Compter les `form_row(`
# littéraux donnait donc **1 champ** pour un écran qui en montre vingt, et zéro
# repli pour un écran qui en a deux. La règle 1 serait passée de mesurée à
# décorative au moment précis où on l'applique enfin.
#
# On lit donc la constante à la source. Ce n'est pas du PHP interprété : la
# constante est une liste littérale de `title` / `fold` / `fields`, et une regex
# suffit — mais elle est la SEULE source, donc si le format change, l'outil se
# tait plutôt que de mentir (compte 0 et laisse le décompte littéral).
SECTIONS_CACHE = {}
def sections_of(cls):
    """(visibles, repliés) déclarés par la constante SECTIONS d'un FormType."""
    if cls in SECTIONS_CACHE:
        return SECTIONS_CACHE[cls]
    hits = glob.glob('src/Form/**/%s.php' % cls, recursive=True)
    vis = fold = 0
    if hits:
        php = open(hits[0], encoding='utf-8').read()
        m = re.search(r'const\s+SECTIONS\s*=\s*\[(.*?)\n    \];', php, re.S)
        if m:
            for blk in re.finditer(r'\[(.*?)\]\s*,?\s*(?=\[|$)', m.group(1), re.S):
                b = blk.group(1)
                fm = re.search(r"'fields'\s*=>\s*\[(.*?)\]", b, re.S)
                if not fm:
                    continue
                n = len(re.findall(r"'[^']+'", fm.group(1)))
                if re.search(r"'fold'\s*=>\s*true", b):
                    fold += n
                else:
                    vis += n
    SECTIONS_CACHE[cls] = (vis, fold)
    return SECTIONS_CACHE[cls]

INCLUDE_SECTIONS = re.compile(
    r"include\s+'site/_form_sections\.html\.twig'.*?constant\(\s*'App\\\\Form\\\\(?:[A-Za-z]+\\\\)*([A-Za-z]+)::SECTIONS'",
    re.S)


rows = []
for p in sorted(glob.glob('templates/site/*.twig')):
    raw = open(p, encoding='utf-8').read()
    src = STYLESCRIPT.sub(blank, COMMENT.sub(blank, raw))
    # ⚠️ **Deux copies, et c'est nécessaire.** On COMPTE sur `src` (les champs sont
    # déclarés par `{{ form_row(...) }}`, donc effacer les expressions Twig les
    # effacerait aussi). On TESTE l'état d'un `<details>` sur une copie où les
    # expressions sont neutralisées : un repli écrit
    # `<details{{ form.vars.submitted ? ' open' : '' }}>` contient le mot `open`
    # dans son EXPRESSION, pas dans son état par défaut — sans ça il est compté
    # comme déplié, et la mesure dit le contraire de la vérité.
    src_attr = re.sub(r'\{\{.*?\}\}', lambda m: ' ' * len(m.group(0)), src, flags=re.S)
    short = os.path.basename(p).replace('.html.twig', '')
    n_forms = len(re.findall(r'\{\{\s*form_start|<form\b', src))
    if n_forms == 0:
        continue
    n_rows = len(re.findall(r'form_row\(', src))
    # les champs déroulés par le partiel partagé, section par section
    sec_visible = sec_folded = 0
    for m in INCLUDE_SECTIONS.finditer(src):
        v, f = sections_of(m.group(1))
        sec_visible += v
        sec_folded += f
        n_rows -= 0  # le `form_row(form[name])` du partiel n'est pas dans CE fichier
    n_rows += sec_visible + sec_folded
    n_raw = len(re.findall(r'<(?:input|select|textarea)\b', src)) - len(re.findall(r'type="hidden"', src))
    total = n_rows + max(0, n_raw)
    # ce qui est REPLIÉ derrière un <details>
    hidden = 0
    # 🔴 **Un `<details>` dans un `<details>` était compté DEUX FOIS** : une fois
    # dans le corps du repli extérieur, une fois pour lui-même. L'écran des packages
    # en a un (la portée d'un grant, repliée à l'intérieur de « Ajouter le grant »)
    # et le total « replié » dépassait le total réel — donc « visible » tombait
    # sous la vérité. Trouvé par la paire A, qui a refusé de corriger l'outil qui la
    # mesurait ; c'est la bonne réponse, et c'est au chef de le faire.
    _spans = []
    for m in re.finditer(r'<details\b(?![^>]*\bopen\b)', src_attr):
        if any(a <= m.start() < b for a, b in _spans):
            continue                     # déjà compté dans un repli englobant
        i, depth = m.start(), 0
        j = m.start()
        while j < len(src):
            if src.startswith('<details', j): depth += 1
            elif src.startswith('</details>', j):
                depth -= 1
                if depth == 0: break
            j += 1
        _spans.append((m.start(), j))
        body = src[m.start():j]
        hidden += len(re.findall(r'form_row\(', body))
        hidden += len(re.findall(r'<(?:input|select|textarea)\b', body)) - len(re.findall(r'type="hidden"', body))
    hidden += sec_folded
    visible = total - max(0, hidden)
    n_required = len(re.findall(r'\brequired\b', src))
    rows.append((visible, total, max(0, hidden), n_forms, n_required, short))

rows.sort(reverse=True)
print('%-38s %7s %7s %7s %6s %6s' % ('écran', 'visible', 'total', 'replié', 'form', 'requis'))
print('-' * 78)
for visible, total, hidden, n_forms, req, short in rows:
    flag = ''
    if visible >= 20: flag = ' 🔴'
    elif visible >= 12: flag = ' ⚠️'
    if n_forms >= 4: flag += ' 🔴%d formulaires' % n_forms
    print('%-38s %7d %7d %7d %6d %6d%s' % (short, visible, total, hidden, n_forms, req, flag))
