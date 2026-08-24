# -*- coding: utf-8 -*-
"""
Valide les 5 catalogues ICU et confronte chaque motif aux arguments des appelants.

    cd FabApp && python3 tools/icu_audit.py

⚠️ **À lancer après TOUTE retouche d'un `messages+intl-icu.*.yaml` ou d'un appelant.**
Ni `lint:yaml` ni `lint:twig` ne voient une faute ICU : le premier valide le YAML, le
second le Twig, et le motif n'est lu qu'au RENDU. En S148, deux motifs mal formés ont
fait tomber `/admin/emails` et `/formations/{id}/suivi` en **500** avec les deux lints
au vert. Ce script rejoue ces deux motifs comme cas de test (voir `icu_lint.py`).

Ne demande ni PHP ni l'extension `intl` : il tourne sur un poste sans la boîte.
"""
import re, sys, glob, subprocess
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from icu_lint import check
from yamlpath import load

LANGS = ('fr', 'en', 'de', 'es', 'it')
cats = {l: load('translations/messages+intl-icu.%s.yaml' % l) for l in LANGS}
keys = sorted(cats['fr'])

def unquote(v):
    v = v.strip()
    if v[:1] == '"' and v[-1:] == '"':
        return v[1:-1].replace('\\"', '"').replace('\\\\', '\\')
    return v

# ---- 1. syntaxe + conflits de type, par langue ----
problems = []
notes = []
pattern_args = {}
for k in keys:
    for l in LANGS:
        raw = cats[l].get(k)
        if raw is None:
            problems.append(('MANQUANT', k, l, 'absent du catalogue %s' % l))
            continue
        args, probs = check(unquote(raw[0]))
        pattern_args.setdefault(k, {})[l] = args
        for p in probs:
            problems.append(('MOTIF', k, l, p))

# ---- 2. les arguments que chaque appelant passe réellement ----
def span_end(src, start):
    depth, i = 0, start
    while i < len(src):
        if src[i] in '([{': depth += 1
        elif src[i] in ')]}':
            depth -= 1
            if depth == 0: return i
        i += 1
    return -1

ARG = re.compile(r"['\"]([A-Za-z0-9_]+)['\"]\s*(?::|=>)")
callers = {}
files = glob.glob('templates/**/*.twig', recursive=True) + glob.glob('src/**/*.php', recursive=True)
for path in files:
    src = open(path, encoding='utf-8').read()
    for k in keys:
        for q in ("'", '"'):
            tok = q + k + q
            pos = 0
            while True:
                idx = src.find(tok, pos)
                if idx < 0: break
                pos = idx + 1
                after = src[idx + len(tok):]
                m = re.match(r"\s*\|\s*trans\s*\(", after)
                if m:
                    op = idx + len(tok) + m.end() - 1
                elif re.match(r"\s*,\s*\[", after):          # PHP trans('k', [...])
                    op = src.find('[', idx + len(tok))
                elif re.match(r"\s*,\s*\[", src[idx + len(tok):]) is None and src[max(0,idx-2):idx] == '[':
                    op = src.find('[', idx + len(tok))       # flash ['k', [...]]
                elif re.match(r"\s*\|\s*trans\b", after):
                    # 'k'|trans  sans parenthèses = aucun argument
                    callers.setdefault(k, []).append((path, set()))
                    continue
                else:
                    # ⚠️ Le ternaire : `(cond ? 'a' : 'b')|trans({...})`. La clé n'est
                    # pas collée à `|trans`, mais elle est dans la parenthèse qui l'est.
                    close = src.find(')', idx)
                    m2 = re.match(r"\s*\|\s*trans\s*\(", src[close + 1:]) if close > 0 else None
                    if not m2:
                        continue
                    op = close + 1 + m2.end() - 1
                end = span_end(src, op)
                if end < 0: continue
                body = src[op:end + 1]
                callers.setdefault(k, []).append((path, set(ARG.findall(body))))

# ---- 3. confrontation ----
for k in keys:
    needed = set()
    for l in LANGS:
        needed |= pattern_args.get(k, {}).get(l, set())
    sites = callers.get(k, [])
    if not sites:
        problems.append(('ORPHELINE', k, '-', 'aucun appelant trouvé (clé morte ?)'))
        continue
    for path, passed in sites:
        missing = needed - passed
        extra = passed - needed
        if missing:
            problems.append(('ARG MANQUANT', k, path, 'le motif attend %s, l’appel ne passe pas %s'
                             % (sorted(needed), sorted(missing))))
        if extra:
            # ⚠️ Bénin : ICU ignore un argument qu'il n'utilise pas. Le cas normal est
            # le ternaire, où deux clés partagent UNE liste d'arguments — la plus
            # riche des deux la dicte. Signalé pour information, pas comme une faute.
            notes.append(('note', k, path, 'l’appel passe %s que le motif n’utilise pas (bénin)' % sorted(extra)))

print('clés ICU : %d × %d langues = %d motifs' % (len(keys), len(LANGS), len(keys) * len(LANGS)))
print('appelants trouvés : %d' % sum(len(v) for v in callers.values()))
print()
if not problems:
    print('✅ aucun problème — syntaxe, types et arguments concordent')
for kind, k, where, msg in notes:
    print('%-13s %-42s %s' % (kind, k, msg))
for kind, k, where, msg in problems:
    print('%-13s %-42s %s' % (kind, k, msg))
    if where not in ('-',):
        print('%-13s   ↳ %s' % ('', where))
