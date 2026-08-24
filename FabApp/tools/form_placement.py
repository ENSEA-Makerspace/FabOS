# -*- coding: utf-8 -*-
"""Placement, en distinguant ÉDITEUR et SOUCHE D'ACTION.

⚠️ Compter les `<form>` d'une page ment : un bouton « archiver » est un `<form>`
avec un jeton caché et rien d'autre. Ce qui pèse sur l'opérateur, c'est le nombre
d'ÉDITEURS — un formulaire qui demande au moins deux champs. La classification
J-22 fait déjà cette distinction (`S147-REVUE.md`).
"""
import re, glob, os
def blank(m): return ''.join('\n' if c=='\n' else ' ' for c in m.group(0))
COMMENT = re.compile(r'\{#.*?#\}|<!--.*?-->', re.S)
SS = re.compile(r'<(style|script)\b.*?</\1>', re.S)

def forms_of(src):
    """Découpe grossièrement chaque formulaire et compte ses champs visibles."""
    out = []
    for m in re.finditer(r'(\{\{\s*form_start|<form\b)', src):
        start = m.start()
        endm = re.search(r'(\{\{\s*form_end|</form>)', src[start:])
        body = src[start:start + (endm.end() if endm else 400)]
        n = len(re.findall(r'form_row\(', body))
        n += len(re.findall(r'<(?:input|select|textarea)\b', body)) - len(re.findall(r'type="hidden"', body))
        out.append(max(0, n))
    return out

rows = []
for p in sorted(glob.glob('templates/site/*.twig')):
    raw = open(p, encoding='utf-8').read()
    src = SS.sub(blank, COMMENT.sub(blank, raw))
    sizes = forms_of(src)
    if not sizes: continue
    editors = [s for s in sizes if s >= 2]
    stubs = [s for s in sizes if s < 2]
    if len(editors) < 2: continue
    sep = []
    if re.search(r'settings-card|admin-content-card', src): sep.append('carte')
    if re.search(r'settings-toc|admin-content-nav', src):   sep.append('sommaire')
    if re.search(r'<details', src):                          sep.append('repli')
    rows.append((len(editors), sum(editors), len(stubs), os.path.basename(p).replace('.html.twig',''), sep))

rows.sort(reverse=True)
print('%-34s %8s %8s %7s  %s' % ('écran', 'éditeurs', 'champs', 'souches', 'séparation'))
print('-'*82)
for ne, champs, ns, name, sep in rows:
    v = ' 🔴 sous-navigation' if ne >= 4 else (' ⚠️' if ne == 3 else '')
    print('%-34s %8d %8d %7d  %-22s%s' % (name, ne, champs, ns, ', '.join(sep) or '—', v))
