# -*- coding: utf-8 -*-
"""Contrôles d'accessibilité qui sont des FAITS statiques, chacun réfuté avant d'être compté.
   ⚠️ Un détecteur naïf produit surtout des faux positifs (leçon 2026-08-21) : un bouton
   « sans nom » porte souvent un aria-label, un champ « sans étiquette » est enveloppé
   dans un <label>. On réfute d'abord."""
import re, glob, collections
def blank(m):
    return ''.join('\n' if c == '\n' else ' ' for c in m.group(0))
COMMENT = re.compile(r'\{#.*?#\}|<!--.*?-->', re.S)
STYLE = re.compile(r'<(style|script)\b.*?</\1>', re.S)

img_no_alt, input_no_label, btn_no_name = [], [], []
for p in sorted(glob.glob('templates/**/*.twig', recursive=True)):
    raw = open(p, encoding='utf-8').read()
    src = STYLE.sub(blank, COMMENT.sub(blank, raw))
    short = p.replace('templates/', '')
    for i, line in enumerate(src.split('\n'), 1):
        for m in re.finditer(r'<img\b[^>]*>', line):
            if 'alt=' not in m.group(0):
                img_no_alt.append((short, i, m.group(0)[:100]))
        for m in re.finditer(r'<input\b[^>]*>', line):
            tag = m.group(0)
            t = re.search(r'type="([a-z]+)"', tag)
            if t and t.group(1) in ('hidden', 'submit', 'button', 'image'):
                continue
            # réfutations : aria-label, aria-labelledby, ou un id repris par un <label for>
            if 'aria-label' in tag:
                continue
            idm = re.search(r'\bid="([^"]+)"', tag)
            if idm and ('for="%s"' % idm.group(1)) in src:
                continue
            # enveloppé dans un <label> — ouvert plus haut ET pas refermé, OU bien
            # ouvert et refermé autour du champ SUR LA MÊME LIGNE. ⚠️ Sans ce second
            # cas, `<label><input …> Texte</label>` est un faux positif : c'est la
            # forme la plus courante des cases à cocher de ce dépôt.
            before = '\n'.join(src.split('\n')[:i - 1])
            if before.count('<label') > before.count('</label>'):
                continue
            head = line[:m.start()]
            if head.count('<label') > head.count('</label>'):
                continue
            # champ Symfony rendu par le thème : le libellé vient de form_label
            if 'form_widget' in line or '{{ form' in line:
                continue
            input_no_label.append((short, i, tag[:100]))
        for m in re.finditer(r'<button\b[^>]*>(.*?)</button>', line):
            tag, inner = m.group(0), m.group(1)
            if 'aria-label' in tag or 'title=' in tag:
                continue
            text = re.sub(r'<[^>]+>', '', inner).strip()
            if text:
                continue
            btn_no_name.append((short, i, tag[:100]))

for title, rows in (('<img> sans alt', img_no_alt),
                    ('<input> sans étiquette atteignable', input_no_label),
                    ('<button> sans nom accessible', btn_no_name)):
    print('=== %s : %d' % (title, len(rows)))
    for f, i, t in rows[:14]:
        print('   %-40s:%-4d %s' % (f, i, t))
