# -*- coding: utf-8 -*-
"""Équilibre des blocs Twig — ce que `lint:twig` attraperait, sans PHP.
   ⚠️ Ce n'est PAS lint:twig : ça ne valide ni les expressions ni les filtres.
   Ça attrape la faute réaliste d'une édition scriptée : un `endif` de trop ou en moins."""
import re, sys, glob
OPEN = {'if': 'endif', 'for': 'endfor', 'block': 'endblock', 'embed': 'endembed',
        'macro': 'endmacro', 'set': 'endset', 'verbatim': 'endverbatim',
        'apply': 'endapply', 'with': 'endwith', 'autoescape': 'endautoescape'}
CLOSE = {v: k for k, v in OPEN.items()}
TAG = re.compile(r'\{%-?\s*(\w+)(.*?)-?%\}', re.S)
COMMENT = re.compile(r'\{#.*?#\}', re.S)

bad = 0
targets = sys.argv[1:] or sorted(glob.glob('templates/**/*.twig', recursive=True))
for p in targets:
    src = COMMENT.sub('', open(p, encoding='utf-8').read())
    stack = []
    for m in TAG.finditer(src):
        tag, rest = m.group(1), m.group(2)
        if tag == 'set' and '=' in rest:      # {% set x = … %} n'ouvre rien
            continue
        if tag in OPEN:
            stack.append((tag, src[:m.start()].count('\n') + 1))
        elif tag in CLOSE:
            want = CLOSE[tag]
            if not stack:
                print('🔴 %s:%d  %s sans ouverture' % (p, src[:m.start()].count('\n') + 1, tag)); bad += 1
            elif stack[-1][0] != want:
                print('🔴 %s:%d  %s ferme %s ouvert ligne %d'
                      % (p, src[:m.start()].count('\n') + 1, tag, stack[-1][0], stack[-1][1])); bad += 1
                stack.pop()
            else:
                stack.pop()
    for tag, ln in stack:
        print('🔴 %s:%d  %s jamais fermé' % (p, ln, tag)); bad += 1
print('%d gabarits vérifiés, %d anomalie(s)' % (len(targets), bad))
