# -*- coding: utf-8 -*-
"""Équilibre des blocs Twig — ce que `lint:twig` attraperait, sans PHP.
   ⚠️ Ce n'est PAS lint:twig : ça ne valide ni les expressions ni les filtres.
   Ça attrape la faute réaliste d'une édition scriptée : un `endif` de trop ou en moins.

   🔴 **Et depuis S177, le commentaire posé DANS un littéral de hash.**
   `{% include '…' with { a: 1, {# note #} b: 2 } %}` n'est pas du Twig valide :
   le gabarit ne compile pas. C'est arrivé TROIS fois dans ce dépôt en deux jours
   — à chaque fois en documentant un paramètre à l'endroit le plus naturel pour
   le lire. `lint:twig` l'attrape, mais seulement sur la boîte, après un
   déploiement ; ici c'est attrapé sur le Mac, avant.
   ⚠️ Le commentaire va JUSTE AU-DESSUS de la balise. C'est la seule place."""
import re, sys, glob
OPEN = {'if': 'endif', 'for': 'endfor', 'block': 'endblock', 'embed': 'endembed',
        'macro': 'endmacro', 'set': 'endset', 'verbatim': 'endverbatim',
        'apply': 'endapply', 'with': 'endwith', 'autoescape': 'endautoescape'}
CLOSE = {v: k for k, v in OPEN.items()}
TAG = re.compile(r'\{%-?\s*(\w+)(.*?)-?%\}', re.S)
COMMENT = re.compile(r'\{#.*?#\}', re.S)


def comments_in_hash(src):
    """Les lignes où un `{# … #}` vit à l'intérieur d'un `{% … %}`.

    Un commentaire dans un littéral de hash est indétectable après le passage de
    `COMMENT.sub('')` — d'où cette passe AVANT, sur la source brute. On cherche
    l'ouverture d'une balise, puis un `{#` avant le `%}` qui la ferme.
    """
    out, i = [], 0
    while True:
        start = src.find('{%', i)
        if start < 0:
            return out
        end = src.find('%}', start)
        if end < 0:
            return out
        inner = src[start + 2:end]
        if '{#' in inner:
            out.append(src[:start + 2 + inner.index('{#')].count('\n') + 1)
        i = end + 2

bad = 0
targets = sys.argv[1:] or sorted(glob.glob('templates/**/*.twig', recursive=True))
for p in targets:
    raw = open(p, encoding='utf-8').read()
    for ln in comments_in_hash(raw):
        print('🔴 %s:%d  commentaire {# … #} DANS une balise {%% … %%} — Twig refuse. '
              'Le mettre juste au-dessus.' % (p, ln))
        bad += 1
    src = COMMENT.sub('', raw)
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
