# -*- coding: utf-8 -*-
"""Point 5 des dix : « aucune affordance morte ».
   ⚠️ Chaque alerte est RÉFUTÉE avant d'être comptée — un détecteur naïf produit
   surtout des faux positifs (leçon 2026-08-21)."""
import re, glob, collections
findings = collections.defaultdict(list)

for p in sorted(glob.glob('templates/**/*.twig', recursive=True)):
    src = open(p, encoding='utf-8').read()
    # ⚠️ Blanchir en GARDANT les sauts de ligne : sinon les numéros décalent et
    # l'audit désigne la mauvaise ligne — payé deux fois dans cette session.
    src_nc = re.sub(r'\{#.*?#\}',
                    lambda m: ''.join('\n' if c == '\n' else ' ' for c in m.group(0)),
                    src, flags=re.S)
    short = p.replace('templates/', '')
    for i, line in enumerate(src_nc.split('\n'), 1):
        # href="#" — un lien qui ne mène nulle part
        for m in re.finditer(r'<a\b[^>]*href="#"[^>]*>', line):
            tag = m.group(0)
            # réfutation : un onglet/accordéon piloté par Stimulus est légitime
            if 'data-action' in tag or 'data-controller' in tag or 'role="tab"' in tag:
                continue
            findings['lien href="#"'].append((short, i, tag[:110]))
        # bouton désactivé en dur (pas conditionnel)
        for m in re.finditer(r'<button\b[^>]*\bdisabled\b[^>]*>', line):
            tag = m.group(0)
            if '{{' in tag or '{%' in tag:      # conditionnel = état, pas affordance morte
                continue
            findings['bouton disabled en dur'].append((short, i, tag[:110]))
        # onclick="" vide, href="javascript:void(0)"
        if 'javascript:void' in line:
            findings['href javascript:void'].append((short, i, line.strip()[:110]))

for kind in sorted(findings):
    print('=== %s : %d' % (kind, len(findings[kind])))
    for f, i, t in findings[kind][:20]:
        print('   %-38s:%-4d %s' % (f, i, t))
if not findings:
    print('aucune affordance morte détectée')
