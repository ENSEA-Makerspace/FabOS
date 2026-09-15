#!/usr/bin/env python3
"""Les couleurs de MARQUE écrites en dur dans les gabarits du site (S166).

🔴 **Le défaut.** `#9E1B56` et `#6b7280` étaient écrits à la main dans les
gabarits du site. Une installation qui change de couleur gardait donc le magenta
de FabOS à ces endroits-là : un éditeur de palette qui laisse des dizaines
d'endroits ignorer la palette ne change pas le thème, il le CONTREDIT. Et un
littéral de texte est en plus un trou de mode sombre — du bordeaux sur un panneau
sombre ne se lit pas.

⚠️ **Le compte de la feuille de route (66, le 2026-09-05) était à la fois trop
haut et trop bas**, ce qui est exactement pourquoi cet outil existe :
  — trop bas : il ne comptait que la forme `#9E1B56`, pas `rgba(158, 27, 86, …)`,
    qui est la même couleur écrite autrement, quinze fois de plus ;
  — trop haut : il comptait des littéraux CITÉS dans des commentaires — qui
    documentent une correction passée — et ceux de `event-ticket.html.twig`.

🔴 **Trois exclusions, et chacune a une raison qui tient :**

  `templates/emails/` — un client de messagerie ne sait pas lire `var()`. Le
  littéral y est la BONNE réponse, pas une dette.

  `event-ticket.html.twig` — délibérément autonome, son propre commentaire le
  dit : « no site stylesheet », parce qu'un billet s'ouvre sur un téléphone avec
  un mauvais réseau et s'imprime sur ce qui traîne. Sans feuille du site, il n'y
  a pas de jeton à lire.

  `admin-design.html.twig` — c'est la PAGE QUI DOCUMENTE le système de design :
  les hex y sont le SUJET, dans des `<code>`, pas du style. Une première version
  de cet outil a réécrit une de ces phrases et l'a rendue absurde (« les icônes
  portaient `stroke="var(--color-primary)"`, le hex littéral de l'accent »).

⚠️ **Les commentaires sont blanchis avant l'analyse** — Twig `{# … #}` et CSS
`/* … */`. Un littéral cité dans un commentaire explique pourquoi il a disparu ;
le signaler ferait supprimer l'explication.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent / 'templates' / 'site'
SKIP = {'event-ticket.html.twig', 'admin-design.html.twig'}

PATTERNS = [
    (re.compile(r'#9[eE]1[bB]56\b'), 'var(--color-primary) — ou var(--color-primary-text) pour du TEXTE'),
    (re.compile(r'#7a1542\b', re.I), 'color-mix(in srgb, var(--color-primary) 78%, black)'),
    (re.compile(r'#6b7280\b', re.I), 'var(--color-text-light)'),
    (re.compile(r'rgba\(\s*158\s*,\s*27\s*,\s*86\s*,'), 'color-mix(in srgb, var(--color-primary) N%, transparent)'),
]


def blank_comments(line, state):
    """Remplace les commentaires par des espaces, en gardant les colonnes."""
    out, i = [], 0
    twig, css = state
    while i < len(line):
        if twig or css:
            close = '#}' if twig else '*/'
            j = line.find(close, i)
            if j == -1:
                out.append(' ' * (len(line) - i))
                i = len(line)
            else:
                out.append(' ' * (j + 2 - i))
                i = j + 2
                twig = css = False
        else:
            t = line.find('{#', i)
            c = line.find('/*', i)
            nxt = min(x for x in (t, c, len(line)) if x != -1)
            out.append(line[i:nxt])
            i = nxt
            if i < len(line):
                if t == i:
                    twig = True
                else:
                    css = True
    return ''.join(out), (twig, css)


def main() -> int:
    findings = []
    scanned = 0
    for path in sorted(ROOT.rglob('*.html.twig')):
        if path.name in SKIP:
            continue
        scanned += 1
        state = (False, False)
        for n, line in enumerate(path.read_text().split('\n'), 1):
            visible, state = blank_comments(line, state)
            for pattern, fix in PATTERNS:
                if pattern.search(visible):
                    findings.append((path.relative_to(ROOT.parent.parent), n, pattern.search(visible).group(0), fix))

    for rel, n, found, fix in findings:
        print(f'{str(rel):52s}:{n:<5d} {found}  →  {fix}')

    print(f'\n{scanned} gabarit(s) de site, {len(findings)} couleur(s) de marque en dur')
    return 1 if findings else 0


if __name__ == '__main__':
    sys.exit(main())
