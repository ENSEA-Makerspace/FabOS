#!/usr/bin/env python3
"""Un `new Foo(...)` passe-t-il assez d'arguments ?

🔴 **Le trou que ce script bouche.** `php -l` vérifie la SYNTAXE et `lint:twig`
les gabarits ; ni l'un ni l'autre ne compile un appel. Un constructeur qui gagne
un paramètre laisse donc derrière lui des `new Foo($a)` qui ne planteront qu'à
l'exécution — et si la ligne est sur un chemin rare (`--write` d'une commande,
une branche d'erreur), rien ne la touche avant le jour où on en a besoin.

Mesuré le 2026-09-03 : `S158BackfillGroupsCommand` construisait
`new AudienceResolver($conn)` avec UN argument depuis que S159g en avait exigé
deux. C'est la commande qui VÉRIFIE que le backfill ne change rien — un témoin
cassé, donc pire qu'un témoin absent, et sur une ligne que ni la sonde ni le
balayage de routes ne peuvent atteindre.

⚠️ **Volontairement conservateur.** Il ne signale que le cas certain : moins
d'arguments que de paramètres OBLIGATOIRES, sur une classe définie dans `src/`,
avec des arguments comptables. Tout ce qui est ambigu — `...$args`, un appel
imbriqué qu'on ne sait pas découper — est ignoré plutôt que deviné : un linter
qui crie à tort n'est plus lu.
"""
import glob, os, re, sys

SRC = 'src'


def constructors():
    """classe courte → (obligatoires, total), pour les classes de src/."""
    out = {}
    for path in glob.glob(os.path.join(SRC, '**', '*.php'), recursive=True):
        # ⚠️ Les commentaires partent AVANT de compter : ce dépôt écrit des
        # docblocks À L'INTÉRIEUR des listes de paramètres (`PackageSpec`), et
        # les compter comme des paramètres faisait crier ce script sur onze
        # appels parfaitement corrects — un linter qui crie à tort n'est plus lu,
        # ce que son propre en-tête annonce.
        src = re.sub(r'/\*.*?\*/|//[^\n]*', '', open(path, encoding='utf-8').read(), flags=re.S)
        m = re.search(r'\b(?:final\s+|abstract\s+|readonly\s+)*class\s+(\w+)', src)
        if not m:
            continue
        name = m.group(1)
        c = re.search(r'function\s+__construct\s*\(', src)
        if not c:
            out[name] = (0, 0)
            continue
        # découpe équilibrée de la liste de paramètres
        i = c.end()
        depth, start = 1, i
        while i < len(src) and depth:
            depth += {'(': 1, ')': -1}.get(src[i], 0)
            i += 1
        params = src[start:i - 1]
        parts, depth, buf = [], 0, ''
        for ch in params:
            depth += {'(': 1, '[': 1, ')': -1, ']': -1}.get(ch, 0)
            if ch == ',' and depth == 0:
                parts.append(buf); buf = ''
            else:
                buf += ch
        if buf.strip():
            parts.append(buf)
        parts = [p for p in parts if p.strip()]
        if any('...' in p for p in parts):
            continue  # variadique : on ne conclut pas
        # Obligatoire = sans valeur par défaut. ⚠️ Un `= []` ou `= true` sur une
        # propriété promue compte comme un défaut, comme sur un paramètre ordinaire.
        required = sum(1 for p in parts if '=' not in p)
        out[name] = (required, len(parts))
    return out


def call_args(src, open_paren):
    depth, i, buf, parts = 1, open_paren + 1, '', []
    while i < len(src) and depth:
        ch = src[i]
        depth += {'(': 1, '[': 1, ')': -1, ']': -1}.get(ch, 0)
        if depth == 0:
            break
        if ch == ',' and depth == 1:
            parts.append(buf); buf = ''
        else:
            buf += ch
        i += 1
    if buf.strip():
        parts.append(buf)
    return [p for p in parts if p.strip()], i


def main():
    ctors = constructors()
    problems = []
    for path in glob.glob(os.path.join(SRC, '**', '*.php'), recursive=True):
        src = open(path, encoding='utf-8').read()
        stripped = re.sub(r'/\*.*?\*/|//[^\n]*', '', src, flags=re.S)
        for m in re.finditer(r'\bnew\s+(\w+)\s*\(', stripped):
            name = m.group(1)
            if name not in ctors:
                continue
            required, _total = ctors[name]
            args, _ = call_args(stripped, m.end() - 1)
            if any('...' in a for a in args):
                continue
            # ⚠️ **Les arguments NOMMÉS ne se comptent pas.** `new Foo(bar: 1)`
            # peut sauter tous les paramètres à défaut ; conclure sur le nombre
            # serait faux à tous les coups.
            if any(re.match(r'\s*\w+\s*:(?!:)', a) for a in args):
                continue
            if len(args) < required:
                line = stripped[:m.start()].count('\n') + 1
                problems.append((path, line, name, len(args), required))

    for path, line, name, got, want in problems:
        print(f'🔴 {path}:{line}  new {name}(...) — {got} argument(s) pour {want} obligatoire(s)')
    print(f'--- {len(ctors)} classes, {len(problems)} appel(s) trop court(s)')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
