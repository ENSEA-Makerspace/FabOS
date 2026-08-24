# -*- coding: utf-8 -*-
"""
Valide les motifs ICU SANS PHP ni extension intl — ce que la boîte faisait, en statique.

Attrape les trois familles qui ont réellement cassé quelque chose ici :
  1. accolades déséquilibrées / branche `plural` sans `other`  → exception au rendu ;
  2. 🔴 MÊME argument utilisé nu `{x}` ET comme sélecteur `{x, plural, …}`
     → U_ARGUMENT_TYPE_MISMATCH, qui est un **500**, pas un repli silencieux ;
  3. argument du motif que l'appelant ne passe pas (ou l'inverse)
     → le motif brut s'affiche à l'écran.

⚠️ Un motif ICU n'est PAS du texte à trous : à l'intérieur d'un bloc `plural`, la
forme est `clé {message}` — et `{message}` n'est pas un argument. Un analyseur naïf
lit « # jour » comme un nom d'argument ; c'est l'erreur que ce fichier évite.
"""
import re

class IcuError(Exception):
    pass

SELECTORS = ('plural', 'select', 'selectordinal')
NAME = re.compile(r'[A-Za-z0-9_]+')

def _match(pattern, i):
    """i pointe sur '{' ; rend l'index du '}' correspondant."""
    depth = 0
    while i < len(pattern):
        if pattern[i] == '{':
            depth += 1
        elif pattern[i] == '}':
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise IcuError('accolade jamais fermée')

def _branches(body):
    """`, one {msg} other {msg}` → liste des messages de branche."""
    out, i = [], 0
    while i < len(body):
        if body[i] == '{':
            end = _match(body, i)
            out.append(body[i + 1:end])
            i = end + 1
        else:
            i += 1
    return out

def parse(pattern, usages=None):
    """Rend {arg: set(types)} ; type = 'bare' | 'plural' | 'select' | 'selectordinal'."""
    if usages is None:
        usages = {}
    i, n = 0, len(pattern)
    while i < n:
        if pattern[i] != '{':
            i += 1
            continue
        end = _match(pattern, i)
        inside = pattern[i + 1:end]
        head, _, rest = inside.partition(',')
        name = head.strip()
        if not NAME.fullmatch(name):
            raise IcuError('nom d’argument invalide : %r' % name[:40])
        kind, _, body = rest.partition(',')
        kind = kind.strip()
        if not rest:
            usages.setdefault(name, set()).add('bare')
        elif kind in SELECTORS:
            usages.setdefault(name, set()).add(kind)
            if kind == 'plural' and not re.search(r'(^|\s)other\s*\{', body):
                raise IcuError('{%s, plural} sans branche `other`' % name)
            for branch in _branches(body):
                parse(branch, usages)
        else:
            # number, date, time, spellout… : un scalaire, comme `bare`
            usages.setdefault(name, set()).add('bare')
        i = end + 1
    return usages

def check(pattern):
    """Rend (args, problèmes)."""
    try:
        usages = parse(pattern)
    except IcuError as e:
        return set(), ['SYNTAXE: %s' % e]
    problems = []
    if pattern.count('{') != pattern.count('}'):
        problems.append('SYNTAXE: %d accolades ouvrantes, %d fermantes'
                        % (pattern.count('{'), pattern.count('}')))
    for name, kinds in sorted(usages.items()):
        sel = kinds & set(SELECTORS)
        if sel and 'bare' in kinds:
            problems.append('TYPE: {%s} est utilisé nu ET comme %s '
                            '— U_ARGUMENT_TYPE_MISMATCH (500)' % (name, sorted(sel)[0]))
    return set(usages), problems
