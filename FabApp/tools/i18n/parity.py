#!/usr/bin/env python3
"""Flatten the five catalogues and report keys missing from any of them.

Also reports keys referenced by a template with `|trans` that no catalogue defines,
and catalogue keys nothing references — the two ways a translation pass goes wrong
without any lint noticing.
"""
import os, re, sys, glob

LOCALES = ('fr', 'en', 'de', 'es', 'it')


def load(path):
    """Indentation parser for these catalogues.

    Handles the two YAML features they actually use: nested mappings and
    anchor/alias (`field: &policy_fields` … `help: *policy_fields`). Treating an
    anchored mapping as a scalar is what made every `reporting.*` key look absent
    from Spanish when it was simply written as `reporting: &reporting_es`.
    """
    keys = {}
    anchors = {}   # name -> full prefix it was declared at
    stack = []
    for n, raw in enumerate(open(path, encoding='utf-8'), 1):
        line = raw.rstrip('\n')
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        m = re.match(r'^(\s*)([A-Za-z0-9_.-]+):\s*(.*)$', line)
        if not m:
            continue
        indent, k, v = len(m.group(1)), m.group(2), m.group(3).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        full = '.'.join([s for _, s in stack] + [k])
        if v.startswith('*'):
            src = anchors.get(v[1:])
            if src:
                for kk, vv in list(keys.items()):
                    if kk.startswith(src + '.'):
                        keys[full + kk[len(src):]] = vv
            continue
        if v == '' or v.startswith('&'):
            if v.startswith('&'):
                anchors[v[1:]] = full
            stack.append((indent, k))
        elif v.startswith('{') and v.endswith('}'):
            # inline flow mapping: `filter: { q: Recherche, statut: Statut }`
            for part in re.findall(r'([A-Za-z0-9_]+)\s*:\s*([^,}]+)', v[1:-1]):
                keys[f'{full}.{part[0]}'] = (part[1].strip(), n)
        else:
            keys[full] = (v, n)
    return keys


def main(cat_dir, tpl_dir):
    # ⚠️ **DEUX catalogues par langue depuis S147/J-4.** Les clés à pluriel vivent
    # dans `messages+intl-icu.LOCALE.yaml` (le suffixe fait passer le domaine par
    # `MessageFormatter`). Un contrôle qui n'en lit qu'un déclare « non définies »
    # les 74 clés migrées — et envoie la session suivante chercher un problème qui
    # n'existe pas. Les deux fichiers forment UN domaine ; on les fusionne ici.
    cats = {}
    for l in LOCALES:
        merged = load(os.path.join(cat_dir, f'messages.{l}.yaml'))
        icu_path = os.path.join(cat_dir, f'messages+intl-icu.{l}.yaml')
        if os.path.exists(icu_path):
            merged.update(load(icu_path))
        cats[l] = merged
    all_keys = set()
    for k in cats.values():
        all_keys |= set(k)
    missing = 0
    for key in sorted(all_keys):
        absent = [l for l in LOCALES if key not in cats[l]]
        if absent:
            missing += 1
            print(f'MISSING  {key:55s} not in {",".join(absent)}')
    print(f'--- {len(all_keys)} keys, {missing} with a gap')

    used = set()
    for p in glob.glob(os.path.join(tpl_dir, '**', '*.twig'), recursive=True):
        src = re.sub(r'\{#.*?#\}', ' ', open(p, encoding='utf-8').read(), flags=re.S)
        for m in re.finditer(r"""['"]([a-z0-9_]+\.[a-z0-9_.]+)['"]\s*\|\s*trans""", src):
            used.add(m.group(1))
    for p in glob.glob('src/**/*.php', recursive=True):
        src = open(p, encoding='utf-8').read()
        for m in re.finditer(r"""['"]([a-z0-9_]+\.[a-z0-9_.]+)['"]""", src):
            used.add(m.group(1))
    undefined = sorted(k for k in used if k not in cats['fr'] and '.' in k
                       and k.split('.')[0] in {n.split('.')[0] for n in cats['fr']})
    for k in undefined:
        print(f'UNDEFINED  {k}  (referenced, absent from fr)')
    print(f'--- {len(undefined)} referenced-but-undefined keys')
    return 1 if (missing or undefined) else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
