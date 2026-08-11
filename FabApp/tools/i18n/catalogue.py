#!/usr/bin/env python3
"""Merge keys into the five FabOS message catalogues without duplicating a namespace.

A second `admin_dashboard:` block at the end of the file would silently shadow the
first one under YAML's last-wins rule, so keys are inserted into the existing block.

Set `CAT` to the translations directory before calling anything. See README.md.
"""
import io, os, re

LOCALES = ('fr', 'en', 'de', 'es', 'it')
CAT = None  # absolute path to translations/, set by the caller


def path(locale):
    return os.path.join(CAT, f'messages.{locale}.yaml')


def quote(v):
    return '"' + v.replace('\\', '\\\\').replace('"', '\\"') + '"'


def block_bounds(lines, ns):
    start = None
    for i, l in enumerate(lines):
        if l.rstrip('\n') == ns + ':':
            start = i
            break
    if start is None:
        return None, None
    end = len(lines)
    for i in range(start + 1, len(lines)):
        l = lines[i]
        if l.strip() and not l[0].isspace() and not l.lstrip().startswith('#'):
            end = i
            break
    # trim trailing blank/comment lines back out of the block
    while end > start + 1 and lines[end - 1].strip() == '':
        end -= 1
    return start, end


def add_keys(locale, ns, pairs, comment=None):
    p = path(locale)
    lines = io.open(p, encoding='utf-8').read().splitlines(keepends=True)
    start, end = block_bounds(lines, ns)
    new = []
    if start is None:
        if lines and not lines[-1].endswith('\n'):
            lines[-1] += '\n'
        lines.append('\n')
        if comment:
            lines.append('# ' + comment + '\n')
        lines.append(ns + ':\n')
        start, end = block_bounds(lines, ns)
    existing = set()
    for l in lines[start + 1:end]:
        m = re.match(r'\s{4}([A-Za-z0-9_]+):', l)
        if m:
            existing.add(m.group(1))
    for k, v in pairs:
        if k in existing:
            continue
        new.append(f'    {k}: {quote(v)}\n')
    if not new:
        return 0
    lines[end:end] = new
    io.open(p, 'w', encoding='utf-8').write(''.join(lines))
    return len(new)


def drop_keys(locale, ns, keys):
    p = path(locale)
    lines = io.open(p, encoding='utf-8').read().splitlines(keepends=True)
    start, end = block_bounds(lines, ns)
    if start is None:
        return 0
    keep, dropped = [], 0
    for i, l in enumerate(lines):
        if start < i < end:
            m = re.match(r'\s{4}([A-Za-z0-9_]+):', l)
            if m and m.group(1) in keys:
                dropped += 1
                continue
        keep.append(l)
    io.open(p, 'w', encoding='utf-8').write(''.join(keep))
    return dropped
