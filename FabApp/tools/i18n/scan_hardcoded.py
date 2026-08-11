#!/usr/bin/env python3
"""Count literal human-facing strings still sitting in Twig templates.

Heuristic, not a proof: strips comments, <style>, <script>, then reports text
nodes and human-facing attribute values that still contain prose.
"""
import re, sys, os, json

TEXT_ATTRS = ('placeholder', 'title', 'alt', 'aria-label', 'value', 'label', 'content')

def strip(src):
    src = re.sub(r'\{#.*?#\}', ' ', src, flags=re.S)
    src = re.sub(r'<style\b.*?</style>', ' ', src, flags=re.S | re.I)
    src = re.sub(r'<script\b.*?</script>', ' ', src, flags=re.S | re.I)
    src = re.sub(r'<pre\b.*?</pre>', ' ', src, flags=re.S | re.I)   # shell/code samples
    src = re.sub(r'<code\b.*?</code>', ' ', src, flags=re.S | re.I)
    return src

def is_prose(s):
    s = s.strip()
    if not s:
        return False
    # strip HTML entities, then require a run of >=3 letters
    s = re.sub(r'&[a-zA-Z]+;|&#\d+;', ' ', s)
    return bool(re.search(r'[A-Za-zÀ-ÖØ-öø-ÿ]{3,}', s))

def scan(path):
    src = strip(open(path, encoding='utf-8').read())
    hits = []
    # text nodes
    for m in re.finditer(r'>([^<>]+)<', src):
        t = m.group(1)
        # remove twig constructs, keep the rest
        rest = re.sub(r'\{\{.*?\}\}|\{%.*?%\}', ' ', t, flags=re.S)
        if is_prose(rest):
            hits.append(('text', ' '.join(rest.split())))
    # attributes
    for m in re.finditer(r'\b(' + '|'.join(TEXT_ATTRS) + r')\s*=\s*"([^"]*)"', src, flags=re.I):
        v = m.group(2)
        rest = re.sub(r'\{\{.*?\}\}|\{%.*?%\}', ' ', v, flags=re.S)
        if is_prose(rest):
            hits.append((m.group(1).lower(), ' '.join(rest.split())))
    # twig hash labels: label: 'Foo'
    for m in re.finditer(r"\b(label|empty|title|placeholder|heading)\s*:\s*'([^']+)'(?!\s*\|\s*trans)", src):
        if is_prose(m.group(2)):
            hits.append(('hash:' + m.group(1), m.group(2)))
    for m in re.finditer(r'\b(label|empty|title|placeholder|heading)\s*:\s*"([^"]+)"(?!\s*\|\s*trans)', src):
        if is_prose(m.group(2)):
            hits.append(('hash:' + m.group(1), m.group(2)))
    return hits

if __name__ == '__main__':
    targets = sys.argv[1:]
    if len(targets) == 1 and os.path.isdir(targets[0]):
        root = targets[0]
        targets = []
        for d, _, fs in os.walk(root):
            for f in fs:
                if f.endswith('.twig'):
                    targets.append(os.path.join(d, f))
    rows = []
    for p in sorted(targets):
        h = scan(p)
        if h:
            rows.append((len(h), p, h))
    rows.sort(reverse=True, key=lambda r: r[0])
    total = 0
    for n, p, h in rows:
        total += n
        print(f'{n:4d}  {p}')
        if os.environ.get('DETAIL'):
            for kind, s in h:
                print(f'        [{kind}] {s[:120]}')
    print(f'--- {total} literals across {len(rows)} templates')
