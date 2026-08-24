import re
def load(path):
    out, stack = {}, []
    for lineno, raw in enumerate(open(path, encoding='utf-8'), 1):
        if not raw.strip() or raw.lstrip().startswith('#'):
            continue
        indent = len(raw) - len(raw.lstrip())
        line = raw.strip()
        m = re.match(r'^("?)([A-Za-z0-9_.\-]+)\1:\s*(.*)$', line)
        if not m:
            continue
        key, rest = m.group(2), m.group(3).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path_ = [k for _, k in stack] + [key]
        if rest == '' or rest.startswith('&'):
            stack.append((indent, key))
            if rest.startswith('&'):
                out['.'.join(path_)] = (rest, lineno, indent)
        else:
            out['.'.join(path_)] = (rest, lineno, indent)
    return out
