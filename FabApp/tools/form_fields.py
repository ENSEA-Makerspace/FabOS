# -*- coding: utf-8 -*-
"""
🔴 Attrape le champ de formulaire qui sort APRÈS le bouton « Enregistrer ».

**Le défaut, deux fois dans ce dépôt.** Un `FormType` déclare un champ ; le
gabarit qui le rend énumère ses champs à la main ; on ajoute un `->add()` et on
oublie le gabarit. Twig ne se plaint pas : `form_end()` appelle `form_rest()`,
qui déverse les champs oubliés **après** le bouton d'envoi et **sans le thème**.

    S175 : `accessPoint` sur `/admin/rfid-readers/{id}/edit`
    (avant) : deux champs de `MachineAdminType` dans aucune des deux listes

⚠️ **Aucun outil ne le voyait.** `lint:twig` valide une syntaxe, `form_placement`
compte des éditeurs par écran. Celui-ci compare ce que le `FormType` DÉCLARE à ce
que le gabarit REND — la seule question qui attrape ce défaut.

**Comment il apparie type et gabarit** : dans le corps d'une même méthode de
contrôleur, un `createForm(XType::class)` et un `render('site/Y.html.twig')`.
C'est l'appariement que fait le lecteur humain, et il n'a pas besoin d'être
exhaustif pour être utile — ce qu'il ne sait pas apparier, il le dit.

⚠️ **Un gabarit qui déroule `SECTIONS` est vérifié AUTREMENT** : on compare alors
les champs déclarés à ceux que la constante énumère, ce qui est le vrai contrat
de `_form_sections.html.twig`.

Usage :  python3 tools/form_fields.py            (depuis FabApp/)
Sortie :  les champs déclarés et jamais rendus, code 1 s'il y en a.
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ADD = re.compile(r"->add\(\s*'([A-Za-z0-9_]+)'")
SECTIONS_FIELDS = re.compile(r"'fields'\s*=>\s*\[([^\]]*)\]", re.S)
QUOTED = re.compile(r"'([A-Za-z0-9_]+)'")
CREATE_FORM = re.compile(r"createForm\(\s*([A-Za-z0-9_\\]+)::class")
RENDER = re.compile(r"render\(\s*'(site/[A-Za-z0-9_.-]+\.html\.twig)'")
METHOD = re.compile(r"^    (?:public|private|protected)\s+function\s+\w+", re.M)

# ⚠️ Jamais rendus par un gabarit, et c'est normal : le bouton est tiré
# explicitement, et un jeton CSRF n'est pas un champ de saisie.
IGNORED = {'save', 'submit', '_token'}



# ⚠️ **Le nom de la variable de formulaire n'est PAS toujours `form`.**
# `/admin/settings`, `/admin/emails`, `/admin/network` et `/admin/wizard` rendent
# plusieurs formulaires d'un coup et les tiennent dans un tableau :
# `form_row(forms.identity.origin)`. Un motif `\bform\.` accusait donc 17 écrans
# parfaitement corrects — la faute que `ctor_arity.py` a déjà commise une fois.
# On retient le DERNIER segment de ce qui est passé aux fonctions de rendu.
RENDER_CALL = re.compile(r'\bform_(?:row|widget|label|help|errors)\(\s*([A-Za-z0-9_.\[\]\'"]+)')


INCLUDE = re.compile(r"""{%\s*(?:include|embed)\s+['"]([^'"]+\.html\.twig)['"]""")


def with_includes(tpl: str, depth: int = 2) -> str:
    """La source du gabarit ET celle des partiels qu'il inclut.

    🔴 **Sans ça, l'outil criait au loup sur un formulaire rendu dans un
    partiel** (S183b) : `formation-messages` et `trainer-thread` incluent tous
    deux `_thread.html.twig`, qui rend le champ. Deux faux « champ perdu » sur un
    formulaire correct — et un outil qui crie au loup apprend à ne plus le lire.
    ⚠️ Deux niveaux, pas une récursion : les partiels de formulaire de ce dépôt
    n'en incluent pas d'autres qui rendraient des champs, et une récursion sans
    garde de cycle serait du code sans cas d'usage.
    """
    full = os.path.join(ROOT, 'templates', tpl)
    if not os.path.exists(full):
        return ''
    html = open(full, encoding='utf-8').read()
    if depth > 0:
        for child in INCLUDE.findall(html):
            html += '\n' + with_includes(child, depth - 1)
    return html


def rendered_fields(html: str) -> set:
    """Les noms de champs qu'un gabarit rend, quel que soit le nom de la variable."""
    out = set()
    for expr in RENDER_CALL.findall(html):
        segment = expr.replace('"', "'").split('.')[-1]
        segment = segment.split('[')[0].strip("'")
        if segment:
            out.add(segment)
    # Les boucles `{% for field in form %}` rendent tout : le gabarit ne peut
    # alors rien perdre, et prétendre le contraire serait un faux positif.
    if re.search(r'{%\s*for\s+\w+\s+in\s+[A-Za-z0-9_.]*form[A-Za-z0-9_.]*\s*%}', html):
        out.add('*')
    return out


def form_types():
    """{ 'RfidReaderAdminType': (chemin, {champs déclarés}, {champs de SECTIONS} | None) }"""
    out = {}
    # ⚠️ **Récursif, et ça compte.** `src/Form/` a cinq sous-dossiers
    # (`Admin`, `Emails`, `FormationContent`, `Settings`, `UsageRights`). Un
    # glob non récursif en voyait 18 sur 30 et n'annonçait AUCUN type non
    # apparié : un vert qui ne mesurait que la moitié du sujet.
    for path in sorted(glob.glob(os.path.join(ROOT, 'src', 'Form', '**', '*.php'), recursive=True)):
        body = open(path, encoding='utf-8').read()
        declared = {f for f in ADD.findall(body)} - IGNORED
        if not declared:
            continue
        sections = None
        if 'const SECTIONS' in body:
            head = body.split('const SECTIONS', 1)[1]
            sections = set()
            for group in SECTIONS_FIELDS.findall(head.split('public function', 1)[0]):
                sections |= set(QUOTED.findall(group))
        out[os.path.basename(path)[:-4]] = (path, declared, sections)
    return out


def method_bodies(path):
    body = open(path, encoding='utf-8').read()
    starts = [m.start() for m in METHOD.finditer(body)]
    for i, start in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(body)
        yield body[start:end]


def main() -> int:
    types = form_types()
    pairs = {}          # type -> set(templates)
    for path in sorted(glob.glob(os.path.join(ROOT, 'src', 'Controller', '**', '*.php'), recursive=True)):
        for chunk in method_bodies(path):
            forms = {f.split('\\')[-1] for f in CREATE_FORM.findall(chunk)}
            tpls = set(RENDER.findall(chunk))
            for form in forms:
                pairs.setdefault(form, set()).update(tpls)

    problems, unpaired = [], []

    for name, (path, declared, sections) in sorted(types.items()):
        # 1. Le contrat de SECTIONS : la constante doit couvrir tout ce qui est déclaré.
        if sections is not None:
            missing = declared - sections
            if missing:
                problems.append((name, 'SECTIONS', sorted(missing)))
            continue

        # 2. Sinon, le gabarit doit nommer chaque champ.
        tpls = pairs.get(name, set())
        if not tpls:
            unpaired.append(name)
            continue
        for tpl in sorted(tpls):
            full = os.path.join(ROOT, 'templates', tpl)
            if not os.path.exists(full):
                continue
            html = with_includes(tpl)
            if 'SECTIONS' in html:
                continue
            rendered = rendered_fields(html)
            if '*' in rendered:
                continue
            missing = declared - rendered
            if missing:
                problems.append((name, tpl, sorted(missing)))

    for name, where, missing in problems:
        print(f'🔴 {name} → {where}')
        print(f'    déclaré mais jamais rendu : {", ".join(missing)}')
        print('    → sortira par form_rest(), APRÈS le bouton et sans thème.')

    if unpaired:
        # ⚠️ Dit, pas caché : un type qu'on n'a pas su apparier n'est pas un type
        # sain, c'est un type non mesuré. Le silence serait un faux vert.
        print(f'\n⚠️ {len(unpaired)} type(s) sans gabarit apparié, donc NON vérifiés :')
        print('   ' + ', '.join(sorted(unpaired)))

    print(f'\n{len(types)} FormType, {len(problems)} champ(s) perdu(s)')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
