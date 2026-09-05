# -*- coding: utf-8 -*-
"""
🔴 Attrape le message qui perd son propre libellé à l'écran.

**Le défaut, mesuré le 2026-09-05.** `/machines/{id}` affichait « 1 » là où le
catalogue anglais dit « Bookings: %count% ». Le catalogue était juste, la
substitution était juste, et l'écran mentait quand même.

**Pourquoi.** Dès qu'un paramètre `%count%` NUMÉRIQUE est passé, Symfony traite le
message comme une forme plurielle et le fait passer par `TranslatorTrait::trans`.
Ce lecteur reconnaît la vieille syntaxe explicite `clé: message` — et **jette tout
ce qui précède les deux-points** :

    'Bookings: %count%'      → '1'              🔴 le libellé disparaît
    'Badge scans: %count%'   → 'Badge scans: 1' ✅ l'espace sauve « Badge scans »
    'Réservations : %count%' → 'Réservations : 1' ✅ l'espace typographique français

⚠️ **C'est silencieux, et c'est le pire.** Aucune exception, aucun journal : la
page rend un nombre nu. Et ça ne se voit que dans les langues où le libellé avant
les deux-points est un SEUL mot — le français y échappait par sa typographie, donc
les quatre autres langues étaient cassées pendant que la nôtre allait bien.

**La règle de la maison.** Un message sans forme plurielle ne passe pas `%count%` :
il passe `%n%`. Un nom de paramètre qui ne déclenche rien ne peut rien casser.
Ce fichier interdit le retour de `%count%` dans un message qui commence par
`unMot:`.

Usage :  python3 tools/i18n/count_colon.py            (depuis FabApp/)
Sortie :  liste des messages piégés, code 1 s'il y en a.
"""
import glob
import os
import re
import sys

# `^\w+:` — un seul mot, puis les deux-points : exactement ce que le lecteur de
# formes plurielles de Symfony prend pour une clé et enlève.
TRAP = re.compile(r'^\w+:')
LINE = re.compile(r'^(\s*)([\w.-]+):\s*(?:"(.*)"|\'(.*)\'|(.*?))\s*$')


def main() -> int:
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    hits = []
    for path in sorted(glob.glob(os.path.join(root, 'translations', 'messages.*.yaml'))):
        with open(path, encoding='utf-8') as handle:
            for number, line in enumerate(handle, 1):
                match = LINE.match(line.rstrip('\n'))
                if not match:
                    continue
                value = match.group(3) or match.group(4) or match.group(5) or ''
                if '%count%' in value and TRAP.match(value):
                    hits.append((os.path.relpath(path, root), number, match.group(2), value))

    for path, number, key, value in hits:
        print(f'{path}:{number}  {key} = {value}')
        print(f'    → rendra « {value.split(":", 1)[1].strip()} », le libellé est perdu. Utiliser %n%.')

    print(f'{len(hits)} message(s) piégé(s)')
    return 1 if hits else 0


if __name__ == '__main__':
    sys.exit(main())
