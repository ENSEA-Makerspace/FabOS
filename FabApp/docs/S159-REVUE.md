# Revue de fin de phase — S158 / S159, les groupes

**2026-09-03.** Mandat « designer d'Apple » : les PARCOURS, pas le diff. Clics
avant/après, évidence du chemin, frappes — surtout en cas d'erreur — et tout champ
demandé sans être indispensable.

Mesuré au navigateur sur les écrans rendus, pas lu dans le code.

**Les URLs de la revue** : `/admin/groupes`, `/admin/groupes/{id}`,
`/admin/utilisateurs`, `/admin/utilisateurs/{id}`, `/admin/usage-rights`,
`/admin/usage-rights/{id}/edit`.

---

## 🔴 R1 — On ne pouvait plus retirer personne d'un groupe intégré ✅ CORRIGÉ

**Le parcours cassé** : `/admin/groupes/3` → « Membres » → *aucun bouton*. Les deux
membres de Staff affichaient « par son rôle, et inscrit ici », sans moyen de les
retirer. Idem pour Administrateur global et Formateurs, et depuis la fiche du
membre aussi.

**Et la phrase était fausse** : `UTILISATEUR_ROLE` n'existe plus.

**La cause** — c'est le motif de toute la phase, une fois de plus.
`roleKeysFor()` demandait « resterait-elle dans ce groupe si la ligne
disparaissait ? » en lisant `getRoles()`. Tant que les rôles étaient une source
indépendante, la question avait un sens. Après le contract, `getRoles()` **dérive
des appartenances** : la question est devenue **circulaire** et répondait toujours
oui. Le bouton disparaissait donc partout, et le libellé accusait un rôle qui
n'existe plus.

✅ **Corrigé** : `roleKeysFor()` est retirée, il n'y a plus de distinction de
source à faire. Pour un groupe non virtuel, l'appartenance est une ligne, et elle
se retire. Vérifié : 2 boutons sur `staff`, 4 sur la fiche de Cédric, et la
mention est redevenue « inscrit ici ».

⚠️ **Ce que ça dit du reste** : il existait un chemin de secours — les cases
« Type de personne » — donc le défaut n'était pas un blocage total. C'est
exactement ce qui l'a rendu invisible.

---

## R2 — L'action principale de la fiche d'un forfait est REPLIÉE

`/admin/usage-rights/21/edit` → « Attribuer à un groupe » est dans un `<details>`
fermé. Or c'est **le seul moyen** de donner un forfait à qui que ce soit depuis
l'étape 1 de S159.

Le repli était juste quand quatre éditeurs « ajouter » se disputaient la place. Il
n'en reste qu'un, et c'est l'action que la page existe pour permettre.

🅿️ **Proposition** : le déplier, ou le sortir du repli. Un clic de moins sur le
geste le plus fréquent de l'écran.

---

## R3 — Une date se met à DEUX endroits, sans que rien ne dise lequel

Depuis S159g, « jusqu'à quand » s'écrit :

- sur l'**appartenance** (`/admin/groupes/{id}`, colonnes « Du » / « Jusqu'au ») ;
- sur l'**attribution du forfait au groupe** (`/admin/usage-rights/{id}/edit`).

Les deux s'appliquent — l'accès s'arrête à la première échéance atteinte. C'est
cohérent, mais **rien à l'écran ne l'explique**, et un opérateur qui veut « Sofia
jusqu'au 30 juin » a deux champs plausibles pour le dire, dont un seul est juste.

⚠️ C'est la forme adoucie du défaut que cette phase a passé trois jours à ranger :
deux endroits pour un fait.

🅿️ **Proposition** : garder la date sur l'appartenance (elle concerne une
personne) et retirer celles de l'attribution de groupe — ou, si une attribution
datée à tout un groupe a un usage réel, dire dans l'aide laquelle sert à quoi.
**À trancher par l'opérateur** : je n'ai pas d'exemple de forfait attribué à un
groupe pour une durée.

---

## R4 — « 6 · dont 6 inscrits ici » ne dit plus rien

Sur `/admin/groupes`, la seconde ligne de la colonne « Membres » est là pour
révéler un écart. Le jour où elle a été écrite, l'écart était réel : Staff comptait
**2 membres effectifs et 0 ligne**. Depuis le backfill et le contract, **les deux
nombres sont égaux sur les onze groupes**.

🅿️ **Proposition** : n'afficher la seconde ligne que lorsque les deux diffèrent.
Elle redeviendra utile le jour où des groupes à règle apporteront des membres sans
ligne — et d'ici là elle ne répète pas un nombre.

---

## R5 — Deux contrôles pour le même fait sur la fiche d'un membre

`/admin/utilisateurs/5` porte, à quelques centimètres d'écart :

- **« Groupes »** — dont Staff et Formateurs, avec leur bouton Retirer ;
- **« Type de personne »** — cases « Staff » et « Formateur », qui écrivent
  exactement les mêmes lignes depuis S159e.

C'est la règle de la phase, prise en défaut sur son propre écran.

🅿️ **Proposition** : retirer « Staff » et « Formateur » du panneau Type de
personne, qui garde « Réservable » — un fait différent, porté par une colonne à
lui. **À trancher** : les cases sont plus rapides que le menu du panneau Groupes,
et c'est un vrai argument.

---

## R6 — Le coût en clics du nouveau modèle, dit honnêtement

**Avant** : donner un forfait à quelqu'un = 1 écran, 1 menu, 1 bouton.
**Après** : 3 écrans — créer le groupe, y mettre la personne, attribuer le forfait
au groupe.

C'est **plus long pour un cas unique**, et c'est le prix assumé du modèle : le
groupe est réutilisable, et à la deuxième personne le coût s'inverse (1 clic
contre 3). L'aide le dit déjà — « pour une personne en particulier, faites-lui un
groupe ».

⚠️ **Ce qui manque pour que ce soit vrai à l'usage** : rien ne mène de la fiche
d'un forfait vers « créer le groupe qui va le porter ». On sort de l'écran, on
crée, on revient.

🅿️ **Proposition** : sur la fiche d'un forfait sans attribution, un lien
« créer un groupe pour ce forfait ».

---

## Ce que la revue n'a PAS trouvé, et qui aurait pu

- **Aucun champ obligatoire superflu.** Créer un groupe demande un nom, rien
  d'autre ; ajouter un membre demande une personne, les deux dates sont
  facultatives ; la saisie d'un forfait n'exige rien.
- **Aucune affordance morte** : les boutons refusés le sont côté serveur, avec une
  phrase — dernier administrateur, audience résolue, groupe intégré, date de fin
  sur `admin`. Les messages sont des phrases, pas des codes.
- **Aucune régression d'accès** : les trois commandes de la phase (backfill,
  ombre, conversion) ont mesuré avant/après et auraient annulé sur écart.

## Ce que la revue ne couvre pas

⚠️ Elle est faite sur des pages **rendues par la console**, pas par une session de
navigateur authentifiée : la validité des formulaires, les jetons CSRF et les
messages de flash **en situation réelle** ne sont pas mesurés ici. Les gardes
elles-mêmes le sont, par la sonde `app:s153:package-probe`.
