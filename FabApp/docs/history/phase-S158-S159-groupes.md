# Phases S158 & S159 — les groupes deviennent le modèle

**2026-09-01 au 2026-09-03.** Parties d'une demande simple de l'opérateur — « des
groupes qui aillent au-delà de ceux qui existent, peut-être en fusionnant les
systèmes » — et finies avec les groupes comme **seule** source des rôles et des
forfaits.

---

## 🔴 Le fait mesuré d'où tout est parti

**`USER_GROUP_MEMBER` n'était écrit par RIEN.** La table existait depuis S133b,
les sept groupes intégrés étaient semés par la migration, et aucune surface ne
permettait de créer un groupe ni d'y mettre quelqu'un. Des sept, seuls comptaient
ceux qu'un **rôle** impliquait et l'audience résolue `user` — et « Stagiaires » ou
« Bénévoles », que `USAGE_RIGHTS_VISION.md` nomme, étaient inatteignables.

C'est la famille de défaut que le dépôt avait déjà nommée un étage plus haut, pour
`USAGE_RIGHT_ASSIGNMENT.groupId` : **un demi-modèle sans surface d'écriture se lit
comme une fonctionnalité et se comporte comme une absence.**

## Ce qui a été livré, dans l'ordre

| | |
|---|---|
| **S158a** | L'écran des groupes. La liste affiche l'appartenance **effective** — sur la boîte, « Administrateur global » 6 et « Staff » 2 avec **zéro** ligne stockée ; un écran qui n'aurait compté que le stocké aurait annoncé « 0 membre » sur des groupes qui ouvrent des droits |
| **S158b** | La même appartenance depuis la fiche du membre, écrite par le même dépôt : deux vues, pas deux surfaces |
| **S158c** | Le backfill rôle → groupe, **11 lignes**, et le filtre « groupe » sur la liste |
| **S159b→f** | La fusion : garde du dernier admin d'abord, entités, `getRoles()` en union puis groupes seuls, écritures déplacées, `UTILISATEUR_ROLE` supprimée |
| **S159 (1,3)** | Un forfait ne s'attribue qu'à un groupe ; les 3 attributions personnelles converties |
| **S159g** | L'appartenance **datée** |

---

## Les leçons, celles qui coûteront cher à réapprendre

🔴 **Un argument de sûreté qui n'est qu'affirmé n'en est pas un.** Le backfill
photographie les audiences de chaque compte avant, écrit, rephotographie avec un
résolveur **neuf**, et annule tout si un seul jeu de clés a bougé. Même discipline
pour la conversion des attributions. Les deux ont conclu « neutre » — mais parce
qu'elles l'ont **mesuré**.

🔴 **Un témoin mémoïsé se prouve lui-même.** `AudienceResolver` mémoïse par
instance : réutiliser celui d'avant l'écriture rend la photo d'avant. Trois
commandes ont dû construire un résolveur neuf pour dire quelque chose.

🔴 **Un témoin mal choisi accuse la mesure.** La sonde du palier de réservation
est tombée sur un ADMIN, dont le palier reste `admin` une fois staff. L'assertion
était fausse, pas le code.

🔴 **Chaque pas de la fusion a créé le défaut du pas suivant**, et c'est le motif
de toute la phase :
- le backfill a rendu « stocké » et « venu d'un rôle » simultanés, donc le bouton
  « Retirer » supprimait la ligne sans sortir la personne du groupe →
  `roleKeysFor()` POSE la question au lieu de la déduire ;
- l'union dans `getRoles()` a laissé `hasRoleNamed()` en arrière, donc `isStaff()`
  disait non pendant que les écrans disaient oui — et le **palier de quota** en
  dépendait ;
- retirer la relation Doctrine a cassé deux requêtes qui la joignaient encore, et
  ni `php -l` ni `lint:twig` ne l'ont vu. **Seul le balayage des routes l'a vu**,
  et seulement parce qu'il compte une page sans statut comme une erreur.

⚠️ **Une garde se pose AVANT d'être nécessaire.** Celle du dernier administrateur
a été écrite alors qu'elle était sans effet — le rôle étant encore la source.
Après la fusion, elle est ce qui empêche un verrouillage.

🔴 **Et une garde ne peut rien contre l'horloge.** D'où le refus d'une date de fin
sur une appartenance `admin` : elle expirerait un jour sans que personne n'ait
rien fait.

⚠️ **`NULL` veut dire « sans limite », des deux côtés.** Les onze lignes du
backfill n'ont pas de dates ; un filtre qui les prendrait pour expirées retirerait
`staff`, `admin` et `trainers` à tout le monde. La clause est écrite **une** fois,
dans `UserGroupSchema`.

---

## Ce que l'opérateur a tranché en route

- **Le mot** : « package » devient **forfait / bundle**, avant que la Phase H ne
  fasse de « package » une chose qu'on achète. Les tables ne bougent pas — la
  correspondance des deux vocabulaires est dans `PROJECT_STATE.md`.
- **Les forfaits ne s'attribuent qu'à des groupes.** Et quand j'ai objecté que le
  commerce ne rentrait pas — un achat est individuel, et la vision interdit qu'un
  remboursement révoque « un droit venu d'un groupe » — la réponse est venue de
  lui : **on n'achète pas un forfait, on achète une APPARTENANCE datée**. Ce qui
  supprime le cas particulier au lieu de le contourner.
- **Une ligne, plus un journal**, plutôt que plusieurs lignes : moins cher, et
  sans `DROP PRIMARY KEY`. ⚠️ À la condition que la ligne soit **dérivée** du
  journal, sinon ce sont deux vérités de plus.

## 🅿️ Ce qui reste

- **Le journal des appartenances** — il n'a de sens que le jour où une MACHINE
  écrit. Une table de journal sans écrivain serait un demi-modèle de plus, ce que
  cette phase a passé trois jours à réparer.
- **`ROLE`** n'accorde plus rien mais reste : le `down()` de
  `Version20260902100000` s'en sert pour reconstruire.
- **Les groupes à règle** (« tous ceux qui ont la formation Laser »), avec
  prévisualisation obligatoire. Les groupes imbriqués restent déconseillés.
