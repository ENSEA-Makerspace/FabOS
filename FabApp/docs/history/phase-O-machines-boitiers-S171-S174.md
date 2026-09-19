# Phase O — Machines & boîtiers (S171–S174)

**Planifiée le 2026-09-04, CLOSE le 2026-09-05.** D'après les huit planches
Équipement et la revue Sol (`/admin/references`). Elle absorbe les **trois P0 de
sécurité** vérifiés le même jour.

**Le résultat** : l'API des boîtiers est `fail-closed`, un boîtier a sept états
distinguables au lieu d'un booléen, la fiche machine est séparée en deux publics
(membre / exploitation), et la matière est ramenée à **une seule vérité**.

---

## 🔴 La règle des trois phases issues des planches

⚠️ **Cette règle reste vivante** : elle est conservée dans `ROADMAP.md`, en tête
de la Phase O, parce que la Phase P et la Phase Q la citent. Elle est reproduite
ici pour que le récit se lise seul.

**On AMÉLIORE, on ne refait pas.** L'opérateur, 2026-09-04 : *« les screenshots
ont de bonnes idées, dérives-en des upgrades, pas des refontes entières »*.
Et une précision qui restreint encore, donnée le 2026-09-05 : *« les screenshots
sont pour de la PRÉSENTATION DE CONTENU, on ne change pas le thème ou autre »*.
Donc une planche ne dit RIEN sur les couleurs, la typographie, le chrome ni
l'identité — seulement sur **ce qu'on montre, dans quel ordre, et regroupé
comment**. ⚠️ Et ce n'est pas une liste de tâches : *« c'est de la clarification,
n'implémente pas tout »*.

---

## Ce qui existait déjà, mesuré avant d'écrire

`Machine`, `MachineDocument` (S152), `MachineFavorite`, `MaintenanceTask`,
`RfidReader`, `AccessRfidLog`, `MachineAccessService`, `Material` +
`MACHINE_MATERIAL`, les kiosques, `/admin/machines` et `/machines/{id}`.

🔴 **Et `Machine::materials`, un tableau texte, EN PLUS de la relation** — deux
sources de vérité, dont l'une retombe sur une liste codée en dur
(`['PLA','PETG','TPU','Support']`) quand elle est vide. **Une découpeuse sans
matériaux annonçait donc du PLA.** C'est le défaut qui a donné S174.

---

## S171 — la sécurité des boîtiers, d'abord

Garde `fail-closed` quand `FABOS_RFID_API_TOKEN` manque ; retrait de l'exemple
`.env` qui donnait `FABOS_DB_*` à un boîtier ; décision explicite sur
`/kiosk/entries`.

**Mesuré** : un POST sans en-tête rend **503 `device_api_not_configured`**, pas une
autorisation.
✅ **Sans risque mesuré au moment de la bascule** : un seul lecteur existe, vu la
dernière fois le 2026-07-10.

⚠️ **Le secret était à DEUX endroits, et un seul avait été nettoyé** : le bloc
affiché du mode d'emploi, et **ce que copie le bouton Copier**. Un nettoyage qui
ne regarde que le rendu visible en rate la moitié.

🔴 **Une erreur de méthode à retenir** : la première vérification a testé
`/api/rfid/machines/1/access`, qui n'existe pas, et a pris son **404 pour une
preuve**. La route est `/authorization`. Un 404 sur une route inexistante ne prouve
rien sur la garde qu'on croit tester.

---

## S172 — identité et santé d'un boîtier

Secret propre au device, révélé UNE fois, rotation et révocation, dernière
connexion, **état réel** — prêt / hors ligne / non configuré / erreur / association
invalide — au lieu d'un booléen plus `lastSeenAt`.

**Mesuré** : chacun des cinq états est atteignable et distinguable à l'écran. La
colonne Statut ne dit **jamais « Actif »** sur un boîtier muet depuis plus d'une
heure ; le lecteur de la boîte, silencieux depuis le 2026-07-10, lit
« Hors ligne ».

⚠️ **Le seuil d'une heure est un CHOIX, pas une mesure** — aucun boîtier ne tourne,
personne ne connaît leur cadence. Il est écrit comme tel dans la ligne de revue
plutôt que présenté comme un fait.

---

## S173 — la fiche machine se sépare en deux publics

Membre (statut utilisable, prochaine action, prérequis exacts, matériaux
compatibles, réserver) et une zone **Exploitation** staff/admin.
⚠️ **La page RESTE une page** — pas deux routes, pas un shell neuf.

**Mesuré** : la carte **« Puis-je l'utiliser ? »** est la PREMIÈRE de l'onglet,
avant la description. Elle s'appelait « Badges requis » et **empruntait le libellé
du KIOSQUE, écrit pour un mur** — un libellé juste dans son contexte d'origine et
faux dans le nouveau. En anonyme, **zéro balise** de la zone Exploitation, vérifié
au rendu et pas seulement à la lecture du gabarit.

### 🔴 Le défaut d'i18n trouvé en passant, et il touchait quatre langues

En anglais, les libellés « Bookings: », « Enrolled: », « Completed: »
**disparaissaient** sur `/machines/{id}` et `/formations/{id}`.

La cause : **un `%count%` numérique fait lire le message comme une forme plurielle**,
et Symfony jette ce qui précède les deux-points quand c'est un seul mot — il le
prend pour l'étiquette d'un intervalle. Le français y échappait par sa typographie
(l'espace insécable avant le `:`), **donc les quatre AUTRES langues étaient seules
cassées** — et c'est exactement le motif qui rend le défaut invisible à qui teste
dans sa propre langue.

`tools/i18n/count_colon.py` l'interdit désormais.

---

## S174 — la matière devient une seule vérité

`MACHINE_MATERIAL` canonique, `Machine::materials` rétrogradé en note de
transition, et une fiche `/materiaux/{id}` avec les machines réellement
compatibles. **Plus de liste codée en dur.**

**Mesuré** : une machine sans matériaux saisis n'annonce **rien** — plus
« PLA, PETG, TPU, Support ».
⚠️ **Aucune machine d'ici n'avait le champ vide** : le défaut était une mine pour
une **installation neuve**, pas pour cette boîte. Prouvé en vidant puis en
remettant la machine 10 **au bit près** — la seule façon d'observer un défaut dont
aucune donnée locale ne déclenche la branche.

**Une seule** section matériaux sur la fiche : il y en avait **deux, côte à côte**,
l'une sur le texte libre et l'autre sur `MACHINE_MATERIAL`. Le doublon de modèle
était visible à l'écran depuis le début.

Sur `/materiaux`, un clic sur un matériau mène à **sa fiche**, avec les machines
qui l'acceptent, cliquables. Avant, chaque carte renvoyait à la liste d'où l'on
venait de cliquer — une affordance morte de la famille J-5.

### 🔴 Et une divulgation trouvée dans l'API publique

`GET /api/machines/5` **sans être connecté** publiait `machineToken` —
`"prusa-mk3s-01"`, le segment qui adresse la machine sur l'API des boîtiers. Il
vaut **null** désormais.
✅ **Pas un contournement** : S171 avait déjà rendu cette API `fail-closed`. Une
divulgation inutile, pas une porte ouverte — et il faut le dire ainsi plutôt que
de laisser croire qu'on a fermé une brèche.

---

## La passe de fond de cette phase

⚠️ Une phase qui ne fait que sa fonctionnalité laisse le socle où il était.

- **Réemploi** : les patterns locaux de la fiche machine (matériaux, maintenance)
  remontent dans le système de design s'ils servent ailleurs — sinon ils restent,
  et on l'écrit.
- **Conformité** : `tools/dead_affordances.py`, `tools/a11y_static.py`,
  `tools/form_placement.py` et `tools/ctor_arity.py` passés en début ET en fin de
  phase, l'écart commenté.
- ⚠️ **Le kiosque garde favicon, CSS et styles locaux** : soit il rejoint le shell
  et le thème publié (ce que la phase Thèmes demande aussi), soit on écrit
  pourquoi il reste à part. **Pas de troisième option silencieuse.**
