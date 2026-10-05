# Historique — index

**Ce fichier est un INDEX.** Une ligne par session livrée, un fichier par phase.
Les récits détaillés (le pourquoi, les pièges payés, les mesures) sont dans
`docs/history/`. **Ne pas lire l'historique entier** — ouvrir seulement la phase
qu'on touche.

- Ce qui reste à faire → [`ROADMAP.md`](/roadmap)
- Comment le produit marche aujourd'hui → [`PROJECT_STATE.md`](/roadmap)
- Où on en est cette semaine → [`WORKING_BRIEF.md`](/roadmap/brief)

⚠️ **Deux « Phase H » existent.** L'ancienne = durcissement (S38–S44, 2026-07).
La nouvelle = commerce (S150–S154, pas commencée). Le numéro de session tranche.

---

## Les fichiers, par phase

| Phase | Sessions | Quand | Fichier |
|---|---|---|---|
| Plan d'origine | — | 2026-07 | `history/2026-07-plan-origine.md` |
| A/B/C — modularité, portails, usabilité | S1–S57 | 2026-07 | `history/phase-A-B-C-S1-S57.md` |
| Durcissement (ancienne « Phase H ») | S38–S44 | 2026-07 | `history/phase-hardening-S38-S44.md` |
| U — design system | S45–S57 | 2026-07/08 | `history/phase-U-design-S45-S57.md` |
| Cohérence UI (S78/S79) | S78–S79 | 2026-08 | `history/ui-consistency-S78-S79.md` |
| Plans LMS + adhésion (jamais numérotés) | — | 2026-07 | `history/plans-LMS-et-membership.md` |
| A→F — fondations multi-lieux, groupes, workspaces, réseau | S102–S128 | 2026-08 | `history/phases-A-F-S102-S128.md` |
| G — droits d'usage, lieux, packages | S129–S144 | 2026-08 | `history/phase-G-S129-S144.md` |
| G3 — les listes et l'interface admin | S134h–S143 | 2026-08 | `history/phase-G3-interface-S134h-S143.md` |
| S146 — le calendrier unique | S146 a→g | 2026-08 | `history/phase-S146-calendrier.md` |
| Ancien `WORKING_BRIEF` complet (journal des positions + anciennes règles) | — | 2026-08 | `history/positions-log-2026-08.md` |
| État du projet, version 2026-08-09 | — | 2026-08 | `history/project-state-2026-08-09.md` |
| S153 — la saisie, et les propositions soldées | S153 | 2026-08 | `history/phase-S153-saisie.md` |
| S158–S159 — les groupes deviennent le modèle | S158–S159 | 2026-09 | `history/phase-S158-S159-groupes.md` |
| J — « boutonner » | S147–S169 | 2026-08/09 | `history/phase-J-boutonner-S147-S169.md` |
| O — machines & boîtiers | S171–S174 | 2026-09 | `history/phase-O-machines-boitiers-S171-S174.md` |
| P — espaces & accès d'entrée | S175–S178 | 2026-09 | `history/phase-P-espaces-acces-S175-S178.md` |
| K — gabarits d'e-mail modifiables | S160–S162 | 2026-09 | `history/phase-K-emails-S160-S162.md` |

---

## Le registre — une ligne par session

**Phase A — fondations (S102–S108).** S102 décisions + roadmap nettoyée · S103
registre Feature Workspace v2 + contrat Thèmes · S104 quotas réparés (compteurs
par type) · S105 gel des portails · S106 entité lieu + horaires migrés · S107
machines/espaces/objets/événements rattachés · S108 préférence de lieu,
`?location=`, composant partagé.

**Phase B — groupes et packages (S109–S111).** S109 sept groupes intégrés
protégés · S110 grants Use/Manage + scopes en simulation · S111 packages v2 en
ombre.

**Phase C — shell et workspaces (S112–S117).** S112 shell listes/filtres/facettes
· S113 Équipement · S114 Événements et Prêts · S115 Espaces · S116 Formations et
Badges (archivage, pas de suppression) · S117 Galerie, Pages, Utilisateurs,
Lieux, Packages, Réseau, Configuration, Thèmes.

**Phase D — réservations et reporting (S118–S120).** S118 politiques par feature
· S119 socle Reporting + `analytics.view/export` · S120 retrait de Réservations
globale.

**Phase E — identité et réseau (S121–S126).** S121 fédération OIDC · S122
`/m/{slug}` opt-in · S123 identité d'instance + API versionnée · S124 import QR
signé et consenti · S125 badges/formations fédérés · S126 marques/modèles
fédérés.

**Phase F — retrait et audit (S127–S128).** S127 portails retirés · S128 audit
transversal (30 tests / 208 assertions, 14 workspaces en 200).

**Phase G — droits d'usage (S129–S134b).** S129 workspace Lieux · S130 (+b/c/d)
navigation admin dédoublée, une seule sous-nav, filtre lieu en tuiles · S131
contexte lieu sur les écrans qui en stockent un · S132 (+b) quatre écrans de
Configuration · S133 (+b) parité + grants v2 en ombre · S134 activation graduelle,
quatre chokepoints basculés · S134b inventaire action-opérateur, 154 flashs
traduits, contrat des tables doublonnées.

**Interface (S134h–S143).** S134h/i/j les listes · S135 le même objet partout ·
S134c/c2 i18n + contenu inventé retiré · S134g compte supprimable, anonymisation
irréversible · S137/S138 vocabulaire d'objet, grilles de cartes · S139 (a→e)
recherche globale, 44 routes legacy supprimées, fil d'Ariane · S140/S141 la carte
fusionnée devient LE format de liste · S142 (+c/d) une seule barre latérale, une
seule forme de page · S143 « sous-lieu » → « lieu », dernier bandeau supprimé.

**Horaires et packages (S144–S145).** S144 (a/b/c) packages : à qui, sur quoi,
quand, combien · S145a `ScheduleResolver` — les horaires savent de quel lieu ils
parlent · S134d plusieurs plages par jour + exceptions datées + portée attachable
· S134e la raison d'une fermeture atteint calendriers et kiosques.

**S146 — le calendrier (a→g, 2026-08-20/21).** a UN composant calendrier · b la
fiche machine porte son calendrier · c `/calendrier` = activité, lecture seule ·
d `Event.formation` + génération de N séances · e s'inscrire à une séance inscrit
à la formation, sans qualifier · f catégories d'événement éditables · g plage de
dates sur les fermetures.

**S153 — la saisie, et les propositions qu'on solde (2026-08-28/31).** Un package
se décrit en quatre lignes plus une, et un compilateur écrit les cinq tables ; la
normalisation « aucune restriction ≡ `fullAccess` » a demandé que la v2 lise enfin
cette colonne. Le regroupement par mois de `/events`, les six affiches de
remplacement sur les vraies cartes, la bande du tableau de bord qui porte un fait.
Trois propositions supprimées du guide de style. Détail →
`history/phase-S153-saisie.md`.

**S158–S159 — les groupes deviennent le modèle (2026-09-01/03).** Parti d'un fait
mesuré : `USER_GROUP_MEMBER` n'était écrit par RIEN. Arrivé aux groupes comme
SEULE source des rôles (`UTILISATEUR_ROLE` supprimée) et des forfaits (plus une
attribution personnelle), avec l'appartenance datée. « package » devient
**forfait**. Quatre écrans obsolètes retirés. Détail →
`history/phase-S158-S159-groupes.md`.

**Phase J — « boutonner » ✅ CLOSE le 2026-09-05 (S147–S169).** S147 la revue :
146 pages rendues et mesurées + passe navigateur → 25 défauts J-1…J-25. Les 25
sont soldés — dont **trois qui se sont révélés caducs ou périmés dans leur
énoncé** parce qu'une autre phase avait supprimé leur sujet pendant que le plan
continuait de les décrire. Au passage : `/prets/{id}` rendu à sa coquille, les six
affiches de remplacement d'événement, les documents attachés à une machine. Détail
→ `history/phase-J-boutonner-S147-S169.md` et `S147-REVUE.md`.
⚠️ **La section « Ce que l'opérateur vérifie » de cette phase reste dans
`ROADMAP.md`** : c'est du travail qui l'attend, pas de l'historique.

**Phase K — les gabarits d'e-mail deviennent modifiables ✅ CLOSE le 2026-09-07
(S160–S162).** Un exploitant réécrit le texte d'un e-mail **par langue**, sans
écrire une ligne de Twig — substitution PHP sur une liste FERMÉE de champs, jamais
le compilateur Twig. `_header` et `_footer` se réécrivent **une fois pour les
vingt**. 🔴 La mesure de sortie : une surcharge volontairement cassée sur
`password_reset`, et le mot de passe oublié part quand même, **identique au bit
près**, incident journalisé. S160 a prouvé le repli dans son état le plus fort —
code déployé, **table absente**, 40 rendus identiques. Détail →
`history/phase-K-emails-S160-S162.md`. ✅ `Version20260907090000` passée par
l'opérateur le 2026-09-07, colonne mesurée écrite.

**Phase O — Machines & boîtiers ✅ CLOSE le 2026-09-05 (S171–S174).** Les trois P0
de sécurité (API des boîtiers `fail-closed`, `FABOS_DB_*` retiré des DEUX endroits
du mode d'emploi, `machineToken` retiré de l'API publique), les états réels d'un
boîtier au lieu d'un booléen, la fiche machine séparée en deux publics, et la
matière ramenée à **une seule vérité** — plus de repli codé en dur qui faisait
annoncer du PLA à une découpeuse. 🔴 Trouvé en passant : un `%count%` numérique
faisait **disparaître** « Bookings: » dans les quatre langues sauf le français.
Détail → `history/phase-O-machines-boitiers-S171-S174.md`.

**Phase P — Espaces & accès d'entrée ✅ CLOSE le 2026-09-06 (S175–S178).**
`AccessPoint` existe : un boîtier commande une **PORTE**, plus seulement une
machine — auparavant il fallait inventer une machine fictive. Migration additive,
et la mesure de S175 est un **non-changement** : deux lignes de diff sur la liste
des boîtiers. Les 72 refus d'accès portent tous une action cliquable, zéro lien
mort. ⚠️ Rien n'est révoqué à l'annulation **parce que rien n'est accordé** : la
question est reposée à chaque badge. Détail →
`history/phase-P-espaces-acces-S175-S178.md`.
🅿️ Une ligne de revue reste ouverte dans `ROADMAP.md` : la pastille « Libre à
14:00 » sur `/places`, non mesurable la nuit.

## Décision de design — le traitement des photos du catalogue (2026-09-30)

Les photos de démo (Wikimedia Commons) venaient de sources différentes : fonds,
lumières et cadrages juraient côte à côte. Trois variantes comparées dans
`/admin/design`, sur les vraies cartes, une seule variable (le filtre) :
**A** duotone de marque (plus forte identité, mais la couleur réelle
disparaît — et un premier essai était criard), **B** gris léger avec la vraie
couleur au survol et au clavier, **C** désaturation partielle chaude.
L'opérateur a choisi **B**, « clairement ». A et C sont retirés du CSS et de la
page de design. B est devenu un réglage du thème (Thèmes → Mise en page →
Traitement des photos : aucun / gris léger), posé en `data-photo-treatment` sur
`<html>` ; cartes du catalogue seulement, la fiche d'une machine garde ses
couleurs (celle d'un filament ou d'un fil est une information).

## La fiche machine repensée (2026-09-30)

Demande de l'opérateur (« repense toute la page, on a des exemples »), d'après la
planche `equipment-machine-member-detail.png` — « tâche d'abord ». En tête : le
titre, une carte **Réservation** (verdict en une phrase, les boutons existants,
lieu et prochain créneau) et une carte **Votre accès** (badges cochés ou non),
la **photo** en paysage à droite — ou l'icône de catégorie par défaut, par le
composant commun `_category_icon` (les cinq pictogrammes recopiés dans la page
sont partis). Dessous : matériaux compatibles, « Avant d'utiliser la machine »
(exigence, documents, quiz), description, et « État et maintenance » replié.
Onglets, calendrier, documents : inchangés. Cartes = `.detail-card` ; seules la
mise en page `.md-*` et la pastille de verdict sont neuves ; 12 règles mortes
supprimées. 🔴 Trouvé en route : un membre à qui manquait la formation
(`training_required`, `physical_training_required`) voyait « Machine
indisponible » — la page ne connaissait que `missing_badge`. Et la boîte photo
était bornée à 200 px par une règle des vignettes de l'accueil. Popularité en
étoiles abandonnée (absente de la planche).

## Phase V — les pages revues d'après les planches (2026-10-01 → 0.5.0 le 2026-10-02)

Demande de l'opérateur (AFK) : revoir chaque planche face à notre page, en garder
les bonnes idées, et poser des **propositions en sous-pages du menu Développement**
(« pas des branches »). Socle : `PageProposals` (registre),
`DesignProposalController` (`/admin/propositions`, une branche de données par
proposition, `?membre=<id>` pour regarder avec un compte de test), un bandeau
commun (`proposals/_banner` : planche, page actuelle, « pris / gardé », liens de
démo) et `proposals.css` (`pp-*`). Le tri des 52 planches vit dans
`docs/references/PROPOSITIONS.md`, rendu sur l'index. Chef de projet + sous-agents
Sonnet sur un brief commun (fichiers neufs seulement ; intégration, rendu et
déploiement faits ici). Trois calculs sortis des contrôleurs pour être lus par la
page ET sa proposition : `PlaceCatalogue`, `MyReservations`. 🔴 Corrigé en vrai au
passage : un espace, un prêt ou un matériau sans photo affichait l'imprimante 3D
(`fallback_icon` sur `_catalogue_card`).

**0.5.0 (2026-10-02).** L'opérateur valide (« tout a l'air très bien »), sauf
l'accueil : il garde le hero avec horaires et événements, que le nouveau format
complète. Mise en place par lots parallèles de sous-agents, un contrôleur chacun,
clés i18n écrites à part puis appliquées (≈ 490 clés, 5 langues). Puis l'échafaudage
des propositions est retiré (contrôleur, registre, gabarits, menu) ; les services
deviennent `App\Page\*`, les feuilles `pages.css` + `page-<page>.css`. Versions
nommées à partir d'ici (`docs/VERSIONS.md`, étiquettes `v0.4.0`, `v0.5.0`).

## Proposition — « Créer une machine » revu (2026-10-06)

Demande de l'opérateur : l'écran est lourd et ne précharge rien de ce qui existe.
Revue d'un designer sur l'écran actuel (40 machines en base) : il ignore ce que le
lab possède (14 X1 Carbon saisies une à une, d'où « BambuLab » / « Bambu Lab ») ;
catégorie, localisation et modèle sont des cases vides ; le statut s'ouvre sur
« idle » quand 34 sur 40 sont « disponible » ; limite et popularité, sans effet à la
création, tiennent le chemin principal ; les badges sont tout en bas en 11 cartes.
Proposition, en sous-page du menu Développement (`/admin/propositions/machine`,
maquette qui n'enregistre rien) : repartir d'un modèle possédé en un clic ; quatre
questions (nom, catégorie, badges, emplacement) en tuiles cliquables `.ml-tile`, la
catégorie proposant ses badges habituels ; le reste replié ; identifiant de boîtier
généré ; statut, limite et popularité sortis de la création. Écartés : un assistant
en étapes, des listes fermées, un catalogue de modèles à part. Données :
`App\Page\MachineCreationHints`. 🅿️ À la décision : retirer la page, sa route et
son entrée de menu.

## 0.6.3 — la page publique « Maintenance » retirée (2026-10-06)

Demande de l'opérateur : la page du menu principal ne sert plus. Elle listait les
tâches ouvertes de tout le lab ; la fiche machine montre l'entretien de la machine
qu'on regarde, et l'équipe travaille dans `/admin/maintenance`. Retirés : route
`app_maintenance`, contrôleur, gabarit, les deux entrées de menu, `landingRoute`,
les clés `nav.maintenance`, `maintenance.subtitle`, `maintenance.empty`. Le module
`maintenance` et tout le reste sont intacts.

## 0.6.1 et 0.6.2 — le design documenté, puis factorisé (2026-10-06)

Règle de l'opérateur : du design neuf est permis **s'il est documenté dans
`/admin/design`**. 0.6.1 : la page documente les motifs nés en 0.5/0.6 (tête de
fiche, motifs `pp-`, bloc membre, liste de motifs, page de compte, fonction
activable) et gagne `.form-narrow` (formulaire public étroit, R8 de la revue).
0.6.2 : les quatre motifs recopiés à la main dans six pages deviennent des partiels
— `_next_card`, `_dated_rows` (variante `search`), `_steps`, `_hcard` — et la page
Design les inclut. Preuve : le `<main>` de sept pages rendu avant et après, comparé
sans les espaces, est identique. `_reason_list` et `_honeypot` prennent un
`id_prefix`. Le choix du mot d'une étape de formation, écrit deux fois, passe dans
`_journey_steps` (même preuve, huit pages). 🅿️ Reste : la carte « à reprendre »
de Mes formations (ni image ni colonne latérale : `_next_card` changerait son allure).

## 0.6.0 — Phase W : six fonctions activables (2026-10-05)

Après la comparaison avec FabtrackJS (Sorbonne), l'opérateur retient six idées,
construites en un lot par cinq sous-agents en parallèle sur un brief commun
(tables neuves en DBAL avec `isReady()` fail-safe, contrôleurs neufs, i18n à part,
une sonde chacun). `SiteFeature` gagne `defaultOn` : stocks, check-in (4 paliers)
et charte naissent ÉTEINTS. S205 panne par QR (page publique, photo, anti-abus,
QR imprimable) ; S206 stocks (éteint = le mot n'apparaît nulle part, prouvé par la
sonde) ; S207 check-in à paliers (la documentation de projet est le dernier palier,
jamais obligatoire — position de l'opérateur) ; S208 avertissements (registre, sans
effet automatique) ; S209 charte (= le règlement du lab, accord redemandé si le
texte change) ; S210 bouton « Signaler ». ⚠️ Le passage de badge n'ouvre pas encore
un check-in : aucune API de porte n'enregistre les passages.

**Revue de sortie par un designer pointilleux** (`docs/PHASE-W-REVUE.md`, 43
constats : 3 bloquants, 18 à corriger, 22 finitions) — 42 appliqués. Décisions :
la pastille de l'en-tête devient « Un retour ? » (deux « Signaler » se
confondaient) et renvoie vers le signalement de panne ; un lien « Signaler une
panne » sur chaque fiche machine ; « stockage » devient « rangement » (stocks
éteints = plus un mot de la famille, sonde élargie) ; la charte s'appelle « Règlement
du lab » partout ; la question du projet quitte la borne. Un seul partiel de
« liste de motifs » pour le check-in et les avertissements ; `feedback.css` divisé
par deux. `app:render --features` : voir une fonction éteinte dans une transaction
annulée.

## 0.5.1 — « Mon compte » (2026-10-02)

`/profil` refait d'après la proposition validée : en-tête compact, onglets serveur
(`?onglet=apercu|acces|activite|reglages`, anciens liens `#ancre` redirigés), « Je
peux » en liste (demande de l'opérateur), réglages en lignes qui ouvrent leur
formulaire sur place. 🔴 Inventaire contrôlé : tous les `name=`, jetons CSRF et
liens de l'ancienne page sont présents. CSS : `.detail-card`, `.md-access-list`,
`ml-tile`, `pages.css` réutilisés ; `page-profil.css` ne garde que l'introuvable ;
CSS mort de l'ancien profil purgé. Piège rencontré : `false|default(true)` vaut
true (le « Bonjour » de `_home_member` restait).

## Les fiches espace, matériau et objet prêté, même tête (2026-10-01)

Suite de la précédente, à la demande de l'opérateur. La tête « tâche d'abord »
sort de la fiche machine dans **un** gabarit partagé, `_detail_hero.html.twig`
(titre, crayon admin, cartes, photo ou image par défaut), que les quatre fiches
emploient. **Espace** (planche `02-detail-espace-reservation.png`) : « Vous
pouvez réserver cet espace » / connexion / formation, bouton vers le calendrier,
lieu · capacité ; carte « Comment on y entre » = badges S204 cochés ou non + portes
S177. **Matériau** : « Où on l'utilise » et « Où le trouver » en tête, cotes
dessous. **Objet prêté** : disponible / emprunté, compteur, son prêt, note du
comptoir ; il quitte `machines-list.css` (22 règles `.loan-item*` supprimées) pour
`details.css`. Image par défaut quand rien n'est saisi : plan (espace), émoji puis
boîte (matériau), boîte (objet). ⚠️ Un espace n'a **pas** de champ photo — à
ajouter le jour où l'on veut une vraie bannière. Clé morte `loans.manage_item`
retirée (le crayon admin la remplace). Sondes S193 (crochet `data-my-loan`) et
S204 vertes.
