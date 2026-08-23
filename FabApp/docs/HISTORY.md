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

**Phase J — boutonner (en cours).** S147 la revue : 146 pages rendues et mesurées
+ passe navigateur → 25 défauts J-1…J-25. Corrigés à ce jour : J-1, J-11, J-12,
J-13, J-20, J-21, J-24, et J-22 pour 5 formulaires sur 6. Détail →
`S147-REVUE.md`.
