# Fabmanager — captures de référence

⚠️ **Fabmanager, pas Fabman.** Deux logiciels différents. Les 73 captures de
`../Fabman UI/` sont l'autre produit, et c'est d'elles que vient le barème
« qualité de formulaire » (`docs/history/phase-U-design-S45-S57.md` § « Les détails
à voler »). Ce dossier-ci est une seconde source, ouverte le 2026-08-27.

---

## 1. La liste des événements (instance Technistub) — 2026-08-27

🅿️ **Le fichier image reste à déposer ici.** La capture a été montrée dans une
conversation ; l'agent ne peut pas écrire dans ce dossier une image qui n'est pas
un fichier. Ce qui suit est la description écrite, faite en la regardant, pour que
l'analyse survive même sans le PNG. Nom suggéré :
`fabmanager-liste-evenements-technistub.png`.

**Ce que l'écran montre**

- Volet de navigation **rouge plein**, à gauche, avec icônes + libellés : Accueil ·
  Agenda · Réserver une machine · Inscriptions formations · **Inscriptions aux
  événements** (l'entrée active, sur fond plus clair) · Galerie de projets ·
  Abonnements. Un « ← Réduire le volet » en bas.
- Bandeau de titre : une flèche « ← » dans sa propre boîte, puis
  **LES ÉVÉNEMENTS** en capitales.
- Un `<select>` « Toutes les catégories » comme seul filtre.
- **Un événement à la une**, pleine largeur, au-dessus des mois : titre, sous-titre,
  date, horaires, deux métadonnées, et la moitié droite occupée par l'image.
- Puis des **en-têtes de mois** — « AOÛT, 2026 », « SEPTEMBRE, 2026 » — et sous
  chacun une grille de cartes, trois par rangée.

**L'anatomie d'une carte**

| zone | contenu |
|---|---|
| titre | « SESSION OPEN STUB (V) » en capitales grasses |
| pastille | la catégorie (« OpenStub »), grise, en haut à droite |
| date | **« Le 28/08/2026 »**, en rouge et en gras — l'élément le plus fort |
| horaires | « de 18:00 à 22:00 », plus petit, juste dessous |
| métadonnées | deux lignes à icône : 👤 « Sans réservation » · 🔖 « Entrée gratuite » |
| image | moitié droite de la carte : **le logo de l'organisation en remplacement** |

**Ce que l'opérateur en retient** (2026-08-27, ses mots) :

1. *« J'aime bien le fait que les dates sont visibles et que les mois aident à
   visuellement voir la quantité d'événements dans le mois. »* — le regroupement par
   mois donne le VOLUME d'un coup d'œil, ce qu'une liste à plat ne fait pas.
2. *« Le logo en placeholder par défaut me semble malin »* — remplir la moitié image
   d'une carte sans photo par le logo, plutôt que par un vide ou une icône générique.

⚠️ **Et ce qu'il ne faut pas copier sans réfléchir** : ici *toutes* les cartes
portent le même logo, parce que aucun de ces événements n'a d'affiche. Répété
quinze fois sur un écran, le remède devient le symptôme — c'est exactement ce que
la suite (les six variations) cherche à corriger.

---

## 2. La liste des formations (même instance) — 2026-08-27

🅿️ Fichier image à déposer également. Nom suggéré :
`fabmanager-liste-formations-technistub.png`.

**Ce que l'écran montre**

Même coquille que la liste d'événements — volet rouge, flèche « ← », titre
**LES FORMATIONS** en capitales — mais une grille **sans regroupement et sans
filtre** : une formation n'a pas de date, c'est un catalogue et pas un calendrier.

**L'anatomie d'une carte, et c'est une AUTRE carte que celle des événements**

| zone | contenu |
|---|---|
| image | **une vraie photo, en haut, pleine largeur**, ratio large (~16:9) |
| titre | en capitales, **centré**, sur sa propre bande blanche |
| pied | **deux verbes côte à côte**, séparés par un filet : 🔖 **Réserver** · 👁 **Consulter** |

Les images sont de trois familles : des photos de la machine réelle, des bannières
de marque avec titre incrusté (« IMPRESSION 3D — Libérez votre créativité »), et des
rendus 3D.

**Ce qu'il y a à en tirer**

- ✅ **La carte porte deux verbes, pas un lien global.** « Réserver » et
  « Consulter » sont deux intentions différentes et la carte ne choisit pas à la
  place du membre. À rapprocher de nos propres cartes machine, où l'opérateur
  n'avait « que le bouton Voir » ([[feedback-fabos-verify-pixels]]).
- ✅ **L'image est la moitié haute et non la moitié droite** — l'inverse de la carte
  d'événement de la même application. Deux objets, deux formes : c'est cohérent, pas
  incohérent, et ça vaut d'être noté avant de vouloir tout unifier.
- ⚠️ **Deux cartes portent la MÊME image** (Fusion 360 niveau 1 et niveau 2). C'est
  la version douce du défaut de la page événements : quand l'image est l'identité de
  la carte, deux cartes identiques deviennent indiscernables au coup d'œil.
- ⚠️ Le titre centré en capitales tient parce qu'il est court. « MODÉLISATION 3D
  FUSION 360 NIVEAU 1 - OPENSTUB » remplit déjà toute la ligne.

---

## 3. La liste des machines (même instance) — 2026-08-27

🅿️ Fichier image à déposer également. Nom suggéré :
`fabmanager-liste-machines-technistub.png`.

**Ce que l'écran montre**

**Exactement la carte des formations** — image en haut, titre centré en capitales,
pied à deux verbes « 🔖 Réserver · 👁 Consulter » — mais en **quatre colonnes** au
lieu de trois, et précédée d'une **bande grise de filtre** alignée à droite :
« Afficher les machines [ Actives ▾ ] ».

Les photos sont des **photos réelles, in situ**, non retouchées : la machine dans
l'atelier, avec le mur d'OSB, l'établi et le désordre autour. Aucune n'est un
gabarit de marque, aucune n'est un remplacement.

**Ce qu'il y a à en tirer**

- ✅ **La même carte sert deux objets** (formations et machines) et seule la densité
  change — 3 colonnes contre 4. C'est le genre d'économie qu'on cherche : une forme,
  deux listes, pas deux composants à faire diverger.
- ✅ **« Archivées » est un FILTRE, pas une page.** Un `<select>` « Afficher les
  machines : Actives » dans une bande dédiée, plutôt qu'un second écran. À
  rapprocher de notre audit par nom de route, qui avait déclaré les packages non
  archivables alors que le retrait est une case sur leur formulaire
  ([[feedback-fabos-verify-pixels]]) : ici aussi, le verbe est un champ.
- 🔴 **Aucune carte ne dit l'ÉTAT de la machine.** Ni « Libre », ni « Occupée », ni
  « Hors service », ni le prochain créneau. Un membre doit ouvrir la fiche pour
  savoir si l'atelier est utilisable maintenant. **Nos cartes le disent déjà** —
  pastille d'état mesurée à 5,02–10,72 de contraste sur les deux thèmes, plus
  « Prochain créneau 27/08 16:00 » et le compte « 8/11 disponibles » en tête de
  liste. C'est un point où FabOS est devant, et il ne faut pas le perdre en
  reprenant cette carte.
- ⚠️ **Le pied à deux verbes reste ce qu'il y a de meilleur ici.** « Réserver » et
  « Consulter » sont deux intentions ; notre carte machine mène tout entière à la
  fiche.

### Ce que les trois captures disent ensemble

| | événements | formations | machines |
|---|---|---|---|
| regroupement | **par mois** | aucun | aucun |
| filtre | catégorie | aucun | actives / archivées |
| image | moitié **droite** | moitié **haute** | moitié **haute** |
| colonnes | 3 | 3 | 4 |
| pied | — (la carte entière est le lien) | 2 verbes | 2 verbes |

Une seule carte, deux variantes d'image, et le regroupement qui n'apparaît que là
où l'objet a une date. C'est cohérent — et c'est la réponse à « faut-il une carte
unique ? » : **non, une carte par question**, mais pas plus.
