# `docs/` — quoi lire, et quand

**Quatre fichiers vivants. Tout le reste est de l'archive.**

| Fichier | Contenu | Quand l'ouvrir |
|---|---|---|
| `WORKING_BRIEF.md` | où on en est, cette semaine. Une page. | **toujours, en premier** |
| `ROADMAP.md` | ce qui reste à faire. Rien de livré. | avant de choisir quoi faire |
| `PROJECT_STATE.md` | comment le produit marche + les pièges | avant de toucher au code |
| `S147-REVUE.md` | le détail des 25 défauts de la Phase J | quand on traite un `J-xx` |

**Référence, stable :**

- `USAGE_RIGHTS_VISION.md` — modèle cible des droits, packages, réseau
- `ARTEMIS_DEPLOYMENT.md` — la recette de déploiement, à suivre sans improviser
- `FABMANAGER-ECARTS.md` — ce que les utilisateurs d'un produit VOISIN réclament,
  et ce que FabOS en a déjà. ⚠️ Des données, pas un plan : le plus haut score du
  tableau est 18 votes

**Archive :**

- `HISTORY.md` — l'INDEX de ce qui est livré : une ligne par session, un fichier
  par phase
- `history/*.md` — les récits détaillés. ⚠️ **Ne jamais lire en entier** : ouvrir
  seulement la phase qu'on touche.

## Règles de tenue

1. **Ce qui est livré sort de `ROADMAP.md` le jour même**, et entre dans
   `HISTORY.md`. Une session livrée laissée dans la roadmap finit par être
   refaite — c'est arrivé deux fois.
2. **`PROJECT_STATE.md` dit ce qui est vrai, pas comment on y est arrivé.** Le
   « comment » va dans `history/`.
3. **Un récit de session va dans le fichier de sa phase**, pas dans un des quatre
   fichiers vivants.
4. Cinq documents sont servis par l'app (`/roadmap`, `/roadmap/brief`,
   `/roadmap/historique`, `/roadmap/droits-usage`) via `MarkdownDocService`.
   ⚠️ **Les fichiers de `history/` ne sont PAS servis** : ne pas les lier en
   markdown depuis un document servi, ce serait un lien mort à l'écran.
