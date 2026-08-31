# Où on en est — 2026-08-31

**Lire ceci en premier.** Une page. Le reste se lit à la demande :

| Besoin | Fichier |
|---|---|
| Ce qui reste à faire | [`ROADMAP.md`](/roadmap) |
| Comment le produit marche, et les pièges | [`PROJECT_STATE.md`](/roadmap) |
| Le détail des défauts de la Phase J | `S147-REVUE.md` |
| ✅ La revue de sortie, **conclue** | `S149-REVUE.md` |
| Ce qui est livré, phase par phase | [`HISTORY.md`](/roadmap/historique) |
| Modèle cible des droits | [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage) |
| Déployer | `ARTEMIS_DEPLOYMENT.md` |

---

## Position

✅ **Phase S153 livrée le 2026-08-31.** Les quatre chantiers sont faits — la
saisie des packages (le compilateur, la case « sans limite d'horaires », la
sonde), le regroupement par mois de `/events`, les six affiches de remplacement
sur les vraies cartes, la bande du tableau de bord qui porte un fait. Les trois
propositions ont été **supprimées** du guide de style, avec la sous-page et sa
route. Détail → `history/phase-S153-saisie.md`.

🔴 **Une décision opérateur est SORTIE de cette phase, et elle est neuve :**
`USAGE_PACKAGE.fullAccess` veut désormais dire « aucune restriction, horaires
compris », et il est lu par les **deux** modèles de droits — il ne l'était que
par la v1. Sur la boîte, « Accès complet » porte déjà ce bit : ses porteurs
gagnent la réservation hors grille hebdomadaire (jamais hors d'une fermeture
datée). C'est le cas n°1 de la liste de l'opérateur, mais c'est un
**élargissement d'accès** — à confirmer package par package.

**Deux décisions opérateur toujours en attente :**
- 🔴🔴 **J-25** — aucun membre ne peut réserver (voir plus bas). ⚠️ À réexaminer
  à la lumière de ce qui précède : un package `fullAccess` sans grants
  n'autorisait rien sous la v2, ce qui est exactement cette forme de panne.
- ⚠️ **R5** de la passe navigateur — deux mots pour le même état de machine.

**Phase J : close sur ce qui se voit.** La revue de sortie `S149-REVUE.md` est
conclue, R1 → R9 sont faits sauf R5/R6 qui sont des décisions.

## État du dépôt

- Branche **`s129/venues-workspace`**. ⚠️ **190 commits d'avance sur
  `origin/main`** — la fusion vers `main` n'a pas eu lieu et reste une décision de
  l'opérateur.
- Déployé et vérifié par hachage sur CT 210, services redémarrés, site 200.
- Cache-buster CSS courant : `?v=20260831-s153b`.
- **Aucune migration en attente.**
- 🔴 **L'agent ne peut ni migrer, ni `git push`** : donner la ligne à l'opérateur.
- ✅ **Déployé et vérifié par hachage sur CT 210 le 2026-08-27**, deux fois :
  la passe navigateur (112 fichiers, balayage de 106 pages) puis la paire D
  (**115 fichiers identiques**, `lint:twig` 211/0, `lint:yaml` 39/0, `php -l`
  propre, **balayage de 171 pages sans échec**, service redémarré).
  La redirection de `/profil` est vérifiée en **https**.
  🔴 **SSH : DEUX chemins, et selon le réseau un seul des deux marche.** Mesuré le
  2026-08-24 sur deux réseaux, avec des résultats exactement inverses :

  | | `proxmox.lab.dryades.org` | `51.68.38.235` (IP) |
  |---|---|---|
  | réseau A | 🔴 ne résout pas | ✅ marche |
  | réseau B | ✅ marche (→ 192.168.100.1) | 🔴 expire |

  ⚠️ **Donc : essayer les DEUX avant de conclure que l'accès est perdu.** Croire le
  premier échec a coûté une demi-session — j'ai écrit une revue entière « sans
  accès » alors que l'autre chemin répondait. ⚠️ `artemis.dryades.org` échoue la
  vérification de clé d'hôte sur les deux réseaux ; ce n'est jamais le bon nom.
- ⏭️ **À pousser** : `git push` sur la branche. Vérifier avec `git status -sb`
  plutôt que de croire cette ligne.

## Les règles de travail

- **Une source de changement.** Si plusieurs pages ont besoin du même
  comportement ou du même dessin, on étend la source partagée — on ne copie pas.
- **Un seul design system.** `/admin/design` est la référence. Un motif récurrent
  y va, pas dans le `<style>` d'une page.
- **Le chemin honnête le plus court gagne.** Moins de clics, moins de choix.
  **Jamais un contrôle qui ne fait rien.**
- **Une session n'est pas finie en local** : doc, commit, déploiement des fichiers
  choisis sur CT 210, **et vérification du résultat qui tourne là-bas**.
- 🔴 **Le balisage rendu n'est pas une vérification visuelle.** Pour une
  affordance, une géométrie ou un contraste : mesurer dans un navigateur.
- **Revue de fin de PHASE**, une fois, mandat « designer d'Apple » : clics avant /
  après, évidence du chemin, frappes — **surtout en cas d'erreur** — et tout champ
  demandé sans être indispensable. Lui donner les **URLs et parcours**, pas le diff.

## Les sept pièges qui reviennent

1. 🔴 **`x|default(true)` est TOUJOURS vrai si on passe `false`** — Twig déclenche
   `default` sur toute valeur *vide*. **Deux fois livré ici.** Écrire
   `x is not defined or x`.
2. 🔴 **Une colonne ORM mappée exige sa migration AVANT le code.** Aucune
   dégradation possible : toute requête la sélectionne, la table entière fait 500.
3. 🔴 **Masquer n'est pas refuser.** Un objet archivé sort des surfaces qui le
   *proposent* **et** `ReservationService` le refuse au point de passage unique.
4. ⚠️ **`php -l` vert ne dit rien du conteneur** (un `use` manquant sur un
   paramètre de contrôleur = 500). **Un nouveau filtre Twig exige `cache:clear`
   AVANT `lint:twig`.**
5. ⚠️ **Un flash porte une CLÉ, pas une phrase.** Une clé inconnue traverse
   inchangée — ne pas « corriger » ça en levant une exception.
6. ⚠️ **`booking: false`** rend le composant calendrier lecture seule. C'est tout
   le mécanisme.
7. ⚠️ **`prod` n'a pas `strict_variables`** : une variable que le contrôleur ne
   passe pas est silencieusement `null`. Le lint ne le voit pas.

## Checklist de fin de session

1. Mettre à jour la doc concernée — **et sortir de `ROADMAP.md` ce qui est livré.**
2. Lint (`lint:twig`, `lint:yaml`, `php -l`) et **lire la sortie**.
3. Commit.
4. Déployer une archive **étroite** sur CT 210, `cache:clear`, restart.
5. Vérifier le résultat **qui tourne**, et écrire ce qui a réellement été vérifié.
