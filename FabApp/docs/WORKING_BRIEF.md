# Où on en est — 2026-08-23

**Lire ceci en premier.** Une page. Le reste se lit à la demande :

| Besoin | Fichier |
|---|---|
| Ce qui reste à faire | [`ROADMAP.md`](/roadmap) |
| Comment le produit marche, et les pièges | [`PROJECT_STATE.md`](/roadmap) |
| Le détail des défauts de la Phase J | `S147-REVUE.md` |
| 🔴 La revue de sortie, **non conclue** | `S149-REVUE.md` |
| Ce qui est livré, phase par phase | [`HISTORY.md`](/roadmap/historique) |
| Modèle cible des droits | [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage) |
| Déployer | `ARTEMIS_DEPLOYMENT.md` |

---

## Position

**Phase J — « boutonner ».** Étape en cours : **S148, le socle**. 🔴 **J bloque la
Phase H (commerce)** — décision opérateur du 2026-08-21.

**Deux choses attendent l'opérateur, pas du code :**

1. 🔴🔴 **J-25 — aucun membre ne peut réserver une machine en ligne.** Les quatre
   chokepoints (`machines`, `places`, `person_booking`, `events`) répondent
   `DENIED / missing_package` à tout non-admin. 9 comptes, 2 packages actifs,
   2 attributions. Trois sorties : (a) attribuer un package aux membres, (b)
   ramener le chokepoint sur v1 avec le levier de `/admin/usage-rights/shadow`,
   (c) assumer qu'on ne réserve pas sans package. **Rien n'a été attribué.**
2. ⏭️ **J-9, J-10, J-23** — décisions d'arbitrage (les trois maquettes S103, les
   formulaires les plus lourds, le sort de l'écran d'ombre).

**J-22, J-4 et J-7 sont terminés** ; J-5 est à moitié fait et le reste est du CSS
réellement propre à sa page. Détail dans `S147-REVUE.md`.

🔴 **La revue de fin de phase est commencée mais NON CONCLUANTE** — `S149-REVUE.md`.
Sa moitié statique a sorti **16 défauts réels**, tous corrigés ; sa moitié visible
n'a pas pu être faite : CT 210 ne résout plus depuis le réseau de travail et le site
public répond **403 au proxy**. Les URLs, les parcours et les mesures à prendre sont
écrits — il n'y a plus qu'à les regarder.

**Prochain travail, dans cet ordre** :
1. 🔴 **La passe navigateur de `S149-REVUE.md`** — l'accès est revenu, elle est
   enfin possible, et c'est la moitié qui manque pour prononcer un verdict. Six
   écrans, six parcours à chronométrer, cinq mesures qui ne se lisent pas.
2. 🅿️ **Les restes des quatre paires**, listés dans `S149-REVUE.md` § « Les quatre
   relecteurs ». Le plus gros : le motif CRUD n'est appliqué qu'à **un** de ses
   trois écrans (les constantes `SECTIONS` de machine et matériau sont du code
   mort), et la règle 4 de `/admin/emails` est calculée sans être affichée.
3. ⏭️ **J-25**, qui reste une décision opérateur : aucun membre ne peut réserver.

## État du dépôt

- Branche **`s129/venues-workspace`**. ⚠️ **190 commits d'avance sur
  `origin/main`** — la fusion vers `main` n'a pas eu lieu et reste une décision de
  l'opérateur.
- Déployé et vérifié par hachage sur CT 210, services redémarrés, site 200.
- Cache-buster CSS courant : `?v=20260824-s150`.
- **Aucune migration en attente.**
- 🔴 **L'agent ne peut ni migrer, ni `git push`** : donner la ligne à l'opérateur.
- ✅ **Déployé et vérifié par hachage sur CT 210 le 2026-08-24** : 154 fichiers
  identiques, `lint:twig` 209/0, `lint:yaml` 39/0, 19 pages rendues, **13 sondes
  d'écriture vertes**, service redémarré, site 200.
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
