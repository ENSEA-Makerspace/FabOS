# Où on en est — 2026-08-27

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

✅ **La revue de fin de phase est CONCLUE — `S149-REVUE.md`.** Ses deux moitiés
sont faites : la statique a sorti 16 défauts, la **passe navigateur du 2026-08-27**
en a sorti **6 de plus, tous corrigés et déployés**. Deux cassaient l'usage :

1. 🔴 **Toute redirection vers `/login` partait en `http://`**, et le filtre
   FortiGuard de l'école bloque le http — un membre déconnecté n'atteignait
   **jamais** le formulaire de connexion. `trusted_proxies` manquait.
   ⚠️ À ne pas confondre avec le « 403 au proxy » : celui-là est la **liste
   blanche NPM** sur `fabos.dstei.fr` (LAN, tailnet, l'école, une IP fixe, puis
   `deny all`). Le site est privé par construction — hors de ces réseaux, tout
   répond 403, y compris le volet navigateur. Réglage voulu, pas une panne.
2. 🔴 **Le mur `/kiosk/events` cachait 902 px de son contenu** : `.ic` n'a pas de
   taille dans `kiosk.css`, l'épingle de lieu rendait **1665 × 1665 px**.

Plus : trois jetons CSS jamais définis (la carte d'authentification avait 0 px de
rembourrage), le bouton « Se connecter » qui disait « Envoyer le lien » dans les
cinq langues, `/forgot-password` qui contredisait son propre formulaire, et le mur
des passages qui annonçait le 10 juillet comme « ce matin ».

**Verdict : la Phase J tient sur ce qui se voit.** Ce qui reste (R1 → R9 dans la
revue) est du travail identifié et chiffré, rien n'y est cassé : 42 `FormType` avec
des libellés français en dur, 22 formulaires larges de 888 px, le repli absent de
55 des 59 pages admin.

✅ **Paire D faite le 2026-08-27 : le motif CRUD sur ses trois écrans.** Les
constantes `SECTIONS` de machine et de matériau étaient du code mort ; les deux
gabarits machine perdaient `manufacturer` et `model` après le bouton Enregistrer.
Un partiel unique déroule maintenant la constante pour les trois types. Trouvé en
corrigeant, et plus grave : **une liste de cases à cocher n'était ni nommée ni
décrite** — zéro `aria-describedby` sur la page. Détail dans `S149-REVUE.md`
§ « Paire D ».

**Prochain travail, dans cet ordre** :
1. 🔴 **J-25**, qui reste une décision opérateur : aucun membre ne peut réserver.
2. 🅿️ **Paire C** — la règle 4 de `/admin/emails` : trois horizons calculés,
   passés à la vue, et le gabarit n'en rend aucun. ⚠️ L'horizon des prêts est un
   `setTime(0,0)` : l'écrire comme une DATE, jamais une heure.
3. 🅿️ **Paires A et B**, puis **R1 → R9** de la passe navigateur.

## État du dépôt

- Branche **`s129/venues-workspace`**. ⚠️ **190 commits d'avance sur
  `origin/main`** — la fusion vers `main` n'a pas eu lieu et reste une décision de
  l'opérateur.
- Déployé et vérifié par hachage sur CT 210, services redémarrés, site 200.
- Cache-buster CSS courant : `?v=20260827-s151b`.
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
