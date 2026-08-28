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

✅ **Paire C faite le 2026-08-27 : la règle 4 sur `/admin/emails`.** Les trois
horizons étaient calculés et jamais rendus ; ils s'affichent sous chaque délai, et
celui des prêts comme une DATE (il vaut `setTime(0,0)` en UTC — avec une heure il
aurait lu *02:00*).

✅ **Paires A et B faites le 2026-08-27 — les quatre paires sont closes.** Côté A :
deux listes voisines portaient le même libellé (« Créneau » deux fois, et le jour
sans nom), un `array_filter()` sans callback faisait disparaître une section nommée
« 0 » du résumé de portée, et la ligne de conséquence arrivait sans sa
fonctionnalité. Côté B, **deux alertes sur trois étaient des faux positifs** : le
compteur manquait sur **deux** replis, pas cinq — trois n'ont rien à compter — et le
`<h2>` dans un `<summary>` **reste bien un titre** dans l'arbre d'accessibilité.

✅ **Corrigé le 2026-08-27 : `/prets/{id}` n'avait pas de navigation.**
`loan-item.html.twig` étendait la coquille nue `base.html.twig` au lieu de
`site/base_public.html.twig` — la seule page publique dans ce cas, sur 29 balayées.
Vérifié en ligne, et re-balayage de 31 pages : 0 sans coquille.

✅ **La proposition d'écran « événements » est REGARDABLE : `/admin/design#evenements`.**
Six affiches de remplacement tirées de façon stable (`id % 6`), et le regroupement
par mois en spécimen. ✅ La question « dark/light » s'annule : en dessinant plutôt
qu'en téléversant, un seul fichier suit les deux thèmes (mesuré). 🅿️ Restent deux
décisions : téléverser SES logos (une table, donc une migration), et le choix entre
« une grille par mois » et « un séparateur dans la grille partagée ».

✅ **Les documents attachés à une machine sont EN LIGNE (2026-08-28).** Bloc admin
sur `/admin/machines/{id}/edit`, onglet « Documents » sur la fiche publique.
🔴 **Ces fichiers sont publics** — leur adresse suffit à les lire. 🅿️ Il reste à y
déposer les vrais documents, ce qui est votre part.

🅿️ **Consigné le 2026-08-27 : une proposition d'écran « événements »**
d'après trois captures de **Fabmanager** (⚠️ pas Fabman — seconde source, décrite
dans `Stage/Drive/Images/Fabmanager UI/README.md`). Dates fortes, regroupement par
mois qui montre le volume, et **plusieurs logos de remplacement tirés au sort** —
six livrés par défaut, clair et sombre. C'est une **proposition à regarder**, donc
elle passe par `/admin/design`. Découpage et points à trancher dans `ROADMAP.md`.

🅿️ **Aussi consigné le 2026-08-27 : les documents attachés à une machine**
(guide d'usage, fiche de sécurité…), à faire **avant la Phase H**. Découpage,
motif d'upload à réutiliser et pièges connus : `ROADMAP.md` § « Documents attachés
à une machine ». ⚠️ Une migration, donc l'opérateur devra la passer.

**Deux choses attendent votre arbitrage, pas du code** :
1. 🔴🔴 **J-25** — aucun membre ne peut réserver (voir plus haut).
2. 🅿️ **Le tableau du programme d'une formation** a perdu deux capacités que le
   `textarea` d'avant avait : coller huit étapes d'un coup, et réordonner en
   déplaçant des lignes. Le codeur argumente l'échange ; le choix est le vôtre.

✅ **R2 fait le 2026-08-27 : la largeur des formulaires admin.** 22 écrans
rendaient leur champ le plus large à 888 px ; `.admin-edit-form` porte maintenant
`--form-measure: 720px`, ce qui donne **351 px par colonne** — la mesure de Fabman.
19 des 20 écrans mesurés plafonnent, et les deux tableaux gardent leurs 888 px
parce que ce sont les MEUBLES du formulaire qui sont bornés, pas le formulaire.

✅ **Et la sonde admin exécute enfin le JavaScript** : en miroitant les 14 fichiers
de l'`importmap` dans le dossier servi localement (les chemins sont racine-relatifs
dans du JSON, et les assets n'ont pas de CORS). Prouvé sur `/admin/events/new` :
`repeatCount` se montre quand on met `repeatEvery` sur « chaque semaine ».

✅ **R3 commencé : `/admin/events/new` converti** (15 → **11 champs à l'arrivée**,
4 repliés, 3 titres de section, et une seule liste pour les deux écrans au lieu de
deux qui divergeaient). Le câblage Stimulus a dû descendre dans le `FormType` — et
la sonde à JavaScript a prouvé qu'il marche toujours. Au passage, l'aide de la case
« Ouvert aux personnes sans compte » réapparaît : le gabarit la dessinait à la main
et sautait `form_help()`.

✅ **R3 est FINI (2026-08-27)** : les six formulaires d'entité sont au motif
`SECTIONS`. 🔴 La conversion a sorti un défaut en ligne : trois champs de
`/admin/places` (Catégorie, Responsable, Département) tombaient **sous le bouton
Enregistrer** sur les deux écrans, mesurés à y=995/1078/1162 contre y=892 pour la
rangée d'actions — et ils avaient l'air normaux, parce que `form_rest()` passe par
le thème. ✅ Et deux écrans ne passaient pas du tout par le thème : l'écran
utilisateur avait **5 champs obligatoires et 0 mention « requis »**.
⚠️ **Pas** les quatre grilles de contrôles (`homepage` 35, `horaires` 31,
`maintenance/batch`, `features`) : ce sont des matrices, les replier les cacherait
sans les raccourcir.

✅ **R1 fait le 2026-08-27** : les libellés des formulaires passent de **186
littéraux à 9**, et les 9 sont des exemples de format (`01:30`, `#9E1B56`,
`https://…`), pas de la prose. Les pages admin rendues en anglais sont maintenant
entièrement anglaises. ⚠️ Dont `/creations/new`, qui n'est pas un écran admin : un
membre publiait son projet sur une page française quelle que soit sa langue.

✅ **Et le petit reste de R3 est fait** : « requis » ne se dit plus que d'un champ
qu'on peut laisser vide. Vérifié sur neuf pages rendues, champ par champ :
**0 désaccord** entre l'attribut HTML et la mention, contre 5 avant.

✅ **R4, R7 et R8 faits le 2026-08-27.** `/lab` montre ses sept destinations (326 →
876 px), `/machines/{id}` ne dit plus « connectez-vous » que deux fois au lieu de
quatre, et 🔴 **`/recherche` a enfin un champ de recherche** — elle n'en avait
aucun, le seul de la page était celui de l'en-tête à 89 px sur mobile.

✅ **R9 aussi** : sur `/profil/password`, l'écart entre une étiquette et son champ
passe de ~500 px à **0** — le motif « valeur + Modifier » ne va pas à un champ de
saisie, il fallait empiler.

**Il ne reste que R5 et R6, et ce sont des décisions, pas du code** : les deux mots
pour le même état de machine (« Hors service » dans la liste, « En maintenance » sur
la fiche — deux faits différents, un seul objet), et la pastille « Indisponible »
inatteignable tant que J-25 n'est pas tranché. 🅿️ Et les **documents attachés à une
machine**, qui demandent une migration.
Puis **R1** (42 `FormType` aux libellés français en dur) et R4 → R9.

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
