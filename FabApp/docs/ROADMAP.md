# FabOS — ce qui reste

**MAJ 2026-08-23.** ⚠️ **Cette page ne contient QUE le travail restant.** Ce qui
est livré en sort le jour même : sinon une session le refait — c'est arrivé deux
fois.

- Comment le produit marche → [`PROJECT_STATE.md`](/roadmap)
- Ce qui est livré → [`HISTORY.md`](/roadmap/historique)
- Où on en est cette semaine → [`WORKING_BRIEF.md`](/roadmap/brief)
- Modèle cible des droits → [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage)

**Livré à ce jour** : phases A→F (S102–S128), **toute la Phase G**, la Phase G2,
l'interface S134h–S143, S144, S145a, toute la phase S146, et la revue S147.

**Ordre : J → H → I.** 🔴 **J bloque H.**

---

## Cap produit

Tout fablab, école ou atelier partagé déploie **les seules fonctions dont il a
besoin**, avec une expérience cohérente.

- une installation, plusieurs **lieux** ; aucun portail ;
- SSO entre instances sans partager droits ni données ;
- sept audiences intégrées protégées + groupes locaux + packages assignables ;
- deux droits, **Use** et **Manage**, par feature / lieu / scope ;
- réservations, quotas et reporting montrés dans chaque feature, moteurs communs ;
- profils publics volontaires, échanges inter-FabOS consentis, badges fédérables ;
- plus tard : Paiements facultatif, puis messagerie Formation ;
- **Configuration → Thèmes** réunit identité visuelle, images, menus et accueil ;
- **un seul** système central de listes, filtres, workspaces, composants et CSS.

---

# Phase J — « boutonner » ⬅️ EN COURS

**Demande opérateur, 2026-08-21** : *« before commerce i want to smooth out a lot
of things… act like apple engineers and button everything up. »*

🔴 **BLOQUANTE avant la Phase H.** G était la barrière du MODÈLE ; J est celle de
la FINITION. Vendre une surface non finie fige ses défauts dans un contrat client.

## Les dix points — un écran est fini quand les dix réponses sont oui

1. **Coquille partagée.** Pas de `<head>` maison, pas de `<style>` local qui ne
   soit pas devenu une règle du guide.
2. **Le titre vient de `NavBuilder`**, jamais recopié.
3. **Listes** : cinq colonnes max, actions comprises ; autant de cellules que
   d'en-têtes ; pas de `colspan` compté à la main.
4. **Chaque objet annoncé est créable, éditable, archivable** depuis son workspace.
5. **Aucune affordance morte** : pas de bouton qui n'aboutit pas, pas de lien qui
   404, pas de filtre qui ne filtre rien.
6. **Cinq langues, sombre, mobile, clavier.** Vérifié à l'écran, pas supposé.
7. **Le nombre de clics est COMPTÉ**, avant et après.
8. 🔴 **Un champ invalide ne fait JAMAIS ressaisir le reste du formulaire.** Prouvé
   par un vrai POST refusé, pas relu.
9. **Zéro champ non indispensable.**
10. **Les primitives sont dans `/admin/design`** avec le vrai composant.

## Découpage

| Étape | Livre | Qui |
|---|---|---|
| ✅ **S147 — LA REVUE** | passée 2026-08-22, aucun code hors la sonde. 146 pages rendues + passe navigateur (375/768/1280, cascade, clavier, sombre, un vrai POST refusé) → **25 défauts J-1…J-25**. Détail : `S147-REVUE.md` | Terra mesure, opérateur arbitre |
| **S148 — le socle** | Réglages, Fonctionnalités, E-mails, Logs RFID, Thèmes, Setup/assistant, Tableau de bord. ⚠️ Absorbe ce qui restait de S132 | Luna + Terra |
| **S149 — feature par feature** | machines, espaces, événements, formations, prêts, matériaux, badges, projets, réservations, packages/quotas. Chacune finie selon les dix points | Luna + Terra |
| **S149z — la sortie** | revue conjointe finale : la liste S147 est vide ou consciemment reportée | Opérateur + Terra |

⚠️ **La revue vient EN PREMIER et elle ne code pas.** Un chiffre inventé a cadré
une session entière (S134j). On mesure, on montre, on décide, puis on fait.
⚠️ **La revue de fin est UNE FOIS PAR PHASE**, pas par étape (opérateur,
2026-08-20). Mandat « designer d'Apple ». Lui donner les **URLs et les parcours**,
pas le diff.

## Les 25 défauts — état au 2026-08-23

| # | Défaut | État | Étape |
|---|---|---|---|
| **J-25** | 🔴🔴 **aucun membre ne peut réserver en ligne** — les 4 chokepoints répondent `DENIED / missing_package` à tout non-admin | ⏭️ **décision opérateur, tout de suite** | — |
| **J-22** | formulaires admin hors thème — **23 conversions réelles** (25 gabarits, dont 5 filtres GET qui restent dans l'URL) | 🔶 `/admin/settings` (5), `/admin/emails` (3), `/admin/wizard` (1), `/admin/usage-rights/{id}/edit` (5/6) faits. **Reste `admin-formation-content` (3) puis sept écrans à un formulaire** | S148 → S149 |
| **J-8** | un champ refusé fait ressaisir le reste | 🔶 **le chiffre de 15 était FAUX** : 1 prouvé fautif (`/profil`), 1 prouvé sain (`/admin/settings`), **13 non vérifiés**. Sonde : `app:s147:form-probe` | S149, écran par écran |
| **J-9** | trois maquettes S103 en prod, titres en dur, clés brutes à l'écran | 🔶 partiel | S148 — ⏭️ décision opérateur |
| **J-10** | formulaires les plus lourds (168 champs sur l'éditeur de package) | ⏭️ à arbitrer | S148 — ⏭️ décision opérateur |
| **J-23** | `/admin/usage-rights/shadow` : bascule finie, audit encore utile | ⚠️ **amendé par J-25 — le levier RESTE** | S148 |
| **J-4** | « (s) » au lieu de pluriels ICU | 🔶 76 → **72** ; le mécanisme ICU existe, reste du travail répétitif. ⚠️ chaque clé déplacée demande de corriger son appelant | S149, en dernier |
| **J-5** | CSS local par page rendue | 🔶 897 dans 43 → **589 dans 39** | S149, avec l'écran |
| **J-7** | emoji bruts comme icônes | 🔶 74 dans 22 → **70 dans 18**, dont 29 sur `admin-design` (le guide, légitime) | S149 |
| **J-1** | déploiement partiel : l'upload d'images fatalait en prod | ✅ 2026-08-22 | — |
| **J-2** | huit objets se supprimaient en dur | ✅ 2026-08-22. ⚠️ **Reste à vérifier la promesse S134f** : archiver une ressource réservable doit annuler ses réservations à venir | — |
| **J-3** | flashs en dur | ✅ 37 → **0** | — |
| **J-6** | `style=""` sur `/admin/utilisateurs/{id}` | ✅ 78 → **1** | — |
| **J-11** | `/machines/{id}` cassé sur téléphone | ✅ 2026-08-22 | — |
| **J-12** | barre d'outils du calendrier | ✅ 2026-08-22 — ⚠️ le constat initial était faux, 5 contrôles réellement inatteignables | — |
| **J-13** | « Réserver une machine » menait au calendrier lecture seule | ✅ 2026-08-22 | — |
| **J-14** | pas de lien d'évitement, focus invisible | ✅ | — |
| **J-15** | fonds clairs sans variante sombre | ✅ 101 → **0** | — |
| **J-16** | `/formations/{id}/suivi` imprimait ID/titre/slug au public | ✅ | — |
| **J-17** | `/machines/{id}` « Connexion requise » ×4, favoris mort | ✅ | — |
| **J-18** | `/admin/maintenance/batch` sans lien | ✅ | — |
| **J-19** | « Loans » ouvrait le catalogue d'objets | ✅ | — |
| **J-20** | le calendrier ignorait les plages horaires | ✅ 2026-08-22 | — |
| **J-21** | catégorie d'un grant comparée par libellé exact | ✅ 2026-08-23 — l'identifiant décide | — |
| **J-24** | messages de validation en français en dur | ✅ 69 → **0**, cinq langues complètes | — |

⚠️ **Rien de cette liste n'est reporté hors de la Phase J.**

## Critères de sortie

- la liste S147 est vide, ou chaque reste est **consciemment reporté et écrit** ;
- **aucun gabarit ne porte de `<style>` local** hors `admin-design`, ou chaque
  exception est une règle nommée du guide ;
- ✅ **gabarits à `<head>` propre : ATTEINT** — ils sont **5** (`event-ticket` +
  4 kiosques), et c'est l'exception que ce critère prévoyait ;
- les dix points passent sur **chaque** écran du socle et des features ;
- `/admin/design` montre chaque primitive utilisée, avec le vrai composant.

## 🅿️ Parqué — n'entre PAS dans J

Sélecteur de langue (`app_switch_locale` n'est lié nulle part) · suppression en
masse d'événements · catégories comme entrées de menu. Ce sont des
fonctionnalités, pas de la finition. ⚠️ Le tableau de bord « qui doit re-briller »
est le seul des quatre qui touche J : il est dans **S148**.

---

# Phase H — commerce facultatif (S150–S154)

🔴 **BLOQUÉE PAR LA PHASE J** (opérateur, 2026-08-21).

Entièrement désactivable. Offres dans leur workspace métier, moteur commun pour
commandes / paiements / rapprochement.

- 🔴 **Le retour navigateur ne confirme JAMAIS un paiement** — seul un webhook
  vérifié ou sa réconciliation.
- Clé unique par événement fournisseur ; fulfillment persistant / outbox par ligne
  → effet **exactement une fois** malgré retries et crashs.
- La livraison passe par le service métier normal : sans toucher voter, badge,
  quota ni réservation.
- Ni carte ni credentials fournisseur en base FabOS.

| Session | Livre |
|---|---|
| **S150** | catalogue d'offres et prix ; aucune transaction |
| **S151** | commandes, paiements, webhooks, réconciliation, remboursements, audit |
| **S152** | livraison packages et matériaux ; hold stock atomique ou backorder explicite |
| **S153** | ledger append-only des crédits de temps et achats de formation |
| **S154** | reporting commerce, rapprochement, audit UX |

---

# Phase I — messagerie Formation (S155–S157)

Très loin après le workspace Formation. FabOS est la source de vérité ; l'e-mail
est une copie et une panne d'envoi ne perd jamais le message interne. Trois
visibilités : annonce formateur→cohorte sans exposer la liste, fil privé, groupe
explicite. **Aucun message privé ne bascule implicitement vers la cohorte.**

| Session | Livre |
|---|---|
| **S155** | conversations, participants, non-lus, permissions |
| **S156** | interface formateur/étudiant + duplication e-mail asynchrone |
| **S157** | modération, archivage, export, rétention |

---

# Restes ouverts, hors phase

## Packages — ce qu'ils ne savent toujours pas dire

**Liste de choix, pas un plan. Rien n'est construit.**

- 🟡 **1. Assouplir un quota de palier — le plus vendable.** « 30 jours d'avance au
  lieu de 7 », « 8 h au lieu de 4 ». `BookingPolicy` porte déjà `maxHorizonDays`,
  `maxDurationMinutes`, `maxActiveReservations` — mais **par palier**, pas par
  package. Forme : un package portant des **surcharges** appliquées **vers le haut
  seulement**.
- 🟡 **2. Validité relative à l'attribution.** « Trois mois à partir de
  l'activation » n'existe pas : `validFrom`/`validUntil` sont deux dates absolues.
- 🟡 **3. Report des heures non consommées.** Une allocation hebdomadaire non
  utilisée est perdue le lundi.
- 🟡 **4. Allocation par catégorie de machines.** Les grants savent le dire, les
  allocations non, faute d'un comptage qui l'honore.
- ⚪ **5. Priorité / préemption** — pas demandé, **contraire à « aucun package ne
  retire un droit »**. Ne pas construire sans décision explicite.
- ⚪ **6/7. Prix, panier, paiement, matériaux inclus** → Phase H.
- ⚪ **8. Formations incluses** — la certification est **délibérément hors** du
  modèle de packages : sécurité, pas commerce. Vendre « la formation laser » est
  une commande Phase H qui **inscrit à une session**, elle ne fabrique pas un badge.

## S144e — « ce package touche N personnes »

`readiness()` et la liste comptent les attributions **directes** ; un package tenu
seulement par un groupe annonce 0. Il faut un
`AudienceResolver::memberIdsFor($groupKey)` **inverse exact** de `keysFor()` :
rôles + lignes `USER_GROUP_MEMBER` + l'audience virtuelle `user`.
⚠️ **Ne pas réécrire cette logique à côté.**

## Thèmes — le chantier entier (session dédiée, non planifiée)

- **Médiathèque d'identité** au lieu du champ texte `logoPath` : logo
  clair/sombre/compact, favicon, image de partage. Validés, renommés serveur,
  référencés par ID stable, supprimables seulement après contrôle des références.
  **Aucun chemin `public/images/…` libre.**
- **Éditeur guidé** : identité, variantes de logo, palette avec contrastes,
  rayon/typo/densité en presets.
- **Workflow** brouillon → aperçu → publication → retour arrière. L'aperçu rend de
  **vraies** surfaces (accueil, catalogue, détail, admin, un kiosk),
  desktop/mobile, clair/sombre. Publication atomique réglages **et** assets.
- **Kiosks** consomment le thème publié. Aucun favicon, logo ou couleur statique
  ne survit dans un kiosk.
- **Navigation & accueil** : ordre et visibilité par drag-and-drop accessible,
  destinations limitées aux routes autorisées, entrées système protégées. Une page
  dépubliée rétablit l'accueil FabOS avec audit, sans page blanche ni boucle.
  ⚠️ `_logo.html.twig` retombe encore sur `Logo_ENSEA.png` et `site_logo_path`
  n'est éditable nulle part — dé-marquer avant la médiathèque laisserait le site
  sans logo et sans moyen d'en remettre un.

## 🟡 Supprimer en masse ce qu'on a créé en masse

**Opérateur, 2026-08-21** : *« if we can create X events, we have to have a way to
mass delete them »*. S146d crée jusqu'à 12 événements d'un envoi ;
`/admin/events` ne les retire qu'un par un.

🔴 **La tension à trancher d'abord** : S146d a fait des lignes **indépendantes**
(pas d'identifiant de série) exprès. Deux voies :
- **sélection multiple** sur `/admin/events` — ne suppose aucune série, réutilisable
  ailleurs, plus de travail d'interface ;
- **identifiant de série** nullable et informatif — moins de clics, mais réintroduit
  la notion de série et la question « que devient une séance déplacée ? ».

⚠️ **Une séance à laquelle des gens sont inscrits ne se supprime pas en silence.**
Supprimer et annuler (`callOff`) ne sont pas la même action.

## 🟡 Une catégorie peut devenir une entrée de menu

**Opérateur, 2026-08-20.** Une entrée de menu **EST un filtre enregistré** :
`/events?category=<slug>` existe déjà. Donc c'est un **réglage de navigation**, pas
une page.

À vérifier avant : ⚠️ **où vit le réglage** (le menu public n'est pas
`NavBuilder::admin()` ; Thèmes est censé être l'endroit des menus) · 🔴 **un menu ne
doit nommer ni une catégorie archivée ni une catégorie vide** · ⚠️ **le slug est la
clé, jamais le libellé** · ⚠️ le libellé est un **contenu**, donc non traduit à côté
d'entrées traduites · ⚠️ plafond d'entrées (le menu principal en a déjà cinq).

## Petits restes datés

- **Le tableau de bord a perdu son caractère** (opérateur, 2026-08-16) : *« the old
  homepage looked more "special", we'll find a way to make it pop again later »*.
  ⚠️ **Pas une régression à annuler** : le rendre distinctif **sans** réintroduire
  un bandeau pleine largeur ni une couleur en dur. Pistes : une bande d'accueil qui
  reste dans la carte mais respire ; les sept chiffres traités comme la figure de la
  page ; une seule surface accentuée réservée à cet écran. **À montrer en
  propositions comparables dans `/admin/design` avant de construire** — c'est le
  protocole qui a marché pour le format de liste (quatre tours).
- **`/events` sans paramètre rend 0 carte** quand il n'y a rien à venir, alors que
  « Tous » en compte 3. Le défaut « À venir » est délibéré ; il manque un état vide
  qui renvoie vers les événements passés.
- **`/admin/homepage` porte six colonnes** (bloc + quatre audiences + ordre). C'est
  une matrice d'audiences, pas une liste. Le plafond de cinq ne lui répond peut-être
  pas. Non tranché.
- **Logs RFID** : les cellules `status` / `reason` / `color` impriment encore les
  mots stockés. À résoudre avec le patron `Machine::getStatusKey()`, pas d'une
  seconde façon.
- **Quiz et validations physiques n'ont aucune UI de création.**
- Les deux écrans RFID montrent les mêmes deux boutons deux fois.

## Travaux transversaux conservés

Sécurité restante de Phase H (**test réel du booking**, requêtes groupées) ·
verrou d'annulation et no-show sur ressources qui ont un signal · files d'attente,
stockage/retrait, motif d'utilisation · audit et notes sur toute action Manage
exercée sur autrui.

**RFID physique et 2FA restent hors scope.** La réservation d'un pool de machines
n'est pas impliquée par les catégories.
