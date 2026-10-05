# Revue de sortie — Phase W (S205 à S210)

**Verdict : pas prêt pour des tests usagers en l'état — trois blocages (deux sur la borne de check-in, un sur le bouton de retour au téléphone), le reste se corrige par ajustements, sans refonte.**

**Compte : 🔴 3 · 🟠 18 · 🟡 22** (43 constats).

## Application de la revue (2026-10-05) — R → fait / non fait

Tout est fait (R8 : règle générique `.form-narrow` dans `components.css`, décision opérateur du 2026-10-05). Les textes (nouveaux ou modifiés) attendent la fusion de `scratchpad/i18n/revue.json` dans `translations/` ; **rien n'est déployé, rien n'est commité**.

| R | Statut | Note |
|---|---|---|
| R1 | fait | `machine-report` : plus de bandeau image, une seule carte, titre `machine_report.heading`, chapeau d'une phrase |
| R2 | fait | `form-required` sur le seul champ requis ; « (facultatif) » retiré des libellés |
| R3 | fait | `.btn-submit button` ajouté à la liste `pointer: coarse` de `style.css` |
| R4 | fait | lien « Signaler une panne » dans le pied de la carte de réservation de la fiche publique ; fonction Twig `machine_reports_open()` (`MachineReportExtension`) |
| R5 | fait | « Imprimer tous les QR codes » (`/admin/signalements/qr`, même gabarit que l'étiquette, une par page) ; état vide réécrit |
| R6 | fait | le champ de note passe dans la colonne du signalement (`form="report-resolve-{id}"`) |
| R7 | fait | « panne » : `mops.reports`, `mail.machine_report.intro`, `admin_attention.st_report` |
| R8 | fait | `.form-narrow` (`components.css`, `--form-measure: 560px`) posée sur la carte de `machine-report` ; documentée dans `/admin/design#formulaire-etroit` |
| R9 | fait | aide « appareil photo » retirée (clé `help_photo` morte) |
| R10 | fait | carte machine : lien « Tous les signalements », bouton « Imprimer le QR code » |
| R11 | fait | `adm.col_storage` « Rangement », `form.storage_location` « Emplacement de rangement » (5 langues) ; sonde S206 étendue à la famille du mot (`stock\w*`, `lager\w*`) |
| R12 | fait | la pastille Stock de la liste est un lien vers `…/edit#stock` |
| R13 | fait | « Ajouter » / « Retirer » |
| R14 | fait | fonction Twig `stock_units()` sur `MaterialStock::UNITS` |
| R15 | fait | `.pp-kq-close` : fond transparent, police héritée ; `.ck-actions` ne vise plus que `.pp-kq-tile` |
| R16 | fait | le champ de projet quitte la borne (gabarit + contrôleur : la note d'un visiteur n'est plus lue) |
| R17 | fait | la note du membre : repli en dernier, sous les boutons de motif |
| R18 | fait | `checkin.title` « Je suis là », `checkin.admin_title` « Présences » |
| R19 | fait | deux titres `form-section` de même forme ; `panel_note` retiré |
| R20 | fait | `checkin.empty_visits` dit quoi faire |
| R21 | fait | une règle dans `admin.css` : sans recherche, ce qui suit le titre du bandeau va à droite |
| R22 | fait | apostrophes typographiques dans les clés `checkin.*` concernées |
| R23 | fait | `checkin.kiosk_member_hint` |
| R24 | fait | `.pp-kq-head .pp-kq-close { margin-top: 0 }` |
| R25 | fait | « ajouter » partout (bouton, vide, flash, e-mail) |
| R26 | fait | `admin-action`, colonne « Actions » |
| R27 | fait | « Règlement du lab » partout ; surtitre « Sécurité » retiré ; fonction `charter` = « Règlement à accepter » ; la page garde `/charte` |
| R28 | fait | après acceptation, retour à `app_home` |
| R29 | fait | `feedback.css` : sous 560 px le panneau se cale sur l'en-tête |
| R30 | fait | pastille « Un retour ? », panneau « Un retour sur le site », ligne « Une machine en panne ? » vers `/signaler` (si `machine_reports_open()`) |
| R31 | fait | `.header-right { flex-wrap: wrap }` et `.header-search { flex: 1 1 100% }` sous 560 px — **à mesurer à 390 px réels** |
| R32 | fait | `.fab-toast(s)`, `.btn.btn-primary`, `.form-field` : 38 lignes de `feedback.css` supprimées ; les messages sortent sous l'en-tête (`_header.html.twig`) |
| R33 | fait | libellé visible « Votre message » |
| R34 | fait | « Gêne » |
| R35 | fait | pastille neutre pour les trois types |
| R36 | fait | l'agent utilisateur passe en `title` du lien ; colonne « Actions » |
| R37 | fait | `admin.css` : la ligne vide est exclue de la colonne épinglée |
| R38 | fait | un seul partiel `site/_reason_list.html.twig` (check-in et avertissements) ; l'avertissement gagne « Renommer » (`UserWarnings::renameReason`) ; ↑ ↓ seulement si `movable` (check-in : l'ordre se voit sur la borne) |
| R39 | fait | `admin_list.all_m` |
| R40 | fait | sans CSS : le partiel enveloppe le repli dans un `<div>`, enfant direct de la carte, qui reçoit le retrait des tuiles |
| R41 | fait | partiel `site/_honeypot.html.twig` (panne + borne) ; `.ck-hp` supprimé ; `.sr-only` ajouté à `kiosk.css` (la borne ne charge pas `style.css`) |
| R42 | fait | « facultatif » retiré des quatre clés |
| R43 | fait | `.fb-kinds` devient `.choice-tiles` (`components.css`) et un exemple vit dans `/admin/design#choix-tuiles` |

## Limites de cette revue (à lire avant les constats)

- **Les captures `*-mobile.png` ne sont pas des rendus à 390 px.** La page y est mise en page à environ 500 px puis rognée à 390 (preuve : le QR de `w04-kiosk-checkin-mobile` et l'icône de `w01-signaler-mobile` sont centrés à x ≈ 250). Je n'en tire donc aucun constat de « débordement horizontal » ; les constats mobiles ci-dessous reposent sur la hauteur des cibles, l'ordre du contenu et le CSS. **À refaire à 390 px réels** avant de clore.
- **Trois blocs neufs ne sont sur aucune capture** : les cartes Avertissements de `w11` (coupée avant), le bloc Stock de `w12` (coupé avant), la carte Signalements de `w17` (section « État et maintenance » repliée). Ils sont jugés sur le code seul.
- **Aucun état rempli** : les listes admin, la carte machine et le bloc Stock n'ont été vus que vides. Les constats R6 et R12 sont lus dans le code, pas mesurés.

---

## S205 — Signaler une panne (QR, page publique)

**R1 · 🟠 · `w01-signaler-mobile`** — Au téléphone, le premier écran est occupé par l'en-tête puis par une carte d'image de 220 px qui ne montre qu'une icône de remplacement ; le champ à remplir commence vers y = 745 sur 812. Puis la même question est posée trois fois : chapeau « Dites-nous ce qui ne va pas », titre de carte « Que se passe-t-il ? », libellé « Ce qui ne va pas ». Contraire à « un signalement ne doit pas décourager » et à « pas de texte pour ne rien dire ».
**Correction** : `templates/site/machine-report.html.twig` — retirer l'`embed` de `_detail_hero` (l. 25-35) et reprendre la structure de `machine-report-choose.html.twig` : une seule `section.detail-card`, `<h1 class="detail-card-title">Signaler une panne : {{ machine.nom }}</h1>`, le chapeau en une phrase (« L'équipe est prévenue tout de suite. Aucun compte n'est nécessaire. »), puis le formulaire. Supprimer la clé `machine_report.form_title`.

**R2 · 🟠 · `w01-signaler`** — Deux libellés portent « (facultatif) » et le seul champ obligatoire ne porte rien. `FORM-DESIGN.md` règle 5 dit l'inverse : « requis » en petit mot gris, rien sur l'optionnel. La classe `is-required` est posée (l. 42) mais sans le `<span class="form-required">` qu'elle stylise (`components.css:557`).
**Correction** : l. 43, ajouter après le libellé `<span class="form-required" aria-hidden="true">{{ 'common.required'|trans }}</span>` (même balisage que `form/admin_theme.html.twig` l. 76). `machine_report.field_photo` → « Une photo », `machine_report.field_contact` → « Comment vous joindre ».

**R3 · 🟠 · `w01-signaler-mobile`** — « Envoyer le signalement », seul verbe d'une page faite pour le téléphone, mesure 34 px de haut. `.btn-submit button` (`components.css:598`) n'a pas de hauteur minimale et n'est pas dans la liste `@media (pointer: coarse)` de `style.css:4494`.
**Correction** : à la source, ajouter `.btn-submit button` à cette liste (`min-height: 44px`). Aucune règle locale.

**R4 · 🟠 · `w17-fiche-machine-staff` + code** — La page `/signaler/{id}` n'est liée de nulle part sur le site : `app_report_machine` n'apparaît que dans les gabarits `machine-report*`. Un membre devant une machine sans étiquette, ou dont l'étiquette est arrachée, n'a aucun chemin — sauf le bouton « Signaler » de l'en-tête, qui mène à autre chose (voir R30).
**Correction** : `machine-detail.html.twig`, sur la ligne de lieu sous les boutons (« Atelier découpe · Libre le… »), un lien texte « Signaler une panne » vers `path('app_report_machine', {id: machine.id})`, sous la même garde que la route (fonction `machine_reports` allumée).

**R5 · 🟠 · `w07-admin-signalements`, `w17`** — L'état vide dit : « Le QR code à coller sur chaque machine se trouve sur sa fiche, carte "Signalements" ». Pour 38 machines : ouvrir la fiche, déplier « État et maintenance », cliquer, imprimer, revenir — 38 fois. La mise en service de la fonction est le geste le plus coûteux de la phase.
**Correction** : dans `admin-machine-reports.html.twig`, bloc `header_extra`, un lien « Imprimer tous les QR codes » vers une variante de `machine-report-qr.html.twig` qui boucle sur les machines (même gabarit, `break-after: page` par étiquette). Remplacer la phrase de l'état vide par : « Aucun signalement ouvert. Collez un QR code sur chaque machine pour en recevoir. »

**R6 · 🟠 · code, non capturé rempli** — `admin-machine-reports.html.twig` l. 66-87 : la cellule d'actions d'un signalement ouvert contient un champ texte (note), « Résoudre » et « Supprimer », alors que `pin_actions` fixe cette colonne à `9.5rem` (`admin.css:1327`). Trois contrôles dans 152 px.
**Correction** : déplacer le champ de note dans la colonne « Signalement » (`is-grow`), relié au formulaire par l'attribut `form="…"` ; ne garder que les deux verbes dans la colonne épinglée. À vérifier sur une capture remplie.

**R7 · 🟡 · `w07`, `w16`, e-mail** — Un même objet a quatre noms : « Pannes signalées » (titre, onglet), « Signalements » (colonne, carte machine), « Signalée » (état du tableau de bord), « un problème sur %machine% » (e-mail `machine_report.intro`).
**Correction** : « panne » partout où l'on nomme la chose. `mops.reports` → « Pannes signalées » ; `emails.machine_report.intro` → « Quelqu'un vient de signaler une panne sur %machine%. »

**R8 · 🟡 · `w01-signaler`** — Sur grand écran les trois champs font 1 180 px de large. `FORM-DESIGN.md` règle 6 : la carte est étroite.
**Correction** : borner le formulaire avec le jeton existant `--form-measure` (celui de `admin.css:83`), sans nouvelle classe.

**R9 · 🟡 · `w01`** — « Sur téléphone, le bouton ouvre l'appareil photo. » décrit ce que l'usager constate en appuyant.
**Correction** : supprimer `machine_report.help_photo` et son `<p class="form-help">`.

**R10 · 🟡 · code (`machine-detail.html.twig` l. 418-441)** — La carte Signalements de la fiche machine ne montre pas la photo et n'offre aucun lien vers la liste ; son bouton s'appelle « QR à coller sur la machine » (un nom, pas un geste).
**Correction** : `mops.reports_qr` → « Imprimer le QR code » ; ajouter sous la liste un lien « Tous les signalements » (clé `all_reports` déjà créée pour le tableau de bord) vers `app_admin_machine_reports`.

---

## S206 — Stocks de consommables

**R11 · 🟠 · `w13-admin-materiaux`** — Colonnes voisines « Stockage » et « Stock ». Surtout : « Stockage » (`adm.col_storage`) et « Emplacement de stockage » (`form.storage_location`, titre de carte de la fiche publique d'un matériau) restent affichés fonction éteinte. La vérification de l'opérateur (« chercher le mot "stock", il n'y en a pas ») échoue, et l'interrupteur promet lui-même : « Éteint : le mot "stock" n'apparaît nulle part ». Les deux autres titres ont bien été renommés (« Quantité et rangement », « Rangement et rachat ») ; ces deux-là ont été oubliés.
**Correction** : `adm.col_storage` → « Rangement » ; `form.storage_location` → « Emplacement de rangement » (cinq catalogues).

**R12 · 🟠 · `w13`, `w12-admin-materiau`** — Enregistrer une sortie de stock, le geste quotidien, demande : liste → « Modifier » → faire défiler tout le formulaire du matériau (dont 31 cases de machines) → saisir. La pastille de la colonne « Stock » n'est pas cliquable.
**Correction** : `admin-materials.html.twig`, envelopper la cellule Stock dans un lien vers `path('app_admin_material_edit', {id: material.id}) ~ '#stock'` (l'ancre `id="stock"` existe déjà).

**R13 · 🟡 · code (`_material_stock_admin.html.twig` l. 35-36)** — Boutons « Entrée » et « Sortie » : deux noms, et « Entrée » se lit aussi comme la touche.
**Correction** : `stock.move_in` → « Ajouter », `stock.move_out` → « Retirer ».

**R14 · 🟡 · code (l. 82)** — La liste des unités est écrite en dur dans le gabarit alors que `MaterialStock::UNITS` existe.
**Correction** : exposer la constante (fonction Twig de `MaterialStockExtension`) et boucler dessus.

---

## S207 — Check-in par paliers

**R15 · 🔴 · `w04-kiosk-checkin`** — Le bouton « Passer » est un rectangle gris clair dont le texte est illisible (blanc sur gris, corps minuscule). Cause : `<button class="pp-kq-close">` — la classe est faite pour un lien et ne fixe pas de fond, le bouton garde donc le fond par défaut du navigateur ; et `.ck-actions button { font: inherit }` (`page-kiosk-checkin.css:13`) écrase sa taille de 2vw. Au palier 3, la seule sortie sans réponse est invisible.
**Correction** : `page-kiosque-accueil.css:58`, ajouter à `.pp-kq-close` : `background: transparent; font-family: inherit; cursor: pointer;`. Dans `page-kiosk-checkin.css:13`, restreindre le sélecteur à `.ck-actions .pp-kq-tile`.

**R16 · 🔴 · `w04-kiosk-checkin`** — Palier 4 allumé, la borne affiche « Sur quoi travaillez-vous ? (facultatif) » en champ pleine largeur, déplié, au même rang que le nom et avant le motif. C'est exactement ce que la position de l'opérateur interdit : la documentation de projet n'est jamais mise en avant.
**Correction** : `kiosk-checkin.html.twig` l. 46-49, supprimer le champ de la borne (un visiteur sans compte, debout devant un écran, est le dernier à documenter un projet). Le palier 4 reste sur la page du membre.

**R17 · 🟠 · `w02-checkin`** — Sur la page du membre, le repli « Sur quoi travaillez-vous ? (facultatif) » est le premier élément de la carte, au-dessus des boutons de motif. Replié, mais lu en premier.
**Correction** : `checkin.html.twig`, déplacer le bloc `<details>` (l. 28-34) après `div.ml-head-actions` ; `checkin.note_label` → « Ajouter une note sur votre projet ».

**R18 · 🟠 · `w02`, `w03-kiosk`, `w10-admin-checkin`** — Trois noms pour la même chose : « Check-in » (titre des pages membre, borne et admin), « Je suis là » (menu, tuile de la borne), « Présences » (onglet admin). Dans `w10`, l'onglet actif dit « Présences » et le titre du panneau « Check-in ».
**Correction** : `checkin.title` → « Je suis là » (on arrive sur ce qu'on a touché) ; `checkin.admin_title` → « Présences ». « Check-in » ne reste que comme nom de la fonction dans Fonctionnalités.

**R19 · 🟡 · `w10`** — Le premier tableau n'a pas de titre, le second a un `<h3>` nu, collé au tableau, sans `<h2>` au-dessus.
**Correction** : `admin-checkin.html.twig` — deux titres de même forme, `<div class="form-section"><h3>…</h3></div>` (le balisage de `_material_stock_admin`) : « Présents maintenant » puis le nom de la période ; retirer `panel_note`.

**R20 · 🟡 · `w10`** — « Personne n'est enregistré au lab en ce moment. » et « Aucune visite sur cette période. » : deux états vides qui ne disent pas quoi faire.
**Correction** : `checkin.empty_visits` → « Aucune visite sur cette période. Les membres s'enregistrent sur la borne ou depuis "Je suis là". » ; garder l'autre phrase telle quelle.

**R21 · 🟡 · `w10`** — « Exporter en CSV » est collé au titre, à gauche, alors que les actions d'en-tête sont à droite partout ailleurs (`w12`, `w13`).
**Correction** : dans `_admin_list.html.twig`, pousser le bloc `header_extra` à droite quand il est seul (règle unique dans `admin.css`, pas dans la page).

**R22 · 🟡 · catalogue** — Les chaînes `checkin.*` ont des apostrophes droites (« J'arrive », « Aujourd'hui », « Personne n'est… ») ; le reste de la phase a l'apostrophe typographique.
**Correction** : remplacer `'` par `’` dans le bloc `checkin:` de `messages.fr.yaml`.

**R23 · 🟡 · `w04`** — « Scannez avec votre téléphone, connecté : un geste. » est difficile à lire.
**Correction** : `checkin.kiosk_member_hint` → « Scannez ce code avec votre téléphone. »

**R24 · 🟡 · `w04`** — « Fermer » est décalé vers le bas par rapport au titre : `.pp-kq-close` porte un `margin-top: 2vh` prévu pour le bloc des horaires.
**Correction** : `page-kiosque-accueil.css` — `.pp-kq-head .pp-kq-close { margin-top: 0; }`.

---

## S208 — Avertissements

**R25 · 🟠 · `w09-admin-avertissements` + code** — Trois verbes pour un geste : « Ajouter un avertissement » (repli), « Enregistrer l'avertissement » (bouton), « Pour en poser un… » (état vide), « Avertissement posé » (e-mail), « Avertissement enregistré » (message).
**Correction** : « ajouter » partout. `warnings.add_submit` → « Ajouter l'avertissement » ; `warnings.empty` → « Aucun avertissement. Pour en ajouter un, ouvrez la fiche d'une personne. » ; `warnings.added` → « Avertissement ajouté pour %name%. » ; e-mail : « Avertissement ajouté ».

**R26 · 🟡 · `w09` + code (`admin-warnings.html.twig` l. 30, 49)** — La dernière colonne s'appelle « État » mais contient le bouton « Lever » ; et ce bouton est un `.admin-secondary-button`, pas la forme unique du verbe de ligne (`.admin-action`, `admin.css:1336`) qu'utilise la liste des retours.
**Correction** : classe `admin-action` sur le bouton ; en-tête de colonne « Actions » ; l'état « Levé le… » reste dans cette cellule quand il n'y a plus de verbe.

---

## S209 — Charte de sécurité

**R27 · 🟠 · `w05-charte`, pied de page** — Un seul texte, deux noms. Le texte affiché est celui du règlement (`CharterAcceptances` : « c'est le règlement du lab ») ; le pied de page le nomme « Règlement du lab », la page et la ligne « À faire » le nomment « Charte de sécurité ». Le membre croit à deux documents ; l'admin ne sait pas que modifier le règlement redemande l'accord de tout le monde. Le surtitre « SÉCURITÉ » au-dessus de « Charte de sécurité » répète en plus le titre.
**Correction** : un seul nom, celui qui existe déjà. `charter.title` → « Règlement du lab » ; `charter.todo` → « Lire le règlement du lab » ; `charter.admin_label` → « Règlement ». Dans Fonctionnalités, description : « Le règlement du lab, que chaque membre accepte une fois ; redemandé quand il change. »

**R28 · 🟡 · code (`CharterController` l. 45)** — Après « J'ai lu et j'accepte », on reste sur la page, sans lien de retour.
**Correction** : rediriger vers `app_home`, qui a déjà sa zone de messages (`index.html.twig` l. 83).

---

## S210 — Bouton « Signaler un problème »

**R29 · 🔴 · `w19-retour-ouvert-mobile`** — Le panneau s'ouvre à moitié hors de l'écran, à gauche : le titre est coupé (« problème »), la tuile « Bug » est invisible, le champ et la note sont tronqués. Cause : `.fb-panel { right: 0; width: min(360px, 92vw) }` s'accroche au bord droit du bouton, qui n'est pas au bord droit de l'écran sur téléphone. Le débordement à gauche ne se rattrape pas en faisant défiler. La fonction est inutilisable au téléphone, alors qu'elle est l'outil des tests usagers.
**Correction** : `feedback.css` — `@media (max-width: 560px) { .fb { position: static; } .fb-panel { left: var(--spacing-md); right: var(--spacing-md); width: auto; } }` (le panneau se cale alors sur l'en-tête, qui est déjà positionné).

**R30 · 🟠 · `w19-retour-ouvert`, `w16-fonctionnalites`** — Le mot « Signaler » sert à deux fonctions : « Signaler une panne » (S205, une machine) et « Signaler » / « Signaler un problème » dans l'en-tête de chaque page (S210, le site). Devant une machine en panne, un membre connecté clique sur « Signaler », choisit « Bug » et décrit la panne : elle arrive dans « Retours des usagers », sans machine, hors de l'historique. La liste admin s'appelle d'ailleurs « Retours », pas « Signalements ».
**Correction** : `feedback.entry` → « Votre avis » ; `feedback.title` → « Votre avis sur le site » ; libellé de la fonction → « Donner son avis sur le site » ; `feedback.empty_open` : remplacer « le bouton "Signaler" » par « le bouton "Votre avis" ». Si S205 est allumée, une ligne en bas du panneau : « Une machine en panne ? » avec un lien vers `/signaler`.

**R31 · 🟠 · calculé, à mesurer à 390 px** — Sous 560 px, `.header-right` tient sur une seule ligne : recherche + « Signaler » + avatar + « Déconnexion ». À 390 px il reste environ 90 px pour la recherche (champ et bouton compris), alors que le commentaire de `style.css:4701` dit que la recherche « mérite sa propre ligne ». La pastille ajoutée par S210 est ce qui la comprime.
**Correction** : `style.css`, bloc `@media (max-width: 560px)` : `.header-right { flex-wrap: wrap; }` et `.header-search { flex: 1 1 100%; }`.

**R32 · 🟠 · code (`feedback.css`)** — Sur 44 lignes, trois blocs refont des composants existants : `.fb-toast` refait `.fab-toast` (`components.css:41`) sans sa disparition automatique, en position fixe par-dessus le bandeau d'annonce ; `.fb-send` refait `.btn.btn-primary` ; `.fb-panel textarea` refait `.form-field textarea`.
**Correction** : `_feedback.html.twig` — messages en `<div class="fab-toasts"><div class="fab-toast fab-toast--success">…` (et `--error`) ; bouton `<button class="btn btn-primary">` ; champ dans un `div.form-field`. Supprimer les lignes 25-34 et 36-44 de `feedback.css`.

**R33 · 🟠 · `w19-retour-ouvert`** — Le champ de message n'a pas de libellé visible : le texte d'aide (« Que s'est-il passé ? Qu'attendiez-vous ? ») sert de libellé et disparaît à la première lettre.
**Correction** : `<label for="fb-message">Votre message</label>` visible au-dessus, `id="fb-message"` sur le champ, retirer `aria-label` ; garder le texte d'aide.

**R34 · 🟡 · `w19`** — « Gêne d'ergonomie » est du jargon et passe sur deux lignes, ce qui déséquilibre les trois tuiles.
**Correction** : `feedback.kind_ux` → « Gêne ».

**R35 · 🟡 · code (`admin-feedback.html.twig` l. 49)** — Le type est peint avec les signaux d'état (Bug = rouge « stop », Idée = vert « go »). Un type n'est pas un état : le vert sur « Idée » se lit « réglé ».
**Correction** : `signal: 'muted'` pour les trois types ; le libellé porte l'information.

**R36 · 🟡 · code (l. 39, 53)** — Chaque ligne affiche 80 caractères d'agent utilisateur du navigateur sous le message ; et la colonne s'appelle « Action » ici, « Actions » sur les pannes.
**Correction** : passer l'agent utilisateur en attribut `title` du lien de page ; `feedback.col_action` → « Actions ».

---

## Transversal

**R37 · 🟠 · `w07`, `w08-admin-retours`, `w09`** — La phrase d'état vide est alignée à droite sur les trois listes à colonne épinglée, centrée sur `w10` (sans épingle). Cause : `.admin-table.has-pinned-actions td:last-child` (`admin.css:1327`) attrape l'unique cellule de la ligne vide et lui donne `text-align: right`, `position: sticky` et une ombre. Défaut du composant partagé (31 listes), mais ce sont ici les trois seuls écrans que l'opérateur verra d'abord.
**Correction** : `admin.css:1327-1328`, exclure la ligne vide : `.admin-table.has-pinned-actions tr:not(.admin-table-empty) > td:last-child`.

**R38 · 🟠 · code (`admin-checkin.html.twig` l. 66-90, `admin-warnings.html.twig` l. 59-86)** — Deux « listes de motifs réglables » livrées dans le même commit, avec deux balisages (`form.cat-inline` et boutons `ml-btn` d'un côté ; `ul > li` et `admin-secondary-button` de l'autre), deux vocabulaires (« Désactiver / Activer » contre « Retirer de la liste / Remettre dans la liste » ; « Motifs de visite » contre « Régler les motifs » ; « Ajouter un motif » contre « Ajouter le motif ») et deux jeux de fonctions (renommer et ordonner d'un seul côté). Côté check-in, chaque ligne porte quatre boutons pleins de couleur primaire.
**Correction** : un seul partiel `site/_reason_list.html.twig` (paramètres : `reasons`, `route`, `token`), sur le modèle `cat-inline` déjà utilisé par les catégories de machines et d'événements ; verbes « Renommer », « Retirer de la liste », « Remettre dans la liste », « Ajouter un motif » ; boutons de ligne en `admin-action`. Titre du repli : « Régler les motifs » des deux côtés.

**R39 · 🟡 · `w09`, `w08`** — « Toutes » devant des masculins : tuiles « Toutes · Actifs · Levés » (avertissements) et filtre « Type : Toutes » (retours). `admin_list.all` n'existe qu'au féminin.
**Correction** : créer `admin_list.all_m: "Tous"` et l'utiliser dans `admin-warnings.html.twig` l. 17 et `admin-feedback.html.twig` l. 26.

**R40 · 🟡 · `w09`, `w10`** — Le repli des motifs (`details.settings-danger--neutral`) touche les bords du panneau : double filet, coins arrondis dans un cadre droit, alors que les tuiles au-dessus ont un retrait.
**Correction** : une règle dans `admin.css` donnant à un `details.settings-danger` enfant direct du contenu d'une liste le même retrait latéral que la rangée de tuiles.

**R41 · 🟡 · code** — Deux pots de miel différents : `.sr-only` et clé traduite sur la page de panne ; `.ck-hp` (classe neuve, `page-kiosk-checkin.css:14`) et libellé « Website » en dur sur la borne.
**Correction** : un partiel `site/_honeypot.html.twig` repris des lignes 56-59 de `machine-report.html.twig` ; supprimer `.ck-hp`.

**R42 · 🟡 · catalogue** — « (facultatif) » revient hors S205 : `checkin.note_label`, `machine_report.note_placeholder`, `stock.note_placeholder` (« Facultatif — … »), `stock.threshold_help` (« Facultatif. … »). Même règle 5 que R2.
**Correction** : retirer la mention dans ces quatre clés.

**R43 · 🟡 · dépôt** — Aucun exemple ajouté à `/admin/design` (aucun gabarit de design dans le diff) alors que la phase introduit un motif récurrent : le choix à trois tuiles du panneau de retour (`.fb-kinds`), seul sélecteur de ce type dans le CSS.
**Correction** : y ajouter l'exemple, et déplacer `.fb-kinds` de `feedback.css` vers `components.css` sous un nom neutre pour qu'il soit réutilisable.

---

## Ce qui est bien et doit rester

- **Le check-in de base ne demande aucune saisie** : un titre, un bouton « J'arrive », puis « Je pars ». Au palier 3, la question est le bouton — un seul geste. La borne ne cherche jamais un membre par son nom.
- **Stocks éteints** : colonne, pastille, bloc et carte publique disparaissent vraiment (`w14-materiau-public` ne montre rien pour un matériau non suivi) ; la garde est dans le partiel, à un seul endroit.
- **La ligne « À faire » de la charte** (`w06-accueil-todo`) : même composant que les autres lignes, un verbe, et le même partiel sert l'accueil et Mon compte.
- **Les quatre listes admin** passent par `_admin_list` et `_data_table`, avec tuiles comptées et états vides rédigés ; pannes et retours disent où se trouve le geste qui les alimente.
- **La page QR imprimable** (`w15-qr`) : autonome, noir sur blanc, une consigne lisible de loin, bouton masqué à l'impression.
- **La charte sur la fiche admin** tient en un champ de la grille existante (`w11`), sans carte neuve.
- **Avertissements** : registre sans effet ni visibilité pour la personne, et c'est dit dans l'aide du formulaire.
- **Les replis** réutilisent `settings-danger--neutral`, conformément à `FORM-DESIGN.md` règle 1.
