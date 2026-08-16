# FabOS — roadmap active

**Mise à jour : 2026-08-16.** ⚠️ **Cette page ne contient que le travail
restant.** Tout ce qui est livré en est sorti : les récits sont dans
`HISTORY.md`, et son **index en fin de fichier** donne une ligne par session de
S102 à S142. Une session livrée qui reste ici finit par être refaite.

Livré à ce jour : phases A à F (S102–S128), la moitié de G (S129–S132b), S134c,
S134g, toute l'interface S134h–S141, puis S138c et S142 (une seule barre
latérale, une seule forme de page, le CSS des partials partagés remonté).

## Cap produit

Tout fablab, école ou atelier partagé déploie **les seules fonctions dont il a
besoin**, avec une expérience cohérente.

- une installation, plusieurs **lieux** physiques ; aucun portail ;
- SSO entre instances sans partager droits ni données ;
- sept audiences intégrées protégées + groupes locaux + packages assignables ;
- deux droits, **Use et Manage**, par feature/lieu/scope ; le reporting est
  dans Manage ;
- réservations, quotas et reporting montrés dans chaque feature, moteurs
  communs ;
- profils publics volontaires, échanges inter-FabOS consentis, badges
  cumulatifs et fédérables ;
- plus tard : Paiements facultatif, puis messagerie Formation ;
- **Configuration → Thèmes** réunit identité visuelle, images, menus et accueil ;
- **un seul** système central de listes, filtres, workspaces, composants et CSS.

Modèle cible détaillé : [`USAGE_RIGHTS_VISION.md`](/roadmap/droits-usage).

## Frontière d'une instance

**Même FabOS** = gouvernance, admins, annuaire, politiques, moteur de réservation
et source de vérité partagés → lieux. **Autre FabOS** dès qu'un service veut
son propre admin, thème, catalogue ou cycle de vie ; la distance ne décide pas.
Le SSO évite un mot de passe, il ne partage rien : le métier passe par le Réseau,
objet par objet, avec provenance. Chaque ressource a **une** instance
autoritaire ; ailleurs, projection lecture seule + lien. **Une réservation
distribuée n'est jamais implicite dans une synchronisation.**

## Règles de construction

1. **Une source de métadonnées.** `FeatureWorkspaceRegistry` décrit la
   navigation et les scopes ; voters et services restent l'autorité métier.
2. **Une liste, un shell.** `_admin_list` + `_data_table` + `_cell_*` ; la page
   n'apporte que ses colonnes et ses données.
3. **Pas de surcharge.** Lieu, tâche et filtre sont trois axes différents.
   Six filtres rapides maximum, le reste dans `Plus de filtres` et en chips.
4. **URL explicable.** Défaut agrégé ; lieu, recherche, filtres et
   pagination partageables ; une préférence de profil n'est jamais un droit.
5. **Sécurité côté serveur.** L'UI lit le même verdict que les voters. Un lien
   caché n'autorise ni n'interdit rien.
6. **Migrations expand/backfill/contract.** Jamais `schema:update --force`.
7. **Simulation avant activation.** Chaque route mutante a son voter atomique.
8. **Cinq langues, sombre, mobile, clavier.** Toute primitive est montrée avec
   le vrai composant dans `/admin/design`.
9. **Artemis est la définition de done.** Doc, commit, deploy CT210, lint,
   restart, vérification réelle. Jamais `deploy.sh`.

## Décisions opérateur fixées

- **Guest** = anonyme sans compte. Visibilité ≠ action. Le réglage FabOS est un
  défaut, pas un plafond : par événement, `inherit`/`allow`/`deny`, séparément
  pour `view` et `register`.
- **Groupes intégrés protégés** : clés stables, non supprimables ; libellés et
  attributions configurables. User = tout compte actif, Guest = virtuel.
- **Institution** : descriptive, ou connectée après découverte sur une URL HTTPS
  et **confirmation explicite** de confiance. Changer d'origin la suspend.
- **Partage** : le personnel sort seulement si l'instance l'autorise **et** que
  le membre consent à cette donnée et cette destination.
- **Un badge n'est jamais effacé** : définition archivable, retrait = révocation
  auditée qui reste au journal.
- **Import QR** automatique après un consentement récapitulatif unique. Un badge
  révoqué n'est jamais réactivé ; aucun conflit local écrasé en silence.
- **Packages cumulatifs, fermés par défaut.** Aucun package ne retire un droit.
  Aucune fusion de champs entre politiques.
- **Matériaux** = catalogue partageable ; disponibilités et emplacements locaux.
  Sous Équipement par navigation, pas par confusion catalogue/stock.
- **Navigation** : « Le lieu » → « Utilisateurs » ; Horaires sous Lieux ;
  interface et accueil sous Configuration → Thèmes.
- **Le titre d'une liste = son entrée de menu** (`admin_nav.entry.<route>`), lu
  depuis `NavBuilder`, jamais recopié. « Quotas », pas « Gestion des quotas ».
  ⚠️ Une exception validée à l'écran : **`/admin/machines` porte sa SECTION**.
- **Admin recovery n'est pas un bypass** : ni badge requis ni arrêt de sécurité.
- **Deux droits.** Pas de Report : consultation/export sont dans Manage, et
  Manage ne confère **jamais** Use.
- **Formation = catalogue global** ; seules les sessions ont un lieu.
- **Exposition publique par surface** : activation opérateur **et** consentement.
- **Sessions IdP** : révocation immédiate si connue, 24 h max en cas de panne.

## Répartition

**Terra** domaine, migrations, services, voters, protocoles · **Luna**
architecture d'information, composants, listes, a11y · **Sol** revue
indépendante — et Sol ne valide jamais sa propre implémentation.

---

# Ce qui reste

## Phase G — bloquante avant le commerce

Ne commencer ni paiement, ni catalogue d'offres, ni ledger tant que S134b n'a
pas validé le cleanup transversal.

| Session | Attendu | Qui |
|---|---|---|
| **S132 ⬅️** | shell/design system : retirer les derniers menus, scaffolds et CSS locaux ; refaire Logs RFID, Réglages, E-mails et Fonctionnalités | Luna + Terra |
| **S133 ⬅️** | parité fonctionnelle : catégories machine, objets/Prêts, Événements (aperçu, inscriptions, tickets), Configuration ; **le champ lieu sur les formulaires** | Terra + Luna |
| **S133b** | droits administrables : groupes, attributions, grants v2 Use/Manage, scopes ; shadow visible et explicable | Terra + Luna |
| **S134** | activation graduelle des grants v2 sur les chokepoints audités, puis retrait du package legacy | Terra + Sol |
| **S134b** | cleanup final : dette, routes orphelines, traductions, a11y, **et l'inventaire action-opérateur** — chaque objet annoncé est créable, éditable, archivable depuis son workspace | Luna + Terra, Sol valide |

✅ **Le critère de sortie de la Phase G est atteint** (vérifié 2026-08-16) : le
**champ lieu existe sur les huit formulaires** machine/espace/événement/objet
via `VenueChoiceType`. Sans lui, tout atterrissait à jamais sur le lieu par
défaut et un second lieu ne pouvait rien contenir. ✅ Les deux régressions
datées sont corrigées aussi : `admin-events` calcule « À venir » sur l'heure du
lab et non sur `date()` en UTC, et `admin_list.all` est traduite. **Ce qui reste
de S133 est la liste ci-dessous.**

### S132 — la liste précise

- **Fonctionnalités** : reconstruire sur le registre des workspaces (workspaces
  réels, dépendances, effet d'une désactivation). Pas de taxonomie parallèle.
- **Réglages** (page par défaut de Configuration) : cartes courtes et liées —
  Général, localisation, alertes, exploitation/sécurité, avancé — avec résumé
  d'état et sauvegarde par section. Les contrôles dangereux (enforcement des
  droits) restent en divulgation avancée, avec préflight et confirmation.
- **E-mails** : trois tâches lisibles — état/diagnostic, modèles/préférences,
  journal filtrable (période, statut, destinataire). Secrets jamais affichés.
- **Logs RFID** : filtres date/lecteur/machine/résultat, détail à la demande.
- **Lecteurs RFID** est une référence **à extraire**, pas à recopier : ses bons
  éléments deviennent génériques dans `/admin/design`, puis la page elle-même
  n'a plus de CSS local.
- ✅ **La variante de sidebar `'edit'` est morte (S142).** Il n'y a plus qu'une
  barre latérale et **aucun paramètre ne la choisit** : ni `shells`, ni
  `admin_sidebar_variant`, ni `sidebar_variant`. Ne pas en réintroduire une pour
  un cas particulier — les quatre précédentes sont toutes nées comme ça.
- ✅ **Il n'y a plus qu'UNE forme de page dans l'admin (S142d).** Le bandeau
  pleine largeur a disparu des 31 pages qui le gardaient ; titre, sous-titre et
  lien de retour sont dans la bande de la carte, via `_admin_form_head.html.twig`
  (26 pages) ou la bande écrite à la main du formulaire de lecteur RFID. Une
  carte de formulaire porte **trois** classes : `.admin-panel`,
  `.admin-list-card`, `.admin-form-card`. ⚠️ Seul le **tableau de bord** garde un
  `.admin-header` — c'est une carte d'accueil, pas une barre de titre. Non
  tranché.
- ✅ **Les partials partagés ne portent plus de CSS (S142)**, sauf
  `_header.html.twig`, dont le `<style>` est la couleur d'accent de l'instance —
  une donnée. `_admin_edit_styles` supprimé ; `_delete_confirm_modal` et les
  neuf règles `.modal*` que les **deux** appelants de `_rfid_pairing_modal`
  recopiaient sont dans `admin.css`.
- ⚠️ **Le CSS local restant : 950 règles sur 47 gabarits** (S142, depuis
  1 118/65 mesurés au même script). ⚠️ **Ne pas soustraire** des chiffres
  antérieurs — 1 321/66 et 1 232/87 comptaient autre chose. Les plus gros
  restent `formation-suivi`, `event-detail` et `machine-historique` ;
  `admin-design` est une exception légitime, c'est le guide. Le sous-ensemble
  dangereux — les partials partagés — est traité ; ce qui reste est du CSS de
  page, qui ne casse que sa page.

### S133 — la liste précise

- **Catégories de machines** est une vue dérivée des libellés saisis. Livrer un
  vrai CRUD (création, renommage, archivage avec impact explicite) **ou** retirer
  l'onglet. Ne jamais présenter une facette comme de la gestion.
- **Événements** : « Tous les lieux » agrège les lieux autorisés ; les
  événements sans lieu ont une option explicite « Ailleurs / externe »,
  jamais mêlés en silence.
- **Prêts** : chaque objet ouvre sa fiche canonique depuis les listes et
  réservations ; tester les objets archivés.
- **Packages** annonce Packages, Attributions, Quotas et Audit mais n'expose
  qu'une liste/édition : livrer les surfaces ou réduire ce qui est annoncé.

### Thèmes — le chantier entier (session dédiée, avant ou dans S134b)

- **Médiathèque d'identité** au lieu du champ texte `logoPath` : logo
  clair/sombre/compact, favicon, image de partage. Validés (MIME, dimensions,
  taille), renommés serveur, référencés par ID stable, supprimables seulement
  après contrôle des références. **Aucun chemin `public/images/…` libre.**
- **Éditeur guidé** : identité, variantes de logo, palette avec contrastes,
  rayon/typo/densité en presets. Pas de réglages épars.
- **Workflow** brouillon → aperçu → publication → retour arrière. L'aperçu rend
  de **vraies** surfaces (accueil, catalogue, détail, admin, **un kiosk**),
  desktop/mobile, clair/sombre. Publication atomique réglages **et** assets.
- **Kiosks** consomment le thème publié. Aucun `images/favicon.png`, logo ou
  couleur statique ne survit dans un kiosk.
- **Navigation & accueil** : ordre et visibilité par drag-and-drop accessible,
  destinations limitées aux routes/pages publiées autorisées, entrées système
  protégées. Page d'accueil au choix ; une page dépubliée rétablit l'accueil
  FabOS avec audit, sans page blanche ni boucle.

## Phase G2 — le produit honnête

Ajoutée le 2026-08-11 : on ne peut pas vendre du temps machine contre un modèle
d'horaires incapable d'exprimer un jour férié. Ces manques concernent **toutes**
les installations, tous les jours ; le commerce est facultatif.

| Session | Attendu | Qui |
|---|---|---|
| **S134c2** | **FabOS cesse d'inventer le contenu d'une formation** — fiche ci-dessous | Luna + opérateur |
| **S134d** | **une seule vérité horaire (modèle)** : un `ScheduleResolver` répond « X est-il ouvert à T » pour l'admin, les deux calendriers, les cartes, les kiosks et l'API ; plusieurs plages par jour, portée attachable, exceptions datées — **livrés ensemble** | Terra |
| **S134e** | **une seule vérité horaire (surfaces)** : Lieux édite plages, portées et exceptions avec aperçu ; public, calendriers et kiosks affichent la fermeture **avec sa raison** | Luna + Terra |
| **S134f** | archiver plutôt que supprimer partout où une suppression dure subsiste (événement, espace, matériau, objet, institution, page, création, lecteur) ; tout archivage d'une ressource réservable annule explicitement ses réservations à venir | Terra + Luna |

### Horaires — ce que le modèle ne peut pas exprimer (constat S131)

`UNIQ_OPENING_HOUR_VENUE_DAY (venueId, dayOfWeek)` existe depuis S106 et l'écran
sait enfin l'utiliser. Trois limites restent, par ordre de coût :

1. **Une seule plage par jour et par lieu.** Fermeture méridienne, service
   du soir, créneau personnel : inexprimables. ⚠️ **Contract au sens migration** :
   supprimer l'unicité oblige à reprendre **avant** tout code qui suppose « une
   ligne = un jour » (`ensureOpeningHourRows`, `OpeningHoursProvider`, les deux
   calendriers).
2. **Aucune granularité sous le lieu.** Cible : un horaire **rattachable** —
   lieu par défaut, surchargeable par workspace puis par ressource — avec
   héritage et une seule réponse effective par instant. ⚠️ **Ne pas dupliquer la
   table par type de ressource** : c'est la même question à des portées
   différentes.
3. **Aucune exception datée.** Jours fériés, fermeture annuelle. Se livre avec
   la portée ou la réécrit.

⚠️ **Prérequis commun** : la résolution effective doit être **un** service que
lisent l'admin, les deux calendriers, les cartes (« labo fermé » avant
« occupée », cf. S59) et les kiosks. Aujourd'hui chacun interroge la table ;
tant que ce service n'existe pas, ajouter une portée multiplie les endroits qui
peuvent se contredire.

### S134c2 — FabOS invente le contenu d'une formation

Quand les champs sont vides, la page publique sert un **programme horaire en
quatre points**, **trois sessions à venir** (« Mardi prochain 14:00-16:30 ·
Places disponibles »), **trois objectifs**, **deux prérequis** et **trois
éléments de matériel**. Rien n'existe : ce sont des exemples de démonstration
servis à un membre comme s'ils décrivaient sa formation — créneaux auxquels il ne
peut pas s'inscrire compris.

**Règle (opérateur) : on traduit l'interface, jamais le contenu. Un contenu que
l'opérateur n'a pas écrit ne doit pas exister.** Un champ vide n'affiche pas de
bloc ; l'écran d'ÉDITION peut proposer ces exemples comme point de départ, la
page publique jamais comme un fait.

⚠️ Les titres de section et les cartes d'explication sont l'interface, sont des
clés depuis S134c, et **restent**. Seules les **valeurs** sont en cause. Fichiers:
`FormationPageContentService::DEFAULTS` (blocs `program`, `sessions`) et les trois
`{% set %}` d'`objectives`/`prerequisites`/`material` dans
`formation-detail.html.twig`.

## Petits restes datés

- **`/events` sans paramètre rend 0 carte** quand il n'y a aucun événement à
  venir, alors que « Tous » en compte 3. Le défaut « À venir » est délibéré ; ce
  qui manque est un état vide qui renvoie vers les événements passés.
- **`/admin/homepage` porte six colonnes** (bloc + quatre audiences + ordre).
  C'est une matrice d'audiences, pas une liste d'enregistrements : le plafond de
  cinq ne lui répond peut-être pas. Non tranché.
- ✅ **« Sous-lieu » est devenu « lieu » — *location* en anglais (S143).**
  Renommage de catalogue fait dans les cinq langues : FR lieu, EN location, DE
  Standort, ES ubicación, IT sede. `Venue`/`VENUE`, `venue_context`, `?location=`
  et les noms de routes n'ont pas bougé — c'est un renommage de mots, pas de
  schéma. ✅ **Collision tranchée : on garde** (opérateur, 2026-08-16 —
  *« laissons comme cela pour l'instant, je n'ai pas mieux »*). La section de
  menu s'appelait déjà « Lieux », donc le mot paraît quatre fois sur
  `/admin/lieux` : barre latérale, libellé du sous-menu, un de ses deux liens, et
  le titre de la carte. C'est la forme `/admin/machines` — une section dont la
  page d'atterrissage porte son nom — avec le même mot des deux côtés. ⚠️ **Ne
  pas « corriger » ça dans une session future** sans le redemander : les trois
  sorties (laisser, renommer la section, distinguer l'entrée) ont été posées et
  aucune n'était meilleure.

## Phase H — commerce facultatif (S150–S154)

🔴 Renumérotée depuis S135–S139 le 2026-08-16 : ces numéros avaient déjà été
livrés comme sessions d'interface. Prochain numéro libre après S141 : **S142**.

Entièrement désactivable. Offres dans leur workspace métier, moteur commun pour
commandes/paiements/rapprochement. **Le retour navigateur ne confirme jamais un
paiement** — seul un webhook vérifié ou sa réconciliation. Clé unique par
événement fournisseur ; fulfillment persistant/outbox par ligne pour un effet
**exactement une fois** malgré retries et crashs. La livraison passe par le
service métier normal, sans toucher voter, badge, quota ni réservation.

| Session | Livre |
|---|---|
| **S150** | catalogue d'offres et prix ; aucune transaction |
| **S151** | commandes, paiements, webhooks, réconciliation, remboursements, audit |
| **S152** | livraison packages et matériaux ; hold stock atomique ou backorder explicite |
| **S153** | ledger append-only des crédits de temps et achats de formation |
| **S154** | reporting commerce, rapprochement, audit UX |

## Phase I — messagerie Formation (S155–S157)

Très loin après le workspace Formation. FabOS est la source de vérité ; l'e-mail
est une copie et une panne d'envoi ne perd jamais le message interne. Trois
visibilités : annonce formateur→cohorte sans exposer la liste, fil privé, groupe
explicite. Une réponse à une annonce est privée par défaut ; **aucun message
privé ne bascule implicitement vers la cohorte.**

| Session | Livre |
|---|---|
| **S155** | conversations, participants, non-lus, permissions |
| **S156** | interface formateur/étudiant + duplication e-mail asynchrone |
| **S157** | modération, archivage, export, rétention |

## Travaux transversaux conservés

Après les fondations dont ils dépendent : sécurité restante de Phase H
(**test réel du booking**, requêtes groupées) · verrou d'annulation et no-show
sur ressources qui ont un signal · files d'attente, stockage/retrait, motif
d'utilisation · exceptions d'horaires dans Lieux (voir S134d/e) · audit et notes
sur toute action Manage exercée sur autrui.

RFID physique et 2FA restent **hors scope**. Ordre : Phase G, puis G2, puis
commerce, puis messagerie. La réservation d'un pool de machines n'est pas
impliquée par les catégories.
