## S132–S134 — les quatre écrans de Configuration, la parité, et les droits en ombre (2026-08-16)

**S132 — ce qu'un interrupteur coûte, mesuré.** `/admin/fonctionnalites`
décrivait en prose l'effet de chaque désactivation, dans un catalogue tenu à côté
de celui qui garde les routes — donc libre de diverger, et il avait déjà divergé :
la note de `SiteFeatureRegistry` sur `machines` enregistre que couper les badges
ne rouvre PAS l'équipement, ce que le plan supposait jusqu'à ce que quelqu'un
lise la source. `SiteFeatureService::simulate()` rejoue l'état des fonctionnalités
le temps d'un appel (override dans le cache résolu, restauré dans un `finally`) ;
`FeatureSurfaces` construit `NavBuilder::admin()` et `header()` deux fois — tout
allumé, puis une seule chose éteinte — et la différence est ce que la carte
imprime. Mesuré sur l'installation : couper `machines` coûte huit entrées d'admin,
dont Logs RFID, Lecteurs RFID et Reporting. Personne n'aurait écrit cette liste à
la main. Les trois sections Ressources/Activités/Annuaires — une taxonomie qui
n'existe nulle part ailleurs — sont supprimées ; l'ordre est celui des workspaces.

`RouteAccessChecker` gagne un mémo : la page interroge toute la navigation quinze
fois, et chaque manque construisait un `Request` jetable pour interroger
`access_control`.

**Réglages : une correction déguisée en mise en forme.** Le formulaire de dix
rubriques devient cinq cartes courtes, ancrées, chacune avec son résumé d'état et
son Enregistrer. 🔴 Le vrai défaut était invisible : **toutes** les écritures du
contrôleur étaient à l'intérieur de `if (array_key_exists($locale, $availableLocales))`,
donc un POST portant une langue non reconnue jetait en silence le fuseau, le
vocabulaire, le règlement, les rôles de confidentialité et le bandeau — et
n'affichait que « Langue invalide. ». Chaque carte poste sa `section` ; rien en
dehors n'est lu ni écrit.

**E-mails et Logs RFID : le même défaut, deux fois.** Les deux rendaient les N
lignes les plus récentes et filtraient dans le navigateur, donc la cinquantième
(ou la centième) était l'horizon : sur une porte badgée cinquante fois par jour,
« Camille a-t-elle été refusée mardi ? » n'avait pas de réponse, et la rangée de
tuiles posée sur ces cent lignes faisait croire le contraire. Filtres serveur des
deux côtés. `_admin_filters` gagne `text_fields` — la liste de toutes les adresses
jamais écrites n'est pas un menu — et les Logs RFID retrouvent leurs trois champs
repliés (motif, message, jeton) dans un `<details>`, cinquième colonne.

✅ `/admin/rfid-readers` n'a plus **aucun** CSS local : sa dernière règle,
`.token`, ne stylait rien depuis S135 (grep : zéro élément portant la classe). Le
roadmap disait « une référence à extraire, pas à recopier » ; ce qui valait la
peine est dans `admin.css` et montré avec le vrai composant dans
`/admin/design#reglages`, et le reste était mort. ⚠️ `.color-dot` monte dans
`components.css` : `_rfid_result` est un partial **partagé** et ses deux appelants
en portaient chacun une copie — le piège qui a déjà coûté trois fois.

**S133 — trois affordances mortes et une facette déguisée en gestion.**
Les catégories de machines étaient un `GROUP BY` sur `MACHINE.categoryLabel`
présenté sous un titre de gestion : renommer une catégorie voulait dire rouvrir
chaque machine et retaper le mot, et une faute de frappe créait une seconde
catégorie en silence. `MACHINE_CATEGORY` arrive en **expand pur** — `categoryLabel`
ne bouge pas et reste la clé de jointure, parce qu'un `MACHINE.categoryId` serait
un contract touchant chaque gabarit, formulaire, filtre et export. Renommer =
deux écritures dans une transaction ; renommer vers un nom existant est une
fusion et le dit ; archiver ne touche aucune machine et le dit aussi. Chaque
contrôle annonce son impact avec le compte de **toute l'installation** — le
compte du lieu affiché sous-estimerait ce que le bouton fait. ⚠️ Le champ du
formulaire machine reste libre avec un `datalist` : un `ChoiceType` rendrait une
machine insauvable le jour où sa catégorie est archivée.

`VenueContext::forRequest()` a une **troisième réponse**, en opt-in : un événement
peut avoir lieu là où cette installation ne tourne pas. ⚠️ `['venue' => null]` est
un critère Doctrine réel et n'est pas `[]` — les confondre est exactement ce qui
rendait ces lignes invisibles en tant qu'ensemble. Et la colonne « Lieu » affichait
`event.lieu`, l'adresse libre : un événement externe et un interne sans adresse
rendaient le même tiret, sur la page dont le travail est de les distinguer.

🔴 **Chaque carte de `/prets` pointait sur `/prets`.** `/prets/{id}` est la fiche
canonique qui manquait ; les deux listes d'admin y mènent. ⚠️ Le nom de route
garde le préfixe `app_loans`, sinon `FeatureAccessSubscriber` ne la garde pas — le
piège que ses propres commentaires enregistrent deux fois. Et l'objet **s'archive**
au lieu de se supprimer : `remove()` emportait ses prêts, donc retirer une batterie
effaçait la trace de qui l'avait empruntée ; son propre message de succès le disait
et personne ne l'avait lu ainsi.

**S133b — les droits en ombre, et le mot « explicable » pris au sérieux.**
`USER_GROUP` / `USER_GROUP_MEMBER` avec les sept entrées intégrées ; `user` et
`guest` sont semés `virtual = 1` et n'auront jamais de ligne d'appartenance — un
provisioning qui l'oublie retirerait l'audience de base en silence. `USAGE_GRANT`
porte ce qu'une ligne feature ne peut pas dire : action (`use`/`manage`), lieu,
section. Backfill un-pour-un depuis `USAGE_PACKAGE_FEATURE`, donc **l'ombre
commence en accord exact avec le vivant** et tout écart ultérieur est une
modification délibérée. Un mur de « diffère » ne serait pas actionnable : chaque
ligne est classée, et les CHEMINS sont l'explication — « refusé » et « refusé
parce que le seul package qui le couvrirait est limité à l'autre lieu » sont deux
réponses, une seule est actionnable.

**S144 — le système de packages, fini : à qui, sur quoi, quand, et combien.**
L'opérateur, mot pour mot : *« finish the all package system. One package must be
able to allow users to 3D print on monday afternoon for exemple. Another one must
allow X hours machine reservations per week. »* Trois sessions dans une, chacune
livrée et commitée séparément.

**S144a — la colonne lue et écrite par rien.** `USAGE_RIGHT_ASSIGNMENT.groupId`
existait depuis S133b, `Version20260816140000` l'avait rempli, et
`UsageGrantRepository::paths()` le lisait sur les quatre chokepoints vivants
depuis S134 — pendant qu'aucun écran du produit ne pouvait créer une ligne.
Donner un package « aux formateurs » demandait un INSERT à la main. 🔴 **C'est
exactement la faute du paramètre de lieu trouvée en S134b, une table plus loin** :
une dimension stockée, migrée, lue et inatteignable se lit comme une
fonctionnalité et se comporte comme une absence. Et `assignmentsForPackage()`
était un `INNER JOIN UTILISATEUR`, donc les lignes de groupe que la migration
avait écrites étaient invisibles sur le seul écran qui peut les révoquer — un
droit qu'on ne voit pas est un droit qu'on ne reprend pas. ⚠️ `guest` est refusé
**et** retiré du sélecteur : `verdict()` ne lit les grants que pour un membre
connecté, donc l'attribuer ne pourrait jamais rien accorder. 🔴
`v2ShadowVerdict()` supprimé — sans appelant, et dernier lecteur de
`USAGE_PACKAGE_GROUP_ASSIGNMENT` ; il joignait `UTILISATEUR_ROLE`, donc il aurait
été en désaccord avec le lecteur vivant sur **tout** groupe créé depuis S133b.

**S144b — « sur quoi » et « quand ».** Trois colonnes de portée sur
`USAGE_PACKAGE_GRANT` et une table `USAGE_GRANT_WINDOW` (jour ISO + minutes
depuis minuit local). 🔴 **Couverture, pas chevauchement** : « lundi 14:00–18:00 »
REFUSE une réservation de 17:00 à 22:00. Un test de chevauchement aurait vendu de
l'accès complet avec des étapes en plus, et c'est le genre de faute qu'un labo ne
découvre qu'en la vendant. `GrantWindowSet` fait l'union des créneaux d'un jour —
deux tranches 09:00–12:00 et 12:00–14:00 couvrent une réservation de 11:00 à
13:00 que ni l'une ni l'autre ne contient — puis marche la réservation jour par
jour, parce qu'une réservation qui passe minuit pose **deux** questions et que
lundi ne dit rien de mardi matin. ⚠️ Minuit **en fin** de journée vaut 1440 et non
0, et « 00:00 » saisi comme FIN est réinterprété : lu comme la minute zéro, un
labo ouvert jusqu'à minuit ne pourrait jamais vendre une soirée. ⚠️ La catégorie
est un **libellé** et pas une clé étrangère — `MACHINE.categoryLabel` est la clé
de jointure gardée en S133, donc une quatrième imprimante entre dans le package en
entrant dans la catégorie, sans rééditer ce qui a été vendu. ⚠️ Les cinq
nullables positionnels de `verdict()` sont devenus un `UsageScope` : c'est
précisément le refactor pendant lequel une dimension cesse silencieusement d'être
passée, donc `VenueScopedGrantTest` suit désormais le lieu de bout en bout.
🔴 `.usage-assignment .admin-action` peignait **tous** les boutons de la ligne aux
couleurs de suppression ; la ligne porte maintenant un éditeur de créneaux dont
trois boutons sur quatre ne détruisent rien.

**S144c — « combien ».** `USAGE_PACKAGE_ALLOWANCE` : quantité, unité (heures
réservées ou nombre de réservations), période (jour/semaine/mois/**total**, ce
dernier étant l'absence de période — un bloc prépayé se dépense une fois).
⚠️ **Aucune allocation = aucun plafond**, jamais un plafond de zéro : c'est ce qui
rend la migration sûre en pleine semaine. ⚠️ Les allocations **s'additionnent**
(règle « packages cumulatifs » de la feuille de route) : prendre le maximum
ferait qu'acheter un second package ne donne rien, prendre le minimum ferait
qu'il en retire. Mais 5 h/semaine et 20 h/mois sont **deux** budgets qui doivent
tenir tous les deux. ⚠️ **Un pass d'accès ne lève pas une allocation** : il
exempte des quotas du labo, pas des heures achetées. ⚠️ Le refus porte les trois
chiffres, parce que « limite atteinte » laisse un membre sans rien à faire de
l'information. ⚠️ Semaine à partir du **lundi**, la même que `maxPerWeek` et que
les horaires : deux frontières de semaine dans un même chemin de réservation est
une faute qui ne se voit qu'un dimanche, à une personne, une fois.

⚠️ **Le code précède les deux migrations, et c'est le dessin.**
`UsageGrantRepository::paths()` est autoritaire sur quatre chokepoints et son
catch rend une liste vide — c'est-à-dire un refus pour tout le monde. Nommer une
colonne non migrée aurait donc retiré la réservation à tout le labo entre le
déploiement et la migration, en silence. `UsageGrantSchema` sonde avant de
nommer, et échoue vers l'ANCIEN comportement. C'est la leçon de
`LOANABLE_ITEM.archivedAt` (S133), en plus cher.

**S134e — la fermeture dit pourquoi.** La raison était calculée depuis S134d et
n'atteignait que le message de refus d'une réservation : `closureReasonFor()`
n'avait aucun appelant dans un gabarit ni un contrôleur. Un membre devant le
calendrier un jour férié lisait « fermé » et devait deviner si le labo était
fermé, en panne, ou s'il avait mal lu. ⚠️ **Même forme que les trois fautes de la
semaine, une couche plus haut** : tout était stocké, rien n'était câblé à un
écran. Catalogues, deux calendriers et kiosques la portent désormais ; côté JS
une exception datée répond AVANT le jour de semaine, exactement comme
`openIntervalsFor()` côté serveur — garder les deux règles identiques est ce qui
fait que le calendrier et la porte sont d'accord sur un jour férié.

🔴 **Et une faute de S134d trouvée en passant.** Sur `/places`, `$venueOpenNow`
était calculé AVANT l'affectation de `$venueContext` : variable indéfinie, filtre
de lieu ignoré, page répondant pour le lieu par défaut. **Prod tourne sans
`strict_variables`**, donc rien n'a bronché pendant deux sessions — c'est un
warning dans un auto-test qui l'a fait sortir, pas une assertion. Un test compare
maintenant les positions des deux lignes dans le fichier.

⚠️ **Deux de mes assertions ont échoué à tort avant de passer** : `json_encode`
échappe le non-ASCII, donc la page porte `Jour f\u00e9ri\u00e9` et jamais le
littéral accentué. Comparer une page à la chaîne brute est un test qui ne peut
que échouer — c'était le test qui avait tort, pas le produit, et le probe l'a
montré en une ligne.

**S134d (suite) — un horaire s'attache sous le lieu.** Deux colonnes nullables
sur la table qui répond déjà à la question, pas une table par type de ressource :
`scopeType IS NULL` est la semaine du lieu, `('machine', NULL)` toutes les
machines, `('machine', 12)` cette machine. ⚠️ Le niveau intermédiaire est le TYPE
de ressource et non un « workspace » — ce qui fait varier un horaire, c'est ce
qu'on réserve, et `ReservableType` nomme déjà cette idée.

🔴 **Les niveaux s'INTERSECTENT** (décision opérateur, 2026-08-19) : une ressource
ne peut que RESTREINDRE son lieu. Personne n'utilise la découpeuse quand le
bâtiment est fermé, et c'est la seule composition où aucun niveau ne peut échouer
en s'ouvrant. ⚠️ Son prix — des heures plus larges que le lieu ne font rien — est
payé par l'écran : une colonne « Effectif » résout jour par jour et dit « sans
effet » ou « le lieu est fermé ». Refuser la saisie aurait été plus simple ;
montrer la résolution est ce qui empêche la quatrième « heures écrites qui ne
font rien ».

🔴 **Et l'ordre de déploiement, que j'ai eu faux.** `scopeType` est une colonne
MAPPÉE : toute requête ORM sur `OpeningHour` la sélectionne, donc aucun try/catch
de dépôt ne dégrade — contrairement aux sondes DBAL de S144 et des plages.
Déployée avant sa migration, elle a mis un 500 sur `/admin/horaires` ; conteneur
remis à l'état précédent dans la minute, site revérifié 200, puis redéployé après
la migration. ⚠️ La règle était déjà écrite dans le brief : **colonne ORM mappée
⇒ migration d'abord ; fonctionnalité DBAL fail-safe ⇒ le code peut partir
devant.** Deux branches, et j'ai pris la mauvaise parce que les deux sessions
précédentes relevaient de l'autre.

Écritures vérifiées : 20/20, transaction annulée. Notamment — samedi 06:00–23:00
écrit sur une machine sous un lieu 08:00–20:00 se résout en 08:00–20:00 et 07:00
est refusé ; un lieu fermé ferme tout ce qui est dessous ; un niveau vide laisse
répondre celui du dessus ; et UN seul niveau répond, la machine gardant ses
10:00–16:00 après l'écriture d'un niveau « toutes les machines ».

**S134d — la semaine sait enfin dire « fermé entre midi et deux », et une date
peut la réécrire.** `UNIQ_OPENING_HOUR_VENUE_DAY` tombe (une détente : l'ancien
code n'écrivait simplement jamais de seconde ligne, donc l'ordre déploiement /
migration est indifférent) et `SCHEDULE_EXCEPTION` porte les fermetures datées
avec leur raison — « fermé » laisse deviner, « fermé — jour férié » répond, et le
message de refus la reprend.

🔴 **Couverture par UNE plage, jamais l'enveloppe.** 11:00–15:00 est à l'intérieur
de 09:00–18:00 et reste une heure de réservation sur un labo fermé. Tester
l'enveloppe aurait rendu la fonctionnalité décorative dès le premier usage. Même
raisonnement en aval : le proposeur de créneaux boucle sur chaque plage (sinon il
invite à un créneau que la porte refuse), les disponibilités d'une personne sont
croisées avec chaque plage, et les deux calendriers — qui construisaient
`hours[row.dayIndex] = …`, donc écrasaient silencieusement la seconde plage —
reçoivent une entrée par JOUR avec ses `ranges`, l'enveloppe ne servant plus qu'à
la mise en page.

🔴 **`count($rows) === 7` était devenu un piège, et il était à DEUX endroits.**
Un lieu avec une pause le mardi a huit lignes ; le test le prenait pour une semaine
incomplète et servait la semaine intégrée — la fonctionnalité s'annulant elle-même
au premier usage. Corrigé dans `rowsFor()`… et manqué dans `defaultVenueRows()`,
où c'était pire : tout appelant sans lieu (accueil, calendrier agrégé, API,
amorçage d'un nouveau lieu, disponibilité de chaque réservation de personne)
recevait 08:00–20:00 en dur. ⚠️ **Trouvé par une ligne de log au milieu d'un
auto-test dont les 29 assertions passaient** — elles interrogeaient le lieu par
défaut par son id et prenaient l'autre branche. Une assertion verte ne dit rien
de la branche qu'elle n'emprunte pas.

⚠️ **L'écran est livré avec le modèle**, et c'est délibéré : après trois
« colonne écrite par rien » dans la même semaine (le lieu d'un grant, `groupId`,
le lieu des horaires), livrer un schéma que personne ne peut atteindre serait la
quatrième. `/admin/horaires` édite N plages par jour — ligne vide pour supprimer,
remplacement transactionnel de la journée, chevauchement **refusé** plutôt que
fusionné, parce qu'un écran qui renvoie une semaine qu'on n'a pas tapée est un
écran auquel on cesse de croire.

Écritures vérifiées sur la vraie base par de vrais POST passés dans le noyau
(routage, pare-feu, CSRF, contrôleur), le tout dans une transaction annulée :
29/29, 7 → 7 lignes, 0 → 0 exception.

**S145a — les horaires du bon lieu, et la troisième fois que la même faute
arrive.** `OpeningHoursProvider` résolvait la semaine en appelant
`VenueRepository::findDefault()` — le lieu dont le slug est littéralement
`default` — et `ReservationService` validait chaque réservation à travers lui sans
passer de lieu. Sur une installation multi-lieux, une machine du second lieu était
contrôlée contre les horaires du premier ; sans lieu `default`, tout était
contrôlé contre sept lignes en dur.

🔴 **C'est la troisième occurrence de la même forme.** Un grant limité à un lieu
n'a rien fait pendant deux sessions (S134b) ; `USAGE_RIGHT_ASSIGNMENT.groupId`
était lu et écrit par rien pendant deux autres (S144a). À chaque fois le schéma
portait la dimension, l'écran l'écrivait, et l'APPELANT ne la fournissait pas.
La règle qui en sort et que `ScheduleResolverWiringTest` épingle : **quand une
table porte `venueId`, la question n'est pas « la requête joint-elle la colonne »
mais « l'appelant passe-t-il une valeur ».**

⚠️ L'échelle de repli préserve exactement le comportement précédent — un lieu sans
lignes retombe sur celles du lieu par défaut, ce que tous recevaient déjà — donc
le correctif ne peut que rendre une réponse plus juste, jamais couper un labo.
⚠️ Et `null` reste **contraint**, à l'inverse de la règle des grants : une
permission inévaluable ne doit pas refuser, une restriction inévaluable ne doit
pas autoriser.

🔴 **Trouvé par le balayage de routes, pas par les tests.** Un renommage en bloc
avait renommé le paramètre de `ensureOpeningHourRows()` sans renommer son
appelant : `/admin/horaires` rendait un `TypeError`, alors que `php -l`, la
compilation du conteneur et 98 tests étaient verts. Deuxième leçon de la journée
sur ce que les vérifications statiques ne voient pas — après la méthode privée
`run()` du harnais de S144.

**S144 vérifié en écriture (2026-08-19).** Les GET ne prouvent rien d'un modèle
dont tout l'intérêt est ce qu'il écrit. Une commande **jetable** — poussée dans le
conteneur, exécutée, supprimée, jamais commitée — a exercé chaque chemin d'écriture
sur la vraie base **dans une transaction annulée à la fin**, comptage avant/après
à l'appui : 29 vérifications, zéro échec, zéro ligne survivante.

Ce qui est désormais prouvé plutôt qu'argumenté, dans les mots de la demande :
« imprimer en 3D le lundi après-midi » ⇒ **lundi 15:00 autorisé, lundi 19:00
refusé, mardi 15:00 refusé** ; « X heures par semaine » ⇒ une réservation de 5 h
contre un budget de 2 h/semaine **refusée de 180 minutes**, une de 1 h passe.
Et les propriétés qui n'étaient jusque-là que des commentaires : un aperçu sans
intervalle n'est **pas** refusé par un créneau ; un grant limité aux machines ne
dit **rien** des espaces ; deux allocations se **somment** (120 + 120 = 240) ;
retirer le dernier créneau **rend** l'accès au mardi ; une suppression adressée au
mauvais package ne fait rien ; `guest` est refusé ; une allocation de zéro est
refusée. L'attribution de groupe écrit bien `groupId` avec `userId` NULL, apparaît
dans la liste et se révoque — les trois moitiés de la faute de S144a.

🔴 **Et le harnais a lui-même trouvé une leçon.** Il a d'abord planté au chargement
de la classe : une méthode privée `run()` qui écrasait `Command::run()`, publique.
⚠️ **`php -l` était passé sans rien dire** — il vérifie la syntaxe d'un fichier,
pas la cohérence d'une hiérarchie. C'est le pendant exact de la leçon de S134g
(« vérifier qu'un setter existe n'est pas vérifier qu'il accepte ») et du fait déjà
noté que `lint:twig`/`lint:yaml` ne lisent pas `src/`. Le seul filet qui l'attrape
est de **charger** la classe : `cache:clear` ou l'exécution.

**S134 — le mécanisme, son garde-fou, et deux fautes trouvées en le construisant.**
`usage_rights_v2_<capacité>` : un interrupteur par chokepoint, tous à false,
subordonné à l'enforcement. Clé inconnue ⇒ **false**, l'inverse délibéré de
`SiteFeatureService::isEnabled()`. Activer est refusé tant qu'un membre y perdrait
l'accès, compté sur **tous** les comptes ; revenir en arrière n'est jamais refusé.

🔴 **La comparaison de l'ombre devait être contrefactuelle.** La première version
comparait à `$live->allowed`, qui vaut `true` pour tout le monde aujourd'hui avec
la raison `not_enforced` — donc la page annonçait zéro personne en risque,
exactement sur les installations que le garde-fou protège. `legacyPackages()`
répond à la vraie question.

🔴 **Et la migration S133 ne se lançait pas.** Son `ALTER TABLE LOANABLE_ITEM`
était après l'accolade fermante de `up()` : ParseError, et
`doctrine:migrations:migrate` refusait de démarrer pour **toutes** les migrations.
Trouvé par l'opérateur en lançant la commande. ⚠️ **`lint:twig` et `lint:yaml` ne
lisent pas `migrations/` ni `src/`** — un fichier PHP nouvellement écrit n'est
vérifié par rien avant d'être exécuté. Le réflexe manquant est
`find src migrations -name "*.php" -exec php -l {} \;` avant le redémarrage.

⚠️ **Ce que S134 n'a pas fait, et pourquoi c'est correct.** Aucun chokepoint n'est
basculé et le package legacy n'est pas retiré : il n'existe aujourd'hui aucun
grant v2 au-delà du backfill, donc basculer refuserait tout membre sans package.
Ce qui reste est de la donnée, plus deux surfaces d'écriture — l'éditeur de grants
v2 et l'attribution à un groupe.

**Phase G — multi-lieux et navigation (S129–S132b, partielle).** S129 workspace
Lieux opérable · S130 navigation dédoublonnée · S130b **une seule**
sous-navigation (les onglets supprimés, 6 écrans repris, 52 libellés aux
catalogues) · S130c sous-lieu en filtre un-clic · S130d Réseau dans
Configuration · S130e propositions de filtres, **closes** — le format a été
tranché en S130e→S140→S141 · S131 contexte sous-lieu uniforme · S132b réparation
`venueContext`/`venue_context`.

**Phase G2 — le produit honnête (partielle).** S134c le back-office parle cinq
langues · S134g le compte appartient au membre (mot de passe oublié, puis
anonymisation irréversible).

**Phase G3 + interface (S134h–S141).** S134h–S134j format de liste unique +
vocabulaire de colonnes `_cell_*` · S135 le même objet partout (sept familles de
pastilles fusionnées) · S136 bandeau compact · S137/S138a-b grille de cartes
publique + `frame: 'full'` · S139a–e la recherche couvre dix catalogues, gagne
des destinations, **44 routes legacy supprimées** · S140 la carte fusionnée sur
`/admin/machines` · **S141 la carte fusionnée devient LE format**, six étapes,
récit complet ci-dessus · **S142 une seule barre latérale** (la variante
`'edit'` retirée de 27 formulaires) et le CSS des partials partagés remonté —
1 118 règles locales → 950 · **S138c** le cadre est la seule forme du catalogue · **S142c/d** la carte des listes devient la forme de TOUTES les pages d'admin, et les 24 px de mauvais fond sous l'en-tête disparaissent (`flow-root`) · **S143** « sous-lieu » devient « lieu » dans les cinq langues, et le dernier bandeau pleine largeur disparaît avec les cinq familles CSS qui le dessinaient.

---


# S132, S133 et S144 — les récits, déplacés de ROADMAP le 2026-08-21

### 🔴 S144 — le système de packages, fini (2026-08-17)

Demande opérateur, mot pour mot : *« finish the all package system. One package
must be able to allow users to 3D print on monday afternoon for exemple. Another
one must allow X hours machine reservations per week. »*

✅ **Les deux migrations sont passées le 2026-08-19** (`Version20260817100000`,
`Version20260817110000`). ⚠️ **Le repli reste dans le code et doit y rester** :
`UsageGrantSchema` et `UsageAllowanceRepository::tableExists()` sondent avant de
nommer une colonne et échouent vers l'ANCIEN comportement. C'est ce qui a permis
de déployer le code AVANT les migrations sans retirer la réservation au labo —
vérifié à l'écran ce jour-là, pas supposé — et c'est ce qui protégera la
prochaine installation qui restaure une base plus vieille que son code.

Ce qu'un package sait dire maintenant :

| Dimension | Où | Depuis |
|---|---|---|
| fonctionnalité, action (use/manage), section | `USAGE_PACKAGE_GRANT` | S111/S133b |
| lieu | `USAGE_PACKAGE_GRANT.venueId` | S134b |
| **ressource** : un type, une machine précise, une catégorie | `reservableType` / `reservableId` / `categoryLabel` | **S144b** |
| **créneau hebdomadaire** : « lundi 14:00–18:00 » | `USAGE_GRANT_WINDOW` | **S144b** |
| **combien** : X heures ou X réservations par jour/semaine/mois/total | `USAGE_PACKAGE_ALLOWANCE` | **S144c** |
| à qui : un membre **ou un groupe** | `USAGE_RIGHT_ASSIGNMENT.userId` / `.groupId` | S111 / **S144a** |

🔴 **Couverture, pas chevauchement.** « Lundi 14:00–18:00 » REFUSE une
réservation de 17:00 à 22:00. Un test de chevauchement aurait vendu de l'accès
complet avec des étapes en plus. `GrantWindowSet` fait l'union des créneaux d'un
jour puis marche la réservation jour par jour ; minuit **en fin** de journée vaut
1440 et non 0.

⚠️ **Les créneaux ne sont évalués que si l'appelant donne un intervalle.** Une
réservation en donne un ; un aperçu (« ce membre a-t-il ce droit ? ») n'en donne
pas et n'est donc jamais refusé « parce qu'on est mardi ». La page des droits
montre les créneaux en toutes lettres à la place.

⚠️ **Aucune allocation = aucun plafond**, jamais un plafond de zéro. Les
allocations de plusieurs packages **s'additionnent** (5 h + 10 h = 15 h) mais
5 h/semaine et 20 h/mois sont deux budgets qui doivent tenir tous les deux.
**Un pass d'accès ne lève PAS une allocation** : il exempte des quotas du labo,
pas des heures achetées.

⚠️ **Ajouter une allocation est la seule écriture de cet écran qui RESTREINT**, et
en retirer une rend des heures. Les deux confirmations le disent dans ce sens.

**Ce que S144 n'a PAS fait, volontairement :**
- Aucun paiement, aucun prix, aucune commande — c'est la Phase H (S150–S154), et
  elle reste facultative. S144 livre l'**entitlement** qui rend un package
  vendable, pas le commerce.
- Pas de `categoryLabel` sur les allocations, alors que les grants en ont un :
  rien ne l'appliquerait, et une colonne que seul un formulaire écrit est
  exactement la faute que S144a a réparée sur `groupId`.
- `readiness()` (préflight de `/admin/settings`) compte toujours
  `COUNT(DISTINCT a.userId)` : un package tenu **uniquement** par un groupe y
  compte pour 0 personne. Voir S144e ci-dessous.

### ✅ S132 — livré

- **Fonctionnalités** ne décrit plus l'effet d'une désactivation, elle le
  **mesure** : `SiteFeatureService::simulate()` rejoue l'état des fonctionnalités
  le temps d'un appel, `FeatureSurfaces` construit la navigation deux fois et la
  différence est la réponse. Ordre = ordre des workspaces ; les trois sections
  Ressources/Activités/Annuaires sont supprimées. ⚠️ **Ne pas réintroduire une
  description en prose de ces effets** — la précédente avait déjà tort.
- **Réglages** : cinq cartes ancrées, résumé d'état, sauvegarde par section.
  🔴 Ce n'était pas de la mise en forme : toutes les écritures étaient à
  l'intérieur du contrôle de la langue, donc un POST avec une langue non reconnue
  jetait en silence le fuseau, le vocabulaire, le règlement, les rôles et le
  bandeau.
- **E-mails** : trois tâches, et le journal filtre **côté serveur**.
- **Logs RFID** : filtres serveur (période/lecteur/machine/résultat) et le détail
  à la demande dans un `<details>`, cinquième colonne.
- ✅ **`/admin/rfid-readers` n'a plus aucun CSS local.** Sa dernière règle,
  `.token`, ne stylait plus rien depuis S135. Les primitives extraites sont dans
  `/admin/design#reglages`.
- ⚠️ `.color-dot` est monté dans `components.css` : `_rfid_result` est un partial
  **partagé** et ses deux appelants en portaient chacun une copie.

### ✅ S133 — livré

- **Catégories de machines** est un vrai CRUD. `MACHINE_CATEGORY` en expand pur :
  `MACHINE.categoryLabel` ne bouge pas et reste la clé de jointure. Renommer vers
  un nom existant est une **fusion** et le dit ; archiver ne touche aucune machine
  et le dit aussi. ⚠️ Le champ du formulaire machine reste **libre** avec un
  `datalist` : un `ChoiceType` rendrait une machine insauvable le jour où sa
  catégorie est archivée.
- **Événements** : `VenueContext::forRequest()` a une troisième réponse en opt-in,
  « Ailleurs / externe ». ⚠️ `['venue' => null]` est un critère Doctrine réel et
  n'est pas `[]`.
- **Prêts** : `/prets/{id}` est la fiche canonique — chaque carte du catalogue
  pointait sur le catalogue. Et l'objet **s'archive** : `remove()` emportait ses
  prêts avec lui.
- **Packages** annonce désormais ce qu'il expose. Quotas vit déjà par type de
  ressource ; Attributions et Audit arrivent avec grants v2.

---

