# Phase K — les gabarits d'e-mail deviennent modifiables (S160–S162)

**2026-09-04 (planifiée) au 2026-09-07 (close).** Demandée par l'opérateur sur la
trouvaille du dépouillement Fabmanager : *« Customize email templates »*,
**10 votes**, et le seul écart à la fois bien voté, petit, et absent de notre plan.
Détail des écarts : `FABMANAGER-ECARTS.md`.

**Le résultat** : un exploitant réécrit le texte d'un e-mail, **par langue**, sans
écrire une ligne de Twig ; l'en-tête et le pied se réécrivent **une seule fois**
pour les vingt ; et 🔴 **une surcharge cassée n'empêche jamais un mot de passe
oublié de partir** — mesuré, avec l'incident journalisé et le journal qui dit
quelle version a servi.

---

## Ce qui existait déjà, mesuré avant d'écrire

- **23 gabarits Twig** dans `templates/emails/`, tous héritant de
  `_layout.html.twig`. Le sujet est un `{% block subject %}`.
- 🔴 **Le texte n'est PAS dans les gabarits : il est en CLÉS DE TRADUCTION**
  (`mail.event.registered.subject`), donc en cinq langues. Ce fait a commandé
  toute la conception.
- `Mailer::queue()` enregistre `template` + `context` + `locale` dans le journal ;
  **le rendu a lieu plus tard, à l'envoi**, par le worker.
- ✅ `sendNow()` avec `NotificationCategory::TEST` existait déjà — l'aperçu et le
  « m'envoyer un test » étaient donc à moitié construits.

## 🔴 Les trois tensions tranchées AVANT d'écrire une ligne

**1. Un texte modifié par l'opérateur est du CONTENU, pas de l'interface.** La
règle de la maison est explicite : *on traduit l'UI, jamais le contenu*. Donc une
surcharge est **par langue**, et les traductions livrées restent le repli. Il n'y
a pas de version « une seule langue » qui tienne : un lab bilingue qui ne
surcharge que le français casserait ses mails anglais s'il remplaçait la clé.

**2. 🔴 L'opérateur n'écrira JAMAIS de Twig.** Laisser saisir du Twig, c'est offrir
l'exécution de code arbitraire dans un gabarit. Deux issues seulement : le bac à
sable Twig, ou une syntaxe de champs restreinte (`{{ event }}`) validée à
l'enregistrement. **La seconde a été retenue** : elle est vérifiable, elle n'a pas
de surface d'évasion, et personne n'a demandé de boucles dans un e-mail.

**3. ⚠️ Le rendu a lieu à l'ENVOI, pas à la mise en file.** Un gabarit modifié
entre les deux change le mail déjà en attente. Retenu : **rendre à l'envoi** —
plus simple et cohérent avec l'existant — à la condition ferme qu'**une surcharge
cassée ne puisse jamais empêcher un mail transactionnel de partir**. C'est devenu
le critère de sortie de la phase.

---

## ✅ S160 — le repli prouvé dans son état le plus fort (2026-09-06)

**Livré** : le modèle, le repli, la substitution de champs. Pas d'éditeur.
**Mesuré** : ✅ **empreinte IDENTIQUE, octet à octet** — `70c853d3ba295630…` avant
comme après, sur **40 rendus** (20 gabarits × 2 langues). Sonde
`app:s160:mail-render-probe`.

🔴 **Le code a été déployé alors que la table n'existait PAS encore** — la
migration attendait l'opérateur. C'est la meilleure preuve possible du repli :
`MailOverrides` est en DBAL, sonde l'existence de la table une fois par processus,
et rend « aucune surcharge » sur n'importe quelle exception. Résultat mesuré : les
40 rendus sont identiques au bit près.
⚠️ C'est aussi la seule direction sûre : **une table NEUVE se déploie avant sa
migration ; une colonne sur une entité chargée partout, jamais**
([[feedback-fabos-migration-hazard]]).

🔴 **Le texte de l'exploitant ne passe JAMAIS par le compilateur Twig.** Il est
substitué en PHP sur une liste FERMÉE de champs (`{{ event }}`), échappé, puis
`nl2br`, puis injecté dans `_override.html.twig` qui n'apporte que le chrome du
layout. Aucune boucle, aucune condition, aucun filtre : personne n'a demandé de
`{% for %}` dans un e-mail, et **chaque construction acceptée serait une surface
d'évasion de plus**.

⚠️ **Un champ inconnu reste écrit tel quel, visible** — pas rendu vide. Une phrase
amputée ne se remarque pas ; `{{ nimportequoi }}` en clair, si.

⚠️ **Une surcharge VIDE n'est pas une surcharge** : elle vaut « rien à ajouter ».
Rendre une chaîne vide enverrait un mail sans objet ni corps — pire que pas de
fonctionnalité.

⚠️ **`locale` fait partie de la clé**, pour la raison donnée en tête : le texte
réécrit est du contenu.

---

## ✅ S161 — les champs sont DÉDUITS, pas retapés (2026-09-07)

**Livré** : l'éditeur. Un écran par gabarit ET par langue, les champs déduits du
gabarit, refus d'un champ inconnu, aperçu par le VRAI moteur.
**Mesuré** : sonde `app:s161:mail-editor-probe`, **12 assertions, aucun courrier
envoyé**.

**`MailTemplateCatalog` lit la source Twig du gabarit et de ses partiels.** Une
liste tenue à la main diverge du jour où quelqu'un ajoute une variable — et
personne ne s'en aperçoit, parce que l'éditeur continue de proposer l'ancienne.
⚠️ Le scan cherche DEUX formes, et **la seconde est le cas majoritaire ici** : le
texte de ces mails vit en clés de traduction, donc `|trans({'%event%': event})`.
Un scan qui ne verrait que `{{ … }}` raterait presque tout.
🅿️ **Ce qu'il ne voit pas, dit franchement** : un champ passé par un appelant PHP
sans jamais apparaître dans le Twig. La liste est donc « ce que le gabarit sait
afficher » — la bonne définition pour un éditeur, pas « tout le contexte ».

✅ **L'aperçu passe par `MailSender::render()`, la méthode qui ENVOIE** (rendue
publique pour ça). Un second moteur de rendu pour la prévisualisation finirait par
diverger de celui qui envoie — **exactement le défaut qu'un aperçu prévient**.
⚠️ Et l'écran DIT que le contexte est un exemple : les valeurs sont en capitales
(« ÉVÉNEMENT ») pour qu'on voie où elles atterrissent. Une valeur plausible ferait
croire qu'on regarde un vrai mail.
⚠️ L'aperçu est dans un `iframe sandbox` : un gabarit d'e-mail porte ses propres
styles en ligne, et sans isolation ses règles fuiraient dans la page d'admin et
inversement.

✅ **Vérifié par sonde, avec écriture puis retrait exact** : la surcharge s'applique
(objet ET corps), le champ est REMPLACÉ et pas écrit tel quel, le chrome du layout
est conservé, `{{ 7 * 7 }}` **n'est pas évalué**, `<script>` est échappé, vider les
deux champs SUPPRIME la ligne, et le rendu revient **identique au bit près**. Table
vérifiée vide avant et après — et **la sonde refuse de démarrer si elle ne l'est
pas**, plutôt que d'écraser le texte de quelqu'un.

🅿️ **L'envoi de test n'est pas livré, et c'est un choix.** `sendNow()` existe et
marcherait. Poser un bouton qui envoie du vrai courrier depuis une session
automatisée n'est pas une décision d'agent : ça s'ajoute quand quelqu'un peut le
regarder partir.

---

## ✅ S162 — le repli n'est plus silencieux, et le chrome se réécrit une fois (2026-09-07)

**Livré** : l'en-tête et le pied réécrivables SÉPARÉMENT, la garde du
transactionnel PROUVÉE, et le journal qui dit quelle version a servi.
**Mesuré** : sonde `app:s162:layout-probe`, **26 assertions, aucun courrier
envoyé**.

🔴 **La mesure de sortie, tenue** : une surcharge volontairement cassée sur
`password_reset` — un objet sur deux lignes, écrit en SQL direct — et le mail part
**avec le texte livré, identique au bit près**, l'objet cassé ne fuit pas dans
l'en-tête, et l'incident est journalisé (`ERROR`, avec le gabarit, la langue, la
partie et la raison).

⚠️ **Casser une surcharge demande de la MALICE, et c'est une bonne nouvelle.** Le
texte de l'exploitant ne voit jamais le compilateur Twig : il n'y a ni boucle, ni
condition, ni filtre à faire échouer. Le seul défaut réellement atteignable est un
objet contenant un saut de ligne — un en-tête SMTP mal formé — et il n'est
atteignable que par un POST fabriqué à la main, un navigateur retirant les retours
d'un `<input>`.
🅿️ **Ce n'était donc pas un défaut observé en production** : c'est une panne
fabriquée pour mettre le repli à l'épreuve. Le dire ainsi vaut mieux que de laisser
croire qu'on a réparé quelque chose de cassé.

🔴 **Le repli reste un FILET, il ne devient pas une porte d'entrée.** L'éditeur
refuse l'objet multi-ligne **avec une phrase, sur le champ concerné**, et `save()`
refuse en plus l'UTF-8 invalide. Absorber à l'envoi ce qu'on laisse entrer à
l'écriture signifierait qu'un exploitant voit son texte enregistré ici et le texte
livré dans sa boîte, **sans rien qui explique l'écart**.

✅ **`_header` et `_footer` — deux clés réservées, la même table.** Ce sont les deux
seuls morceaux de chrome communs aux vingt e-mails : les réécrire gabarit par
gabarit obligerait à saisir cent fois le même pied, et à le corriger cent fois.
Une seconde table pour deux lignes aurait été un second endroit où chercher « qui a
changé ce texte », et un second repli à écrire et à prouver.

🔴 **Le lien de désinscription reste émis par le layout, sous le texte du pied, et
`unsubscribe_url` n'est PAS proposé comme champ du pied.** L'exploitant réécrit la
phrase ; il ne déplace ni ne retire la sortie de secours. Un pied qui l'aurait
« déplacé » puis perdu supprimerait une obligation légale par inadvertance.

⚠️ **Les trois replis sont INDÉPENDANTS**, et la sonde le mesure d'un coup : un
corps cassé laisse l'en-tête et le pied réécrits en place
(`override_failed+header+footer`).

✅ **Le journal dit QUELLE VERSION a servi** — critère de sortie de la phase.
`EMAIL_LOG.renderedFrom` porte une **trace, pas un booléen** : le corps (livré,
réécrit, ou réécrit-mais-cassé) **et** les deux parties du chrome. « Réécrit » tout
court ne dirait pas que c'est le pied commun qui a changé le mail, alors que c'est
justement le cas qui touche vingt gabarits d'un coup.
⚠️ **`—` pour les mails partis avant la colonne** : on ne le sait pas
rétroactivement, et écrire « livré » par défaut serait une affirmation inventée.
🔴 **Et l'écran des textes marque en ROUGE les couples dont le dernier envoi est
retombé** — dérivé du journal, pas d'un drapeau stocké qu'il faudrait penser à
remettre à zéro quand quelqu'un répare son texte.

✅ **La sortie des 40 rendus est IDENTIQUE à celle d'avant le changement de
layout** — mesuré en remettant l'ancien `_layout.html.twig` sur la boîte, en vidant
le cache, en rendant les 40, puis en le remettant : `diff` vide. **C'est la seule
forme de preuve qui ne dépende ni de la date ni de l'état de la base.**

---

## Les critères de sortie, et comment ils sont tombés

- ✅ Aucune surcharge en base ⇒ aucun changement visible nulle part (S160, 40 rendus
  identiques au bit près).
- ✅ 🔴 **Un mot de passe oublié part toujours**, quelle que soit la bêtise saisie
  (S162, panne fabriquée, mail parti, incident journalisé).
- ✅ Une surcharge s'applique dans la langue du destinataire, et seulement là.
- ✅ Le journal des mails dit **quelle version** a servi — livrée, surchargée, ou
  surchargée-mais-retombée, et les deux parties du chrome séparément.

## ✅ La migration, et la mesure qui a suivi

**`Version20260907090000` a été lancée par l'opérateur le 2026-09-07 à 18:30**,
service redémarré, 65 migrations sur 65.

⚠️ **Le code avait été déployé AVANT, et s'en passait** : `markSent()` sonde la
colonne une fois par processus, `fallbackKeys()` retombe sur une liste vide, et la
colonne « Texte » du journal affichait « avant le suivi » partout. C'est la même
discipline qu'en S160 — le code tolère l'absence de son schéma, donc l'ordre de
déploiement n'est pas un piège.
🔴 **Le redémarrage APRÈS la migration n'est pas cosmétique** : la sonde de colonne
est mise en cache pour la vie du processus, donc sans redémarrage la colonne
serait restée vide alors qu'elle existe.

✅ **Et la colonne est ÉCRITE, pas seulement présente** — mesuré par
`app:s162:layout-probe --log-write` : une ligne marquée est insérée en statut
`sent` (**jamais `queued`**, qu'un worker prendrait pour un vrai envoi), la trace
est relue telle quelle, `fallbackKeys()` la remonte, puis la ligne est supprimée et
le journal rendu à son compte de départ (126 lignes).
⚠️ **L'option est OPT-IN, jamais par défaut** : c'est la seule section de la sonde
qui écrit dans le VRAI journal des envois. Une sonde qui écrirait en production
sans qu'on l'ait demandé est une sonde qu'on finit par ne plus lancer.
🅿️ Les 126 lignes existantes affichent « avant le suivi » — elles sont parties
avant la colonne, et **l'inventer serait une affirmation fausse**.

## 🅿️ Ce que la phase laisse ouvert

**L'envoi de test n'est pas livré, délibérément** — voir S161 ci-dessus.
