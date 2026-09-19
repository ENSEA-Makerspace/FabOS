# Phase P — Espaces & accès d'entrée (S175–S178)

**Planifiée le 2026-09-04, CLOSE le 2026-09-06.** D'après les huit planches et la
revue Sol. Elle absorbe le todo « contrôle d'accès aux LIEUX » du 2026-09-03.
⚠️ La règle de lecture des planches est celle de la Phase O : présentation de
contenu uniquement, jamais le thème, et **pas une liste de tâches**.

**Le résultat** : `AccessPoint` existe — un boîtier peut commander une **PORTE**,
plus seulement une machine.

---

## 🔴 Le fait de modèle qui commandait toute la phase

**Un lecteur RFID était rattaché OBLIGATOIREMENT à une machine.** Une porte ne
pouvait donc être représentée qu'en **inventant une machine fictive** — exactement
le genre de contournement qui se paie deux ans plus tard. La revue proposait
`AccessPoint`, distinct de `Machine` : porte, portail, casier, zone.
✅ C'est la réponse au todo de l'opérateur sur les gâches électriques.

⚠️ **Et le verdict d'une porte est plus RICHE que celui d'une machine** : les axes
lieu / jours / horaires d'un forfait le décrivent DÉJÀ (`PackageSpec`), là où
l'accès machine est un booléen sur les badges. **On ne réinvente rien ; on
branche.** C'est ce qui a permis à la phase de tenir en quatre sessions.

---

## S175 — `AccessPoint`, et la mesure qui vaut pour la session entière

Le lecteur s'y rattache aussi bien qu'à une machine. **Migration additive**, aucun
lecteur existant déplacé.

🔴 **La mesure de la session est un NON-CHANGEMENT** : le rendu de la liste des
boîtiers diffère d'exactement **deux lignes** avant/après — le compteur du menu
Espaces (4→5) et l'en-tête de colonne. Rien d'autre n'a bougé. « LECTEUR ZÉRO »
affiche toujours exactement ce qu'il affichait : `Imprimante 3D test`, `Inactif`.
C'est la forme de preuve qu'une migration additive appelle : on ne montre pas ce
qu'on a ajouté, on montre que **rien d'autre** n'a bougé.

Décisions du modèle :
- **Le champ « Espace ouvert » peut rester VIDE** — un portail d'entrée n'ouvre
  aucune salle en particulier. **Vide est une réponse, pas un oubli**, et l'écran
  doit le dire.
- **Choisir une machine ET un point d'accès est REFUSÉ**, avec l'erreur sur le
  champ. Un boîtier commande une chose.
- La colonne s'appelle désormais **Commande**, plus « Machine » : le vocabulaire
  suit le modèle, sinon la colonne ment dès la première porte.
- **L'écran `/admin/access-points` est VIDE à la livraison.** ⚠️ C'est le bon
  résultat : la migration crée la table, **elle n'invente aucune porte**.

---

## S176 — la mise en service, et les incidents actionnables

Créer → associer porte/lieu → révéler le secret UNE fois → tester la connexion.
Et les **incidents** d'accès actionnables : une cause mène vers le membre, le
badge, la formation, le lecteur.

🔴 **Une étape bloquante était INVISIBLE.** Depuis S171 la garde refuse tout appel
sans `FABOS_RFID_API_TOKEN` — et **aucun écran ne le disait**. Le bloc « Mise en
service » de `/admin/rfid-readers/{id}/edit` montre maintenant 4 étapes faites et
**1 bloquante en rouge** : « Allumer l'API des boîtiers ». C'est la même famille
que le demi-modèle : une garde correcte dont aucune surface ne dit qu'elle est
active se lit comme une panne.

⚠️ **Une cause commune ne doit pas se lire comme plusieurs pannes.** Le bandeau
rouge le dit **une fois pour tous** les boîtiers en haut de la liste ; la colonne
Statut ne le répète PAS.

**Mesuré sur les incidents** : `/admin/access-rfid-logs?days=0&result=no` —
**72 refus sur 72** portent une colonne « À faire » avec un **VERBE**, et le clic
mène au bon endroit : 42 vers la fiche du membre, 19 vers LE lecteur fautif, 11
vers la liste quand la cible n'existe pas par construction. **Zéro lien mort.**

✅ Les deux composants neufs sont dans `/admin/design`, avec leurs trois états et
**le défaut qui les a fait naître**.

---

## S177 — le parcours membre

Disponibilité lisible dès la carte (« Disponible à 14:00 » plutôt que « Occupé »),
fiche d'espace qui répond « puis-je réserver, quand, qu'est-ce qui est inclus,
comment j'entre », et « Mes réservations » avec la prochaine et sa fenêtre d'accès.
🔴 **Compté en clics**, cibles de la revue : trouver un espace libre 1–2, réserver
2–3, retrouver sa réservation 1.

Sur `/places/{id}`, une section **« Comment on y entre »**. Aujourd'hui elle dit
qu'**aucun accès n'est déclaré** — c'est vrai, et **le dire vaut mieux qu'une page
qui a l'air complète**.

🅿️ **Le seul point de la phase qui n'a pas pu être mesuré**, et il reste ouvert
dans `ROADMAP.md` : sur `/places`, la pastille d'une salle occupée doit dire
« Libre à 14:00 », plus « Occupé ». À 01h30 le labo est fermé et la branche
« fermé » gagne — **correctement**. La prouver demandait d'insérer une réservation
en production : **le classificateur a refusé, et il a raison**. Vérifié qu'aucune
écriture n'a eu lieu. À regarder un jour ouvré, aux heures d'ouverture.

---

## S178 — l'accès temporaire lié à une réservation

Une marge courte avant/après, révoqué à l'annulation. Et `Espaces > Exploitation` :
réservations proches, fermetures, points hors ligne, refus.

**Mesuré** : `php bin/console app:s178:door-probe` — huit sections vertes, dont
**« refusé immédiatement après l'annulation »**. Base rendue à l'identique.

⚠️ **Et la formulation exacte compte** : *rien n'est révoqué parce que rien n'est
accordé*. La question est **reposée à chaque badge**. Un modèle qui accorderait un
droit à la réservation puis le retirerait à l'annulation aurait un état à tenir
synchronisé — et un état à révoquer est un état qu'on peut oublier de révoquer.

Sur `/admin/access-points`, une porte sans boîtier lit **« Aucun boîtier »** en
ambre, pas « Actif » en vert. **Une porte annoncée au membre et que rien n'ouvre
est une affordance morte** — de la famille J-5, sur le chemin où elle coûte le plus
cher.

---

## ⚠️ Ce que cette boîte ne permet PAS de mesurer

L'API des boîtiers rend `503 device_api_not_configured` faute de
`FABOS_RFID_API_TOKEN` (**S171 fait son travail**). **Aucune autorisation de bout
en bout n'est vérifiable ici** — seulement la base et l'écran. Poser la variable
dans le `.env.local` de CT 210 reste à l'opérateur.

## La passe de fond de cette phase

- **Réemploi** : le calendrier, les créneaux et les politiques de réservation
  existent — cette phase ne devait pas en écrire une seconde version, et ne l'a
  pas fait.
- ⚠️ **Le kiosque d'entrée est PUBLIC** : ni identité, ni UID de badge, ni journal,
  ni secret. **C'est un critère de sortie, pas une intention.**
- **Conformité** : mêmes outils, début et fin.
