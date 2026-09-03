# Ce que les utilisateurs de Fab-Manager réclament, et ce que FabOS en a déjà

**2026-09-03.** Dépouillement de <https://feedback.fab-manager.com> — **99 demandes
avec leurs votes**, chargées en entier (le tri par défaut est « TRENDING », pas le
nombre de votes, donc la première page ne suffit pas).

🔴 **Ce sont des DONNÉES, pas un plan.** Ce sont les priorités des utilisateurs
d'un AUTRE produit, votées par une poignée de personnes — le plus haut score du
tableau est **18**. Rien ici n'est une décision pour FabOS ; l'intérêt est de
savoir ce qu'un marché voisin réclame et que nous n'avons pas vu venir.

⚠️ **Chaque ligne « FabOS » ci-dessous a été vérifiée dans le dépôt**, pas
déduite du plan.

---

## Le résultat en une ligne

Sur les 22 demandes les mieux votées : **7 existent déjà dans FabOS**, **8 sont
du commerce** (notre Phase H, déjà planifiée), et **7 sont de vrais écarts** —
dont **trois seulement** valent qu'on s'y arrête.

## Ce que FabOS a DÉJÀ, et qu'ils réclament encore

| Demande | Votes | Ce que FabOS a |
|---|---|---|
| **Support for RFID badges** — leur demande n°1, marquée *Planned* chez eux | **18** | `RfidReader`, `AccessRfidLog`, `MachineAccessService`, `/admin/rfid-readers`, `/admin/access-rfid-logs`. Le boîtier machine est **construit et en service** |
| **Export iCalendar** | 13 | `IcalFeedService` + `CalendarFeedController` : `/calendar/machine/{id}.ics`, `/calendar/place/{id}.ics` |
| **Adding a maintenance system to machines and rooms** | 5 | `MaintenanceTask`, `/admin/maintenance`, `/admin/maintenance-batch` |
| **Document Management** | 3 | `MachineDocument` + son écran (livré en S152) |
| **LDAP as an additional SSO protocol** | 11 | ⚠️ **À moitié** : OIDC est construit (`OidcProvider`, `ExternalIdentityService`). LDAP, non — voir plus bas |
| **Cancel reservations via « My Reservations »** | 4 | `/mes-reservations` porte l'annulation |
| **Users can save their favorite projects** | 4 | ⚠️ **À moitié** : `MachineFavorite` existe, mais sur les MACHINES, pas les projets |

🔴 **Le fait le plus intéressant du tableau** : leur demande la mieux votée, le
RFID, est chez nous **déjà en service** — et l'opérateur vient d'en demander
l'extension aux LIEUX (gâches électriques, todo du 2026-09-03). Nous sommes en
avance sur leur n°1, pas en retard.

## Ce qui est du COMMERCE — notre Phase H, déjà planifiée

Huit demandes, dont plusieurs bien votées, tombent dans le même seau :
**crédit du portefeuille par le membre (9)**, **facturation manuelle (7)**,
**paiements SEPA (6)**, **facture après consommation (4)**, **journées à prix
réduit (3)**, **cartes prépayées (1)**, **notifier les admins qu'un coupon est
utilisé (0)**, **empêcher une réservation après expiration de l'abonnement (3)**.

⚠️ **Ce n'est pas « déjà prévu, donc rien à faire ».** Ce que ça dit, c'est que
la Phase H est le bon chantier, et que la **notion de portefeuille** y revient
plus souvent que je ne l'aurais parié. `wallet` n'existe nulle part dans le
dépôt.

## Les VRAIS écarts — ce qu'ils veulent et que nous n'avons ni fait ni prévu

| Demande | Votes | Ce que ça vaut pour FabOS |
|---|---|---|
| 🔴 **Customize email templates** | **10** | **Le meilleur candidat.** `/admin/emails` règle l'ÉTAT du mail (pause, préférences, journal), pas le TEXTE : aucune entité de gabarit. Un lab qui veut changer le ton de ses mails doit toucher au code |
| 🔴 **Notify members on event creation** | 7 | `EventMailer` ne sait qu'accompagner une inscription — inscrit, liste d'attente, promu, annulé. **Aucune diffusion** à la création d'un événement. Petit, visible, et l'opérateur a déjà retravaillé `/events` cette phase |
| 🔴 **Use OpenBadge to validate trainings** | **14** | Les badges sont au cœur de FabOS, mais **locaux**. Eux veulent qu'un badge obtenu ailleurs valide une formation ici. C'est notre S125 (« badges fédérés »), très loin dans le plan — la demande dit que ça compte plus qu'on ne le croit |
| **LDAP** | 11 | Prévu (S121, « LDAP/SAML restent des adaptateurs ») mais **pas construit**. Deux demandes distinctes le réclament, plus Shibboleth (2) : c'est le monde universitaire, exactement le nôtre |
| **Advanced CMS features** | 12 | Nous avons `LabPage` + `LabPageImage` + les sections d'accueil. Leur demande est vague ; à lire avant d'en conclure quoi que ce soit |
| **Improved user interface** / **Migrate to React** | 5 / 6 | ⚪ **Sans objet** : ce sont leurs dettes techniques (AngularJS 1.8), pas les nôtres |
| **Concurrent project edition** | 7 | ⚪ Édition simultanée façon Google Docs. Coût très élevé, valeur douteuse pour un FabLab |

## Ce que je retiens

1. **`Customize email templates` (10 votes) est le seul écart à la fois bien
   voté, petit, et absent de notre plan.** C'est ce que je proposerais en
   premier.
2. **`Notify members on event creation` (7)** est encore plus petit, et tombe
   pile sur un écran qu'on vient de retravailler.
3. 🅿️ **OpenBadge (14) mérite une décision, pas une implémentation.** La
   fédération de badges est un gros morceau (S125) ; savoir qu'elle est la 2ᵉ
   demande d'un marché voisin peut la faire remonter — ou pas.
4. ⚠️ **Ne pas lire ce tableau comme une liste de tâches.** Le plus haut score
   est 18 votes ; c'est un signal faible. Il sert à corriger nos angles morts,
   pas à réordonner la feuille de route.
