# Revue Sol — Équipement, matériaux et contrôle d'accès

**Statut :** revue lecture seule, 2026-09-04. Aucune modification de FabOS.

## Verdict

Le socle est prometteur : listes admin partagées, fiche machine avec réservation, archivage plutôt que suppression et lecteurs RFID déjà lisibles. Mais les surfaces restent organisées autour de tables techniques plutôt que des tâches : un membre veut savoir s'il peut utiliser une machine et avec quel matériau ; le staff veut corriger un refus ; l'admin veut rendre une machine ou un boîtier réellement opérationnel.

## Parcours et clics

| Tâche | Parcours actuel | Clics | Cible |
|---|---|---:|---:|
| Identifier une machine et réserver | Machines → fiche → réserver → créneau → confirmer | 4–5 | 3–4 |
| Comprendre un accès refusé | Machines → fiche → état/badge → Formations | 4–6 | 2 |
| Trouver un matériau compatible | Matériaux → carte (retour liste) → recherche machine | 3+, sans réponse fiable | 2 |
| Créer/appairer un lecteur | Lecteurs → créer → remplir → enregistrer → config externe | 5–6 + hors produit | 3 + vérification |
| Diagnostiquer un refus RFID | Logs → filtre → détail → interprétation | 4 | 2 |

## P0 — sécurité et exploitation

1. Si `FABOS_RFID_API_TOKEN` est absent, `RfidMachineController::rejectUnauthorizedDevice()` laisse l'API device ouverte. En production, l'absence de secret doit refuser l'accès ou empêcher le démarrage.
2. `/kiosk/entries` expose noms, avatars et passages RFID sans protection équivalente. Décider explicitement : signalétique publique anonymisée, ou kiosk authentifié par device grant.
3. Le formulaire Lecteur montre un exemple `.env` incluant `FABOS_DB_*`. Un boîtier ne doit jamais recevoir d'accès SQL : API-only avec secret device restreint.
4. Tokens machine/lecteur insuffisants : ajouter identité device, secret rotation/révocation, dernière connexion, version, santé et audit.
5. Le booléen actif + `lastSeenAt` ne distingue pas prêt, hors ligne, non configuré, erreur ou association invalide.

## P1 — produit et parcours

1. Deux sources de vérité matière : tableau texte `Machine::materials` et relation `MACHINE_MATERIAL`. La relation doit devenir canonique ; les textes historiques restent des notes transitoires.
2. Les cartes Matériaux renvoient à la liste, sans fiche de matériau ni compatibilité vérifiable.
3. Le catalogue promet implicitement du stock sans quantité, unité, seuil, lot ou mouvements. Rester sur un référentiel compatible, ou livrer le stock réellement.
4. La fiche machine mélange trop d'audiences. Le membre doit avoir statut utilisable, prochaine action, prérequis exacts, matériaux compatibles et réservation ; staff/admin une zone Exploitation distincte.
5. Les prérequis de formation sont génériques : montrer la formation précise, progression et étape pratique restante plutôt que rediriger au catalogue.
6. Les logs RFID sont des journaux, pas encore des incidents actionnables. Une cause doit mener vers membre/badge, formation, lecteur ou machine.

## P2 — design system

- Les kiosks gardent favicon, CSS et styles locaux : créer un `kiosk-shell` consommant thème publié, logo, favicon, palette et contraste.
- La fiche machine possède encore des patterns locaux pour matériaux et maintenance : les extraire vers le design system.
- Le formulaire Lecteur RFID garde header, CSS et JS propres : conserver le contenu spécialisé mais employer le shell/formulaire admin canonique.

## Direction recommandée

### Workspace Équipement unique

Conserver un seul workspace avec **Vue d'ensemble**, **Machines**, **Matériaux** et **Accès**. Sous Accès : lecteurs/boîtiers, incidents d'accès puis journal RFID. La vue d'ensemble montre ce qui demande une action : maintenance, machines indisponibles, lecteurs hors ligne, refus récents et stock faible seulement si celui-ci existe.

### Machine

Fiche à deux vues cohérentes : membre (action suivante) et staff/admin (Exploitation : santé, maintenance, lecteur, dernier contact, refus, badges et corrections). Une action principale par état.

### Matériau

Créer `/materiaux/{id}` : description, variantes, préparation/sécurité, stockage, machines compatibles et CTA vers ces machines. Les cartes ouvrent cette fiche.

### Accès

Mise en service : créer → associer → révéler le secret une fois → connexion vérifiée. Les refus doivent proposer leur correction. Ne jamais exposer UID RFID ou secrets au-delà du besoin.

### Kiosks

Signalétique publique sans PII ; kiosks staff authentifiés/attribués à un device pour noms, scans, logs ou actions. Consommation obligatoire du thème publié.

## Prompt d'implémentation

> Agis comme un senior product designer et ingénieur Symfony. Refonte le stack FabOS Équipement — Machines, Matériaux, lecteurs/boîtiers RFID et kiosks — comme un seul système opérationnel, sans créer de navigation ou CSS locaux.
>
> Utilise le shell admin/public canonique, les composants partagés de listes/formulaires et ajoute tout nouveau pattern durable à `/admin/design`.
>
> **Priorité sécurité** : en production, refuse l'API RFID si `FABOS_RFID_API_TOKEN` est absent ; remplace tout accès DB boîtier par une API avec secret device unique, révocable/rotatable et révélé une seule fois ; ajoute identité device, dernière connexion, version, santé et audit ; protège les kiosks par mode, sans PII publique.
>
> **Information architecture** : un seul workspace Équipement avec Vue d'ensemble, Machines, Matériaux et Accès. Sous Accès : Lecteurs/boîtiers, Incidents puis Journal. Ajoute une vue d'exploitation des actions urgentes.
>
> **Machines** : fiche membre task-first (disponibilité, droit réel, qualification exacte, matériaux compatibles, sécurité, réservation) ; zone Exploitation staff/admin (santé, maintenance, lecteur, incidents, badges, liens de correction). Une action principale par état.
>
> **Matériaux** : `Material ↔ Machine` devient la source canonique ; valeurs texte migrées comme notes transitoires. Ajouter une fiche routable avec specs, sécurité, stockage, machines compatibles et CTA. Ne pas promettre de stock sans modèle réel.
>
> **Accès** : lecteur avec association, sous-lieu, état prêt/hors-ligne/désactivé/erreur, dernière communication, version et test sûr. Mise en service en trois étapes. Chaque refus RFID est humainement expliqué et actionnable.
>
> **Kiosks** : `kiosk-shell` consommant le thème publié (logo, favicon, nom, palette, contraste, typographie), sans styles ou favicon répétés.
>
> **Qualité** : identifier/réserver ≤4 clics ; comprendre un refus ≤2 clics depuis Incidents ; mise en service + connexion vérifiée ≤3 clics dans FabOS. Tests API/secret/révocation/kiosk PII/compatibilité/archivage ; validation Artemis et documentation.
