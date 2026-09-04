# Revue Sol — Espaces, réservations et futur accès d’entrée

## Diagnostic

FabOS possède une bonne base de réservation, mais pas encore une expérience complète d’espace ni une exploitation unifiée. L’actuel lecteur RFID est rattaché obligatoirement à une machine : il ne peut pas représenter proprement une porte sans machine fictive.

Le parcours doit répondre, dans cet ordre : **puis-je réserver ? quand ? qu’est-ce qui est inclus ? comment entrer ? quelles consignes ?**

## Frictions relevées

| Tâche | Aujourd’hui | Cible |
| --- | ---: | ---: |
| Trouver un espace libre | 2–3 clics | 1–2 |
| Réserver | 3–4 clics | 2–3 |
| Savoir si l’espace convient | 2 clics | 1 depuis la carte |
| Retrouver/modifier une réservation | 2–3 clics | 1 |
| Créer un espace | 3–4 clics | 3 |
| Traiter une urgence d’exploitation | indisponible / 4+ | 1 |

Les bons fondamentaux à conserver : catalogue public partagé, calendrier de créneaux, horaires par lieu, archivage qui préserve l’historique et filtres admin.

## Priorités

### P0 — sécurité et modèle

1. Introduire `AccessPoint`, distinct de `Machine`, pour porte/portail/casier/zone.
2. Supprimer tout comportement *fail-open* si le jeton API RFID n’est pas configuré : authentifier chaque boîtier et refuser par défaut.
3. Secret d’appairage affiché une seule fois ; rotation, révocation, santé et journal append-only.
4. Décision d’accès côté serveur ; mode hors-ligne verrouillé par défaut, sauf procédure d’urgence explicite et auditée.

### P1 — expérience membre et équipe

1. Fiche espace : photos/plan, équipements, capacité de confort et maximum, accessibilité, règles, contact, conditions de réservation, prochaine ouverture et conditions d’entrée.
2. Carte espace : statut temporel non ambigu ; préférer « Disponible à 14:00 » à « Occupé ».
3. Ajouter « Mes réservations » persistant et une carte prochaine réservation avec point d’entrée et fenêtre d’accès temporaire.
4. Ajouter `Espaces > Exploitation` : réservations proches, fermetures, points hors ligne, refus — une action contextualisée par ligne.
5. Dans `Lieux > Points d’accès`, proposer l’appairage : créer → associer porte/lieu/zones → révéler le secret une fois → tester → suivre/faire tourner/révoquer.

### P2 — cohérence

- Employer strictement Lieu / Espace / Point d’accès ; clarifier « Repère » en salle, étage ou porte.
- Utiliser filtres de lieu sous forme de puces à un clic, avant une recherche textuelle.
- Appliquer la charte FabOS : un seul menu admin, composants et barres d’action partagés, pas de CSS local ni de dashboard générique.

## Prompt de mise en œuvre

> Agis comme product designer senior et ingénieur Symfony. Fais évoluer FabOS Espaces en parcours clair « découvrir → vérifier → réserver → accéder → terminer ». Ne crée ni navigation ni CSS locaux : utilise les composants, le shell et les patterns de `/admin/design`. Conserve les données de réservation existantes. Ajoute un modèle `AccessPoint` séparé de `Machine`, et une relation explicite optionnelle d’accès temporaire depuis une réservation. Les points d’accès représentent porte, portail, casier ou zone ; leurs boîtiers s’authentifient avec un secret révélé une fois, révocable et rotatif. Toutes les décisions sont côté serveur et journalisées ; aucun *fail-open* sans jeton configuré. Ajoute les écrans catalogue, fiche décisionnelle, mes réservations, exploitation et incidents conformément aux maquettes. Chaque filtre courant doit être à un clic ; la recherche seule demande une saisie. Kiosk public sans PII, UID, journal ni secret. Ajoute tests fonctionnels d’autorisation, annulation, fenêtre pré/post réservation, point hors ligne et révocation. Mets à jour la documentation FabApp/docs, déploie sélectivement sur Artemis, vérifie et commit.
