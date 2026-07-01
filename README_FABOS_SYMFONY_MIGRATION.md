
# Migration du site HTML FabOS/TLSE vers Symfony

Ce projet part du dossier Symfony fourni et intègre le code source HTML/CSS/JS du site dans une structure Symfony.

## Ce qui a été adapté

- Pages HTML converties en templates Twig dans `templates/site/`.
- Assets copiés dans `public/css`, `public/js`, `public/images`.
- Liens `.html` remplacés par des routes Symfony quand c'était possible.
- Controllers publics : accueil, calendrier, machines, formations, leaderboard, login, profil, recherche.
- Controllers admin : dashboard, machines, formations, réservations, utilisateurs, badges.
- Entités Doctrine/MariaDB : `MachineCategory`, `Machine`, `TlseUser`, `Reservation`, `Formation`, `Badge`, `UserBadge`, `TrainingEnrollment`, `MachineUsageHistory`, `Notification`.
- API JSON : `/api/navigation`, `/api/machines`, `/api/calendar`, `/api/formations`, `/api/leaderboard`, `/api/search`, `/api/reservations`.
- Commande de données de test : `php bin/console app:fabos:seed --reset`.

## Installation rapide

```powershell
cd C:\Users\alvar\Documents\TLSE\fabos-symfony
composer install
notepad .env.local
```

Exemple MariaDB :

```env
DATABASE_URL="mysql://root:admin@127.0.0.1:3306/fabos_symfony?serverVersion=mariadb-12.3.2&charset=utf8mb4"
```

Puis :

```powershell
php bin/console cache:clear
php bin/console doctrine:database:create --if-not-exists
php bin/console doctrine:migrations:migrate
php bin/console app:fabos:seed --reset
symfony server:start --no-tls
```

## Routes principales

- `/`
- `/calendrier`
- `/machines`
- `/machines/1`
- `/machines/1/calendrier`
- `/machines/1/historique`
- `/formations`
- `/formations/1`
- `/formations/1/suivi`
- `/leaderboard`
- `/login`
- `/register`
- `/profil`
- `/admin`
- `/api/machines`
- `/api/calendar`

Les anciennes URL `.html` sont conservées en aliases pour limiter la casse pendant la migration.

## Limite volontaire

Les pages visuelles reprennent la maquette HTML d'origine. La vraie logique métier complète n'est pas inventée : authentification réelle, droits admin, validation officielle des formations et règles de réservation devront être câblées ensuite.
