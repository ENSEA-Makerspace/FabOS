# FabOS — Lancer l'application en local

Guide pour démarrer FabOS sur ta machine. Stack : **Symfony 8.1 / PHP 8.4 / MariaDB**.

## Prérequis

- **PHP 8.4** avec les extensions `ctype`, `iconv`, `pdo_mysql`, `intl`
- **Composer** 2.x
- **Docker** + Docker Compose (pour la base MariaDB et le serveur mail de test)
- (Recommandé) le binaire **Symfony CLI** : https://symfony.com/download

## 1. Base de données (Docker)

Le `compose.yaml` fournit MariaDB (base `fabos_symfony`) et Mailpit (capture des mails de test).

```bash
cd FabApp
docker compose up -d
```

Vérifie que la base répond :

```bash
docker compose ps
```

> Le port MariaDB est publié sur un port aléatoire par `compose.override.yaml`. Pour le voir : `docker compose port database 3306`. Si tu préfères le port fixe 3306, remplace `- "3306"` par `- "3306:3306"` dans `compose.override.yaml`.

## 2. Dépendances PHP

```bash
composer install
```

## 3. Configuration locale

```bash
cp .env.local.example .env.local
```

Ouvre `.env.local` et vérifie la ligne `DATABASE_URL`. Par défaut (conteneur Docker fourni) :

```
DATABASE_URL="mysql://root:!ChangeMe!@127.0.0.1:3306/fabos_symfony?serverVersion=mariadb-11.4.0&charset=utf8mb4"
```

Adapte l'hôte/port si tu n'utilises pas le port 3306 fixe (voir note ci-dessus), et `serverVersion` à ta version réelle de MariaDB.

Génère aussi un `APP_SECRET` (actuellement vide) et ajoute-le dans `.env.local` :

```bash
php -r "echo 'APP_SECRET='.bin2hex(random_bytes(16)).PHP_EOL;" >> .env.local
```

## 4. Construire la base + données de démo

Le chemin canonique est **les migrations Doctrine** : la première migration charge le schéma MariaDB historique (tables + jeu de démo), les suivantes ajoutent les fonctionnalités récentes (favoris, horaires d'ouverture, lecteurs RFID, badges machine, etc.).

```bash
php bin/console doctrine:database:create --if-not-exists
php bin/console doctrine:migrations:migrate --no-interaction
```

> N'utilise **pas** `app:fabos:seed` pour un premier build : il charge une copie plus ancienne du schéma (`resources/database/`) sans les patchs. Réserve-le à une remise à zéro des données de démo une fois le modèle stabilisé.

## 5. Lancer le serveur

```bash
symfony server:start --no-tls
# ou sans la CLI Symfony :
php -S 127.0.0.1:8000 -t public
```

Ouvre http://127.0.0.1:8000.

## 6. Se connecter

Les utilisateurs de démo existent mais leur mot de passe est un placeholder (`password_hash_test`) — **non utilisable en l'état**. Définis un vrai mot de passe pour un compte de démo :

```bash
# 1) Génère un hash compatible Symfony pour le mot de passe de ton choix
php bin/console security:hash-password
```

Puis applique-le à un utilisateur (par ex. `yanis@example.com`) :

```bash
docker compose exec database mariadb -uroot -p'!ChangeMe!' fabos_symfony \
  -e "UPDATE UTILISATEUR SET password='COLLE_LE_HASH_ICI' WHERE email='yanis@example.com';"
```

Connecte-toi ensuite via `/login`.

## Dépannage

- **`SQLSTATE ... server version` / erreurs SQL** : vérifie que `serverVersion` dans `DATABASE_URL` correspond bien à ta MariaDB (`docker compose exec database mariadb --version`).
- **`Access denied` MySQL** : le mot de passe dans `.env.local` doit correspondre à `MARIADB_ROOT_PASSWORD` du conteneur (`!ChangeMe!` par défaut).
- **Migration qui échoue à mi-chemin** : repars propre avec `docker compose down -v && docker compose up -d`, puis reprends à l'étape 4.
- **Mails** : l'interface Mailpit est exposée (voir `docker compose port mailer 8025`).

## À savoir pour la suite (dette technique repérée)

- **Entités dupliquées** issues de la migration : `Utilisateur` vs `TlseUser`, `UserBadge` / `UtilisateurBadge`, `Role` / `UtilisateurRole`. La sécurité utilise `Utilisateur`. À consolider.
- **Deux copies du schéma legacy** (`migrations/sql/` = à jour, `resources/database/` = ancienne, utilisée par la commande de seed). À unifier.
- **Logique métier** encore partielle : droits admin, validation des formations, règles de réservation (voir `README_FABOS_SYMFONY_MIGRATION.md`).
- **Avant open source** : licence `proprietary` à changer, audit des secrets, README public.
