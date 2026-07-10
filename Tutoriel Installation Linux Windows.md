# Tuto installation du site



## Tuto installation site Linux



Ce tuto est prévu pour Ubuntu / Debian / Linux Mint récent.



---



## 1) Installation des dépendances



### Mise à jour du système



```bash
sudo apt update
sudo apt upgrade -y
```



### Installation de Git, PHP 8.1 ou plus, MariaDB, Composer et des extensions PHP nécessaires



```bash
sudo apt install -y git curl unzip ca-certificates mariadb-server composer php php-cli php-common php-mbstring php-intl php-mysql php-curl php-zip php-xml
```



### Démarrer MariaDB



```bash
sudo systemctl enable --now mariadb
```



### Vérification de l'installation



```bash
sudo mariadb -e "SELECT VERSION();"
```



---



## 2) Installation Symfony-CLI, optionnel mais recommandé



### Installation de Symfony-CLI



```bash
curl -1sLf 'https://dl.cloudsmith.io/public/symfony/stable/setup.deb.sh' | sudo -E bash
sudo apt install -y symfony-cli
```



### Vérification



```bash
symfony version
```



---



## 3) Vérification des dépendances



```bash
php -v
composer --version
git --version
mariadb --version
```



Si Symfony-CLI a été installé :



```bash
symfony version
```



### Vérification des extensions PHP



```bash
php -m | grep -Ei "pdo_mysql|intl|mbstring|openssl|curl|zip|json|xml"
```



Tout doit se faire sans erreur.



---



## 4) Installation du projet et configuration du site



```bash
git clone --filter=blob:none --sparse https://github.com/ENSEA-Makerspace/FabOS.git FabOS
cd FabOS
git sparse-checkout set FabApp
cd FabApp
```



### Installer les dépendances du projet



```bash
composer install
```



### Création de la base de données

### Pour trouver VERSION_MARIADB



```bash
sudo mariadb -e "SELECT VERSION();"
```



Exemple :



Si la commande affiche :



```text
11.8.8-MariaDB
```



Alors mettre dans `DATABASE_URL` :



```text

serverVersion=mariadb-11.8.8

```

```bash
nano .env.local
```



Dans `.env.local` coller et modifier les valeurs en majuscule, le mot de passe ne doit pas contenir de caractères spéciaux :



```env
DATABASE_URL="mysql://NOM_UTILISATEUR:MOT_DE_PASSE@127.0.0.1:3306/NOM_DE_LA_BASE?serverVersion=mariadb-VERSION_MARIADB&charset=utf8mb4"
```







Sauvegarder `.env.local`.



### Créer un utilisateur MariaDB si pas existant



```bash
sudo mariadb
```



Puis dans MariaDB :



```sql
CREATE USER IF NOT EXISTS 'NOM_UTILISATEUR'@'127.0.0.1' IDENTIFIED BY 'MOT_DE_PASSE';
GRANT ALL PRIVILEGES ON NOM_DE_LA_BASE.* TO 'NOM_UTILISATEUR'@'127.0.0.1';
FLUSH PRIVILEGES;
EXIT;
```



Les valeurs `NOM_UTILISATEUR`, `MOT_DE_PASSE` et `NOM_DE_LA_BASE` doivent être exactement les mêmes que dans `.env.local`.



### Test MariaDB



```bash
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 -e "SELECT CURRENT_USER(), VERSION();"
```



### Création de la base



```bash
php bin/console doctrine:database:create
```



### Création des tables



```bash
php bin/console doctrine:schema:create
```



### Vérifications



```bash
php bin/console doctrine:schema:validate
```



### Nettoyage du cache



```bash
php bin/console cache:clear
```



---



## 5) Démarrage



### Avec Symfony-CLI



```bash
symfony server:start
```



### Si Symfony-CLI n'a pas été installé



```bash
php -S 127.0.0.1:8000 -t public
```



Ouvrir ensuite :



```text
http://127.0.0.1:8000
```



Après cette installation, la base est vide. Le site peut démarrer, mais les machines, formations, utilisateurs, rôles, réservations et contenus devront être ajoutés ensuite.



---

## 6) Mise en place du premier Admin

> **!!! TRÈS IMPORTANT !!!**  
> Il faut d'abord créer un utilisateur via le site, sur la page :
>
> ```text
> /register
> ```

Les champs à remplacer par leurs vraies valeurs sont :

- `NOM_UTILISATEUR`
- `NOM_DE_LA_BASE`
- `EMAIL_DU_COMPTE`

### Création des rôles STAFF et ADMIN

```bash
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 NOM_DE_LA_BASE -e "INSERT INTO ROLE (nom) SELECT 'ROLE_ADMIN' WHERE NOT EXISTS (SELECT 1 FROM ROLE WHERE nom = 'ROLE_ADMIN'); INSERT INTO ROLE (nom) SELECT 'ROLE_STAFF' WHERE NOT EXISTS (SELECT 1 FROM ROLE WHERE nom = 'ROLE_STAFF');"
```

### Attribuer le rôle Admin au compte créé

```bash
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 NOM_DE_LA_BASE -e "INSERT INTO UTILISATEUR_ROLE (utilisateurId, roleId) SELECT u.id, r.id FROM UTILISATEUR u JOIN ROLE r WHERE u.email = 'EMAIL_DU_COMPTE' AND r.nom = 'ROLE_ADMIN' AND NOT EXISTS (SELECT 1 FROM UTILISATEUR_ROLE ur WHERE ur.utilisateurId = u.id AND ur.roleId = r.id);"
```

# Tuto installation site sur Windows



## 1) Installation des dépendances



### Git



```text
https://git-scm.com/download/win
```



### PHP 8.1 ou plus



```text
https://windows.php.net/download/
```



- Ajouter PHP au PATH

- Configuration de `php.ini` :

   - Copier `php.ini-development` et le renommer en `php.ini`

   - Dans le nouveau `php.ini`, chercher les lignes suivantes et retirer les `;` au début :



```ini
extension_dir = "ext"

extension=curl
extension=fileinfo
extension=intl
extension=mbstring
extension=mysqli
extension=openssl
extension=pdo_mysql
extension=zip
```



### Vérification que PHP utilise le bon fichier



```powershell
php --ini
```



### Vérification de l'installation des extensions



```powershell
php -m | findstr /I "pdo_mysql intl mbstring openssl curl zip json"
```



### Composer



```text
https://getcomposer.org/download/
```



### MariaDB



```text
https://mariadb.org/download/
```



- Ajouter MariaDB au PATH

- Vérification de l'installation :



```powershell
mariadb -u root -p -e "SELECT VERSION();"
```



`root` user par défaut.



---



## 2) Installation Symfony-CLI via Scoop, optionnel mais mieux



### Autorisation d'exécution de scripts PowerShell



```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```



### Installation de Scoop



```powershell
irm get.scoop.sh | iex
scoop update
```



### Installation de Symfony-CLI



```powershell
scoop install symfony-cli
```



---



## 3) Vérification des dépendances



```powershell
php -v
composer --version
git --version
mariadb --version
```



Si Symfony-CLI a été installé :



```powershell
symfony version
```



### Vérification des extensions PHP



```powershell
php -m | findstr /I "pdo_mysql intl mbstring openssl curl zip json"
```



Tout doit se faire sans erreur.



---



## 4) Installation du projet et configuration du site



```powershell
git clone --filter=blob:none --sparse https://github.com/ENSEA-Makerspace/FabOS.git FabOS
cd FabOS
git sparse-checkout set FabApp
cd FabApp
composer install
```



### Création de la base de données



```powershell
notepad .env.local
```



Dans `.env.local` coller et modifier les valeurs en majuscule, le mot de passe ne doit pas contenir de caractères spéciaux :



```env
DATABASE_URL="mysql://NOM_UTILISATEUR:MOT_DE_PASSE@127.0.0.1:3306/NOM_DE_LA_BASE?serverVersion=mariadb-VERSION_MARIADB&charset=utf8mb4"
```



### Créer un utilisateur MariaDB si pas existant



```powershell
mariadb -u root -p
```



Puis dans MariaDB :



```sql
CREATE USER IF NOT EXISTS 'NOM_UTILISATEUR'@'127.0.0.1' IDENTIFIED BY 'MOT_DE_PASSE';
GRANT ALL PRIVILEGES ON NOM_DE_LA_BASE.* TO 'NOM_UTILISATEUR'@'127.0.0.1';
FLUSH PRIVILEGES;
```



### Test MariaDB



```powershell
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 -e "SELECT CURRENT_USER(), VERSION();"
```



### Création de la base



```powershell
php bin/console doctrine:database:create
```



### Création des tables



```powershell
php bin/console doctrine:schema:create
```



### Vérifications



```powershell
php bin/console doctrine:schema:validate
```



### Nettoyage du cache



```powershell
php bin/console cache:clear
```



---



## 5) Démarrage



### Avec Symfony-CLI



```powershell
symfony server:start
```



### Si Symfony-CLI n'a pas été installé



```powershell
php -S 127.0.0.1:8000 -t public
```



Après cette installation, la base est vide. Le site peut démarrer, mais les machines, formations, utilisateurs, rôles, réservations et contenus devront être ajoutés ensuite.

---

## 6) Mise en place du premier Admin

> **!!! TRÈS IMPORTANT !!!**  
> Il faut d'abord créer un utilisateur via le site, sur la page :
>
> ```text
> /register
> ```

Les champs à remplacer par leurs vraies valeurs sont :

- `NOM_UTILISATEUR`
- `NOM_DE_LA_BASE`
- `EMAIL_DU_COMPTE`

### Création des rôles STAFF et ADMIN

```bash
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 NOM_DE_LA_BASE -e "INSERT INTO ROLE (nom) SELECT 'ROLE_ADMIN' WHERE NOT EXISTS (SELECT 1 FROM ROLE WHERE nom = 'ROLE_ADMIN'); INSERT INTO ROLE (nom) SELECT 'ROLE_STAFF' WHERE NOT EXISTS (SELECT 1 FROM ROLE WHERE nom = 'ROLE_STAFF');"
```

### Attribuer le rôle Admin au compte créé

```bash
mariadb -u NOM_UTILISATEUR -p -h 127.0.0.1 NOM_DE_LA_BASE -e "INSERT INTO UTILISATEUR_ROLE (utilisateurId, roleId) SELECT u.id, r.id FROM UTILISATEUR u JOIN ROLE r WHERE u.email = 'EMAIL_DU_COMPTE' AND r.nom = 'ROLE_ADMIN' AND NOT EXISTS (SELECT 1 FROM UTILISATEUR_ROLE ur WHERE ur.utilisateurId = u.id AND ur.roleId = r.id);"
```