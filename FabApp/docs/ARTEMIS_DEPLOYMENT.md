# Déployer FabOS sur Artemis depuis macOS

Cette procédure est la référence courte pour les sessions Codex. Artemis est uniquement une cible de déploiement et de vérification ; personne n'y développe ni n'y modifie directement le code.

## Connexion

```bash
ssh -i ~/.ssh/id_ovh -p 4002 artemis.dryades.org
```

Le compte SSH n'est pas root. Toutes les commandes du conteneur utilisent :

```bash
sudo pct exec 210 -- bash -lc "…"
```

L'application se trouve dans `/opt/fabos/FabApp` dans CT 210.

## 1. Vérifier avant d'écraser

Comparer les empreintes des fichiers existants avec le `HEAD` local. Une différence ne vient pas d'un autre développeur : elle signale un ancien déploiement Codex qui n'a peut-être pas été reporté dans le checkout local.

Ne jamais synchroniser tout le dépôt et ne jamais utiliser `deploy.sh`.

## 2. Créer une archive macOS propre

Le `tar` macOS ajoute sinon des attributs étendus `LIBARCHIVE.xattr.*`, des entrées AppleDouble `._fichier` et des UID/GID macOS impossibles à restaurer dans CT 210. Cela provoque notamment :

```text
Cannot change ownership to uid …: Invalid argument
Ignoring unknown extended header keyword 'LIBARCHIVE.xattr.com.apple.provenance'
```

Toujours désactiver explicitement les métadonnées macOS et archiver uniquement les chemins intentionnels :

```bash
COPYFILE_DISABLE=1 tar --no-xattrs --no-mac-metadata -czf /tmp/fabos-change.tar.gz \
  FabApp/chemin/fichier-1 \
  FabApp/chemin/fichier-2
```

Contrôler l'archive avant transfert :

```bash
tar -tzf /tmp/fabos-change.tar.gz
```

Elle ne doit contenir ni fichier `._*`, ni chemin non prévu.

## 3. Transférer et prévalider dans `/tmp`

```bash
scp -i ~/.ssh/id_ovh -P 4002 \
  /tmp/fabos-change.tar.gz \
  artemis.dryades.org:/tmp/fabos-change.tar.gz
```

Puis pousser l'archive dans le conteneur et l'extraire dans un répertoire temporaire. `--no-same-owner` est obligatoire même si l'archive a été nettoyée :

```bash
ssh -i ~/.ssh/id_ovh -p 4002 artemis.dryades.org \
  'sudo pct push 210 /tmp/fabos-change.tar.gz /tmp/fabos-change.tar.gz && \
   sudo pct exec 210 -- bash -lc "
     rm -rf /tmp/fabos-change-check &&
     mkdir -p /tmp/fabos-change-check &&
     tar --no-same-owner -xzf /tmp/fabos-change.tar.gz -C /tmp/fabos-change-check
   "'
```

Lancer les `php -l` sur les fichiers temporaires avant de toucher `/opt/fabos`.

## 4. Rollback puis extraction ciblée

Créer dans CT 210 une archive de rollback des fichiers qui existent déjà. Une archive ne peut pas restaurer l'absence d'un fichier nouvellement créé : noter séparément ces nouveaux chemins afin de pouvoir les supprimer lors d'un rollback.

Extraire ensuite avec :

```bash
tar --no-same-owner -xzf /tmp/fabos-change.tar.gz -C /opt/fabos
```

Préserver les propriétaires observés avant déploiement. Les fichiers applicatifs sont généralement lisibles en `root:root`; certains fichiers publics et documents sont actuellement en `www-data:www-data`. Vérifier avec `stat` au lieu de deviner.

## 5. Valider avant le redémarrage

Depuis `/opt/fabos/FabApp` :

```bash
php bin/console lint:twig templates
php bin/console lint:yaml translations config
php bin/phpunit chemin/du/test-cible.php
```

Lire les sorties. Ne jamais redémarrer après un lint en erreur.

Ensuite seulement :

```bash
rm -rf var/cache/prod
APP_ENV=prod php bin/console cache:clear
systemctl restart fabos.service
systemctl is-active fabos.service
```

Redémarrer aussi `fabos-worker.service` uniquement lorsque du code mail/worker a changé.

## 6. Vérifier le résultat réel

- vérifier les routes ciblées avec `debug:router` ;
- rendre les pages privilégiées avec `app:render` ;
- vérifier les journaux `fabos.service` ;
- pour tout changement visuel, contrôler réellement les pages connectées sur ordinateur et mobile ;
- mettre à jour `ROADMAP.md`, `PROJECT_STATE.md` et `HISTORY.md`, puis committer.

Le travail n'est pas livré tant que CT 210 n'a pas été vérifié.
