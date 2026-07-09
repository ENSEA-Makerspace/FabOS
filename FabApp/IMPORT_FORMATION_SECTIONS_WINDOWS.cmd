@echo off
setlocal
cd /d "%~dp0"

echo ============================================================
echo FABOS - Import des parcours guides et mini-quiz
echo ============================================================
echo.
echo Cet import utilise la connexion DATABASE_URL du projet.
echo Il ne cree aucune table, aucune colonne et aucune migration.
echo.

php bin/console app:import-guided-sections
if errorlevel 1 (
    echo.
    echo ECHEC DE L'IMPORT.
    echo Verifiez que MariaDB est demarre et que .env.local pointe vers fabos_local.
    pause
    exit /b 1
)

echo.
echo Import termine. Nettoyage du cache Symfony...
php bin/console cache:clear

echo.
echo Termine. Rechargez ensuite la page avec Ctrl + F5.
pause
endlocal
