#!/usr/bin/env bash
# Installe (ou remet en route) le Keycloak de TEST sur CT 210. Idempotent.
#
# 🔴 Les secrets sont tirés ICI, sur la boîte, et ne quittent jamais la boîte :
#   /opt/keycloak-test/.env            mot de passe de la console d'admin Keycloak
#   /opt/keycloak-test/test-users.txt  mots de passe des comptes de test
#   /opt/fabos/FabApp/.env.local       KEYCLOAK_TEST_CLIENT_SECRET (lu par FabOS)
# Tous en 600, propriété de root. Rien n'est affiché.
set -euo pipefail
cd "$(dirname "$0")"
umask 077

rand() { head -c 48 /dev/urandom | base64 | tr -dc 'A-Za-z0-9' | head -c "${1:-24}"; }

if [ ! -f .env ]; then
    printf 'KC_BOOTSTRAP_ADMIN_USERNAME=admin\nKC_BOOTSTRAP_ADMIN_PASSWORD=%s\n' "$(rand 28)" > .env
fi
if [ ! -f secrets.env ]; then
    {
        echo "CLIENT_SECRET=$(rand 40)"
        for u in ALICE BOB CAROL FRANK DAVE; do echo "PW_${u}=$(rand 16)"; done
    } > secrets.env
fi
# shellcheck disable=SC1091
. ./secrets.env

mkdir -p import
sed -e "s/__CLIENT_SECRET__/${CLIENT_SECRET}/" \
    -e "s/__PW_ALICE__/${PW_ALICE}/" -e "s/__PW_BOB__/${PW_BOB}/" -e "s/__PW_CAROL__/${PW_CAROL}/" \
    -e "s/__PW_FRANK__/${PW_FRANK}/" -e "s/__PW_DAVE__/${PW_DAVE}/" \
    realm.template.json > import/realm-fabos-test.json
chmod 755 import && chmod 644 import/realm-fabos-test.json   # lisible par l'utilisateur du conteneur

cat > test-users.txt <<USERS
Comptes de TEST du royaume fabos-test (https://auth-test.dstei.fr/realms/fabos-test)
alice  / ${PW_ALICE}   adresse garantie, nom complet   → compte ouvert directement (S196)
bob    / ${PW_BOB}     aucune adresse                  → « Complétez votre compte » (S197)
carol  / ${PW_CAROL}   adresse NON vérifiée             → proposée, confirmation par courrier (S197)
frank  / ${PW_FRANK}   adresse garantie, aucun nom      → la page demande le nom (S197)
dave   / ${PW_DAVE}    désactivé dans Keycloak          → Keycloak refuse lui-même
USERS

# FabOS lit le secret client par le NOM de sa variable (réglage « KEYCLOAK_TEST_CLIENT_SECRET »).
ENV_LOCAL=/opt/fabos/FabApp/.env.local
if ! grep -q '^KEYCLOAK_TEST_CLIENT_SECRET=' "$ENV_LOCAL"; then
    echo "KEYCLOAK_TEST_CLIENT_SECRET=${CLIENT_SECRET}" >> "$ENV_LOCAL"
fi

# CT 210 ne joint pas son propre nom public (pas de hairpin) : on passe par NPM sur le LAN.
if ! grep -q 'auth-test.dstei.fr' /etc/hosts; then
    echo '192.168.100.20 auth-test.dstei.fr   # S196 — Keycloak de test via NPM (pas de hairpin)' >> /etc/hosts
fi

docker compose up -d
