# Keycloak de test — les modules de connexion de FabOS (Phase U)

Un fournisseur OpenID Connect (et plus tard SAML) **de test**, pour éprouver
« Connexion & annuaires » de bout en bout sans jamais toucher l'annuaire d'un
établissement.

- **Où** : CT 210, `/opt/keycloak-test`, conteneur `keycloak` (port 8081 sur le LAN).
- **Adresse** : `https://auth-test.dstei.fr` — DNS + hôte NPM (certificat
  Let's Encrypt, même liste blanche que `fabos.dstei.fr`) vers
  `http://192.168.100.21:8081`.
- **Royaume** `fabos-test`, client `fabos` (confidentiel, PKCE S256, retour
  `https://fabos.dstei.fr/login/oidc/callback`).
- **Comptes** : un par cas de S196/S197 (voir `test-users.txt` sur la boîte),
  tous en `@example.org` — aucun courrier ne peut y arriver.

## Installer / remettre en route

```
sudo pct exec 210 -- /opt/keycloak-test/setup.sh
```

Le script tire les secrets SUR la boîte (`.env`, `secrets.env`,
`test-users.txt`, en 600) et ajoute `KEYCLOAK_TEST_CLIENT_SECRET` à
`/opt/fabos/FabApp/.env.local`. Rien n'est affiché, rien n'entre dans le dépôt.

## Dans FabOS

Configuration → « Connexion & annuaires » → « Ajouter un fournisseur » :

| Champ | Valeur |
|---|---|
| Nom affiché | Keycloak de test |
| Clé | `keycloak_test` |
| Émetteur | `https://auth-test.dstei.fr/realms/fabos-test` |
| Identifiant client | `fabos` |
| Variable du secret | `KEYCLOAK_TEST_CLIENT_SECRET` |
| Préréglage | OpenID Connect standard |

Puis « Tester » avec chaque compte : le rapport dit ce que FabOS ferait.

⚠️ `/etc/hosts` de CT 210 fait pointer `auth-test.dstei.fr` vers NPM
(192.168.100.20) : la boîte ne joint pas son propre nom public.
