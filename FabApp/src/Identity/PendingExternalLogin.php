<?php

declare(strict_types=1);

namespace App\Identity;

use Symfony\Component\HttpFoundation\Session\SessionInterface;

/**
 * S197 — une connexion externe RÉUSSIE chez le fournisseur, mais qui ne peut
 * pas encore ouvrir de compte : on garde son profil, le temps de compléter.
 *
 * ⚠️ Dans la session de CE navigateur, 15 minutes au plus, jamais en base : la
 * preuve « le fournisseur a authentifié cette personne » ne vaut que pour la
 * session qui l'a reçue, et pas longtemps.
 * 🔴 `wantsLink` n'est posé que par un clic explicite sur « J'ai déjà un compte
 * FabOS » : sans lui, une connexion locale ultérieure dans le même navigateur
 * ne lierait RIEN.
 */
final class PendingExternalLogin
{
    public const KEY = 'identity_pending';
    private const TTL = 900;

    public function store(SessionInterface $session, AuthProvider $provider, ExternalProfile $profile, IdentityDecision $decision): void
    {
        $session->set(self::KEY, [
            'at' => time(),
            'providerLabel' => $provider->label,
            'needs' => $decision->needs,
            'emailTaken' => $decision->emailTaken,
            'wantsLink' => false,
            'profile' => [
                'providerKey' => $profile->providerKey, 'issuer' => $profile->issuer, 'subject' => $profile->subject,
                'email' => $profile->email, 'emailVerified' => $profile->emailVerified,
                'firstName' => $profile->firstName, 'lastName' => $profile->lastName, 'displayName' => $profile->displayName,
                'affiliations' => $profile->affiliations,
            ],
        ]);
    }

    /** @return array{at: int, providerLabel: string, needs: list<string>, emailTaken: bool, wantsLink: bool, profile: array<string, mixed>}|null */
    public function get(SessionInterface $session): ?array
    {
        $pending = $session->get(self::KEY);
        if (!\is_array($pending) || time() - (int) ($pending['at'] ?? 0) > self::TTL) {
            $session->remove(self::KEY);

            return null;
        }

        return $pending;
    }

    public function profile(SessionInterface $session): ?ExternalProfile
    {
        $p = $this->get($session)['profile'] ?? null;
        if (!\is_array($p)) {
            return null;
        }

        return new ExternalProfile(
            providerKey: (string) $p['providerKey'], issuer: (string) $p['issuer'], subject: (string) $p['subject'],
            email: $p['email'], emailVerified: (bool) $p['emailVerified'],
            firstName: $p['firstName'], lastName: $p['lastName'], displayName: $p['displayName'],
            affiliations: (array) $p['affiliations'], disabledAtSource: false, attributes: [],
        );
    }

    public function markWantsLink(SessionInterface $session): void
    {
        $pending = $this->get($session);
        if ($pending !== null) {
            $pending['wantsLink'] = true;
            $session->set(self::KEY, $pending);
        }
    }

    public function clear(SessionInterface $session): void
    {
        $session->remove(self::KEY);
    }
}
