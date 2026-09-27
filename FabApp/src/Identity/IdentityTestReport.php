<?php

declare(strict_types=1);

namespace App\Identity;

use Symfony\Component\HttpFoundation\Session\SessionInterface;

/**
 * S196 — ce qu'a vu le dernier « Tester » d'un fournisseur : les attributs
 * REÇUS, le profil qu'on en tire, et ce que FabOS en ferait (lier, créer,
 * refuser — et pourquoi). Ou, en cas d'échec, la raison ET le détail technique.
 *
 * ⚠️ Rangé dans la SESSION de l'administrateur qui teste, en tableaux simples :
 * rien n'est écrit en base, rien n'est visible d'un autre compte. Le test ne
 * connecte personne.
 */
final class IdentityTestReport
{
    private const PREFIX = 'identity_test_';

    public function store(SessionInterface $session, string $key, ?OidcResult $result = null, ?IdentityDecision $decision = null, ?IdentityRefusal $refusal = null): void
    {
        $report = ['at' => time(), 'ok' => $refusal === null];
        if ($refusal !== null) {
            $report['refusal'] = ['key' => $refusal->reasonKey, 'params' => $refusal->params, 'detail' => $refusal->detail];
        }
        if ($result !== null) {
            $p = $result->profile;
            $report['attributes'] = self::flatten($result->claims);
            $report['profile'] = [
                'subject' => $p->subject, 'email' => $p->email, 'emailVerified' => $p->emailVerified,
                'firstName' => $p->firstName, 'lastName' => $p->lastName, 'displayName' => $p->displayName,
                'affiliations' => $p->affiliations,
            ];
        }
        if ($decision !== null) {
            $report['decision'] = [
                'outcome' => $decision->outcome, 'userLabel' => $decision->userLabel, 'email' => $decision->email,
                'firstName' => $decision->firstName, 'lastName' => $decision->lastName,
                'reason' => $decision->reason, 'notes' => $decision->notes, 'needs' => $decision->needs,
            ];
        }
        $session->set(self::PREFIX . $key, $report);
    }

    /** @return array<string, mixed>|null */
    public function get(SessionInterface $session, string $key): ?array
    {
        $report = $session->get(self::PREFIX . $key);

        return \is_array($report) ? $report : null;
    }

    /**
     * Des claims imbriqués (`address`, rôles…) deviennent des lignes lisibles.
     *
     * @param array<string, mixed> $claims
     *
     * @return array<string, string>
     */
    private static function flatten(array $claims): array
    {
        $out = [];
        foreach ($claims as $name => $value) {
            $out[(string) $name] = \is_scalar($value) || $value === null
                ? var_export($value, true)
                : (string) json_encode($value, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES);
        }
        ksort($out);

        return $out;
    }
}
