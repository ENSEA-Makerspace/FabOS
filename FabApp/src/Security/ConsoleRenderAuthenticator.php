<?php

namespace App\Security;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Security\Core\Authentication\Token\TokenInterface;
use Symfony\Component\Security\Core\Exception\AuthenticationException;
use Symfony\Component\Security\Http\Authenticator\AbstractAuthenticator;
use Symfony\Component\Security\Http\Authenticator\Passport\Badge\UserBadge;
use Symfony\Component\Security\Http\Authenticator\Passport\Passport;
use Symfony\Component\Security\Http\Authenticator\Passport\SelfValidatingPassport;

/**
 * Signs in the request that `app:render` makes to itself, and nothing else.
 *
 * This replaces `LOCAL_ADMIN_BYPASS`, which existed for the same reason — an
 * agent with no password needs to see whether an admin screen renders — and was
 * a genuine authentication bypass reachable over HTTP. It was held off by three
 * conditions, one of them debug mode, so it went inert when the site moved to
 * `prod` and took the only way of inspecting admin pages with it.
 *
 * **The difference that matters: this one cannot be reached by a request at
 * all.** Arming it is not a flag, an IP range or an environment variable — all
 * of which are things a request travels through and an operator can leave on by
 * accident. It is a method call, on this exact service instance, made by the
 * console command a line before it hands the request to the kernel:
 *
 *   1. `renderAs()` must have been called in this process. Only `app:render`
 *      does that, and it needs the service, which means it needs to already be
 *      executing inside the application.
 *   2. `PHP_SAPI === 'cli'`, belt and braces. A web request is never `cli` —
 *      the built-in server is `cli-server`, php-fpm is `fpm-fcgi` — so even a
 *      future caller that armed this in a request handler would achieve nothing.
 *
 * Which leaves the honest summary: this grants nothing that a shell on the box
 * did not already grant, because a shell can edit the database directly. That is
 * the bar an inspection aid has to clear, and the old one did not.
 */
final class ConsoleRenderAuthenticator extends AbstractAuthenticator
{
    /** null = disarmed, and it is disarmed until a command says otherwise. */
    private ?string $identifier = null;

    public function __construct(private readonly UtilisateurRepository $people)
    {
    }

    /**
     * Arms the next request handled in this process.
     *
     * @param string|null $email the account to render as, or null for the first admin
     *
     * @return string the identifier that will be used, for the command to report
     */
    public function renderAs(?string $email = null): string
    {
        if ($email === null) {
            $admin = $this->firstAdmin();
            if ($admin === null) {
                throw new \RuntimeException('No account holds ROLE_ADMIN, so there is nobody to render as.');
            }
            $email = $admin->getUserIdentifier();
        }

        return $this->identifier = $email;
    }

    public function supports(Request $request): ?bool
    {
        return $this->identifier !== null && \PHP_SAPI === 'cli';
    }

    public function authenticate(Request $request): Passport
    {
        if ($this->identifier === null) {
            throw new AuthenticationException('app:render was not armed.');
        }

        return new SelfValidatingPassport(new UserBadge($this->identifier));
    }

    public function onAuthenticationSuccess(Request $request, TokenInterface $token, string $firewallName): ?Response
    {
        return null;
    }

    public function onAuthenticationFailure(Request $request, AuthenticationException $exception): ?Response
    {
        return null;
    }

    private function firstAdmin(): ?Utilisateur
    {
        foreach ($this->people->findAll() as $person) {
            if ($person instanceof Utilisateur && \in_array('ROLE_ADMIN', $person->getRoles(), true)) {
                return $person;
            }
        }

        return null;
    }
}
