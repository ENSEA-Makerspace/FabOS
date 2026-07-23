<?php

namespace App\Security;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Psr\Log\LoggerInterface;
use Symfony\Component\HttpFoundation\IpUtils;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Security\Core\Authentication\Token\TokenInterface;
use Symfony\Component\Security\Core\Exception\AuthenticationException;
use Symfony\Component\Security\Http\Authenticator\AbstractAuthenticator;
use Symfony\Component\Security\Http\Authenticator\Passport\Badge\UserBadge;
use Symfony\Component\Security\Http\Authenticator\Passport\Passport;
use Symfony\Component\Security\Http\Authenticator\Passport\SelfValidatingPassport;

/**
 * TEMPORARY DEBUG AID — signs you in as an admin without a password, so the
 * admin screens can be rendered and inspected from inside the container.
 *
 * ⚠️ This is an authentication bypass. It is switched off unless **all three**
 * of the following hold, and each one alone is enough to keep it off:
 *
 *   1. LOCAL_ADMIN_BYPASS=1 is set in the environment (absent by default, and
 *      absent from the committed .env — it has to be added by hand);
 *   2. the request arrives from loopback, checked against REMOTE_ADDR with
 *      proxy headers ignored, so a request forwarded by the reverse proxy can
 *      never qualify no matter what it claims about itself;
 *   3. the kernel is in debug mode, so a production build cannot honour it at
 *      all even if someone copies the flag across.
 *
 * The loopback rule is what makes this safe on a box whose port 8000 is
 * published to the internet through a reverse proxy: proxied traffic presents
 * the proxy's address, not 127.0.0.1, so only something already executing
 * inside the container can use this.
 *
 * DELETE THIS CLASS, and its entry in security.yaml, once the admin templates
 * are done being worked on.
 */
final class LocalAdminAuthenticator extends AbstractAuthenticator
{
    private const FLAG = 'LOCAL_ADMIN_BYPASS';

    /** RFC-reserved loopback ranges; nothing routable is included. */
    private const LOOPBACK = ['127.0.0.0/8', '::1'];

    public function __construct(
        private readonly UtilisateurRepository $people,
        private readonly bool $debug,
        private readonly ?LoggerInterface $logger = null,
    ) {
    }

    public function supports(Request $request): ?bool
    {
        if (!$this->debug || ($_ENV[self::FLAG] ?? $_SERVER[self::FLAG] ?? '0') !== '1') {
            return false;
        }

        // Deliberately NOT $request->getClientIp(): that consults X-Forwarded-For
        // when trusted proxies are configured, which is attacker-influenced. The
        // raw connecting address is the only thing that cannot be spoofed from
        // outside the box.
        $remoteAddr = (string) $request->server->get('REMOTE_ADDR', '');
        if ($remoteAddr === '' || !IpUtils::checkIp($remoteAddr, self::LOOPBACK)) {
            return false;
        }

        // Only bother on the screens this exists to inspect, and never fight an
        // already-authenticated session.
        $path = $request->getPathInfo();

        return (str_starts_with($path, '/admin') || str_starts_with($path, '/staff'))
            && $request->getSession()->get('_security_main') === null;
    }

    public function authenticate(Request $request): Passport
    {
        $admin = $this->firstAdmin();
        if ($admin === null) {
            throw new AuthenticationException('LOCAL_ADMIN_BYPASS is on but no admin account exists.');
        }

        $this->logger?->warning('LOCAL_ADMIN_BYPASS: signed in as {email} from loopback without a password.', [
            'email' => $admin->getEmail(),
            'path' => $request->getPathInfo(),
        ]);

        return new SelfValidatingPassport(new UserBadge($admin->getUserIdentifier()));
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
            if ($person instanceof Utilisateur && in_array('ROLE_ADMIN', $person->getRoles(), true)) {
                return $person;
            }
        }

        return null;
    }
}
