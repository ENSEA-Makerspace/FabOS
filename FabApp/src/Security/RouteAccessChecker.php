<?php

namespace App\Security;

use Symfony\Component\DependencyInjection\Attribute\Autowire;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Security\Core\Authorization\AuthorizationCheckerInterface;
use Symfony\Component\Security\Http\AccessMapInterface;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * Would the firewall let THIS person open that route?
 *
 * The logic used to live only in `RouteAccessExtension`, which made it available
 * to templates and to nothing else. `NavBuilder` needs exactly the same answer —
 * the admin sidebar is built in PHP now (S82) and has to hide the entries a
 * staff member cannot reach — and the one thing that must not happen is a second
 * copy of it. Both callers share this.
 *
 * ⚠️ It asks the same `access_control` map the firewall consults, rather than
 * naming a role. `ROLE_ADMIN` does not imply `ROLE_STAFF` in this app, so any
 * hand-written role test drifts from the firewall the first time a line moves.
 *
 * ⚠️ Presentation only. Hiding an entry hides an entry; the firewall is still
 * what refuses the request.
 */
final class RouteAccessChecker
{
    public function __construct(
        private readonly UrlGeneratorInterface $urls,
        private readonly AuthorizationCheckerInterface $auth,
        // Private service: `access_control` has no public alias, and copying the
        // rules into PHP to avoid asking for it would recreate the drift this
        // class exists to remove.
        #[Autowire(service: 'security.access_map')]
        private readonly AccessMapInterface $accessMap,
    ) {
    }

    /** @var array<string, bool> */
    private array $memo = [];

    /** @param array<string, mixed> $params */
    public function canReach(string $route, array $params = []): bool
    {
        // Same route, same person, same request: the same answer. Memoised in S132
        // because the features screen asks the whole admin navigation fifteen
        // times over — once per feature — and each miss builds a throwaway
        // `Request` to interrogate `access_control`.
        $memoKey = $route . '?' . http_build_query($params);

        return $this->memo[$memoKey] ??= $this->resolve($route, $params);
    }

    /** @param array<string, mixed> $params */
    private function resolve(string $route, array $params): bool
    {
        try {
            $path = $this->urls->generate($route, $params);
        } catch (\Throwable) {
            // An affordance pointing at a route that no longer exists should
            // vanish quietly rather than take the page down with it.
            return false;
        }

        [$attributes] = $this->accessMap->getPatterns(Request::create($path));

        // No rule covers the path: `access_control` is not what guards it, so as
        // far as this question goes it is reachable.
        if ($attributes === null || $attributes === []) {
            return true;
        }

        // Several roles on one `access_control` line mean "any of these" — the
        // firewall decides affirmatively — so `[ROLE_STAFF, ROLE_ADMIN]` must be
        // true for a plain admin *and* for a plain staff member.
        foreach ($attributes as $attribute) {
            try {
                if ($this->auth->isGranted($attribute)) {
                    return true;
                }
            } catch (\Throwable) {
                // No token yet (sub-requests, console rendering): treat as denied
                // and let the firewall be the one to say so.
                return false;
            }
        }

        return false;
    }
}
