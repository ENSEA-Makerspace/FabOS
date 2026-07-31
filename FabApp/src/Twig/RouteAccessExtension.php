<?php

namespace App\Twig;

use Symfony\Component\DependencyInjection\Attribute\Autowire;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Core\Authorization\AuthorizationCheckerInterface;
use Symfony\Component\Security\Http\AccessMapInterface;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * `can_reach('some_route')` — would the firewall let this person open it?
 *
 * Affordances used to name a role by hand (`is_granted('ROLE_STAFF')`) and hope
 * it matched the rule guarding the page they linked to. It is very easy to get
 * wrong, because **this app has no role hierarchy**: `ROLE_ADMIN` does not imply
 * `ROLE_STAFF`, so a chip gated on `ROLE_STAFF` whose route sits under `/admin`
 * disappears for exactly the people entitled to use it — which is what happened
 * to the event-registrations chip (S30). The reverse is just as bad: a button
 * offered to someone the firewall then bounces to `/login`.
 *
 * So this asks **the same `access_control` map the firewall consults**, rather
 * than keeping a second copy of the rules in template conditions. There is
 * nothing to keep in sync: edit `security.yaml` and every affordance follows.
 *
 * ⚠️ **Still presentation, never permission.** This decides whether to *draw* a
 * link. The firewall decides whether the request succeeds, on every request,
 * and it is what actually stops anyone. A bug here is a usability bug in one
 * direction and a wasted click in the other — never an access-control hole.
 *
 * ⚠️ **It answers "may you reach this URL", not "may you do this thing".**
 * Per-record rules — is this *your* booking, is this event yours to cancel —
 * are the controller's and the voter's business. Do not push those in here.
 */
final class RouteAccessExtension extends AbstractExtension
{
    public function __construct(
        private readonly UrlGeneratorInterface $urls,
        private readonly AuthorizationCheckerInterface $auth,
        // Private service: `access_control` has no public alias, and copying the
        // rules into PHP to avoid asking for it would recreate the drift this
        // whole extension exists to remove.
        #[Autowire(service: 'security.access_map')]
        private readonly AccessMapInterface $accessMap,
    ) {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('can_reach', $this->canReach(...)),
        ];
    }

    /** @param array<string, mixed> $params */
    public function canReach(string $route, array $params = []): bool
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
