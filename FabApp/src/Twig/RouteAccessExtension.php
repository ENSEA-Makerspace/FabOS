<?php

namespace App\Twig;

use App\Security\RouteAccessChecker;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * `can_reach('some_route')` — would the firewall let this person open it?
 *
 * ⚠️ The logic moved to `App\Security\RouteAccessChecker` (S82) so `NavBuilder`
 * could ask the same question when it builds the admin sidebar in PHP. This is a
 * thin exposure of it and must stay thin: a second implementation of "can they
 * reach it" is the drift both are here to prevent.
 */
final class RouteAccessExtension extends AbstractExtension
{
    public function __construct(private readonly RouteAccessChecker $access)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('can_reach', $this->access->canReach(...)),
        ];
    }
}
