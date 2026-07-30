<?php

namespace App\Twig;

use App\Nav\NavBuilder;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes the assembled navigation so the header and footer render a list rather
 * than carrying one hand-written condition per feature.
 */
final class NavExtension extends AbstractExtension
{
    public function __construct(private readonly NavBuilder $nav)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('nav_header', $this->nav->header(...)),
            new TwigFunction('nav_footer', $this->nav->footer(...)),
        ];
    }
}
