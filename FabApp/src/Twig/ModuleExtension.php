<?php

namespace App\Twig;

use App\Service\ModuleService;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes {{ module_enabled('badges') }} to templates so nav links / sections
 * can be hidden when an optional module is turned off in the admin.
 */
final class ModuleExtension extends AbstractExtension
{
    public function __construct(private readonly ModuleService $modules)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('module_enabled', $this->modules->isEnabled(...)),
        ];
    }
}
