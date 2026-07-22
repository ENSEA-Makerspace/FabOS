<?php

namespace App\Twig;

use App\Entity\LabPage;
use App\Repository\LabPageRepository;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes {{ lab_page_nav() }} so the shared header can render the Lab pages
 * hierarchy (top-level pages + their children) directly in the nav bar without
 * every controller having to pass the page list. Fail-safe like the other nav
 * extensions: if the table is missing (fresh install / migration drift) it just
 * returns an empty list instead of breaking every page.
 *
 * @see ModuleExtension, SiteSettingExtension
 */
final class LabPageExtension extends AbstractExtension
{
    public function __construct(private readonly LabPageRepository $labPages)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('lab_page_nav', $this->navPages(...)),
        ];
    }

    /** @return LabPage[] */
    public function navPages(): array
    {
        try {
            return $this->labPages->findTopLevelWithChildren();
        } catch (\Throwable) {
            return [];
        }
    }
}
