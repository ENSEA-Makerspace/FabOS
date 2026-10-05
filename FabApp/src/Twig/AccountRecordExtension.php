<?php

declare(strict_types=1);

namespace App\Twig;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Service\CharterAcceptances;
use App\Service\UserWarnings;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Ce que la fiche admin d'une personne dit d'elle sans que son contrôleur ait à
 * le calculer (S208, S209) — `AdminController` est trop disputé pour y ajouter
 * des variables.
 *
 *   - `user_warnings(user)` : null si la fonction est éteinte ou la migration
 *     absente (le gabarit n'affiche alors RIEN), sinon {rows, reasons}.
 *   - `charter_state(user)` : null si la charte n'est pas disponible, sinon
 *     {acceptedAt} (null tant qu'elle n'est pas acceptée).
 */
final class AccountRecordExtension extends AbstractExtension
{
    public function __construct(
        private readonly UserWarnings $warnings,
        private readonly CharterAcceptances $charter,
        private readonly SiteFeatureService $features,
    ) {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('user_warnings', $this->userWarnings(...)),
            new TwigFunction('charter_state', $this->charterState(...)),
        ];
    }

    /** @return array{rows: list<array<string, mixed>>, reasons: list<array<string, mixed>>}|null */
    public function userWarnings(Utilisateur $user): ?array
    {
        if (!$this->features->allowsSurface('warnings') || !$this->warnings->isReady()) {
            return null;
        }

        return ['rows' => $this->warnings->forUser((int) $user->getId()), 'reasons' => $this->warnings->reasons(true)];
    }

    /** @return array{acceptedAt: ?\DateTimeImmutable}|null */
    public function charterState(Utilisateur $user): ?array
    {
        if (!$this->charter->isAvailable()) {
            return null;
        }

        return ['acceptedAt' => $this->charter->acceptedAt((int) $user->getId())];
    }
}
