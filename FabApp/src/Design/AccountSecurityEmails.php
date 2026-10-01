<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use App\Security\MfaService;
use App\Security\SessionRegistry;

/**
 * « Sécurité et e-mails » (proposition du 2026-10-01, planches `05-profil-securite`
 * et `06-preferences-email`) : ce que `/profil#settings` calcule déjà
 * (`MfaService::status`, `SessionRegistry::aliveFor`, `NotificationPreferences::forUser`),
 * posé en lignes et regroupé par domaine. Lecture seule, rien n'est recalculé.
 *
 * Les e-mails ESSENTIELS sont les catégories absentes de `NotificationCategory::OPTOUTABLE`
 * (réservation, événement) : ils n'ont pas d'interrupteur, donc pas de case ici.
 */
final class AccountSecurityEmails
{
    public function __construct(
        private readonly MfaService $mfa,
        private readonly SessionRegistry $sessions,
        private readonly NotificationPreferences $preferences,
    ) {
    }

    /** @return array{email: string, verified: bool, mfa: ?string, sessions: ?int, masterOn: bool, groups: list<array<string, mixed>>} */
    public function for(Utilisateur $user): array
    {
        $userId = $user->getId();
        $state = $userId !== null
            ? $this->preferences->forUser($userId)
            : array_fill_keys(NotificationCategory::OPTOUTABLE, true);
        $staff = array_diff($user->getRoles(), ['ROLE_USER']) !== [];

        // Une ligne = une catégorie réelle. `essential` ⇔ pas dans OPTOUTABLE.
        $line = static fn (string $category, string $what) => [
            'category' => $category,
            'what' => $what,
            'essential' => !NotificationCategory::isOptOutable($category),
            'received' => $state[$category] ?? true,
        ];

        $groups = [
            ['icon' => 'calendar', 'title' => 'Réservations et prêts', 'lines' => [
                $line(NotificationCategory::BOOKING, 'Confirmations, réponses de l’équipe et changements de vos réservations.'),
                $line(NotificationCategory::REMINDER, 'Un rappel avant une réservation, et pour un emprunt à rendre.'),
            ]],
            ['icon' => 'ticket', 'title' => 'Événements', 'lines' => [
                $line(NotificationCategory::EVENT, 'Inscription, liste d’attente et « une place s’est libérée ».'),
            ]],
            ['icon' => 'bell', 'title' => 'Formations', 'lines' => [
                $line(NotificationCategory::MESSAGE, 'Une copie des messages échangés avec l’équipe de formation (le fil reste lisible dans FabOS).'),
            ]],
            ['icon' => 'pin', 'title' => 'Vie du lieu', 'lines' => [
                $line(NotificationCategory::NEWS, 'Résumés et annonces du lieu.'),
                $line(NotificationCategory::GENERAL, 'Les messages qui n’entrent dans aucune autre catégorie.'),
            ]],
        ];
        if ($staff) {
            $groups[] = ['icon' => 'tool', 'title' => 'Équipe', 'lines' => [
                $line(NotificationCategory::MAINTENANCE, 'Alertes de maintenance en retard (réservé à l’équipe).'),
            ]];
        }

        return [
            'email' => $user->getEmail(),
            'verified' => $user->isVerified(),
            'mfa' => $this->mfa->isReady() ? $this->mfa->status($user) : null,
            'sessions' => $this->sessions->isReady() ? \count($this->sessions->aliveFor($user)) : null,
            'masterOn' => $user->isNotificationEmail(),
            'groups' => $groups,
        ];
    }
}
