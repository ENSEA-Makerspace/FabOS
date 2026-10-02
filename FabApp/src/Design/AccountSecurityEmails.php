<?php

declare(strict_types=1);

namespace App\Design;

use App\Entity\Utilisateur;
use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use App\Security\MfaService;
use App\Security\SessionRegistry;
use Symfony\Contracts\Translation\TranslatorInterface;

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
        private readonly TranslatorInterface $translator,
    ) {
    }

    /** @return array{email: string, verified: bool, mfa: ?string, sessions: ?int, masterOn: bool, groups: list<array<string, mixed>>, keptCategories: list<string>} */
    public function for(Utilisateur $user): array
    {
        $userId = $user->getId();
        $state = $userId !== null
            ? $this->preferences->forUser($userId)
            : array_fill_keys(NotificationCategory::OPTOUTABLE, true);
        $staff = array_diff($user->getRoles(), ['ROLE_USER']) !== [];

        // Une ligne = une catégorie réelle. `essential` ⇔ pas dans OPTOUTABLE.
        $line = fn (string $category, string $what) => [
            'category' => $category,
            'what' => $this->translator->trans($what),
            'essential' => !NotificationCategory::isOptOutable($category),
            'received' => $state[$category] ?? true,
        ];

        $groups = [
            ['icon' => 'calendar', 'title' => $this->translator->trans('accsec.group_bookings'), 'lines' => [
                $line(NotificationCategory::BOOKING, 'accsec.what_booking'),
                $line(NotificationCategory::REMINDER, 'accsec.what_reminder'),
            ]],
            ['icon' => 'ticket', 'title' => $this->translator->trans('accsec.group_events'), 'lines' => [
                $line(NotificationCategory::EVENT, 'accsec.what_event'),
            ]],
            ['icon' => 'bell', 'title' => $this->translator->trans('accsec.group_trainings'), 'lines' => [
                $line(NotificationCategory::MESSAGE, 'accsec.what_message'),
            ]],
            ['icon' => 'pin', 'title' => $this->translator->trans('accsec.group_venue'), 'lines' => [
                $line(NotificationCategory::NEWS, 'accsec.what_news'),
                $line(NotificationCategory::GENERAL, 'accsec.what_general'),
            ]],
        ];
        if ($staff) {
            $groups[] = ['icon' => 'tool', 'title' => $this->translator->trans('accsec.group_team'), 'lines' => [
                $line(NotificationCategory::MAINTENANCE, 'accsec.what_maintenance'),
            ]];
        }

        // Les catégories optionnelles que cette personne ne voit pas (ex. la maintenance,
        // réservée à l'équipe) : le formulaire de `/profil` les rejoue telles quelles, sans
        // quoi enregistrer les réglages les couperait en silence.
        $shown = [];
        foreach ($groups as $group) {
            foreach ($group['lines'] as $groupLine) {
                $shown[$groupLine['category']] = true;
            }
        }
        $kept = [];
        foreach (NotificationCategory::OPTOUTABLE as $category) {
            if (!isset($shown[$category]) && ($state[$category] ?? true)) {
                $kept[] = $category;
            }
        }

        return [
            'keptCategories' => $kept,
            'email' => $user->getEmail(),
            'verified' => $user->isVerified(),
            'mfa' => $this->mfa->isReady() ? $this->mfa->status($user) : null,
            'sessions' => $this->sessions->isReady() ? \count($this->sessions->aliveFor($user)) : null,
            'masterOn' => $user->isNotificationEmail(),
            'groups' => $groups,
        ];
    }
}
