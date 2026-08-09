<?php

namespace App\Service;

final class ThemeManager
{
    private const DRAFT_KEY = 'theme_draft_v1';

    public function __construct(private readonly SiteSettingService $settings) {}

    /** @return array{orgName: string, venueLabel: string, primaryColor: string, logoPath: string} */
    public function published(): array
    {
        return [
            'orgName' => $this->settings->getOrgName(),
            'venueLabel' => $this->settings->getVenueLabel(),
            'primaryColor' => $this->settings->get('portal_primary_color') ?? '',
            'logoPath' => $this->settings->get('portal_logo_path') ?? '',
        ];
    }

    public function draft(): array
    {
        $decoded = json_decode($this->settings->get(self::DRAFT_KEY) ?? '', true);

        return is_array($decoded) ? $decoded + $this->published() : $this->published();
    }

    /** @param array<string, mixed> $input */
    public function saveDraft(array $input): array
    {
        $draft = [
            'orgName' => mb_substr(trim((string) ($input['orgName'] ?? '')), 0, 80),
            'venueLabel' => mb_substr(trim((string) ($input['venueLabel'] ?? '')), 0, 80),
            'primaryColor' => trim((string) ($input['primaryColor'] ?? '')),
            'logoPath' => trim((string) ($input['logoPath'] ?? '')),
        ];
        if ($draft['orgName'] === '' || $draft['venueLabel'] === '') {
            throw new \InvalidArgumentException('Les deux noms publics sont obligatoires.');
        }
        if ($draft['primaryColor'] !== '' && preg_match('/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i', $draft['primaryColor']) !== 1) {
            throw new \InvalidArgumentException('La couleur doit être un code hexadécimal, par exemple #9E1B56.');
        }
        if ($draft['logoPath'] !== '' && preg_match('/^[A-Za-z0-9._-]+\.(png|jpe?g|webp|svg)$/i', $draft['logoPath']) !== 1) {
            throw new \InvalidArgumentException('Le logo doit être un nom de fichier image dans public/images/.');
        }
        $this->settings->set(self::DRAFT_KEY, json_encode($draft, JSON_THROW_ON_ERROR));

        return $draft;
    }

    public function publish(): void
    {
        $draft = $this->draft();
        $this->settings->set('org_name', $draft['orgName']);
        $this->settings->set('venue_label', $draft['venueLabel']);
        $this->settings->set('portal_primary_color', $draft['primaryColor']);
        $this->settings->set('portal_logo_path', $draft['logoPath']);
    }

    public function discardDraft(): void
    {
        $this->settings->set(self::DRAFT_KEY, json_encode($this->published(), JSON_THROW_ON_ERROR));
    }
}
