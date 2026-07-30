<?php

namespace App\Portal;

use App\Service\SiteSettingService;

/**
 * The settings a portal is allowed to give its own answer to.
 *
 * ⚠️ **The list is short for one structural reason, and it is the thing to
 * understand before adding to it: a portal is resolved from the request's
 * hostname.** So a setting can only be overridden per portal if it is read
 * *during a request*. Anything read outside one resolves at the global scope no
 * matter what a portal's row says:
 *
 *  - **Mail sender identity, and `public_base_url`** are read by the queue
 *    worker (`messenger:consume`) when it renders and sends a message. That
 *    process has no request and therefore no hostname, so a per-portal sender
 *    would be stored, shown in the admin, and never once used. A setting nothing
 *    reads is worse than no setting — it is a promise the app does not keep — so
 *    these are deliberately absent rather than offered and quietly ignored.
 *  - Everything below is read while rendering a page, so the portal is known.
 *
 * The second rule is the tri-state: **null means "no override"**, not "empty".
 * Blanking a field removes the row and the portal inherits the site-wide value
 * again, which is why every value here is nullable all the way down.
 */
final class PortalOverrides
{
    /**
     * @var list<array{key: string, label: string, help: string, type: string}>
     */
    public const FIELDS = [
        [
            'key' => 'portal_logo_path',
            'label' => 'Logo',
            'help' => 'Nom du fichier dans public/images/ — par exemple mon-logo.png. Vide : le logo du site.',
            'type' => 'text',
        ],
        [
            'key' => 'portal_primary_color',
            'label' => 'Couleur principale',
            'help' => 'Code hexadécimal, par exemple #1D4ED8. Vide : la couleur du site.',
            'type' => 'color',
        ],
        [
            'key' => 'default_locale',
            'label' => 'Langue par défaut',
            'help' => 'fr, en, de, es ou it. Vide : la langue du site.',
            'type' => 'text',
        ],
        [
            'key' => 'lab_pages_nav_label',
            'label' => 'Nom du menu « Lab »',
            'help' => 'Le libellé du groupe de pages dans le menu. Vide : celui du site.',
            'type' => 'text',
        ],
        [
            'key' => 'alert_banner_enabled',
            'label' => 'Bandeau d\'alerte affiché',
            'help' => '1 pour afficher, 0 pour masquer. Vide : comme le site.',
            'type' => 'text',
        ],
        [
            'key' => 'alert_banner_text',
            'label' => 'Texte du bandeau d\'alerte',
            'help' => 'Vide : le texte du site.',
            'type' => 'text',
        ],
    ];

    public function __construct(private readonly SiteSettingService $settings)
    {
    }

    /**
     * Each field with this portal's own value (null when it inherits) and the
     * site-wide value it would fall back to, so the screen can show what
     * "inherit" actually means rather than an empty box.
     *
     * @return list<array{key: string, label: string, help: string, type: string, value: ?string, global: ?string}>
     */
    public function forPortal(int $portalId): array
    {
        $fields = [];
        foreach (self::FIELDS as $field) {
            $fields[] = [
                ...$field,
                'value' => $this->settings->getForScope($portalId, $field['key']),
                'global' => $this->settings->getForScope(PortalContext::GLOBAL_SCOPE, $field['key']),
            ];
        }

        return $fields;
    }

    /**
     * @param array<string, string> $submitted raw form values, keyed by setting key
     *
     * @return int how many keys this portal now overrides
     */
    public function save(int $portalId, array $submitted): int
    {
        $overridden = 0;
        foreach (self::FIELDS as $field) {
            $value = trim($submitted[$field['key']] ?? '');
            // Blank is not a value, it is the absence of one: the row goes away
            // and the portal follows the site again.
            $this->settings->setForScope($portalId, $field['key'], $value === '' ? null : $value);
            $overridden += $value === '' ? 0 : 1;
        }

        return $overridden;
    }
}
