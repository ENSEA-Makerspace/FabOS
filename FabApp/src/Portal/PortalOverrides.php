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
            // A route name chosen from a list, never a path. The options are built
            // per request from the enabled features (see PortalHome), so this is
            // the one field whose choices are not fixed.
            'key' => PortalHome::SETTING,
            'label' => 'Page d\'accueil',
            'help' => 'Où arrive un visiteur sur ce portail. Vide : la page d\'accueil habituelle, avec ses blocs.',
            'type' => 'select',
            'error' => 'La page d\'accueil doit être choisie dans la liste : seules les fonctionnalités actives peuvent servir de porte d\'entrée.',
        ],
        [
            'key' => 'portal_logo_path',
            'label' => 'Logo',
            'help' => 'Nom du fichier dans public/images/ — par exemple mon-logo.png. Vide : le logo du site.',
            'type' => 'text',
            'pattern' => '/^[A-Za-z0-9._-]+\.(png|jpe?g|webp|svg)$/i',
            'error' => 'Le logo doit être un nom de fichier de public/images/, par exemple mon-logo.png — pas un chemin ni une adresse.',
        ],
        [
            'key' => 'portal_primary_color',
            'label' => 'Couleur principale',
            'help' => 'Code hexadécimal, par exemple #1D4ED8. Vide : la couleur du site.',
            'type' => 'color',
            'pattern' => '/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i',
            'error' => 'La couleur doit être un code hexadécimal complet, par exemple #1D4ED8 — sans point ni texte autour.',
        ],
        [
            'key' => 'default_locale',
            'label' => 'Langue par défaut',
            'help' => 'fr, en, de, es ou it. Vide : la langue du site.',
            'type' => 'text',
            'pattern' => '/^(fr|en|de|es|it)$/',
            'error' => 'La langue doit être fr, en, de, es ou it.',
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
            'pattern' => '/^[01]$/',
            'error' => 'Le bandeau d\'alerte se règle avec 1 (affiché) ou 0 (masqué).',
        ],
        [
            'key' => 'alert_banner_text',
            'label' => 'Texte du bandeau d\'alerte',
            'help' => 'Vide : le texte du site.',
            'type' => 'text',
        ],
    ];

    public function __construct(
        private readonly SiteSettingService $settings,
        private readonly PortalHome $home,
    ) {
    }

    /**
     * Each field with this portal's own value (null when it inherits) and the
     * site-wide value it would fall back to, so the screen can show what
     * "inherit" actually means rather than an empty box.
     *
     * @return list<array{key: string, label: string, help: string, type: string, value: ?string, global: ?string, options: array<string, string>}>
     */
    public function forPortal(int $portalId): array
    {
        $fields = [];
        foreach (self::FIELDS as $field) {
            $fields[] = [
                ...$field,
                'value' => $this->settings->getForScope($portalId, $field['key']),
                'global' => $this->settings->getForScope(PortalContext::GLOBAL_SCOPE, $field['key']),
                // ⚠️ These are the options for the *site's* current feature set, not
                // the portal's. A portal that switches a feature off after choosing
                // it as its home is not corrected here — it does not need to be,
                // because PortalHome re-checks on every request and falls back to
                // the ordinary homepage. Nothing breaks; the choice simply stops
                // applying until the feature comes back.
                'options' => $field['type'] === 'select' ? $this->home->options() : [],
            ];
        }

        return $fields;
    }

    /**
     * ⚠️ **Everything is checked before anything is written.**
     *
     * The readers already refuse a malformed value — the accent colour is
     * validated on read because it lands in a `<style>` block — but refusing it
     * there and accepting it here produces the worst outcome available: the
     * setting is stored, shown back in the form as though it took effect, and
     * silently does nothing. That happened for real, to `#1D4ED8.` with a
     * trailing dot: saved, displayed, never once applied, and nothing anywhere
     * said why. A stored setting nothing reads is a promise the app does not
     * keep. So a bad value is a refusal with a sentence explaining it, and a
     * partial save is not offered either — one bad field must not leave the
     * other five half-applied.
     *
     * @param array<string, string> $submitted raw form values, keyed by setting key
     *
     * @throws \RuntimeException with a message meant for the admin to read
     *
     * @return int how many keys this portal now overrides
     */
    public function save(int $portalId, array $submitted): int
    {
        $values = [];
        $errors = [];

        foreach (self::FIELDS as $field) {
            $value = trim($submitted[$field['key']] ?? '');

            // Blank is not a value, it is the absence of one: the row goes away
            // and the portal follows the site again. Nothing to validate.
            if ($value === '') {
                $values[$field['key']] = null;
                continue;
            }

            $valid = match (true) {
                // The home page is a route name, and the only acceptable ones are
                // the pages that exist right now.
                $field['key'] === PortalHome::SETTING => isset($this->home->options()[$value]),
                isset($field['pattern']) => preg_match($field['pattern'], $value) === 1,
                default => true,
            };

            if ($valid) {
                $values[$field['key']] = $value;
            } else {
                $errors[] = $field['error'] ?? sprintf('« %s » n\'a pas une valeur valide.', $field['label']);
            }
        }

        if ($errors !== []) {
            throw new \RuntimeException(implode(' ', $errors));
        }

        $overridden = 0;
        foreach ($values as $key => $value) {
            $this->settings->setForScope($portalId, $key, $value);
            $overridden += $value === null ? 0 : 1;
        }

        return $overridden;
    }
}
