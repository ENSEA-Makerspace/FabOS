<?php

namespace App\Service;

/**
 * The one list of languages this installation speaks.
 *
 * S134c found four copies of it, and two of them were wrong: the profile page
 * (`profil.html.twig`) and its save path (`SiteController`) offered only `fr` and
 * `en`, as did the admin's create-user form — while `config/packages/translation.yaml`
 * enables five and every catalogue carries five. A member whose language was German,
 * Spanish or Italian could not choose it, and a POST that tried was rejected with
 * "Langue invalide."
 *
 * The enabled set therefore comes from `%kernel.enabled_locales%` — the same value
 * the framework itself uses — and this class only supplies the display names. Adding
 * a sixth language becomes one line in `translation.yaml` plus one line in NAMES,
 * rather than a hunt through controllers and form types.
 *
 * ⚠️ Names are deliberately endonyms ("Deutsch", not "German"): a language picker is
 * read by someone who does not necessarily read the current interface language.
 */
final class LocaleCatalog
{
    /** @var array<string, string> */
    private const NAMES = [
        'fr' => 'Français',
        'en' => 'English',
        'es' => 'Español',
        'de' => 'Deutsch',
        'it' => 'Italiano',
    ];

    /** @param string[] $enabledLocales */
    public function __construct(private readonly array $enabledLocales)
    {
    }

    /**
     * Locale code => display name, in the order `translation.yaml` lists them.
     *
     * A locale enabled without a name here falls back to its own code rather than
     * disappearing: an unlabelled option is a bug worth seeing, a missing one is not.
     *
     * @return array<string, string>
     */
    public function choices(): array
    {
        $out = [];
        foreach ($this->enabledLocales as $code) {
            $out[$code] = self::NAMES[$code] ?? $code;
        }

        return $out;
    }

    /** @return string[] */
    public function codes(): array
    {
        return array_values($this->enabledLocales);
    }

    public function supports(string $locale): bool
    {
        return in_array($locale, $this->enabledLocales, true);
    }
}
