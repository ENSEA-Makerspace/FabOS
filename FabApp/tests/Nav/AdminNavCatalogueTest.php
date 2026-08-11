<?php

namespace App\Tests\Nav;

use PHPUnit\Framework\TestCase;
use Symfony\Component\Yaml\Yaml;

/**
 * The admin navigation says what it means, in every language.
 *
 * ⚠️ **This replaces the two `FeatureWorkspaceRegistryTest` cases that asserted
 * the workspace tabs.** S130b deleted that second sub-navigation, so those tests
 * described machinery that no longer exists — but the guarantees behind them do:
 * every destination in the one remaining navigation has to name itself, and name
 * itself in all five catalogues.
 *
 * ⚠️ **Why a source-text test and not a service test.** `NavBuilder::admin()`
 * needs `RouteAccessChecker`, which is `final` and therefore not mockable, and
 * the labels are literals in the method bodies. Reading the declarations is what
 * is available; it happens to be exactly the right altitude for this question,
 * because the failure being guarded against is a key that was written in one
 * place and never added to the other five.
 *
 * A missing key is not an exception. Twig renders the key itself, so the screen
 * reads `admin_nav.entry.app_admin_machines` — visible only to someone who opens
 * that page in that language, which is how the hardcoded French survived in this
 * file for as long as it did.
 */
final class AdminNavCatalogueTest extends TestCase
{
    private const LOCALES = ['fr', 'en', 'de', 'es', 'it'];

    /** @return list<string> */
    private function referencedKeys(): array
    {
        $source = file_get_contents(__DIR__ . '/../../src/Nav/NavBuilder.php');
        self::assertNotFalse($source);

        preg_match_all("/'(admin_nav\.[a-z_.]+)'/", $source, $matches);

        return array_values(array_unique($matches[1]));
    }

    /** @return array<string, mixed> */
    private function catalogue(string $locale): array
    {
        return Yaml::parseFile(__DIR__ . '/../../translations/messages.' . $locale . '.yaml');
    }

    private function resolve(array $catalogue, string $key): ?string
    {
        $value = $catalogue;
        foreach (explode('.', $key) as $segment) {
            if (!is_array($value) || !array_key_exists($segment, $value)) {
                return null;
            }
            $value = $value[$segment];
        }

        return is_string($value) ? $value : null;
    }

    public function testEveryNavigationLabelIsTranslatedInAllFiveLocales(): void
    {
        $keys = $this->referencedKeys();
        self::assertGreaterThan(30, \count($keys), 'NavBuilder should declare its labels as catalogue keys');

        foreach (self::LOCALES as $locale) {
            $catalogue = $this->catalogue($locale);
            foreach ($keys as $key) {
                $value = $this->resolve($catalogue, $key);
                self::assertNotNull($value, sprintf('%s is missing from messages.%s.yaml', $key, $locale));
                self::assertNotSame('', trim($value), sprintf('%s is empty in messages.%s.yaml', $key, $locale));
            }
        }
    }

    /**
     * ⚠️ The positive assertion, not a count. Until S130b every one of these
     * labels was a French literal passed to `|trans` — which resolves to itself,
     * so the sidebar and its strip stayed French on an English, German, Spanish
     * or Italian account while the catalogues sat complete beside them. A count
     * of keys would have passed the whole time.
     */
    public function testNoAdminNavigationLabelIsStillAHardcodedString(): void
    {
        $source = file_get_contents(__DIR__ . '/../../src/Nav/NavBuilder.php');
        self::assertNotFalse($source);

        preg_match_all("/\\\$this->adminItem\(\s*'([^']+)'/", $source, $items);
        preg_match_all("/\\\$this->adminSection\(\s*'([^']+)'/", $source, $sections);

        $admin = array_merge($items[1], $sections[1]);
        self::assertNotEmpty($admin);

        foreach ($admin as $label) {
            self::assertStringStartsWith(
                'admin_nav.',
                $label,
                sprintf('"%s" is a literal; admin navigation labels must be catalogue keys', $label),
            );
        }

        // The section labels declared in `adminByFeature()`'s array reach the
        // sidebar the same way but never pass through `adminSection()` in the
        // source text, so they need their own sweep. ⚠️ `header()` and
        // `safeDestinations()` also use `'label' =>`, with public-side keys like
        // `nav.home` — legitimate, just not this namespace. The shape of a
        // catalogue key is what separates them from `'Équipement'`.
        preg_match_all("/'label' => '([^']+)'/", $source, $labels);
        foreach ($labels[1] as $label) {
            self::assertMatchesRegularExpression(
                '/^[a-z][a-z0-9_]*(\.[a-z0-9_]+)+$/',
                $label,
                sprintf('"%s" is a literal; navigation labels must be catalogue keys', $label),
            );
        }
    }

    /**
     * The entry keys are named after the route they lead to, so a destination
     * that gains a navigation entry cannot be given a label belonging to another
     * screen — and a reader can go from the strip to the router in one step.
     */
    public function testEntryKeysAreNamedAfterTheirRoute(): void
    {
        $source = file_get_contents(__DIR__ . '/../../src/Nav/NavBuilder.php');
        self::assertNotFalse($source);

        preg_match_all("/\\\$this->adminItem\(\s*'(admin_nav\.entry\.[a-z_]+)',\s*'([a-z_]+)'/", $source, $matches, PREG_SET_ORDER);
        self::assertNotEmpty($matches);

        foreach ($matches as [, $key, $route]) {
            self::assertSame('admin_nav.entry.' . $route, $key);
        }
    }
}
