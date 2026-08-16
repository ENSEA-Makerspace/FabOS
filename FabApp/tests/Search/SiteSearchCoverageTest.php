<?php

namespace App\Tests\Search;

use PHPUnit\Framework\TestCase;
use Symfony\Component\Yaml\Yaml;

/**
 * Search looks at the whole product, in every language, and only at what is on.
 *
 * 🔴 **The fault this guards against shipped and was found by the operator, not
 * by a test.** `/recherche` searched four kinds of thing out of ten. `usb`
 * returned nothing while a loanable item was named that; a project called
 * `valentin`, a bookable place `D251`, the materials catalogue and the body of
 * every custom page were all equally invisible. Nothing failed — the page
 * rendered a tidy "no results". A search that does not search is indistinguish-
 * able from a search that found nothing, which is exactly why it lasted.
 *
 * ⚠️ **Why a source-text test.** `SiteSearch` takes ten repositories and a
 * feature service; standing that up needs a database, and the question here is
 * not "does the SQL work" but "did someone add an entity and forget to index
 * it". That question is answered by reading the declarations, and it is the
 * altitude the failure actually lives at.
 */
final class SiteSearchCoverageTest extends TestCase
{
    private const LOCALES = ['fr', 'en', 'de', 'es', 'it'];

    /**
     * Every catalogue a member can reach from the public navigation. Adding a
     * public catalogue to FabOS means adding it here **and** to `SiteSearch`.
     */
    private const EXPECTED_GROUPS = [
        'destinations', 'users', 'machines', 'places', 'events', 'formations',
        'badges', 'loans', 'materials', 'creations', 'lab_pages',
    ];

    /** Groups that are not gated through `collect()`: destinations carry their
     *  own per-entry feature, users are gated by ROLE_ADMIN. */
    private const UNGATED_GROUPS = ['destinations', 'users'];

    /** Feature keys `SiteFeatureRegistry` actually declares. */
    private function declaredFeatures(): array
    {
        $source = file_get_contents(__DIR__ . '/../../src/Feature/SiteFeatureRegistry.php');
        self::assertNotFalse($source);

        preg_match_all("/new SiteFeature\(\s*'([a-z_]+)'/", $source, $matches);

        return array_values(array_unique($matches[1]));
    }

    private function source(): string
    {
        $source = file_get_contents(__DIR__ . '/../../src/Search/SiteSearch.php');
        self::assertNotFalse($source);

        return $source;
    }

    /** @return list<array{key: string, label_key: string}> */
    private function declaredGroups(): array
    {
        preg_match_all(
            "/\['key' => '([a-z_]+)', 'label_key' => '([a-z_.]+)'/",
            $this->source(),
            $matches,
            PREG_SET_ORDER,
        );

        return array_map(static fn (array $m): array => ['key' => $m[1], 'label_key' => $m[2]], $matches);
    }

    public function testEveryPublicCatalogueIsSearched(): void
    {
        $keys = array_column($this->declaredGroups(), 'key');

        foreach (self::EXPECTED_GROUPS as $expected) {
            self::assertContains($expected, $keys, sprintf('SiteSearch does not look at %s at all', $expected));
        }
    }

    /**
     * ⚠️ Operator instruction, 2026-08-16: "search should only list toggled on
     * features of the site". A hit whose module is off is worse than no hit —
     * it is a link to a page that 404s.
     */
    public function testEveryRecordGroupIsGatedByARealFeature(): void
    {
        preg_match_all("/collect\(\\\$needle, '([a-z_]+)'/", $this->source(), $matches);
        $gates = array_values(array_unique($matches[1]));

        self::assertCount(\count(self::EXPECTED_GROUPS) - \count(self::UNGATED_GROUPS), $gates);

        $declared = $this->declaredFeatures();
        foreach ($gates as $gate) {
            self::assertContains($gate, $declared, sprintf('%s is not a site feature — the gate would never open', $gate));
        }
    }

    /**
     * ⚠️ A destination's feature is written inline rather than through
     * `collect()`, so it needs its own check — `'feature' => 'bookngs'` would
     * silently hide the entry forever instead of failing.
     */
    public function testEveryDestinationFeatureIsRealOrDeliberatelyNull(): void
    {
        preg_match_all("/'feature' => (null|'[a-z_]+')/", $this->source(), $matches);
        self::assertNotEmpty($matches[1], 'SiteSearch declares no destinations');

        // `bookings` is deliberately not a registry key — it means "any bookable
        // layer at all" and `SiteFeatureService::allowsSurface()` resolves it.
        $declared = array_merge($this->declaredFeatures(), ['bookings']);
        foreach ($matches[1] as $feature) {
            if ($feature === 'null') {
                continue;
            }
            self::assertContains(trim($feature, "'"), $declared, sprintf('%s is not a site feature', $feature));
        }
    }

    /**
     * 🔴 The reason destinations exist at all: `horaires` and `heures` matched
     * nothing, because opening hours are seven rows nobody searches by name.
     * Every language must be able to ask for them in its own words.
     */
    public function testEveryDestinationStringIsTranslatedInAllFiveLocales(): void
    {
        preg_match_all("/'(label|hint|keys)' => '([a-z_.]+)'/", $this->source(), $matches, PREG_SET_ORDER);
        $keys = array_values(array_unique(array_map(static fn (array $m): string => $m[2], $matches)));
        self::assertGreaterThan(15, \count($keys), 'destinations should declare a label, a hint and a synonym list each');

        foreach (self::LOCALES as $locale) {
            $catalogue = Yaml::parseFile(__DIR__ . '/../../translations/messages.' . $locale . '.yaml');
            foreach ($keys as $key) {
                self::assertNotSame('', trim((string) $this->resolve($catalogue, $key)), sprintf('%s is missing or empty in messages.%s.yaml', $key, $locale));
            }
        }
    }

    /**
     * ⚠️ Synonyms are matched by prefix, so a one- or two-letter entry would
     * make its destination match nearly every query and bury the record hits.
     */
    public function testNoDestinationSynonymIsShortEnoughToMatchEverything(): void
    {
        preg_match_all("/'keys' => '([a-z_.]+)'/", $this->source(), $matches);

        foreach (self::LOCALES as $locale) {
            $catalogue = Yaml::parseFile(__DIR__ . '/../../translations/messages.' . $locale . '.yaml');
            foreach ($matches[1] as $key) {
                foreach (explode(',', (string) $this->resolve($catalogue, $key)) as $synonym) {
                    $synonym = trim($synonym);
                    self::assertGreaterThanOrEqual(3, mb_strlen($synonym), sprintf('"%s" in %s (%s) is too short to be a useful prefix', $synonym, $key, $locale));
                }
            }
        }
    }

    private function resolve(array $catalogue, string $key): ?string
    {
        $value = $catalogue;
        foreach (explode('.', $key) as $segment) {
            $value = \is_array($value) && \array_key_exists($segment, $value) ? $value[$segment] : null;
        }

        return \is_string($value) ? $value : null;
    }

    public function testEveryGroupHeadingIsTranslatedInAllFiveLocales(): void
    {
        $keys = array_column($this->declaredGroups(), 'label_key');
        self::assertCount(\count(self::EXPECTED_GROUPS), $keys);

        foreach (self::LOCALES as $locale) {
            $catalogue = Yaml::parseFile(__DIR__ . '/../../translations/messages.' . $locale . '.yaml');
            foreach ($keys as $key) {
                $value = $catalogue;
                foreach (explode('.', $key) as $segment) {
                    $value = \is_array($value) && \array_key_exists($segment, $value) ? $value[$segment] : null;
                }
                self::assertIsString($value, sprintf('%s is missing from messages.%s.yaml', $key, $locale));
                self::assertNotSame('', trim($value), sprintf('%s is empty in messages.%s.yaml', $key, $locale));
            }
        }
    }

    /**
     * 🔴 Every badge hit linked to `app_admin_badges` — the admin list — so a
     * member following one met the login wall, while `/badges/{id}` existed the
     * whole time. A public search may not route a member into `/admin`.
     */
    public function testNoPublicGroupLinksIntoTheAdmin(): void
    {
        preg_match_all("/generate\('(app_[a-z_]+)'/", $this->source(), $matches);
        preg_match_all("/'route' => '(app_[a-z_]+)'/", $this->source(), $destinations);
        $routes = array_merge($matches[1], $destinations[1]);
        self::assertNotEmpty($destinations[1], 'destinations should declare their routes');

        foreach ($routes as $route) {
            if ($route === 'app_admin_user_detail') {
                continue; // the users group is ROLE_ADMIN-only by construction
            }
            self::assertStringStartsNotWith('app_admin_', $route, sprintf('%s is an admin route offered to the public', $route));
        }
    }
}
