<?php

namespace App\Tests\Nav;

use PHPUnit\Framework\TestCase;
use Symfony\Component\Yaml\Yaml;

/**
 * Every literal key a template asks for exists in all five catalogues.
 *
 * ⚠️ **A missing key is not an error anywhere.** Twig renders the key itself, so
 * the screen reads `admin_emails.col_status` where it should read "Statut" —
 * valid HTML, HTTP 200, lint green, tests green. It is visible only to someone
 * who opens that page in that language, which is why three of them had been
 * live for months when S141f swept the rendered pages for dotted identifiers:
 * `admin_emails.col_status` in the mail log's header and `login.email` on the
 * **public** password-reset form, plus `admin_user_detail.col_status` and its
 * siblings.
 *
 * 🔴 And one of them was made during S141 itself. A regex meant to retire two
 * keys inside `rfid_logs` was written unanchored — `^ {4}col_status: .*$` — and
 * removed every four-space-indented `col_status` in the file, in all five
 * languages: **eleven keys across eight admin lists and three member pages**,
 * silently, between two green test runs. This test is what makes that class of
 * edit safe to make again.
 *
 * ⚠️ **Only literal keys.** `('rfid_logs.status_' ~ log.status)|trans` cannot be
 * checked from source without executing the template, and pretending otherwise
 * is how a sweep deletes thirteen live keys because they are "unreferenced" —
 * see the note in S141d. Keys built by concatenation are the caller's
 * responsibility, and `_rfid_result` is the pattern for handling them: a
 * declared map with a humanising fallback, never `|trans` on a computed key.
 */
final class TranslationKeyTest extends TestCase
{
    private const LOCALES = ['fr', 'en', 'de', 'es', 'it'];
    private const TEMPLATES = __DIR__ . '/../../templates';

    /**
     * A dotted lowercase identifier in single quotes, immediately piped to
     * `trans`. Anything else — a concatenation, a variable, a `default()` — does
     * not match, and is meant not to.
     */
    private const KEY = "/'([a-z][a-z0-9_]*(?:\.[a-z0-9_]+)+)'\s*\|\s*trans/";

    /** @return list<string> */
    private function referencedKeys(): array
    {
        $keys = [];
        $files = new \RecursiveIteratorIterator(new \RecursiveDirectoryIterator(self::TEMPLATES));
        foreach ($files as $file) {
            if (!$file->isFile() || !str_ends_with($file->getFilename(), '.twig')) {
                continue;
            }
            $source = (string) file_get_contents($file->getPathname());
            // Comments quote keys constantly — including keys that were just
            // deleted on purpose — so they are stripped before matching.
            $source = (string) preg_replace('/\{#.*?#\}/s', '', $source);
            preg_match_all(self::KEY, $source, $matches);
            foreach ($matches[1] as $key) {
                $keys[$key] = true;
            }
        }

        self::assertGreaterThan(500, \count($keys), 'the templates should reference hundreds of keys');

        return array_keys($keys);
    }

    /** @return array<string, mixed> */
    private function catalogue(string $locale): array
    {
        return Yaml::parseFile(self::TEMPLATES . '/../translations/messages.' . $locale . '.yaml');
    }

    /** @param array<string, mixed> $catalogue */
    private function has(array $catalogue, string $key): bool
    {
        $value = $catalogue;
        foreach (explode('.', $key) as $segment) {
            if (!\is_array($value) || !\array_key_exists($segment, $value)) {
                return false;
            }
            $value = $value[$segment];
        }

        return \is_string($value) && trim($value) !== '';
    }

    public function testEveryLiteralKeyExistsInAllFiveCatalogues(): void
    {
        $keys = $this->referencedKeys();

        foreach (self::LOCALES as $locale) {
            $catalogue = $this->catalogue($locale);
            $missing = [];
            foreach ($keys as $key) {
                if (!$this->has($catalogue, $key)) {
                    $missing[] = $key;
                }
            }

            self::assertSame(
                [],
                $missing,
                sprintf('messages.%s.yaml is missing %d key(s) a template asks for', $locale, \count($missing)),
            );
        }
    }
}
