<?php

namespace App\Tests\Nav;

use PHPUnit\Framework\TestCase;

/**
 * The admin list shell has one format, and one place each page's heading comes
 * from.
 *
 * ⚠️ **Why a source-text test.** Rendering `_admin_list` needs a request, a
 * firewall, a `NavBuilder` and a database, and none of that would catch what
 * actually goes wrong here: a page quietly passing an argument the shell used to
 * honour. Three of the four guarantees below are about arguments that no longer
 * exist — a template that keeps passing one is not an error, it is a silent
 * no-op, which is the failure mode a test has to speak up about.
 *
 * The fourth is the one that already bit: `{% embed %}` without `only` merges
 * the parent context, so while the override was called `title`, any controller
 * that happened to pass a variable of that name became the page heading.
 * `/admin/machines/categories` does, and it printed the raw key
 * `machine_taxonomy.categories_title` in the band the first time S141 was tried.
 */
final class AdminListShellTest extends TestCase
{
    private const SHELL = __DIR__ . '/../../templates/site/_admin_list.html.twig';
    private const TEMPLATES = __DIR__ . '/../../templates/site';

    /**
     * The three screens the admin navigation cannot name, and which are
     * therefore allowed to pass their own heading. Adding a fourth is a
     * decision, not a detail: the whole point of S141 is that a list is titled
     * by the menu entry that leads to it.
     */
    private const MAY_OVERRIDE_TITLE = [
        'admin-event-registrations.html.twig',
        'admin-usage-package-form.html.twig',
        'staff-access-passes.html.twig',
    ];

    /** @return array<string, string> basename => source */
    private function callers(): array
    {
        $callers = [];
        foreach (glob(self::TEMPLATES . '/*.html.twig') ?: [] as $path) {
            $source = (string) file_get_contents($path);
            if (str_contains($source, "embed 'site/_admin_list.html.twig'")) {
                $callers[basename($path)] = $source;
            }
        }

        self::assertGreaterThan(25, \count($callers), 'the admin list shell should still have its callers');

        return $callers;
    }

    /**
     * `hero: 'compact'` and `hero: 'merged'` selected between three page shapes
     * while the merged card was on trial. The operator kept the merged one on
     * 2026-08-16, so the parameter is gone and the shell has no branch left to
     * take. A template still passing it would render correctly and say something
     * false about the codebase.
     */
    public function testNoPageStillSelectsAListFormat(): void
    {
        foreach ($this->callers() as $name => $source) {
            self::assertDoesNotMatchRegularExpression(
                '/^\s*hero:\s/m',
                $source,
                sprintf('%s still passes `hero:`; there is only one list format', $name),
            );
        }

        self::assertStringNotContainsString(
            "hero|default",
            (string) file_get_contents(self::SHELL),
            'the shell should no longer read a `hero` parameter',
        );
    }

    /**
     * The heading is the name of the menu entry. A page that passes its own is
     * maintaining a second copy of a word already written in the navigation
     * strip above it — in five languages, and free to drift from it.
     */
    public function testOnlyTheNamedScreensPassTheirOwnHeading(): void
    {
        foreach ($this->callers() as $name => $source) {
            $overrides = preg_match('/^\s*page_title:\s/m', $source) === 1;

            if (\in_array($name, self::MAY_OVERRIDE_TITLE, true)) {
                self::assertTrue($overrides, sprintf('%s is listed as a screen the menu cannot name, but passes no `page_title`', $name));

                continue;
            }

            self::assertFalse($overrides, sprintf('%s passes `page_title`; its heading should come from `NavBuilder`', $name));
        }
    }

    /**
     * ⚠️ Both of these were arguments of the shell and are not any more. `title`
     * is the context-leak described in the class docblock; `description` was the
     * line under the heading, which the count replaced.
     */
    public function testNoPagePassesTheRetiredTitleOrDescription(): void
    {
        foreach ($this->callers() as $name => $source) {
            // Only at the embed's own indentation: `title:` and `description:`
            // are also legitimate arguments of `_cell_title` and of the usage
            // summary partial, several levels deeper inside the same files.
            self::assertDoesNotMatchRegularExpression(
                '/^ {4}(title|description):\s/m',
                $source,
                sprintf('%s passes a retired `title`/`description` to the list shell', $name),
            );
        }
    }

    /**
     * The guard for the context leak itself: whatever the override is called, it
     * must not be a name a controller is likely to be using for something else.
     */
    public function testTheShellReadsPageTitleAndNotTheBareContextVariable(): void
    {
        $shell = (string) file_get_contents(self::SHELL);

        self::assertStringContainsString("page_title|default('')", $shell);
        self::assertDoesNotMatchRegularExpression(
            '/\{%\s*set\s+_title\s*=\s*title\|default/',
            $shell,
            'the shell must not fall back to a bare `title` from the parent context',
        );
    }
}
