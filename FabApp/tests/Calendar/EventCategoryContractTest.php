<?php

declare(strict_types=1);

namespace App\Tests\Calendar;

use PHPUnit\Framework\TestCase;

/**
 * A category is a LABEL. `Event::$formation` is a LINK. (S146f / S146d.)
 *
 * 🔴 **The failure this exists to prevent.** Categories are the operator's own
 * words, renamable on a whim. The moment any code reads a particular category to
 * decide behaviour — `slug === 'formation'`, `label === 'Atelier'` — renaming a row
 * in the admin silently changes what the product does, and the label has become an
 * enum nobody declared. When behaviour must depend on what an event IS, it gets a
 * column: which is exactly why `Event::$formation` exists beside the category and
 * was not folded into it.
 *
 * ⚠️ Source-text assertions: they see the shape, not the pixels. The route sweep
 * and looking at the page are what see the rest.
 */
final class EventCategoryContractTest extends TestCase
{
    private const ROOT = __DIR__ . '/../..';

    public function testTheEventCarriesTwoDifferentColumns(): void
    {
        $entity = file_get_contents(self::ROOT . '/src/Entity/Event.php');

        self::assertStringContainsString("#[ORM\\JoinColumn(name: 'categoryId', nullable: true, onDelete: 'SET NULL')]", $entity);
        self::assertStringContainsString("#[ORM\\JoinColumn(name: 'formationId', nullable: true, onDelete: 'SET NULL')]", $entity);

        // ⚠️ Both nullable: an event with neither is an event, not an error, and an
        // install that has made no categories still works.
        self::assertStringContainsString('private ?EventCategory $category = null;', $entity);
        self::assertStringContainsString('private ?Formation $formation = null;', $entity);
    }

    /**
     * 🔴 **The table is `EVENEMENT`.** `EVENT` is a reserved word in MySQL, so the
     * entity is `App\Entity\Event` over a differently-named table — and a migration
     * that wrote the class name would pass every static check and fail on a live
     * database.
     */
    public function testTheMigrationNamesTheRealTable(): void
    {
        $migration = file_get_contents(self::ROOT . '/migrations/Version20260820100000.php');

        self::assertStringContainsString("addColumnIfMissing('EVENEMENT', 'categoryId'", $migration);
        self::assertStringContainsString("addColumnIfMissing('EVENEMENT', 'formationId'", $migration);
        self::assertStringNotContainsString('ALTER TABLE EVENT ', $migration);
    }

    /**
     * 🔴 Nothing may branch on WHICH category. Any literal category slug or label
     * compared in PHP or Twig is this bug.
     */
    public function testNothingBranchesOnAParticularCategory(): void
    {
        $offenders = [];

        foreach ($this->sourceFiles() as $file) {
            $source = file_get_contents($file);
            if ($source === false) {
                continue;
            }

            // `getCategory()`/`.category` compared against a string literal.
            if (preg_match('/(getCategory\(\)[^;\n]{0,40}|\.category[^;\n]{0,40})(===|==|!=|!==)\s*[\'"]/', $source) === 1) {
                $offenders[] = str_replace(self::ROOT . '/', '', $file);
            }
        }

        self::assertSame(
            [],
            $offenders,
            'A category is a label an operator renames; branching on one turns it into an undeclared enum.',
        );
    }

    /**
     * ⚠️ Archived categories leave the PICKER and the FILTER, never the display.
     * An event already carrying one keeps showing it — otherwise an event's kind
     * vanishes from the page the day somebody tidies the list.
     */
    public function testArchivingHidesTheCategoryFromPickersOnly(): void
    {
        $repository = file_get_contents(self::ROOT . '/src/Repository/EventCategoryRepository.php');

        self::assertStringContainsString('public function findSelectable(): array', $repository);
        self::assertStringContainsString("->andWhere('c.archivedAt IS NULL')", $repository);

        // `findAllOrdered()` is what the admin list reads and must NOT filter.
        $allOrdered = substr($repository, (int) strpos($repository, 'public function findAllOrdered'));
        $allOrdered = substr($allOrdered, 0, (int) strpos($allOrdered, 'public function findOneBySlug'));
        self::assertStringNotContainsString('archivedAt IS NULL', $allOrdered);
    }

    /**
     * ⚠️ **The slug does not follow the label.** It is in every shared filter link,
     * so fixing a typo on Tuesday must not 404 a link sent on Monday.
     */
    public function testRenamingDoesNotMoveTheSlug(): void
    {
        $controller = file_get_contents(self::ROOT . '/src/Controller/AdminController.php');
        $handler = substr($controller, (int) strpos($controller, 'private function handleEventCategoryAction'));
        $handler = substr($handler, 0, (int) strpos($handler, 'private function moveEventCategory'));

        self::assertStringContainsString("case 'rename':", $handler);
        self::assertStringNotContainsString('setSlug(', $handler, 'A rename must not recompute the slug.');
    }

    /**
     * 🔴 The training page shows REAL sessions now — the block S134c2 had to delete
     * because FabOS was inventing them.
     */
    public function testTheTrainingPageShowsRealSessions(): void
    {
        self::assertStringContainsString(
            'public function findUpcomingSessionsFor(Formation $formation',
            file_get_contents(self::ROOT . '/src/Repository/EventRepository.php'),
        );

        $page = file_get_contents(self::ROOT . '/templates/site/formation-detail.html.twig');
        self::assertStringContainsString('upcomingSessions|default([]) is not empty', $page);
        // ⚠️ And it says plainly that turning up is not certification.
        self::assertStringContainsString("'fdet.sessions_not_certifying'|trans", $page);
    }

    /** @return list<string> */
    private function sourceFiles(): array
    {
        $files = [];
        foreach (['src', 'templates', 'assets'] as $dir) {
            $iterator = new \RecursiveIteratorIterator(new \RecursiveDirectoryIterator(self::ROOT . '/' . $dir));
            foreach ($iterator as $file) {
                if ($file->isFile() && in_array($file->getExtension(), ['php', 'twig', 'js'], true)) {
                    $files[] = $file->getPathname();
                }
            }
        }

        return $files;
    }
}
