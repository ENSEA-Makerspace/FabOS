<?php

declare(strict_types=1);

namespace App\Command;

use App\Entity\Event;
use App\Entity\EventCategory;
use App\Repository\EventCategoryRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\VenueRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * THROWAWAY — example data for S146f, at the operator's request (2026-08-20).
 * Delete this file after running it; it is not part of the product.
 *
 * ⚠️ Idempotent: a category whose slug already exists is left alone, and an event
 * that already carries a category is not overwritten. Re-running changes nothing.
 * ⚠️ `--dry-run` prints the plan and writes nothing.
 */
#[AsCommand(name: 'app:seed:event-categories')]
final class SeedEventCategoriesCommand extends Command
{
    /** label => [slug, icon] */
    private const CATEGORIES = [
        'Atelier' => 'atelier',
        'Séance de formation' => 'seance-de-formation',
        'Portes ouvertes' => 'portes-ouvertes',
        'Rencontre' => 'rencontre',
    ];

    /** Existing events keep their titles; only the category is added. */
    private const ASSIGN = [
        'Grand opening du FabLab' => 'portes-ouvertes',
        'Anniversaire Cédric' => 'rencontre',
        'Test' => 'atelier',
    ];

    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly EventCategoryRepository $categories,
        private readonly EventRepository $events,
        private readonly FormationRepository $formations,
        private readonly VenueRepository $venues,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('dry-run', null, InputOption::VALUE_NONE, 'Print the plan and write nothing.');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $dry = (bool) $input->getOption('dry-run');

        // 1. The categories.
        $bySlug = [];
        foreach (self::CATEGORIES as $label => $slug) {
            $existing = $this->categories->findOneBySlug($slug);
            if ($existing !== null) {
                $io->text(sprintf('  = category "%s" already exists', $existing->getLabel()));
                $bySlug[$slug] = $existing;
                continue;
            }

            $category = (new EventCategory())->setLabel($label);
            $category->setPosition(count($bySlug));
            $io->text(sprintf('  + category "%s" (%s)', $label, $category->getSlug()));
            if (!$dry) {
                $this->em->persist($category);
            }
            $bySlug[$slug] = $category;
        }

        if (!$dry) {
            $this->em->flush();
        }

        // 2. Existing events keep everything they have; they only gain a category.
        foreach (self::ASSIGN as $title => $slug) {
            $event = $this->events->findOneBy(['titre' => $title]);
            if ($event === null) {
                $io->text(sprintf('  ? event "%s" not found — skipped', $title));
                continue;
            }
            if ($event->getCategory() !== null) {
                $io->text(sprintf('  = event "%s" already categorised (%s)', $title, $event->getCategory()->getLabel()));
                continue;
            }
            $io->text(sprintf('  ~ event "%s" → %s', $title, $bySlug[$slug]->getLabel()));
            if (!$dry) {
                $event->setCategory($bySlug[$slug]);
            }
        }

        // 3. Three UPCOMING events, because /events defaults to upcoming and all
        //    three existing rows are in the past — without these the categories are
        //    invisible on the page they were built for.
        $fablab = $this->venues->findOneBy(['slug' => 'default']);
        $printer = $this->formations->findOneBy(['titre' => 'Formation imprimante 3D']);

        $planned = [
            ['Atelier découverte impression 3D', 'atelier', '+7 days 14:00', '+7 days 17:00', 'FabLab - D173', 12, null],
            ['Séance — Formation imprimante 3D', 'seance-de-formation', '+14 days 09:00', '+14 days 12:00', 'FabLab - D173', 8, $printer],
            ['Portes ouvertes de rentrée', 'portes-ouvertes', '+23 days 10:00', '+23 days 18:00', 'FabLab - D173', null, null],
        ];

        foreach ($planned as [$title, $slug, $from, $to, $where, $seats, $formation]) {
            if ($this->events->findOneBy(['titre' => $title]) !== null) {
                $io->text(sprintf('  = event "%s" already exists', $title));
                continue;
            }

            $io->text(sprintf(
                '  + event "%s" (%s%s)',
                $title,
                $bySlug[$slug]->getLabel(),
                $formation !== null ? ', session of ' . $formation->getTitre() : '',
            ));

            if ($dry) {
                continue;
            }

            $event = (new Event())
                ->setTitre($title)
                ->setDateDebut(new \DateTimeImmutable($from))
                ->setDateFin(new \DateTimeImmutable($to))
                ->setLieu($where)
                ->setVenue($fablab)
                ->setCapacite($seats)
                ->setGuestsAllowed(true)
                ->setLocationMode(Event::LOCATION_ONSITE)
                ->setCategory($bySlug[$slug])
                ->setFormation($formation);

            $this->em->persist($event);
        }

        if (!$dry) {
            $this->em->flush();
        }

        $io->success($dry ? 'Dry run — nothing written.' : 'Example data in place.');

        return Command::SUCCESS;
    }
}
