<?php

declare(strict_types=1);

namespace App\Command;

use App\Entity\Event;
use App\Repository\EventCategoryRepository;
use App\Repository\EventRepository;
use App\Repository\FormationRepository;
use App\Repository\VenueRepository;
use App\Security\ConsoleRenderAuthenticator;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\RequestStack;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\Security\Csrf\CsrfTokenManagerInterface;

/** THROWAWAY — S146d write paths, in a transaction that is ROLLED BACK. */
#[AsCommand(name: 'app:probe:s146d')]
final class S146dProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly EventRepository $events,
        private readonly EventCategoryRepository $categories,
        private readonly FormationRepository $formations,
        private readonly VenueRepository $venues,
        private readonly HttpKernelInterface $kernel,
        private readonly RequestStack $stack,
        private readonly CsrfTokenManagerInterface $csrf,
        private readonly ConsoleRenderAuthenticator $auth,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $connection = $this->em->getConnection();
        @$connection->setNestTransactionsWithSavepoints(true);

        $before = (int) $connection->fetchOne('SELECT COUNT(*) FROM EVENEMENT');
        $io->text(sprintf('before: %d events', $before));

        $this->auth->renderAs(null);
        $connection->beginTransaction();
        $pass = 0;
        $fail = 0;
        $check = function (string $what, bool $ok) use ($io, &$pass, &$fail): void {
            $ok ? $pass++ : $fail++;
            $io->text(($ok ? '  OK   ' : '  FAIL ') . $what);
        };

        try {
            $formation = $this->formations->findOneBy(['titre' => 'Formation découpe laser CO2']);
            $category = $this->categories->findOneBySlug('seance-de-formation');
            $venue = $this->venues->findOneBy(['slug' => 'default']);

            $flashes = $this->post('/admin/events/new', [
                'event_admin' => [
                    'titre' => 'PROBE Cours hebdomadaire',
                    'dateDebut' => '2026-09-07T18:00',
                    'dateFin' => '2026-09-07T20:00',
                    'venue' => (string) $venue->getId(),
                    'lieu' => 'FabLab - D173',
                    'locationMode' => Event::LOCATION_ONSITE,
                    'address' => '',
                    'description' => 'Sonde S146d',
                    'capacite' => '10',
                    'guestsAllowed' => '1',
                    'category' => (string) $category->getId(),
                    'formation' => (string) $formation->getId(),
                    'repeatEvery' => 'week',
                    'repeatCount' => '4',
                ],
            ], 'event_admin');
            $io->text('  flash: ' . implode(' | ', $flashes));

            $this->em->clear();
            $made = $this->events->findBy(['titre' => 'PROBE Cours hebdomadaire'], ['dateDebut' => 'ASC']);
            $check(sprintf('4 events created from one form (%d)', count($made)), count($made) === 4);

            if (count($made) === 4) {
                $dates = array_map(static fn (Event $e): string => $e->getDateDebut()->format('Y-m-d H:i'), $made);
                $check('weekly, at the hour typed: ' . implode(', ', $dates), $dates === [
                    '2026-09-07 18:00', '2026-09-14 18:00', '2026-09-21 18:00', '2026-09-28 18:00',
                ]);
                $ends = array_map(static fn (Event $e): string => $e->getDateFin()->format('H:i'), $made);
                $check('every session still ends at 20:00', $ends === ['20:00', '20:00', '20:00', '20:00']);
                $check('all four carry the category', count(array_filter($made, static fn (Event $e): bool => $e->getCategory()?->getSlug() === 'seance-de-formation')) === 4);
                $check('all four are sessions of the training', count(array_filter($made, static fn (Event $e): bool => $e->getFormation()?->getId() === $formation->getId())) === 4);
                $check('all four carry the venue', count(array_filter($made, static fn (Event $e): bool => $e->getVenue()?->getId() === $venue->getId())) === 4);

                // 🔴 Independent rows: cancelling one must not touch the others.
                $made[1]->callOff('Formateur absent');
                $this->em->flush();
                $this->em->clear();
                $again = $this->events->findBy(['titre' => 'PROBE Cours hebdomadaire'], ['dateDebut' => 'ASC']);
                $check('cancelling session 2 leaves the other three alone',
                    $again[1]->isCancelled() && !$again[0]->isCancelled() && !$again[2]->isCancelled() && !$again[3]->isCancelled());

                $sessions = $this->events->findUpcomingSessionsFor($this->formations->find($formation->getId()), 10);
                $check(sprintf('the training lists them, cancelled one included (%d)', count($sessions)), count($sessions) >= 4);
            }

            // ⚠️ "Once only creates exactly one" is NOT exercised over HTTP here.
            // Symfony caches a form's choice list per type, so a second request in the
            // same process reuses entities that this probe's `em->clear()` detached —
            // an artefact of running several requests in one PHP process, which a real
            // request never does. `EventSeriesTest::testNothingIsGeneratedWithoutARepeat`
            // covers the rule itself, with no database at all.
        } catch (\Throwable $e) {
            $io->error($e->getMessage() . ' @ ' . $e->getFile() . ':' . $e->getLine());
            $fail++;
        } finally {
            while ($connection->isTransactionActive()) {
                $connection->rollBack();
            }
        }

        $this->em->clear();
        $after = (int) $connection->fetchOne('SELECT COUNT(*) FROM EVENEMENT');
        $io->text(sprintf('after rollback: %d events', $after));
        $clean = $after === $before;
        $io->text($clean ? '  OK   nothing survived the rollback' : '  FAIL rows survived — INVESTIGATE');
        $io->success(sprintf('%d passed, %d failed, rollback %s', $pass, $fail, $clean ? 'clean' : 'DIRTY'));

        return $fail === 0 && $clean ? Command::SUCCESS : Command::FAILURE;
    }

    /**
     * A real browser round trip: GET the form, take its CSRF token AND the cookie
     * the stateless double-submit scheme sets on that response, then POST both with
     * a matching Origin. Anything less is refused, exactly as a forged post would be.
     *
     * @return list<string>
     */
    private function post(string $path, array $fields, string $tokenId): array
    {
        $get = Request::create($path);
        $get->setSession(new Session(new MockArraySessionStorage()));
        $page = $this->kernel->handle($get, HttpKernelInterface::MAIN_REQUEST, false);

        $html = (string) $page->getContent();
        preg_match('/name="' . preg_quote($tokenId, '/') . '\[_token\]"[^>]*value="([^"]+)"/', $html, $m);
        $token = $m[1] ?? '';

        $cookies = [];
        foreach ($page->headers->getCookies() as $cookie) {
            $cookies[$cookie->getName()] = $cookie->getValue();
        }

        $data = $fields;
        $data[$tokenId]['_token'] = $token;

        $request = Request::create($path, 'POST', $data, $cookies, [], ['HTTP_ORIGIN' => 'http://localhost']);
        $request->setSession(new Session(new MockArraySessionStorage()));
        $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, false);

        $out = [];
        foreach ($request->getSession()->getFlashBag()->all() as $type => $messages) {
            foreach ($messages as $message) {
                $out[] = $type . ': ' . $message;
            }
        }

        return $out;
    }
}
