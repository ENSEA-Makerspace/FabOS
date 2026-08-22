<?php

namespace App\Command;

use App\Security\ConsoleRenderAuthenticator;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;

/**
 * S147, point 8 — does a refused field make you type the rest again?
 *
 * The rule the operator set for Phase J is testable and this is the test:
 * **a form that rejects one value must give the others back.** The static pass
 * found 15 hand-rolled POST handlers that `addFlash('error')` then
 * `redirectToRoute()`, which cannot give anything back — a redirect re-renders
 * from the database, so everything typed in that request is gone. Reading the
 * code says that; only a real POST proves it.
 *
 * ⚠️ Two things this has to get right, both learned the hard way in this base:
 *   - the CSRF token here is a **session** token (`csrf_token('x')` in Twig), so
 *     the GET that mints it and the POST that spends it must share the SAME
 *     `Session` instance — a fresh one per request just fails the check, and a
 *     failed check looks exactly like the bug being measured;
 *   - everything runs inside a transaction that is rolled back, so a probe that
 *     accidentally hits a *valid* path writes nothing to the live database.
 *
 *   php bin/console app:s147:form-probe
 */
#[AsCommand(name: 'app:s147:form-probe', description: 'S147: prove whether a refused form keeps what was typed.')]
final class S147FormProbeCommand extends Command
{
    public function __construct(
        private readonly ConsoleRenderAuthenticator $authenticator,
        private readonly HttpKernelInterface $kernel,
        private readonly EntityManagerInterface $em,
        private readonly \App\UsageRights\UsageRightsService $rights,
        private readonly \App\UsageRights\UsageGrantRepository $grants,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $as = $this->authenticator->renderAs(null);
        $io->text(sprintf('signed in as %s', $as));

        $this->em->getConnection()->beginTransaction();

        try {
            $this->probeProfile($io);
            $this->probeMachineForm($io);
            $this->probeGrantWindows($io);
        } finally {
            $this->em->getConnection()->rollBack();
            $io->text('transaction rolled back — nothing written');
        }

        return Command::SUCCESS;
    }

    /**
     * Hand-rolled handler: /profil, the "public profile" form.
     *
     * The slug is validated first and the bio is read four lines later, so a bad
     * slug returns before the bio is ever looked at. The bio is the field a
     * member actually writes prose into, which is what makes this the expensive
     * one to lose.
     */
    private function probeProfile(SymfonyStyle $io): void
    {
        $io->section('/profil — public profile form (hand-rolled)');

        $session = new Session(new MockArraySessionStorage());

        $get = Request::create('/profil');
        $get->setSession($session);
        $html = (string) $this->kernel->handle($get, HttpKernelInterface::MAIN_REQUEST, false)->getContent();

        $token = $this->tokenNear($html, 'public_profile');
        if ($token === null) {
            $io->warning('no public_profile token found in the page — probe inconclusive');

            return;
        }

        $typed = 'BIO TAPEE PAR LE MEMBRE S147 ' . str_repeat('texte ', 12);

        $post = Request::create('/profil', 'POST', [
            '_profile_form' => 'public_profile',
            '_token' => $token,
            'publicProfileEnabled' => '1',
            'publicSlug' => '!!!',          // becomes '' after the slug filter → refused
            'publicBio' => $typed,
            'publicFields' => ['email', 'badges'],
        ]);
        $post->setSession($session);

        $response = $this->kernel->handle($post, HttpKernelInterface::MAIN_REQUEST, false);
        $status = $response->getStatusCode();
        $body = (string) $response->getContent();

        $io->definitionList(
            ['status' => $status],
            ['redirect' => $response->headers->get('location') ?? '—'],
            ['typed bio echoed back in the response' => str_contains($body, 'BIO TAPEE') ? 'YES' : 'NO'],
        );

        // Follow the redirect the way a browser would, and look for the value there.
        if ($status >= 300 && $status < 400) {
            $follow = Request::create('/profil');
            $follow->setSession($session);
            $after = (string) $this->kernel->handle($follow, HttpKernelInterface::MAIN_REQUEST, false)->getContent();
            $io->definitionList(
                ['bio present after following the redirect' => str_contains($after, 'BIO TAPEE') ? 'YES' : 'NO'],
                ['flash shown' => $this->flashText($after)],
            );
        }
    }

    /**
     * Control case: a Symfony Form. Same shape of mistake, different machinery —
     * the form object holds the submitted data and the template re-renders it,
     * so the other fields come back on their own.
     */
    private function probeMachineForm(SymfonyStyle $io): void
    {
        $io->section('/admin/machines/new — Symfony Form (control)');

        $session = new Session(new MockArraySessionStorage());

        $get = Request::create('/admin/machines/new');
        $get->setSession($session);
        $html = (string) $this->kernel->handle($get, HttpKernelInterface::MAIN_REQUEST, false)->getContent();

        if (!preg_match('/name="([a-z_]+)\\[_token\\]"/i', $html, $m)) {
            $io->warning('no form token field found — probe inconclusive');

            return;
        }
        $formName = $m[1];

        // ⚠️ This form uses the STATELESS double-submit token, not a session one:
        // the HTML ships the placeholder `csrf-token` and a Stimulus controller
        // swaps in the value of the cookie the response just set. Nothing swaps it
        // here, so the probe has to do what the browser does — read the cookie off
        // the GET response and send it back as the field value.
        $getResponse = $this->kernel->handle(Request::create('/admin/machines/new'), HttpKernelInterface::MAIN_REQUEST, false);
        $token = null;
        foreach ($getResponse->headers->getCookies() as $cookie) {
            if (str_contains($cookie->getName(), 'csrf') || $cookie->getName() === $formName) {
                $token = $cookie->getValue();
            }
        }
        if ($token === null && preg_match('/name="' . preg_quote($formName, '/') . '\\[_token\\]"[^>]*value="([^"]+)"/i', $html, $t)) {
            $token = $t[1];
        }
        if ($token === null || $token === 'csrf-token') {
            $io->warning('stateless CSRF cookie not reproducible from the console — control NOT run');

            return;
        }

        $typed = 'DESCRIPTION TAPEE S147';

        $post = Request::create('/admin/machines/new', 'POST', [
            $formName => [
                '_token' => $token,
                'nom' => '',                 // required → the form refuses
                'description' => $typed,
            ],
        ], [], [], ['HTTP_ORIGIN' => 'https://fabos.dstei.fr']);
        $post->setSession($session);

        $response = $this->kernel->handle($post, HttpKernelInterface::MAIN_REQUEST, false);
        $body = (string) $response->getContent();

        $io->definitionList(
            ['status' => $response->getStatusCode()],
            ['redirect' => $response->headers->get('location') ?? '—'],
            ['typed description echoed back' => str_contains($body, 'DESCRIPTION TAPEE') ? 'YES' : 'NO'],
        );
    }

    /**
     * S147, J-20 — does a package window actually reach the calendar?
     *
     * The read path is: a window row → `UsageGrantRepository::windowsFor()` →
     * `UsageRightsService::bookingWindowsFor()` → the calendar payload → the grid.
     * Nothing in the live database exercises it (0 windows exist), so the probe
     * writes one inside the transaction this command rolls back, asks the service,
     * and leaves no trace.
     */
    private function probeGrantWindows(SymfonyStyle $io): void
    {
        $io->section('J-20 — a Thursday-afternoon window, read back through the service');

        $db = $this->em->getConnection();

        // ⚠️ Roles are not a column — `getRoles()` walks a join table — so the
        // non-admin has to be found in PHP, not in DQL.
        // ⚠️ **Not just any non-admin: one who actually HOLDS a machines grant.**
        // The first version took the first non-admin it found, got an empty window
        // list, and would have read as "the feature does not work". It read as
        // nothing at all: that account holds no package, so there was no grant to
        // put a window on. Pick someone the model has something to say about.
        $member = null;
        foreach ($this->em->getRepository(\App\Entity\Utilisateur::class)->findBy([], ['id' => 'ASC'], 200) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                continue;
            }
            if ($this->grants->paths($candidate, 'machines', \App\UsageRights\UsageGrantAction::Use) !== []) {
                $member = $candidate;
                break;
            }
        }
        if ($member === null) {
            $io->warning('no non-admin account holds a machines grant');
            // 🔴 That is not a probe failure, it is a finding: enforcement is ON and
            // the packages reach nobody. Report what a member's verdict actually is,
            // because "may nobody book?" is a much bigger question than J-20.
            $rows = [];
            foreach ($this->em->getRepository(\App\Entity\Utilisateur::class)->findBy([], ['id' => 'ASC'], 200) as $candidate) {
                if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                    continue;
                }
                $v = $this->rights->verdict($candidate, 'machines');
                $rows[($v->allowed ? 'allowed' : 'denied') . ' / ' . ($v->reason ?? '—')] = ($rows[($v->allowed ? 'allowed' : 'denied') . ' / ' . ($v->reason ?? '—')] ?? 0) + 1;
                if (\count($rows) > 6) {
                    break;
                }
            }
            $io->section('what non-admins actually get for `machines`');
            foreach ($rows as $verdict => $count) {
                $io->text(sprintf('  %-40s %d account(s)', $verdict, $count));
            }
            $io->text(sprintf('  enforcement: %s', $this->rights->isEnforced() ? 'ON' : 'off'));

            // The booking gate itself, not just the overview verdict. Read-only:
            // `allowsReservableDuring()` decides, it does not write.
            $slot = new \DateTimeImmutable('2026-08-27 15:00');
            foreach ($this->em->getRepository(\App\Entity\Utilisateur::class)->findBy([], ['id' => 'ASC'], 200) as $candidate) {
                if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                    continue;
                }
                $ok = $this->rights->allowsReservableDuring(
                    $candidate, \App\Reservation\ReservableType::Machine, $slot, $slot->modify('+1 hour'), null, 1, null,
                );
                $io->text(sprintf('  booking gate for a member: %s', $ok ? 'ALLOWED' : 'REFUSED'));
                break;
            }

            return;
        }

        // 🔴 **Every covering grant, not one of them — and the first version of this
        // probe got it wrong in exactly the way an operator will.** Windowing a
        // single grant returned "unrestricted", because grants combine with OR and
        // one unwindowed grant opens the whole week. That is the model working, and
        // it is the thing to say out loud: a window added BESIDE a blank cheque
        // does nothing at all.
        $grantIds = $db->fetchFirstColumn(
            "SELECT g.id FROM USAGE_PACKAGE_GRANT g
             INNER JOIN USAGE_PACKAGE p ON p.id = g.packageId AND p.active = 1
             WHERE g.featureKey = 'machines' AND g.action = 'use'",
        );
        if ($grantIds === []) {
            $io->warning('no active machines/use grant — probe inconclusive');

            return;
        }

        $before = $this->rights->bookingWindowsFor($member, \App\Reservation\ReservableType::Machine, 1);

        foreach ($grantIds as $grantId) {
            // Thursday (ISO 4), 14:00 to 18:00.
            $db->insert('USAGE_GRANT_WINDOW', [
                'grantId' => (int) $grantId, 'dayOfWeek' => 4, 'startMinute' => 840, 'endMinute' => 1080,
                // ⚠️ `createdAt` has no default in the S144b migration, so a hand-written
                // INSERT has to supply it. The editor screen goes through the repository,
                // which does; a probe that skips the repository does not.
                'createdAt' => (new \DateTimeImmutable())->format('Y-m-d H:i:s'),
            ]);
        }

        $after = $this->rights->bookingWindowsFor($member, \App\Reservation\ReservableType::Machine, 1);

        $io->definitionList(
            ['windows before the insert' => \count($before) . ' (0 = unrestricted, which is today)'],
            ['windows after the insert' => \count($after)],
            ['the window that came back' => $after === [] ? '—' : sprintf(
                'day %d, %d→%d minutes',
                $after[0]['dayOfWeek'], $after[0]['startMinute'], $after[0]['endMinute'],
            )],
            ['grants windowed' => \count($grantIds)],
            ['enforcement on' => $this->rights->isEnforced() ? 'yes' : 'NO — that is why it may read 0'],
        );

        // And the coverage rule itself, on the value the calendar will use.
        $thursday = new \DateTimeImmutable('2026-08-27 15:00');   // a Thursday
        $monday = new \DateTimeImmutable('2026-08-24 15:00');
        $windows = array_map(
            static fn (array $w): \App\UsageRights\GrantWindow => new \App\UsageRights\GrantWindow(
                $w['dayOfWeek'], $w['startMinute'], $w['endMinute'],
            ),
            $after,
        );
        $io->definitionList(
            ['Thursday 15:00–16:00 covered' => \App\UsageRights\GrantWindowSet::covers($windows, $thursday, $thursday->modify('+1 hour')) ? 'YES' : 'no'],
            ['Thursday 19:00–20:00 covered' => \App\UsageRights\GrantWindowSet::covers($windows, $thursday->setTime(19, 0), $thursday->setTime(20, 0)) ? 'yes' : 'NO — correct'],
            ['Monday 15:00–16:00 covered' => \App\UsageRights\GrantWindowSet::covers($windows, $monday, $monday->modify('+1 hour')) ? 'yes' : 'NO — correct'],
        );
    }

    /** Pulls the hidden `_token` that sits in the same form as the given marker. */
    private function tokenNear(string $html, string $marker): ?string
    {
        foreach (explode('<form', $html) as $chunk) {
            if (!str_contains($chunk, $marker)) {
                continue;
            }
            if (preg_match('/name="_token"\s+value="([^"]+)"/', $chunk, $m)) {
                return $m[1];
            }
            if (preg_match('/value="([^"]+)"\s+name="_token"/', $chunk, $m)) {
                return $m[1];
            }
        }

        return null;
    }

    private function flashText(string $html): string
    {
        if (preg_match('/class="[^"]*(?:alert|flash)[^"]*"[^>]*>(.{0,120}?)</s', $html, $m)) {
            return trim(strip_tags($m[1])) ?: '—';
        }

        return '—';
    }
}
