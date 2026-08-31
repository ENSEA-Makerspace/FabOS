<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use App\Reservation\LabClock;
use App\Reservation\ReservableResolver;
use App\Reservation\ReservableType;
use App\Reservation\ReservationService;
use App\Schedule\ScheduleResolver;
use App\UsageRights\PackageSpec;
use App\UsageRights\PackageSpecCompiler;
use App\UsageRights\UsageAllowanceRepository;
use App\UsageRights\UsagePackageRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S153 — la SONDE D'ÉCRITURE de la saisie des packages.
 *
 * 🔴 **Pourquoi une sonde et pas une relecture du code.** Le compilateur écrit
 * dans cinq tables et la case « sans limite d'horaires » change une décision
 * d'AUTORISATION. Lire le code prouve que le balisage existe ; seule une écriture
 * suivie d'une relecture prouve que la ligne est là, et seule une réservation
 * refusée puis acceptée prouve que l'exemption agit — c'est la demande explicite
 * de la feuille de route : « réserver un créneau hors horaires avec et sans le
 * package, avant/après ».
 *
 * ⚠️ **Tout se passe dans une transaction qui est ANNULÉE.** La sonde crée un
 * package, l'attribue, réserve — et la base ressort inchangée. Une sonde qui
 * laisse des lignes derrière elle devient un jeu de données que personne
 * n'assume.
 *
 *   php bin/console app:s153:package-probe
 */
#[AsCommand(name: 'app:s153:package-probe', description: 'S153 : prouve que le compilateur écrit, relit, et que « sans limite d’horaires » agit.')]
final class S153PackageProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly PackageSpecCompiler $compiler,
        private readonly UsagePackageRepository $packages,
        private readonly UsageAllowanceRepository $allowances,
        private readonly ReservationService $reservations,
        private readonly ScheduleResolver $schedule,
        private readonly ReservableResolver $reservables,
        private readonly MachineRepository $machines,
        private readonly UtilisateurRepository $users,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = 0;
        $this->em->getConnection()->beginTransaction();

        try {
            $failures += $this->probeCompile($io);
            $failures += $this->probeNormalisation($io);
            $failures += $this->probeHoursExemption($io);
        } finally {
            // ⚠️ `rollBack()` dans un `finally` : une exception au milieu ne doit
            // pas laisser un package de sonde en base.
            $this->em->getConnection()->rollBack();
        }

        if ($failures > 0) {
            $io->error(sprintf('%d assertion(s) en échec.', $failures));

            return Command::FAILURE;
        }

        $io->success('Sonde S153 verte.');

        return Command::SUCCESS;
    }

    /** Une spec restreinte doit produire les grants, les fenêtres et les quotas qu'elle annonce. */
    private function probeCompile(SymfonyStyle $io): int
    {
        $io->section('1. Une spec restreinte s’écrit, et se relit à l’identique');

        $id = $this->packages->save(null, 'Sonde S153 — restreint', '', true, false, []);
        $this->compiler->compile($id, 'Sonde S153 — restreint', '', true, new PackageSpec(
            featuresAll: false,
            features: ['machines'],
            daysAll: false,
            days: [4],
            startTime: '14:00',
            endTime: '18:00',
        ));

        $grants = $this->packages->grantsFor($id);
        $failures = 0;
        $failures += $this->check($io, 'un seul grant écrit', count($grants) === 1, sprintf('%d grant(s)', count($grants)));
        if ($grants !== []) {
            $failures += $this->check($io, 'sur la feature machines', $grants[0]['featureKey'] === 'machines', (string) $grants[0]['featureKey']);
            $failures += $this->check($io, 'sans lieu ni ressource', $grants[0]['venueId'] === null && $grants[0]['reservableId'] === null, 'portée');
            $failures += $this->check($io, 'une fenêtre, le jeudi', count($grants[0]['windows']) === 1 && ($grants[0]['windows'][0]['dayOfWeek'] ?? 0) === 4, 'fenêtres');
        }
        $failures += $this->check($io, 'aucun quota', $this->allowances->forPackage($id) === [], 'allocations');

        // 🔴 L'aller-retour : ce que la saisie rouvre doit être ce qu'elle a écrit.
        // Sans ça, ouvrir puis enregistrer sans rien toucher changerait le package.
        $package = $this->packages->find($id);
        $spec = $package === null ? null : $this->compiler->decompile($id, $package['fullAccess'], $package['features']);
        $failures += $this->check($io, 'le package se relit comme une spec', $spec !== null, 'decompile');
        if ($spec !== null) {
            $failures += $this->check($io, 'relu : machines seulement', !$spec->featuresAll && $spec->features === ['machines'], implode(',', $spec->features));
            $failures += $this->check($io, 'relu : le jeudi 14:00–18:00', !$spec->daysAll && $spec->days === [4] && $spec->startTime === '14:00' && $spec->endTime === '18:00', $spec->startTime . '–' . $spec->endTime);
        }

        return $failures;
    }

    /** « Tout coché » plus la cinquième ligne ≡ `fullAccess`, et rien d'autre en base. */
    private function probeNormalisation(SymfonyStyle $io): int
    {
        $io->section('2. Aucune restriction ≡ fullAccess, sans grants');

        $id = $this->packages->save(null, 'Sonde S153 — tout', '', true, false, []);
        $this->compiler->compile($id, 'Sonde S153 — tout', '', true, new PackageSpec(hoursExempt: true));

        $package = $this->packages->find($id);
        $failures = 0;
        $failures += $this->check($io, 'fullAccess écrit', $package !== null && $package['fullAccess'], 'fullAccess');
        $failures += $this->check($io, 'aucune ligne de feature', $package !== null && $package['features'] === [], 'features');
        $failures += $this->check($io, 'aucun grant', $this->packages->grantsFor($id) === [], 'grants');
        $failures += $this->check($io, 'aucun quota', $this->allowances->forPackage($id) === [], 'allocations');

        // La cinquième ligne sur un package restreint doit être REFUSÉE : elle
        // s'écrit dans un bit, qui ne sait pas dire « tout sauf le jeudi ».
        $refused = false;
        try {
            $this->compiler->compile($id, 'Sonde S153 — tout', '', true, new PackageSpec(
                daysAll: false,
                days: [4],
                hoursExempt: true,
            ));
        } catch (\Throwable) {
            $refused = true;
        }
        $failures += $this->check($io, '« sans limite d’horaires » refusée sur un package restreint', $refused, 'garde serveur');

        return $failures;
    }

    /**
     * La demande explicite de la feuille de route : le même créneau hors horaires,
     * refusé sans le package et accepté avec.
     */
    private function probeHoursExemption(SymfonyStyle $io): int
    {
        $io->section('3. Réserver hors horaires, avant / après');

        // 🔴 **Il faut ISOLER les horaires, et trois refus arrivent AVANT eux.**
        // `validate()` teste dans l'ordre : les droits d'usage, l'accès à la
        // machine (formation pratique, badge), les dates, puis seulement la
        // fermeture. Les deux premières versions de cette sonde ont donc mesuré
        // `USAGE_RIGHTS_DENIED` puis `PRACTICAL_TRAINING_REQUIRED` en croyant
        // mesurer la fermeture — et l'une des deux était même VERTE.
        // La sonde cherche donc un couple (membre, machine) qui atteint vraiment
        // le contrôle d'horaires : le témoin négatif doit dire `FABLAB_CLOSED`,
        // sinon il ne prouve rien.
        $members = [];
        foreach ($this->users->findBy([], ['id' => 'ASC']) as $candidate) {
            // ⚠️ Un ADMIN ne prouve rien : `verdict()` court-circuite pour lui.
            if ($candidate instanceof Utilisateur && !in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $members[] = $candidate;
            }
        }
        $machines = array_values(array_filter(
            $this->machines->findBy([], ['nom' => 'ASC']),
            static fn (object $machine): bool => !$machine->isArchived(),
        ));
        if ($members === [] || $machines === []) {
            $io->warning('Pas de machine ou pas de membre non-admin : cette partie de la sonde ne peut pas se poser.');

            return 0;
        }

        foreach ($members as $member) {
            // Le témoin négatif : un package qui autorise TOUT sauf l'exemption
            // d'horaires. La seule chose qui changera entre les deux mesures est
            // la case elle-même.
            $id = $this->packages->save(null, 'Sonde S153 — staff', '', true, false, []);
            $this->compiler->compile($id, 'Sonde S153 — staff', '', true, new PackageSpec(hoursExempt: false));
            $this->packages->assign($id, $member, null, null, null);

            foreach ($machines as $machine) {
                $venueId = $this->reservables->venueIdFor(ReservableType::Machine, (int) $machine->getId());
                $slot = $this->closedSlot($venueId);
                if ($slot === null) {
                    continue;
                }
                [$start, $end] = $slot;
                $before = $this->reservations->book(ReservableType::Machine, (int) $machine->getId(), $member, $start, $end);
                if ($before->code !== 'FABLAB_CLOSED') {
                    continue;
                }

                $io->text(sprintf(
                    '%s sur « %s », %s → %s',
                    $member->getDisplayName(),
                    $machine->getNom(),
                    $start->format('Y-m-d H:i'),
                    $end->format('H:i'),
                ));
                $failures = $this->check($io, 'sans la case : refusé FABLAB_CLOSED', true, (string) $before->code);

                $this->compiler->compile($id, 'Sonde S153 — staff', '', true, new PackageSpec(hoursExempt: true));
                $after = $this->reservations->book(ReservableType::Machine, (int) $machine->getId(), $member, $start, $end);
                // ⚠️ On n'exige pas « créé » : d'autres règles peuvent refuser ce
                // créneau (quota, chevauchement) et ce n'est pas ce qu'on mesure.
                // Ce qui doit changer, c'est que la FERMETURE ne soit plus la raison.
                $failures += $this->check(
                    $io,
                    'avec la case : ce n’est plus la fermeture qui refuse',
                    $after->code !== 'FABLAB_CLOSED',
                    (string) ($after->code ?? 'créé'),
                );

                return $failures;
            }
        }

        $io->warning('Aucun couple (membre, machine) n’atteint le contrôle d’horaires : une règle antérieure refuse d’abord. Rien mesuré ici.');

        return 0;
    }

    /**
     * Un créneau d'une heure que la grille hebdomadaire refuse, sur une date SANS
     * règle datée — sinon on mesurerait la mauvaise chose : une fermeture datée
     * reste opposable à tout le monde, exemption comprise.
     *
     * @return array{0: \DateTimeImmutable, 1: \DateTimeImmutable}|null
     */
    private function closedSlot(?int $venueId): ?array
    {
        $day = $this->clock->now()->modify('+1 day')->setTime(0, 0);
        for ($i = 0; $i < 14; $i++, $day = $day->modify('+1 day')) {
            if ($this->schedule->hasDatedRulesOn($venueId, $day)) {
                continue;
            }
            for ($hour = 0; $hour < 24; $hour++) {
                $start = $day->setTime($hour, 0);
                $end = $start->modify('+1 hour');
                // ⚠️ Une réservation ne peut pas traverser deux jours : 23:00 est
                // écarté, sinon la sonde mesurerait ce refus-là.
                if ($hour === 23) {
                    continue;
                }
                if ($this->schedule->refusalFor($venueId, $start, $end) !== null) {
                    return [$start, $end];
                }
            }
        }

        return null;
    }

    private function check(SymfonyStyle $io, string $label, bool $ok, string $detail): int
    {
        $io->text(sprintf('%s %s — %s', $ok ? '✅' : '🔴', $label, $detail));

        return $ok ? 0 : 1;
    }
}
