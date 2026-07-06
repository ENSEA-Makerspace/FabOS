<?php

namespace App\Service;

use App\Entity\LogUtilisation;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\LogUtilisationRepository;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use Doctrine\ORM\EntityManagerInterface;

final class WorkSessionService
{
    public function __construct(
        private readonly MachineAccessService $machineAccess,
        private readonly MachineRepository $machines,
        private readonly UtilisateurRepository $users,
        private readonly LogUtilisationRepository $usageLogs,
        private readonly EntityManagerInterface $entityManager,
    ) {
    }

    /** @return array<string, mixed> */
    public function start(string $machineToken, string $rfid): array
    {
        $access = $this->machineAccess->authorize($machineToken, $rfid);
        if (($access['authorized'] ?? false) !== true) {
            return $access;
        }

        $machine = $access['machine'] ?? null;
        $user = $access['user'] ?? null;
        if (!$machine instanceof Machine || !$user instanceof Utilisateur) {
            return [
                'status' => 'server_error',
                'message' => 'Autorisation incohérente',
                'httpStatus' => 500,
            ];
        }

        if ($this->usageLogs->findOpenForMachine($machine) instanceof LogUtilisation) {
            return [
                'status' => 'machine_already_in_use',
                'message' => 'Une session est déjà ouverte pour cette machine',
                'httpStatus' => 409,
            ];
        }

        if ($this->usageLogs->findOpenForUser($user) instanceof LogUtilisation) {
            return [
                'status' => 'user_already_working',
                'message' => 'Cet utilisateur a déjà une session ouverte',
                'httpStatus' => 409,
            ];
        }

        $session = (new LogUtilisation())
            ->setMachine($machine)
            ->setUtilisateur($user)
            ->setDateDebut(new \DateTimeImmutable())
            ->setDateFin(null)
            ->setDuree(null)
            ->setSource('rfid');

        if (in_array($machine->getStatut(), ['idle', 'disponible'], true)) {
            $machine->setStatut('active');
        }

        $this->entityManager->persist($session);
        $this->entityManager->flush();

        return [
            'status' => 'work_session_started',
            'message' => 'Session d’utilisation démarrée',
            'session' => $session,
            'httpStatus' => 201,
        ];
    }

    /** @return array<string, mixed> */
    public function stop(string $machineToken, string $rfid): array
    {
        $machine = $this->machines->findOneByMachineToken($machineToken);
        if (!$machine instanceof Machine) {
            return [
                'authorized' => false,
                'status' => 'unknown_machine',
                'message' => 'Machine inconnue',
                'httpStatus' => 404,
            ];
        }

        $user = $this->users->findOneByRfid($rfid);
        if (!$user instanceof Utilisateur) {
            return [
                'authorized' => false,
                'status' => 'unknown_rfid',
                'message' => 'Badge RFID inconnu',
                'httpStatus' => 404,
            ];
        }

        $session = $this->usageLogs->findCurrentForMachineAndUser($machine, $user);
        if (!$session instanceof LogUtilisation) {
            return [
                'status' => 'no_open_session',
                'message' => 'Aucune session ouverte pour cette machine et cet utilisateur',
                'httpStatus' => 404,
            ];
        }

        $dateFin = new \DateTimeImmutable();
        $duration = max(0, $dateFin->getTimestamp() - $session->getDateDebut()->getTimestamp());

        $session
            ->setDateFin($dateFin)
            ->setDuree($duration);

        if ($machine->getStatut() === 'active') {
            $machine->setStatut('idle');
        }

        $this->entityManager->flush();

        return [
            'status' => 'work_session_stopped',
            'message' => 'Session d’utilisation terminée',
            'session' => $session,
            'httpStatus' => 200,
        ];
    }
}
