<?php

namespace App\Service;

use App\Entity\AccessRfidLog;
use App\Entity\Badge;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Repository\MachineBadgeRepository;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Repository\UtilisateurRepository;
use Doctrine\ORM\EntityManagerInterface;

final class MachineAccessService
{
    public function __construct(
        private readonly MachineRepository $machines,
        private readonly UtilisateurRepository $users,
        private readonly MachineBadgeRepository $machineBadges,
        private readonly UtilisateurBadgeRepository $userBadges,
        private readonly EntityManagerInterface $entityManager,
    ) {
    }

    /**
     * @return array<string, mixed>
     */
    public function authorize(string $machineToken, string $rfid): array
    {
        $machineToken = trim($machineToken);
        $rfid = trim($rfid);
        $machine = $this->machines->findOneByMachineToken($machineToken);

        if (!$machine instanceof Machine) {
            $result = $this->buildResult(false, 'unknown_machine', 'Machine inconnue', 404, null, null);
            $this->logAttempt($rfid, null, null, $result);

            return $result;
        }

        $user = $this->users->findOneByRfid($rfid);
        if (!$user instanceof Utilisateur) {
            $result = $this->buildResult(false, 'unknown_rfid', 'Badge RFID inconnu', 404, $machine, null);
            $this->logAttempt($rfid, $machine, null, $result);

            return $result;
        }

        $requiredBadgeEntities = [];
        foreach ($this->machineBadges->findRequiredForMachine($machine) as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if ($badge instanceof Badge) {
                $requiredBadgeEntities[] = $badge;
            }
        }

        $userBadgeEntities = [];
        foreach ($this->userBadges->findBy(['utilisateur' => $user]) as $userBadge) {
            $badge = $userBadge->getBadge();
            if ($badge instanceof Badge) {
                $userBadgeEntities[] = $badge;
            }
        }

        $requiredBadges = $this->badgeNames($requiredBadgeEntities);
        $userBadges = $this->badgeNames($userBadgeEntities);
        $matchedBadges = [];

        foreach ($requiredBadgeEntities as $requiredBadge) {
            foreach ($userBadgeEntities as $userBadge) {
                if ($requiredBadge->getId() !== null && $requiredBadge->getId() === $userBadge->getId()) {
                    $matchedBadges[] = $requiredBadge->getNom();
                    break;
                }
            }
        }

        if ($requiredBadges === []) {
            $result = $this->buildResult(true, 'no_badge_required', 'Accès autorisé sans badge requis', 200, $machine, $user, $requiredBadges, $userBadges, []);
            $this->markAuthorization($machine);
            $this->logAttempt($rfid, $machine, $user, $result);

            return $result;
        }

        if ($matchedBadges !== []) {
            $result = $this->buildResult(true, 'authorized', 'Accès autorisé', 200, $machine, $user, $requiredBadges, $userBadges, array_values(array_unique($matchedBadges)));
            $this->markAuthorization($machine);
            $this->logAttempt($rfid, $machine, $user, $result);

            return $result;
        }

        $result = $this->buildResult(false, 'missing_badge', 'Badge requis manquant', 403, $machine, $user, $requiredBadges, $userBadges, []);
        $this->logAttempt($rfid, $machine, $user, $result);

        return $result;
    }

    public function logInvalidPayload(string $machineToken, ?string $rfid = null): void
    {
        $machine = $this->machines->findOneByMachineToken($machineToken);
        $result = $this->buildResult(false, 'invalid_payload', 'Payload JSON invalide ou champ rfid manquant', 400, $machine, null);
        $this->logAttempt(trim((string) $rfid), $machine, null, $result);
    }

    /**
     * @param array<string, mixed> $result
     */
    private function logAttempt(string $rfid, ?Machine $machine, ?Utilisateur $user, array $result): void
    {
        $log = (new AccessRfidLog())
            ->setBadgeUid($rfid)
            ->setMachine($machine)
            ->setUtilisateur($user)
            ->setAuthorized((bool) ($result['authorized'] ?? false))
            ->setStatus((string) ($result['status'] ?? 'server_error'))
            ->setReason((string) ($result['status'] ?? 'server_error'))
            ->setMessage((string) ($result['message'] ?? 'Erreur serveur'))
            ->setColor($this->colorForStatus((string) ($result['status'] ?? 'server_error')));

        $this->entityManager->persist($log);
        $this->entityManager->flush();
    }

    private function markAuthorization(Machine $machine): void
    {
        $machine->setLastAuthorizationTime(new \DateTimeImmutable());
        $this->entityManager->flush();
    }

    /**
     * @param string[] $requiredBadges
     * @param string[] $userBadges
     * @param string[] $matchedBadges
     * @return array<string, mixed>
     */
    private function buildResult(
        bool $authorized,
        string $status,
        string $message,
        int $httpStatus,
        ?Machine $machine,
        ?Utilisateur $user,
        array $requiredBadges = [],
        array $userBadges = [],
        array $matchedBadges = [],
    ): array {
        return [
            'authorized' => $authorized,
            'status' => $status,
            'message' => $message,
            'machine' => $machine,
            'user' => $user,
            'requiredBadges' => $requiredBadges,
            'userBadges' => $userBadges,
            'matchedBadges' => $matchedBadges,
            'httpStatus' => $httpStatus,
        ];
    }

    /** @param Badge[] $badges */
    private function badgeNames(array $badges): array
    {
        return array_values(array_unique(array_map(static fn (Badge $badge): string => $badge->getNom(), $badges)));
    }

    private function colorForStatus(string $status): string
    {
        return match ($status) {
            'authorized', 'no_badge_required' => 'green',
            'missing_badge' => 'orange',
            'unknown_rfid', 'unknown_machine', 'invalid_payload' => 'red',
            'unauthorized_device', 'server_error' => 'purple',
            default => 'purple',
        };
    }
}
