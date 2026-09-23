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

        // 🔴 **S190 — un compte désactivé n'ouvre plus rien avec son badge.**
        // Avant, rien ici ne lisait le statut : « inactif » coupait l'écran de
        // connexion, et le badge continuait d'ouvrir les machines. Désactiver
        // quelqu'un qui part en mauvais termes, c'est d'abord ça qu'on attend.
        // ⚠️ Avant les badges : aucun badge détenu ne rachète un compte coupé.
        if ($user->getStatut() !== 'actif') {
            $result = $this->buildResult(false, 'account_inactive', 'Compte désactivé', 403, $machine, $user);
            $this->logAttempt($rfid, $machine, $user, $result);

            return $result;
        }

        $requiredBadgeEntities = $this->requiredBadges($machine);
        $userBadgeEntities = $this->heldBadges($user);
        $requiredBadges = $this->badgeNames($requiredBadgeEntities);
        $userBadges = $this->badgeNames($userBadgeEntities);
        $rule = self::badgeRule($requiredBadgeEntities, $userBadgeEntities);

        if ($rule['status'] === 'no_badge_required') {
            $result = $this->buildResult(true, 'no_badge_required', 'Accès autorisé sans badge requis', 200, $machine, $user, $requiredBadges, $userBadges, []);
            $this->markAuthorization($machine);
            $this->logAttempt($rfid, $machine, $user, $result);

            return $result;
        }

        if ($rule['status'] === 'authorized') {
            $result = $this->buildResult(true, 'authorized', 'Accès autorisé', 200, $machine, $user, $requiredBadges, $userBadges, $this->badgeNames($rule['matched']));
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
    /**
     * 🔴 S192 — LA règle du badge, une seule fois. `authorize()` la suit au scan,
     * `reachFor()` la suit pour l'EXPLIQUER à l'écran : une seconde copie aurait
     * dérivé, et l'écran aurait dit « ouvre » là où le boîtier refuse.
     * Aucun badge exigé → ouvert ; sinon il suffit d'UN des badges exigés.
     *
     * @param list<Badge> $required
     * @param list<Badge> $held
     *
     * @return array{status: 'no_badge_required'|'authorized'|'missing_badge', matched: list<Badge>}
     */
    public static function badgeRule(array $required, array $held): array
    {
        if ($required === []) {
            return ['status' => 'no_badge_required', 'matched' => []];
        }
        $heldIds = [];
        foreach ($held as $badge) {
            if ($badge->getId() !== null) {
                $heldIds[$badge->getId()] = true;
            }
        }
        $matched = array_values(array_filter($required, static fn (Badge $badge): bool => $badge->getId() !== null && isset($heldIds[$badge->getId()])));

        return ['status' => $matched !== [] ? 'authorized' : 'missing_badge', 'matched' => $matched];
    }

    /**
     * Ce que le badge de cette personne ouvre, machine par machine — pour
     * l'expliquer, jamais pour décider (le scan passe par `authorize()`).
     * ⚠️ Un compte désactivé n'ouvre rien : même garde qu'au scan. Les machines
     * archivées sortent, comme sur la fiche d'une formation (S179).
     *
     * @return list<array{machine: Machine, status: string, matched: list<Badge>, required: list<Badge>}>
     */
    public function reachFor(Utilisateur $user): array
    {
        $held = $this->heldBadges($user);
        $rows = [];
        foreach ($this->machines->findBy([], ['nom' => 'ASC']) as $machine) {
            if (!$machine instanceof Machine || $machine->getArchivedAt() !== null) {
                continue;
            }
            $required = $this->requiredBadges($machine);
            $rule = $user->getStatut() !== 'actif'
                ? ['status' => 'account_inactive', 'matched' => []]
                : self::badgeRule($required, $held);
            $rows[] = ['machine' => $machine, 'status' => $rule['status'], 'matched' => $rule['matched'], 'required' => $required];
        }

        return $rows;
    }

    /** @return list<Badge> */
    private function requiredBadges(Machine $machine): array
    {
        $badges = [];
        foreach ($this->machineBadges->findRequiredForMachine($machine) as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if ($badge instanceof Badge) {
                $badges[] = $badge;
            }
        }

        return $badges;
    }

    /** @return list<Badge> */
    private function heldBadges(Utilisateur $user): array
    {
        $badges = [];
        foreach ($this->userBadges->findBy(['utilisateur' => $user]) as $userBadge) {
            $badge = $userBadge->getBadge();
            if ($badge instanceof Badge) {
                $badges[] = $badge;
            }
        }

        return $badges;
    }

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
