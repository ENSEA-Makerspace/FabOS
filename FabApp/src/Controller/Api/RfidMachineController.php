<?php

namespace App\Controller\Api;

use App\Entity\LogUtilisation;
use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Service\MachineAccessService;
use App\Service\WorkSessionService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\Routing\Attribute\Route;

#[Route('/api/rfid/machines')]
final class RfidMachineController extends AbstractController
{
    public function __construct(
        private readonly MachineAccessService $machineAccess,
        private readonly WorkSessionService $workSessions,
    ) {
    }

    #[Route('/{machineToken}/authorization', name: 'api_rfid_machine_authorization', requirements: ['machineToken' => '[^/]+'], methods: ['POST'])]
    public function authorize(string $machineToken, Request $request): JsonResponse
    {
        if ($response = $this->rejectUnauthorizedDevice($request)) {
            return $response;
        }

        $payload = $this->decodeRfidPayload($request);
        if ($payload instanceof JsonResponse) {
            $this->machineAccess->logInvalidPayload($machineToken);
            return $payload;
        }

        try {
            $result = $this->machineAccess->authorize($machineToken, $payload['rfid']);

            return $this->json($this->formatAccessResult($result), (int) $result['httpStatus']);
        } catch (\Throwable) {
            return $this->json($this->serverErrorPayload(), 500);
        }
    }

    #[Route('/{machineToken}/work-sessions', name: 'api_rfid_machine_work_session_start', requirements: ['machineToken' => '[^/]+'], methods: ['POST'])]
    public function startWorkSession(string $machineToken, Request $request): JsonResponse
    {
        if ($response = $this->rejectUnauthorizedDevice($request)) {
            return $response;
        }

        $payload = $this->decodeRfidPayload($request);
        if ($payload instanceof JsonResponse) {
            $this->machineAccess->logInvalidPayload($machineToken);
            return $payload;
        }

        try {
            $result = $this->workSessions->start($machineToken, $payload['rfid']);
            $status = (string) ($result['status'] ?? 'server_error');

            if (($result['session'] ?? null) instanceof LogUtilisation) {
                return $this->json([
                    'status' => $status,
                    'message' => (string) $result['message'],
                    'session' => $this->sessionToArray($result['session']),
                ], (int) $result['httpStatus']);
            }

            if (array_key_exists('authorized', $result)) {
                return $this->json($this->formatAccessResult($result), (int) $result['httpStatus']);
            }

            return $this->json([
                'status' => $status,
                'message' => (string) ($result['message'] ?? 'Conflit session'),
            ], (int) ($result['httpStatus'] ?? 500));
        } catch (\Throwable) {
            return $this->json($this->serverErrorPayload(), 500);
        }
    }

    #[Route('/{machineToken}/work-sessions/current', name: 'api_rfid_machine_work_session_stop', requirements: ['machineToken' => '[^/]+'], methods: ['PATCH'])]
    public function stopWorkSession(string $machineToken, Request $request): JsonResponse
    {
        if ($response = $this->rejectUnauthorizedDevice($request)) {
            return $response;
        }

        $payload = $this->decodeRfidPayload($request);
        if ($payload instanceof JsonResponse) {
            $this->machineAccess->logInvalidPayload($machineToken);
            return $payload;
        }

        try {
            $result = $this->workSessions->stop($machineToken, $payload['rfid']);

            if (($result['session'] ?? null) instanceof LogUtilisation) {
                return $this->json([
                    'status' => (string) $result['status'],
                    'message' => (string) $result['message'],
                    'session' => $this->sessionToArray($result['session']),
                ], (int) $result['httpStatus']);
            }

            return $this->json([
                'authorized' => $result['authorized'] ?? false,
                'status' => (string) ($result['status'] ?? 'server_error'),
                'message' => (string) ($result['message'] ?? 'Erreur serveur'),
            ], (int) ($result['httpStatus'] ?? 500));
        } catch (\Throwable) {
            return $this->json($this->serverErrorPayload(), 500);
        }
    }

    /** @return array{rfid: string}|JsonResponse */
    private function decodeRfidPayload(Request $request): array|JsonResponse
    {
        $payload = json_decode($request->getContent(), true);
        if (!is_array($payload)) {
            return $this->invalidPayloadResponse();
        }

        $rfid = $payload['rfid'] ?? null;
        if (!is_string($rfid) || trim($rfid) === '') {
            return $this->invalidPayloadResponse();
        }

        return ['rfid' => trim($rfid)];
    }

    private function invalidPayloadResponse(): JsonResponse
    {
        return $this->json([
            'authorized' => false,
            'status' => 'invalid_payload',
            'message' => 'Payload JSON invalide ou champ rfid manquant',
        ], 400);
    }

    private function rejectUnauthorizedDevice(Request $request): ?JsonResponse
    {
        $configuredToken = getenv('FABOS_RFID_API_TOKEN');
        $expectedToken = trim((string) ($configuredToken !== false ? $configuredToken : ($_ENV['FABOS_RFID_API_TOKEN'] ?? $_SERVER['FABOS_RFID_API_TOKEN'] ?? '')));
        if ($expectedToken === '') {
            return null;
        }

        $providedToken = trim((string) $request->headers->get('X-FABOS-DEVICE-TOKEN', ''));
        if (!hash_equals($expectedToken, $providedToken)) {
            return $this->json([
                'authorized' => false,
                'status' => 'unauthorized_device',
                'message' => 'Token machine invalide ou manquant',
            ], 401);
        }

        return null;
    }

    /** @param array<string, mixed> $result */
    private function formatAccessResult(array $result): array
    {
        $payload = [
            'authorized' => (bool) ($result['authorized'] ?? false),
            'status' => (string) ($result['status'] ?? 'server_error'),
            'message' => (string) ($result['message'] ?? 'Erreur serveur'),
        ];

        if (($result['machine'] ?? null) instanceof Machine) {
            $payload['machine'] = $this->machineToArray($result['machine']);
        }

        if (($result['user'] ?? null) instanceof Utilisateur) {
            $payload['user'] = $this->userToArray($result['user']);
        }

        $status = (string) ($result['status'] ?? 'server_error');
        if (in_array($status, ['authorized', 'no_badge_required', 'missing_badge'], true)) {
            $payload['requiredBadges'] = $result['requiredBadges'] ?? [];
        }

        if (($result['status'] ?? null) === 'missing_badge') {
            $payload['userBadges'] = $result['userBadges'] ?? [];
        }

        if (($result['authorized'] ?? false) === true) {
            $payload['matchedBadges'] = $result['matchedBadges'] ?? [];
        }

        return $payload;
    }

    private function sessionToArray(LogUtilisation $session): array
    {
        $machine = $session->getMachine();
        $user = $session->getUtilisateur();

        return [
            'id' => $session->getId(),
            'machine' => $machine instanceof Machine ? $this->machineToArray($machine) : null,
            'user' => $user instanceof Utilisateur ? $this->userToArray($user) : null,
            'dateDebut' => $session->getDateDebut()->format(DATE_ATOM),
            'dateFin' => $session->getDateFin()?->format(DATE_ATOM),
            'duree' => $session->getDuree(),
            'source' => $session->getSource(),
        ];
    }

    private function machineToArray(Machine $machine): array
    {
        return [
            'id' => $machine->getId(),
            'token' => $machine->getMachineToken(),
            'name' => $machine->getNom(),
        ];
    }

    private function userToArray(Utilisateur $user): array
    {
        return [
            'id' => $user->getId(),
            'name' => $user->getDisplayName(),
        ];
    }

    private function serverErrorPayload(): array
    {
        return [
            'authorized' => false,
            'status' => 'server_error',
            'message' => 'Erreur serveur imprévue',
        ];
    }
}
