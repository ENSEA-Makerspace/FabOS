<?php

namespace App\Controller\Api;

use App\Entity\MachineFavorite;
use App\Entity\Utilisateur;
use App\Repository\MachineFavoriteRepository;
use App\Repository\MachineRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\JsonResponse;
use Symfony\Component\Routing\Attribute\Route;

#[Route('/api/me/favorite-machines')]
final class MachineFavoriteController extends AbstractController
{
    #[Route('', name: 'api_me_favorite_machines', methods: ['GET'])]
    public function index(MachineFavoriteRepository $favorites): JsonResponse
    {
        $user = $this->getCurrentUser();
        if (!$user instanceof Utilisateur) {
            return $this->unauthorizedResponse();
        }

        if (!$favorites->isStorageReady()) {
            return $this->storageMissingResponse();
        }

        return new JsonResponse([
            'favorites' => $favorites->findMachineSummariesForUser($user),
        ]);
    }

    #[Route('/{machineId}', name: 'api_me_favorite_machine_add', requirements: ['machineId' => '\d+'], methods: ['POST'])]
    public function add(int $machineId, MachineRepository $machines, MachineFavoriteRepository $favorites, EntityManagerInterface $entityManager): JsonResponse
    {
        $user = $this->getCurrentUser();
        if (!$user instanceof Utilisateur) {
            return $this->unauthorizedResponse();
        }

        if (!$favorites->isStorageReady()) {
            return $this->storageMissingResponse();
        }

        $machine = $machines->find($machineId);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable', 'code' => 'MACHINE_NOT_FOUND'], 404);
        }

        $favorite = $favorites->findOneForUserAndMachine($user, $machine);
        if (!$favorite instanceof MachineFavorite) {
            $favorite = (new MachineFavorite())
                ->setUtilisateur($user)
                ->setMachine($machine);
            $entityManager->persist($favorite);
            $entityManager->flush();
        }

        return new JsonResponse([
            'favorited' => true,
            'machineId' => $machine->getId(),
            'message' => 'Machine ajoutée aux favoris.',
        ], 200);
    }

    #[Route('/{machineId}', name: 'api_me_favorite_machine_remove', requirements: ['machineId' => '\d+'], methods: ['DELETE'])]
    public function remove(int $machineId, MachineRepository $machines, MachineFavoriteRepository $favorites, EntityManagerInterface $entityManager): JsonResponse
    {
        $user = $this->getCurrentUser();
        if (!$user instanceof Utilisateur) {
            return $this->unauthorizedResponse();
        }

        if (!$favorites->isStorageReady()) {
            return $this->storageMissingResponse();
        }

        $machine = $machines->find($machineId);
        if (!$machine) {
            return new JsonResponse(['error' => 'Machine introuvable', 'code' => 'MACHINE_NOT_FOUND'], 404);
        }

        $favorite = $favorites->findOneForUserAndMachine($user, $machine);
        if ($favorite instanceof MachineFavorite) {
            $entityManager->remove($favorite);
            $entityManager->flush();
        }

        return new JsonResponse([
            'favorited' => false,
            'machineId' => $machine->getId(),
            'message' => 'Machine retirée des favoris.',
        ]);
    }

    private function getCurrentUser(): ?Utilisateur
    {
        $user = $this->getUser();

        return $user instanceof Utilisateur ? $user : null;
    }

    private function unauthorizedResponse(): JsonResponse
    {
        return new JsonResponse([
            'error' => 'Connectez-vous pour utiliser les favoris.',
            'code' => 'AUTHENTICATION_REQUIRED',
        ], 401);
    }

    private function storageMissingResponse(): JsonResponse
    {
        return new JsonResponse([
            'error' => 'La table des favoris machines n’est pas encore installée.',
            'code' => 'FAVORITES_STORAGE_MISSING',
        ], 503);
    }
}
