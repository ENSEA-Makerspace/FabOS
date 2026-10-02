<?php

declare(strict_types=1);

namespace App\Catalogue;

use App\Repository\AccessPointRepository;
use App\Repository\MachineRepository;
use App\Reservation\ReservableType;

/**
 * « Mes réservations » : la photo de chaque machine réservée et les portes de
 * chaque espace réservé, que `MyReservations::build()` ne charge pas.
 */
final class MyReservationVisuals
{
    public function __construct(
        private readonly MachineRepository $machines,
        private readonly AccessPointRepository $doors,
    ) {
    }

    /**
     * @param array<string, mixed> $data résultat de `MyReservations::build()`
     * @return array<string, mixed> `$data` + `photoOf` + `doorsOf`
     */
    public function add(array $data): array
    {
        $photoOf = [];
        $doorsOf = [];
        foreach ($data['reservations'] ?? [] as $reservation) {
            $id = (int) $reservation->getReservableId();
            if ($reservation->getReservableType() === ReservableType::Machine) {
                $photoOf[$reservation->getId()] = $this->machines->find($id)?->getPhoto();
            } elseif ($reservation->getReservableType() === ReservableType::Place) {
                $doorsOf[$reservation->getId()] = array_map(static fn ($d): string => (string) $d->getNom(), $this->doors->findForPlace($id));
            }
        }

        return $data + ['photoOf' => $photoOf, 'doorsOf' => $doorsOf];
    }
}
