<?php

namespace App\Service;

use App\Entity\OpeningHour;
use App\Repository\OpeningHourRepository;
use App\Repository\VenueRepository;
use Psr\Log\LoggerInterface;

class OpeningHoursProvider
{
    private const DEFAULT_ROWS = [
        1 => ['label' => 'Lundi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        2 => ['label' => 'Mardi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        3 => ['label' => 'Mercredi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        4 => ['label' => 'Jeudi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '20:00'],
        5 => ['label' => 'Vendredi', 'isClosed' => false, 'openTime' => '08:00', 'closeTime' => '22:00'],
        6 => ['label' => 'Samedi', 'isClosed' => false, 'openTime' => '09:00', 'closeTime' => '18:00'],
        7 => ['label' => 'Dimanche', 'isClosed' => true, 'openTime' => null, 'closeTime' => null],
    ];

    public function __construct(
        private readonly OpeningHourRepository $openingHours,
        private readonly VenueRepository $venues,
        private readonly LoggerInterface $logger,
    ) {
    }

    /** @return OpeningHour[] */
    public function getOpeningHours(): array
    {
        $venue = $this->venues->findDefault();
        $rows = $venue === null ? [] : $this->openingHours->findOrdered($venue);
        if (count($rows) === 7) {
            return $rows;
        }

        $this->logger->warning('OPENING_HOUR table is incomplete; using default opening hours fallback.', [
            'count' => count($rows),
        ]);

        return $this->getDefaultOpeningHours();
    }

    /** @return array<int, array<string, mixed>> */
    public function getOpeningHoursForJson(): array
    {
        return array_map(static fn (OpeningHour $hour): array => [
            'dayOfWeek' => $hour->getDayOfWeek(),
            'dayIndex' => $hour->getDayOfWeek() - 1,
            'label' => $hour->getLabel(),
            'isClosed' => $hour->isClosed(),
            'openTime' => $hour->getOpenTime()?->format('H:i'),
            'closeTime' => $hour->getCloseTime()?->format('H:i'),
            'displayLabel' => $hour->getDisplayLabel(),
        ], $this->getOpeningHours());
    }

    public function getCalendarStartHour(): int
    {
        $openHours = array_filter($this->getOpeningHours(), static fn (OpeningHour $hour): bool => !$hour->isClosed() && $hour->getOpenTime() !== null);
        if ($openHours === []) {
            return 8;
        }

        return min(array_map(static fn (OpeningHour $hour): int => (int) $hour->getOpenTime()->format('G'), $openHours));
    }

    public function getCalendarEndHour(): int
    {
        $openHours = array_filter($this->getOpeningHours(), static fn (OpeningHour $hour): bool => !$hour->isClosed() && $hour->getCloseTime() !== null);
        if ($openHours === []) {
            return 20;
        }

        return max(array_map(static fn (OpeningHour $hour): int => (int) ceil(((int) $hour->getCloseTime()->format('H') * 60 + (int) $hour->getCloseTime()->format('i')) / 60), $openHours));
    }

    public function validateReservationPeriod(\DateTimeImmutable $dateDebut, \DateTimeImmutable $dateFin): ?string
    {
        if ($dateDebut->format('Y-m-d') !== $dateFin->format('Y-m-d')) {
            return 'Une réservation ne peut pas traverser deux jours différents.';
        }

        $openingHour = $this->findOpeningHourForDate($dateDebut);
        if (!$openingHour || $openingHour->isClosed() || !$openingHour->getOpenTime() || !$openingHour->getCloseTime()) {
            return 'Le FabLab est fermé sur ce créneau.';
        }

        $openAt = $dateDebut->setTime((int) $openingHour->getOpenTime()->format('H'), (int) $openingHour->getOpenTime()->format('i'));
        $closeAt = $dateDebut->setTime((int) $openingHour->getCloseTime()->format('H'), (int) $openingHour->getCloseTime()->format('i'));
        if ($dateDebut < $openAt || $dateFin > $closeAt) {
            return 'Le FabLab est fermé sur ce créneau.';
        }

        return null;
    }

    /**
     * The lab's open window on a given date, as minutes since midnight, or null
     * when it is closed. The slot engine intersects a person's availability with
     * this rather than re-reading the OPENING_HOUR rows itself.
     *
     * @return array{start: int, end: int}|null
     */
    public function getOpenMinutesFor(\DateTimeInterface $date): ?array
    {
        $openingHour = $this->findOpeningHourForDate($date);
        if (!$openingHour || $openingHour->isClosed() || !$openingHour->getOpenTime() || !$openingHour->getCloseTime()) {
            return null;
        }

        return [
            'start' => (int) $openingHour->getOpenTime()->format('H') * 60 + (int) $openingHour->getOpenTime()->format('i'),
            'end' => (int) $openingHour->getCloseTime()->format('H') * 60 + (int) $openingHour->getCloseTime()->format('i'),
        ];
    }

    private function findOpeningHourForDate(\DateTimeInterface $dateTime): ?OpeningHour
    {
        foreach ($this->getOpeningHours() as $openingHour) {
            if ($openingHour->appliesTo($dateTime)) {
                return $openingHour;
            }
        }

        return null;
    }

    /** @return OpeningHour[] */
    private function getDefaultOpeningHours(): array
    {
        $rows = [];
        foreach (self::DEFAULT_ROWS as $dayOfWeek => $data) {
            $rows[] = (new OpeningHour())
                ->setDayOfWeek($dayOfWeek)
                ->setLabel($data['label'])
                ->setIsClosed($data['isClosed'])
                ->setOpenTime($data['openTime'] ? \DateTime::createFromFormat('H:i', $data['openTime']) : null)
                ->setCloseTime($data['closeTime'] ? \DateTime::createFromFormat('H:i', $data['closeTime']) : null)
                ->setSortOrder($dayOfWeek);
        }

        return $rows;
    }
}
