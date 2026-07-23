<?php

namespace App\Reservation;

/**
 * A resolved reservable resource, in the uniform shape every listing page and
 * feed needs — so nothing downstream has to branch on machine-vs-place again.
 *
 * $exists is false when the resource has been deleted since the booking (no FK,
 * so nothing cascaded the reservation away). In that case $name falls back to
 * the label snapshotted at booking time and the links are null.
 */
final readonly class ReservableRef
{
    public function __construct(
        public ?ReservableType $type,
        public ?int $id,
        public string $name,
        public bool $exists,
        public ?string $url = null,
        public ?string $calendarUrl = null,
    ) {
    }

    /** A reservation whose resource kind is unknown — pre-backfill or bad data. */
    public static function unknown(string $name): self
    {
        return new self(null, null, $name, false);
    }
}
