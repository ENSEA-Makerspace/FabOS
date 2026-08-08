<?php

namespace App\Reservation;

/**
 * The kinds of resource a Reservation can point at. Stored as the string value
 * in RESERVATION.reservableType — adding a bookable resource means adding a case
 * here plus a branch in ReservableResolver, not a column on RESERVATION.
 */
enum ReservableType: string
{
    case Machine = 'machine';
    case Place = 'place';
    case User = 'user';

    /** Null-tolerant parse: unknown/legacy values resolve to null rather than throwing. */
    public static function tryParse(?string $value): ?self
    {
        return $value === null ? null : self::tryFrom($value);
    }

    /** Translation key for the resource-kind label ("Machine", "Espace", "Personne"). */
    public function labelKey(): string
    {
        return 'reservable.type.' . $this->value;
    }
}
