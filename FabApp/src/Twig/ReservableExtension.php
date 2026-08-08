<?php

namespace App\Twig;

use App\Reservation\ReservableResolver;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Exposes `reservable(reservation)` so templates read one uniform ReservableRef
 * (name/url/calendarUrl/exists) instead of branching on machine-vs-place. The
 * resolver caches per request, so repeated calls in a loop are free once the
 * controller has warmed it.
 */
final class ReservableExtension extends AbstractExtension
{
    public function __construct(private readonly ReservableResolver $resolver)
    {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('reservable', $this->resolver->resolve(...)),
        ];
    }
}
