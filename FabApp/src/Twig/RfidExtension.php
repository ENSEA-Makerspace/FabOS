<?php

namespace App\Twig;

use App\UsageRights\RightsExplainer;
use Twig\Extension\AbstractExtension;
use Twig\TwigFilter;

/**
 * S192 — `|rfid_mask` : « ••••A1F2 » au lieu d'un identifiant de badge.
 *
 * Pour beaucoup de cartes bon marché, l'identifiant EST le secret : le lire
 * suffit à cloner la carte. Les écrans d'un membre (et sa fiche vue par l'équipe)
 * n'en montrent que les quatre derniers caractères — assez pour reconnaître SA
 * carte. ⚠️ Le journal RFID de l'administration garde l'identifiant entier :
 * c'est là qu'on repère une carte inconnue pour l'attribuer.
 */
final class RfidExtension extends AbstractExtension
{
    public function getFilters(): array
    {
        return [new TwigFilter('rfid_mask', static fn (?string $uid): string => $uid === null || trim($uid) === '' ? '—' : RightsExplainer::maskRfid($uid))];
    }
}
