<?php

namespace App\Controller;

use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Repository\ReservationRepository;
use App\Service\IcalFeedService;
use App\Service\SiteSettingService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpKernel\Exception\AccessDeniedHttpException;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Read-only iCal (.ics) feeds per reservable resource, so anyone with the feed
 * link can subscribe to a machine's or space's booking calendar on their phone.
 * Gated by a shared, rotatable token (query string); feeds expose busy slots
 * only, never who booked them.
 *
 * The place route is named `app_place_ical` so ModuleAccessSubscriber 404s it
 * when the `places` module is turned off, matching the rest of that module.
 */
final class CalendarFeedController extends AbstractController
{
    #[Route('/calendar/machine/{id}.ics', name: 'app_machine_ical', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function machineFeed(
        int $id,
        Request $request,
        MachineRepository $machines,
        ReservationRepository $reservations,
        IcalFeedService $ical,
        SiteSettingService $siteSettings,
        TranslatorInterface $translator,
    ): Response {
        $this->assertValidToken($request, $siteSettings);

        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $body = $ical->build(
            $machine->getNom(),
            $this->busyLabel($translator, $siteSettings),
            $reservations->findActiveByMachine($machine, ['dateDebut' => 'ASC']),
        );

        return $this->icalResponse($body, 'machine-' . $id);
    }

    #[Route('/calendar/place/{id}.ics', name: 'app_place_ical', requirements: ['id' => '\d+'], methods: ['GET'])]
    public function placeFeed(
        int $id,
        Request $request,
        PlaceRepository $places,
        ReservationRepository $reservations,
        IcalFeedService $ical,
        SiteSettingService $siteSettings,
        TranslatorInterface $translator,
    ): Response {
        $this->assertValidToken($request, $siteSettings);

        $place = $places->find($id);
        if (!$place) {
            throw $this->createNotFoundException('Espace introuvable');
        }

        $body = $ical->build(
            $place->getNom(),
            $this->busyLabel($translator, $siteSettings),
            $reservations->findActiveByPlace($place, ['dateDebut' => 'ASC']),
        );

        return $this->icalResponse($body, 'place-' . $id);
    }

    private function assertValidToken(Request $request, SiteSettingService $siteSettings): void
    {
        $expected = $siteSettings->getIcalFeedToken();
        $provided = (string) $request->query->get('token', '');

        if ($expected === '' || !hash_equals($expected, $provided)) {
            throw new AccessDeniedHttpException('Jeton de flux invalide.');
        }
    }

    /** Localised event-title suffix, rendered in the site's default locale. */
    private function busyLabel(TranslatorInterface $translator, SiteSettingService $siteSettings): string
    {
        return $translator->trans('ical.busy', [], null, $siteSettings->getDefaultLocale());
    }

    private function icalResponse(string $body, string $filename): Response
    {
        $response = new Response($body);
        $response->headers->set('Content-Type', 'text/calendar; charset=utf-8');
        $response->headers->set('Content-Disposition', 'inline; filename="fabos-' . $filename . '.ics"');

        return $response;
    }
}
