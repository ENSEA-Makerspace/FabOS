<?php

namespace App\Controller;

use App\Repository\ReservationRepository;
use App\Reservation\ReservableResolver;
use App\Reservation\ReservableType;
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
 * The place route is named `app_place_ical` so FeatureAccessSubscriber 404s it
 * when the `places` module is turned off, matching the rest of that module.
 */
final class CalendarFeedController extends AbstractController
{
    #[Route('/calendar/machine/{id}.ics', name: 'app_machine_ical', requirements: ['id' => '\d+'], defaults: ['type' => 'machine'], methods: ['GET'])]
    public function machineFeed(int $id, string $type, Request $request, ReservationRepository $reservations, ReservableResolver $reservables, IcalFeedService $ical, SiteSettingService $siteSettings, TranslatorInterface $translator): Response
    {
        return $this->feed($type, $id, $request, $reservations, $reservables, $ical, $siteSettings, $translator);
    }

    #[Route('/calendar/place/{id}.ics', name: 'app_place_ical', requirements: ['id' => '\d+'], defaults: ['type' => 'place'], methods: ['GET'])]
    public function placeFeed(int $id, string $type, Request $request, ReservationRepository $reservations, ReservableResolver $reservables, IcalFeedService $ical, SiteSettingService $siteSettings, TranslatorInterface $translator): Response
    {
        return $this->feed($type, $id, $request, $reservations, $reservables, $ical, $siteSettings, $translator);
    }

    /**
     * One feed for any reservable kind. The two routes above stay as stable,
     * separately module-gated URLs onto this; a new bookable kind only needs its
     * own route arm, not another copy of the body.
     */
    private function feed(
        string $type,
        int $id,
        Request $request,
        ReservationRepository $reservations,
        ReservableResolver $reservables,
        IcalFeedService $ical,
        SiteSettingService $siteSettings,
        TranslatorInterface $translator,
    ): Response {
        $this->assertValidToken($request, $siteSettings);

        $reservableType = ReservableType::tryParse($type);
        if ($reservableType === null) {
            throw $this->createNotFoundException('Type de ressource inconnu');
        }

        $rows = $reservations->findActiveForReservable($reservableType, $id);
        $reservables->warm($rows);

        $name = $reservables->nameFor($reservableType, $id);
        if ($name === null) {
            throw $this->createNotFoundException('Ressource introuvable');
        }

        $body = $ical->build($name, $this->busyLabel($translator, $siteSettings), $rows);

        return $this->icalResponse($body, $type . '-' . $id);
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
