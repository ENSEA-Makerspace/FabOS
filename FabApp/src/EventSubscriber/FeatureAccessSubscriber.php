<?php

namespace App\EventSubscriber;

use App\Feature\SiteFeatureService;
use Symfony\Component\EventDispatcher\EventSubscriberInterface;
use Symfony\Component\HttpKernel\Event\RequestEvent;
use Symfony\Component\HttpKernel\Exception\NotFoundHttpException;
use Symfony\Component\HttpKernel\KernelEvents;

/**
 * Returns 404 for the public routes of a module that has been turned off in the admin.
 * Admin routes (managing the module) are never blocked. Runs at priority 0, after the
 * router has resolved `_route`.
 *
 * ⚠️ This maps by route *name*, so the map is only as good as the naming. Two rules
 * learned the hard way:
 *
 *  - **A prefix is a promise** that no unrelated feature will ever be named under it.
 *    Use one only where the whole namespace belongs to the module (`app_machine…`).
 *    Where a name is shared by something that is *not* the module — `app_staff` the
 *    directory versus `app_staff_*` the staff desk — match exactly.
 *  - **This gate is access control, not presentation.** It decides whether a page
 *    exists. Nothing about who someone *is* — their roles, their authorisation —
 *    may ever be routed through here; that is kernel.
 */
final class FeatureAccessSubscriber implements EventSubscriberInterface
{
    public function __construct(private readonly SiteFeatureService $modules)
    {
    }

    public function onKernelRequest(RequestEvent $event): void
    {
        if (!$event->isMainRequest()) {
            return;
        }

        $route = (string) $event->getRequest()->attributes->get('_route');
        if ($route === '' || str_contains($route, 'admin')) {
            return;
        }

        // The calendar belongs to no single module: it is the shared grid the
        // resource layers project onto. It stands down only when every one of
        // them is off, rather than following machines around as it used to.
        if (str_starts_with($route, 'app_calendar') && !$this->modules->hasCalendarLayer()) {
            throw new NotFoundHttpException();
        }

        $module = match (true) {
            // Every equipment route is named `app_machine…` (list, detail, its
            // own calendar, history, quiz, iCal feed) and nothing else is, so a
            // prefix is safe here — `app_maintenance` diverges at the fourth
            // letter. The admin side is `app_admin_machine…`, already exempt.
            str_starts_with($route, 'app_machine'), $route === 'app_kiosk_machine' => 'machines',
            // Booking someone's time, including a bookable person's own
            // availability screen. Nothing about *being* staff or a trainer
            // depends on this — see the directory note below.
            str_starts_with($route, 'app_person') => 'person_booking',
            // The gallery has its own namespace since S22. It used to live under
            // `app_leaderboard_creation…`, which had to be matched before
            // `app_leaderboard` or the gallery inherited the leaderboard's gate —
            // the same prefix trap that once 404'd the ticket scanner.
            str_starts_with($route, 'app_creation') => 'projects',
            str_starts_with($route, 'app_leaderboard') => 'leaderboard',
            str_starts_with($route, 'app_formation') => 'formations',
            str_starts_with($route, 'app_lab_page') => 'lab_pages',
            str_starts_with($route, 'app_place') => 'places',
            str_starts_with($route, 'app_event'), $route === 'app_kiosk_events' => 'events',
            // Directories are a *page and a menu entry*, nothing more. Exact
            // matches, NOT prefixes: `app_staff_*` is the staff desk (pass
            // issuing, ticket scanning) and `ROLE_STAFF` authorisation is kernel.
            // A prefix here once made turning the directory off also 404 the door
            // scanner — two unrelated things sharing a name.
            $route === 'app_staff' => 'staff',
            $route === 'app_trainers' => 'trainers',
            str_starts_with($route, 'app_materials') => 'materials',
            str_starts_with($route, 'app_loans') => 'loans',
            str_starts_with($route, 'app_maintenance') => 'maintenance',
            $route === 'app_badges' => 'badges',
            default => null,
        };

        if ($module !== null && !$this->modules->isEnabled($module)) {
            throw new NotFoundHttpException();
        }
    }

    public static function getSubscribedEvents(): array
    {
        return [KernelEvents::REQUEST => [['onKernelRequest', 0]]];
    }
}
