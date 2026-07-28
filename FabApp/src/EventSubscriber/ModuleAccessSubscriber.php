<?php

namespace App\EventSubscriber;

use App\Service\ModuleService;
use Symfony\Component\EventDispatcher\EventSubscriberInterface;
use Symfony\Component\HttpKernel\Event\RequestEvent;
use Symfony\Component\HttpKernel\Exception\NotFoundHttpException;
use Symfony\Component\HttpKernel\KernelEvents;

/**
 * Returns 404 for the public routes of a module that has been turned off in the admin.
 * Admin routes (managing the module) are never blocked. Runs at priority 0, after the
 * router has resolved `_route`.
 */
final class ModuleAccessSubscriber implements EventSubscriberInterface
{
    public function __construct(private readonly ModuleService $modules)
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
        if (str_starts_with($route, 'app_calendar') && !$this->modules->hasResourceLayer()) {
            throw new NotFoundHttpException();
        }

        $module = match (true) {
            // Every equipment route is named `app_machine…` (list, detail, its
            // own calendar, history, quiz, iCal feed) and nothing else is, so a
            // prefix is safe here — `app_maintenance` diverges at the fourth
            // letter. The admin side is `app_admin_machine…`, already exempt.
            str_starts_with($route, 'app_machine'), $route === 'app_kiosk_machine' => 'machines',
            str_starts_with($route, 'app_leaderboard_creation') => 'projects',
            str_starts_with($route, 'app_leaderboard') => 'leaderboard',
            str_starts_with($route, 'app_formation') => 'formations',
            str_starts_with($route, 'app_lab_page') => 'lab_pages',
            str_starts_with($route, 'app_place') => 'places',
            str_starts_with($route, 'app_event'), $route === 'app_kiosk_events' => 'events',
            // Exact match, NOT a prefix: the staff *directory* is this one route,
            // whereas `app_staff_*` is the staff desk (pass issuing, ticket
            // scanning). A prefix here made turning the directory off also 404 the
            // door scanner — two unrelated things sharing a name.
            $route === 'app_staff' => 'staff',
            str_starts_with($route, 'app_trainer') => 'trainers',
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
