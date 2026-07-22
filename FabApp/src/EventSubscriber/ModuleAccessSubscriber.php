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

        $module = match (true) {
            str_starts_with($route, 'app_leaderboard_creation') => 'projects',
            str_starts_with($route, 'app_leaderboard') => 'leaderboard',
            str_starts_with($route, 'app_formation') => 'formations',
            str_starts_with($route, 'app_lab_page') => 'lab_pages',
            str_starts_with($route, 'app_place') => 'places',
            str_starts_with($route, 'app_event'), $route === 'app_kiosk_events' => 'events',
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
