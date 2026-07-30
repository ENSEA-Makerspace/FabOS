<?php

namespace App\EventSubscriber;

use App\Http\MissingPageLog;
use Symfony\Component\EventDispatcher\EventSubscriberInterface;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpKernel\Event\ExceptionEvent;
use Symfony\Component\HttpKernel\Exception\NotFoundHttpException;
use Symfony\Component\HttpKernel\KernelEvents;

/**
 * Counts the pages people ask for and do not get.
 *
 * Almost all of the work here is deciding what *not* to record, because a raw
 * 404 log is mostly bots and is therefore useless. Three filters, each removing
 * a whole class of noise:
 *
 *  1. **No route matched, or the feature is off.** A 404 raised by a controller
 *     that *did* match a route — reservation 4211 does not exist — is not a
 *     missing page, it is a missing row, and it would bury the signal. The one
 *     exception is the gate: a disabled feature matches its route and then
 *     refuses, and that case is the most interesting one on the screen.
 *  2. **A referrer, or a known reason.** Probes for `/wp-admin` arrive without
 *     one; a person following a broken link brings the page they came from. So a
 *     bare 404 with no referrer and no reason is dropped — with the deliberate
 *     exception of a switched-off feature, where a bookmark is exactly how
 *     somebody with an old link arrives, and nothing links there any more.
 *  3. **Pages, not assets.** A dead stylesheet or favicon is a broken asset, and
 *     mixing those into a list of missing *pages* makes the list unreadable.
 *
 * `ExceptionEvent` is only listened to, never touched: this must not influence
 * the response, and it runs at a low priority so the error page wins the race.
 */
final class MissingPageSubscriber implements EventSubscriberInterface
{
    /**
     * Request attribute set by FeatureAccessSubscriber when it refuses a page,
     * so the reason survives the exception and reaches this log.
     */
    public const DISABLED_FEATURE_ATTRIBUTE = '_disabled_feature';

    private const ASSET_EXTENSIONS = [
        'css', 'js', 'map', 'ico', 'png', 'jpg', 'jpeg', 'gif', 'svg', 'webp', 'avif',
        'woff', 'woff2', 'ttf', 'eot', 'pdf', 'zip', 'txt', 'xml', 'json',
    ];

    public function __construct(private readonly MissingPageLog $log)
    {
    }

    public function onKernelException(ExceptionEvent $event): void
    {
        if (!$event->isMainRequest() || !$event->getThrowable() instanceof NotFoundHttpException) {
            return;
        }

        $request = $event->getRequest();
        if (!\in_array($request->getMethod(), [Request::METHOD_GET, Request::METHOD_HEAD], true)) {
            return;
        }

        $path = $request->getPathInfo();
        if ($path === '' || str_starts_with($path, '/_')) {
            // `/_profiler`, `/_wdt`, `/_error` — the framework talking to itself.
            return;
        }

        if (\in_array(strtolower(pathinfo($path, \PATHINFO_EXTENSION)), self::ASSET_EXTENSIONS, true)) {
            return;
        }

        $feature = $request->attributes->get(self::DISABLED_FEATURE_ATTRIBUTE);
        $referrer = $this->referrer($request);

        if (\is_string($feature) && $feature !== '') {
            $this->log->record($path, MissingPageLog::REASON_FEATURE, $referrer);

            return;
        }

        // A route matched and the controller still said "not found": a missing
        // record, not a missing page.
        if ((string) $request->attributes->get('_route') !== '') {
            return;
        }

        if ($referrer === null) {
            return;
        }

        $internal = str_starts_with($referrer, $request->getSchemeAndHttpHost() . '/') || $referrer === $request->getSchemeAndHttpHost();
        $this->log->record($path, $internal ? MissingPageLog::REASON_INTERNAL : MissingPageLog::REASON_EXTERNAL, $referrer);
    }

    /** The referring page, or null when there isn't a usable one. */
    private function referrer(Request $request): ?string
    {
        $referrer = trim((string) $request->headers->get('referer'));

        return $referrer === '' ? null : $referrer;
    }

    public static function getSubscribedEvents(): array
    {
        // ⚠️ **Must be above Symfony's ErrorListener (-128), not below it.**
        // `ExceptionEvent` extends `RequestEvent`, and `RequestEvent::setResponse()`
        // calls `stopPropagation()` — so the moment the error listener attaches the
        // error page, every lower-priority listener is skipped. Sitting politely at
        // -256 out of the response's way meant this never ran at all, and the log
        // stayed empty while looking perfectly healthy in `debug:event-dispatcher`.
        // -100 is after RouterListener (-64) and before the response exists.
        return [KernelEvents::EXCEPTION => [['onKernelException', -100]]];
    }
}
