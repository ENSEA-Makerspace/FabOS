<?php

declare(strict_types=1);

namespace App\Twig;

use Symfony\Contracts\Translation\TranslatorInterface;
use Twig\Extension\AbstractExtension;
use Twig\TwigFilter;

/**
 * `|flash_text` — a flash message, translated at RENDER time (S134b).
 *
 * 🔴 **The problem it solves.** 172 `addFlash()` calls across seven controllers held
 * French literals, so an admin running the product in English got a fully translated
 * interface and then "Affiche mise à jour." — the product ships five languages and
 * its confirmations spoke one.
 *
 * ⚠️ **Why at render time rather than at `addFlash()`.** Translating where the message
 * is created needs a `TranslatorInterface` in every one of those 172 actions, and
 * these controllers inject per action, not per constructor. Passing a KEY costs
 * nothing at the call site and moves the one translation to the one place that
 * already knows the request's locale.
 *
 * ⚠️ **Two shapes, because half the messages carry a value:**
 *   `addFlash('success', 'flash.event.created')`
 *   `addFlash('success', ['flash.event.created_named', ['%name%' => $title]])`
 *
 * ⚠️ **An unknown key passes through unchanged**, which is what makes this safe to
 * adopt gradually: Symfony's translator returns the id it was given when no catalogue
 * matches, so a flash that has not been converted yet still reads exactly as before.
 * That property is deliberate — do not "fix" it by throwing on a missing key.
 */
final class FlashExtension extends AbstractExtension
{
    public function __construct(private readonly TranslatorInterface $translator) {}

    public function getFilters(): array
    {
        return [new TwigFilter('flash_text', $this->flashText(...))];
    }

    public function flashText(mixed $message): string
    {
        if (\is_array($message)) {
            // [key, params] — the parameterised shape.
            $key = (string) ($message[0] ?? '');
            $params = \is_array($message[1] ?? null) ? $message[1] : [];

            return $key === '' ? '' : $this->translator->trans($key, $params);
        }

        return $this->translator->trans((string) $message);
    }
}
