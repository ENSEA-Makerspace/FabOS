<?php

namespace App\Command;

use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * Une sonde qui se comporte comme un navigateur : se connecter, ouvrir une
 * page, poster un formulaire — par `$kernel->handle()`, dans la transaction
 * annulée de la commande. Écrit une fois ici (S202) ; chaque piège ci-dessous a
 * déjà fait mentir une sonde (voir la mémoire « kernel probe traps »).
 *
 * La classe hôte fournit `$this->kernel`, `$this->tokens` et, pour
 * `inAnyLocale()`, `$this->translator`.
 *
 * @property KernelInterface       $kernel
 * @property TokenStorageInterface $tokens
 * @property TranslatorInterface   $translator
 */
trait ProbeBrowser
{
    private string $lastLogin = '';

    private function login(string $email, string $password): Session
    {
        $session = new Session(new MockArraySessionStorage());
        $form = (string) $this->handle(Request::create('/login'), $session)->getContent();
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $response = $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => $password, '_csrf_token' => $token]), $session);
        $this->lastLogin = $response->getStatusCode() . ' → ' . $response->headers->get('Location') . ($token === '' ? ' (sans jeton CSRF)' : '');

        return $session;
    }

    private function status(string $path, Session $session): int
    {
        return $this->handle(Request::create($path), $session)->getStatusCode();
    }

    private function page(string $path, Session $session): string
    {
        return (string) $this->handle(Request::create($path), $session)->getContent();
    }

    /**
     * ⚠️ Le conteneur survit d'une requête à l'autre dans une commande : sans
     * remise à zéro, le jeton de la session PRÉCÉDENTE resterait dans le
     * stockage et la requête suivante serait authentifiée par erreur.
     */
    private function handle(Request $request, Session $session): Response
    {
        $this->tokens->setToken(null);
        $request->setSession($session);
        // ⚠️ Le pare-feu ne relit le jeton en session QUE si la requête porte le
        // cookie de session (`hasPreviousSession()`) : sans lui, chaque requête
        // repart anonyme et une « session ouverte » n'est jamais mesurée.
        if ($session->getId() !== '') {
            $request->cookies->set($session->getName(), $session->getId());
        }

        return $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true);
    }

    /** @param array<string, string> $fields ⚠️ `Origin` posé : les formulaires sans état le vérifient. */
    private function post(string $path, array $fields, Session $session): Response
    {
        $request = Request::create($path, 'POST', $fields);
        $request->headers->set('Origin', $request->getSchemeAndHttpHost());

        return $this->handle($request, $session);
    }

    /** Le `_token` du formulaire dont l'action est `$action`, ou '' s'il n'y en a pas. */
    private function formToken(string $html, string $action): string
    {
        return preg_match('#action="' . preg_quote($action, '#') . '".*?name="_token" value="([^"]+)"#s', $html, $m) ? $m[1] : '';
    }

    /**
     * La langue de la page suit le compte connecté : on accepte les cinq.
     *
     * @param array<string, mixed> $params
     */
    private function inAnyLocale(string $html, string $key, array $params): bool
    {
        foreach (['fr', 'en', 'de', 'es', 'it'] as $locale) {
            $text = $this->translator->trans($key, $params, null, $locale);
            if ($text !== $key && str_contains($html, htmlspecialchars($text, ENT_QUOTES))) {
                return true;
            }
        }

        return false;
    }

    /** @param list<string> $failures */
    private function check(SymfonyStyle $io, array &$failures, string $what, bool $ok): void
    {
        $io->writeln(($ok ? '   <info>✓</info> ' : '   <error>✗</error> ') . $what);
        if (!$ok) {
            $failures[] = $what;
        }
    }
}
