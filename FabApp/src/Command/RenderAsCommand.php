<?php

namespace App\Command;

use App\Security\ConsoleRenderAuthenticator;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputArgument;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;

/**
 * Renders any page of this site from the shell, signed in.
 *
 * The problem it solves: the operator's account is the only real login, the
 * agent has no password, and the firewall 302s an anonymous `curl` to `/login`
 * before the controller runs — so **every admin and staff screen was invisible**
 * to anything that could not click. The previous answer was an HTTP-reachable
 * password-less sign-in (`LOCAL_ADMIN_BYPASS`), which is a hole on a public box.
 * This runs the same request through the same kernel, firewall and templates,
 * but it starts from a shell rather than from the network — see
 * ConsoleRenderAuthenticator for why that distinction is structural.
 *
 * What it is *not*: a test. It proves a page answers and what it contains, which
 * is exactly the class of bug that hides behind a login — a template that throws,
 * a route that 404s, a panel that renders empty. It says nothing about how the
 * page looks, and it is not a substitute for a real session in anything
 * auth-shaped: it authenticates by construction, so it can never show you an
 * access rule that is too permissive.
 *
 *   php bin/console app:render /admin/features
 *   php bin/console app:render /admin/setup --grep='setup-pill'
 *   php bin/console app:render /profil --as=someone@example.org --save=/tmp/p.html
 */
#[AsCommand(name: 'app:render', description: 'Render a page from the shell as a signed-in user.')]
final class RenderAsCommand extends Command
{
    public function __construct(
        private readonly ConsoleRenderAuthenticator $authenticator,
        private readonly HttpKernelInterface $kernel,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this
            ->addArgument('path', InputArgument::REQUIRED, 'Path to render, e.g. /admin/features')
            ->addOption('as', null, InputOption::VALUE_REQUIRED, 'Account to render as (default: the first admin)')
            ->addOption('grep', null, InputOption::VALUE_REQUIRED, 'Print only body lines matching this substring')
            ->addOption('save', null, InputOption::VALUE_REQUIRED, 'Write the full body to this file')
            ->addOption('anonymous', null, InputOption::VALUE_NONE, 'Do not sign in — see what a visitor sees');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $path = (string) $input->getArgument('path');

        $as = 'anonymous';
        if (!$input->getOption('anonymous')) {
            try {
                $as = $this->authenticator->renderAs($input->getOption('as'));
            } catch (\RuntimeException $e) {
                $io->error($e->getMessage());

                return Command::FAILURE;
            }
        }

        $request = Request::create($path);
        // Templates reach for flash messages, and a request built by hand has no
        // session to hold them — `app.flashes` would throw before the page ever
        // rendered. An in-memory session keeps that self-contained: nothing is
        // written to disk and nothing survives the command.
        $request->setSession(new Session(new MockArraySessionStorage()));

        $response = $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, false);
        $body = (string) $response->getContent();

        $io->definitionList(
            ['path' => $path],
            ['as' => $as],
            ['status' => $response->getStatusCode()],
            ['redirect' => $response->headers->get('location') ?? '—'],
            ['bytes' => \strlen($body)],
        );

        if ($save = $input->getOption('save')) {
            file_put_contents($save, $body);
            $io->text(sprintf('Body written to %s', $save));
        }

        if ($grep = $input->getOption('grep')) {
            $hits = array_filter(explode("\n", $body), static fn (string $line): bool => str_contains($line, (string) $grep));
            $io->section(sprintf('%d line(s) containing "%s"', \count($hits), $grep));
            foreach (\array_slice($hits, 0, 40) as $line) {
                $io->text(trim($line));
            }
        }

        // A redirect to /login means the sign-in did not take, which is a failure
        // of this command rather than a finding about the page.
        if ($response->getStatusCode() >= 500) {
            return Command::FAILURE;
        }

        return Command::SUCCESS;
    }
}
