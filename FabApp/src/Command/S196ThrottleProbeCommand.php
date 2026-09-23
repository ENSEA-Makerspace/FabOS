<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S196a — la limite aux essais de mot de passe, éprouvée comme un navigateur.
 *
 *   1. une adresse INCONNUE : 5 refus « identifiants », puis le frein ;
 *   2. un MEMBRE : exactement le même comportement, et le même texte — la limite
 *      ne doit pas devenir un oracle d'appartenance ;
 *   3. seuls les ÉCHECS comptent : une connexion réussie ne consomme rien
 *      (et ne remet pas non plus le compteur à zéro — Symfony 8).
 *
 * ⚠️ Les essais viennent de 127.0.0.1 et d'identifiants tirés au hasard (le
 * membre mis à part) : les compteurs laissés dans le cache expirent en cinq
 * minutes et ne touchent aucun visiteur réel, qui arrive par le proxy avec sa
 * propre adresse. Le mot de passe de sonde du membre vit dans une transaction
 * annulée.
 */
#[AsCommand(name: 'app:s196:throttle-probe', description: 'S196a : 5 échecs de mot de passe puis frein, identique pour une adresse inconnue et pour un membre ; une réussite ne consomme rien. Transaction annulée.')]
final class S196ThrottleProbeCommand extends Command
{
    protected function configure(): void
    {
        $this->addOption('measure-global', null, InputOption::VALUE_NONE, 'Mesure seulement : à quel essai le frein PAR IP tombe (identifiants tous différents)');
    }

    private const PASSWORD = 'sonde-S196-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        if ($input->getOption('measure-global')) {
            // Des identifiants tous différents : seul le compteur par IP avance.
            for ($i = 1; $i <= 40; ++$i) {
                $alert = $this->attempts('sonde-s196-g' . $i . '-' . bin2hex(random_bytes(3)) . '@example.invalid', 'mauvais', 1)[0];
                if (str_contains($alert, 'Trop') || str_contains($alert, 'Too')) {
                    $io->writeln('   frein par IP au ' . $i . 'ᵉ essai');

                    return Command::SUCCESS;
                }
            }
            $io->writeln('   pas de frein en 40 essais');

            return Command::SUCCESS;
        }

        $member = $this->users->findOneBy(['statut' => 'actif', 'isVerified' => true]);
        if (!$member instanceof Utilisateur) {
            $io->error('Aucun compte actif.');

            return Command::FAILURE;
        }
        $hashBefore = $member->getPassword();
        $unknown = 'sonde-s196-' . bin2hex(random_bytes(4)) . '@example.invalid';

        $io->section('1. Une adresse INCONNUE');
        $unknownAlerts = $this->attempts($unknown, 'mauvais', 6);
        foreach ($unknownAlerts as $i => $alert) {
            $io->writeln(sprintf('   essai %d : « %s »', $i + 1, $alert));
        }
        $this->check($io, $failures, 'les 5 premiers : le refus habituel, toujours le même', count(array_unique(array_slice($unknownAlerts, 0, 5))) === 1 && $unknownAlerts[0] !== '');
        $this->check($io, $failures, '🔴 le 6ᵉ : freiné, avec un autre message', $unknownAlerts[5] !== $unknownAlerts[0] && $unknownAlerts[5] !== '');
        $this->check($io, $failures, 'et ce message est traduit (pas de clé, pas d\'anglais brut)', !str_contains($unknownAlerts[5], 'Too many') && !str_contains($unknownAlerts[5], '.'.'too_many'));

        $this->db->beginTransaction();
        try {
            // ⚠️ En SQL, pas par l'entité : chaque requête simulée remet les
            // services à zéro, gestionnaire d'entités compris — un compte chargé
            // avant n'est plus suivi, et un `flush()` n'écrirait RIEN.
            $this->db->executeStatement('UPDATE UTILISATEUR SET password = ? WHERE id = ?', [$this->hasher->hashPassword($member, self::PASSWORD), $member->getId()]);

            $io->section('2. Un MEMBRE : exactement pareil');
            $memberAlerts = $this->attempts($member->getEmail(), 'mauvais', 6);
            $this->check($io, $failures, '🔴 mêmes six réponses que pour l\'adresse inconnue', $memberAlerts === $unknownAlerts);

            $io->section('3. Freiné, même le BON mot de passe attend');
            [$blocked] = $this->login($member->getEmail(), self::PASSWORD);
            $this->check($io, $failures, 'le bon mot de passe renvoie à /login tant que le frein dure', str_ends_with((string) $blocked->headers->get('Location'), '/login'));

            $io->section('4. Seuls les ÉCHECS comptent : une réussite ne consomme rien');
            $fresh = 'sonde-s196-' . bin2hex(random_bytes(4));
            // Un autre compte de la même IP : 4 fautes, puis la réussite.
            $freshEmail = $fresh . '@example.invalid';
            $this->db->executeStatement('UPDATE UTILISATEUR SET email = ? WHERE id = ?', [$freshEmail, $member->getId()]);
            $this->attempts($freshEmail, 'mauvais', 4);
            [$ok, $okPage] = $this->login($freshEmail, self::PASSWORD);
            $io->writeln('   affiché : « ' . (preg_match('#<div class="auth-alert auth-alert-error">(.*?)</div>#s', $okPage, $m) ? trim((string) preg_replace('/\s+/', ' ', strip_tags($m[1]))) : '—') . ' »');
            $this->check($io, $failures, 'après 4 fautes, la bonne réponse passe (' . $ok->headers->get('Location') . ')', !str_ends_with((string) $ok->headers->get('Location'), '/login'));
            // ⚠️ Symfony 8 ne REMET PAS le compteur à zéro après une réussite :
            // il ne la décompte simplement pas. 4 échecs + 1 réussite = 4.
            $again = $this->attempts($freshEmail, 'mauvais', 2);
            $io->writeln('   ensuite : « ' . implode(' » | « ', $again) . ' »');
            $this->check($io, $failures, 'la réussite n\'a rien coûté : le 5ᵉ échec passe encore, le 6ᵉ est freiné', $again[0] === $unknownAlerts[0] && $again[1] === $unknownAlerts[5]);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $this->check($io, $failures, 'le mot de passe du membre est intact', $this->users->find($member->getId())?->getPassword() === $hashBefore);

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S196a verte.');

        return Command::SUCCESS;
    }

    /** @return list<string> le message affiché après chaque essai */
    private function attempts(string $email, string $password, int $count): array
    {
        $alerts = [];
        for ($i = 0; $i < $count; ++$i) {
            [, $page] = $this->login($email, $password);
            $alerts[] = preg_match('#<div class="auth-alert auth-alert-error">(.*?)</div>#s', $page, $m) ? trim((string) preg_replace('/\s+/', ' ', strip_tags($m[1]))) : '';
        }

        return $alerts;
    }

    /** @return array{0: Response, 1: string} */
    private function login(string $email, string $password): array
    {
        $session = new Session(new MockArraySessionStorage());
        $form = (string) $this->handle(Request::create('/login'), $session)->getContent();
        $token = preg_match('#name="_csrf_token"\s+value="([^"]+)"#', $form, $m) ? $m[1] : '';
        $response = $this->handle(Request::create('/login', 'POST', ['_username' => $email, '_password' => $password, '_csrf_token' => $token]), $session);

        return [$response, (string) $this->handle(Request::create('/login'), $session)->getContent()];
    }

    private function handle(Request $request, Session $session): Response
    {
        $this->tokens->setToken(null);
        $request->setSession($session);
        if ($session->getId() !== '') {
            $request->cookies->set($session->getName(), $session->getId());
        }

        return $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, true);
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
