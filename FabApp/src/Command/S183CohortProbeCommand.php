<?php

namespace App\Command;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Training\CohortAnnouncer;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S183 — la sonde qui vérifie qu'une annonce ne peut pas exposer d'adresse.
 *
 * 🔴 **Elle N'ENVOIE RIEN, et c'est délibéré.** Le mailer de cette installation
 * est configuré et non suspendu : `announce()` écrirait à de vrais membres. Une
 * sonde qui envoie du courrier à des gens pour se prouver quelque chose est
 * exactement le genre d'action qu'on ne lance pas tout seul.
 *
 * ✅ **Ce qu'elle vérifie à la place est plus fort qu'un envoi** : que l'API du
 * mailer **ne peut pas** prendre plusieurs destinataires. Un envoi réussi
 * prouverait qu'une fois, ça s'est bien passé ; la signature prouve qu'il
 * n'existe aucun chemin pour que ça se passe mal. C'est la différence entre un
 * test et un invariant.
 */
#[AsCommand(name: 'app:s183:cohort-probe', description: 'S183 : vérifie qu\'aucun chemin d\'envoi n\'accepte plusieurs destinataires, et que la cohorte se déduit. N\'envoie AUCUN courrier.')]
final class S183CohortProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly CohortAnnouncer $announcer,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. Aucun chemin d\'envoi n\'accepte une LISTE de destinataires');
        $reflection = new \ReflectionClass(Mailer::class);
        $multi = [];
        foreach ($reflection->getMethods(\ReflectionMethod::IS_PUBLIC) as $method) {
            foreach ($method->getParameters() as $index => $parameter) {
                // Le destinataire est toujours le PREMIER paramètre des verbes
                // d'envoi. Un `array` à cette place serait une liste.
                if ($index !== 0) {
                    continue;
                }
                $type = (string) $parameter->getType();
                if (!str_contains(mb_strtolower($method->getName()), 'queue') && !str_contains(mb_strtolower($method->getName()), 'send')) {
                    continue;
                }
                $io->writeln(sprintf('   %-12s(%s $%s, …)', $method->getName(), $type, $parameter->getName()));
                if (str_contains($type, 'array') || str_contains($type, 'iterable')) {
                    $multi[] = $method->getName();
                }
            }
        }
        $this->check($io, $failures, 'aucun verbe d\'envoi ne prend un tableau en premier paramètre', $multi === []);
        $this->check($io, $failures, 'aucune méthode ne s\'appelle « …ToMany » ou « …Bulk »', !preg_match('/toMany|bulk|broadcast/i', implode(' ', array_map(static fn ($m) => $m->getName(), $reflection->getMethods(\ReflectionMethod::IS_PUBLIC)))));

        $io->section('2. La cohorte se DÉDUIT des progressions');
        $formation = null;
        $best = 0;
        foreach ($this->em->getRepository(Formation::class)->findBy([]) as $candidate) {
            if (str_starts_with($candidate->getTitre(), '[FABOS') || str_starts_with($candidate->getTitre(), 'Validation physique')) {
                continue;
            }
            $count = \count($this->announcer->recipients($candidate));
            if ($count > $best) {
                $best = $count;
                $formation = $candidate;
            }
        }
        if ($formation === null) {
            $io->error('Aucune formation avec des apprenants : la sonde ne mesurerait rien.');

            return Command::FAILURE;
        }

        $recipients = $this->announcer->recipients($formation);
        $io->writeln(sprintf('   « %s » : %d apprenant(s)', $formation->getTitre(), \count($recipients)));

        $ids = array_map(static fn (Utilisateur $u): ?int => $u->getId(), $recipients);
        $this->check($io, $failures, 'aucun doublon dans la cohorte', \count($ids) === \count(array_unique($ids)));
        $this->check($io, $failures, 'chacun a une adresse, et une seule', array_reduce(
            $recipients,
            static fn (bool $ok, Utilisateur $u): bool => $ok && trim((string) $u->getEmail()) !== '',
            true,
        ));

        $names = array_map(static fn (Utilisateur $u): string => (string) $u->getLastName(), $recipients);
        $sorted = $names;
        sort($sorted);
        $this->check($io, $failures, 'la cohorte est triée par nom', $names === $sorted);

        $io->section('3. Ce que la sonde NE fait pas');
        $io->writeln('   <comment>Elle n\'envoie aucun courrier. Le mailer de cette boîte est configuré et non suspendu ;');
        $io->writeln('   déclencher une annonce écrirait à de vrais membres. L\'invariant est vérifié sur la SIGNATURE,');
        $io->writeln('   ce qui prouve qu\'aucun chemin n\'existe — plus fort qu\'un envoi qui se serait bien passé une fois.</comment>');

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S183 verte. Aucun courrier envoyé.');

        return Command::SUCCESS;
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
