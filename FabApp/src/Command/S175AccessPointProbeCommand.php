<?php

namespace App\Command;

use App\Entity\AccessPoint;
use App\Entity\RfidReader;
use App\Repository\RfidReaderRepository;
use App\Repository\VenueRepository;
use App\Rfid\ReaderHealth;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S175 — la sonde qui prouve qu'une PORTE tient, et qu'aucun lecteur n'a bougé.
 *
 * 🔴 **Pourquoi une sonde et pas une relecture du code.** La mesure de sortie de
 * S175 est écrite dans la feuille de route : « un lecteur existant continue de
 * répondre exactement comme avant — comparaison avant/après, annulée sinon ».
 * Lire le code prouve qu'une colonne existe ; seule une écriture suivie d'une
 * relecture prouve qu'une porte se crée, se rattache, et que la règle
 * « exactement une cible » tient vraiment.
 *
 * ⚠️ **Tout se passe dans une transaction ANNULÉE.** La sonde crée un point
 * d'accès et un lecteur, les lit, puis la base ressort inchangée. Une sonde qui
 * laisse des lignes derrière elle est une sonde qui se lance une fois.
 */
#[AsCommand(name: 'app:s175:access-point-probe', description: 'S175 : prouve qu\'une porte se crée, qu\'un boîtier s\'y rattache, et que les lecteurs existants n\'ont pas bougé.')]
final class S175AccessPointProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly RfidReaderRepository $readers,
        private readonly VenueRepository $venues,
        private readonly ReaderHealth $health,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        /*
         * ⚠️ **L'empreinte AVANT est prise hors transaction et sur des valeurs
         * SCALAIRES.** Comparer des objets hydratés comparerait des identités
         * d'objets, pas des données — et l'unité de travail de Doctrine les
         * rendrait identiques quoi qu'il arrive.
         */
        $before = $this->fingerprint();
        $io->section('1. Les lecteurs existants, avant');
        foreach ($before as $line) {
            $io->writeln('   ' . $line);
        }

        $this->em->beginTransaction();

        try {
            $venue = $this->venues->findOneBy([]);
            if ($venue === null) {
                $io->error('Aucun lieu en base : la sonde ne peut rien rattacher.');
                $this->em->rollback();

                return Command::FAILURE;
            }

            $io->section('2. Une porte se crée et se relit');
            $point = (new AccessPoint())
                ->setNom('SONDE — porte S175')
                ->setKind('door')
                ->setVenue($venue);
            $this->em->persist($point);
            $this->em->flush();

            $reread = $this->em->getRepository(AccessPoint::class)->find($point->getId());
            $this->check($io, $failures, 'la porte se relit', $reread !== null && $reread->getNom() === 'SONDE — porte S175');
            $this->check($io, $failures, 'sa nature est fermée à la liste', $reread?->getKind() === 'door');
            $this->check($io, $failures, 'une nature inconnue retombe sur « door »', (new AccessPoint())->setKind('n_importe_quoi')->getKind() === 'door');

            $io->section('3. Un boîtier se rattache à la porte — sans machine');
            $reader = (new RfidReader())
                ->setName('SONDE — boîtier de porte')
                ->setReaderToken('sonde-s175-' . bin2hex(random_bytes(4)))
                ->setIsActive(true)
                ->setAccessPoint($point);
            $this->em->persist($reader);
            $this->em->flush();

            $this->check($io, $failures, 'le boîtier vit sans machine', $reader->getMachine() === null);
            $this->check($io, $failures, 'sa cible est le point d\'accès', $reader->targetKind() === 'access_point');
            $this->check($io, $failures, 'il se nomme par sa cible', $reader->targetLabel() === 'SONDE — porte S175');
            $this->check($io, $failures, 'la cible est valide', $reader->hasValidTarget());

            /*
             * 🔴 **Le cas que le booléen `isActive` ne pouvait pas dire.** Un
             * boîtier de porte JAMAIS VU doit être « jamais connecté », pas
             * « non associé » : avant S175, `ReaderHealth` cherchait une
             * machine, n'en trouvait pas, et aurait déclaré non associé chaque
             * lecteur d'entrée du labo — en jaune, en permanence.
             */
            $io->section('4. La santé sait lire une porte');
            $state = $this->health->of($reader);
            $this->check($io, $failures, 'jamais vu ⇒ « jamais connecté », pas « non associé »', $state['state'] === ReaderHealth::NEVER_SEEN);

            $orphan = (new RfidReader())->setName('SONDE — sans cible')->setIsActive(true);
            $this->check($io, $failures, 'aucune cible ⇒ « non associé »', $this->health->of($orphan)['state'] === ReaderHealth::UNPAIRED);
            $this->check($io, $failures, 'aucune cible ⇒ cible invalide', !$orphan->hasValidTarget());

            $io->section('5. Les deux cibles s\'excluent, dans les deux sens');
            $machine = $this->em->getRepository(\App\Entity\Machine::class)->findOneBy([]);
            if ($machine === null) {
                $io->warning('Aucune machine en base : l\'exclusion mutuelle n\'est pas mesurée ici.');
            } else {
                $reader->setMachine($machine);
                $this->check($io, $failures, 'poser une machine efface la porte', $reader->getAccessPoint() === null && $reader->targetKind() === 'machine');
                $reader->setAccessPoint($point);
                $this->check($io, $failures, 'poser une porte efface la machine', $reader->getMachine() === null && $reader->targetKind() === 'access_point');
                $this->check($io, $failures, 'jamais les deux à la fois', $reader->hasValidTarget());
            }
        } finally {
            // ⚠️ `finally` et pas une ligne à la fin : une assertion qui jette
            // laisserait sinon la transaction ouverte ET les lignes de sonde en
            // base, c'est-à-dire exactement ce que la sonde promet d'éviter.
            $this->em->rollback();
            $this->em->clear();
        }

        $io->section('6. Les lecteurs existants, après — et rien n\'a bougé');
        $after = $this->fingerprint();
        foreach ($after as $line) {
            $io->writeln('   ' . $line);
        }
        $this->check($io, $failures, 'empreinte identique avant/après', $before === $after);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S175 verte. La base est rendue telle qu\'elle était.');

        return Command::SUCCESS;
    }

    /** @return string[] */
    private function fingerprint(): array
    {
        $lines = [];
        foreach ($this->readers->findForAdmin() as $reader) {
            $lines[] = sprintf(
                '#%d %s · jeton=%s · cible=%s(%s) · actif=%s · vu=%s',
                (int) $reader->getId(),
                $reader->getName(),
                $reader->getReaderToken(),
                $reader->targetKind() ?? 'aucune',
                $reader->targetLabel() ?? '-',
                $reader->isActive() ? 'oui' : 'non',
                $reader->getLastSeenAt()?->format('Y-m-d H:i:s') ?? 'jamais',
            );
        }

        return $lines;
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
