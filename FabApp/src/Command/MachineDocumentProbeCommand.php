<?php

namespace App\Command;

use App\Entity\MachineDocument;
use App\Repository\MachineDocumentRepository;
use App\Repository\MachineRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;

/**
 * Sonde jetable : le chemin d'ÉCRITURE des documents machine (S152).
 *
 * 🔴 **Une page qui s'affiche ne prouve pas qu'on peut écrire.** Les deux écrans
 * rendent une liste vide sans rien toucher ; ce qui n'est pas prouvé tant qu'on
 * n'a pas écrit, c'est que le mapping tient à l'aller ET au retour — et c'est
 * exactement là que la stratégie de nommage a mordu la première fois.
 *
 * Elle écrit une ligne, la relit par le dépôt, puis la supprime. ⚠️ Elle ne passe
 * PAS par le téléversement HTTP : ce que la sonde couvre est le mapping et le
 * dépôt, pas la validation de type ni le déplacement du fichier, qui demandent un
 * vrai POST authentifié. C'est dit ici pour qu'on ne lui prête pas plus qu'elle ne
 * mesure.
 *
 * ⚠️ À supprimer une fois la fonctionnalité stabilisée — comme les sondes S147.
 */
#[AsCommand(name: 'app:s152:document-probe', description: 'Écrit, relit et supprime un document machine.')]
final class MachineDocumentProbeCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $entityManager,
        private readonly MachineRepository $machines,
        private readonly MachineDocumentRepository $documents,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $machine = $this->machines->findOneBy([], ['id' => 'ASC']);
        if ($machine === null) {
            $output->writeln('<error>Aucune machine en base.</error>');

            return Command::FAILURE;
        }

        $before = count($this->documents->forMachine($machine));
        $position = $this->documents->nextPosition($machine);

        $doc = (new MachineDocument())
            ->setMachine($machine)
            ->setLabel('SONDE — à supprimer')
            ->setStoredName('probe-s152.pdf')
            ->setOriginalName('sonde.pdf')
            ->setMimeType('application/pdf')
            ->setSizeBytes(123456)
            ->setPosition($position);

        $this->entityManager->persist($doc);
        $this->entityManager->flush();
        $id = $doc->getId();
        $this->entityManager->clear();

        $reread = $this->documents->find($id);
        $ok = $reread !== null
            && $reread->getLabel() === 'SONDE — à supprimer'
            && $reread->getStoredName() === 'probe-s152.pdf'
            && $reread->getOriginalName() === 'sonde.pdf'
            && $reread->getMimeType() === 'application/pdf'
            && $reread->getSizeBytes() === 123456
            && $reread->getKind() === 'pdf'
            && $reread->getMachine()?->getId() === $machine->getId();

        $listed = $reread !== null ? count($this->documents->forMachine($reread->getMachine())) : -1;

        if ($reread !== null) {
            $this->entityManager->remove($reread);
            $this->entityManager->flush();
        }

        $after = count($this->documents->forMachine($this->machines->find($machine->getId())));

        $output->writeln(sprintf('machine #%d — avant %d, pendant %d, après %d', $machine->getId(), $before, $listed, $after));
        $output->writeln(sprintf('aller-retour du mapping : %s', $ok ? 'OK' : 'ÉCHEC'));
        $output->writeln(sprintf('position attribuée : %d', $position));
        $output->writeln(sprintf('nettoyage : %s', $after === $before ? 'OK' : 'ÉCHEC — une ligne est restée'));

        return $ok && $after === $before ? Command::SUCCESS : Command::FAILURE;
    }
}
