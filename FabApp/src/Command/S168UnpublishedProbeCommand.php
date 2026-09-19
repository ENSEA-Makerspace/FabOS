<?php

namespace App\Command;

use App\Entity\LabPage;
use Doctrine\DBAL\Connection;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\HttpKernelInterface;
use Symfony\Component\HttpKernel\KernelInterface;

/**
 * S168 — « dépublier » dépublie-t-il vraiment ?
 *
 * 🔴 **Le défaut mesuré, sur trois pages en production.** Archiver une page du
 * lab la retirait du MENU — `findTopLevelWithChildrenLive()` filtre — et la
 * laissait entièrement lisible à son URL. Un signet, un lien dans un mail, un
 * partage : le contenu restait servi à tout le monde. « Dépubliée » n'était vrai
 * que dans une requête sur quatre : deux gabarits listaient encore les
 * sous-pages archivées, et la route de détail les rendait.
 *
 * ⚠️ **La sonde fait la moitié VISITEUR ; `app:render` fait la moitié
 * ADMINISTRATEUR.** Le noyau appelé ici n'a pas de session, ce qui est exactement
 * la situation d'un visiteur — et c'est la moitié dont l'échec est une fuite.
 *
 * ⚠️ **La page jetable naît ARCHIVÉE**, donc elle n'apparaît nulle part entre sa
 * création et sa suppression, et la sonde vérifie le compte de la table des deux
 * côtés.
 */
#[AsCommand(name: 'app:s168:unpublished-probe', description: 'S168 : prouve qu\'une page dépubliée renvoie un visiteur à l\'accueil, avec trace et sans boucle, et que les sous-pages archivées disparaissent des listes.')]
final class S168UnpublishedProbeCommand extends Command
{
    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];

        $io->section('1. `getLiveChildren()` — en mémoire, sans base');
        // 🔴 UNE définition que tout le monde traverse. Le dépôt ne sert que les
        // appelants qui pensent à lui ; un gabarit écrit `page.liveChildren` sans
        // rien savoir de la règle.
        $parent = new LabPage();
        $live = (new LabPage())->setTitre('vivante');
        $gone = (new LabPage())->setTitre('archivée');
        $gone->archive();
        // ⚠️ Pas d'`addChild()` sur cette entité : on passe par la collection,
        // qui est ce que Doctrine remplit de toute façon.
        $parent->getChildren()->add($live);
        $parent->getChildren()->add($gone);

        $this->check($io, $failures, 'les deux enfants existent', $parent->getChildren()->count() === 2);
        $this->check($io, $failures, '🔴 un seul est VIVANT', $parent->getLiveChildren()->count() === 1);
        $this->check($io, $failures, 'et c\'est le bon', $parent->getLiveChildren()->first()?->getTitre() === 'vivante');
        // ⚠️ `getChildren()` garde les deux, délibérément : la liste
        // d'administration montre les archivées, c'est de là qu'on les restaure.
        $this->check($io, $failures, '⚠️ `getChildren()` les garde TOUS — l\'admin en a besoin', $parent->getChildren()->count() === 2);

        $io->section('2. 🔴 Un visiteur n\'ouvre PAS une page dépubliée');
        $before = (int) $this->db->fetchOne('SELECT COUNT(*) FROM LAB_PAGE');
        $id = $this->createArchivedPage();

        try {
            $request = Request::create('/lab/' . $id);
            $request->setSession(new Session(new MockArraySessionStorage()));
            $response = $this->kernel->handle($request, HttpKernelInterface::MAIN_REQUEST, false);
            $status = $response->getStatusCode();
            $target = (string) $response->headers->get('Location');

            $io->writeln('   statut ' . $status . '  →  ' . ($target ?: '(aucune redirection)'));
            $this->check($io, $failures, '🔴 il est REDIRIGÉ, pas servi', $status === 302);
            // ⚠️ Ni 404 ni page blanche : le plan demande explicitement « rétablit
            // l'accueil », parce qu'un 404 sur un lien qui marchait hier ressemble
            // à une panne du site.
            $this->check($io, $failures, 'vers l\'ACCUEIL, pas une erreur', str_ends_with($target, '/'));
            // 🔴 La cible est écrite en dur : viser le référent est exactement la
            // façon dont on fabrique une boucle.
            $home = $this->kernel->handle(Request::create($target ?: '/'), HttpKernelInterface::MAIN_REQUEST, false);
            $this->check($io, $failures, '🔴 et l\'accueil RÉPOND (aucune boucle)', $home->getStatusCode() === 200);

            $io->section('3. La trace');
            // Sans message, la page d'accueil qui apparaît à la place ressemble à
            // un clic raté.
            $flashes = $request->getSession()->getFlashBag()->peekAll();
            $all = [];
            foreach ($flashes as $type => $messages) {
                foreach ($messages as $m) {
                    $all[] = $type . ':' . (is_string($m) ? $m : json_encode($m));
                }
            }
            $io->writeln('   ' . (implode(' | ', $all) ?: '(aucun)'));
            $this->check($io, $failures, 'un message explique le renvoi', $all !== []);
        } finally {
            $this->db->executeStatement('DELETE FROM LAB_PAGE WHERE id = ?', [$id]);
        }

        $this->check($io, $failures, 'la page jetable est supprimée', (int) $this->db->fetchOne('SELECT COUNT(*) FROM LAB_PAGE') === $before);

        $io->section('La moitié ADMINISTRATEUR se mesure ailleurs');
        $io->writeln('   La console n\'a pas de session. `app:render` en a une :');
        $io->writeln('     app:render /lab/<id d\'une page archivée>  →  200 + bandeau « DÉPUBLIÉE »');

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }

        $io->success('Sonde S168 verte. Table des pages rendue à son compte de départ.');

        return Command::SUCCESS;
    }

    /** ⚠️ Archivée dès l'insertion : invisible partout entre sa naissance et sa mort. */
    private function createArchivedPage(): int
    {
        $this->db->executeStatement(
            "INSERT INTO LAB_PAGE (titre, contenu, position, createdAt, archivedAt)
             VALUES ('[SONDE S168]', 'sonde', 0, NOW(), NOW())",
        );

        return (int) $this->db->lastInsertId();
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
