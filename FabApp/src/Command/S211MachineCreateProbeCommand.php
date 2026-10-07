<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Page\MachineCreationHints;
use App\Repository\UtilisateurRepository;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S211 (0.7.0) — « Créer une machine » propose au lieu de demander.
 *
 *   1. l'écran : les modèles possédés, les tuiles (`suggest`), pas de statut ni de
 *      limite ni de popularité, un identifiant de boîtier déjà généré ;
 *   2. `?copier=<id>` préremplit depuis la fiche (nom suivant, catégorie, badges) ;
 *   3. créer : la machine naît « disponible », avec ses badges et ses étiquettes,
 *      et on arrive sur SA fiche ; « puis une autre identique » rouvre le
 *      formulaire prérempli ;
 *   4. l'édition garde statut, limite et popularité, et les enregistre.
 *
 * ✅ Transaction annulée ; le nombre de machines est comparé avant/après.
 */
#[AsCommand(name: 'app:s211:machine-create-probe', description: 'S211 : créer une machine (modèles connus, copie, tuiles, création « disponible »). Transaction annulée.')]
final class S211MachineCreateProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S211-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly MachineCreationHints $hints,
        private readonly UserPasswordHasherInterface $hasher,
        private readonly TokenStorageInterface $tokens,
        private readonly TranslatorInterface $translator,
    ) {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $count = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE');
        $before = $count();

        $admin = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin = $candidate;
                break;
            }
        }
        $source = $this->db->fetchAssociative("SELECT m.id, m.categoryLabel FROM MACHINE m JOIN MACHINE_BADGE mb ON mb.machineId = m.id WHERE m.archivedAt IS NULL AND m.model <> '' AND m.categoryLabel <> '' ORDER BY m.id DESC LIMIT 1");
        if (!$admin instanceof Utilisateur || $source === false) {
            $io->error('Il faut un administrateur et une machine avec modèle, catégorie et badge.');

            return Command::FAILURE;
        }

        $this->db->beginTransaction();
        try {
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$admin->getId()]);
            $session = $this->login($admin->getEmail(), self::PASSWORD);
            $copy = $this->hints->copyOf((int) $source['id']);

            $io->section('1. L’écran de création');
            $html = $this->page('/admin/machines/new', $session);
            $this->check($io, $failures, 'les modèles possédés sont proposés (connexion ' . $this->lastLogin . ')', str_contains($html, '?copier='));
            $this->check($io, $failures, 'catégorie et localisation portent le contrôleur `suggest`', substr_count($html, 'data-controller="suggest"') >= 2 && str_contains($html, 'data-suggest-checks-name-value="requiredBadges[]"'));
            $this->check($io, $failures, '🔴 ni statut, ni limite, ni popularité à la création', !str_contains($html, 'machine_admin[statut]') && !str_contains($html, 'machine_admin[limiteReservations]') && !str_contains($html, 'machine_admin[popularite]'));
            $this->check($io, $failures, 'l’identifiant du boîtier est déjà généré', preg_match('/name="machine_admin\[machineToken\]"[^>]*value="[0-9a-f]{16}"/', $html) === 1);
            $this->check($io, $failures, 'les badges sont des tuiles à cocher', preg_match('/<label class="ml-tile"[^>]*>\s*<input type="checkbox" name="requiredBadges\[\]"/', $html) === 1);

            $io->section('2. Repartir d’une machine existante');
            $html = $this->page('/admin/machines/new?copier=' . $source['id'], $session);
            $this->check($io, $failures, 'le nom suivant est proposé (« ' . $copy['nom'] . ' »)', str_contains($html, 'value="' . htmlspecialchars($copy['nom'], ENT_QUOTES) . '"'));
            $this->check($io, $failures, 'la catégorie est reprise', preg_match('/name="machine_admin\[categorie\]"[^>]*value="' . preg_quote(htmlspecialchars((string) $source['categoryLabel'], ENT_QUOTES), '/') . '"/', $html) === 1);
            $this->check($io, $failures, 'ses badges sont cochés', $copy['badges'] !== [] && preg_match('/name="requiredBadges\[\]" value="' . $copy['badges'][0] . '" checked/', $html) === 1);
            $token = preg_match('/name="machine_admin\[machineToken\]"[^>]*value="([0-9a-f]{16})"/', $html, $m) ? $m[1] : 'sonde-s211';

            $io->section('3. Créer');
            $fields = ['machine_admin' => [
                '_token' => 'csrf-token', 'nom' => 'Sonde S211', 'categorie' => (string) $source['categoryLabel'], 'venue' => (string) $copy['venueId'],
                'localisation' => 'Établi sonde', 'niveau' => '2', 'granularite' => '30', 'machineToken' => $token,
                'materiaux' => "PLA\nSonde S211", 'caracteristiques' => '', 'saveAgain' => '',
            ], 'requiredBadges' => [(string) $copy['badges'][0]]];
            $response = $this->post('/admin/machines/new', $fields, $session);
            $row = $this->db->fetchAssociative("SELECT * FROM MACHINE WHERE nom = 'Sonde S211'");
            $this->check($io, $failures, '🔴 la machine est créée « disponible », limite 0 (réponse ' . $response->getStatusCode() . ')', $row !== false && $row['statut'] === 'disponible' && (int) $row['limiteReservations'] === 0);
            $this->check($io, $failures, 'niveau, créneau, étiquettes et badge sont enregistrés', $row !== false && $row['levelSlug'] === 'niveau-2' && $row['granularite'] === '30'
                && json_decode((string) $row['materials'], true) === ['PLA', 'Sonde S211']
                && (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE_BADGE WHERE machineId = ?', [$row['id']]) === 1);
            $this->check($io, $failures, '« puis une autre identique » rouvre le formulaire prérempli', $row !== false && $response->headers->get('Location') === '/admin/machines/new?copier=' . $row['id']);
            unset($fields['machine_admin']['saveAgain']);
            $fields['machine_admin'] = ['nom' => 'Sonde S211 bis', 'machineToken' => 'sonde-s211-bis', 'save' => ''] + $fields['machine_admin'];
            $response = $this->post('/admin/machines/new', $fields, $session);
            $bisId = (int) $this->db->fetchOne("SELECT id FROM MACHINE WHERE nom = 'Sonde S211 bis'");
            $this->check($io, $failures, '« Créer la machine » mène à SA fiche', $bisId > 0 && $response->headers->get('Location') === '/admin/machines/' . $bisId . '/edit');
            $response = $this->post('/admin/machines/new', ['machine_admin' => ['nom' => 'Sonde S211 ter', 'machineToken' => $token] + $fields['machine_admin']], $session);
            $this->check($io, $failures, 'un identifiant de boîtier déjà pris est refusé (' . $response->getStatusCode() . ')', $response->getStatusCode() === 422);

            $io->section('4. L’édition garde l’état');
            $html = $this->page('/admin/machines/' . $bisId . '/edit', $session);
            $this->check($io, $failures, 'statut, limite et popularité y sont', str_contains($html, 'machine_admin[statut]') && str_contains($html, 'machine_admin[limiteReservations]') && str_contains($html, 'machine_admin[popularite]'));
            $this->check($io, $failures, 'pas de « modèles possédés » ni de second bouton à l’édition', !str_contains($html, '?copier=') && !str_contains($html, 'machine_admin[saveAgain]'));
            $response = $this->post('/admin/machines/' . $bisId . '/edit', ['machine_admin' => [
                '_token' => 'csrf-token', 'nom' => 'Sonde S211 bis', 'categorie' => (string) $source['categoryLabel'], 'venue' => (string) $copy['venueId'],
                'statut' => 'maintenance', 'limiteReservations' => '3', 'popularite' => '5', 'materiaux' => 'PETG', 'save' => '',
            ]], $session);
            $row = $this->db->fetchAssociative('SELECT statut, limiteReservations, popularity, materials FROM MACHINE WHERE id = ?', [$bisId]);
            $this->check($io, $failures, 'ils s’enregistrent (réponse ' . $response->getStatusCode() . ')', $row !== false && $row['statut'] === 'maintenance' && (int) $row['limiteReservations'] === 3 && (int) $row['popularity'] === 5 && json_decode((string) $row['materials'], true) === ['PETG']);

            $io->section('5. Les mêmes tuiles ailleurs (0.7.1)');
            $html = $this->page('/admin/materials/new', $session);
            $this->check($io, $failures, 'matériau : les machines sont des tuiles à cocher, repliées au-delà de douze', str_contains($html, 'name="material_admin[machines][]"') && str_contains($html, 'class="ml-tile"')
                && ($before <= 12 || str_contains($html, 'data-controller="admin-list-filter"')));
            $response = $this->post('/admin/materials/new', ['material_admin' => ['_token' => 'csrf-token', 'name' => 'Sonde S211 matériau', 'category' => 'Sonde', 'machines' => [(string) $bisId], 'save' => '']], $session);
            $materialId = (int) $this->db->fetchOne("SELECT id FROM MATERIAL WHERE name = 'Sonde S211 matériau'");
            $this->check($io, $failures, 'matériau : une machine cochée en tuile est enregistrée (réponse ' . $response->getStatusCode() . ')', $materialId > 0 && (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE_MATERIAL WHERE materialId = ?', [$materialId]) === 1);
            foreach (['/admin/loanable-items/new' => 'objet en prêt', '/admin/places/new' => 'espace'] as $path => $what) {
                $html = $this->page($path, $session);
                $this->check($io, $failures, $what . ' : l’écran se rend, avec ses propositions s’il y a des valeurs connues', str_contains($html, 'class="admin-edit-form"'));
            }
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
        }

        $this->check($io, $failures, 'la base est intacte (' . $before . ' machines)', $count() === $before);
        if ($failures !== []) {
            $io->error(\count($failures) . ' échec(s).');

            return Command::FAILURE;
        }
        $io->success('S211 : créer une machine — tout passe.');

        return Command::SUCCESS;
    }
}
