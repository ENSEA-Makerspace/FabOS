<?php

namespace App\Command;

use App\Entity\Machine;
use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use App\Service\MachineReports;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S205 — signaler une panne, sans compte, depuis le QR de la machine.
 *
 * Prouve, par `$kernel->handle()` et dans une transaction ANNULÉE, chaque ligne
 * de « Ce que l'opérateur vérifie » :
 *   1. un visiteur NON connecté ouvre la page de la machine (champ photo avec
 *      `capture`), la défense tient (sans jeton, pot de miel, texte trop court,
 *      fichier qui n'est pas une image : rien n'est écrit) ;
 *   2. il envoie une photo : une ligne « open », le fichier est rangé, l'écran
 *      « merci » s'affiche et l'ÉTAT DE LA MACHINE n'a pas bougé ;
 *   3. la limite de débit coupe au-delà de 3 envois en 10 minutes ;
 *   4. l'équipe le voit (liste, accueil admin, carte de la fiche machine), le
 *      résout avec une note, le rouvre, le supprime (la photo disparaît) ;
 *   5. le QR s'imprime ; fonction éteinte : tout répond 404 et rien ne s'affiche.
 *
 * ✅ Les comptes de tables sont comparés avant/après et les fichiers de la sonde
 * sont effacés du disque.
 */
#[AsCommand(name: 'app:s205:machine-report-probe', description: 'S205 : signaler une panne par QR, sans compte (anti-abus, photo, résolution, accueil admin, QR). Transaction annulée.')]
final class S205MachineReportProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S205-motdepasse';
    private const TEXT = 'Sonde S205 : la buse chauffe mais le fil ne sort plus';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly MachineRepository $machines,
        private readonly MachineReports $reports,
        private readonly SiteFeatureService $features,
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
        if (!$this->reports->isReady()) {
            $io->error('La migration S205 (MACHINE_REPORT) n’est pas passée.');

            return Command::FAILURE;
        }
        $admin = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin = $candidate;
                break;
            }
        }
        $machine = $this->machines->findLive()[0] ?? null;
        if (!$admin instanceof Utilisateur || !$machine instanceof Machine) {
            $io->error('Il faut un administrateur actif et une machine.');

            return Command::FAILURE;
        }

        $tables = ['MACHINE_REPORT', 'MACHINE', 'UTILISATEUR', 'SITE_MODULE'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();
        $directory = \dirname(__DIR__, 2) . '/public/uploads/machine-reports';
        $filesBefore = is_dir($directory) ? (scandir($directory) ?: []) : [];
        $created = [];
        $png = tempnam(sys_get_temp_dir(), 's205') ?: '';

        $this->db->beginTransaction();
        try {
            $this->features->setEnabled('machines', true);
            $this->features->setEnabled('machine_reports', true);
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$admin->getId()]);
            [$id, $name, $adminId, $adminEmail, $statusBefore] = [(int) $machine->getId(), $machine->getNom(), (int) $admin->getId(), $admin->getEmail(), $machine->getStatusKey()];
            $rows = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM MACHINE_REPORT WHERE machineId = ?', [$id]);
            $page = '/signaler/' . $id;
            $guest = fn (): Session => new Session(new MockArraySessionStorage());

            $io->section('1. Un visiteur sans compte');
            $anon = $guest();
            $html = $this->page($page, $anon);
            $this->check($io, $failures, 'la page de la machine s’ouvre sans connexion et la nomme', $this->status($page, $anon) === 200 && str_contains($html, htmlspecialchars($name, ENT_QUOTES)));
            $this->check($io, $failures, '🔴 le champ photo ouvre l’appareil du téléphone (accept image + capture)', str_contains($html, 'accept="image/*"') && str_contains($html, 'capture="environment"'));
            $this->check($io, $failures, 'un bouton d’envoi, et le pot de miel hors écran', str_contains($html, 'type="submit"') && str_contains($html, 'name="website"'));
            $token = $this->formToken($html, $page);
            $this->check($io, $failures, 'le formulaire porte un jeton CSRF', $token !== '');
            $this->check($io, $failures, '/signaler répond (liste, ou la machine si elle est seule)', \in_array($this->status('/signaler', $anon), [200, 302], true));
            $this->check($io, $failures, 'une machine inconnue : 404', $this->status('/signaler/999999999', $anon) === 404);

            $io->section('2. L’anti-abus');
            $this->post($page, ['description' => self::TEXT], $anon);
            $this->check($io, $failures, 'sans jeton : refusé, rien n’est écrit', $rows() === 0);
            $r = $this->post($page, ['_token' => $token, 'description' => self::TEXT, 'website' => 'http://spam.example'], $anon);
            $this->check($io, $failures, 'pot de miel rempli : même « merci », mais rien n’est écrit', $r->getStatusCode() === 302 && $rows() === 0);
            $r = $this->post($page, ['_token' => $token, 'description' => 'ab'], $anon);
            $this->check($io, $failures, 'texte trop court : 422, rien n’est écrit', $r->getStatusCode() === 422 && $rows() === 0);
            file_put_contents($png, 'ceci n’est pas une image');
            $r = $this->submit($page, ['_token' => $token, 'description' => self::TEXT], new UploadedFile($png, 'faux.jpg', 'image/jpeg', null, true), $anon);
            $this->check($io, $failures, '🔴 un fichier qui n’est pas une image : 422, rien n’est écrit', $r->getStatusCode() === 422 && $rows() === 0);
            $r = $this->submit($page, ['_token' => $token, 'description' => self::TEXT], new UploadedFile($png, 'faux.php', 'application/x-php', null, true), $anon);
            $this->check($io, $failures, 'un script : 422, rien n’est écrit', $r->getStatusCode() === 422 && $rows() === 0);

            $io->section('3. Envoyer une photo');
            $gd = imagecreatetruecolor(40, 30);
            imagepng($gd, $png);
            $r = $this->submit($page, ['_token' => $token, 'description' => self::TEXT, 'contact' => 'visiteur@example.org'], new UploadedFile($png, 'panne.png', 'image/png', null, true), $anon);
            $this->check($io, $failures, '🔴 envoyé : redirigé vers l’écran « merci »', $r->getStatusCode() === 302 && str_ends_with((string) $r->headers->get('Location'), '/signaler/merci'));
            $report = $this->db->fetchAssociative('SELECT * FROM MACHINE_REPORT WHERE machineId = ?', [$id]);
            $this->check($io, $failures, 'une ligne « open », description et contact gardés', $report !== false && $report['status'] === 'open' && $report['description'] === self::TEXT && $report['contact'] === 'visiteur@example.org');
            $photo = (string) ($report['photo'] ?? '');
            $this->check($io, $failures, '🔴 la photo est rangée (nom aléatoire, dossier des signalements)', $photo !== '' && preg_match('/^report-[0-9a-f]{16}\.(jpg|png|webp)$/', $photo) === 1 && is_file($directory . '/' . $photo));
            $created[] = $photo;
            $this->check($io, $failures, 'l’adresse n’est pas stockée en clair (empreinte de 64 caractères)', $report !== false && strlen((string) $report['ipHash']) === 64 && !str_contains((string) $report['ipHash'], '127.0.0.1'));
            $thanks = $this->page('/signaler/merci', $anon);
            $this->check($io, $failures, '🔴 « Merci, l’équipe est prévenue », sans retour au formulaire', $this->inAnyLocale($thanks, 'machine_report.thanks_title', []) && !str_contains($thanks, 'name="description"'));
            $this->entityManager->clear();
            $this->check($io, $failures, '🔴 l’ÉTAT de la machine n’a pas bougé', $this->machines->find($id)?->getStatusKey() === $statusBefore);

            $io->section('4. La limite de débit');
            $this->post($page, ['_token' => $token, 'description' => self::TEXT . ' (2)'], $anon);
            $this->post($page, ['_token' => $token, 'description' => self::TEXT . ' (3)'], $anon);
            $this->check($io, $failures, 'trois envois en dix minutes passent', $rows() === 3);
            $r = $this->post($page, ['_token' => $token, 'description' => self::TEXT . ' (4)'], $anon);
            $this->check($io, $failures, '🔴 le quatrième est refusé (429), rien n’est écrit', $r->getStatusCode() === 429 && $rows() === 3);

            $io->section('5. L’équipe');
            $this->check($io, $failures, 'un anonyme n’ouvre pas la liste admin', $this->status('/admin/signalements', $guest()) !== 200);
            $adminSession = $this->login($adminEmail, self::PASSWORD);
            $list = $this->page('/admin/signalements', $adminSession);
            $this->check($io, $failures, 'la liste admin montre le signalement (connexion ' . $this->lastLogin . ')', str_contains($list, self::TEXT));
            $this->check($io, $failures, 'tuiles Ouverts / Résolus', $this->inAnyLocale($list, 'machine_report.tile_open', []) && $this->inAnyLocale($list, 'machine_report.tile_resolved', []));
            $this->check($io, $failures, '🔴 « ce qui demande votre attention » de /admin le porte', $this->inAnyLocale($this->page('/admin', $adminSession), 'admin_attention.g_reports', []));
            $detail = $this->page('/machines/' . $id, $adminSession);
            $reportId = (int) $this->db->fetchOne('SELECT id FROM MACHINE_REPORT WHERE machineId = ? ORDER BY id LIMIT 1', [$id]);
            $resolve = '/admin/signalements/' . $reportId . '/resoudre';
            $this->check($io, $failures, '🔴 la carte « Signalements » de la fiche machine porte « Résoudre » et le lien du QR', $this->inAnyLocale($detail, 'mops.reports', []) && $this->formToken($detail, $resolve) !== '' && str_contains($detail, '/signaler-qr'));

            $r = $this->post($resolve, ['_token' => 'faux', 'note' => 'x'], $adminSession);
            $this->check($io, $failures, 'sans bon jeton : rien n’est résolu', (string) $this->db->fetchOne('SELECT status FROM MACHINE_REPORT WHERE id = ?', [$reportId]) === 'open');
            $this->post($resolve, ['_token' => $this->formToken($list, $resolve), 'note' => 'Buse changée', 'statut' => 'open'], $adminSession);
            $row = $this->db->fetchAssociative('SELECT * FROM MACHINE_REPORT WHERE id = ?', [$reportId]);
            $this->check($io, $failures, '🔴 résolu : statut, note, date et QUI', $row['status'] === 'resolved' && $row['resolutionNote'] === 'Buse changée' && $row['resolvedAt'] !== null && (int) $row['resolvedBy'] === $adminId);
            $resolved = $this->page('/admin/signalements?statut=resolved', $adminSession);
            $this->check($io, $failures, 'il passe dans « Résolus », avec sa note', str_contains($resolved, 'Buse changée'));
            $reopen = '/admin/signalements/' . $reportId . '/rouvrir';
            $this->post($reopen, ['_token' => $this->formToken($resolved, $reopen), 'statut' => 'resolved'], $adminSession);
            $row = $this->db->fetchAssociative('SELECT * FROM MACHINE_REPORT WHERE id = ?', [$reportId]);
            $this->check($io, $failures, 'rouvert : de nouveau ouvert, note et date effacées', $row['status'] === 'open' && $row['resolutionNote'] === null && $row['resolvedAt'] === null);

            $delete = '/admin/signalements/' . $reportId . '/supprimer';
            $openList = $this->page('/admin/signalements', $adminSession);
            $this->check($io, $failures, 'la suppression porte sa confirmation', str_contains($openList, 'data-controller="confirm"'));
            $this->post($delete, ['_token' => 'faux'], $adminSession);
            $this->check($io, $failures, 'sans bon jeton : rien n’est supprimé', $rows() === 3);
            $this->post($delete, ['_token' => $this->formToken($openList, $delete . '?statut=open')], $adminSession);
            $this->check($io, $failures, '🔴 supprimé : la ligne ET la photo', $rows() === 2 && !is_file($directory . '/' . $photo));

            $io->section('6. Le QR à coller');
            $qr = $this->page('/admin/machines/' . $id . '/signaler-qr', $adminSession);
            $this->check($io, $failures, 'la page imprimable nomme la machine', str_contains($qr, htmlspecialchars($name, ENT_QUOTES)));
            $this->check($io, $failures, 'le QR (ou, sans adresse publique réglée, la phrase qui dit quoi faire)', str_contains($qr, 'data:image/svg+xml') || $this->inAnyLocale($qr, 'machine_report.qr_no_url', []));
            $this->check($io, $failures, 'un anonyme n’y accède pas', $this->status('/admin/machines/' . $id . '/signaler-qr', $guest()) !== 200);
            $all = $this->page('/admin/signalements/qr', $adminSession);
            $this->check($io, $failures, 'toutes les étiquettes d’un coup : la page nomme la machine, un lien y mène depuis la liste', str_contains($all, htmlspecialchars($name, ENT_QUOTES)) && str_contains($this->page('/admin/signalements', $adminSession), 'data-qr-all'));
            $this->check($io, $failures, '🔴 la fiche PUBLIQUE de la machine mène à « Signaler une panne »', str_contains($this->page('/machines/' . $id, $guest()), 'href="' . $page . '"'));

            $io->section('7. Fonction éteinte');
            $this->features->setEnabled('machine_reports', false);
            $this->check($io, $failures, 'la page publique : 404', $this->status($page, $guest()) === 404 && $this->status('/signaler', $guest()) === 404);
            $this->check($io, $failures, 'la liste admin : 404', $this->status('/admin/signalements', $adminSession) === 404);
            $this->check($io, $failures, 'plus aucun groupe sur /admin, plus de carte sur la fiche', !$this->inAnyLocale($this->page('/admin', $adminSession), 'admin_attention.g_reports', []) && !str_contains($this->page('/machines/' . $id, $adminSession), '/signaler-qr') && !str_contains($this->page('/machines/' . $id, $guest()), 'href="' . $page . '"'));

            foreach ($this->db->fetchFirstColumn('SELECT photo FROM MACHINE_REPORT WHERE photo IS NOT NULL') as $leftover) {
                $created[] = (string) $leftover;
            }
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
            @unlink($png);
            // Les photos de la sonde vivent sur le disque, hors transaction : on les efface.
            $after = is_dir($directory) ? (scandir($directory) ?: []) : [];
            foreach (array_diff($after, $filesBefore) as $file) {
                if (str_starts_with($file, 'report-')) {
                    @unlink($directory . '/' . $file);
                }
            }
        }

        $io->section('8. Rien n’est resté');
        $afterCounts = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $afterCounts[$table]), $afterCounts[$table] === $count);
        }
        $this->check($io, $failures, 'aucun fichier de la sonde ne reste sur le disque', array_values(array_diff(is_dir($directory) ? (scandir($directory) ?: []) : [], $filesBefore)) === []);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S205 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    /** Un POST avec un fichier téléversé (le trait `ProbeBrowser` n’en porte pas). */
    private function submit(string $path, array $fields, UploadedFile $file, Session $session): \Symfony\Component\HttpFoundation\Response
    {
        $request = Request::create($path, 'POST', $fields, [], ['photo' => $file]);
        $request->headers->set('Origin', $request->getSchemeAndHttpHost());

        return $this->handle($request, $session);
    }
}
