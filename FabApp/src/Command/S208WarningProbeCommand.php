<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Mail\Mailer;
use App\Repository\UtilisateurRepository;
use App\Service\UserWarnings;
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
 * S208 — les avertissements : en poser un depuis une fiche, le retrouver dans
 * l'historique et dans la liste, le lever, régler les motifs, être prévenu par
 * e-mail, et tout se tait quand la fonction est éteinte.
 *
 * ✅ Transaction annulée ; les comptes de tables sont comparés avant/après.
 */
#[AsCommand(name: 'app:s208:warning-probe', description: 'S208 : avertissements (poser, historique, liste, lever, motifs, e-mail aux admins, fonction éteinte). Transaction annulée.')]
final class S208WarningProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S208-motdepasse';
    private const NOTE = 'Sonde S208 : casque de soudure laissé allumé';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly UserWarnings $warnings,
        private readonly SiteFeatureService $features,
        private readonly Mailer $mailer,
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
        if (!$this->warnings->isReady()) {
            $io->error('La migration S208/S209 (USER_WARNING, WARNING_REASON) n’est pas passée.');

            return Command::FAILURE;
        }

        $tables = ['UTILISATEUR', 'USER_WARNING', 'WARNING_REASON', 'EMAIL_LOG', 'SITE_MODULE', 'USER_MFA', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();

        $admin = $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin ??= $candidate;
            } else {
                $member ??= $candidate;
            }
        }
        if (!$admin instanceof Utilisateur || !$member instanceof Utilisateur) {
            $io->error('Il faut un administrateur et un membre actifs et vérifiés.');

            return Command::FAILURE;
        }

        $this->db->beginTransaction();
        try {
            $this->features->setEnabled('warnings', true);
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$admin->getId()]);
            [$memberId, $adminEmail] = [(int) $member->getId(), $admin->getEmail()];
            $fiche = '/admin/utilisateurs/' . $memberId;
            $reason = $this->warnings->reasons(true)[0] ?? null;
            $this->check($io, $failures, 'la migration a semé des motifs', $reason !== null);
            if ($reason === null) {
                return Command::FAILURE;
            }
            $mine = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM USER_WARNING WHERE userId = ?', [$memberId]);
            $mails = fn (): int => (int) $this->db->fetchOne("SELECT COUNT(*) FROM EMAIL_LOG WHERE template = 'warning_issued'");

            $io->section('1. Poser, depuis la fiche');
            $session = $this->login($adminEmail, self::PASSWORD);
            $html = $this->page($fiche, $session);
            $token = $this->formToken($html, $fiche . '/avertissements');
            $this->check($io, $failures, 'la fiche porte le formulaire « Ajouter un avertissement » (connexion ' . $this->lastLogin . ')', $token !== '');
            $this->check($io, $failures, 'le formulaire est REPLIÉ dans un <details>', (bool) preg_match('#<details[^>]*>\s*<summary>[^<]*</summary>\s*<form[^>]*action="' . preg_quote($fiche . '/avertissements', '#') . '"#', $html));
            $this->check($io, $failures, 'le motif est dans la liste', str_contains($html, '<option value="' . $reason['id'] . '">'));
            $mailsBefore = $mails();
            $this->post($fiche . '/avertissements', ['_token' => 'faux', 'reason' => (string) $reason['id'], 'note' => self::NOTE], $session);
            $this->check($io, $failures, 'jeton CSRF faux : refusé, rien posé', $mine() === 0);
            $this->post($fiche . '/avertissements', ['_token' => $token, 'reason' => '0', 'note' => self::NOTE], $session);
            $this->check($io, $failures, 'sans motif : refusé, rien posé', $mine() === 0);
            $this->post($fiche . '/avertissements', ['_token' => $token, 'reason' => (string) $reason['id'], 'note' => self::NOTE], $session);
            $this->check($io, $failures, '🔴 avec motif : posé', $mine() === 1);
            $row = $this->db->fetchAssociative('SELECT * FROM USER_WARNING WHERE userId = ?', [$memberId]);
            $this->check($io, $failures, 'la ligne dit QUI, le motif et la note', $row !== false && (int) $row['issuedBy'] === (int) $admin->getId() && (int) $row['reasonId'] === $reason['id'] && $row['note'] === self::NOTE && $row['liftedAt'] === null);
            $warningId = (int) ($row['id'] ?? 0);

            $io->section('2. L’historique et la liste');
            $html = $this->page($fiche, $session);
            $this->check($io, $failures, '🔴 la fiche retrouve la note et le motif', str_contains($html, self::NOTE) && str_contains($html, $reason['label']));
            $this->check($io, $failures, 'la fiche porte « Lever » (POST)', $this->formToken($html, '/admin/avertissements/' . $warningId . '/lever') !== '');
            $this->check($io, $failures, '/admin/avertissements répond 200 et la liste porte la note', $this->status('/admin/avertissements', $session) === 200 && str_contains($this->page('/admin/avertissements', $session), self::NOTE));
            $this->check($io, $failures, 'l’état « Levés » ne la montre pas', !str_contains($this->page('/admin/avertissements?etat=lifted', $session), self::NOTE));

            $io->section('3. L’e-mail aux administrateurs');
            if ($this->mailer->isOperational()) {
                $this->check($io, $failures, '🔴 un e-mail « warning_issued » est en file pour au moins un administrateur', $mails() > $mailsBefore);
                $ctx = (string) $this->db->fetchOne("SELECT contextJson FROM EMAIL_LOG WHERE template = 'warning_issued' ORDER BY id DESC LIMIT 1");
                // Le contexte est du JSON : accents et barres y sont échappés — on le relit.
                $ctx = (string) json_encode(json_decode($ctx, true), JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES);
                $this->check($io, $failures, 'il porte la note, le motif et le lien de la fiche', str_contains($ctx, 'S208') && str_contains($ctx, $reason['label']) && str_contains($ctx, '/admin/utilisateurs/' . $memberId));
            } else {
                $io->writeln('   <comment>– courrier non configuré ou en pause : l’envoi n’est pas mesuré ici</comment>');
            }

            $io->section('4. Lever');
            $this->post('/admin/avertissements/' . $warningId . '/lever', ['_token' => 'faux'], $session);
            $this->check($io, $failures, 'jeton faux : toujours actif', $this->db->fetchOne('SELECT liftedAt FROM USER_WARNING WHERE id = ?', [$warningId]) === null);
            $leverToken = $this->formToken($this->page($fiche, $session), '/admin/avertissements/' . $warningId . '/lever');
            $this->post('/admin/avertissements/' . $warningId . '/lever', ['_token' => $leverToken], $session);
            $this->check($io, $failures, '🔴 levé : liftedAt posé, la ligne reste', $this->db->fetchOne('SELECT liftedAt FROM USER_WARNING WHERE id = ?', [$warningId]) !== null && $mine() === 1);
            $this->check($io, $failures, 'la fiche garde l’historique mais n’offre plus « Lever »', str_contains($this->page($fiche, $session), self::NOTE) && $this->formToken($this->page($fiche, $session), '/admin/avertissements/' . $warningId . '/lever') === '');
            $this->check($io, $failures, 'la liste « Levés » la montre', str_contains($this->page('/admin/avertissements?etat=lifted', $session), self::NOTE));

            $io->section('5. Les motifs');
            $listToken = $this->formToken($this->page('/admin/avertissements', $session), '/admin/avertissements/motifs');
            $this->check($io, $failures, 'le réglage des motifs est replié sur la même page', $listToken !== '' && str_contains($this->page('/admin/avertissements', $session), 'id="motifs"'));
            $this->post('/admin/avertissements/motifs', ['_token' => $listToken, 'action' => 'add', 'label' => 'Sonde S208 : motif neuf'], $session);
            $newId = (int) $this->db->fetchOne("SELECT id FROM WARNING_REASON WHERE label = 'Sonde S208 : motif neuf'");
            $this->check($io, $failures, 'un motif neuf est ajouté', $newId > 0);
            $this->check($io, $failures, 'il est proposé sur la fiche', str_contains($this->page($fiche, $session), 'data-warning-reason="' . $newId . '"'));
            $this->post('/admin/avertissements/motifs', ['_token' => $listToken, 'action' => 'disable', 'id' => (string) $newId], $session);
            $this->check($io, $failures, 'retiré de la liste : plus proposé, mais pas effacé', !str_contains($this->page($fiche, $session), 'data-warning-reason="' . $newId . '"') && (int) $this->db->fetchOne('SELECT COUNT(*) FROM WARNING_REASON WHERE id = ?', [$newId]) === 1);

            $io->section('6. Fonction éteinte');
            $this->features->setEnabled('warnings', false);
            $this->check($io, $failures, '🔴 /admin/avertissements répond 404', $this->status('/admin/avertissements', $session) === 404);
            $html = $this->page($fiche, $session);
            $this->check($io, $failures, 'la fiche ne porte plus la carte', !str_contains($html, 'id="warnings"'));
            $this->post($fiche . '/avertissements', ['_token' => $token, 'reason' => (string) $reason['id'], 'note' => 'ne doit pas passer'], $session);
            $this->check($io, $failures, 'et un POST ne pose rien', $mine() === 1);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('7. Rien n’est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S208 verte. Transaction annulée.');

        return Command::SUCCESS;
    }
}
