<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\UtilisateurRepository;
use App\Service\CharterAcceptances;
use App\Service\SiteSettingService;
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
 * S209 — la charte de sécurité : un compte voit « Lire la charte » dans « À
 * faire », l'accepte, la ligne disparaît ; changer le texte la fait revenir ;
 * éteinte (ou sans texte), la fonction ne montre rien.
 *
 * ⚠️ Les phrases traduites ne sont lues qu'une fois les clés `charter.*` posées
 * dans `translations/` : la sonde cherche donc la STRUCTURE (le lien `/charte`,
 * le formulaire), et accepte la clé brute ou sa traduction pour les textes.
 *
 * ✅ Transaction annulée ; comptes et réglages comparés avant/après.
 */
#[AsCommand(name: 'app:s209:charter-probe', description: 'S209 : charte de sécurité (À faire, acceptation, nouvelle version, fonction éteinte). Transaction annulée.')]
final class S209CharterProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S209-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly CharterAcceptances $charter,
        private readonly SiteFeatureService $features,
        private readonly SiteSettingService $settings,
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
        if (!$this->charter->isReady()) {
            $io->error('La migration S208/S209 (CHARTER_ACCEPTANCE) n’est pas passée.');

            return Command::FAILURE;
        }

        $tables = ['UTILISATEUR', 'CHARTER_ACCEPTANCE', 'SITE_MODULE', 'SITE_SETTING', 'USER_MFA', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();
        $rulesBefore = $this->settings->getLabRulesHtml();

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
        $has = fn (string $html, string $key, array $params = []): bool => str_contains($html, $key) || $this->inAnyLocale($html, $key, $params);

        $this->db->beginTransaction();
        try {
            foreach ([$member, $admin] as $account) {
                $account->setPassword($this->hasher->hashPassword($account, self::PASSWORD));
            }
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId IN (?, ?)', [$member->getId(), $admin->getId()]);
            [$memberId, $memberEmail, $adminEmail] = [(int) $member->getId(), $member->getEmail(), $admin->getEmail()];
            $this->db->executeStatement('DELETE FROM CHARTER_ACCEPTANCE WHERE userId = ?', [$memberId]);
            $accepted = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM CHARTER_ACCEPTANCE WHERE userId = ?', [$memberId]);
            $todo = fn (string $html): bool => str_contains($html, 'href="/charte"');

            $io->section('1. Éteinte, ou sans texte : rien');
            $this->features->setEnabled('charter', false);
            $this->settings->setLabRules('<p>Charte de sonde, première version.</p>', '');
            $session = $this->login($memberEmail, self::PASSWORD);
            $this->check($io, $failures, 'éteinte : /charte répond 404 (connexion ' . $this->lastLogin . ')', $this->status('/charte', $session) === 404);
            $this->check($io, $failures, 'éteinte : aucune ligne dans « À faire »', !$todo($this->page('/', $session)));
            $this->features->setEnabled('charter', true);
            $this->settings->setLabRules('', '');
            $this->check($io, $failures, 'allumée mais SANS texte : /charte répond 404', $this->status('/charte', $session) === 404);
            $this->check($io, $failures, 'allumée mais SANS texte : aucune ligne dans « À faire »', !$todo($this->page('/', $session)));

            $io->section('2. Un compte neuf');
            $this->settings->setLabRules('<p>Charte de sonde, première version.</p>', '');
            $home = $this->page('/', $session);
            $this->check($io, $failures, '🔴 « À faire » porte la ligne vers /charte', $todo($home));
            $this->check($io, $failures, 'elle dit « Lire et accepter »', $has($home, 'charter.todo_cta'));
            $page = $this->page('/charte', $session);
            $this->check($io, $failures, '/charte porte le texte', str_contains($page, 'première version'));
            $token = $this->formToken($page, '/charte');
            $this->check($io, $failures, '/charte porte le bouton d’acceptation (POST + jeton)', $token !== '');
            $admin1 = $this->page('/admin/utilisateurs/' . $memberId, $this->login($adminEmail, self::PASSWORD));
            $this->check($io, $failures, 'la fiche admin dit « Pas encore acceptée »', $has($admin1, 'charter.admin_pending'));

            $io->section('3. Accepter');
            $this->post('/charte', ['_token' => 'faux'], $session);
            $this->check($io, $failures, 'jeton faux : rien d’enregistré', $accepted() === 0);
            $this->post('/charte', ['_token' => $token], $session);
            $this->check($io, $failures, '🔴 accepté : une ligne, à la version courante', $accepted() === 1 && (string) $this->db->fetchOne('SELECT version FROM CHARTER_ACCEPTANCE WHERE userId = ?', [$memberId]) === $this->charter->version());
            $this->post('/charte', ['_token' => $token], $session);
            $this->check($io, $failures, 'accepter deux fois ne double pas la ligne', $accepted() === 1);
            $this->check($io, $failures, '🔴 la ligne a DISPARU de « À faire »', !$todo($this->page('/', $session)));
            $page = $this->page('/charte', $session);
            $this->check($io, $failures, '/charte dit « Acceptée le … » et n’offre plus le bouton', $this->formToken($page, '/charte') === '' && (str_contains($page, 'charter.accepted_on') || preg_match('#\d{2}/\d{2}/\d{4}#', $page) === 1));
            $admin2 = $this->page('/admin/utilisateurs/' . $memberId, $this->login($adminEmail, self::PASSWORD));
            $this->check($io, $failures, 'la fiche admin ne dit plus « Pas encore »', !$has($admin2, 'charter.admin_pending'));

            $io->section('4. Le texte change');
            $this->settings->setLabRules('<p>Charte de sonde, DEUXIÈME version.</p>', '');
            $this->check($io, $failures, '🔴 la ligne REVIENT dans « À faire »', $todo($this->page('/', $session)));
            $page = $this->page('/charte', $session);
            $this->check($io, $failures, '/charte redemande l’accord, avec le nouveau texte', str_contains($page, 'DEUXIÈME') && $this->formToken($page, '/charte') !== '');
            $this->check($io, $failures, 'l’ancienne acceptation n’est pas effacée', $accepted() === 1);
            $this->post('/charte', ['_token' => $this->formToken($page, '/charte')], $session);
            $this->check($io, $failures, 'accepter la nouvelle version : deux lignes, la ligne disparaît de nouveau', $accepted() === 2 && !$todo($this->page('/', $session)));
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('5. Rien n’est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }
        $this->check($io, $failures, 'le règlement du lab est celui d’avant', $this->settings->getLabRulesHtml() === $rulesBefore);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S209 verte. Transaction annulée.');

        return Command::SUCCESS;
    }
}
