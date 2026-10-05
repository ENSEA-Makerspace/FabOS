<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Repository\UtilisateurRepository;
use App\Service\Feedback;
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
 * S210 — « Signaler un problème » : envoyer un retour depuis une page, le
 * recevoir avec l'adresse de la page.
 *
 *   1. l'en-tête d'un MEMBRE porte l'entrée et l'adresse de LA PAGE ; un
 *      visiteur anonyme n'a rien ;
 *   2. un envoi crée la ligne (type, message, page, navigateur, version),
 *      revient sur la page avec le remerciement, et l'équipe reçoit l'e-mail ;
 *   3. refus : message vide, jeton CSRF manquant, adresse extérieure (jamais
 *      suivie), sixième envoi dans la fenêtre (limite de débit) ;
 *   4. l'équipe : la liste montre la page, « Marquer traité » / « Rouvrir »
 *      changent l'état (et `doneAt`), « Retours des usagers » apparaît à
 *      l'accueil admin, un membre n'entre pas.
 *
 * ✅ Transaction annulée ; les comptes de tables sont comparés avant/après.
 * (« 404 si la fonction est éteinte » n'est pas éprouvé ici : l'état d'une
 * fonction est mis en cache dans le conteneur, qui survit d'une requête à l'autre.)
 */
#[AsCommand(name: 'app:s210:feedback-probe', description: 'S210 : signaler un problème depuis une page (en-tête, envoi, e-mail, liste d’équipe, limite de débit). Transaction annulée.')]
final class S210FeedbackProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S210-motdepasse';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly Feedback $feedback,
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
        if (!$this->feedback->isReady()) {
            $io->error('La migration S210 (FEEDBACK) n’est pas passée.');

            return Command::FAILURE;
        }

        $tables = ['FEEDBACK', 'EMAIL_LOG', 'messenger_messages', 'UTILISATEUR'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();

        $team = $this->feedback->teamIds();
        $admin = $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin ??= $candidate;
            } elseif (!\in_array((int) $candidate->getId(), $team, true)) {
                $member ??= $candidate;
            }
        }
        if (!$admin instanceof Utilisateur || !$member instanceof Utilisateur) {
            $io->error('Il faut un administrateur et un membre ordinaire (hors équipe).');

            return Command::FAILURE;
        }

        $this->db->beginTransaction();
        try {
            $member->setPassword($this->hasher->hashPassword($member, self::PASSWORD));
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId IN (?, ?)', [$member->getId(), $admin->getId()]);
            [$memberId, $memberEmail, $adminEmail] = [(int) $member->getId(), $member->getEmail(), $admin->getEmail()];
            $lastLog = (int) $this->db->fetchOne('SELECT COALESCE(MAX(id), 0) FROM EMAIL_LOG');
            $rows = fn (): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM FEEDBACK WHERE userId = ?', [$memberId]);
            $message = 'Sonde S210 : le bouton ne répond pas';
            $page = '/?sonde=s210';

            $io->section('1. L’en-tête');
            $anon = new \Symfony\Component\HttpFoundation\Session\Session(new \Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage());
            $this->check($io, $failures, 'un visiteur anonyme ne voit pas l’entrée', !str_contains($this->page('/', $anon), 'name="kind"'));
            $memberSession = $this->login($memberEmail, self::PASSWORD);
            $html = $this->page($page, $memberSession);
            $this->check($io, $failures, 'un membre voit l’entrée (connexion ' . $this->lastLogin . ')', str_contains($html, 'name="kind"') && str_contains($html, 'name="message"'));
            $this->check($io, $failures, '🔴 l’adresse jointe est celle de la page (requête), pas le Referer', str_contains($html, 'name="page" value="' . $page . '"'));
            $this->check($io, $failures, 'le choix du type est le composant partagé `.choice-tiles`, le message a son libellé', str_contains($html, 'class="choice-tiles"') && str_contains($html, 'for="fb-message"'));
            $token = $this->formToken($html, '/retour');
            $this->check($io, $failures, 'le formulaire porte son jeton CSRF', $token !== '');

            $io->section('2. Envoyer');
            $response = $this->post('/retour', ['_token' => $token, 'kind' => 'ux', 'message' => $message, 'page' => $page], $memberSession);
            $this->check($io, $failures, 'retour sur la page d’origine', $response->isRedirect() && $response->headers->get('Location') === $page);
            $row = $this->db->fetchAssociative('SELECT * FROM FEEDBACK WHERE userId = ? ORDER BY id DESC LIMIT 1', [$memberId]);
            $this->check($io, $failures, '🔴 la ligne : type, message, page, ouvert', $row !== false && $row['kind'] === 'ux' && $row['message'] === $message && $row['pageUrl'] === $page && $row['status'] === 'open');
            $this->check($io, $failures, 'la version de FabOS est jointe', $row !== false && (string) $row['appVersion'] !== '');
            $this->check($io, $failures, 'le remerciement s’affiche', $this->inAnyLocale($this->page($page, $memberSession), 'feedback.thanks', []));
            if ($this->mailer->isOperational() && $team !== []) {
                $mails = (int) $this->db->fetchOne("SELECT COUNT(*) FROM EMAIL_LOG WHERE id > ? AND template = 'feedback_received'", [$lastLog]);
                $this->check($io, $failures, 'l’équipe reçoit l’e-mail (' . $mails . ' en file)', $mails >= 1);
            } else {
                $io->writeln('   <comment>courrier non opérationnel : l’envoi d’e-mail n’est pas éprouvé</comment>');
            }

            $io->section('3. Refus');
            $n = $rows();
            $this->post('/retour', ['_token' => $token, 'kind' => 'bug', 'message' => '   ', 'page' => $page], $memberSession);
            $this->check($io, $failures, 'message vide : rien d’enregistré', $rows() === $n);
            $this->post('/retour', ['kind' => 'bug', 'message' => 'sans jeton', 'page' => $page], $memberSession);
            $this->check($io, $failures, 'sans jeton CSRF : rien d’enregistré', $rows() === $n);
            $this->post('/retour', ['_token' => $token, 'kind' => 'nimporte', 'message' => 'type inconnu', 'page' => $page], $memberSession);
            $this->check($io, $failures, 'type inconnu : rien d’enregistré', $rows() === $n);
            $evil = $this->post('/retour', ['_token' => $token, 'kind' => 'bug', 'message' => 'adresse extérieure', 'page' => '//exemple.invalid/x'], $memberSession);
            $this->check($io, $failures, '🔴 une adresse extérieure n’est jamais suivie', $evil->isRedirect() && !str_contains((string) $evil->headers->get('Location'), 'exemple.invalid'));
            for ($i = 0; $i < Feedback::RATE_MAX + 1; ++$i) {
                $this->post('/retour', ['_token' => $token, 'kind' => 'idea', 'message' => 'rafale ' . $i, 'page' => $page], $memberSession);
            }
            $this->check($io, $failures, sprintf('limite de débit : %d envois dans la fenêtre, pas plus', Feedback::RATE_MAX), $rows() === Feedback::RATE_MAX);
            $this->check($io, $failures, 'et la page le dit', $this->inAnyLocale($this->page($page, $memberSession), 'feedback.too_many', []));

            $io->section('4. L’équipe');
            $this->check($io, $failures, 'un membre n’entre pas dans /admin/retours', $this->status('/admin/retours', $memberSession) === 403);
            $adminSession = $this->login($adminEmail, self::PASSWORD);
            $list = $this->page('/admin/retours', $adminSession);
            $this->check($io, $failures, '🔴 la liste montre le message et la page concernée', str_contains($list, htmlspecialchars($message, ENT_QUOTES)) && str_contains($list, 'href="' . $page . '"'));
            $this->check($io, $failures, 'l’accueil admin a son groupe « Retours des usagers »', $this->inAnyLocale($this->page('/admin', $adminSession), 'admin_attention.g_feedback', []));
            $id = (int) $row['id'];
            $action = '/admin/retours/' . $id . '/statut';
            $adminToken = $this->formToken($list, $action);
            $this->post($action, ['_token' => $adminToken, 'to' => 'done'], $adminSession);
            $done = $this->db->fetchAssociative('SELECT status, doneAt FROM FEEDBACK WHERE id = ?', [$id]);
            $this->check($io, $failures, '« Marquer traité » : traité, avec sa date', $done !== false && $done['status'] === 'done' && $done['doneAt'] !== null);
            $this->check($io, $failures, 'il passe dans l’onglet Traités', str_contains($this->page('/admin/retours?statut=done', $adminSession), htmlspecialchars($message, ENT_QUOTES)));
            $this->post($action, ['_token' => $adminToken, 'to' => 'open'], $adminSession);
            $open = $this->db->fetchAssociative('SELECT status, doneAt FROM FEEDBACK WHERE id = ?', [$id]);
            $this->check($io, $failures, '« Rouvrir » : de nouveau ouvert, sans date', $open !== false && $open['status'] === 'open' && $open['doneAt'] === null);
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

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S210 verte. Transaction annulée.');

        return Command::SUCCESS;
    }
}
