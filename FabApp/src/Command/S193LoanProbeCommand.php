<?php

namespace App\Command;

use App\Entity\LoanableItem;
use App\Entity\Utilisateur;
use App\Repository\LoanableItemRepository;
use App\Repository\UtilisateurRepository;
use App\Security\ConsoleRenderAuthenticator;
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
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;

/**
 * S193 — la circulation d'un objet, depuis sa fiche : prêter, voir qui l'a, le
 * rendre, l'historique ; le stock qui refuse un prêt de trop ; le membre qui lit
 * SON prêt. Comme un navigateur (admin via le rendu console), transaction annulée.
 */
#[AsCommand(name: 'app:s193:loan-probe', description: 'S193 : prêter depuis la fiche, qui l\'a, rendre depuis la fiche, historique, stock plein refusé, le membre voit son prêt. Transaction annulée.')]
final class S193LoanProbeCommand extends Command
{
    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly LoanableItemRepository $items,
        private readonly UtilisateurRepository $users,
        private readonly TokenStorageInterface $tokens,
        private readonly ConsoleRenderAuthenticator $renderAs,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('save', null, InputOption::VALUE_REQUIRED, 'Dossier où enregistrer la fiche avec deux prêts en cours, et la page du membre');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $save = $input->getOption('save');
        $io = new SymfonyStyle($input, $output);
        $failures = [];
        $item = null;
        foreach ($this->items->findBy([], ['id' => 'ASC']) as $candidate) {
            if ($candidate instanceof LoanableItem && !$candidate->isArchived()) {
                $item = $candidate;
                break;
            }
        }
        $member = null;
        $other = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $u) {
            if (!in_array('ROLE_ADMIN', $u->getRoles(), true)) {
                $member === null ? $member = $u : $other ??= $u;
            }
        }
        if (!$item instanceof LoanableItem || !$member instanceof Utilisateur) {
            $io->error('Il faut un objet prêtable et un membre non admin.');

            return Command::FAILURE;
        }
        $itemId = (int) $item->getId();
        $quantity = $item->getQuantity();
        $loansBefore = (int) $this->db->fetchOne('SELECT COUNT(*) FROM LOAN');
        $io->writeln(sprintf('   objet « %s », %d exemplaire(s)', $item->getName(), $quantity));

        $this->db->beginTransaction();
        try {
            // Un stock de 2 exemplaires, tous rentrés : l'état de départ de la sonde.
            $this->db->executeStatement('UPDATE LOANABLE_ITEM SET quantity = 2 WHERE id = ?', [$itemId]);
            $this->db->executeStatement("UPDATE LOAN SET status = 'returned', actualReturnDate = CURDATE() WHERE itemId = ? AND status <> 'returned'", [$itemId]);
            $admin = new Session(new MockArraySessionStorage());

            $io->section('1. La fiche de l\'objet');
            $sheet = $this->get('/admin/loanable-items/' . $itemId . '/edit', $admin);
            $this->check($io, $failures, 'elle s\'ouvre sur « En circulation »', str_contains($sheet, 'id="circulation"'));
            $this->check($io, $failures, 'et propose « Prêter cet objet » (2 libres)', str_contains($sheet, '/admin/loans/new?item=' . $itemId));

            $io->section('2. Prêter depuis la fiche');
            $form = $this->get('/admin/loans/new?item=' . $itemId, $admin);
            $this->check($io, $failures, 'l\'objet arrive pré-choisi', (bool) preg_match('#<option value="' . $itemId . '" selected#', $form));
            $r1 = $this->lend($admin, $form, $itemId, 'Visiteur Sonde', (new \DateTimeImmutable('-3 days'))->format('Y-m-d'));
            if (!$r1->isRedirect()) {
                preg_match_all('#class="[^"]*(?:form-errors|invalid-feedback|form-error)[^"]*"[^>]*>(.*?)</#s', (string) $r1->getContent(), $errs);
                $io->writeln('   réponse ' . $r1->getStatusCode() . ' : ' . implode(' | ', array_map(static fn ($e) => trim(strip_tags($e)), $errs[1])));
            }
            $this->check($io, $failures, 'le prêt enregistré ramène à la fiche (#circulation)', $r1->isRedirect() && str_contains((string) $r1->headers->get('Location'), '/admin/loanable-items/' . $itemId . '/edit#circulation'));
            $sheet = $this->get('/admin/loanable-items/' . $itemId . '/edit', $admin);
            $this->check($io, $failures, 'la fiche dit QUI l\'a', str_contains($sheet, 'Visiteur Sonde'));
            $this->check($io, $failures, 'et qu\'il est EN RETARD (date passée)', str_contains($sheet, 'loan-row is-overdue'));

            $io->section('3. Le stock refuse un prêt de trop');
            $memberLoan = $this->db->executeStatement(
                "INSERT INTO LOAN (itemId, borrowerId, dateTaken, status, createdAt) VALUES (?, ?, NOW(), 'out', NOW())",
                [$itemId, $member->getId()],
            );
            $form = $this->get('/admin/loans/new?item=' . $itemId, $admin);
            $r3 = $this->lend($admin, $form, $itemId, 'Troisième Sonde', null);
            $this->check($io, $failures, '🔴 2 exemplaires sortis sur 2 : le 3ᵉ prêt est REFUSÉ (' . $r3->getStatusCode() . ')', $r3->getStatusCode() === 422 && (int) $this->db->fetchOne("SELECT COUNT(*) FROM LOAN WHERE itemId = ? AND status = 'out'", [$itemId]) === 2);
            $body = (string) $r3->getContent();
            $this->check($io, $failures, '… et pour la BONNE raison : le stock (pas le jeton CSRF)', !str_contains($body, 'CSRF') && (str_contains($body, 'Plus aucun exemplaire') || str_contains($body, 'No unit of')));
            $sheet = $this->get('/admin/loanable-items/' . $itemId . '/edit', $admin);
            $this->check($io, $failures, 'la fiche ne propose plus « Prêter » et dit pourquoi', !str_contains($sheet, '/admin/loans/new?item=' . $itemId));
            if (is_string($save)) {
                file_put_contents($save . '/s193-fiche.html', $sheet);
                file_put_contents($save . '/s193-membre.html', $this->get('/prets/' . $itemId, new Session(new MockArraySessionStorage()), $member->getEmail()));
            }

            $io->section('4. Le membre lit SON prêt ; un autre, non');
            $this->check($io, $failures, 'l\'emprunteur voit « Vous l\'avez depuis… »', str_contains($this->get('/prets/' . $itemId, new Session(new MockArraySessionStorage()), $member->getEmail()), 'loan-item__mine'));
            if ($other instanceof Utilisateur) {
                $this->check($io, $failures, 'un autre membre, non', !str_contains($this->get('/prets/' . $itemId, new Session(new MockArraySessionStorage()), $other->getEmail()), 'loan-item__mine'));
            }

            $io->section('5. Rendre depuis la fiche');
            $list = $this->get('/admin/loans', $admin);
            $this->check($io, $failures, 'la liste des prêts ne rend plus en ligne : elle mène à la fiche', !str_contains($list, 'return-inline') && str_contains($list, '/admin/loanable-items/' . $itemId . '/edit#loan-'));
            $sheet = $this->get('/admin/loanable-items/' . $itemId . '/edit', $admin);
            preg_match('#action="/admin/loans/(\d+)/return">\s*<input type="hidden" name="_token" value="([^"]+)"#', $sheet, $m);
            $back = $this->handle(Request::create('/admin/loans/' . ($m[1] ?? 0) . '/return', 'POST', ['_token' => $m[2] ?? '', 'conditionReturn' => 'rayure sonde']), $admin);
            $this->check($io, $failures, 'rendu : retour à la fiche', str_contains((string) $back->headers->get('Location'), '/admin/loanable-items/' . $itemId . '/edit#circulation'));
            $sheet = $this->get('/admin/loanable-items/' . $itemId . '/edit', $admin);
            $this->check($io, $failures, 'l\'historique montre le retour et son état', str_contains($sheet, 'rayure sonde'));
            $this->check($io, $failures, 'un exemplaire est de nouveau prêtable', str_contains($sheet, '/admin/loans/new?item=' . $itemId));
            $replay = (string) $this->db->fetchOne('SELECT actualReturnDate FROM LOAN WHERE id = ?', [$m[1] ?? 0]);
            $this->handle(Request::create('/admin/loans/' . ($m[1] ?? 0) . '/return', 'POST', ['_token' => $m[2] ?? '', 'conditionReturn' => 'rejouée']), $admin);
            $this->check($io, $failures, 'rendre deux fois ne change rien', !str_contains($this->get('/admin/loanable-items/' . $itemId . '/edit', $admin), 'rejouée') && (string) $this->db->fetchOne('SELECT actualReturnDate FROM LOAN WHERE id = ?', [$m[1] ?? 0]) === $replay);
            unset($memberLoan);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
            (new \ReflectionProperty(ConsoleRenderAuthenticator::class, 'identifier'))->setValue($this->renderAs, null);
        }

        $this->check($io, $failures, 'LOAN revenue à son compte de départ', (int) $this->db->fetchOne('SELECT COUNT(*) FROM LOAN') === $loansBefore);
        $this->check($io, $failures, 'la quantité de l\'objet est intacte', (int) $this->db->fetchOne('SELECT quantity FROM LOANABLE_ITEM WHERE id = ?', [$itemId]) === $quantity);

        if ($failures !== []) {
            $io->error(count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S193 verte.');

        return Command::SUCCESS;
    }

    private function lend(Session $admin, string $form, int $itemId, string $name, ?string $due): Response
    {
        $fields = [];
        preg_match_all('#<input[^>]*name="(loan_admin[^"]*)"[^>]*value="([^"]*)"#', $form, $m, PREG_SET_ORDER);
        foreach ($m as $row) {
            $fields[html_entity_decode($row[1])] = html_entity_decode($row[2]);
        }
        $prefix = preg_match('#name="([a-z_]+)\[item\]"#', $form, $p) ? $p[1] : 'loan_admin';
        $fields[$prefix . '[item]'] = (string) $itemId;
        $fields[$prefix . '[borrowerName]'] = $name;
        $fields[$prefix . '[dateTaken]'] = (new \DateTimeImmutable('-3 days'))->format('Y-m-d\TH:i');
        $fields[$prefix . '[expectedReturnDate]'] = (string) $due;
        if (preg_match('#name="' . preg_quote($prefix, '#') . '\[_token\]" value="([^"]+)"#', $form, $t)) {
            $fields[$prefix . '[_token]'] = $t[1];
        }
        unset($fields[$prefix . '[save]']);

        return $this->handle(Request::create('/admin/loans/new?item=' . $itemId, 'POST', self::nest($fields)), $admin);
    }

    /** @param array<string, string> $flat « a[b] » → ['a' => ['b' => …]] */
    private static function nest(array $flat): array
    {
        $out = [];
        parse_str(http_build_query($flat), $out);

        return $out;
    }

    private function get(string $path, Session $session, ?string $as = null): string
    {
        return (string) $this->handle(Request::create($path), $session, $as)->getContent();
    }

    /** Admin par défaut (premier admin, rendu console) ; `$as` pour un membre. */
    private function handle(Request $request, Session $session, ?string $as = null): Response
    {
        $this->tokens->setToken(null);
        $this->renderAs->renderAs($as);
        $request->setSession($session);
        // ⚠️ Les formulaires ont un CSRF « sans état » (jeton `submit`) : Symfony
        // vérifie que le POST vient du même site par l'en-tête Origin, qu'un vrai
        // navigateur envoie toujours. Sans lui : « The CSRF token is invalid ».
        if ($request->isMethod('POST')) {
            $request->headers->set('Origin', $request->getSchemeAndHttpHost());
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
