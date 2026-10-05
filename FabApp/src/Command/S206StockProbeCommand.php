<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Mail\Mailer;
use App\Mail\MailTemplateCatalog;
use App\Repository\MachineRepository;
use App\Repository\UtilisateurRepository;
use App\Service\MaterialStock;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;
use Symfony\Component\HttpFoundation\Session\Session;
use Symfony\Component\HttpFoundation\Session\Storage\MockArraySessionStorage;
use Symfony\Component\HttpKernel\KernelInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;
use Symfony\Component\Security\Core\Authentication\Token\Storage\TokenStorageInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * S206 — stocks de consommables, sous-fonction activable de Matériaux.
 *
 *   1. 🔴 ÉTEINT : `/materiaux`, la fiche, l'admin des matériaux (liste et fiche),
 *      une fiche machine et `/admin` ne contiennent pas le mot « stock » (ni
 *      Bestand/Lager, existencias, scorte) — dans les CINQ langues ; les routes
 *      d'écriture répondent 404 ;
 *   2. ALLUMÉ : régler quantité / unité / seuil depuis la fiche admin ; une
 *      sortie et une entrée écrivent un mouvement (les 10 derniers sont
 *      listés) ; une sortie trop grande est refusée ;
 *   3. l'état « en stock / bientôt épuisé / épuisé » : pastille sur le
 *      catalogue et la fiche, quantité exacte réservée au personnel, colonne de
 *      la liste admin, groupe « Stocks bas » de `/admin` ;
 *   4. l'alerte e-mail : UNE par franchissement du seuil (pas à chaque
 *      mouvement) ; le gabarit est dans l'éditeur.
 *
 * ✅ Transaction annulée ; les comptes de tables sont comparés avant/après.
 * ⚠️ L'envoi réel n'est mesuré que si un compte expéditeur est configuré
 * (`Mailer::isOperational()`), comme S208/S210.
 */
#[AsCommand(name: 'app:s206:stock-probe', description: 'S206 : stocks de consommables (éteint = aucun mot « stock », réglage, mouvements, pastille, alerte au seuil). Transaction annulée.')]
final class S206StockProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S206-motdepasse';

    /** Le mot, dans les cinq langues. « Lagerort » (emplacement) n'en est pas un. */
    private const WORD = '/\b(stocks?|bestand|lager|existencias|scorte)\b/iu';

    /** Deux libellés d'archivage allemands qui disent « Bestand » au sens « parc » : hors sujet, signalés. */
    private const NOT_STOCK = ['Aus dem Bestand nehmen', 'Zurück in den Bestand'];

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly MachineRepository $machines,
        private readonly MaterialStock $stock,
        private readonly SiteFeatureService $features,
        private readonly Mailer $mailer,
        private readonly MailTemplateCatalog $catalog,
        private readonly UrlGeneratorInterface $urls,
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
        if (!$this->stock->isReady()) {
            $io->error('La migration S206 (MATERIAL_STOCK, MATERIAL_STOCK_MOVE) n’est pas passée.');

            return Command::FAILURE;
        }

        $tables = ['MATERIAL', 'MATERIAL_STOCK', 'MATERIAL_STOCK_MOVE', 'EMAIL_LOG', 'messenger_messages', 'SITE_MODULE', 'UTILISATEUR'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();

        $admin = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin = $candidate;
                break;
            }
        }
        $materialId = (int) $this->db->fetchOne('SELECT id FROM MATERIAL WHERE archivedAt IS NULL ORDER BY id LIMIT 1');
        $machine = $this->machines->findOneBy([]);
        if (!$admin instanceof Utilisateur || $materialId === 0 || $machine === null) {
            $io->error('Il faut un administrateur, un matériau vivant et une machine.');

            return Command::FAILURE;
        }
        $materialName = (string) $this->db->fetchOne('SELECT name FROM MATERIAL WHERE id = ?', [$materialId]);
        $io->writeln(sprintf('   matériau #%d « %s », machine #%d', $materialId, $materialName, $machine->getId()));

        $this->db->beginTransaction();
        try {
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId = ?', [$admin->getId()]);
            $adminEmail = $admin->getEmail();
            $this->db->executeStatement('DELETE FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]);
            $this->features->setEnabled('materials', true);

            $edit = '/admin/materials/' . $materialId . '/edit';
            $pages = [
                '/materiaux' => 'catalogue public',
                '/materiaux/' . $materialId => 'fiche publique',
                '/admin/materials' => 'liste admin',
                $edit => 'fiche admin',
                $this->urls->generate('app_machine_detail', ['id' => $machine->getId()]) => 'fiche machine',
                '/admin' => 'tableau de bord',
            ];

            $io->section('1. Éteint : aucune mention de stock');
            $this->features->setEnabled('stock', false);
            $session = $this->login($adminEmail, self::PASSWORD);
            foreach (['fr', 'en', 'de', 'es', 'it'] as $locale) {
                $session->set('_locale', $locale);
                $hits = [];
                foreach ($pages as $path => $label) {
                    $html = $this->page($path, $session);
                    $text = strip_tags((string) preg_replace('#<(script|style)\b.*?</\1>#is', '', str_replace(self::NOT_STOCK, '', $html)));
                    if ($html === '' || $this->status($path, $session) !== 200) {
                        $hits[] = $label . ' (pas 200)';
                    } elseif (preg_match(self::WORD, $text, $m)) {
                        $hits[] = $label . ' : « ' . $m[0] . ' »';
                    }
                }
                $this->check($io, $failures, sprintf('🔴 [%s] %d pages sans le mot%s', $locale, \count($pages), $hits === [] ? '' : ' — ' . implode(' ; ', $hits)), $hits === []);
            }
            $session->set('_locale', 'fr');
            $html = $this->page($edit, $session);
            $this->check($io, $failures, 'la fiche admin ne porte ni bloc ni formulaire de stock', !str_contains($html, 'id="stock"') && !str_contains($html, '/stock'));
            $this->check($io, $failures, 'écrire est 404 (réglage)', $this->post('/admin/materials/' . $materialId . '/stock', ['_token' => 'x', 'quantity' => '5', 'unit' => 'kg'], $session)->getStatusCode() === 404);
            $this->check($io, $failures, 'écrire est 404 (mouvement)', $this->post('/admin/materials/' . $materialId . '/stock/move', ['_token' => 'x', 'amount' => '1'], $session)->getStatusCode() === 404);
            $this->check($io, $failures, 'aucune ligne écrite', (int) $this->db->fetchOne('SELECT COUNT(*) FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]) === 0);

            $io->section('2. Allumé : régler, puis entrée / sortie');
            $this->features->setEnabled('stock', true);
            $html = $this->page($edit, $session);
            $saveAction = '/admin/materials/' . $materialId . '/stock';
            $this->check($io, $failures, 'la fiche admin porte le bloc Stock', str_contains($html, 'id="stock"') && str_contains($html, 'name="quantity"'));
            $this->check($io, $failures, 'le matériau n’est pas suivi : pas de formulaire d’entrée / sortie', !str_contains($html, '/stock/move'));
            $list = $this->page('/admin/materials', $session);
            $this->check($io, $failures, 'la liste admin gagne sa colonne Stock', $this->inAnyLocale($list, 'stock.col_stock', []));

            $this->post($saveAction, ['_token' => $this->formToken($html, $saveAction), 'quantity' => '10', 'unit' => 'kg', 'lowThreshold' => '3'], $session);
            $row = $this->db->fetchAssociative('SELECT * FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]);
            $this->check($io, $failures, 'réglé : 10 kg, seuil 3', $row !== false && (float) $row['quantity'] === 10.0 && $row['unit'] === 'kg' && (float) $row['lowThreshold'] === 3.0);
            $this->check($io, $failures, 'la quantité initiale est écrite au journal', (int) $this->db->fetchOne('SELECT COUNT(*) FROM MATERIAL_STOCK_MOVE WHERE materialId = ? AND delta = 10', [$materialId]) === 1);

            $html = $this->page($edit, $session);
            $moveAction = '/admin/materials/' . $materialId . '/stock/move';
            $moveToken = $this->formToken($html, $moveAction);
            $this->check($io, $failures, 'suivi : le geste Entrée / Sortie est là', $moveToken !== '' && str_contains($html, 'value="in"') && str_contains($html, 'value="out"'));
            $logged = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG WHERE template = \'stock_low\'');

            $this->post($moveAction, ['_token' => $moveToken, 'amount' => '4', 'direction' => 'out', 'note' => 'Sonde S206 : essai'], $session);
            $qty = fn (): float => (float) $this->db->fetchOne('SELECT quantity FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]);
            $this->check($io, $failures, 'sortie de 4 : il reste 6', $qty() === 6.0);
            $move = $this->db->fetchAssociative('SELECT * FROM MATERIAL_STOCK_MOVE WHERE materialId = ? ORDER BY id DESC LIMIT 1', [$materialId]);
            $this->check($io, $failures, 'le mouvement dit delta, note et auteur', $move !== false && (float) $move['delta'] === -4.0 && $move['note'] === 'Sonde S206 : essai' && (int) $move['userId'] === (int) $admin->getId());
            $this->check($io, $failures, 'au-dessus du seuil : aucune alerte', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG WHERE template = \'stock_low\'') === $logged);
            $this->check($io, $failures, 'l’historique de la fiche cite la note', str_contains($this->page($edit, $session), 'Sonde S206 : essai'));
            $this->post($moveAction, ['_token' => $moveToken, 'amount' => '99', 'direction' => 'out', 'note' => ''], $session);
            $this->check($io, $failures, 'une sortie trop grande est refusée : rien ne change', $qty() === 6.0);
            $this->post($moveAction, ['_token' => $moveToken, 'amount' => '0', 'direction' => 'in', 'note' => ''], $session);
            $this->check($io, $failures, 'un mouvement nul est refusé', $qty() === 6.0);
            $this->post($moveAction, ['_token' => 'faux', 'amount' => '1', 'direction' => 'in', 'note' => ''], $session);
            $this->check($io, $failures, 'sans jeton CSRF valide : refusé', $qty() === 6.0);

            $io->section('3. L’état, la pastille, le personnel');
            $anon = new Session(new MockArraySessionStorage());
            $detail = '/materiaux/' . $materialId;
            $this->check($io, $failures, 'en stock : pastille sur la fiche et sur la carte', $this->inAnyLocale($this->page($detail, $anon), 'stock.state_in', []) && $this->inAnyLocale($this->page('/materiaux', $anon), 'stock.state_in', []));
            $anonPage = $this->page($detail, $anon);
            if (str_contains($anonPage, '6 kg')) {
                $at = strpos($anonPage, '6 kg');
                $io->writeln('   contexte : ' . preg_replace('/\s+/', ' ', substr($anonPage, max(0, $at - 160), 220)));
            }
            $this->check($io, $failures, '🔴 le public ne voit PAS la quantité exacte', !str_contains($anonPage, '6 kg'));
            $this->check($io, $failures, 'le personnel la voit', str_contains($this->page($detail, $session), '6 kg'));

            $this->post($moveAction, ['_token' => $moveToken, 'amount' => '4', 'direction' => 'out', 'note' => ''], $session);
            $after1 = (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG WHERE template = \'stock_low\'');
            $this->check($io, $failures, 'sous le seuil : « bientôt épuisé » (fiche, carte, liste admin)', $this->inAnyLocale($this->page($detail, $anon), 'stock.state_low', []) && $this->inAnyLocale($this->page('/materiaux', $anon), 'stock.state_low', []) && $this->inAnyLocale($this->page('/admin/materials', $session), 'stock.state_low', []));
            $this->check($io, $failures, '/admin : le groupe « Stocks bas » cite le matériau', $this->inAnyLocale($this->page('/admin', $session), 'admin_attention.g_stock', []) && str_contains($this->page('/admin', $session), htmlspecialchars($materialName, ENT_QUOTES)));

            $io->section('4. L’alerte : une par franchissement');
            if ($this->mailer->isOperational()) {
                $sent = $after1 - $logged;
                $this->check($io, $failures, sprintf('🔴 le franchissement envoie l’alerte à l’équipe (%d e-mail(s))', $sent), $sent >= 1);
                $this->stock->move($materialId, -1.0, '', (int) $admin->getId());
                $this->stock->move($materialId, -1.0, '', (int) $admin->getId());
                $this->check($io, $failures, '🔴 déjà dessous (jusqu’à épuisé) : AUCUNE alerte de plus', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG WHERE template = \'stock_low\'') === $after1);
                $this->stock->move($materialId, 8.0, 'réassort', (int) $admin->getId());
                $this->stock->move($materialId, -8.0, '', (int) $admin->getId());
                $this->check($io, $failures, 'réassort puis nouvelle chute : une nouvelle alerte', (int) $this->db->fetchOne('SELECT COUNT(*) FROM EMAIL_LOG WHERE template = \'stock_low\'') === $after1 + $sent);
                $ctx = (string) $this->db->fetchOne("SELECT contextJson FROM EMAIL_LOG WHERE template = 'stock_low' ORDER BY id DESC LIMIT 1");
                $this->check($io, $failures, 'le contexte porte matériau, quantité, seuil, lien', str_contains($ctx, 'material') && str_contains($ctx, 'threshold') && str_contains($ctx, 'link'));
            } else {
                $io->writeln('   <comment>aucun compte expéditeur configuré : l’envoi n’est pas mesurable ici (le franchissement l’est dans la logique ci-dessous)</comment>');
                $this->stock->move($materialId, -2.0, '', (int) $admin->getId());
            }
            $this->check($io, $failures, 'le gabarit « stock_low » est dans l’éditeur, avec ses champs', $this->catalog->exists('stock_low') && array_diff(['material', 'quantity', 'threshold', 'link'], $this->catalog->fieldsOf('stock_low')) === []);

            $io->section('5. Épuisé');
            $this->stock->configure($materialId, 0.0, 'kg', 3.0, (int) $admin->getId());
            $this->check($io, $failures, 'quantité 0 : « épuisé » sur la fiche publique', $this->inAnyLocale($this->page($detail, $anon), 'stock.state_out', []));
            $this->check($io, $failures, '/admin : épuisé aussi', $this->inAnyLocale($this->page('/admin', $session), 'admin_attention.st_stock_out', []));

            $io->section('6. De nouveau éteint');
            $this->features->setEnabled('stock', false);
            $off = $this->page($detail, $anon) . $this->page('/materiaux', $anon) . $this->page('/admin/materials', $session);
            $this->check($io, $failures, '🔴 la donnée existe encore, mais rien ne se dit', !preg_match(self::WORD, strip_tags($off)) && (int) $this->db->fetchOne('SELECT COUNT(*) FROM MATERIAL_STOCK WHERE materialId = ?', [$materialId]) === 1);
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
        $io->success('Sonde S206 verte. Transaction annulée.');

        return Command::SUCCESS;
    }
}
