<?php

namespace App\Command;

use App\Entity\Badge;
use App\Entity\Machine;
use App\Entity\MachineBadge;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurBadge;
use App\Repository\FormationRepository;
use App\Repository\UtilisateurRepository;
use App\Service\MachineAccessService;
use App\Service\TrainingQualificationService;
use App\Training\BadgeGrants;
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
 * S202 — un badge donné à la main ouvre, se lit, se retire, et ne revient pas tout seul.
 *
 *   1. Par l'ÉCRAN, comme un administrateur : sans motif → refusé ; avec motif →
 *      le lecteur ouvre la machine qui l'exige (fermée juste avant), le journal
 *      dit qui et pourquoi, « Mes badges » du membre le dit aussi.
 *   2. Retirer : le lecteur refuse de nouveau, le retrait reste listé sur la fiche.
 *   3. Le verrou, avec son TÉMOIN : un badge venu d'une formation validée,
 *      effacé SANS passer par le retrait, revient à la sauvegarde suivante de la
 *      progression (le chemin automatique marche) ; retiré par l'équipe, il ne
 *      revient pas.
 *
 * ✅ Transaction annulée ; les comptes de tables sont comparés avant/après.
 */
#[AsCommand(name: 'app:s202:badge-grant-probe', description: 'S202 : attribuer / retirer un badge à la main (motif, lecteur, « Mes badges », verrou contre la ré-attribution). Transaction annulée.')]
final class S202BadgeGrantProbeCommand extends Command
{
    use ProbeBrowser;

    private const PASSWORD = 'sonde-S202-motdepasse';
    private const REASON = 'Sonde S202 : formée au fablab voisin';

    public function __construct(
        private readonly KernelInterface $kernel,
        private readonly Connection $db,
        private readonly EntityManagerInterface $entityManager,
        private readonly UtilisateurRepository $users,
        private readonly FormationRepository $formations,
        private readonly MachineAccessService $machineAccess,
        private readonly TrainingQualificationService $qualification,
        private readonly BadgeGrants $grants,
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
        if (!$this->grants->isReady()) {
            $io->error('La migration S202 (BADGE_GRANT) n’est pas passée.');

            return Command::FAILURE;
        }

        $tables = ['UTILISATEUR', 'UTILISATEUR_BADGE', 'BADGE_GRANT', 'PROGRESSION', 'ACCESS_RFID_LOG', 'USER_SESSION'];
        $counts = fn (): array => array_combine($tables, array_map(fn (string $t): int => (int) $this->db->fetchOne('SELECT COUNT(*) FROM ' . $t), $tables));
        $before = $counts();
        $heldBefore = $this->db->fetchAllNumeric('SELECT utilisateurId, badgeId FROM UTILISATEUR_BADGE ORDER BY 1, 2');

        // Un membre (non administrateur) et une machine à jeton qui EXIGE un badge qu'il n'a pas.
        $admin = $member = null;
        foreach ($this->users->findBy(['statut' => 'actif', 'isVerified' => true]) as $candidate) {
            if (\in_array('ROLE_ADMIN', $candidate->getRoles(), true)) {
                $admin ??= $candidate;
            } else {
                $member ??= $candidate;
            }
        }
        $machine = $badge = null;
        if ($member instanceof Utilisateur) {
            foreach ($this->entityManager->getRepository(MachineBadge::class)->findBy(['requiredForAccess' => true]) as $link) {
                $m = $link->getMachine();
                $b = $link->getBadge();
                if ($m instanceof Machine && $b instanceof Badge && (string) $m->getMachineToken() !== ''
                    && \count($m->getMachineBadges()) === 1 && !$this->grants->holds($member, $b)) {
                    [$machine, $badge] = [$m, $b];
                    break;
                }
            }
        }
        if (!$admin instanceof Utilisateur || !$member instanceof Utilisateur || !$machine instanceof Machine || !$badge instanceof Badge) {
            $io->error('Il faut un administrateur, un membre, et une machine à jeton qui exige UN badge que ce membre n’a pas.');

            return Command::FAILURE;
        }
        $io->writeln(sprintf('   membre #%d, badge « %s », machine « %s »', $member->getId(), $badge->getNom(), $machine->getNom()));

        $this->db->beginTransaction();
        try {
            if ((string) $member->getIdentifiantRfid() === '') {
                $member->setIdentifiantRfid('SONDE-S202-' . bin2hex(random_bytes(3)));
            }
            $member->setPassword($this->hasher->hashPassword($member, self::PASSWORD));
            $admin->setPassword($this->hasher->hashPassword($admin, self::PASSWORD));
            $this->entityManager->flush();
            $this->db->executeStatement('DELETE FROM USER_MFA WHERE userId IN (?, ?)', [$member->getId(), $admin->getId()]);
            [$memberId, $adminId, $badgeId] = [(int) $member->getId(), (int) $admin->getId(), (int) $badge->getId()];
            [$email, $adminEmail, $adminName] = [$member->getEmail(), $admin->getEmail(), $admin->getDisplayName()];
            $rfid = (string) $member->getIdentifiantRfid();
            $token = (string) $machine->getMachineToken();
            $opens = fn (): bool => (bool) ($this->machineAccess->authorize($token, $rfid)['authorized'] ?? false);
            $held = fn (): bool => (bool) $this->db->fetchOne('SELECT 1 FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?', [$memberId, $badgeId]);

            $io->section('Avant');
            $this->check($io, $failures, 'le lecteur REFUSE : le badge manque', !$opens());

            $io->section('1. Attribuer, depuis la fiche');
            $adminSession = $this->login($adminEmail, self::PASSWORD);
            $fiche = '/admin/utilisateurs/' . $memberId;
            $html = $this->page($fiche, $adminSession);
            $grantToken = $this->formToken($html, $fiche . '/badges/attribuer');
            $this->check($io, $failures, 'la fiche porte « Attribuer » (connexion ' . $this->lastLogin . ')', $grantToken !== '');
            $this->check($io, $failures, 'le badge est dans la liste', str_contains($html, '<option value="' . $badgeId . '">'));
            $this->post($fiche . '/badges/attribuer', ['_token' => $grantToken, 'badge' => (string) $badgeId, 'reason' => '  '], $adminSession);
            $this->check($io, $failures, 'sans motif : refusé, rien n’est donné', !$held() && $this->inAnyLocale($this->page($fiche, $adminSession), 'badge_grants.reason_required', []));
            $this->post($fiche . '/badges/attribuer', ['_token' => $grantToken, 'badge' => (string) $badgeId, 'reason' => self::REASON], $adminSession);
            $this->check($io, $failures, '🔴 avec motif : détenu', $held());
            $row = $this->db->fetchAssociative('SELECT * FROM BADGE_GRANT WHERE userId = ? AND badgeId = ? ORDER BY id DESC LIMIT 1', [$memberId, $badgeId]);
            $this->check($io, $failures, 'le journal dit QUI et POURQUOI', $row !== false && (int) $row['grantedById'] === $adminId && $row['reason'] === self::REASON && $row['origin'] === 'manual');
            $this->check($io, $failures, '🔴 le lecteur OUVRE la machine', $opens());

            $memberSession = $this->login($email, self::PASSWORD);
            $profile = $this->page('/profil', $memberSession);
            $this->check($io, $failures, '« Mes badges » : « Attribué par ' . $adminName . ' le … : motif »', $this->inAnyLocale($profile, 'badge_grants.profile_granted_by_reason', [
                '%by%' => $adminName, '%date%' => (new \DateTimeImmutable())->format('d/m/Y'), '%reason%' => self::REASON,
            ]) || $this->inAnyLocale($profile, 'badge_grants.profile_granted_by_reason', [
                '%by%' => $adminName, '%date%' => (new \DateTimeImmutable('+1 day'))->format('d/m/Y'), '%reason%' => self::REASON,
            ]));

            $io->section('2. Retirer');
            $revokeToken = $this->formToken($this->page($fiche, $adminSession), $fiche . '/badges/retirer');
            $this->check($io, $failures, 'la fiche porte « Retirer », avec confirmation', $revokeToken !== '');
            $this->post($fiche . '/badges/retirer', ['_token' => $revokeToken, 'badge' => (string) $badgeId, 'reason' => 'Sonde S202 : retrait'], $adminSession);
            $this->check($io, $failures, '🔴 plus détenu', !$held());
            $this->check($io, $failures, '🔴 le lecteur REFUSE de nouveau', !$opens());
            $this->check($io, $failures, 'le retrait reste listé sur la fiche', str_contains($this->page($fiche, $adminSession), 'Sonde S202 : retrait'));
            $this->check($io, $failures, 'la ligne est fermée, pas effacée', (int) $this->db->fetchOne('SELECT COUNT(*) FROM BADGE_GRANT WHERE userId = ? AND badgeId = ? AND revokedById = ?', [$memberId, $badgeId, $adminId]) === 1);

            $io->section('3. Le verrou (avec témoin)');
            $this->probeLock($io, $failures, $adminId);
        } finally {
            $this->db->rollBack();
            $this->entityManager->clear();
            $this->tokens->setToken(null);
        }

        $io->section('4. Rien n’est resté');
        $after = $counts();
        foreach ($before as $table => $count) {
            $this->check($io, $failures, sprintf('%s : %d avant, %d après', $table, $count, $after[$table]), $after[$table] === $count);
        }
        $this->check($io, $failures, 'les badges détenus sont exactement ceux d’avant', $this->db->fetchAllNumeric('SELECT utilisateurId, badgeId FROM UTILISATEUR_BADGE ORDER BY 1, 2') === $heldBefore);

        if ($failures !== []) {
            $io->error(\count($failures) . ' assertion(s) en échec.');

            return Command::FAILURE;
        }
        $io->success('Sonde S202 verte. Transaction annulée.');

        return Command::SUCCESS;
    }

    /**
     * Une paire (personne, badge) dont la formation est VALIDÉE. Témoin : on
     * efface la ligne à la main, on sauve la progression → le badge revient.
     * Puis on la retire par `BadgeGrants` → il ne revient plus.
     *
     * @param list<string> $failures
     */
    private function probeLock(SymfonyStyle $io, array &$failures, int $adminId): void
    {
        $this->entityManager->clear();
        $pair = null;
        foreach ($this->entityManager->getRepository(UtilisateurBadge::class)->findAll() as $held) {
            $user = $held->getUtilisateur();
            $badge = $held->getBadge();
            $formation = $badge instanceof Badge ? $this->formations->findVisibleByBadge($badge) : null;
            if ($user instanceof Utilisateur && $formation !== null && $this->qualification->getStatus($formation, $user)['eligible']) {
                $progression = $this->entityManager->getRepository(Progression::class)->findOneBy(['utilisateur' => $user, 'formation' => $formation]);
                if ($progression instanceof Progression) {
                    $pair = [$user, $badge, $progression];
                    break;
                }
            }
        }
        if ($pair === null) {
            $this->check($io, $failures, 'une personne dont la formation est validée ET qui a une progression (aucune trouvée)', false);

            return;
        }
        [$user, $badge, $progression] = $pair;
        [$userId, $badgeId] = [(int) $user->getId(), (int) $badge->getId()];
        $io->writeln(sprintf('   compte #%d, badge « %s »', $userId, $badge->getNom()));
        $held = fn (): bool => (bool) $this->db->fetchOne('SELECT 1 FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?', [$userId, $badgeId]);
        $progressionId = (int) $progression->getId();
        // ⚠️ Tout vider avant chaque sauvegarde : une ligne effacée en SQL resterait
        // dans la carte d'identité de Doctrine, et l'abonné la croirait détenue.
        $touch = function () use ($progressionId): void {
            $this->entityManager->clear();
            $fresh = $this->entityManager->find(Progression::class, $progressionId);
            $fresh?->setDateEnd(new \DateTimeImmutable('-' . random_int(1, 999) . ' seconds'));
            $this->entityManager->flush();
        };

        $this->db->executeStatement('DELETE FROM UTILISATEUR_BADGE WHERE utilisateurId = ? AND badgeId = ?', [$userId, $badgeId]);
        $touch();
        $this->check($io, $failures, 'témoin : effacé sans retrait, il REVIENT à la sauvegarde suivante', $held());

        $this->entityManager->clear();
        $actor = $this->users->find($adminId);
        $user = $this->users->find($userId);
        $badge = $this->entityManager->find(Badge::class, $badgeId);
        $this->check($io, $failures, 'retiré par l’équipe', $actor instanceof Utilisateur && $user instanceof Utilisateur && $badge instanceof Badge && $this->grants->revoke($user, $badge, $actor, 'Sonde S202 : verrou'));
        $touch();
        $this->check($io, $failures, '🔴 la formation validée ne le redonne PAS', !$held());
        $this->check($io, $failures, 'et « Attribuer » le rend', $actor instanceof Utilisateur && $user instanceof Utilisateur && $badge instanceof Badge && $this->grants->grant($user, $badge, $actor, 'Sonde S202 : rendu') && $held() && !$this->grants->isRevoked($userId, $badgeId));
    }
}
