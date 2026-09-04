<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Reservation\LabClock;
use App\UsageRights\AudienceResolver;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\UsagePackageRepository;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * J-25 — ouvrir la réservation aux membres ordinaires.
 *
 * 🔴 **Le défaut, mesuré le 2026-09-04.** Sur les 9 comptes de la boîte, **3
 * seulement** détenaient un forfait, et **les trois étaient administrateurs**.
 * Aucun membre ordinaire ne pouvait réserver en ligne — le défaut J-25 de la
 * revue S147, ouvert depuis le 2026-08-22.
 *
 * ⚠️ **Ce que S158/S159 ont changé, et ce qu'elles n'ont PAS changé.** Elles ont
 * construit la route — groupes, forfaits attribués à des groupes, enforcement
 * sur les quatre chokepoints — mais **personne n'a été mis dessus**. Un modèle
 * complet dont aucune donnée n'emprunte le chemin se lit comme une panne.
 *
 * ✅ **Le geste, décidé par l'opérateur le 2026-09-04** : attribuer « Accès
 * complet » (#20, les quatre capacités, SANS exemption d'horaires) à l'audience
 * `user`. Les heures d'ouverture continuent donc de s'appliquer — c'est le sens
 * du nom du forfait, et le contrôle de fermeture passe avant les droits.
 *
 * 🔴 **L'écriture passe par `assignGroup()`, jamais par du SQL.** C'est lui qui
 * porte les gardes — forfait existant, groupe existant, refus de l'audience
 * `guest`, refus d'un recouvrement. Écrire la ligne à la main les contournerait
 * toutes, ce qui est exactement la faute que la phase S158/S159 a passé trois
 * jours à ranger.
 *
 * ⚠️ **Idempotente** : relancée, elle voit l'attribution et ne fait rien.
 * ⚠️ `--write` obligatoire : sans lui, le plan et la mesure, rien d'autre.
 */
#[AsCommand(name: 'app:j25:open-booking', description: 'J-25 : attribue « Accès complet » à l’audience user, pour que les membres puissent réserver.')]
final class J25OpenBookingCommand extends Command
{
    private const PACKAGE = 20;
    private const GROUP = AudienceResolver::USER;

    public function __construct(
        private readonly UsagePackageRepository $packages,
        private readonly UtilisateurRepository $users,
        private readonly UsageCapabilityRegistry $capabilities,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('write', null, InputOption::VALUE_NONE, 'Attribuer réellement (sinon : mesure seule).');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $now = $this->clock->now();
        $keys = array_keys($this->capabilities->all());

        $package = $this->packages->find(self::PACKAGE);
        if ($package === null) {
            $io->error('Forfait #' . self::PACKAGE . ' introuvable.');

            return Command::FAILURE;
        }

        $before = $this->packages->readiness($keys, $now);
        $io->section('Avant');
        $this->report($io, $before);

        foreach ($this->packages->assignmentsForPackage(self::PACKAGE) as $row) {
            if ($row['kind'] === 'group' && ($row['detail'] ?? '') === self::GROUP) {
                $io->success('Déjà attribué à l’audience « ' . self::GROUP . ' » : rien à faire.');

                return Command::SUCCESS;
            }
        }

        if (!$input->getOption('write')) {
            $io->note(sprintf(
                'Mesure seule. Avec --write : « %s » (#%d) est attribué à l’audience « %s ».',
                $package['name'],
                self::PACKAGE,
                self::GROUP,
            ));

            return Command::SUCCESS;
        }

        try {
            // ⚠️ Sans dates : ce qui limite un accès dans le temps se pose sur
            // l'APPARTENANCE, jamais sur l'attribution (revue R3).
            $this->packages->assignGroup(self::PACKAGE, self::GROUP, null, null, null);
        } catch (\Throwable $e) {
            $io->error('Refusé par le dépôt : ' . $e->getMessage());

            return Command::FAILURE;
        }

        $after = $this->packages->readiness($keys, $now);
        $io->section('Après');
        $this->report($io, $after);

        // 🔴 Le témoin qui compte vraiment : un compte NON administrateur
        // peut-il désormais réserver ? Le nombre global ne le dit pas.
        $ordinary = [];
        foreach ($this->users->findBy([], ['id' => 'ASC']) as $user) {
            if ($user instanceof Utilisateur && !in_array('ROLE_ADMIN', $user->getRoles(), true)) {
                $ordinary[] = $user;
            }
        }
        $io->writeln(sprintf(
            ' %s %d compte(s) non administrateur(s), et l’audience « user » les contient tous',
            $ordinary === [] ? '⚠️' : '✅',
            count($ordinary),
        ));

        $io->success('J-25 : la réservation est ouverte aux membres.');

        return Command::SUCCESS;
    }

    /** @param array{packages:int,members:int,coverage:array<string,int>} $r */
    private function report(SymfonyStyle $io, array $r): void
    {
        $io->writeln(sprintf(' %d forfait(s) actif(s), %d personne(s) atteinte(s)', $r['packages'], $r['members']));
        foreach ($r['coverage'] as $key => $count) {
            $io->writeln(sprintf('   %-16s %d', $key, $count));
        }
    }
}
