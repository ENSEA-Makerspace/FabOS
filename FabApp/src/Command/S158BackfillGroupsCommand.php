<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\UsageRights\AudienceResolver;
use App\Reservation\LabClock;
use App\UsageRights\UserGroupRepository;
use App\UsageRights\UserGroupSchema;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Input\InputOption;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S158c — écrire les appartenances que les RÔLES produisent déjà.
 *
 * 🔴 **C'est l'étape « expand », et elle ne doit RIEN changer.**
 * `AudienceResolver` rend l'UNION de trois sources : les lignes stockées, les
 * rôles, et l'audience résolue `user`. Écrire une ligne pour une appartenance
 * qu'un rôle produit déjà est donc, par construction, sans effet sur la réponse.
 * C'est tout l'argument de sûreté du plan — et un argument de sûreté qui n'est
 * qu'affirmé n'en est pas un.
 *
 * ✅ **Donc il est EXÉCUTÉ, pas affirmé.** La commande photographie les audiences
 * de chaque compte AVANT, écrit, rephotographie APRÈS avec un résolveur neuf, et
 * **annule tout si un seul jeu de clés a bougé**. Une installation où le backfill
 * changerait quelque chose est une installation où il ne doit pas passer.
 *
 * ⚠️ **`--write` est obligatoire.** Sans lui la commande montre le plan et
 * n'écrit rien : c'est une écriture sur des données vivantes, elle se regarde
 * avant de se lancer.
 *
 * ⚠️ **Ce qu'elle n'écrit PAS.** Les audiences virtuelles (`user`, `guest`) :
 * elles se résolvent depuis le compte et n'ont jamais de ligne — leur en donner
 * une créerait une appartenance que personne ne lit, et pire, qui cesserait de
 * suivre le compte le jour où il est désactivé.
 *
 * 🅿️ **Et elle ne retire rien.** Le « contract » — sortir l'amorçage par les
 * rôles de `AudienceResolver::compute()` — est une étape SÉPARÉE, qui demande
 * une passe d'ombre et une décision de l'opérateur sur ce que devient le lien
 * rôle → groupe. Tant qu'elle n'a pas eu lieu, les deux moitiés cohabitent et
 * l'union les couvre.
 *
 *   php bin/console app:s158:backfill-groups            # le plan, sans écrire
 *   php bin/console app:s158:backfill-groups --write    # écrit, et vérifie
 */
#[AsCommand(name: 'app:s158:backfill-groups', description: 'S158c : écrit les appartenances que les rôles produisent déjà, sans changer une seule réponse.')]
final class S158BackfillGroupsCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly UtilisateurRepository $users,
        private readonly UserGroupRepository $groups,
        private readonly AudienceResolver $audiences,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('write', null, InputOption::VALUE_NONE, 'Écrire réellement les lignes (sinon : plan seul).');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $write = (bool) $input->getOption('write');

        $groupsByKey = [];
        foreach ($this->groups->all() as $group) {
            $groupsByKey[$group['key']] = $group;
        }
        if ($groupsByKey === []) {
            $io->error("Aucun groupe : l'installation n'a pas la migration S133b.");

            return Command::FAILURE;
        }

        $people = $this->users->findBy([], ['id' => 'ASC']);
        $this->audiences->primeFor($people);

        // Ce qui manque : une clé qu'un rôle produit, sur un groupe RÉEL, sans
        // ligne pour la porter.
        $plan = [];
        foreach ($people as $person) {
            if (!$person instanceof Utilisateur || $person->getId() === null) {
                continue;
            }
            $stored = [];
            foreach ($this->groups->storedGroupIdsFor((int) $person->getId()) as $groupId) {
                $stored[$groupId] = true;
            }
            foreach ($this->audiences->keysFor($person) as $key) {
                $group = $groupsByKey[$key] ?? null;
                if ($group === null || $group['virtual'] || isset($stored[$group['id']])) {
                    continue;
                }
                $plan[] = ['user' => $person, 'group' => $group];
            }
        }

        if ($plan === []) {
            $io->success('Rien à écrire : chaque appartenance produite par un rôle a déjà sa ligne.');

            return Command::SUCCESS;
        }

        $io->section(sprintf('%d ligne(s) à écrire', count($plan)));
        $io->table(['Compte', 'Groupe'], array_map(
            static fn (array $row): array => [$row['user']->getDisplayName() . ' (#' . $row['user']->getId() . ')', $row['group']['label']],
            $plan,
        ));

        if (!$write) {
            $io->note('Plan seul. Relancer avec --write pour écrire.');

            return Command::SUCCESS;
        }

        // 🔴 **La photo AVANT, sur laquelle tout repose.**
        $before = $this->snapshot($people);

        $connection = $this->em->getConnection();
        $connection->beginTransaction();
        try {
            foreach ($plan as $row) {
                $this->groups->addMember($row['group']['id'], (int) $row['user']->getId());
            }

            // ⚠️ **Un résolveur NEUF.** Celui injecté a déjà répondu pour ces
            // comptes et rendrait sa réponse mémoïsée, c'est-à-dire la photo
            // d'avant l'écriture — la vérification se prouverait elle-même.
            // 🔴 Deux arguments depuis S159g, plus l'horloge du labo : cette
            // ligne n'en passait qu'UN et la commande aurait planté au lieu de
            // vérifier — trouvé le 2026-09-03, après que le backfill a tourné.
            $after = $this->snapshot($people, new AudienceResolver($connection, new UserGroupSchema($connection), $this->clock));

            $drift = [];
            foreach ($before as $userId => $keys) {
                if (($after[$userId] ?? null) !== $keys) {
                    $drift[] = sprintf('#%d : %s → %s', $userId, implode(',', $keys), implode(',', $after[$userId] ?? []));
                }
            }

            if ($drift !== []) {
                $connection->rollBack();
                $io->error('Le backfill CHANGERAIT des réponses. Rien n’a été écrit.');
                $io->listing($drift);

                return Command::FAILURE;
            }

            $connection->commit();
        } catch (\Throwable $e) {
            if ($connection->isTransactionActive()) {
                $connection->rollBack();
            }
            $io->error('Écriture interrompue, rien n’a été écrit : ' . $e->getMessage());

            return Command::FAILURE;
        }

        $io->success(sprintf(
            '%d ligne(s) écrite(s), et les audiences des %d comptes sont identiques au jeton près.',
            count($plan),
            count($people),
        ));

        return Command::SUCCESS;
    }

    /**
     * Les clés d'audience de chaque compte, triées — la forme comparable.
     *
     * ⚠️ Trié : `keysFor()` renvoie dans l'ordre où les sources ont parlé, et
     * après le backfill une clé arrive par une autre source. Comparer sans trier
     * signalerait une dérive là où il n'y a qu'un changement d'ordre.
     *
     * @param list<Utilisateur> $people
     * @return array<int, list<string>>
     */
    private function snapshot(array $people, ?AudienceResolver $resolver = null): array
    {
        $resolver ??= $this->audiences;
        $resolver->primeFor($people);

        $out = [];
        foreach ($people as $person) {
            if (!$person instanceof Utilisateur || $person->getId() === null) {
                continue;
            }
            $keys = $resolver->keysFor($person);
            sort($keys);
            $out[(int) $person->getId()] = $keys;
        }

        return $out;
    }
}
