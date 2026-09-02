<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\Reservation\LabClock;
use App\UsageRights\AudienceResolver;
use App\UsageRights\UsagePackageRepository;
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
 * S159, étape 3 — convertir les attributions PERSONNELLES en groupes.
 *
 * L'écran n'offre plus que l'attribution à un groupe depuis l'étape 1 ; les
 * lignes personnelles écrites avant continuent d'accorder ce qu'elles accordent.
 * Cette commande les déplace, une par une, vers un groupe dédié.
 *
 * 🔴 **La DATE suit la personne, pas le groupe, et c'est tout l'enjeu.** Sur la
 * boîte, *OpenLab* est attribué à Alvaro **jusqu'au 2029-09-01**. Porter cette
 * date sur l'attribution de GROUPE la donnerait à tous ceux qu'on ajouterait
 * ensuite ; elle va donc sur **l'appartenance**, ce que la migration
 * `Version20260902160000` vient de rendre possible. C'est précisément pour ça que
 * l'appartenance datée était une dépendance dure de cette étape.
 *
 * 🔴 **Un groupe PAR FORFAIT, contenant exactement qui le tient aujourd'hui.**
 * Réutiliser un groupe existant — mettre « Accès complet » sur `staff`, par
 * exemple — donnerait le forfait à des gens qui ne l'ont pas. Une conversion
 * fidèle n'ajoute personne et n'en retire aucun.
 *
 * ✅ **Et elle se vérifie plutôt que de s'affirmer.** La commande photographie,
 * pour CHAQUE forfait, l'ensemble des personnes qu'il atteint ; elle convertit ;
 * elle rephotographie avec des résolveurs NEUFS ; et elle **annule tout** si un
 * seul ensemble a changé. Même discipline que le backfill de S158c.
 *
 * ⚠️ `--write` obligatoire : sans lui, le plan et rien d'autre.
 *
 *   php bin/console app:s159:convert-assignments
 *   php bin/console app:s159:convert-assignments --write
 */
#[AsCommand(name: 'app:s159:convert-assignments', description: 'S159 : déplace les attributions personnelles vers des groupes, sans changer qui a quoi.')]
final class S159ConvertAssignmentsCommand extends Command
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly UsagePackageRepository $packages,
        private readonly UserGroupRepository $groups,
        private readonly UtilisateurRepository $users,
        private readonly LabClock $clock,
    ) {
        parent::__construct();
    }

    protected function configure(): void
    {
        $this->addOption('write', null, InputOption::VALUE_NONE, 'Convertir réellement (sinon : plan seul).');
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);
        $write = (bool) $input->getOption('write');
        $now = $this->clock->now();

        // Le plan : chaque forfait, et ses attributions personnelles vivantes.
        $plan = [];
        foreach ($this->packages->findAll() as $package) {
            foreach ($this->packages->assignmentsForPackage($package['id']) as $assignment) {
                if ($assignment['kind'] !== 'member' || $assignment['userId'] === null) {
                    continue;
                }
                $plan[] = ['package' => $package, 'assignment' => $assignment];
            }
        }

        if ($plan === []) {
            $io->success("Rien à convertir : plus une seule attribution personnelle.");

            return Command::SUCCESS;
        }

        $io->section(sprintf('%d attribution(s) personnelle(s) à convertir', count($plan)));
        $io->table(['Forfait', 'Personne', 'Du', 'Jusqu’au', 'Groupe créé'], array_map(
            static fn (array $row): array => [
                $row['package']['name'],
                $row['assignment']['name'],
                $row['assignment']['validFrom'] ?? '—',
                $row['assignment']['validUntil'] ?? '—',
                $row['package']['name'],
            ],
            $plan,
        ));

        if (!$write) {
            $io->note('Plan seul. Relancer avec --write pour convertir.');

            return Command::SUCCESS;
        }

        $before = $this->reachSnapshot($now);

        $connection = $this->em->getConnection();
        $connection->beginTransaction();
        try {
            foreach ($plan as $row) {
                $packageId = (int) $row['package']['id'];
                $userId = (int) $row['assignment']['userId'];

                // Le groupe porte le nom du forfait : « qui tient ce forfait ».
                $groupId = $this->groups->create(
                    $row['package']['name'],
                    'Créé par la conversion des attributions personnelles (S159).',
                );
                $group = $this->groups->find($groupId);
                if ($group === null) {
                    throw new \RuntimeException('Groupe créé puis introuvable.');
                }

                // 🔴 Les dates de l'attribution deviennent celles de
                // l'APPARTENANCE : elles concernent cette personne, pas le groupe.
                $this->groups->addMember(
                    $groupId,
                    $userId,
                    $this->momentOf($row['assignment']['validFrom']),
                    $this->momentOf($row['assignment']['validUntil']),
                );

                // ⚠️ L'attribution de groupe, elle, est SANS dates : le groupe n'a
                // pas de fin, ce sont ses membres qui en ont une.
                $this->packages->assignGroup($packageId, $group['key'], null, null, null);

                // Et seulement maintenant : la ligne personnelle est révoquée.
                $this->packages->revoke((int) $row['assignment']['id'], null);
            }

            $after = $this->reachSnapshot($now);

            $drift = [];
            foreach ($before as $packageId => $reached) {
                $now2 = $after[$packageId] ?? [];
                if ($reached !== $now2) {
                    $drift[] = sprintf(
                        'forfait #%d : %s → %s',
                        $packageId,
                        $reached === [] ? '(personne)' : implode(',', $reached),
                        $now2 === [] ? '(personne)' : implode(',', $now2),
                    );
                }
            }

            if ($drift !== []) {
                $connection->rollBack();
                $io->error('La conversion CHANGERAIT qui a quoi. Rien n’a été écrit.');
                $io->listing($drift);

                return Command::FAILURE;
            }

            $connection->commit();
        } catch (\Throwable $e) {
            if ($connection->isTransactionActive()) {
                $connection->rollBack();
            }
            $io->error('Conversion interrompue, rien n’a été écrit : ' . $e->getMessage());

            return Command::FAILURE;
        }

        $io->success(sprintf(
            '%d attribution(s) converties, et les %d forfaits atteignent exactement les mêmes personnes.',
            count($plan),
            count($before),
        ));

        return Command::SUCCESS;
    }

    /**
     * Pour chaque forfait, l'ensemble TRIÉ des personnes qu'il atteint.
     *
     * 🔴 **Résolveur NEUF à chaque appel.** `AudienceResolver` mémoïse par
     * instance : réutiliser celui d'avant l'écriture rendrait la photo d'avant, et
     * la vérification se prouverait elle-même. Même piège que le backfill.
     *
     * ⚠️ C'est la même question que pose le filtre « droit d'usage » de la liste
     * des utilisateurs — les deux chemins d'attribution, personnel et collectif —
     * donc le témoin mesure ce que l'écran montre.
     *
     * @return array<int, list<int>>
     */
    private function reachSnapshot(\DateTimeImmutable $at): array
    {
        $resolver = new AudienceResolver($this->em->getConnection(), new UserGroupSchema($this->em->getConnection()));
        $people = array_values(array_filter(
            $this->users->findBy([], ['id' => 'ASC']),
            static fn ($u): bool => $u instanceof Utilisateur && $u->getId() !== null,
        ));
        $resolver->primeFor($people);

        $out = [];
        foreach ($this->packages->findAll() as $package) {
            $reach = $this->packages->reachOf((int) $package['id'], $at);
            $direct = array_flip($reach['userIds']);
            $keys = array_flip($reach['groupKeys']);

            $ids = [];
            foreach ($people as $person) {
                $id = (int) $person->getId();
                if (isset($direct[$id])) {
                    $ids[] = $id;
                    continue;
                }
                foreach ($resolver->keysFor($person) as $key) {
                    if (isset($keys[$key])) {
                        $ids[] = $id;
                        break;
                    }
                }
            }
            sort($ids);
            $out[(int) $package['id']] = $ids;
        }

        return $out;
    }

    /** ⚠️ Une chaîne de la base, relue telle quelle : elle est déjà dans le fuseau du labo. */
    private function momentOf(?string $raw): ?\DateTimeImmutable
    {
        if ($raw === null || trim($raw) === '') {
            return null;
        }

        try {
            return new \DateTimeImmutable($raw, $this->clock->zone());
        } catch (\Throwable) {
            return null;
        }
    }
}
