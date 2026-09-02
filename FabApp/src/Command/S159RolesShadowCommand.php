<?php

namespace App\Command;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Symfony\Component\Console\Attribute\AsCommand;
use Symfony\Component\Console\Command\Command;
use Symfony\Component\Console\Input\InputInterface;
use Symfony\Component\Console\Output\OutputInterface;
use Symfony\Component\Console\Style\SymfonyStyle;

/**
 * S159b — la passe d'ombre de la fusion rôle → groupe.
 *
 * 🔴 **`getRoles()` est sur le chemin de la sécurité, et il vient de changer.**
 * Il rend désormais l'UNION de deux sources : la table `UTILISATEUR_ROLE`, comme
 * toujours, et l'appartenance aux groupes intégrés. Une union ne peut
 * qu'AJOUTER — personne ne perd un rôle — mais « ne peut qu'ajouter » est une
 * affirmation, et cette commande la transforme en mesure.
 *
 * Elle compare, compte par compte, ce que la seule table des rôles rendait et ce
 * que l'union rend. Trois issues :
 *
 *  - **identiques** : la fusion est neutre, ce qui est l'état attendu après le
 *    backfill de S158c — les groupes ne portent que ce que les rôles portaient ;
 *  - **l'union ajoute** : un groupe accorde un rôle que la table ne donnait pas.
 *    Ce n'est pas forcément une faute — c'est même le but à terme — mais ça doit
 *    être VU, ligne par ligne, avant d'être vécu ;
 *  - **l'union retire** : impossible par construction. Si ça arrive, quelque
 *    chose est faux dans `getRoles()`, et la commande sort en erreur.
 *
 * ⚠️ **Lecture seule**, aucune transaction : elle ne touche rien.
 *
 *   php bin/console app:s159:roles-shadow
 */
#[AsCommand(name: 'app:s159:roles-shadow', description: 'S159 : compare les rôles avant/après la fusion rôle → groupe, compte par compte.')]
final class S159RolesShadowCommand extends Command
{
    public function __construct(private readonly UtilisateurRepository $users)
    {
        parent::__construct();
    }

    protected function execute(InputInterface $input, OutputInterface $output): int
    {
        $io = new SymfonyStyle($input, $output);

        $identical = 0;
        $added = [];
        $lost = [];

        foreach ($this->users->findBy([], ['id' => 'ASC']) as $person) {
            if (!$person instanceof Utilisateur) {
                continue;
            }

            $before = $this->fromRoleTable($person);
            $after = $person->getRoles();
            sort($before);
            sort($after);

            if ($before === $after) {
                ++$identical;
                continue;
            }

            $gained = array_values(array_diff($after, $before));
            $dropped = array_values(array_diff($before, $after));
            $label = sprintf('#%d %s', $person->getId(), $person->getDisplayName());

            if ($dropped !== []) {
                $lost[] = $label . ' : perd ' . implode(', ', $dropped);
            }
            if ($gained !== []) {
                $added[] = $label . ' : gagne ' . implode(', ', $gained);
            }
        }

        $io->text(sprintf('%d compte(s) inchangé(s).', $identical));

        // 🔴 **Le palier de quota, parce que c'est la conséquence qui se voit le
        // moins.** `hasRoleNamed()` est désormais dérivée de `getRoles()`, donc
        // `BookingTier::forUser()` suit l'union — et un palier qui change, c'est
        // un plafond de réservation qui change, en silence. Neutre par
        // construction si les rôles le sont ; mesuré quand même.
        $tierMoves = [];
        foreach ($this->users->findBy([], ['id' => 'ASC']) as $person) {
            if (!$person instanceof Utilisateur) {
                continue;
            }
            $was = $this->tierFrom($this->fromRoleTable($person));
            $now = $this->tierFrom($person->getRoles());
            if ($was !== $now) {
                $tierMoves[] = sprintf('#%d %s : %s → %s', $person->getId(), $person->getDisplayName(), $was, $now);
            }
        }
        if ($tierMoves === []) {
            $io->text('Palier de réservation : aucun changement.');
        } else {
            $io->section(sprintf('⚠️ %d palier(s) de réservation changent', count($tierMoves)));
            $io->listing($tierMoves);
        }

        if ($added !== []) {
            $io->section(sprintf('%d compte(s) GAGNENT un rôle par leur groupe', count($added)));
            $io->listing($added);
            $io->note("Ce n'est pas une faute en soi — c'est ce que la fusion permet. Mais chaque ligne est un accès qui n'existait pas.");
        }

        if ($lost !== []) {
            $io->section(sprintf('🔴 %d compte(s) PERDENT un rôle', count($lost)));
            $io->listing($lost);
            $io->error("Impossible par construction : l'union ne retire rien. `getRoles()` est en faute.");

            return Command::FAILURE;
        }

        if ($added === []) {
            $io->success('La fusion est neutre : les deux sources rendent exactement les mêmes rôles.');
        }

        return Command::SUCCESS;
    }

    /**
     * Le palier qu'une liste de rôles donne, sans passer par l'entité — le
     * témoin doit rester indépendant de ce qu'il mesure.
     *
     * @param list<string> $roles
     */
    private function tierFrom(array $roles): string
    {
        foreach (['admin', 'staff', 'trainer'] as $tier) {
            if (\in_array(Utilisateur::securityRoleFor($tier), $roles, true)) {
                return $tier;
            }
        }

        return 'member';
    }

    /**
     * Ce que `getRoles()` rendait AVANT la fusion : la seule table des rôles.
     *
     * ⚠️ Recopié ici, et c'est le seul endroit du dépôt où une duplication de
     * cette boucle est justifiée — c'est le témoin, et un témoin qui appelle le
     * code mesuré ne mesure rien.
     *
     * @return list<string>
     */
    private function fromRoleTable(Utilisateur $user): array
    {
        $roles = ['ROLE_USER'];
        foreach ($user->getUtilisateurRoles() as $utilisateurRole) {
            $roleName = $utilisateurRole->getRole()?->getNom();
            if (!$roleName) {
                continue;
            }
            $roles[] = Utilisateur::securityRoleFor($roleName);
        }

        return array_values(array_unique($roles));
    }
}
