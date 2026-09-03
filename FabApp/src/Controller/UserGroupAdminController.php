<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\UsageRights\AudienceResolver;
use App\Service\SiteSettingService;
use App\UsageRights\UsagePackageRepository;
use App\UsageRights\UserGroupRepository;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * LES GROUPES — l'écran qui manquait (S158a).
 *
 * 🔴 **Ce que la mesure disait avant cet écran.** `USER_GROUP_MEMBER` n'était
 * écrit par rien : la migration semait les sept intégrés et aucune surface ne
 * permettait de créer un groupe ni d'y mettre quelqu'un. Le formulaire
 * « attribuer un forfait à un groupe » existait donc au-dessus d'un modèle que
 * personne ne pouvait remplir.
 *
 * 🔴 **Et la difficulté de cet écran est de ne pas mentir sur l'appartenance.**
 * Elle est l'UNION de trois choses, et `AudienceResolver` est le seul à savoir
 * la calculer :
 *
 *   1. les lignes de `USER_GROUP_MEMBER` — ce que cet écran écrit ;
 *   2. les **rôles** — `ROLE_STAFF` met dans `staff` sans aucune ligne ;
 *   3. l'audience résolue **`user`**, qui vaut « tout compte actif » et n'est
 *      écrite nulle part par construction.
 *
 * Un écran qui n'afficherait que (1) annoncerait « Staff : 0 membre » pendant que
 * le groupe couvre la moitié du lab. Il montre donc l'appartenance **effective**,
 * et dit de chaque personne PAR OÙ elle est là. ⚠️ Et il n'offre pas de bouton
 * « retirer » à quelqu'un qui est là par son rôle : ce bouton ne pourrait rien
 * faire, et un contrôle qui ne fait rien est la règle n°3 de la maison.
 */
#[Route('/admin/groupes')]
#[IsGranted('ROLE_ADMIN')]
final class UserGroupAdminController extends AbstractController
{
    public function __construct(
        private readonly TranslatorInterface $translator,
        private readonly SiteSettingService $settings,
    ) {
    }

    #[Route('', name: 'app_admin_groups', methods: ['GET', 'POST'])]
    public function index(
        Request $request,
        UserGroupRepository $groups,
        UtilisateurRepository $users,
        AudienceResolver $audiences,
    ): Response {
        if ($request->isMethod('POST')) {
            return $this->handle($request, $groups, 'app_admin_groups');
        }

        // ⚠️ **L'effectif se compte sur les personnes, pas sur les lignes.** Une
        // seule requête grâce à `primeFor()`, puis le résolveur répond de
        // mémoire — sans quoi ce serait une requête par compte.
        $rows = $users->findForAdminFilters(['q' => '', 'statut' => '', 'role' => '']);
        $audiences->primeFor($rows);
        $effective = [];
        foreach ($rows as $user) {
            foreach ($audiences->keysFor($user) as $key) {
                $effective[$key] = ($effective[$key] ?? 0) + 1;
            }
        }

        return $this->render('site/admin-groups.html.twig', [
            'rows' => $groups->all(),
            'effective' => $effective,
            'accounts' => count($rows),
        ]);
    }

    #[Route('/{id<\d+>}', name: 'app_admin_group_edit', methods: ['GET', 'POST'])]
    public function edit(
        int $id,
        Request $request,
        UserGroupRepository $groups,
        UtilisateurRepository $users,
        AudienceResolver $audiences,
        UsagePackageRepository $packages,
    ): Response {
        $group = $groups->find($id);
        if ($group === null) {
            throw $this->createNotFoundException();
        }

        if ($request->isMethod('POST')) {
            return $this->handle($request, $groups, 'app_admin_group_edit', $id, $packages);
        }

        $all = $users->findForAdminFilters(['q' => '', 'statut' => '', 'role' => '']);
        $audiences->primeFor($all);
        $stored = array_flip($groups->storedMemberIds($id));

        // 🔴 **La liste dit PAR OÙ chacun est là.** « Ajouté ici » se retire d'un
        // bouton ; « par son rôle » ne se retire que sur la fiche du membre, et
        // l'écran le dit au lieu d'offrir un bouton inerte.
        $members = [];
        $candidates = [];
        foreach ($all as $user) {
            $isMember = in_array($group['key'], $audiences->keysFor($user), true);
            if ($isMember) {
                // ⚠️ **Plus de distinction de source depuis S159h.** Elle avait un
                // sens tant que les rôles étaient une source indépendante ; ils
                // dérivent désormais des appartenances, donc pour un groupe non
                // virtuel la ligne est la seule raison — et elle se retire.
                $members[] = [
                    'user' => $user,
                    'stored' => isset($stored[(int) $user->getId()]),
                ];
                continue;
            }
            $candidates[] = $user;
        }

        // 🔴 **Les forfaits du groupe, et c'est ici qu'ils s'écrivent
        // désormais** (revue de design, 2026-09-03). « Qui est dans ce groupe »
        // et « ce que ce groupe donne » sont les deux moitiés d'une même
        // question ; les séparer sur deux écrans obligeait à retenir un nom de
        // groupe en traversant la page des forfaits.
        $held = $packages->assignmentsForGroup($id);
        $taken = array_flip(array_map(static fn (array $row): int => $row['packageId'], $held));
        // ⚠️ **`guest` ne reçoit RIEN, et le menu ne doit pas le proposer.**
        // `assignGroup()` refuse l'audience anonyme — elle n'a pas de compte, donc
        // un forfait ne peut rien lui accorder. Proposer le choix quand même
        // afficherait un formulaire dont chaque envoi est refusé : une affordance
        // morte, que l'ancien sélecteur de la fiche du forfait filtrait déjà pour
        // cette raison exacte. Perdu au déménagement, remis ici.
        $offer = ($group['key'] ?? '') === AudienceResolver::GUEST ? [] : array_values(array_filter(
            $packages->findAll(),
            static fn (array $package): bool => !isset($taken[(int) $package['id']]),
        ));

        return $this->render('site/admin-group-form.html.twig', [
            'group' => $group,
            'members' => $members,
            'candidates' => $candidates,
            'bundles' => $held,
            // ⚠️ Ce qu'on peut encore ajouter, pas le catalogue entier : proposer
            // un forfait déjà attribué mènerait au refus de recouvrement du
            // dépôt, c'est-à-dire à un choix qui ne peut qu'échouer.
            'offerable' => $offer,
        ]);
    }

    /**
     * L'appartenance vue depuis la FICHE DU MEMBRE (S158b) — les mêmes lignes,
     * par l'autre bout.
     *
     * ⚠️ **Même dépôt, mêmes gardes, même jeton.** On pense « dans quels groupes
     * est cette personne » aussi souvent que « qui est dans ce groupe » ; ce sont
     * deux vues, pas deux surfaces d'écriture. Un second chemin d'écriture aurait
     * ses propres refus, et celui des deux qu'on oublie de corriger est celui qui
     * laisse passer.
     *
     * 🔴 **La redirection est FIXE.** Elle vise `app_admin_user_detail` avec l'id
     * de la route, jamais une cible venue de la requête : un `?back=` recopié
     * dans un `redirect()` est une redirection ouverte, et cet écran est derrière
     * une session d'administrateur.
     */
    #[Route('/membre/{userId<\d+>}', name: 'app_admin_group_member', methods: ['POST'])]
    public function member(int $userId, Request $request, UserGroupRepository $groups): Response
    {
        if (!$this->isCsrfTokenValid('admin_groups', (string) $request->request->get('_token'))) {
            $this->addFlash('error', $this->translator->trans('groups.csrf_error'));

            return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
        }

        try {
            $groupId = $request->request->getInt('id');
            if ($request->request->get('action') === 'remove_member') {
                $groups->removeMember($groupId, $userId);
                $this->addFlash('success', $this->translator->trans('groups.member_removed'));
            } else {
                $groups->addMember($groupId, $userId);
                $this->addFlash('success', $this->translator->trans('groups.member_added'));
            }
        } catch (\Throwable $e) {
            $this->addFlash('error', $e->getMessage());
        }

        return $this->redirectToRoute('app_admin_user_detail', ['id' => $userId]);
    }

    /**
     * Une date de formulaire, dans le fuseau du LABO (S159g).
     *
     * 🔴 **Jamais `new \DateTimeImmutable($raw)` tout seul.** Sans fuseau
     * explicite, PHP prend le sien — UTC ici — et « jusqu'au 30 juin » devient
     * une expiration décalée de deux heures. C'est la même leçon que
     * `UsageRightsAdminController::date()` pour les attributions, et le piège
     * n°4 de la reprise.
     *
     * ⚠️ Vide rend `null`, qui veut dire « sans limite ». Une saisie illisible
     * aussi : mieux vaut une appartenance sans limite qu'une exception sur un
     * écran d'administration — et la ligne se corrige.
     */
    private function momentOf(mixed $raw): ?\DateTimeImmutable
    {
        $value = trim((string) $raw);
        if ($value === '') {
            return null;
        }

        try {
            return new \DateTimeImmutable($value, new \DateTimeZone($this->settings->getTimezone()));
        } catch (\Throwable) {
            return null;
        }
    }

    /**
     * Les cinq écritures, toutes derrière le même jeton.
     *
     * ⚠️ **Chaque action porte un identifiant, pas une liste de champs** — même
     * raison que les suppressions de la fiche forfait : un `FormType` n'aurait
     * rien à y valider. La création et le renommage, eux, ont des champs, et ils
     * sont refusés au dépôt qui seul connaît les longueurs et l'unicité de clé.
     */
    private function handle(
        Request $request,
        UserGroupRepository $groups,
        string $route,
        ?int $id = null,
        ?UsagePackageRepository $packages = null,
    ): Response {
        $back = $id === null ? [] : ['id' => $id];
        if (!$this->isCsrfTokenValid('admin_groups', (string) $request->request->get('_token'))) {
            $this->addFlash('error', $this->translator->trans('groups.csrf_error'));

            return $this->redirectToRoute($route, $back);
        }

        $action = (string) $request->request->get('action');
        try {
            switch ($action) {
                case 'create':
                    $new = $groups->create(
                        (string) $request->request->get('label', ''),
                        (string) $request->request->get('description', ''),
                    );
                    $this->addFlash('success', $this->translator->trans('groups.created'));

                    return $this->redirectToRoute('app_admin_group_edit', ['id' => $new]);

                case 'rename':
                    $groups->rename(
                        $request->request->getInt('id'),
                        (string) $request->request->get('label', ''),
                        (string) $request->request->get('description', ''),
                    );
                    $this->addFlash('success', $this->translator->trans('groups.saved'));
                    break;

                case 'delete':
                    $groups->delete($request->request->getInt('id'));
                    $this->addFlash('success', $this->translator->trans('groups.deleted'));

                    // Le groupe n'existe plus : rester sur sa page rendrait un 404.
                    return $this->redirectToRoute('app_admin_groups');

                case 'add_member':
                    // ⚠️ **Les deux dates sont facultatives, et vides veulent dire
                    // « sans limite »** — pas « maintenant », pas « jamais ». C'est
                    // ce que sont toutes les appartenances écrites jusqu'ici, et
                    // le cas de loin le plus courant.
                    $groups->addMember(
                        $request->request->getInt('id'),
                        $request->request->getInt('user_id'),
                        $this->momentOf($request->request->get('valid_from')),
                        $this->momentOf($request->request->get('valid_until')),
                    );
                    $this->addFlash('success', $this->translator->trans('groups.member_added'));
                    break;

                case 'remove_member':
                    $groups->removeMember($request->request->getInt('id'), $request->request->getInt('user_id'));
                    $this->addFlash('success', $this->translator->trans('groups.member_removed'));
                    break;

                case 'assign_bundle':
                case 'revoke_bundle':
                    // ⚠️ **Un refus PLUTÔT qu'un succès silencieux.** Ces deux
                    // actions n'existent que sur la fiche du groupe ; postées
                    // ailleurs, `$packages` est absent, et un `?->` qui ne fait
                    // rien suivi d'un flash « enregistré » serait un mensonge.
                    if ($packages === null) {
                        $this->addFlash('error', $this->translator->trans('groups.unknown_action'));
                        break;
                    }
                    if ($action === 'revoke_bundle') {
                        // 🔴 **L'attribution doit appartenir À CE GROUPE.**
                        // `revoke()` ne connaît que l'identifiant : sans ce
                        // contrôle, un identifiant posté depuis la fiche d'un
                        // groupe retirerait le forfait d'un AUTRE, et l'écran
                        // annoncerait un succès. L'écran est derrière une session
                        // d'administrateur et un jeton, mais « seul un admin peut
                        // le faire » n'a jamais voulu dire « il l'a voulu ».
                        $assignmentId = $request->request->getInt('assignment_id');
                        $groupId = $request->request->getInt('id');
                        $owned = false;
                        foreach ($packages->assignmentsForGroup($groupId) as $held) {
                            if ((int) $held['id'] === $assignmentId) {
                                $owned = true;
                                break;
                            }
                        }
                        if (!$owned) {
                            // ⚠️ Un refus PLUTÔT qu'un succès muet : la ligne a pu
                            // être révoquée depuis un autre onglet, et dire
                            // « retiré » alors que rien n'a bougé est un mensonge.
                            $this->addFlash('error', $this->translator->trans('groups.bundle_not_here'));
                            break;
                        }
                        $packages->revoke($assignmentId, $this->getUser() instanceof Utilisateur ? $this->getUser()->getId() : null);
                        $this->addFlash('success', $this->translator->trans('groups.bundle_revoked'));
                        break;
                    }
                    // ⚠️ **Sans dates, et volontairement.** Ce qui limite un
                    // accès dans le temps se pose sur l'APPARTENANCE, où cela
                    // concerne une personne (revue R3). Un forfait attribué à un
                    // groupe l'est tant que l'attribution vit.
                    $packages->assignGroup(
                        $request->request->getInt('package_id'),
                        (string) ($groups->find($request->request->getInt('id'))['key'] ?? ''),
                        null,
                        null,
                        $this->getUser() instanceof Utilisateur ? $this->getUser()->getId() : null,
                    );
                    $this->addFlash('success', $this->translator->trans('groups.bundle_assigned'));
                    break;

                default:
                    $this->addFlash('error', $this->translator->trans('groups.unknown_action'));
            }
        } catch (\Throwable $e) {
            $this->addFlash('error', $e->getMessage());
        }

        return $this->redirectToRoute($route, $back);
    }
}
