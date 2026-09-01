<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use App\UsageRights\AudienceResolver;
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
    public function __construct(private readonly TranslatorInterface $translator)
    {
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
    ): Response {
        $group = $groups->find($id);
        if ($group === null) {
            throw $this->createNotFoundException();
        }

        if ($request->isMethod('POST')) {
            return $this->handle($request, $groups, 'app_admin_group_edit', $id);
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
                $members[] = [
                    'user' => $user,
                    'stored' => isset($stored[(int) $user->getId()]),
                ];
                continue;
            }
            $candidates[] = $user;
        }

        return $this->render('site/admin-group-form.html.twig', [
            'group' => $group,
            'members' => $members,
            'candidates' => $candidates,
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
     * Les cinq écritures, toutes derrière le même jeton.
     *
     * ⚠️ **Chaque action porte un identifiant, pas une liste de champs** — même
     * raison que les suppressions de la fiche forfait : un `FormType` n'aurait
     * rien à y valider. La création et le renommage, eux, ont des champs, et ils
     * sont refusés au dépôt qui seul connaît les longueurs et l'unicité de clé.
     */
    private function handle(Request $request, UserGroupRepository $groups, string $route, ?int $id = null): Response
    {
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
                    $groups->addMember($request->request->getInt('id'), $request->request->getInt('user_id'));
                    $this->addFlash('success', $this->translator->trans('groups.member_added'));
                    break;

                case 'remove_member':
                    $groups->removeMember($request->request->getInt('id'), $request->request->getInt('user_id'));
                    $this->addFlash('success', $this->translator->trans('groups.member_removed'));
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
