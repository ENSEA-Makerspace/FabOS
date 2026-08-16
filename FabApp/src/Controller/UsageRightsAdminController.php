<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\UtilisateurRepository;
use App\UsageRights\UsagePackageRepository;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\AudienceResolver;
use App\UsageRights\UsageRightsShadow;
use App\Service\SiteSettingService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Http\Attribute\IsGranted;
use Symfony\Contracts\Translation\TranslatorInterface;

#[Route('/admin/usage-rights')]
#[IsGranted('ROLE_ADMIN')]
final class UsageRightsAdminController extends AbstractController
{
    public function __construct(
        private readonly TranslatorInterface $translator,
        private readonly SiteSettingService $settings,
    ) {
    }
    #[Route('', name: 'app_admin_usage_rights', methods: ['GET'])]
    public function index(UsagePackageRepository $packages): Response
    {
        $rows = $packages->findAll();
        $active = count(array_filter($rows, static fn (array $package): bool => $package['active']));

        return $this->render('site/admin-usage-rights.html.twig', [
            'packages' => $rows,
            'tiles' => [
                ['label' => 'usage_rights.all_packages', 'label_is_key' => true, 'count' => count($rows), 'category' => 'all'],
                ['label' => 'usage_rights.active_packages', 'label_is_key' => true, 'count' => $active, 'category' => 'active'],
                ['label' => 'usage_rights.inactive_packages', 'label_is_key' => true, 'count' => count($rows) - $active, 'category' => 'inactive'],
            ],
        ]);
    }

    /**
     * Grants v2, side by side with what is actually in force (S133b).
     *
     * ⚠️ **This page decides nothing and is the entire point.** The roadmap's
     * requirement is a shadow that is "visible et explicable" *before* S134 turns
     * any of it on. What makes it explicable rather than a wall of mismatches is
     * that every row is classified: agreement, admin recovery, "the live yes comes
     * from enforcement being off", and the one that matters —
     * `shadow_would_deny`, a member who would lose access the day enforcement is
     * switched on. That list is the work S134 has to finish before it starts.
     *
     * ⚠️ The counts are over **the members shown**, and the page says so. A
     * summary computed from one page and printed as a total is how a fabricated
     * number once framed a whole session.
     */
    #[Route('/shadow', name: 'app_admin_usage_rights_shadow', methods: ['GET'])]
    public function shadow(Request $request, UtilisateurRepository $users, UsageRightsShadow $shadow, AudienceResolver $audiences): Response
    {
        // Bounded on purpose: this builds one verdict pair per member per
        // capability, and an unbounded sweep of an installation with thousands of
        // accounts is a page that times out rather than a page that informs.
        $limit = min(max($request->query->getInt('limit', 50), 10), 200);
        $members = array_slice($users->findBy([], ['lastName' => 'ASC', 'firstName' => 'ASC']), 0, $limit);

        $rows = [];
        foreach ($members as $member) {
            $verdicts = $shadow->forUser($member);
            $rows[] = [
                'user' => $member,
                'audiences' => $shadow->audiencesOf($member),
                'verdicts' => $verdicts,
                // The row is worth the operator's attention only if something on
                // it would change. Sorting on this is what keeps the page a
                // worklist instead of a directory.
                'attention' => \count(array_filter($verdicts, static fn (array $v): bool => $v['status'] === 'shadow_would_deny')),
            ];
        }
        usort($rows, static fn (array $a, array $b): int => $b['attention'] <=> $a['attention']);

        return $this->render('site/admin-usage-rights-shadow.html.twig', [
            'rows' => $rows,
            'summary' => $shadow->summary($members),
            'shown' => \count($members),
            'totalMembers' => \count($users->findAll()),
            'groups' => $audiences->catalogue(),
            'ready' => $shadow->isReady(),
            'enforced' => $this->settings->isUsageRightsEnforced(),
        ]);
    }

    #[Route('/new', name: 'app_admin_usage_rights_new', methods: ['GET', 'POST'])]
    public function new(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities): Response
    {
        return $this->form($request, $packages, $features, $capabilities);
    }

    #[Route('/{id<\d+>}/edit', name: 'app_admin_usage_rights_edit', methods: ['GET', 'POST'])]
    public function edit(int $id, Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, UtilisateurRepository $users, SiteSettingService $settings): Response
    {
        $package = $packages->find($id);
        if ($package === null) {
            throw $this->createNotFoundException();
        }

        if ($request->isMethod('POST') && $request->request->get('action') === 'assign') {
            if (!$this->isCsrfTokenValid('usage_package_assign_' . $id, (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));
            } else {
                $member = $users->find($request->request->getInt('user_id'));
                try {
                    if (!$member instanceof Utilisateur) {
                        throw new \InvalidArgumentException($this->translator->trans('usage_rights.member_required'));
                    }
                    $zone = new \DateTimeZone($settings->getTimezone());
                    $from = $this->date($request->request->get('valid_from'), $zone);
                    $until = $this->date($request->request->get('valid_until'), $zone);
                    $actor = $this->getUser();
                    $packages->assign($id, $member, $from, $until, $actor instanceof Utilisateur ? $actor->getId() : null);
                    $this->addFlash('success', $this->translator->trans('usage_rights.assignment_created'));
                } catch (\Throwable $e) {
                    $this->addFlash('error', $e->getMessage());
                }
            }

            return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
        }

        if ($request->isMethod('POST') && $request->request->get('action') === 'revoke') {
            if ($this->isCsrfTokenValid('usage_package_revoke_' . $id, (string) $request->request->get('_token'))) {
                $actor = $this->getUser();
                $packages->revoke($request->request->getInt('assignment_id'), $actor instanceof Utilisateur ? $actor->getId() : null);
                $this->addFlash('success', $this->translator->trans('usage_rights.assignment_revoked'));
            }
            return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
        }

        return $this->form($request, $packages, $features, $capabilities, $package, $users);
    }

    /** @param array{id:int,name:string,description:string,active:bool,features:list<string>}|null $package */
    private function form(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, ?array $package = null, ?UtilisateurRepository $users = null): Response
    {
        $available = $capabilities->all();
        $enabled = array_filter($available, static fn ($capability): bool => $features->isEnabled($capability->featureKey));
        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('usage_package_form_' . ($package['id'] ?? 'new'), (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));
            } else {
                $fullAccess = $request->request->getBoolean('full_access');
                $selected = array_values(array_intersect(array_map('strval', (array) $request->request->all('features')), array_keys($available)));
                // A disabled site feature is suspended, not silently erased when
                // an operator edits the package description. Explicitly unticking
                // it while enabled remains the removal path.
                foreach (($package['features'] ?? []) as $existing) {
                    if (isset($available[$existing]) && !isset($enabled[$existing]) && !in_array($existing, $selected, true)) {
                        $selected[] = $existing;
                    }
                }
                try {
                    $id = $packages->save(
                        $package['id'] ?? null,
                        (string) $request->request->get('name'),
                        (string) $request->request->get('description'),
                        $request->request->getBoolean('active'),
                        $fullAccess,
                        $selected,
                    );
                    $this->addFlash('success', $this->translator->trans('usage_rights.package_saved'));
                    return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
                } catch (\Throwable $e) {
                    $this->addFlash('error', $e->getMessage());
                }
            }
        }

        return $this->render('site/admin-usage-package-form.html.twig', [
            'package' => $package ?? ['name' => '', 'description' => '', 'active' => true, 'fullAccess' => false, 'features' => []],
            'availableFeatures' => $available,
            'enabledFeatures' => array_keys($enabled),
            'assignments' => $package !== null ? $packages->assignmentsForPackage($package['id']) : [],
            'users' => $users?->findBy([], ['lastName' => 'ASC', 'firstName' => 'ASC']) ?? [],
        ]);
    }

    private function date(mixed $raw, \DateTimeZone $zone): ?\DateTimeImmutable
    {
        $value = trim((string) $raw);
        return $value === '' ? null : new \DateTimeImmutable($value, $zone);
    }
}
