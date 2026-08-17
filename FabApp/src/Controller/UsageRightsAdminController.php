<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\UtilisateurRepository;
use App\UsageRights\UsagePackageRepository;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\AudienceResolver;
use App\UsageRights\UsageRightsShadow;
use App\UsageRights\UsageGrantAction;
use App\UsageRights\GrantWindow;
use App\Reservation\ReservableType;
use App\Repository\MachineCategoryRepository;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Repository\VenueRepository;
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
        private readonly UsageCapabilityRegistry $capabilityRegistry,
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
    #[Route('/shadow', name: 'app_admin_usage_rights_shadow', methods: ['GET', 'POST'])]
    public function shadow(Request $request, UtilisateurRepository $users, UsageRightsShadow $shadow, AudienceResolver $audiences): Response
    {
        if ($request->isMethod('POST')) {
            return $this->moveChokepoint($request, $users, $shadow);
        }

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
            'chokepoints' => $shadow->chokepoints($users->findAll()),
        ]);
    }

    /**
     * Move one chokepoint onto grants v2, or move it back (S134).
     *
     * ⚠️ **Enabling is refused while anybody would lose access.** That check is
     * the difference between "activation graduelle sur les chokepoints audités"
     * and a flag: the audit is not a document somebody wrote, it is this count
     * being zero, computed over **every** account rather than over the page the
     * operator happens to be looking at. A safety gate read from a sample is not
     * a safety gate.
     *
     * ⚠️ **Disabling is never refused.** Rolling back must not require the
     * installation to be in a good state — it is what an operator reaches for
     * precisely when it is not.
     */
    private function moveChokepoint(Request $request, UtilisateurRepository $users, UsageRightsShadow $shadow): Response
    {
        if (!$this->isCsrfTokenValid('usage_rights_shadow', (string) $request->request->get('_token'))) {
            $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));

            return $this->redirectToRoute('app_admin_usage_rights_shadow');
        }

        $capability = (string) $request->request->get('capability');
        $enable = $request->request->getBoolean('enable');

        if ($this->capabilityRegistry->get($capability) === null) {
            $this->addFlash('error', $this->translator->trans('usage_rights.shadow_unknown_capability'));

            return $this->redirectToRoute('app_admin_usage_rights_shadow');
        }

        if (!$enable) {
            $this->settings->setUsageRightsV2Active($capability, false);
            $this->addFlash('success', $this->translator->trans('usage_rights.shadow_moved_back', ['%capability%' => $capability]));

            return $this->redirectToRoute('app_admin_usage_rights_shadow');
        }

        $deniers = $shadow->deniersFor($capability, $users->findAll());
        if ($deniers > 0) {
            $this->addFlash('error', $this->translator->trans('usage_rights.shadow_refused', [
                '%capability%' => $capability, '%count%' => $deniers,
            ]));

            return $this->redirectToRoute('app_admin_usage_rights_shadow');
        }

        $this->settings->setUsageRightsV2Active($capability, true);
        $this->addFlash('success', $this->translator->trans('usage_rights.shadow_moved', ['%capability%' => $capability]));

        return $this->redirectToRoute('app_admin_usage_rights_shadow');
    }

    #[Route('/new', name: 'app_admin_usage_rights_new', methods: ['GET', 'POST'])]
    public function new(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities): Response
    {
        return $this->form($request, $packages, $features, $capabilities);
    }

    #[Route('/{id<\d+>}/edit', name: 'app_admin_usage_rights_edit', methods: ['GET', 'POST'])]
    public function edit(int $id, Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, UtilisateurRepository $users, SiteSettingService $settings, VenueRepository $venues, AudienceResolver $audiences, MachineRepository $machines, PlaceRepository $places, MachineCategoryRepository $categories): Response
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

        // ⚠️ **Assigning to a group (S144a).** Deliberately its own action and its
        // own CSRF token rather than a "member or group" switch on the one above:
        // the two write different columns, refuse for different reasons, and a
        // single form that silently ignores one of its two selects is exactly the
        // kind of control an operator cannot reason about.
        if ($request->isMethod('POST') && $request->request->get('action') === 'assign_group') {
            if (!$this->isCsrfTokenValid('usage_package_assign_group_' . $id, (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));
            } else {
                try {
                    $zone = new \DateTimeZone($settings->getTimezone());
                    $actor = $this->getUser();
                    $packages->assignGroup(
                        $id,
                        trim((string) $request->request->get('group_key')),
                        $this->date($request->request->get('valid_from'), $zone),
                        $this->date($request->request->get('valid_until'), $zone),
                        $actor instanceof Utilisateur ? $actor->getId() : null,
                    );
                    $this->addFlash('success', $this->translator->trans('usage_rights.group_assignment_created'));
                } catch (\Throwable $e) {
                    $this->addFlash('error', $e->getMessage());
                }
            }

            return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
        }

        // ⚠️ **The grants editor (S134b).** Until this existed a package could only
        // carry its v1 feature list, so the three dimensions grants v2 adds —
        // action, location, section — were reachable by SQL and by nothing else.
        // A model whose only editor is a database client is not administrable, and
        // "droits administrables" is what S133b was for.
        if ($request->isMethod('POST') && in_array($request->request->get('action'), ['grant_add', 'grant_delete', 'window_add', 'window_delete'], true)) {
            if (!$this->isCsrfTokenValid('usage_package_grants_' . $id, (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            }

            try {
                if ($request->request->get('action') === 'grant_delete') {
                    $packages->deleteGrant($id, $request->request->getInt('grant_id'));
                    $this->addFlash('success', $this->translator->trans('usage_rights.grant_deleted'));
                } elseif ($request->request->get('action') === 'window_delete') {
                    // ⚠️ Removing the last window WIDENS the grant back to the
                    // whole week. It is the one delete in this editor that grants
                    // more than it took, which is why the button says so.
                    $packages->deleteWindow($id, $request->request->getInt('window_id'));
                    $this->addFlash('success', $this->translator->trans('usage_rights.window_deleted'));
                } elseif ($request->request->get('action') === 'window_add') {
                    $packages->addWindow($id, $request->request->getInt('grant_id'), GrantWindow::fromClock(
                        $request->request->getInt('day_of_week'),
                        (string) $request->request->get('start_time'),
                        (string) $request->request->get('end_time'),
                    ));
                    $this->addFlash('success', $this->translator->trans('usage_rights.window_added'));
                } else {
                    $capability = $capabilities->get((string) $request->request->get('feature'));
                    if ($capability === null) {
                        throw new \InvalidArgumentException($this->translator->trans('usage_rights.shadow_unknown_capability'));
                    }
                    // ⚠️ `tryFrom`, not a cast: an unrecognised action string would
                    // otherwise land in the column and match nothing, which reads
                    // as "the grant does nothing" rather than as a rejected input.
                    $action = UsageGrantAction::tryFrom((string) $request->request->get('grant_action'));
                    if ($action === null) {
                        throw new \InvalidArgumentException($this->translator->trans('usage_rights.grant_bad_action'));
                    }
                    $venueId = $request->request->getInt('venue_id') ?: null;
                    if ($venueId !== null && $venues->find($venueId) === null) {
                        throw new \InvalidArgumentException($this->translator->trans('usage_rights.grant_bad_venue'));
                    }
                    $section = trim((string) $request->request->get('section')) ?: null;

                    // ⚠️ **The resource axis arrives as ONE field**, `machine` or
                    // `machine:12`, and the kind is derived from it rather than
                    // asked for separately (S144b). Two selects would let an
                    // operator choose "espace" and then a machine, and the pair
                    // would be stored, read, and match nothing — a restriction
                    // that silently refuses everything is worse than one that
                    // cannot be expressed.
                    [$reservableType, $reservableId] = $this->parseResource((string) $request->request->get('reservable'));
                    $categoryLabel = trim((string) $request->request->get('category_label')) ?: null;

                    // A category is a sentence about machines — "the 3D printers"
                    // — so it implies the kind rather than needing it chosen too.
                    if ($categoryLabel !== null && $reservableType === null) {
                        $reservableType = ReservableType::Machine;
                    }
                    // And it cannot narrow anything else: keeping it on a place
                    // grant would be a restriction that can never match.
                    if ($reservableType !== ReservableType::Machine) {
                        $categoryLabel = null;
                    }

                    $grantId = $packages->addGrant(
                        $id,
                        $capability->featureKey,
                        $action,
                        $venueId,
                        $section,
                        $reservableType?->value,
                        $reservableId,
                        $categoryLabel,
                    );

                    // The common case the operator described — "3D print on Monday
                    // afternoons" — is one grant and one window, so it is one
                    // submission. Further windows are added from the grant's row.
                    $day = $request->request->getInt('day_of_week');
                    if ($day >= 1 && $day <= 7 && $grantId > 0) {
                        // ⚠️ Its own try: the grant is already written, and a
                        // window that fails — an install without the S144b
                        // migration, a backwards interval — must not be reported
                        // as "the grant was not created". It was, and it is wider
                        // than intended, which is the thing an operator has to be
                        // told plainly.
                        try {
                            $packages->addWindow($id, $grantId, GrantWindow::fromClock(
                                $day,
                                (string) $request->request->get('start_time'),
                                (string) $request->request->get('end_time'),
                            ));
                        } catch (\Throwable $e) {
                            $this->addFlash('error', $this->translator->trans('usage_rights.window_failed_grant_kept') . ' ' . $e->getMessage());
                        }
                    }

                    $this->addFlash('success', $this->translator->trans('usage_rights.grant_added'));
                }
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
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

        return $this->form($request, $packages, $features, $capabilities, $package, $users, $venues, $audiences, [
            // ⚠️ Machines and places by (name, id) only. The picker needs a label
            // and an id; handing whole entities to a template is how a list screen
            // ends up lazy-loading a venue per row.
            'machines' => array_map(
                static fn (object $machine): array => ['id' => $machine->getId(), 'name' => $machine->getNom()],
                $machines->findBy([], ['nom' => 'ASC']),
            ),
            'places' => array_map(
                static fn (object $place): array => ['id' => $place->getId(), 'name' => $place->getNom()],
                $places->findBy([], ['nom' => 'ASC']),
            ),
            // The category CRUD's labels, which are the same strings
            // `MACHINE.categoryLabel` holds — see `ReservableResolver`.
            'categories' => array_values(array_unique(array_map(
                static fn (object $category): string => (string) $category->getLabel(),
                $categories->allOrdered(false),
            ))),
        ]);
    }

    /** @param array{id:int,name:string,description:string,active:bool,features:list<string>}|null $package */
    private function form(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, ?array $package = null, ?UtilisateurRepository $users = null, ?VenueRepository $venues = null, ?AudienceResolver $audiences = null, array $resources = []): Response
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
            // ⚠️ Grants belong to a saved package, so a package being CREATED has
            // none and shows no editor — there is no id to hang them on yet. The
            // form redirects to the edit screen on save, which is where they are.
            'grants' => $package !== null ? $packages->grantsFor($package['id']) : [],
            'venues' => $venues?->findBy(['active' => true], ['name' => 'ASC']) ?? [],
            // ⚠️ `guest` is filtered out here as well as refused in the
            // repository. The rule is enforced server-side either way; keeping it
            // out of the picker is so nobody is offered a choice that can only be
            // answered with an error message.
            'groups' => array_values(array_filter(
                $audiences?->catalogue() ?? [],
                static fn (array $group): bool => $group['key'] !== AudienceResolver::GUEST,
            )),
            'machines' => $resources['machines'] ?? [],
            'places' => $resources['places'] ?? [],
            'categories' => $resources['categories'] ?? [],
            // ⚠️ A grant row stores an id; a screen must not show one. "machine
            // #12" tells an operator nothing about whether the package they are
            // selling covers the laser cutter. Keyed the same way the picker
            // submits, so the two can never disagree about what a value means.
            'resourceNames' => $this->resourceNames($resources),
        ]);
    }

    /**
     * @param array{machines?: list<array{id:?int,name:string}>, places?: list<array{id:?int,name:string}>} $resources
     * @return array<string, string> "machine:12" => "Prusa MK4"
     */
    private function resourceNames(array $resources): array
    {
        $names = [];
        foreach (['machine' => 'machines', 'place' => 'places'] as $kind => $bucket) {
            foreach ($resources[$bucket] ?? [] as $resource) {
                if ($resource['id'] !== null) {
                    $names[$kind . ':' . $resource['id']] = $resource['name'];
                }
            }
        }

        return $names;
    }

    /**
     * `""` | `machine` | `machine:12` → [kind, id]. (S144b)
     *
     * ⚠️ An id that does not parse drops to null rather than to 0: `reservableId
     * = 0` would be stored, would match no machine, and would turn a grant into a
     * silent refusal — the failure mode this model has already produced twice.
     *
     * @return array{0: ?ReservableType, 1: ?int}
     */
    private function parseResource(string $raw): array
    {
        $raw = trim($raw);
        if ($raw === '') {
            return [null, null];
        }

        [$kind, $id] = array_pad(explode(':', $raw, 2), 2, null);
        $type = ReservableType::tryParse($kind);
        if ($type === null) {
            return [null, null];
        }

        return [$type, ctype_digit((string) $id) ? (int) $id : null];
    }

    private function date(mixed $raw, \DateTimeZone $zone): ?\DateTimeImmutable
    {
        $value = trim((string) $raw);
        return $value === '' ? null : new \DateTimeImmutable($value, $zone);
    }
}
