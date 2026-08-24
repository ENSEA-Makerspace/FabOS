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
use App\UsageRights\UsageAllowance;
use App\UsageRights\UsageAllowanceRepository;
use App\Reservation\ReservableType;
use App\Repository\MachineCategoryRepository;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Form\UsageRights\PackageAllowanceType;
use App\Form\UsageRights\PackageAssignGroupType;
use App\Form\UsageRights\PackageAssignType;
use App\Form\UsageRights\PackageDetailsType;
use App\Form\UsageRights\PackageGrantType;
use App\Service\SiteSettingService;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\Form\FormInterface;
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
                'capability' => $capability, 'count' => $deniers,
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
    public function edit(int $id, Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, UtilisateurRepository $users, SiteSettingService $settings, AudienceResolver $audiences, MachineRepository $machines, PlaceRepository $places, MachineCategoryRepository $categories, UsageAllowanceRepository $allowances): Response
    {
        $package = $packages->find($id);
        if ($package === null) {
            throw $this->createNotFoundException();
        }

        // ⚠️ **Les cinq ACTIONS restent des actions** (S147, J-22). Supprimer un
        // grant, une plage, une allocation, révoquer une attribution, ajouter une
        // plage à une ligne précise : ce sont des boutons qui portent un
        // identifiant, pas des listes de champs. Elles gardent leur `action`
        // cachée, leur jeton posé à la main, et leur redirection — un
        // `FormType` n'aurait rien à y valider.
        // 🔴 Et `window_add` est PAR GRANT : un type unique produirait le même
        // `name` sur chaque ligne de la liste, donc chaque soumission écrirait
        // sur la première.
        if ($request->isMethod('POST') && in_array($request->request->get('action'), ['grant_delete', 'window_add', 'window_delete'], true)) {
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
                } else {
                    $packages->addWindow($id, $request->request->getInt('grant_id'), GrantWindow::fromClock(
                        $request->request->getInt('day_of_week'),
                        (string) $request->request->get('start_time'),
                        (string) $request->request->get('end_time'),
                    ));
                    $this->addFlash('success', $this->translator->trans('usage_rights.window_added'));
                }
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }

            return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
        }

        if ($request->isMethod('POST') && $request->request->get('action') === 'allowance_delete') {
            if (!$this->isCsrfTokenValid('usage_package_allowances_' . $id, (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            }

            try {
                $allowances->delete($id, $request->request->getInt('allowance_id'));
                $this->addFlash('success', $this->translator->trans('usage_rights.allowance_deleted'));
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

        // ⚠️ Les listes que les quatre formulaires proposent, lues UNE fois. Le
        // gabarit en recevait déjà les mêmes, pour dessiner les mêmes `<option>`.
        $machineList = array_map(
            static fn (object $machine): array => ['id' => $machine->getId(), 'name' => $machine->getNom()],
            $machines->findBy([], ['nom' => 'ASC']),
        );
        $placeList = array_map(
            static fn (object $place): array => ['id' => $place->getId(), 'name' => $place->getNom()],
            $places->findBy([], ['nom' => 'ASC']),
        );
        // The category CRUD's labels, which are the same strings
        // `MACHINE.categoryLabel` holds — see `ReservableResolver`.
        $categoryList = array_values(array_unique(array_map(
            static fn (object $category): string => (string) $category->getLabel(),
            $categories->allOrdered(false),
        )));
        $venueList = $venues->findBy(['active' => true], ['name' => 'ASC']);
        // ⚠️ `guest` is filtered out here as well as refused in the repository.
        // The rule is enforced server-side either way; keeping it out of the
        // picker is so nobody is offered a choice that can only be answered with
        // an error message.
        $groupList = array_values(array_filter(
            $audiences->catalogue(),
            static fn (array $group): bool => $group['key'] !== AudienceResolver::GUEST,
        ));
        $userList = $users->findBy([], ['lastName' => 'ASC', 'firstName' => 'ASC']);

        $available = $capabilities->all();
        $zone = new \DateTimeZone($settings->getTimezone());

        // 🔴 **Chaque liste arrive TRADUITE et `choice_translation_domain` est
        // `false`.** Un `ChoiceType` passe ses libellés au traducteur, et la
        // moitié de ceux-ci sont des DONNÉES — un nom de machine, un libellé de
        // catégorie, le nom d'un membre. Les laisser traduire, c'est chercher
        // « Prusa MK4 » dans le catalogue : ça ressort inchangé tant que
        // personne n'a de machine nommée comme une clé, et ce jour-là l'écran
        // ment. Traduire ici, une fois, retire la question.
        $trans = fn (string $key): string => $this->translator->trans($key);

        $featureChoices = [];
        foreach ($available as $key => $capability) {
            $featureChoices[$trans($capability->labelKey)] = $key;
        }

        $venueChoices = [];
        foreach ($venueList as $venue) {
            $venueChoices[$venue->getName()] = (string) $venue->getId();
        }

        // ⚠️ **Un seul champ, pas deux** (S144b). Le genre se déduit de ce qui
        // est choisi — `machine` ou `machine:12` — pour qu'aucune soumission ne
        // puisse apparier « espace » et une machine.
        $resourceChoices = [
            $trans('usage_rights.grant_all_machines') => 'machine',
            $trans('usage_rights.grant_all_places') => 'place',
        ];
        if ($machineList !== []) {
            $group = [];
            foreach ($machineList as $machine) {
                $group[$machine['name']] = 'machine:' . $machine['id'];
            }
            $resourceChoices[$trans('usage_rights.grant_one_machine')] = $group;
        }
        if ($placeList !== []) {
            $group = [];
            foreach ($placeList as $place) {
                $group[$place['name']] = 'place:' . $place['id'];
            }
            $resourceChoices[$trans('usage_rights.grant_one_place')] = $group;
        }

        // ⚠️ **S149 — plus de « À toute heure » dans les jours.** Le `0` disait
        // la même chose que le préréglage « à toute heure », à un deuxième
        // endroit : deux contrôles pour une décision, et « personnalisé » + « à
        // toute heure » était une combinaison qui ne voulait rien dire. Le
        // préréglage porte désormais ce choix seul.
        $dayChoices = [];
        foreach (range(1, 7) as $day) {
            $dayChoices[$trans('usage_rights.day_' . $day)] = (string) $day;
        }

        // ⚠️ **Le préréglage de plage (S149), et son ordre.** Le cas courant
        // arrive en tête et il est le défaut : un grant est presque toujours
        // éveillé tout le temps, et c'est la restriction qui se demande.
        // 🔴 **« Horaires d'ouverture » a été proposé puis RETIRÉ (décision du
        // chef, S149).** Notre modèle ne sait stocker que des plages hebdomadaires,
        // donc l'option ne pouvait qu'en COPIER une au moment de la soumission —
        // et la copie ne suit pas un changement d'horaires. Le libellé pouvait le
        // dire (« copiés maintenant »), mais c'est un avertissement d'un instant :
        // une fois enregistré, le grant est indiscernable d'une plage tapée à la
        // main, et six mois plus tard **rien à l'écran ne révèle la divergence**.
        // C'est précisément la famille de défauts que cette phase range.
        // 🅿️ À reproposer le jour où un grant peut stocker un MODE relu à chaque
        // vérification, plutôt qu'une copie.
        $presetChoices = [
            $trans('usage_rights.window_any_day') => 'any',
            $trans('usage_rights.window_preset_custom') => 'custom',
        ];

        $memberChoices = [];
        foreach ($userList as $member) {
            $memberChoices[$member->getDisplayName() . ' — ' . $member->getEmail()] = (string) $member->getId();
        }

        $groupChoices = [];
        foreach ($groupList as $group) {
            $suffix = $group['virtual']
                ? $trans('usage_rights.group_everyone')
                : $this->translator->trans('usage_rights.group_members', ['count' => $group['members']]);
            $groupChoices[$group['label'] . ' — ' . $suffix] = $group['key'];
        }

        $actionChoices = [
            $trans('usage_rights.grant_use') => UsageGrantAction::Use->value,
            $trans('usage_rights.grant_manage') => UsageGrantAction::Manage->value,
        ];
        $categoryChoices = array_combine($categoryList, $categoryList);

        $grantForm = $this->createForm(PackageGrantType::class, [
            // Les valeurs que le gabarit posait en dur, remontées là où elles se
            // lisent : l'exemple de l'opérateur est « le jeudi de 14 h à 18 h ».
            // ⚠️ **S149 — le défaut de la PLAGE est maintenant le préréglage**, et
            // il vaut « à toute heure » : c'est le cas courant, et c'est ce que
            // faisaient déjà tous les grants écrits avant S144b. Les trois champs
            // gardent des valeurs d'exemple pour le jour où on ouvre
            // « personnalisé… », mais elles ne décident plus rien toutes seules.
            'grant_action' => UsageGrantAction::Use->value,
            'window_preset' => 'any',
            'day_of_week' => '1',
            'start_time' => '14:00',
            'end_time' => '18:00',
        ], [
            'package_key' => (string) $id,
            'feature_choices' => $featureChoices,
            'action_choices' => $actionChoices,
            'venue_choices' => $venueChoices,
            'resource_choices' => $resourceChoices,
            'category_choices' => $categoryChoices,
            'day_choices' => $dayChoices,
            'window_preset_choices' => $presetChoices,
        ]);

        // ⚠️ **Les mêmes listes, retournées.** Le formulaire les lit
        // « libellé => valeur » ; la ligne de conséquence et le résumé de portée
        // les lisent « valeur => libellé ». Retourner ici, une fois, évite qu'un
        // deuxième endroit se remette à traduire des DONNÉES — voir le 🔴
        // au-dessus de `$trans`. `$resourceChoices` est groupé, d'où l'aplatissage.
        $flip = static function (array $choices): array {
            $out = [];
            foreach ($choices as $label => $value) {
                if (is_array($value)) {
                    foreach ($value as $subLabel => $subValue) {
                        $out[(string) $subValue] = (string) $subLabel;
                    }
                    continue;
                }
                $out[(string) $value] = (string) $label;
            }

            return $out;
        };
        $grantLabels = [
            'feature' => $flip($featureChoices),
            'action' => $flip($actionChoices),
            // La clé vide est le libellé du « sans restriction » de la dimension,
            // celui que le `placeholder` du champ affiche déjà.
            'venue' => $flip($venueChoices) + ['' => $trans('usage_rights.grant_any_venue')],
            'resource' => $flip($resourceChoices) + ['' => $trans('usage_rights.grant_any_resource')],
            'category' => $flip($categoryChoices) + ['' => $trans('usage_rights.grant_any_category')],
            'day' => $flip($dayChoices),
            'preset' => $flip($presetChoices),
        ];

        $unitChoices = [];
        foreach (UsageAllowance::UNITS as $unit) {
            $unitChoices[$trans('usage_rights.allowance_unit_' . $unit)] = $unit;
        }
        $periodChoices = [];
        foreach (UsageAllowance::PERIODS as $period) {
            $periodChoices[$trans('usage_rights.allowance_period_' . $period)] = $period;
        }

        $allowanceForm = $this->createForm(PackageAllowanceType::class, [
            'allowance_unit' => UsageAllowance::UNIT_MINUTES,
            'allowance_hours' => 10,
            'allowance_count' => 1,
            'allowance_period' => 'week',
        ], [
            'package_key' => (string) $id,
            'feature_choices' => $featureChoices,
            'reservable_choices' => [
                $trans('usage_rights.grant_all_machines') => 'machine',
                $trans('usage_rights.grant_all_places') => 'place',
                $trans('usage_rights.grant_all_users') => 'user',
            ],
            'unit_choices' => $unitChoices,
            'period_choices' => $periodChoices,
        ]);

        $assignForm = $this->createForm(PackageAssignType::class, null, [
            'package_key' => (string) $id,
            'member_choices' => $memberChoices,
            'lab_timezone' => $settings->getTimezone(),
        ]);

        $assignGroupForm = $this->createForm(PackageAssignGroupType::class, null, [
            'package_key' => (string) $id,
            'group_choices' => $groupChoices,
            'lab_timezone' => $settings->getTimezone(),
        ]);

        // ⚠️ **The grants editor (S134b).** Until this existed a package could only
        // carry its v1 feature list, so the three dimensions grants v2 adds —
        // action, location, section — were reachable by SQL and by nothing else.
        // A model whose only editor is a database client is not administrable, and
        // "droits administrables" is what S133b was for.
        $grantForm->handleRequest($request);
        if ($grantForm->isSubmitted() && $grantForm->isValid()) {
            $data = $grantForm->getData();
            try {
                $capability = $capabilities->get((string) $data['feature']);
                if ($capability === null) {
                    throw new \InvalidArgumentException($this->translator->trans('usage_rights.shadow_unknown_capability'));
                }
                // ⚠️ `tryFrom`, not a cast: an unrecognised action string would
                // otherwise land in the column and match nothing, which reads
                // as "the grant does nothing" rather than as a rejected input.
                $action = UsageGrantAction::tryFrom((string) $data['grant_action']);
                if ($action === null) {
                    throw new \InvalidArgumentException($this->translator->trans('usage_rights.grant_bad_action'));
                }
                $venueId = (int) ($data['venue_id'] ?? 0) ?: null;
                if ($venueId !== null && $venues->find($venueId) === null) {
                    throw new \InvalidArgumentException($this->translator->trans('usage_rights.grant_bad_venue'));
                }
                $section = trim((string) ($data['section'] ?? '')) ?: null;

                [$reservableType, $reservableId] = $this->parseResource((string) ($data['reservable'] ?? ''));
                $categoryLabel = trim((string) ($data['category_label'] ?? '')) ?: null;

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

                // ⚠️ **S147, J-21 — on écrit AUSSI l'identité de la catégorie.**
                // Le libellé seul se décrochait au premier renommage : l'écran des
                // catégories renomme pour de vrai et déplace les machines avec lui,
                // si bien que le package continuait de nommer une chaîne à laquelle
                // plus rien ne répondait, et refusait sans cause visible.
                // ⚠️ Un libellé ORPHELIN — une catégorie qui n'existe que sur des
                // machines, ce que l'écran des catégories propose d'« adopter » —
                // ne trouve pas de ligne : l'identifiant reste null et le grant se
                // comporte exactement comme avant. Une portée n'est jamais perdue.
                $categoryId = $categoryLabel === null
                    ? null
                    : $categories->findOneByLabel($categoryLabel)?->getId();

                $grantId = $packages->addGrant(
                    $id,
                    $capability->featureKey,
                    $action,
                    $venueId,
                    $section,
                    $reservableType?->value,
                    $reservableId,
                    $categoryLabel,
                    $categoryId,
                );

                // The common case the operator described — "3D print on Monday
                // afternoons" — is one grant and one window, so it is one
                // submission. Further windows are added from the grant's row.
                //
                // 🔴 **S149 — c'est le PRÉRÉGLAGE qui décide, jamais les champs
                // de plage.** Ils arrivent remplis de leurs défauts même quand
                // l'écran ne les montrait pas : le repli du gabarit et le
                // contrôleur `conditional-field` cachent, ils ne désactivent
                // rien, et un champ caché poste. Déduire l'intention de
                // `day_of_week` rendrait « à toute heure » impossible à
                // exprimer — exactement le bug que le préréglage supprime.
                $preset = (string) ($data['window_preset'] ?? 'any');
                $windows = [];
                if ($grantId > 0 && $preset === 'custom') {
                    // ⚠️ Le `0` n'est plus offert par la liste (S149) mais reste
                    // toléré ici : un POST d'un onglet resté ouvert doit produire
                    // « pas de plage », pas une exception.
                    $day = (int) ($data['day_of_week'] ?? 0);
                    if ($day >= 1 && $day <= 7) {
                        $windows[] = GrantWindow::fromClock(
                            $day,
                            (string) ($data['start_time'] ?? ''),
                            (string) ($data['end_time'] ?? ''),
                        );
                    }
                }

                // ⚠️ Its own try: the grant is already written, and a
                // window that fails — an install without the S144b
                // migration, a backwards interval — must not be reported
                // as "the grant was not created". It was, and it is wider
                // than intended, which is the thing an operator has to be
                // told plainly.
                // ⚠️ Un seul message pour toute la fournée : sept plages
                // recopiées d'un coup ne doivent pas produire sept bandeaux.
                $windowError = null;
                foreach ($windows as $window) {
                    try {
                        $packages->addWindow($id, $grantId, $window);
                    } catch (\Throwable $e) {
                        $windowError ??= $e->getMessage();
                    }
                }
                if ($windowError !== null) {
                    $this->addFlash('error', $this->translator->trans('usage_rights.window_failed_grant_kept') . ' ' . $windowError);
                }

                $this->addFlash('success', $this->translator->trans('usage_rights.grant_added'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            } catch (\Throwable $e) {
                // ⚠️ On REND, on ne redirige plus. Le grant refusé garde ses
                // champs à l'écran : rediriger renverrait une carte vide et
                // l'opérateur retaperait la phrase entière pour changer un mot.
                $this->addFlash('error', $e->getMessage());
            }
        }

        // ⚠️ **Ce que le grant va faire, en toutes lettres (S149, règle 4).**
        // Lu APRÈS `handleRequest()`, donc : les défauts à l'arrivée, et ce qui a
        // été tapé après un refus. Les mêmes mots que la ligne qui apparaîtra
        // dans la liste en dessous — le formulaire montre son résultat avant de
        // le produire, plutôt que de le décrire.
        // 🔴 Calculé ICI et pas dans le gabarit : `prod` n'a pas
        // `strict_variables`, et une phrase assemblée à partir de `form.vars`
        // absents s'y afficherait à moitié sans que rien ne le signale.
        $grantData = (array) ($grantForm->getData() ?? []);
        $grantParts = $this->grantParts($grantData, $grantLabels);
        $grantConsequence = $this->translator->trans('usage_rights.grant_consequence', [
            '%summary%' => implode(' · ', array_filter($grantParts)),
        ]);
        // ⚠️ **Le résumé de portée est la ligne qui REMPLACE quatre champs**
        // (règle 7) : « Tous les lieux · Toutes les ressources · Toutes les
        // catégories », avec un lien qui déplie les contrôles.
        $grantScope = implode(' · ', array_filter([
            $grantParts['venue'], $grantParts['section'], $grantParts['resource'], $grantParts['category'],
        ]));
        // 🔴 **Et le repli se ROUVRE quand la portée n'est pas celle par défaut**,
        // pour la même raison que `refusedEditor` : un grant refusé sur son lieu
        // doit montrer le lieu qui a été choisi, pas une ligne qui affirme
        // « tous les lieux » au-dessus d'un repli fermé. Drapeau explicite depuis
        // le contrôleur, jamais `form.vars` lu dans le gabarit.
        $grantScopeOpen = false;
        foreach (['venue_id', 'section', 'reservable', 'category_label'] as $scopeField) {
            $grantScopeOpen = $grantScopeOpen || trim((string) ($grantData[$scopeField] ?? '')) !== '';
        }

        // ⚠️ **Allocations (S144c), and their own token.** This is the one editor
        // on this page where ADDING narrows access: a package with no allowance is
        // unlimited, so the first row turns "as much as you like" into "ten hours
        // a week". Kept apart from the grants form for that reason — the two read
        // as the same kind of write and are the opposite kind.
        $allowanceForm->handleRequest($request);
        if ($allowanceForm->isSubmitted() && $allowanceForm->isValid()) {
            $data = $allowanceForm->getData();
            try {
                $capability = $capabilities->get((string) ($data['allowance_feature'] ?? ''));
                $reservable = ReservableType::tryParse((string) ($data['allowance_reservable'] ?? ''));
                $unit = (string) $data['allowance_unit'];
                // Hours in the form, minutes in the column: an operator sells
                // hours and a booking is measured in minutes, and converting
                // at the door keeps every reader of the table in one unit.
                // 🔴 Deux champs de quantité, et c'est l'UNITÉ qui décide lequel
                // compte. Les fusionner écrirait des séances dans une colonne de
                // minutes — un quota faux et muet.
                $amount = $unit === UsageAllowance::UNIT_MINUTES
                    ? (int) round(((float) ($data['allowance_hours'] ?? 0)) * 60)
                    : (int) ($data['allowance_count'] ?? 0);

                $allowances->add(
                    $id,
                    $capability?->featureKey,
                    $reservable?->value,
                    $unit,
                    $amount,
                    (string) $data['allowance_period'],
                );
                $this->addFlash('success', $this->translator->trans('usage_rights.allowance_added'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }
        }

        $assignForm->handleRequest($request);
        if ($assignForm->isSubmitted() && $assignForm->isValid()) {
            $data = $assignForm->getData();
            try {
                $member = $users->find((int) $data['user_id']);
                if (!$member instanceof Utilisateur) {
                    throw new \InvalidArgumentException($this->translator->trans('usage_rights.member_required'));
                }
                // 🔴 Les deux dates sont des CHAÎNES et le sont restées : c'est
                // ce helper qui les construit dans le fuseau du labo. Laisser le
                // formulaire hydrater un `DateTimeImmutable` l'aurait fait dans
                // le fuseau PHP — UTC ici — et décalé toute validité sans rien
                // dire à l'écran.
                $from = $this->date($data['valid_from'] ?? null, $zone);
                $until = $this->date($data['valid_until'] ?? null, $zone);
                $actor = $this->getUser();
                $packages->assign($id, $member, $from, $until, $actor instanceof Utilisateur ? $actor->getId() : null);
                $this->addFlash('success', $this->translator->trans('usage_rights.assignment_created'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }
        }

        // ⚠️ **Assigning to a group (S144a).** Deliberately its own form and its
        // own CSRF token rather than a "member or group" switch on the one above:
        // the two write different columns, refuse for different reasons, and a
        // single form that silently ignores one of its two selects is exactly the
        // kind of control an operator cannot reason about.
        $assignGroupForm->handleRequest($request);
        if ($assignGroupForm->isSubmitted() && $assignGroupForm->isValid()) {
            $data = $assignGroupForm->getData();
            try {
                $actor = $this->getUser();
                $packages->assignGroup(
                    $id,
                    trim((string) $data['group_key']),
                    $this->date($data['valid_from'] ?? null, $zone),
                    $this->date($data['valid_until'] ?? null, $zone),
                    $actor instanceof Utilisateur ? $actor->getId() : null,
                );
                $this->addFlash('success', $this->translator->trans('usage_rights.group_assignment_created'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            } catch (\Throwable $e) {
                $this->addFlash('error', $e->getMessage());
            }
        }

        return $this->form($request, $packages, $features, $capabilities, $package, [
            // ⚠️ Machines and places by (name, id) only. The picker needs a label
            // and an id; handing whole entities to a template is how a list screen
            // ends up lazy-loading a venue per row.
            'machines' => $machineList,
            'places' => $placeList,
            'categories' => $categoryList,
            'allowances' => $allowances->forPackage($id),
            // ⚠️ Said in words on the screen rather than assumed: an install
            // whose migration has not been run must not show an editor whose
            // every submission fails.
            'allowancesReady' => $allowances->tableExists(),
            'grantConsequence' => $grantConsequence,
            'grantScope' => $grantScope,
            'grantScopeOpen' => $grantScopeOpen,
        ], [
            'grantForm' => $grantForm,
            'allowanceForm' => $allowanceForm,
            'assignForm' => $assignForm,
            'assignGroupForm' => $assignGroupForm,
        ]);
    }

    /**
     * Lequel des quatre éditeurs « ajouter » a été soumis — donc refusé.
     *
     * @param array<string, FormInterface|null> $extraForms
     */
    private function refusedEditor(array $extraForms): ?string
    {
        foreach ($extraForms as $name => $form) {
            if ($form !== null && $form->isSubmitted()) {
                return $name;
            }
        }

        return null;
    }

    /**
     * @param array{id:int,name:string,description:string,active:bool,features:list<string>}|null $package
     * @param array<string, FormInterface> $extraForms les quatre éditeurs de la colonne de droite, déjà traités par `edit()`
     */
    private function form(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, ?array $package = null, array $resources = [], array $extraForms = []): Response
    {
        $available = $capabilities->all();
        $enabled = array_filter($available, static fn ($capability): bool => $features->isEnabled($capability->featureKey));

        // ⚠️ **La MATRICE de fonctionnalités reste du balisage** et poste
        // `features[]` à la racine, hors du nom du formulaire.
        // `_usage_rights_matrix.html.twig` est un partial PARTAGÉ — la
        // prévisualisation d'un droit s'en sert ailleurs — et l'absorber dans le
        // type le casserait pour ses autres appelants pour ne gagner qu'une
        // uniformité de façade.
        $detailsForm = $this->createForm(PackageDetailsType::class, [
            'name' => $package['name'] ?? '',
            'description' => $package['description'] ?? '',
            'active' => $package['active'] ?? true,
            'full_access' => $package['fullAccess'] ?? false,
        ], ['package_key' => (string) ($package['id'] ?? 'new')]);

        $detailsForm->handleRequest($request);
        if ($detailsForm->isSubmitted() && $detailsForm->isValid()) {
            $data = $detailsForm->getData();
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
                    (string) $data['name'],
                    (string) $data['description'],
                    (bool) $data['active'],
                    (bool) $data['full_access'],
                    $selected,
                );
                $this->addFlash('success', $this->translator->trans('usage_rights.package_saved'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            } catch (\Throwable $e) {
                // ⚠️ On tombe sur le rendu, pas sur une redirection : le nom
                // refusé reste dans le champ.
                $this->addFlash('error', $e->getMessage());
            }
        }

        return $this->render('site/admin-usage-package-form.html.twig', [
            'package' => $package ?? ['name' => '', 'description' => '', 'active' => true, 'fullAccess' => false, 'features' => []],
            'detailsForm' => $detailsForm->createView(),
            // ⚠️ Grants belong to a saved package, so a package being CREATED has
            // none and shows no editor — there is no id to hang them on yet. The
            // form redirects to the edit screen on save, which is where they are.
            'grantForm' => $extraForms['grantForm']?->createView(),
            'allowanceForm' => $extraForms['allowanceForm']?->createView(),
            'assignForm' => $extraForms['assignForm']?->createView(),
            'assignGroupForm' => $extraForms['assignGroupForm']?->createView(),
            // ⚠️ **S149 — quel repli rouvrir.** Les quatre éditeurs « ajouter » sont
            // repliés derrière un `<details>` : replié, l'opérateur n'y verrait ni ce
            // qu'il vient de taper ni pourquoi c'est refusé. On atteint ce rendu
            // seulement si la soumission a échoué (le succès redirige), donc « soumis »
            // vaut « refusé ».
            // 🔴 Un drapeau EXPLICITE, pas `form.vars.submitted` lu dans le gabarit :
            // `prod` n'a pas `strict_variables`, donc une variable absente y serait
            // silencieusement `null` et le repli resterait fermé sans que rien ne le
            // dise. C'est le piège n°7 de la reprise.
            'refusedEditor' => $this->refusedEditor($extraForms),
            'availableFeatures' => $available,
            'enabledFeatures' => array_keys($enabled),
            'assignments' => $package !== null ? $packages->assignmentsForPackage($package['id']) : [],
            'grants' => $package !== null ? $packages->grantsFor($package['id']) : [],
            'allowances' => $resources['allowances'] ?? [],
            'allowancesReady' => $resources['allowancesReady'] ?? false,
            // ⚠️ Trois valeurs de l'éditeur de grants, et elles ont des défauts
            // INERTES : `new()` passe par la même méthode sans grant à décrire,
            // et le gabarit n'y dessine pas la section. Une chaîne vide s'y lit
            // comme « rien à dire », pas comme « pas encore calculé ».
            'grantConsequence' => $resources['grantConsequence'] ?? '',
            'grantScope' => $resources['grantScope'] ?? '',
            'grantScopeOpen' => $resources['grantScopeOpen'] ?? false,
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
     * La phrase du grant, dimension par dimension (S149, règles 4 et 7).
     *
     * ⚠️ **Les mêmes morceaux servent deux lignes** : la conséquence sous
     * l'éditeur, et le résumé de portée qui remplace quatre champs. Les calculer
     * une fois est ce qui garantit que les deux ne peuvent pas se contredire.
     *
     * ⚠️ **Une valeur inconnue retombe sur elle-même**, elle ne disparaît pas :
     * une section est du texte libre, et un libellé de catégorie ORPHELIN — une
     * catégorie qui n'existe que sur des machines — n'est dans aucune liste. Les
     * effacer ferait dire à la ligne « toutes les catégories » alors que le grant
     * en vise une.
     *
     * @param array<string, mixed>                 $data
     * @param array<string, array<string, string>> $labels valeur → libellé traduit, par dimension ; la clé `''` porte le « sans restriction »
     * @return array<string, ?string>
     */
    private function grantParts(array $data, array $labels): array
    {
        $pick = static function (string $dimension, mixed $raw) use ($labels): ?string {
            $value = trim((string) $raw);

            return $labels[$dimension][$value] ?? ($value === '' ? null : $value);
        };

        $preset = trim((string) ($data['window_preset'] ?? 'any'));
        $window = $preset === 'custom'
            ? trim(($labels['day'][trim((string) ($data['day_of_week'] ?? ''))] ?? '')
                . ' ' . trim((string) ($data['start_time'] ?? '')) . '–' . trim((string) ($data['end_time'] ?? '')))
            : ($labels['preset'][$preset] ?? null);

        return [
            'feature' => $pick('feature', $data['feature'] ?? ''),
            'action' => $pick('action', $data['grant_action'] ?? ''),
            'venue' => $pick('venue', $data['venue_id'] ?? ''),
            'section' => $pick('section', $data['section'] ?? ''),
            'resource' => $pick('resource', $data['reservable'] ?? ''),
            'category' => $pick('category', $data['category_label'] ?? ''),
            'window' => $window === '' ? null : $window,
        ];
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
