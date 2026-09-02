<?php

namespace App\Controller;

use App\Entity\Utilisateur;
use App\Feature\SiteFeatureService;
use App\Repository\UtilisateurRepository;
use App\Repository\VenueRepository;
use App\UsageRights\UsagePackageRepository;
use App\UsageRights\PackageSpec;
use App\UsageRights\PackageSpecCompiler;
use App\Form\UsageRights\PackageSpecType;
use App\UsageRights\UsageCapabilityRegistry;
use App\UsageRights\AudienceResolver;
use App\UsageRights\UsageAllowance;
use App\UsageRights\UsageAllowanceRepository;
use App\Repository\MachineCategoryRepository;
use App\Repository\MachineRepository;
use App\Repository\PlaceRepository;
use App\Form\UsageRights\PackageAssignGroupType;
use App\Form\UsageRights\PackageAssignType;
use App\Form\UsageRights\PackageDetailsType;
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

    // 🔴 **S159 — « Aperçu grants v2 » est RETIRÉ, et sa raison d'être était
    // épuisée.** Cet écran existait pour préparer et déclencher le basculement des
    // chokepoints sur grants v2 ; **les quatre sont basculés** (`usage_rights_v2_*`
    // = 1, mesuré). Il comparait donc deux modèles dont l'un n'est plus consulté :
    // une page qui n'a plus de second terme.
    //
    // 🔴 **Et il portait un retour en arrière qui était devenu un PIÈGE.**
    // `moveChokepoint()` savait remettre une capacité sur la v1 — mais le lecteur
    // v1, `UsagePackageRepository::grantingPackages()`, ne regarde que
    // `a.userId` : il ne voit pas les attributions par GROUPE. Rétrograder
    // aujourd'hui retirerait donc, en silence, les droits de tous ceux qui les
    // tiennent d'un groupe. Un bouton présenté comme une issue de secours et qui
    // casse ce qu'il devait sauver ne se garde pas.
    // ⚠️ Le réglage existe toujours en base (`usage_rights_v2_<capacité>`) : une
    // rétrogradation reste possible par une écriture explicite, délibérée, qui
    // n'a pas l'air d'un bouton.


    #[Route('/new', name: 'app_admin_usage_rights_new', methods: ['GET', 'POST'])]
    public function new(Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities): Response
    {
        return $this->form($request, $packages, $features, $capabilities);
    }

    #[Route('/{id<\d+>}/edit', name: 'app_admin_usage_rights_edit', methods: ['GET', 'POST'])]
    public function edit(int $id, Request $request, UsagePackageRepository $packages, SiteFeatureService $features, UsageCapabilityRegistry $capabilities, UtilisateurRepository $users, SiteSettingService $settings, VenueRepository $venues, AudienceResolver $audiences, MachineRepository $machines, PlaceRepository $places, MachineCategoryRepository $categories, UsageAllowanceRepository $allowances, PackageSpecCompiler $compiler): Response
    {
        $package = $packages->find($id);
        if ($package === null) {
            throw $this->createNotFoundException();
        }

        // ⚠️ **Les SUPPRESSIONS restent des actions** (S147, J-22). Supprimer un
        // grant, une plage, une allocation, révoquer une attribution : ce sont des
        // boutons qui portent un identifiant, pas des listes de champs. Elles
        // gardent leur `action` cachée, leur jeton posé à la main, et leur
        // redirection — un `FormType` n'aurait rien à y valider.
        // 🔴 **S153b — `window_add` a disparu avec les deux éditeurs d'ajout.** La
        // semaine d'un package se dit ligne « quand » de la saisie, en un seul
        // endroit ; l'ajouter aussi ici aurait été une troisième écriture pour le
        // même fait, et la saisie l'aurait effacée sans un mot à la sauvegarde
        // suivante. Retirer une plage reste possible : c'est l'issue de secours.
        if ($request->isMethod('POST') && in_array($request->request->get('action'), ['grant_delete', 'window_delete'], true)) {
            if (!$this->isCsrfTokenValid('usage_package_grants_' . $id, (string) $request->request->get('_token'))) {
                $this->addFlash('error', $this->translator->trans('usage_rights.csrf_error'));

                return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
            }

            try {
                if ($request->request->get('action') === 'grant_delete') {
                    $packages->deleteGrant($id, $request->request->getInt('grant_id'));
                    $this->addFlash('success', $this->translator->trans('usage_rights.grant_deleted'));
                } else {
                    // ⚠️ Removing the last window WIDENS the grant back to the
                    // whole week. It is the one delete in this editor that grants
                    // more than it took, which is why the button says so.
                    $packages->deleteWindow($id, $request->request->getInt('window_id'));
                    $this->addFlash('success', $this->translator->trans('usage_rights.window_deleted'));
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


        // Les sept jours, pour la ligne « quand » de la saisie.
        $dayChoices = [];
        foreach (range(1, 7) as $day) {
            $dayChoices[$trans('usage_rights.day_' . $day)] = (string) $day;
        }


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

        $categoryChoices = array_combine($categoryList, $categoryList);



        // Les quatre périodes, pour la ligne « combien » de la saisie.
        $periodChoices = [];
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

        // ⚠️ **LA SAISIE (S153), et elle passe AVANT l'éditeur détaillé.**
        // Quatre lignes de restriction plus une d'extension, qui se lisent comme
        // une phrase, et un compilateur qui les écrit dans les cinq tables. La
        // mesure qui la justifie : 21 grants sur les deux packages existants, et
        // les 21 sans lieu, sans ressource, sans catégorie, sans fenêtre.
        //
        // 🔴 **`decompile()` peut refuser, et son refus est la garde.** Un package
        // que ces cinq lignes ne savent pas exprimer — un grant `Manage`, une
        // section, une machine précise, deux semaines différentes — ne doit PAS
        // s'ouvrir dans un formulaire qui, à la première soumission, écraserait ce
        // qu'il n'a pas su afficher. Dans ce cas la carte ne s'affiche pas, elle
        // dit pourquoi, et l'éditeur détaillé reste le chemin.
        $spec = $compiler->decompile($id, $package['fullAccess'], $package['features']);
        $specForm = null;
        if ($spec !== null) {
            // ⚠️ **Quand un axe est sur « aucune restriction », sa liste arrive
            // TOUTE COCHÉE.** Elle est masquée, donc invisible — mais elle est ce
            // qu'on découvre en décochant, et la restriction qu'on écrit alors
            // est presque toujours « tout sauf un ». Une liste vide obligerait à
            // recocher ce qu'on ne voulait pas retirer : le coût du cas courant
            // payé pour rien. C'est la case qui décide côté serveur, donc ces
            // valeurs ne changent rien tant qu'elle est cochée.
            $specForm = $this->createForm(PackageSpecType::class, [
                'features_all' => $spec->featuresAll,
                'features' => $spec->featuresAll ? array_values($featureChoices) : $spec->features,
                'days_all' => $spec->daysAll,
                'days' => $spec->daysAll ? array_values($dayChoices) : array_map('strval', $spec->days),
                'start_time' => $spec->startTime,
                'end_time' => $spec->endTime,
                'venues_all' => $spec->venuesAll,
                'venues' => $spec->venuesAll ? array_values($venueChoices) : array_map('strval', $spec->venues),
                'categories_all' => $spec->categoriesAll,
                'categories' => $spec->categoriesAll ? array_values($categoryChoices) : $spec->categories,
                'quota_unlimited' => $spec->quotaUnlimited,
                'quota_hours' => $spec->quotaHours,
                'quota_period' => $spec->quotaPeriod,
                'hours_exempt' => $spec->hoursExempt,
            ], [
                'package_key' => (string) $id,
                'feature_choices' => $featureChoices,
                'day_choices' => $dayChoices,
                'venue_choices' => $venueChoices,
                'category_choices' => $categoryChoices,
                'period_choices' => $periodChoices,
            ]);

            $specForm->handleRequest($request);
            if ($specForm->isSubmitted() && $specForm->isValid()) {
                $data = $specForm->getData();
                try {
                    // 🔴 **C'est la CASE qui décide, jamais la liste.** Les listes
                    // sont masquées par du CSS (`:has()`), pas désactivées : elles
                    // postent leur contenu même repliées. Déduire l'intention de la
                    // liste rendrait « tout » impossible à exprimer dès qu'une case
                    // de détail est restée cochée — le bug exact que le préréglage
                    // de plage a supprimé en S149.
                    $compiler->compile($id, $package['name'], $package['description'], $package['active'], new PackageSpec(
                        featuresAll: (bool) ($data['features_all'] ?? false),
                        features: array_map('strval', (array) ($data['features'] ?? [])),
                        daysAll: (bool) ($data['days_all'] ?? false),
                        days: array_map('intval', (array) ($data['days'] ?? [])),
                        startTime: (string) ($data['start_time'] ?? ''),
                        endTime: (string) ($data['end_time'] ?? ''),
                        venuesAll: (bool) ($data['venues_all'] ?? false),
                        venues: array_map('intval', (array) ($data['venues'] ?? [])),
                        categoriesAll: (bool) ($data['categories_all'] ?? false),
                        categories: array_map('strval', (array) ($data['categories'] ?? [])),
                        quotaUnlimited: (bool) ($data['quota_unlimited'] ?? false),
                        quotaHours: (float) ($data['quota_hours'] ?? 0),
                        quotaPeriod: (string) ($data['quota_period'] ?? 'week'),
                        hoursExempt: (bool) ($data['hours_exempt'] ?? false),
                    ));
                    $this->addFlash('success', $this->translator->trans('usage_rights.spec_saved'));

                    return $this->redirectToRoute('app_admin_usage_rights_edit', ['id' => $id]);
                } catch (\Throwable $e) {
                    // On REND, on ne redirige pas : la saisie refusée reste à
                    // l'écran, comme l'éditeur de grants depuis S149.
                    $this->addFlash('error', $e->getMessage());
                }
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
            // ⚠️ Drapeau EXPLICITE plutôt qu'un `is null` lu dans le gabarit :
            // `prod` n'a pas `strict_variables`, donc une variable absente y
            // serait silencieusement `null` et la carte disparaîtrait sans que
            // rien ne le dise. Piège n°7 de la reprise.
            'specFits' => $spec !== null,
        ], [
            'specForm' => $specForm,
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
            try {
                // 🔴 **S153b — cette carte n'ÉCRIT plus que l'identité, elle ne
                // la recopie plus.** La première version repassait à `save()` le
                // `fullAccess` et les lignes de feature lus au chargement de la
                // page ; une soumission partie d'un onglet ouvert avant une
                // modification de la saisie réécrivait donc l'ancienne liste, et
                // renommer un package RESSUSCITAIT des droits qu'on venait de lui
                // retirer. Mesuré en base sur un package réel — quatre lignes de
                // feature pour deux grants.
                // ⚠️ `saveIdentity()` ne touche ni `fullAccess` ni
                // `USAGE_PACKAGE_FEATURE`. Un écran qui n'a rien à dire sur les
                // droits ne doit pas les écrire, fût-ce en les recopiant.
                if (($package['id'] ?? null) === null) {
                    $id = $packages->save(null, (string) $data['name'], (string) $data['description'], (bool) $data['active'], false, []);
                } else {
                    $id = (int) $package['id'];
                    $packages->saveIdentity($id, (string) $data['name'], (string) $data['description'], (bool) $data['active']);
                }
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
            // ⚠️ La saisie n'existe que pour un package ENREGISTRÉ : elle écrit
            // des grants, et un package en cours de création n'a pas encore d'id
            // à quoi les accrocher. `new()` enregistre puis redirige ici.
            'specForm' => ($extraForms['specForm'] ?? null)?->createView(),
            'specFits' => $resources['specFits'] ?? false,
            'assignForm' => $extraForms['assignForm']?->createView(),
            'assignGroupForm' => $extraForms['assignGroupForm']?->createView(),
            // ⚠️ **Quel repli rouvrir.** Les deux éditeurs d'attribution sont
            // repliés derrière un `<details>` : replié, l'opérateur n'y verrait ni
            // ce qu'il vient de taper ni pourquoi c'est refusé. On atteint ce rendu
            // seulement si la soumission a échoué (le succès redirige), donc
            // « soumis » vaut « refusé ».
            // 🔴 Un drapeau EXPLICITE, pas `form.vars.submitted` lu dans le
            // gabarit : `prod` n'a pas `strict_variables`, donc une variable
            // absente y serait silencieusement `null` et le repli resterait fermé
            // sans que rien ne le dise. C'est le piège n°7 de la reprise.
            'refusedEditor' => $this->refusedEditor($extraForms),
            'availableFeatures' => $available,
            'enabledFeatures' => array_keys($enabled),
            'assignments' => $package !== null ? $packages->assignmentsForPackage($package['id']) : [],
            'grants' => $package !== null ? $packages->grantsFor($package['id']) : [],
            'allowances' => $resources['allowances'] ?? [],
            'allowancesReady' => $resources['allowancesReady'] ?? false,
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


    private function date(mixed $raw, \DateTimeZone $zone): ?\DateTimeImmutable
    {
        $value = trim((string) $raw);
        return $value === '' ? null : new \DateTimeImmutable($value, $zone);
    }
}
