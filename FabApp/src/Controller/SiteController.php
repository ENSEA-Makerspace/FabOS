<?php

namespace App\Controller;

use App\Repository\AccessRfidLogRepository;
use App\Repository\BadgeRepository;
use App\Repository\FormationRepository;
use App\Repository\LogUtilisationRepository;
use App\Repository\MachineRepository;
use App\Repository\ProgressionRepository;
use App\Repository\ReservationRepository;
use App\Repository\SectionRepository;
use App\Repository\QuizRepository;
use App\Repository\QuestionRepository;
use App\Repository\ChoixRepository;
use App\Repository\UtilisateurBadgeRepository;
use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Repository\UtilisateurRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Security\Core\User\PasswordAuthenticatedUserInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;

final class SiteController extends AbstractController
{
    #[Route('/', name: 'app_home', methods: ['GET'])]
    #[Route('/index.html', name: 'app_home_html', methods: ['GET'])]
    public function home(
        UtilisateurRepository $users,
        MachineRepository $machines,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        ReservationRepository $reservations,
        AccessRfidLogRepository $rfidLogs,
        BadgeRepository $badges,
        UtilisateurBadgeRepository $userBadges,
    ): Response
    {
        $topUsers = [];
        foreach ($users->findBy([], ['tempsPresenceTotal' => 'DESC'], 3) as $index => $user) {
            $topUsers[] = [
                'rank' => $index + 1,
                'user' => $user,
                'completedFormations' => $progressions->count(['utilisateur' => $user, 'completed' => true]),
                'rfidLogs' => $rfidLogs->count(['utilisateur' => $user]),
                'badges' => $userBadges->count(['utilisateur' => $user]),
            ];
        }

        return $this->render('site/index.html.twig', [
            'homeStats' => [
                'users' => $users->count([]),
                'machines' => $machines->count([]),
                'formations' => $formations->count([]),
                'reservations' => $reservations->count([]),
                'rfidLogs' => $rfidLogs->count([]),
                'badges' => $badges->count([]),
                'completedFormations' => $progressions->count(['completed' => true]),
            ],
            'machines' => $machines->findBy([], ['createdAt' => 'DESC'], 6),
            'latestRfidLogs' => $rfidLogs->findBy([], ['createdAt' => 'DESC'], 5),
            'topUsers' => $topUsers,
        ]);
    }

    #[Route('/calendrier', name: 'app_calendar', methods: ['GET'])]
    #[Route('/calendrier.html', name: 'app_calendar_html', methods: ['GET'])]
    #[Route('/calendar', name: 'app_calendar_legacy_en', methods: ['GET'])]
    #[Route('/calendar.html', name: 'app_calendar_legacy_en_html', methods: ['GET'])]
    public function calendar(MachineRepository $machines, ReservationRepository $reservations): Response
    {
        return $this->render('site/calendrier.html.twig', [
            'machines' => $machines->findBy([], ['nom' => 'ASC']),
            'reservations' => $reservations->findAllActive(['dateDebut' => 'ASC']),
        ]);
    }

    #[Route('/machines', name: 'app_machines', methods: ['GET'])]
    #[Route('/machines.html', name: 'app_machines_html', methods: ['GET'])]
    #[Route('/machine', name: 'app_machine_legacy_singular', methods: ['GET'])]
    #[Route('/machine.html', name: 'app_machine_legacy_singular_html', methods: ['GET'])]
    public function machines(MachineRepository $machines): Response
    {
        return $this->render('site/machines.html.twig', [
            'machines' => $machines->findBy([], ['createdAt' => 'DESC']),
        ]);
    }


    #[Route('/machine/new', name: 'app_machine_new_legacy', methods: ['GET'])]
    #[Route('/machines/new', name: 'app_machines_new_legacy', methods: ['GET'])]
    public function machineNewLegacy(MachineRepository $machines): Response
    {
        return $this->render('site/admin-machines.html.twig', [
            'machines' => $machines->findBy([], ['createdAt' => 'DESC']),
        ]);
    }

    #[Route('/machines/{id}', name: 'app_machine_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}', name: 'app_machine_detail_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-detail.html', name: 'app_machine_detail_html', methods: ['GET'])]
    public function machineDetail(Request $request, MachineRepository $machines, AccessRfidLogRepository $rfidLogs, LogUtilisationRepository $usageLogs, ReservationRepository $reservations, UtilisateurBadgeRepository $userBadges, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        $currentUser = $this->getUser();
        $requiredBadgeRows = [];
        $hasRequiredBadge = false;

        foreach ($machine->getRequiredMachineBadges() as $machineBadge) {
            $badge = $machineBadge->getBadge();
            if (!$badge) {
                continue;
            }

            $owned = $currentUser instanceof Utilisateur && $userBadges->findOneBy([
                'utilisateur' => $currentUser,
                'badge' => $badge,
            ]) !== null;

            $hasRequiredBadge = $hasRequiredBadge || $owned;
            $requiredBadgeRows[] = [
                'machineBadge' => $machineBadge,
                'badge' => $badge,
                'owned' => $owned,
            ];
        }

        $authorizationStatus = $requiredBadgeRows === [] ? 'no_badge_required' : ($hasRequiredBadge ? 'authorized' : 'missing_badge');

        return $this->render('site/machine-detail.html.twig', [
            'machine' => $machine,
            'requiredBadgeRows' => $requiredBadgeRows,
            'hasRequiredBadge' => $hasRequiredBadge,
            'authorizationStatus' => $authorizationStatus,
            'rfidLogCount' => $rfidLogs->count(['machine' => $machine]),
            'usageLogCount' => $usageLogs->count(['machine' => $machine]),
            'reservationCount' => $reservations->count(['machine' => $machine]),
        ]);
    }

    #[Route('/machines/{id}/calendrier', name: 'app_machine_calendar', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/calendar', name: 'app_machine_calendar_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendrier', name: 'app_machine_calendar_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/calendar', name: 'app_machine_calendar_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-calendrier.html', name: 'app_machine_calendar_html', methods: ['GET'])]
    public function machineCalendar(Request $request, MachineRepository $machines, ReservationRepository $reservations, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        return $this->render('site/machine-calendrier.html.twig', [
            'machine' => $machine,
            'machines' => $machines->findBy([], ['nom' => 'ASC']),
            'reservations' => $reservations->findActiveByMachine($machine, ['dateDebut' => 'ASC']),
        ]);
    }

    #[Route('/machines/{id}/historique', name: 'app_machine_history', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machines/{id}/history', name: 'app_machine_history_legacy_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/historique', name: 'app_machine_history_legacy_singular', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine/{id}/history', name: 'app_machine_history_legacy_singular_en', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/machine-historique.html', name: 'app_machine_history_html', methods: ['GET'])]
    public function machineHistory(Request $request, MachineRepository $machines, AccessRfidLogRepository $rfidLogs, LogUtilisationRepository $usageLogs, ReservationRepository $reservations, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $machine = $machines->find($id);
        if (!$machine) {
            throw $this->createNotFoundException('Machine introuvable');
        }

        return $this->render('site/machine-historique.html.twig', [
            'machine' => $machine,
            'rfidLogs' => $rfidLogs->findBy(['machine' => $machine], ['createdAt' => 'DESC']),
            'usageLogs' => $usageLogs->findBy(['machine' => $machine], ['dateDebut' => 'DESC']),
            'reservations' => $reservations->findBy(['machine' => $machine], ['dateDebut' => 'DESC']),
        ]);
    }

    #[Route('/formations', name: 'app_formations', methods: ['GET'])]
    #[Route('/formations.html', name: 'app_formations_html', methods: ['GET'])]
    public function formations(FormationRepository $formations, ProgressionRepository $progressions): Response
    {
        $progressionStats = $this->buildFormationProgressionStats($progressions);

        $formationItems = $formations->findBy([], ['id' => 'DESC']);

        return $this->render('site/formations.html.twig', [
            'formations' => $formationItems,
            'formationVisuals' => $this->buildFormationVisuals($formationItems),
            'progressionStats' => $progressionStats,
        ]);
    }

    #[Route('/formations/{id}', name: 'app_formation_detail', requirements: ['id' => '\\d+'], methods: ['GET'])]
    #[Route('/formation-detail.html', name: 'app_formation_detail_html', methods: ['GET'])]
    public function formationDetail(Request $request, FormationRepository $formations, ProgressionRepository $progressions, ?int $id = null): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $formation = $formations->find($id);
        if (!$formation) {
            throw $this->createNotFoundException('Formation introuvable');
        }

        return $this->render('site/formation-detail.html.twig', [
            'formation' => $formation,
            'formationVisual' => $this->buildFormationVisual($formation),
            'progressions' => $progressions->findBy(['formation' => $formation], ['dateDebut' => 'DESC']),
            'stats' => $this->buildFormationProgressionStats($progressions)[$formation->getId()] ?? ['total' => 0, 'completed' => 0, 'incomplete' => 0],
        ]);
    }

    #[Route('/formations/{id}/suivi', name: 'app_formation_follow', requirements: ['id' => '\d+'], methods: ['GET'])]
    #[Route('/formation-suivi.html', name: 'app_formation_follow_html', methods: ['GET'])]
    public function formationFollow(
        Request $request,
        FormationRepository $formations,
        ProgressionRepository $progressions,
        SectionRepository $sections,
        QuizRepository $quizzes,
        QuestionRepository $questions,
        ChoixRepository $choices,
        ?int $id = null,
    ): Response
    {
        $id ??= max(1, (int) $request->query->get('id', 1));
        $formation = $formations->find($id);
        if (!$formation) {
            throw $this->createNotFoundException('Formation introuvable');
        }

        $formationSections = $sections->findBy(['formation' => $formation], ['ordre' => 'ASC']);
        $formationQuizzes = $quizzes->findBy(['formation' => $formation], ['id' => 'ASC']);

        $quizQuestions = [];
        $questionChoices = [];
        foreach ($formationQuizzes as $quiz) {
            $quizQuestions[$quiz->getId()] = $questions->findBy(['quiz' => $quiz], ['ordre' => 'ASC']);
            foreach ($quizQuestions[$quiz->getId()] as $question) {
                $questionChoices[$question->getId()] = $choices->findBy(['question' => $question], ['ordre' => 'ASC']);
            }
        }

        return $this->render('site/formation-suivi.html.twig', [
            'formation' => $formation,
            'formationVisual' => $this->buildFormationVisual($formation),
            'progressions' => $progressions->findBy(['formation' => $formation], ['dateDebut' => 'DESC']),
            'sections' => $formationSections,
            'quizzes' => $formationQuizzes,
            'quizQuestions' => $quizQuestions,
            'questionChoices' => $questionChoices,
        ]);
    }

    #[Route('/leaderboard', name: 'app_leaderboard', methods: ['GET'])]
    #[Route('/leaderboard.html', name: 'app_leaderboard_html', methods: ['GET'])]
    public function leaderboard(
        UtilisateurRepository $users,
        UtilisateurBadgeRepository $userBadges,
        ProgressionRepository $progressions,
        LogUtilisationRepository $usageLogs,
        MachineRepository $machines,
        FormationRepository $formations,
        ReservationRepository $reservations,
    ): Response
    {
        $items = $users->findBy([], ['tempsPresenceTotal' => 'DESC'], 20);
        $leaderboard = [];

        foreach ($items as $index => $user) {
            $userUsageLogs = $usageLogs->findBy(['utilisateur' => $user]);
            $machineIds = [];

            foreach ($userUsageLogs as $usageLog) {
                $machine = $usageLog->getMachine();
                if ($machine?->getId() !== null) {
                    $machineIds[$machine->getId()] = true;
                }
            }

            $leaderboard[] = [
                'rank' => $index + 1,
                'user' => $user,
                'badges' => $userBadges->count(['utilisateur' => $user]),
                'completedFormations' => $progressions->count(['utilisateur' => $user, 'completed' => true]),
                'machinesUsed' => count($machineIds),
            ];
        }

        return $this->render('site/leaderboard.html.twig', [
            'leaderboard' => $leaderboard,
            'stats' => [
                'users' => $users->count([]),
                'machines' => $machines->count([]),
                'formations' => $formations->count([]),
                'reservations' => $reservations->count([]),
            ],
        ]);
    }

    #[Route('/register', name: 'app_register', methods: ['GET'])]
    #[Route('/register.html', name: 'app_register_html', methods: ['GET'])]
    public function register(): Response
    {
        return $this->render('site/register.html.twig');
    }

    #[Route('/forgot-password', name: 'app_forgot_password', methods: ['GET'])]
    #[Route('/forgot-password.html', name: 'app_forgot_password_html', methods: ['GET'])]
    public function forgotPassword(): Response
    {
        return $this->render('site/forgot-password.html.twig');
    }

    #[Route('/profil', name: 'app_profile', methods: ['GET', 'POST'])]
    #[Route('/profil.html', name: 'app_profile_html', methods: ['GET'])]
    public function profile(
        Request $request,
        EntityManagerInterface $entityManager,
        ProgressionRepository $progressions,
        UtilisateurBadgeRepository $userBadges,
        ReservationRepository $reservations,
        AccessRfidLogRepository $rfidLogs,
        LogUtilisationRepository $usageLogs,
    ): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur) {
            throw $this->createAccessDeniedException('Utilisateur connecte requis');
        }

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('profile_preferences', (string) $request->request->get('_token'))) {
                $this->addFlash('error', 'La sauvegarde des preferences a ete refusee. Rechargez la page puis reessayez.');

                return $this->redirectToRoute('app_profile');
            }

            $theme = (string) $request->request->get('theme', '');
            $langue = (string) $request->request->get('langue', '');

            if (!in_array($theme, ['light', 'dark', 'system'], true)) {
                $this->addFlash('error', 'Theme invalide.');

                return $this->redirectToRoute('app_profile');
            }

            if (!in_array($langue, ['fr', 'en'], true)) {
                $this->addFlash('error', 'Langue invalide.');

                return $this->redirectToRoute('app_profile');
            }

            $user
                ->setNotificationEmail($request->request->has('notificationEmail'))
                ->setNotificationPush($request->request->has('notificationPush'))
                ->setRappelReservation($request->request->has('rappelReservation'))
                ->setTheme($theme)
                ->setLangue($langue);

            $entityManager->flush();
            $this->addFlash('success', 'Preferences mises a jour.');

            return $this->redirectToRoute('app_profile');
        }

        $userProgressions = $progressions->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']);
        $completedProgressions = array_values(array_filter($userProgressions, static fn ($progression): bool => $progression->isCompleted()));
        $userUsageLogs = $usageLogs->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']);
        $userRfidLogs = $rfidLogs->findBy(['utilisateur' => $user], ['createdAt' => 'DESC']);

        $machineIds = [];
        foreach ($userUsageLogs as $usageLog) {
            $machine = $usageLog->getMachine();
            if ($machine?->getId() !== null) {
                $machineIds[$machine->getId()] = true;
            }
        }
        foreach ($userRfidLogs as $rfidLog) {
            $machine = $rfidLog->getMachine();
            if ($machine?->getId() !== null) {
                $machineIds[$machine->getId()] = true;
            }
        }

        return $this->render('site/profil.html.twig', [
            'user' => $user,
            'progressions' => $userProgressions,
            'completedProgressions' => $completedProgressions,
            'userBadges' => $userBadges->findBy(['utilisateur' => $user], ['dateObtention' => 'DESC']),
            'reservations' => $reservations->findBy(['utilisateur' => $user], ['dateDebut' => 'DESC']),
            'rfidLogs' => $userRfidLogs,
            'usageLogs' => $userUsageLogs,
            'profileStats' => [
                'completedFormations' => count($completedProgressions),
                'badges' => $userBadges->count(['utilisateur' => $user]),
                'reservations' => $reservations->count(['utilisateur' => $user]),
                'rfidLogs' => count($userRfidLogs),
                'usageLogs' => count($userUsageLogs),
                'machinesUsed' => count($machineIds),
            ],
        ]);
    }


    #[Route('/profil/password', name: 'app_profile_password', methods: ['GET', 'POST'])]
    public function profilePassword(
        Request $request,
        EntityManagerInterface $entityManager,
        UserPasswordHasherInterface $passwordHasher,
    ): Response
    {
        $user = $this->getUser();
        if (!$user instanceof Utilisateur || !$user instanceof PasswordAuthenticatedUserInterface) {
            throw $this->createAccessDeniedException('Utilisateur connecte requis');
        }

        $errors = [];

        if ($request->isMethod('POST')) {
            if (!$this->isCsrfTokenValid('profile_password', (string) $request->request->get('_token'))) {
                $errors[] = 'La demande de changement de mot de passe a ete refusee. Rechargez la page puis reessayez.';
            }

            $currentPassword = (string) $request->request->get('currentPassword', '');
            $newPassword = (string) $request->request->get('newPassword', '');
            $confirmPassword = (string) $request->request->get('confirmPassword', '');

            if ($currentPassword === '') {
                $errors[] = 'Le mot de passe actuel est obligatoire.';
            } elseif (!$passwordHasher->isPasswordValid($user, $currentPassword)) {
                $errors[] = 'Le mot de passe actuel est incorrect.';
            }

            if ($newPassword === '') {
                $errors[] = 'Le nouveau mot de passe est obligatoire.';
            } elseif (mb_strlen($newPassword) < 8) {
                $errors[] = 'Le nouveau mot de passe doit contenir au moins 8 caracteres.';
            }

            if ($confirmPassword === '') {
                $errors[] = 'La confirmation du nouveau mot de passe est obligatoire.';
            } elseif ($newPassword !== $confirmPassword) {
                $errors[] = 'Le nouveau mot de passe et sa confirmation doivent etre identiques.';
            }

            if ($newPassword !== '' && $currentPassword !== '' && $newPassword === $currentPassword) {
                $errors[] = 'Le nouveau mot de passe doit etre different du mot de passe actuel.';
            }

            if ($errors === []) {
                $user->setPassword($passwordHasher->hashPassword($user, $newPassword));
                $entityManager->flush();

                $this->addFlash('success', 'Mot de passe mis a jour.');

                return $this->redirectToRoute('app_profile');
            }
        }

        return $this->render('site/profile-password.html.twig', [
            'user' => $user,
            'errors' => $errors,
        ]);
    }

    #[Route('/search', name: 'app_search', methods: ['GET'])]
    #[Route('/search.html', name: 'app_search_html', methods: ['GET'])]
    public function search(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges);
    }

    #[Route('/recherche', name: 'app_recherche', methods: ['GET'])]
    #[Route('/recherche.html', name: 'app_recherche_html', methods: ['GET'])]
    public function recherche(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        return $this->renderSearchPage($request, $users, $machines, $formations, $badges);
    }

    private function renderSearchPage(Request $request, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): Response
    {
        $query = trim((string) $request->query->get('q', ''));
        $categories = $this->buildSearchCategories($query, $users, $machines, $formations, $badges);

        return $this->render('site/search.html.twig', [
            'query' => $query,
            'categories' => $categories,
            'totalResults' => array_sum(array_map('count', $categories)),
        ]);
    }

    private function buildSearchCategories(string $query, UtilisateurRepository $users, MachineRepository $machines, FormationRepository $formations, BadgeRepository $badges): array
    {
        $categories = [
            'Utilisateurs' => [],
            'Machines' => [],
            'Formations' => [],
            'Badges' => [],
        ];
        $needle = mb_strtolower(trim($query));
        if ($needle === '') {
            return $categories;
        }

        if ($this->isGranted('ROLE_ADMIN')) {
            foreach ($users->findAll() as $user) {
                $haystack = mb_strtolower(implode(' ', [
                    $user->getFirstName() ?? '',
                    $user->getLastName() ?? '',
                    $user->getUsername(),
                    $user->getEmail(),
                    $user->getIdentifiantRfid() ?? '',
                    $user->getNumeroId() ?? '',
                ]));
                if (str_contains($haystack, $needle)) {
                    $categories['Utilisateurs'][] = [
                        'title' => $user->getDisplayName(),
                        'description' => trim($user->getEmail() . ' ' . ($user->getIdentifiantRfid() ? '- RFID ' . $user->getIdentifiantRfid() : '')),
                        'meta' => $user->getUsername(),
                        'url' => $this->generateUrl('app_admin_user_detail', ['id' => $user->getId()]),
                    ];
                }
            }
        }

        foreach ($machines->findAll() as $machine) {
            $haystack = mb_strtolower(implode(' ', [
                $machine->getNom(),
                $machine->getDescription() ?? '',
                $machine->getLocalisation() ?? '',
                $machine->getMachineToken(),
            ]));
            if (str_contains($haystack, $needle)) {
                $categories['Machines'][] = [
                    'title' => $machine->getNom(),
                    'description' => $machine->getDescription() ?: 'Sans description',
                    'meta' => trim(($machine->getLocalisation() ?: 'Localisation non renseignée') . ' - ' . $machine->getStatut()),
                    'url' => $this->generateUrl('app_machine_detail', ['id' => $machine->getId()]),
                ];
            }
        }

        foreach ($formations->findAll() as $formation) {
            $haystack = mb_strtolower($formation->getTitre() . ' ' . ($formation->getDescription() ?? ''));
            if (str_contains($haystack, $needle)) {
                $categories['Formations'][] = [
                    'title' => $formation->getTitre(),
                    'description' => $formation->getDescription() ?: 'Sans description',
                    'meta' => $formation->getBadge()?->getNom() ? 'Badge : ' . $formation->getBadge()->getNom() : 'Aucun badge associé',
                    'url' => $this->generateUrl('app_formation_detail', ['id' => $formation->getId()]),
                ];
            }
        }

        foreach ($badges->findAll() as $badge) {
            $haystack = mb_strtolower(implode(' ', [$badge->getNom(), $badge->getDescription() ?? '', $badge->getIcone() ?? '']));
            if (str_contains($haystack, $needle)) {
                $categories['Badges'][] = [
                    'title' => $badge->getNom(),
                    'description' => $badge->getDescription() ?: 'Sans description',
                    'meta' => $badge->getIcone() ? 'Icône : ' . $badge->getIcone() : 'Icône non renseignée',
                    'url' => $this->generateUrl('app_admin_badges'),
                ];
            }
        }

        return $categories;
    }

    /**
     * @param iterable<Formation> $formations
     * @return array<int, array{type: string, icon: string, src: ?string, label: string}>
     */
    private function buildFormationVisuals(iterable $formations): array
    {
        $visuals = [];
        foreach ($formations as $formation) {
            if ($formation->getId() !== null) {
                $visuals[$formation->getId()] = $this->buildFormationVisual($formation);
            }
        }

        return $visuals;
    }

    /** @return array{type: string, icon: string, src: ?string, label: string} */
    private function buildFormationVisual(Formation $formation): array
    {
        $image = trim((string) $formation->getImage());
        if ($image !== '') {
            if (preg_match('#^(https?://|/|images/|uploads/)#', $image) === 1) {
                return ['type' => 'image', 'icon' => 'generic', 'src' => $image, 'label' => $formation->getTitre()];
            }

            return ['type' => 'icon', 'icon' => $this->normalizeFormationIconSlug($image), 'src' => null, 'label' => $formation->getTitre()];
        }


        return ['type' => 'icon', 'icon' => 'generic', 'src' => null, 'label' => $formation->getTitre()];
    }

    private function inferMachineIconSlug(string $token, string $name, string $fallback): string
    {
        $haystack = mb_strtolower($token . ' ' . $name);
        if (str_contains($haystack, 'laser')) {
            return 'laser';
        }
        if (str_contains($haystack, 'soudure')) {
            return 'soldering';
        }
        if (str_contains($haystack, 'vinyl') || str_contains($haystack, 'vinyle')) {
            return 'vinyl';
        }
        if (str_contains($haystack, 'cnc') || str_contains($haystack, 'fraiseuse')) {
            return 'cnc';
        }
        if (str_contains($haystack, 'oscilloscope')) {
            return 'oscilloscope';
        }
        if (str_contains($haystack, 'brodeuse')) {
            return 'embroidery';
        }

        return $this->normalizeFormationIconSlug($fallback);
    }

    private function normalizeFormationIconSlug(string $slug): string
    {
        return match ($slug) {
            'printer', 'printer-3d', 'impression-3d' => 'printer-3d',
            'decoupe', 'laser-co2' => 'laser',
            'electronique' => 'soldering',
            'textile' => 'embroidery',
            default => $slug,
        };
    }

    private function buildFormationProgressionStats(ProgressionRepository $progressions): array
    {
        $stats = [];
        foreach ($progressions->findAll() as $progression) {
            $formation = $progression->getFormation();
            if (!$formation || !$formation->getId()) {
                continue;
            }

            $id = $formation->getId();
            $stats[$id] ??= ['total' => 0, 'completed' => 0, 'incomplete' => 0];
            $stats[$id]['total']++;
            $progression->isCompleted() ? $stats[$id]['completed']++ : $stats[$id]['incomplete']++;
        }

        return $stats;
    }
}
