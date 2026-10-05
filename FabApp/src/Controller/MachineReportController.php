<?php

declare(strict_types=1);

namespace App\Controller;

use App\Entity\Machine;
use App\Feature\SiteFeatureService;
use App\Image\ImageNormalizer;
use App\Repository\MachineRepository;
use App\Service\MachineReportNotifier;
use App\Service\MachineReports;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\File\Exception\FileException;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * S205 — « Signaler une panne » : la page PUBLIQUE, sans compte.
 *
 * `/signaler` (choisir la machine) et `/signaler/{id}` (machine déjà choisie,
 * c'est l'adresse du QR collé dessus). Rien ici n'est derrière le pare-feu : la
 * défense est donc faite à la main, et chaque ligne est là pour une raison :
 *
 *  - jeton CSRF (une page publique reste une page avec un POST) ;
 *  - champ « pot de miel » : un robot qui le remplit reçoit le même « merci »
 *    qu'une personne, et rien n'est enregistré — il n'apprend pas qu'il a été vu ;
 *  - limite de débit PAR ADRESSE (empreinte salée, jamais l'adresse) et globale,
 *    comptée dans la table elle-même (le projet n'a pas de limiteur de débit
 *    configuré hors de la connexion) ;
 *  - tout est borné : longueur du texte et du contact, type et poids de l'image.
 *
 * ⚠️ Un signalement ne change PAS l'état de la machine : c'est l'équipe qui décide.
 * ⚠️ Éteint (clé `machine_reports`) ou migration absente : 404, comme une page
 * qui n'existe pas.
 */
final class MachineReportController extends AbstractController
{
    private const MAX_PHOTO_BYTES = 12_582_912; // 12 Mo : ce que fait un téléphone récent
    private const PHOTO_EDGE = 1600;
    private const PER_ADDRESS_10_MIN = 3;
    private const PER_ADDRESS_DAY = 10;
    private const GLOBAL_HOUR = 60;

    public function __construct(
        private readonly SiteFeatureService $features,
        private readonly MachineReports $reports,
        private readonly MachineRepository $machines,
    ) {
    }

    #[Route('/signaler', name: 'app_report_choose', methods: ['GET'])]
    public function choose(): Response
    {
        $this->guard();
        $live = $this->machines->findLive();

        // Une seule machine : le choix est déjà fait.
        if (\count($live) === 1) {
            return $this->redirectToRoute('app_report_machine', ['id' => $live[0]->getId()]);
        }

        return $this->render('site/machine-report-choose.html.twig', ['machines' => $live]);
    }

    #[Route('/signaler/merci', name: 'app_report_thanks', methods: ['GET'])]
    public function thanks(): Response
    {
        $this->guard();

        return $this->render('site/machine-report-thanks.html.twig');
    }

    #[Route('/signaler/{id<\d+>}', name: 'app_report_machine', methods: ['GET', 'POST'])]
    public function machine(int $id, Request $request, ImageNormalizer $images, MachineReportNotifier $notifier): Response
    {
        $this->guard();
        $machine = $this->machines->find($id);
        if (!$machine instanceof Machine || $machine->isArchived()) {
            throw $this->createNotFoundException();
        }

        if (!$request->isMethod('POST')) {
            return $this->form($machine);
        }

        // Pot de miel : un champ que seul un robot remplit.
        if (trim((string) $request->request->get('website')) !== '') {
            return $this->redirectToRoute('app_report_thanks');
        }

        $description = trim((string) $request->request->get('description'));
        $contact = trim((string) $request->request->get('contact'));
        $keep = ['description' => $description, 'contact' => $contact];

        if (!$this->isCsrfTokenValid('machine_report_' . $id, (string) $request->request->get('_token'))) {
            return $this->form($machine, 'machine_report.err_csrf', $keep, Response::HTTP_FORBIDDEN);
        }

        $hash = $this->addressHash($request);
        $now = new \DateTimeImmutable();
        if ($this->reports->countSince($now->modify('-10 minutes'), $hash) >= self::PER_ADDRESS_10_MIN
            || $this->reports->countSince($now->modify('-1 day'), $hash) >= self::PER_ADDRESS_DAY
            || $this->reports->countSince($now->modify('-1 hour')) >= self::GLOBAL_HOUR) {
            return $this->form($machine, 'machine_report.err_rate', $keep, Response::HTTP_TOO_MANY_REQUESTS);
        }

        $length = mb_strlen($description);
        if ($length < MachineReports::DESCRIPTION_MIN || $length > MachineReports::DESCRIPTION_MAX || mb_strlen($contact) > MachineReports::CONTACT_MAX) {
            return $this->form($machine, 'machine_report.err_description', $keep, Response::HTTP_UNPROCESSABLE_ENTITY);
        }

        $photo = null;
        $upload = $request->files->get('photo');
        if ($upload instanceof UploadedFile && $upload->getError() !== \UPLOAD_ERR_NO_FILE) {
            $photo = $this->storePhoto($upload, $images);
            if ($photo === null) {
                return $this->form($machine, 'machine_report.err_photo', $keep, Response::HTTP_UNPROCESSABLE_ENTITY);
            }
        }

        $reportId = $this->reports->create($id, $description, $photo, $contact, $hash);
        if ($reportId === null) {
            throw $this->createNotFoundException();
        }

        $notifier->notify(
            $machine->getNom(),
            $description,
            $contact !== '' ? $contact : null,
            $this->generateUrl('app_admin_machine_reports', ['statut' => 'open'], UrlGeneratorInterface::ABSOLUTE_URL),
        );

        return $this->redirectToRoute('app_report_thanks');
    }

    /** Éteint, ou migration pas passée : la page n'existe pas. */
    private function guard(): void
    {
        if (!$this->features->allowsSurface('machine_reports') || !$this->features->allowsSurface('machines') || !$this->reports->isReady()) {
            throw $this->createNotFoundException();
        }
    }

    /** @param array<string, string> $keep */
    private function form(Machine $machine, ?string $error = null, array $keep = [], int $status = Response::HTTP_OK): Response
    {
        return $this->render('site/machine-report.html.twig', [
            'machine' => $machine,
            'error' => $error,
            'description' => $keep['description'] ?? '',
            'contact' => $keep['contact'] ?? '',
            'maxLength' => MachineReports::DESCRIPTION_MAX,
        ], new Response('', $status));
    }

    /** Empreinte salée de l'adresse : sert à compter, jamais à retrouver qui. */
    private function addressHash(Request $request): string
    {
        return hash_hmac('sha256', (string) $request->getClientIp(), (string) $this->getParameter('kernel.secret'));
    }

    /**
     * Même discipline que les autres envois d'image (type lu dans le contenu,
     * nom aléatoire, `ImageNormalizer` redresse l'EXIF et réduit) — mais ici le
     * déposant est un inconnu : on refuse tout ce qui n'est pas une vraie image
     * JPEG, PNG ou WebP, et au-delà du poids permis.
     *
     * @return string|null le nom du fichier rangé, ou null si l'image est refusée
     */
    private function storePhoto(UploadedFile $file, ImageNormalizer $images): ?string
    {
        if (!$file->isValid() || $file->getSize() > self::MAX_PHOTO_BYTES) {
            return null;
        }
        $extension = match ($file->getMimeType()) {
            'image/jpeg' => 'jpg',
            'image/png' => 'png',
            'image/webp' => 'webp',
            default => null,
        };
        if ($extension === null || @getimagesize($file->getPathname()) === false) {
            return null;
        }

        $directory = $this->getParameter('kernel.project_dir') . '/public/uploads/machine-reports';
        if (!is_dir($directory) && !@mkdir($directory, 0775, true) && !is_dir($directory)) {
            return null;
        }

        // ⚠️ Le nom vient de ce que `capUploaded` a RÉELLEMENT écrit (un PNG sans
        // transparence ressort en JPEG), jamais de ce que le navigateur a annoncé.
        $extension = $images->capUploaded($file->getPathname(), $extension, self::PHOTO_EDGE);
        $name = sprintf('report-%s.%s', bin2hex(random_bytes(8)), $extension);
        try {
            $file->move($directory, $name);
        } catch (FileException) {
            return null;
        }

        return $name;
    }
}
