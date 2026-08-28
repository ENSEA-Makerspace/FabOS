<?php

namespace App\Controller;

use App\Entity\Machine;
use App\Entity\MachineDocument;
use App\Repository\MachineDocumentRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Bundle\FrameworkBundle\Controller\AbstractController;
use Symfony\Component\HttpFoundation\BinaryFileResponse;
use Symfony\Component\HttpFoundation\File\UploadedFile;
use Symfony\Component\HttpFoundation\Request;
use Symfony\Component\HttpFoundation\Response;
use Symfony\Component\HttpFoundation\ResponseHeaderBag;
use Symfony\Component\Routing\Attribute\Route;
use Symfony\Component\String\Slugger\SluggerInterface;

/**
 * Les documents attachés à une machine (S152).
 *
 * Demandé par l'opérateur le 2026-08-27 : un guide d'usage, une fiche de
 * sécurité, à téléverser côté admin et à consulter depuis la fiche publique.
 *
 * 🔴 **Ces fichiers sont PUBLICS, et il faut le savoir en les téléversant.** Ils
 * vivent sous `public/uploads/`, donc leur URL suffit à les lire — la route de
 * téléchargement ci-dessous n'ajoute pas une serrure, elle ajoute un joli nom de
 * fichier. C'est le bon défaut pour une fiche de sécurité, qui doit se lire AVANT
 * d'être formé et non après ; ça ne l'est pas pour un document interne, et
 * l'écran d'admin le dit en toutes lettres.
 * 🅿️ Si un jour un document doit être réservé aux membres, il faudra le sortir de
 * `public/` — un contrôle d'accès devant un fichier que le serveur web sert
 * directement ne contrôle rien.
 *
 * ⚠️ **Le type est constaté, jamais annoncé.** `UploadedFile::getMimeType()` lit
 * le CONTENU (finfo) ; `getClientMimeType()` répète ce que le navigateur a bien
 * voulu dire. Un `.pdf` renommé reste ce qu'il est.
 */
final class MachineDocumentController extends AbstractController
{
    /**
     * Ce qu'on accepte, par type constaté → extension rangée sur le disque.
     *
     * ⚠️ Une liste blanche, pas une liste noire : l'inventaire de ce qui est
     * dangereux n'est jamais fini, celui de ce qui est utile tient en dix lignes.
     */
    private const ALLOWED = [
        'application/pdf' => 'pdf',
        'image/png' => 'png',
        'image/jpeg' => 'jpg',
        'image/webp' => 'webp',
        'text/plain' => 'txt',
        'application/msword' => 'doc',
        'application/vnd.openxmlformats-officedocument.wordprocessingml.document' => 'docx',
        'application/vnd.oasis.opendocument.text' => 'odt',
        'application/vnd.ms-excel' => 'xls',
        'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet' => 'xlsx',
        'application/vnd.oasis.opendocument.spreadsheet' => 'ods',
        'application/zip' => 'zip',
    ];

    private const MAX_BYTES = 20 * 1024 * 1024;

    #[Route('/admin/machines/{id}/documents', name: 'app_admin_machine_document_add', requirements: ['id' => '\d+'], methods: ['POST'])]
    public function add(
        Machine $machine,
        Request $request,
        EntityManagerInterface $entityManager,
        MachineDocumentRepository $documents,
        SluggerInterface $slugger,
    ): Response {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $back = $this->redirectToRoute('app_admin_machine_edit', ['id' => $machine->getId()]);

        if (!$this->isCsrfTokenValid('machine_documents_' . $machine->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.ajout_refuse_token_csrf_invalide');

            return $back;
        }

        $uploaded = $request->files->get('document');
        if (!$uploaded instanceof UploadedFile) {
            $this->addFlash('error', 'machine_documents.flash_no_file');

            return $back;
        }

        // ⚠️ La taille se vérifie AVANT le type : lire le contenu d'un fichier de
        // 400 Mo pour ensuite le refuser sur son type, c'est le lire pour rien.
        if ($uploaded->getSize() !== false && $uploaded->getSize() > self::MAX_BYTES) {
            $this->addFlash('error', 'machine_documents.flash_too_big');

            return $back;
        }

        $mime = (string) $uploaded->getMimeType();
        if (!isset(self::ALLOWED[$mime])) {
            $this->addFlash('error', 'machine_documents.flash_bad_type');

            return $back;
        }

        $label = trim((string) $request->request->get('label', ''));
        if ($label === '') {
            // ⚠️ Un document sans libellé prend le nom du fichier plutôt que d'être
            // refusé : l'opérateur a déjà choisi le fichier, lui redemander de
            // tout recommencer pour un champ qu'on peut deviner serait mesquin.
            $label = pathinfo((string) $uploaded->getClientOriginalName(), PATHINFO_FILENAME) ?: 'Document';
        }
        $label = mb_substr($label, 0, 180);

        $dir = $this->getParameter('kernel.project_dir') . '/public/uploads/machine-documents';
        if (!is_dir($dir) && !mkdir($dir, 0775, true) && !is_dir($dir)) {
            $this->addFlash('error', 'flash.impossible_de_creer_le_dossier_des');

            return $back;
        }

        // ⚠️ Le nom sur le disque est construit ICI, à partir du type CONSTATÉ —
        // jamais du nom que le navigateur a envoyé, qui peut contenir n'importe
        // quoi, y compris des `../`.
        $stored = sprintf(
            'machine-%d-%s.%s',
            $machine->getId(),
            bin2hex(random_bytes(6)),
            self::ALLOWED[$mime],
        );

        $original = (string) $uploaded->getClientOriginalName();
        $size = (int) ($uploaded->getSize() ?: 0);

        try {
            $uploaded->move($dir, $stored);
        } catch (\Throwable) {
            $this->addFlash('error', 'machine_documents.flash_move_failed');

            return $back;
        }

        $document = (new MachineDocument())
            ->setMachine($machine)
            ->setLabel($label)
            ->setStoredName($stored)
            // ⚠️ `basename` : ce nom est réaffiché et proposé au téléchargement.
            ->setOriginalName(mb_substr(basename($original) ?: $stored, 0, 255))
            ->setMimeType($mime)
            ->setSizeBytes($size)
            ->setPosition($documents->nextPosition($machine));

        $entityManager->persist($document);
        $entityManager->flush();

        $this->addFlash('success', 'machine_documents.flash_added');

        return $back;
    }

    #[Route('/admin/machines/{id}/documents/{documentId}/delete', name: 'app_admin_machine_document_delete', requirements: ['id' => '\d+', 'documentId' => '\d+'], methods: ['POST'])]
    public function delete(
        Machine $machine,
        int $documentId,
        Request $request,
        EntityManagerInterface $entityManager,
        MachineDocumentRepository $documents,
    ): Response {
        $this->denyAccessUnlessGranted('ROLE_ADMIN');

        $back = $this->redirectToRoute('app_admin_machine_edit', ['id' => $machine->getId()]);

        if (!$this->isCsrfTokenValid('machine_documents_' . $machine->getId(), (string) $request->request->get('_token'))) {
            $this->addFlash('error', 'flash.suppression_refusee_token_csrf');

            return $back;
        }

        $document = $documents->find($documentId);
        // ⚠️ On vérifie que le document appartient bien à CETTE machine : sans
        // ça, l'identifiant seul suffirait à supprimer le document d'une autre.
        if ($document === null || $document->getMachine()?->getId() !== $machine->getId()) {
            $this->addFlash('error', 'machine_documents.flash_not_found');

            return $back;
        }

        // ⚠️ **Le fichier est effacé, et c'est une décision.** Ailleurs dans
        // FabOS, retirer la ligne laisse l'octet sur le disque (les avatars le
        // font, `services.yaml` le dit). Ici l'octet est PUBLIC : le laisser,
        // c'est laisser une fiche de sécurité retirée toujours lisible par son
        // URL. `@unlink` parce qu'un fichier déjà absent n'est pas une erreur —
        // l'état voulu est « il n'est plus là ».
        $path = $this->getParameter('kernel.project_dir') . '/public/uploads/machine-documents/' . $document->getStoredName();
        if (is_file($path)) {
            @unlink($path);
        }

        $entityManager->remove($document);
        $entityManager->flush();

        $this->addFlash('success', 'machine_documents.flash_deleted');

        return $back;
    }

    /**
     * Le téléchargement, sous le nom que l'opérateur a téléversé.
     *
     * ⚠️ **Cette route n'est pas une serrure.** Le fichier est sous `public/` et
     * reste joignable par son URL directe ; ce que cette route ajoute, c'est un
     * `Content-Disposition` qui rend `Guide d'usage.pdf` plutôt que
     * `machine-7-a1b2c3.pdf`. Recevoir un nom qu'on n'a pas choisi est une petite
     * trahison, répétée à chaque téléchargement.
     */
    #[Route('/machines/{id}/documents/{documentId}', name: 'app_machine_document_download', requirements: ['id' => '\d+', 'documentId' => '\d+'], methods: ['GET'])]
    public function download(Machine $machine, int $documentId, MachineDocumentRepository $documents): Response
    {
        $document = $documents->find($documentId);
        if ($document === null || $document->getMachine()?->getId() !== $machine->getId()) {
            throw $this->createNotFoundException('Document introuvable.');
        }

        $path = $this->getParameter('kernel.project_dir') . '/public/uploads/machine-documents/' . $document->getStoredName();
        if (!is_file($path)) {
            throw $this->createNotFoundException('Fichier introuvable.');
        }

        $response = new BinaryFileResponse($path);
        $response->setContentDisposition(
            ResponseHeaderBag::DISPOSITION_ATTACHMENT,
            $document->getOriginalName(),
            // ⚠️ Le repli ASCII : un nom accentué doit rester téléchargeable par
            // un client qui ne sait pas lire l'en-tête étendu.
            preg_replace('/[^A-Za-z0-9._-]+/', '-', $document->getOriginalName()) ?: 'document',
        );

        return $response;
    }
}
