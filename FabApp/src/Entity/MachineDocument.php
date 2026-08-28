<?php

namespace App\Entity;

use App\Repository\MachineDocumentRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * 🔴 **Les noms de colonnes sont DÉCLARÉS, et il le faut.** `doctrine.yaml` pose
 * `naming_strategy: underscore` : sans `name:`, Doctrine cherche `stored_name`
 * quand la table dit `storedName`, et la page rend un « Unknown column » — ce qui
 * est exactement arrivé au premier déploiement de cette entité. Le reste du
 * schéma est en camelCase (`machineId`, `settingKey`, `machineToken`…), donc une
 * table neuve le suit, et l'entité le dit explicitement plutôt que de compter sur
 * une stratégie qui pense l'inverse. Voir [[feedback-fabos-schema-drift]].
 *
 * Un document attaché à une machine — guide d'usage, fiche de sécurité… (S152)
 *
 * Demandé par l'opérateur le 2026-08-27 : *« add on the machine pages files to
 * download related to each machine, example: usage guide, safety sheet, etc. »*
 *
 * ⚠️ **Le fichier n'est PAS stocké ici, seulement son nom.** L'octet vit dans
 * `public/uploads/machine-documents/`, comme les images des pages du lab
 * (`AdminController` ~3282). La table dit qu'un document existe, ce qu'il
 * s'appelle pour un humain, et sous quel nom il a été rangé sur le disque.
 *
 * ⚠️ **Deux noms, et les deux comptent.** `storedName` est ce que le disque
 * connaît — horodaté et slugifié, donc sans collision et sans surprise pour le
 * système de fichiers. `originalName` est ce que l'opérateur a téléversé, et
 * c'est ce que le navigateur doit proposer au téléchargement : recevoir
 * `guide-2026-08-28-a1b2c3.pdf` quand on a cliqué « Guide d'usage » est une
 * petite trahison, répétée à chaque téléchargement.
 *
 * 🔴 **`mimeType` est celui que le SERVEUR a constaté, pas celui que le client a
 * annoncé.** Un `.pdf` renommé reste ce qu'il est, et c'est cette valeur-là qui
 * décide de l'icône et de l'en-tête de réponse.
 */
#[ORM\Entity(repositoryClass: MachineDocumentRepository::class)]
#[ORM\Table(name: 'MACHINE_DOCUMENT')]
#[ORM\Index(name: 'IDX_MACHINE_DOCUMENT_MACHINE', columns: ['machineId'])]
class MachineDocument
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    /**
     * ⚠️ `onDelete: CASCADE` : un document sans machine n'est plus un document,
     * c'est un fichier orphelin dont plus rien ne dit à quoi il servait.
     */
    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    /** Ce que le membre lit : « Guide d'usage », « Fiche de sécurité »… */
    #[ORM\Column(length: 180)]
    private string $label = '';

    #[ORM\Column(name: 'storedName', length: 255)]
    private string $storedName = '';

    #[ORM\Column(name: 'originalName', length: 255)]
    private string $originalName = '';

    #[ORM\Column(name: 'mimeType', length: 120)]
    private string $mimeType = '';

    #[ORM\Column(name: 'sizeBytes')]
    private int $sizeBytes = 0;

    /**
     * L'ordre voulu par l'opérateur. ⚠️ Une fiche de sécurité se lit AVANT un
     * guide d'usage ; l'ordre d'ajout ne le sait pas.
     */
    #[ORM\Column]
    private int $position = 0;

    #[ORM\Column(name: 'uploadedAt')]
    private \DateTimeImmutable $uploadedAt;

    public function __construct()
    {
        $this->uploadedAt = new \DateTimeImmutable();
    }

    public function getId(): ?int
    {
        return $this->id;
    }

    public function getMachine(): ?Machine
    {
        return $this->machine;
    }

    public function setMachine(?Machine $machine): self
    {
        $this->machine = $machine;

        return $this;
    }

    public function getLabel(): string
    {
        return $this->label;
    }

    public function setLabel(string $label): self
    {
        $this->label = $label;

        return $this;
    }

    public function getStoredName(): string
    {
        return $this->storedName;
    }

    public function setStoredName(string $storedName): self
    {
        $this->storedName = $storedName;

        return $this;
    }

    public function getOriginalName(): string
    {
        return $this->originalName;
    }

    public function setOriginalName(string $originalName): self
    {
        $this->originalName = $originalName;

        return $this;
    }

    public function getMimeType(): string
    {
        return $this->mimeType;
    }

    public function setMimeType(string $mimeType): self
    {
        $this->mimeType = $mimeType;

        return $this;
    }

    public function getSizeBytes(): int
    {
        return $this->sizeBytes;
    }

    public function setSizeBytes(int $sizeBytes): self
    {
        $this->sizeBytes = $sizeBytes;

        return $this;
    }

    public function getPosition(): int
    {
        return $this->position;
    }

    public function setPosition(int $position): self
    {
        $this->position = $position;

        return $this;
    }

    public function getUploadedAt(): \DateTimeImmutable
    {
        return $this->uploadedAt;
    }

    /**
     * Le type en un mot, pour l'icône et pour la ligne « PDF · 2,3 Mo ».
     *
     * ⚠️ Dérivé du `mimeType` constaté, jamais de l'extension du nom : c'est
     * précisément ce qu'un fichier renommé fait mentir.
     */
    public function getKind(): string
    {
        return match (true) {
            $this->mimeType === 'application/pdf' => 'pdf',
            str_starts_with($this->mimeType, 'image/') => 'image',
            str_contains($this->mimeType, 'word') || str_contains($this->mimeType, 'opendocument.text') => 'doc',
            str_contains($this->mimeType, 'sheet') || str_contains($this->mimeType, 'excel') => 'sheet',
            str_contains($this->mimeType, 'zip') || str_contains($this->mimeType, 'compressed') => 'archive',
            default => 'file',
        };
    }
}
