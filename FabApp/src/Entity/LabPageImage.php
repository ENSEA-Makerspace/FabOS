<?php

namespace App\Entity;

use App\Repository\LabPageImageRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: LabPageImageRepository::class)]
#[ORM\Table(name: 'LAB_PAGE_IMAGE')]
class LabPageImage
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: LabPage::class, inversedBy: 'images')]
    #[ORM\JoinColumn(name: 'labPageId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?LabPage $labPage = null;

    #[ORM\Column(name: 'imageFilename', length: 255)]
    private string $imageFilename = '';

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getLabPage(): ?LabPage { return $this->labPage; }
    public function setLabPage(?LabPage $labPage): self { $this->labPage = $labPage; return $this; }
    public function getImageFilename(): string { return $this->imageFilename; }
    public function setImageFilename(string $imageFilename): self { $this->imageFilename = $imageFilename; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
