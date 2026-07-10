<?php

namespace App\Entity;

use App\Repository\HomepageSectionVisibilityRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: HomepageSectionVisibilityRepository::class)]
#[ORM\Table(name: 'HOMEPAGE_SECTION_VISIBILITY')]
#[ORM\UniqueConstraint(name: 'uniq_homepage_section_visibility_key', columns: ['sectionKey'])]
class HomepageSectionVisibility
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(name: 'sectionKey', length: 100)]
    private string $sectionKey = '';

    #[ORM\Column(length: 150)]
    private string $label = '';

    #[ORM\Column(name: 'visibleAnonymous', options: ['default' => false])]
    private bool $visibleAnonymous = false;

    #[ORM\Column(name: 'visibleUser', options: ['default' => false])]
    private bool $visibleUser = false;

    #[ORM\Column(name: 'visibleStaff', options: ['default' => false])]
    private bool $visibleStaff = false;

    #[ORM\Column(name: 'visibleAdmin', options: ['default' => true])]
    private bool $visibleAdmin = true;

    #[ORM\Column(name: 'sortOrder', options: ['default' => 0])]
    private int $sortOrder = 0;

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    public function getId(): ?int { return $this->id; }
    public function getSectionKey(): string { return $this->sectionKey; }
    public function setSectionKey(string $sectionKey): self { $this->sectionKey = $sectionKey; return $this; }
    public function getLabel(): string { return $this->label; }
    public function setLabel(string $label): self { $this->label = $label; return $this; }
    public function isVisibleAnonymous(): bool { return $this->visibleAnonymous; }
    public function setVisibleAnonymous(bool $visibleAnonymous): self { $this->visibleAnonymous = $visibleAnonymous; return $this; }
    public function isVisibleUser(): bool { return $this->visibleUser; }
    public function setVisibleUser(bool $visibleUser): self { $this->visibleUser = $visibleUser; return $this; }
    public function isVisibleStaff(): bool { return $this->visibleStaff; }
    public function setVisibleStaff(bool $visibleStaff): self { $this->visibleStaff = $visibleStaff; return $this; }
    public function isVisibleAdmin(): bool { return $this->visibleAdmin; }
    public function setVisibleAdmin(bool $visibleAdmin): self { $this->visibleAdmin = $visibleAdmin; return $this; }
    public function getSortOrder(): int { return $this->sortOrder; }
    public function setSortOrder(int $sortOrder): self { $this->sortOrder = $sortOrder; return $this; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }
}
