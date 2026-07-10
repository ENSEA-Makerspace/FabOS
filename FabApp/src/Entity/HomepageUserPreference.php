<?php

namespace App\Entity;

use App\Repository\HomepageUserPreferenceRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: HomepageUserPreferenceRepository::class)]
#[ORM\Table(name: 'HOMEPAGE_USER_PREFERENCE')]
#[ORM\UniqueConstraint(name: 'uniq_homepage_user_preference_user', columns: ['userId'])]
class HomepageUserPreference
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $user = null;

    #[ORM\Column(name: 'sectionOrder', type: 'text')]
    private string $sectionOrder = '[]';

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    public function getId(): ?int { return $this->id; }
    public function getUser(): ?Utilisateur { return $this->user; }
    public function setUser(?Utilisateur $user): self { $this->user = $user; return $this; }
    public function getSectionOrder(): string { return $this->sectionOrder; }
    public function setSectionOrder(string $sectionOrder): self { $this->sectionOrder = $sectionOrder; return $this; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }

    /** @return string[] */
    public function getSectionOrderArray(): array
    {
        $decoded = json_decode($this->sectionOrder, true);
        if (!is_array($decoded)) {
            return [];
        }

        return array_values(array_filter($decoded, static fn (mixed $value): bool => is_string($value) && $value !== ''));
    }

    /** @param string[] $sectionOrder */
    public function setSectionOrderArray(array $sectionOrder): self
    {
        $this->sectionOrder = json_encode(array_values($sectionOrder), JSON_THROW_ON_ERROR);

        return $this;
    }
}
