<?php

namespace App\Entity;

use App\Repository\MachineCategoryRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A machine category an operator can actually manage (S133).
 *
 * ⚠️ **This class used to be an empty stub** — "ancienne entité neutralisée",
 * left behind when the schema moved, with no table under it. `/admin/machines/
 * categories` therefore showed a *derived* view: the distinct values of
 * `MACHINE.categoryLabel`, grouped and counted, presented under a heading that
 * read like management. The roadmap's rule is the one being applied here — never
 * present a facet as management — and the choice it offered was a real CRUD or
 * removing the tab.
 *
 * ⚠️ **`label` is the join key, and there is no `MACHINE.categoryId`.** Adding
 * the foreign key would be a contract step touching every template, form, filter
 * and export that reads `categoryLabel`, with a half-migrated install running two
 * category systems at once. Instead this table carries the three things a derived
 * facet cannot have — existence before first use, one icon per category rather
 * than per machine, and an archived state — and a rename is a pair of statements
 * the service performs together.
 */
#[ORM\Entity(repositoryClass: MachineCategoryRepository::class)]
#[ORM\Table(name: 'MACHINE_CATEGORY')]
#[ORM\UniqueConstraint(name: 'UNIQ_MACHINE_CATEGORY_LABEL', columns: ['label'])]
class MachineCategory
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 100)]
    private string $label = '';

    #[ORM\Column(name: 'iconSlug', length: 50, nullable: true)]
    private ?string $iconSlug = null;

    /**
     * ⚠️ Archived, never deleted — the roadmap's rule for every other resource,
     * and here it is load-bearing: machines keep the label whatever happens to
     * this row, so deleting would leave categories that exist in the data and
     * nowhere in the screen that manages them.
     */
    #[ORM\Column(name: 'archivedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $archivedAt = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }

    public function getLabel(): string { return $this->label; }
    public function setLabel(string $label): self { $this->label = $label; return $this; }

    public function getIconSlug(): ?string { return $this->iconSlug; }
    public function setIconSlug(?string $iconSlug): self { $this->iconSlug = $iconSlug ?: null; return $this; }

    public function getArchivedAt(): ?\DateTimeImmutable { return $this->archivedAt; }
    public function isArchived(): bool { return $this->archivedAt !== null; }
    public function archive(): self { $this->archivedAt ??= new \DateTimeImmutable(); return $this; }
    public function restore(): self { $this->archivedAt = null; return $this; }

    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
}
