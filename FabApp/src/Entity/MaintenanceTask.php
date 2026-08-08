<?php

namespace App\Entity;

use App\Repository\MaintenanceTaskRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A single maintenance task for a machine — a due/overdue backlog item, NOT a
 * calendar appointment (v1 is non-blocking). A task is preventive (scheduled)
 * or corrective (something broke). If recurrenceDays is set, marking it done
 * spawns the next occurrence. Blocking-calendar + usage-hour triggers are
 * deferred to the control-box work.
 */
#[ORM\Entity(repositoryClass: MaintenanceTaskRepository::class)]
#[ORM\Table(name: 'MAINTENANCE_TASK')]
class MaintenanceTask
{
    public const TYPE_PREVENTIVE = 'preventive';
    public const TYPE_CORRECTIVE = 'corrective';
    public const STATUS_PENDING = 'pending';
    public const STATUS_DONE = 'done';

    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    #[ORM\Column(length: 180)]
    private string $title = '';

    #[ORM\Column(length: 20, options: ['default' => self::TYPE_PREVENTIVE])]
    private string $type = self::TYPE_PREVENTIVE;

    /** Optional link, e.g. to the wiki tutorial for this task. */
    #[ORM\Column(length: 500, nullable: true)]
    private ?string $link = null;

    #[ORM\Column(name: 'dueDate', type: 'date_immutable', nullable: true)]
    private ?\DateTimeImmutable $dueDate = null;

    #[ORM\Column(length: 20, options: ['default' => self::STATUS_PENDING])]
    private string $status = self::STATUS_PENDING;

    #[ORM\Column(name: 'doneDate', type: 'date_immutable', nullable: true)]
    private ?\DateTimeImmutable $doneDate = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'doneById', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $doneBy = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $notes = null;

    /** If set, marking done spawns a new pending task due this many days later. */
    #[ORM\Column(name: 'recurrenceDays', nullable: true)]
    private ?int $recurrenceDays = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getMachine(): ?Machine { return $this->machine; }
    public function setMachine(?Machine $machine): self { $this->machine = $machine; return $this; }
    public function getTitle(): string { return $this->title; }
    public function setTitle(string $title): self { $this->title = $title; return $this; }
    public function getType(): string { return $this->type; }
    public function setType(string $type): self { $this->type = $type; return $this; }
    public function getLink(): ?string { return $this->link; }
    public function setLink(?string $link): self { $this->link = $link; return $this; }
    public function getDueDate(): ?\DateTimeImmutable { return $this->dueDate; }
    public function setDueDate(?\DateTimeImmutable $dueDate): self { $this->dueDate = $dueDate; return $this; }
    public function getStatus(): string { return $this->status; }
    public function setStatus(string $status): self { $this->status = $status; return $this; }
    public function getDoneDate(): ?\DateTimeImmutable { return $this->doneDate; }
    public function setDoneDate(?\DateTimeImmutable $doneDate): self { $this->doneDate = $doneDate; return $this; }
    public function getDoneBy(): ?Utilisateur { return $this->doneBy; }
    public function setDoneBy(?Utilisateur $doneBy): self { $this->doneBy = $doneBy; return $this; }
    public function getNotes(): ?string { return $this->notes; }
    public function setNotes(?string $notes): self { $this->notes = $notes; return $this; }
    public function getRecurrenceDays(): ?int { return $this->recurrenceDays; }
    public function setRecurrenceDays(?int $recurrenceDays): self { $this->recurrenceDays = $recurrenceDays !== null && $recurrenceDays > 0 ? $recurrenceDays : null; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    public function isDone(): bool
    {
        return $this->status === self::STATUS_DONE;
    }

    /** 'done' | 'overdue' | 'due_soon' | 'pending' — derived from due date. */
    public function getEffectiveStatus(): string
    {
        if ($this->isDone()) {
            return self::STATUS_DONE;
        }
        if ($this->dueDate === null) {
            return self::STATUS_PENDING;
        }
        $today = new \DateTimeImmutable('today');
        if ($this->dueDate < $today) {
            return 'overdue';
        }
        if ($this->dueDate <= $today->modify('+7 days')) {
            return 'due_soon';
        }

        return self::STATUS_PENDING;
    }
}
