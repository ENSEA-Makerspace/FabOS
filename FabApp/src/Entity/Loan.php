<?php

namespace App\Entity;

use App\Repository\LoanRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * A single checkout of a LoanableItem. The borrower may be a registered user
 * (FK) or, failing that, free-text contact details. Status is 'out' or
 * 'returned'; "overdue" is derived from the expected return date.
 */
#[ORM\Entity(repositoryClass: LoanRepository::class)]
#[ORM\Table(name: 'LOAN')]
class Loan
{
    public const STATUS_OUT = 'out';
    public const STATUS_RETURNED = 'returned';

    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: LoanableItem::class)]
    #[ORM\JoinColumn(name: 'itemId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?LoanableItem $item = null;

    /** Registered borrower, when there is one; else the free-text fields below. */
    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'borrowerId', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $borrower = null;

    #[ORM\Column(name: 'borrowerName', length: 180, nullable: true)]
    private ?string $borrowerName = null;

    #[ORM\Column(name: 'borrowerEmail', length: 180, nullable: true)]
    private ?string $borrowerEmail = null;

    #[ORM\Column(name: 'borrowerPhone', length: 60, nullable: true)]
    private ?string $borrowerPhone = null;

    #[ORM\Column(name: 'dateTaken', type: 'datetime_immutable')]
    private \DateTimeImmutable $dateTaken;

    #[ORM\Column(name: 'expectedReturnDate', type: 'date_immutable', nullable: true)]
    private ?\DateTimeImmutable $expectedReturnDate = null;

    #[ORM\Column(name: 'actualReturnDate', type: 'date_immutable', nullable: true)]
    private ?\DateTimeImmutable $actualReturnDate = null;

    #[ORM\Column(length: 20, options: ['default' => self::STATUS_OUT])]
    private string $status = self::STATUS_OUT;

    #[ORM\Column(name: 'conditionOut', type: 'text', nullable: true)]
    private ?string $conditionOut = null;

    #[ORM\Column(name: 'conditionReturn', type: 'text', nullable: true)]
    private ?string $conditionReturn = null;

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $notes = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'lentById', referencedColumnName: 'id', nullable: true, onDelete: 'SET NULL')]
    private ?Utilisateur $lentBy = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->dateTaken = new \DateTimeImmutable();
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getItem(): ?LoanableItem { return $this->item; }
    public function setItem(?LoanableItem $item): self { $this->item = $item; return $this; }
    public function getBorrower(): ?Utilisateur { return $this->borrower; }
    public function setBorrower(?Utilisateur $borrower): self { $this->borrower = $borrower; return $this; }
    public function getBorrowerName(): ?string { return $this->borrowerName; }
    public function setBorrowerName(?string $borrowerName): self { $this->borrowerName = $borrowerName; return $this; }
    public function getBorrowerEmail(): ?string { return $this->borrowerEmail; }
    public function setBorrowerEmail(?string $borrowerEmail): self { $this->borrowerEmail = $borrowerEmail; return $this; }
    public function getBorrowerPhone(): ?string { return $this->borrowerPhone; }
    public function setBorrowerPhone(?string $borrowerPhone): self { $this->borrowerPhone = $borrowerPhone; return $this; }
    public function getDateTaken(): \DateTimeImmutable { return $this->dateTaken; }
    public function setDateTaken(\DateTimeImmutable $dateTaken): self { $this->dateTaken = $dateTaken; return $this; }
    public function getExpectedReturnDate(): ?\DateTimeImmutable { return $this->expectedReturnDate; }
    public function setExpectedReturnDate(?\DateTimeImmutable $expectedReturnDate): self { $this->expectedReturnDate = $expectedReturnDate; return $this; }
    public function getActualReturnDate(): ?\DateTimeImmutable { return $this->actualReturnDate; }
    public function setActualReturnDate(?\DateTimeImmutable $actualReturnDate): self { $this->actualReturnDate = $actualReturnDate; return $this; }
    public function getStatus(): string { return $this->status; }
    public function setStatus(string $status): self { $this->status = $status; return $this; }
    public function getConditionOut(): ?string { return $this->conditionOut; }
    public function setConditionOut(?string $conditionOut): self { $this->conditionOut = $conditionOut; return $this; }
    public function getConditionReturn(): ?string { return $this->conditionReturn; }
    public function setConditionReturn(?string $conditionReturn): self { $this->conditionReturn = $conditionReturn; return $this; }
    public function getNotes(): ?string { return $this->notes; }
    public function setNotes(?string $notes): self { $this->notes = $notes; return $this; }
    public function getLentBy(): ?Utilisateur { return $this->lentBy; }
    public function setLentBy(?Utilisateur $lentBy): self { $this->lentBy = $lentBy; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    public function isReturned(): bool
    {
        return $this->status === self::STATUS_RETURNED;
    }

    /** 'returned' | 'overdue' | 'out' — overdue derived from the expected date. */
    public function getEffectiveStatus(): string
    {
        if ($this->isReturned()) {
            return self::STATUS_RETURNED;
        }
        if ($this->expectedReturnDate !== null && $this->expectedReturnDate < new \DateTimeImmutable('today')) {
            return 'overdue';
        }

        return self::STATUS_OUT;
    }

    /** Best display name for the borrower, registered or free-text. */
    public function getBorrowerDisplay(): string
    {
        if ($this->borrower !== null) {
            return $this->borrower->getDisplayName();
        }

        return $this->borrowerName !== null && trim($this->borrowerName) !== ''
            ? $this->borrowerName
            : 'Emprunteur anonyme';
    }
}
