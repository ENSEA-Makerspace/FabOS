<?php

declare(strict_types=1);

namespace App\Entity;

use App\Repository\ScheduleExceptionRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * One date that does not follow the week — a holiday, a closure, a special
 * opening (S134d).
 *
 * ⚠️ **An exception REPLACES the weekday, it does not add to it.** If any row
 * exists for a date, that date's opening is exactly what those rows say. Merging
 * with the weekly grid instead would make "open 09:00–12:00 only, for
 * stocktaking" indistinguishable from a half-finished edit.
 *
 * ⚠️ **The reason is not decoration.** "Closed" leaves a member wondering whether
 * the lab is broken or whether they misread; "closed — public holiday" ends the
 * question. The surfaces show it, so it is part of the model rather than a note
 * an operator keeps somewhere else.
 *
 * ⚠️ The date is a DATE and not a datetime: a closure belongs to a calendar day
 * in the lab's own zone, and storing an instant would drag it across midnight for
 * exactly the operators who configured a timezone.
 */
#[ORM\Entity(repositoryClass: ScheduleExceptionRepository::class)]
#[ORM\Table(name: 'SCHEDULE_EXCEPTION')]
#[ORM\Index(name: 'IDX_SCHEDULE_EXCEPTION_VENUE_DATE', columns: ['venueId', 'exceptionDate'])]
class ScheduleException
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'venueId', nullable: false, onDelete: 'CASCADE')]
    private ?Venue $venue = null;

    #[ORM\Column(name: 'exceptionDate', type: 'date_immutable')]
    private \DateTimeImmutable $exceptionDate;

    #[ORM\Column(name: 'isClosed', type: 'boolean', options: ['default' => true])]
    private bool $isClosed = true;

    #[ORM\Column(name: 'openTime', type: 'time', nullable: true)]
    private ?\DateTimeInterface $openTime = null;

    #[ORM\Column(name: 'closeTime', type: 'time', nullable: true)]
    private ?\DateTimeInterface $closeTime = null;

    #[ORM\Column(length: 120, nullable: true)]
    private ?string $reason = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;

    public function __construct()
    {
        $this->exceptionDate = new \DateTimeImmutable('today');
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }

    public function getVenue(): ?Venue { return $this->venue; }
    public function setVenue(?Venue $venue): self { $this->venue = $venue; return $this; }

    public function getExceptionDate(): \DateTimeImmutable { return $this->exceptionDate; }
    public function setExceptionDate(\DateTimeImmutable $date): self { $this->exceptionDate = $date; return $this; }

    public function isClosed(): bool { return $this->isClosed; }
    public function setIsClosed(bool $isClosed): self { $this->isClosed = $isClosed; return $this; }

    public function getOpenTime(): ?\DateTimeInterface { return $this->openTime; }
    public function setOpenTime(?\DateTimeInterface $openTime): self { $this->openTime = $openTime; return $this; }

    public function getCloseTime(): ?\DateTimeInterface { return $this->closeTime; }
    public function setCloseTime(?\DateTimeInterface $closeTime): self { $this->closeTime = $closeTime; return $this; }

    public function getReason(): ?string { return $this->reason; }
    public function setReason(?string $reason): self { $this->reason = $reason !== null && trim($reason) !== '' ? trim($reason) : null; return $this; }

    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    /**
     * ⚠️ An exception with no usable times IS a closure, whatever `isClosed`
     * says. The two can disagree — a form that unticks "closed" and leaves the
     * times empty produces exactly that — and a row nobody can read as an opening
     * must not be treated as one.
     */
    public function opensTheDay(): bool
    {
        return !$this->isClosed && $this->openTime !== null && $this->closeTime !== null;
    }
}
