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

    /**
     * The last day of the closure, when it lasts more than one (S146g).
     *
     * 🔴 **A range on the row, not one row per day.** "Closed for the fortnight of
     * Christmas" is ONE fact about the lab: fourteen rows would make cancelling it a
     * bulk delete, and that is the complaint this exists to answer. (Event series went
     * the other way, deliberately — those are occurrences, and each one moves, fills
     * and is cancelled on its own.)
     *
     * ⚠️ **Null means "one day", and every reader must say so with `COALESCE`.** Every
     * row written before S146g has a null here and must keep meaning exactly what it
     * meant; a reader that forgets the fallback silently stops seeing all of them.
     */
    #[ORM\Column(name: 'endDate', type: 'date_immutable', nullable: true)]
    private ?\DateTimeImmutable $endDate = null;

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

    public function getEndDate(): ?\DateTimeImmutable { return $this->endDate; }

    /** ⚠️ An end before the start is not a range; it is stored as a single day. */
    public function setEndDate(?\DateTimeImmutable $endDate): self
    {
        $this->endDate = $endDate !== null && $endDate > $this->exceptionDate ? $endDate : null;

        return $this;
    }

    /** The last day this row decides — itself, when no end was given. */
    public function getLastDate(): \DateTimeImmutable { return $this->endDate ?? $this->exceptionDate; }

    public function spansSeveralDays(): bool { return $this->endDate !== null; }

    /** How many days it covers, inclusive — 1 for a single date. */
    public function dayCount(): int
    {
        return 1 + (int) $this->exceptionDate->diff($this->getLastDate())->days;
    }

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
