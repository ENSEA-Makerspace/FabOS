<?php

namespace App\Entity;

use App\Repository\UserAvailabilityRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * One weekly window during which a bookable person accepts appointments —
 * "Tuesdays 14:00–18:00". Recurring by weekday rather than dated, because a
 * trainer's office hours are a habit, not a list of dates.
 *
 * A window is an offer, not a guarantee: the slots actually shown are this
 * intersected with the lab's opening hours, minus existing bookings (see
 * PersonAvailabilityService). Booking outside every window is still possible,
 * but only as a request the person has to accept.
 */
#[ORM\Entity(repositoryClass: UserAvailabilityRepository::class)]
#[ORM\Table(name: 'USER_AVAILABILITY')]
#[ORM\Index(name: 'IDX_USER_AVAILABILITY_USER', columns: ['userId', 'dayOfWeek'])]
class UserAvailability
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Utilisateur::class)]
    #[ORM\JoinColumn(name: 'userId', referencedColumnName: 'id', nullable: false, onDelete: 'CASCADE')]
    private ?Utilisateur $utilisateur = null;

    /** ISO-8601 weekday, 1 = Monday … 7 = Sunday — same convention as OpeningHour. */
    #[ORM\Column(name: 'dayOfWeek', type: 'smallint')]
    private int $dayOfWeek = 1;

    #[ORM\Column(name: 'startTime', type: 'time')]
    private \DateTimeInterface $startTime;

    #[ORM\Column(name: 'endTime', type: 'time')]
    private \DateTimeInterface $endTime;

    public function __construct()
    {
        $this->startTime = new \DateTime('09:00');
        $this->endTime = new \DateTime('17:00');
    }

    public function getId(): ?int { return $this->id; }
    public function getUtilisateur(): ?Utilisateur { return $this->utilisateur; }
    public function setUtilisateur(?Utilisateur $utilisateur): self { $this->utilisateur = $utilisateur; return $this; }
    public function getDayOfWeek(): int { return $this->dayOfWeek; }
    public function setDayOfWeek(int $dayOfWeek): self { $this->dayOfWeek = max(1, min(7, $dayOfWeek)); return $this; }
    public function getStartTime(): \DateTimeInterface { return $this->startTime; }
    public function setStartTime(\DateTimeInterface $startTime): self { $this->startTime = $startTime; return $this; }
    public function getEndTime(): \DateTimeInterface { return $this->endTime; }
    public function setEndTime(\DateTimeInterface $endTime): self { $this->endTime = $endTime; return $this; }

    public function getDisplayLabel(): string
    {
        return $this->startTime->format('H:i') . ' - ' . $this->endTime->format('H:i');
    }

    /** Minutes since midnight, the form the slot engine compares on. */
    public function getStartMinutes(): int
    {
        return (int) $this->startTime->format('H') * 60 + (int) $this->startTime->format('i');
    }

    public function getEndMinutes(): int
    {
        return (int) $this->endTime->format('H') * 60 + (int) $this->endTime->format('i');
    }

    public function covers(int $dayOfWeek, int $startMinutes, int $endMinutes): bool
    {
        return $this->dayOfWeek === $dayOfWeek
            && $startMinutes >= $this->getStartMinutes()
            && $endMinutes <= $this->getEndMinutes();
    }
}
