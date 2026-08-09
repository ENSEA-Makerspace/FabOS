<?php

namespace App\Entity;

use App\Repository\OpeningHourRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: OpeningHourRepository::class)]
#[ORM\Table(name: 'OPENING_HOUR')]
#[ORM\UniqueConstraint(name: 'uniq_opening_hour_day_of_week', columns: ['dayOfWeek'])]
class OpeningHour
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'venueId', nullable: false, onDelete: 'RESTRICT')]
    private ?Venue $venue = null;

    #[ORM\Column(name: 'dayOfWeek', type: 'smallint')]
    private int $dayOfWeek = 1;

    #[ORM\Column(length: 20)]
    private string $label = '';

    #[ORM\Column(name: 'isClosed', type: 'boolean', options: ['default' => false])]
    private bool $isClosed = false;

    #[ORM\Column(name: 'openTime', type: 'time', nullable: true)]
    private ?\DateTimeInterface $openTime = null;

    #[ORM\Column(name: 'closeTime', type: 'time', nullable: true)]
    private ?\DateTimeInterface $closeTime = null;

    #[ORM\Column(name: 'sortOrder', options: ['default' => 0])]
    private int $sortOrder = 0;

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    public function getId(): ?int { return $this->id; }
    public function getVenue(): ?Venue { return $this->venue; }
    public function setVenue(Venue $venue): self { $this->venue = $venue; return $this; }
    public function getDayOfWeek(): int { return $this->dayOfWeek; }
    public function setDayOfWeek(int $dayOfWeek): self { $this->dayOfWeek = $dayOfWeek; return $this; }
    public function getLabel(): string { return $this->label; }
    public function setLabel(string $label): self { $this->label = $label; return $this; }
    public function isClosed(): bool { return $this->isClosed; }
    public function setIsClosed(bool $isClosed): self { $this->isClosed = $isClosed; return $this; }
    public function getOpenTime(): ?\DateTimeInterface { return $this->openTime; }
    public function setOpenTime(?\DateTimeInterface $openTime): self { $this->openTime = $openTime; return $this; }
    public function getCloseTime(): ?\DateTimeInterface { return $this->closeTime; }
    public function setCloseTime(?\DateTimeInterface $closeTime): self { $this->closeTime = $closeTime; return $this; }
    public function getSortOrder(): int { return $this->sortOrder; }
    public function setSortOrder(int $sortOrder): self { $this->sortOrder = $sortOrder; return $this; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }

    public function getDisplayLabel(): string
    {
        if ($this->isClosed || !$this->openTime || !$this->closeTime) {
            return 'Fermé';
        }

        return $this->openTime->format('H:i') . ' - ' . $this->closeTime->format('H:i');
    }

    public function appliesTo(\DateTimeInterface $dateTime): bool
    {
        return (int) $dateTime->format('N') === $this->dayOfWeek;
    }

    public function isOpenAt(\DateTimeInterface $dateTime): bool
    {
        if (!$this->appliesTo($dateTime) || $this->isClosed || !$this->openTime || !$this->closeTime) {
            return false;
        }

        $time = (int) $dateTime->format('Hi');

        return $time >= (int) $this->openTime->format('Hi') && $time < (int) $this->closeTime->format('Hi');
    }
}
