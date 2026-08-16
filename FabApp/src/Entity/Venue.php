<?php

namespace App\Entity;

use App\Repository\VenueRepository;
use Doctrine\ORM\Mapping as ORM;
use Symfony\Component\Validator\Constraints as Assert;

#[ORM\Entity(repositoryClass: VenueRepository::class)]
#[ORM\Table(name: 'VENUE')]
class Venue
{
    /**
     * The slug of the venue that can never be archived.
     *
     * ⚠️ It is matched by value, not by id: `VenueRepository::findDefault()` and
     * every guard below agree on this one constant so the invariant cannot drift
     * between the service that enforces it and the UI that explains it.
     */
    public const DEFAULT_SLUG = 'default';

    #[ORM\Id, ORM\GeneratedValue, ORM\Column]
    private ?int $id = null;

    /**
     * ⚠️ **Stable and write-once.** The slug is what `?location=` carries, so it
     * is in every shareable admin URL and in members' saved links. Renaming it
     * silently 404s those. The form exposes it on creation only; there is no
     * setter path from the edit screen.
     */
    #[ORM\Column(length: 80, unique: true)]
    #[Assert\NotBlank(message: 'venues.error.slug_required')]
    #[Assert\Regex(pattern: '/^[a-z0-9]([a-z0-9-]{0,78}[a-z0-9])?$/', message: 'venues.error.slug_format')]
    private string $slug = self::DEFAULT_SLUG;

    #[ORM\Column(length: 140)]
    #[Assert\NotBlank(message: 'venues.error.name_required')]
    #[Assert\Length(max: 140)]
    private string $name = '';

    #[ORM\Column(length: 255, nullable: true)]
    #[Assert\Length(max: 255)]
    private ?string $address = null;

    /**
     * ⚠️ Per-venue, not per-install. A location in another country keeps its own
     * opening hours and its own idea of "today", so the booking engine must read
     * this rather than `SiteSettingService::getTimezone()` once venues diverge.
     */
    #[ORM\Column(length: 64)]
    #[Assert\NotBlank(message: 'venues.error.timezone_required')]
    #[Assert\Timezone(message: 'venues.error.timezone_invalid')]
    private string $timezone = 'Europe/Paris';

    /** Archive flag. `false` hides the venue from context pickers; it never deletes rows. */
    #[ORM\Column(options: ['default' => true])]
    private bool $active = true;

    public function getId(): ?int { return $this->id; }

    public function getSlug(): string { return $this->slug; }

    public function setSlug(string $slug): self { $this->slug = $slug; return $this; }

    public function getName(): string { return $this->name; }

    public function setName(string $name): self { $this->name = trim($name); return $this; }

    public function getAddress(): ?string { return $this->address; }

    public function setAddress(?string $address): self
    {
        $address = $address !== null ? trim($address) : null;
        $this->address = $address === '' ? null : $address;

        return $this;
    }

    public function getTimezone(): string { return $this->timezone; }

    public function setTimezone(string $timezone): self { $this->timezone = $timezone; return $this; }

    public function isActive(): bool { return $this->active; }

    public function setActive(bool $active): self { $this->active = $active; return $this; }

    /** Whether this row is the protected default. Read it instead of comparing slugs by hand. */
    public function isDefault(): bool { return $this->slug === self::DEFAULT_SLUG; }
}
