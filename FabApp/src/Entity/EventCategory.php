<?php

namespace App\Entity;

use App\Repository\EventCategoryRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * What KIND of event this is — atelier, séance de formation, portes ouvertes…
 * The lab's own vocabulary, editable by the operator (S146f).
 *
 * 🔴 **No code may branch on a particular category.** This is a label, not an
 * enum: the operator can rename "Séance de formation" to "Cours" on a Tuesday,
 * and anything that read `slug === 'formation'` to decide behaviour would break
 * silently. When behaviour must depend on what an event IS, that belongs in a
 * column of its own — which is exactly why `Event::$formation` exists beside this
 * and is not folded into it. The category answers "how do I describe it"; the
 * link answers "what is it a session of".
 *
 * ⚠️ **The label is CONTENT and is never translated.** Same rule as machine
 * categories and every other catalogue: the UI ships in five languages, what an
 * operator types does not. `slug` exists for stable URLs and filters, so renaming
 * the label does not break a shared link.
 *
 * ⚠️ **Archived, never deleted** — the house rule, and load-bearing here: events
 * keep pointing at their category, so deleting a row would leave events whose kind
 * exists in the data and nowhere in the screen that manages it. Archiving hides it
 * from the pickers and the filters while the events that already carry it keep
 * reading correctly.
 */
#[ORM\Entity(repositoryClass: EventCategoryRepository::class)]
#[ORM\Table(name: 'EVENT_CATEGORY')]
#[ORM\UniqueConstraint(name: 'UNIQ_EVENT_CATEGORY_SLUG', columns: ['slug'])]
class EventCategory
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 100)]
    private string $label = '';

    #[ORM\Column(length: 120)]
    private string $slug = '';

    #[ORM\Column(name: 'iconSlug', length: 50, nullable: true)]
    private ?string $iconSlug = null;

    /** Operator order. Categories are a short list somebody arranges by hand. */
    #[ORM\Column(options: ['default' => 0])]
    private int $position = 0;

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

    public function setLabel(string $label): self
    {
        $this->label = mb_substr(trim($label), 0, 100);

        if ($this->slug === '') {
            $this->setSlug($this->label);
        }

        return $this;
    }

    public function getSlug(): string { return $this->slug; }

    /**
     * ⚠️ Set from the label ONCE, at creation, and then left alone on rename.
     * A slug that follows the label would change the URL of every filter link
     * somebody has shared the moment a typo is fixed.
     */
    public function setSlug(string $slug): self
    {
        $slug = strtolower(trim($slug));
        $slug = preg_replace('/[^a-z0-9]+/u', '-', $this->deaccent($slug)) ?? '';
        $this->slug = mb_substr(trim($slug, '-'), 0, 120);

        return $this;
    }

    public function getIconSlug(): ?string { return $this->iconSlug; }

    public function setIconSlug(?string $iconSlug): self
    {
        $iconSlug = $iconSlug !== null ? trim($iconSlug) : '';
        $this->iconSlug = $iconSlug !== '' ? mb_substr($iconSlug, 0, 50) : null;

        return $this;
    }

    public function getPosition(): int { return $this->position; }
    public function setPosition(int $position): self { $this->position = max(0, $position); return $this; }

    public function getArchivedAt(): ?\DateTimeImmutable { return $this->archivedAt; }
    public function isArchived(): bool { return $this->archivedAt !== null; }

    public function archive(?\DateTimeImmutable $at = null): self
    {
        $this->archivedAt = $at ?? new \DateTimeImmutable();

        return $this;
    }

    public function restore(): self
    {
        $this->archivedAt = null;

        return $this;
    }

    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    /**
     * ⚠️ `iconv` is not used: it throws or returns false depending on the libc,
     * and a category called "Portes ouvertes" must not be able to take the admin
     * down. A plain table covers the five languages the product ships.
     */
    private function deaccent(string $value): string
    {
        return strtr($value, [
            'à' => 'a', 'á' => 'a', 'â' => 'a', 'ã' => 'a', 'ä' => 'a', 'å' => 'a',
            'ç' => 'c',
            'è' => 'e', 'é' => 'e', 'ê' => 'e', 'ë' => 'e',
            'ì' => 'i', 'í' => 'i', 'î' => 'i', 'ï' => 'i',
            'ñ' => 'n',
            'ò' => 'o', 'ó' => 'o', 'ô' => 'o', 'õ' => 'o', 'ö' => 'o',
            'ù' => 'u', 'ú' => 'u', 'û' => 'u', 'ü' => 'u',
            'ý' => 'y', 'ÿ' => 'y',
            'ß' => 'ss', 'æ' => 'ae', 'œ' => 'oe',
        ]);
    }
}
