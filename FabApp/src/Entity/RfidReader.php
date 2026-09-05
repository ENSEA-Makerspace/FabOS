<?php

namespace App\Entity;

use App\Repository\RfidReaderRepository;
use Doctrine\ORM\Mapping as ORM;

#[ORM\Entity(repositoryClass: RfidReaderRepository::class)]
#[ORM\Table(name: 'RFID_READER')]
#[ORM\UniqueConstraint(name: 'unique_rfid_reader_token', columns: ['readerToken'])]
class RfidReader
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 120)]
    private string $name = '';

    #[ORM\Column(name: 'readerToken', length: 120, unique: true)]
    private string $readerToken = '';

    /**
     * 🔴 **S175 — `nullable: false` est parti, et c'était le verrou de toute la
     * Phase P.** Tant que cette colonne était obligatoire, une porte ne pouvait
     * être représentée qu'en inventant une machine fictive — qui serait apparue
     * dans le catalogue, les réservations et les statistiques, et que chaque
     * écran aurait dû apprendre à ignorer.
     */
    #[ORM\ManyToOne(targetEntity: Machine::class)]
    #[ORM\JoinColumn(name: 'machineId', referencedColumnName: 'id', onDelete: 'CASCADE')]
    private ?Machine $machine = null;

    /**
     * ⚠️ **L'autre cible possible, et exactement l'autre.** Un boîtier commande
     * une machine OU une gâche ; les deux à la fois n'a pas de sens physique, et
     * aucun des deux non plus (le boîtier ne commanderait rien). La règle est
     * dans `targetKind()` et `hasValidTarget()` ci-dessous, pas dans le schéma :
     * voir la migration `Version20260906090000` pour pourquoi le `CHECK` a été
     * refusé plutôt qu'oublié.
     */
    #[ORM\ManyToOne(targetEntity: AccessPoint::class)]
    #[ORM\JoinColumn(name: 'accessPointId', referencedColumnName: 'id', onDelete: 'CASCADE')]
    private ?AccessPoint $accessPoint = null;

    #[ORM\Column(name: 'isActive', options: ['default' => true])]
    private bool $isActive = true;

    #[ORM\Column(name: 'lastSeenAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $lastSeenAt = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'updatedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $updatedAt = null;

    /**
     * ⚠️ **S147, J-2 — archivé, pas supprimé.** L'action d'administration
     * appelait `->remove()` : la ligne disparaissait, et avec elle tout ce qui
     * la nommait. Les journaux d'accès pointent sur le lecteur. Le supprimer laissait des lignes de log orphelines dont plus personne ne savait de quelle porte elles parlaient.
     *
     * ⚠️ `findBy()` reste la question de l'ADMIN : la liste d'administration
     * montre les archivés, marqués comme tels, parce que c'est de là qu'on les
     * restaure. Ce sont les surfaces qui PROPOSENT qui doivent filtrer.
     */
    #[ORM\Column(name: 'archivedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $archivedAt = null;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getName(): string { return $this->name; }
    public function setName(string $name): self { $this->name = $name; return $this; }
    public function getReaderToken(): string { return $this->readerToken; }
    public function setReaderToken(string $readerToken): self { $this->readerToken = $readerToken; return $this; }
    public function getMachine(): ?Machine { return $this->machine; }
    public function getAccessPoint(): ?AccessPoint { return $this->accessPoint; }

    /**
     * ⚠️ **Les deux affectations s'excluent, et c'est le SETTER qui le tient.**
     * Laisser les deux setters indépendants aurait fait de « exactement une
     * cible » une consigne — quelque chose qu'on respecte jusqu'au jour où on
     * oublie. Poser une machine efface le point d'accès, et l'inverse.
     */
    public function setMachine(?Machine $machine): self
    {
        $this->machine = $machine;
        if ($machine !== null) {
            $this->accessPoint = null;
        }

        return $this;
    }

    public function setAccessPoint(?AccessPoint $accessPoint): self
    {
        $this->accessPoint = $accessPoint;
        if ($accessPoint !== null) {
            $this->machine = null;
        }

        return $this;
    }

    /**
     * `'machine'`, `'access_point'`, ou `null` quand le boîtier ne commande rien.
     *
     * 🔴 **Le troisième cas EXISTE et doit se dire.** Un boîtier sans cible est
     * un boîtier posé au mur qui n'ouvre rien : `ReaderHealth` en fait l'état
     * « association invalide », qui est précisément celui qu'un booléen
     * `isActive` ne pouvait pas exprimer (S172). Rendre `'machine'` par défaut
     * ferait disparaître ce cas de tous les écrans.
     */
    public function targetKind(): ?string
    {
        if ($this->machine !== null) {
            return 'machine';
        }

        return $this->accessPoint !== null ? 'access_point' : null;
    }

    /** Le nom de ce que le boîtier commande, pour un écran ; `null` s'il ne commande rien. */
    public function targetLabel(): ?string
    {
        return $this->machine?->getNom() ?? $this->accessPoint?->getNom();
    }

    public function hasValidTarget(): bool
    {
        return ($this->machine !== null) !== ($this->accessPoint !== null);
    }
    public function isActive(): bool { return $this->isActive; }
    public function setIsActive(bool $isActive): self { $this->isActive = $isActive; return $this; }
    public function getLastSeenAt(): ?\DateTimeImmutable { return $this->lastSeenAt; }
    public function setLastSeenAt(?\DateTimeImmutable $lastSeenAt): self { $this->lastSeenAt = $lastSeenAt; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
    public function getUpdatedAt(): ?\DateTimeImmutable { return $this->updatedAt; }
    public function setUpdatedAt(?\DateTimeImmutable $updatedAt): self { $this->updatedAt = $updatedAt; return $this; }

    public function getArchivedAt(): ?\DateTimeImmutable { return $this->archivedAt; }
    public function isArchived(): bool { return $this->archivedAt !== null; }
    public function archive(): self { $this->archivedAt ??= new \DateTimeImmutable(); return $this; }
    public function restore(): self { $this->archivedAt = null; return $this; }
}
