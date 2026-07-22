<?php

namespace App\Entity;

use App\Repository\UtilisateurRepository;
use Doctrine\Common\Collections\ArrayCollection;
use Doctrine\Common\Collections\Collection;
use Doctrine\ORM\Mapping as ORM;
use Symfony\Component\Security\Core\User\PasswordAuthenticatedUserInterface;
use Symfony\Component\Security\Core\User\UserInterface;

#[ORM\Entity(repositoryClass: UtilisateurRepository::class)]
#[ORM\Table(name: 'UTILISATEUR')]
class Utilisateur implements UserInterface, PasswordAuthenticatedUserInterface
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 255, unique: true)]
    private string $email = '';

    #[ORM\Column(length: 255, unique: true)]
    private string $username = '';

    #[ORM\Column(length: 255)]
    private string $password = '';

    #[ORM\Column(name: 'firstName', length: 255, nullable: true)]
    private ?string $firstName = null;

    #[ORM\Column(name: 'lastName', length: 255, nullable: true)]
    private ?string $lastName = null;

    #[ORM\Column(name: 'numeroId', length: 100, nullable: true)]
    private ?string $numeroId = null;

    #[ORM\Column(length: 50, options: ['default' => 'actif'])]
    private string $statut = 'actif';

    #[ORM\Column(name: 'identifiantRfid', length: 255, unique: true, nullable: true)]
    private ?string $identifiantRfid = null;

    #[ORM\Column(name: 'tempsPresenceTotal', options: ['default' => 0])]
    private int $tempsPresenceTotal = 0;

    #[ORM\Column(name: 'isVerified', options: ['default' => false])]
    private bool $isVerified = false;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    #[ORM\Column(name: 'derniereConnexion', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $derniereConnexion = null;

    #[ORM\Column(name: 'notificationEmail', options: ['default' => true])]
    private bool $notificationEmail = true;

    #[ORM\Column(name: 'notificationPush', options: ['default' => false])]
    private bool $notificationPush = false;

    #[ORM\Column(name: 'rappelReservation', options: ['default' => true])]
    private bool $rappelReservation = true;

    #[ORM\Column(length: 50, options: ['default' => 'light'])]
    private string $theme = 'light';

    #[ORM\Column(length: 10, options: ['default' => 'fr'])]
    private string $langue = 'fr';

    #[ORM\Column(name: 'avatarFilename', length: 255, nullable: true)]
    private ?string $avatarFilename = null;

    #[ORM\Column(name: 'bannerFilename', length: 255, nullable: true)]
    private ?string $bannerFilename = null;

    /** @var Collection<int, UtilisateurRole> */
    #[ORM\OneToMany(mappedBy: 'utilisateur', targetEntity: UtilisateurRole::class)]
    private Collection $utilisateurRoles;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); $this->utilisateurRoles = new ArrayCollection(); }
    public function getId(): ?int { return $this->id; }
    public function getEmail(): string { return $this->email; }
    public function setEmail(string $email): self { $this->email = $email; return $this; }
    public function getUsername(): string { return $this->username; }
    public function setUsername(string $username): self { $this->username = $username; return $this; }
    public function getPassword(): string { return $this->password; }
    public function setPassword(string $password): self { $this->password = $password; return $this; }
    public function getUserIdentifier(): string { return $this->email; }
    public function getRoles(): array
    {
        $roles = ['ROLE_USER'];
        foreach ($this->utilisateurRoles as $utilisateurRole) {
            $roleName = $utilisateurRole->getRole()?->getNom();
            if (!$roleName) {
                continue;
            }
            $roles[] = match (strtolower($roleName)) {
                'admin' => 'ROLE_ADMIN',
                'staff' => 'ROLE_STAFF',
                'user' => 'ROLE_USER',
                default => str_starts_with(strtoupper($roleName), 'ROLE_') ? strtoupper($roleName) : 'ROLE_' . strtoupper($roleName),
            };
        }

        return array_values(array_unique($roles));
    }
    public function eraseCredentials(): void {}
    /** @return Collection<int, UtilisateurRole> */
    public function getUtilisateurRoles(): Collection { return $this->utilisateurRoles; }
    public function getFirstName(): ?string { return $this->firstName; }
    public function setFirstName(?string $firstName): self { $this->firstName = $firstName; return $this; }
    public function getLastName(): ?string { return $this->lastName; }
    public function setLastName(?string $lastName): self { $this->lastName = $lastName; return $this; }
    public function getNumeroId(): ?string { return $this->numeroId; }
    public function setNumeroId(?string $numeroId): self { $this->numeroId = $numeroId; return $this; }
    public function getDisplayName(): string { return trim(($this->firstName ?? '') . ' ' . ($this->lastName ?? '')) ?: $this->username; }
    public function getStatut(): string { return $this->statut; }
    public function setStatut(string $statut): self { $this->statut = $statut; return $this; }
    public function getStatus(): string { return $this->statut; }
    public function getIdentifiantRfid(): ?string { return $this->identifiantRfid; }
    public function setIdentifiantRfid(?string $identifiantRfid): self { $this->identifiantRfid = $identifiantRfid; return $this; }
    public function getTempsPresenceTotal(): int { return $this->tempsPresenceTotal; }
    public function setTempsPresenceTotal(int $tempsPresenceTotal): self { $this->tempsPresenceTotal = $tempsPresenceTotal; return $this; }
    public function getPoints(): int { return $this->tempsPresenceTotal; }
    public function setPoints(int $points): self { return $this->setTempsPresenceTotal($points); }
    public function isVerified(): bool { return $this->isVerified; }
    public function setIsVerified(bool $isVerified): self { $this->isVerified = $isVerified; return $this; }
    // Person-type flags for the directory pages, derived from the ROLE membership
    // (no dedicated column). A user can be staff, trainer, both, or neither.
    public function isStaff(): bool { return $this->hasRoleNamed('staff'); }
    public function isTrainer(): bool { return $this->hasRoleNamed('trainer'); }
    public function hasRoleNamed(string $name): bool
    {
        $name = strtolower($name);
        foreach ($this->utilisateurRoles as $utilisateurRole) {
            if (strtolower((string) $utilisateurRole->getRole()?->getNom()) === $name) {
                return true;
            }
        }

        return false;
    }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }
    public function setCreatedAt(\DateTimeImmutable $createdAt): self { $this->createdAt = $createdAt; return $this; }
    public function getDerniereConnexion(): ?\DateTimeImmutable { return $this->derniereConnexion; }
    public function setDerniereConnexion(?\DateTimeImmutable $derniereConnexion): self { $this->derniereConnexion = $derniereConnexion; return $this; }
    public function isNotificationEmail(): bool { return $this->notificationEmail; }
    public function setNotificationEmail(bool $notificationEmail): self { $this->notificationEmail = $notificationEmail; return $this; }
    public function isNotificationPush(): bool { return $this->notificationPush; }
    public function setNotificationPush(bool $notificationPush): self { $this->notificationPush = $notificationPush; return $this; }
    public function isRappelReservation(): bool { return $this->rappelReservation; }
    public function setRappelReservation(bool $rappelReservation): self { $this->rappelReservation = $rappelReservation; return $this; }
    public function getTheme(): string { return $this->theme; }
    public function setTheme(string $theme): self { $this->theme = $theme; return $this; }
    public function getLangue(): string { return $this->langue; }
    public function setLangue(string $langue): self { $this->langue = $langue; return $this; }
    public function getAvatarFilename(): ?string { return $this->avatarFilename; }
    public function setAvatarFilename(?string $avatarFilename): self { $this->avatarFilename = $avatarFilename; return $this; }
    public function getBannerFilename(): ?string { return $this->bannerFilename; }
    public function setBannerFilename(?string $bannerFilename): self { $this->bannerFilename = $bannerFilename; return $this; }
}
