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
    /** Bounds on an offered appointment length: 5 minutes to a full 8-hour day. */
    public const BOOKING_DURATION_MIN = 5;
    public const BOOKING_DURATION_MAX = 480;
    public const BOOKING_DURATION_DEFAULT = 60;

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

    #[ORM\Column(name: 'publicSlug', length: 80, unique: true, nullable: true)]
    private ?string $publicSlug = null;

    #[ORM\Column(name: 'publicProfileEnabled', options: ['default' => false])]
    private bool $publicProfileEnabled = false;

    /** @var list<string> */
    #[ORM\Column(name: 'publicFields', type: 'json', nullable: true)]
    private ?array $publicFields = null;

    #[ORM\Column(name: 'publicBio', length: 500, nullable: true)]
    private ?string $publicBio = null;

    /** Display preference only; it never grants access to a venue or resource. */
    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'preferredVenueId', nullable: true, onDelete: 'SET NULL')]
    private ?Venue $preferredVenue = null;

    // Bookable-person settings. bookable is the admin's switch: until it is on,
    // the person has no booking page and ReservationService refuses the kind.
    // The person owns the rest — their weekly windows live in USER_AVAILABILITY.
    #[ORM\Column(name: 'bookable', options: ['default' => false])]
    private bool $bookable = false;

    /** Offered appointment lengths in minutes, comma-separated ("30,60"). */
    #[ORM\Column(name: 'bookingDurations', length: 60, nullable: true)]
    private ?string $bookingDurations = null;

    #[ORM\Column(name: 'bookingNote', length: 500, nullable: true)]
    private ?string $bookingNote = null;

    /**
     * Les groupes de cette personne (S159b).
     *
     * 🔴 Lus par `getRoles()`, donc sur le chemin de la SÉCURITÉ. En lecture
     * seule : l'écriture reste dans `UserGroupRepository`, qui porte les gardes.
     *
     * @var Collection<int, UserGroupMember>
     */
    #[ORM\OneToMany(mappedBy: 'utilisateur', targetEntity: UserGroupMember::class)]
    private Collection $groupMemberships;

    public function __construct() { $this->createdAt = new \DateTimeImmutable(); $this->groupMemberships = new ArrayCollection(); }
    public function getId(): ?int { return $this->id; }
    public function getEmail(): string { return $this->email; }
    public function setEmail(string $email): self { $this->email = $email; return $this; }
    public function getUsername(): string { return $this->username; }
    public function setUsername(string $username): self { $this->username = $username; return $this; }
    public function getPassword(): string { return $this->password; }
    public function setPassword(string $password): self { $this->password = $password; return $this; }
    public function getUserIdentifier(): string { return $this->email; }
    /**
     * 🔴 **S159f — les rôles viennent des GROUPES, et d'eux seuls.**
     *
     * L'appartenance aux cinq groupes intégrés (`admin`, `manager`, `staff`,
     * `superuser`, `trainers`) est ce qui accorde un rôle de sécurité. La table
     * `UTILISATEUR_ROLE` n'est plus lue.
     *
     * Le chemin qui a mené ici, parce qu'il explique pourquoi c'était sûr :
     * S158c a écrit les lignes d'appartenance que les rôles produisaient déjà
     * (backfill, purement additif) ; S159b a fait rendre l'UNION des deux
     * sources, en prouvant qu'elle était neutre ; et ce pas-ci retire l'ancienne
     * moitié, une fois la correspondance vérifiée personne par personne.
     * `app:s159:roles-shadow` reste le témoin : il compare ce que l'ancienne
     * table rendrait à ce que cette méthode rend, et **sort en erreur si
     * quelqu'un perd un rôle**.
     *
     * ⚠️ **Sans cette étape, ajouter quelqu'un au groupe `staff` ne lui donnait
     * pas `ROLE_STAFF`** : le groupe portait des forfaits, le rôle ouvrait des
     * écrans, et les deux ne se parlaient pas. C'est ce que la fusion répare.
     *
     * 🅿️ **Le « contract » — retirer `UTILISATEUR_ROLE` — est une étape séparée**,
     * qui demande que plus rien n'écrive dans cette table. Tant qu'elle existe,
     * l'union la respecte.
     *
     * ⚠️ **Seuls les groupes INTÉGRÉS deviennent des rôles.** Un groupe libre —
     * « Bénévoles », « week-end » — ne doit pas fabriquer un `ROLE_BENEVOLES` que
     * personne ne teste : il porte des forfaits, pas des écrans d'administration.
     */
    public function getRoles(): array
    {
        // 🔴 **S159f — `UTILISATEUR_ROLE` n'est PLUS LU. Les groupes sont la
        // seule source.** C'est l'étape de contract, et elle n'a été franchie
        // qu'après avoir prouvé, ligne par ligne, que les deux disaient la même
        // chose : `admin` {1,3,4,5,7,9}, `staff` {5,7}, `trainer` {5,7,9} des
        // deux côtés, et les deux lignes de rôle `user` couvertes par le
        // `ROLE_USER` inconditionnel ci-dessous.
        //
        // ⚠️ **`ROLE_USER` reste accordé à tout compte, sans ligne.** C'est
        // l'audience RÉSOLUE `user` : « tout compte actif », que rien n'écrit et
        // que personne ne peut oublier de créer ou de retirer.

        $roles = ['ROLE_USER'];

        foreach ($this->groupMemberships as $membership) {
            $group = $membership->getGroup();
            if ($group === null || !$group->isBuiltin()) {
                continue;
            }
            $roleName = self::ROLE_FOR_GROUP[$group->getGroupKey()] ?? null;
            if ($roleName !== null) {
                // 🔴 **On repasse par `securityRoleFor()`, jamais par une chaîne
                // écrite ici.** C'est elle qui décide de l'orthographe d'un rôle
                // de sécurité ; une seconde table de correspondance produirait
                // une TROISIÈME orthographe, et une permission qui ne s'applique
                // plus est exactement ce que le commentaire de cette méthode
                // met en garde. La table ci-dessous ne fait que le passage
                // clé de groupe → nom de rôle, là où les deux diffèrent.
                $roles[] = self::securityRoleFor($roleName);
            }
        }

        return array_values(array_unique($roles));
    }

    /**
     * Clé de groupe → **nom de rôle** (celui de la table `ROLE`), pour les cinq
     * intégrés qui SONT des rôles.
     *
     * ⚠️ Ce n'est pas une table de rôles de sécurité : la valeur repasse par
     * `securityRoleFor()`. Elle n'existe que parce que les deux nommages
     * diffèrent d'un côté — le groupe est `trainers`, la ligne de rôle est
     * `trainer`.
     *
     * ⚠️ `user` et `guest` n'y sont pas : ce sont des audiences résolues.
     * `ROLE_USER` est déjà accordé plus haut à tout compte, et `guest` est
     * l'absence de compte — les faire figurer ici accorderait un rôle à partir
     * d'une ligne qui n'existe jamais.
     */
    private const ROLE_FOR_GROUP = [
        'admin' => 'admin',
        'manager' => 'manager',
        'staff' => 'staff',
        'superuser' => 'superuser',
        'trainers' => 'trainer',
    ];

    /**
     * The security role a `ROLE` table row maps to.
     *
     * Public and static because the admin screens have to offer the *same* list of
     * security roles this method will later hand the firewall — an operator ticking
     * "formateur" in a settings screen must produce exactly the `ROLE_FORMATEUR`
     * that `isGranted()` will be asked about. Duplicating the match somewhere else
     * is how the two drift apart and a permission silently stops applying.
     */
    public static function securityRoleFor(string $roleName): string
    {
        return match (strtolower($roleName)) {
            'admin' => 'ROLE_ADMIN',
            'staff' => 'ROLE_STAFF',
            'user' => 'ROLE_USER',
            default => str_starts_with(strtoupper($roleName), 'ROLE_') ? strtoupper($roleName) : 'ROLE_' . strtoupper($roleName),
        };
    }
    public function eraseCredentials(): void {}
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
    /**
     * The catalogue key for the stored status — the one place that decides how
     * `statut` is *shown*. Same shape as `Machine::getStatusKey()`, deliberately.
     *
     * 🔴 S134c: three screens printed the raw column — the users list, the user
     * detail page and the member's own profile — so a French page read `actif`,
     * and the other four languages read `actif` too.
     *
     * ⚠️ Unlike `Machine`, this needs no filter counterpart: `UserAdminType`
     * offers exactly `actif` and `inactif`, so each stored value maps to one label
     * and the list's existing raw-value filters stay correct. If a second word ever
     * means "active", add `statusFilterForKey()` here before the list tiles
     * duplicate the way the machine ones did.
     */
    public function getStatusKey(): string
    {
        return match (mb_strtolower(trim($this->statut))) {
            'inactif', 'inactive' => 'user_status.inactive',
            'pending', 'en attente' => 'user_status.pending',
            'banned', 'banni' => 'user_status.banned',
            default => 'user_status.active',
        };
    }
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
    public function isBookable(): bool { return $this->bookable; }
    public function setBookable(bool $bookable): self { $this->bookable = $bookable; return $this; }
    public function getBookingDurations(): ?string { return $this->bookingDurations; }
    public function getBookingNote(): ?string { return $this->bookingNote; }
    public function setBookingNote(?string $bookingNote): self { $this->bookingNote = $bookingNote === null ? null : (mb_substr(trim($bookingNote), 0, 500) ?: null); return $this; }

    /**
     * Appointment lengths this person offers, in minutes. Stored as free text so
     * the person can type "30, 60, 90"; parsed defensively here rather than at
     * every call site, and never empty — a bookable person with no durations set
     * still gets the default hour.
     *
     * @return int[]
     */
    public function getBookingDurationsMinutes(): array
    {
        $minutes = [];
        foreach (explode(',', (string) $this->bookingDurations) as $chunk) {
            $value = (int) trim($chunk);
            if ($value >= self::BOOKING_DURATION_MIN && $value <= self::BOOKING_DURATION_MAX) {
                $minutes[$value] = $value;
            }
        }

        if ($minutes === []) {
            return [self::BOOKING_DURATION_DEFAULT];
        }

        sort($minutes);

        return $minutes;
    }

    /** @param int[]|string $durations */
    public function setBookingDurations(array|string|null $durations): self
    {
        $list = is_array($durations) ? $durations : explode(',', (string) $durations);
        $clean = [];
        foreach ($list as $chunk) {
            $value = (int) trim((string) $chunk);
            if ($value >= self::BOOKING_DURATION_MIN && $value <= self::BOOKING_DURATION_MAX) {
                $clean[$value] = $value;
            }
        }
        sort($clean);
        $this->bookingDurations = $clean === [] ? null : implode(',', $clean);

        return $this;
    }

    public function offersDuration(int $minutes): bool
    {
        return in_array($minutes, $this->getBookingDurationsMinutes(), true);
    }
    public function hasRoleNamed(string $name): bool
    {
        // 🔴 **S159d — cette méthode DOIT répondre comme `getRoles()`, et elle ne
        // le faisait plus.** Elle ne lisait que `UTILISATEUR_ROLE` : depuis que
        // les rôles sont l'union des deux sources (S159b), quelqu'un ajouté au
        // groupe `staff` obtenait `ROLE_STAFF` — donc les écrans — pendant que
        // `isStaff()` répondait NON. Deux réponses à « cette personne est-elle
        // du staff », et elles décidaient de choses différentes : les écrans
        // d'un côté, le PALIER DE QUOTA (`BookingTier::forUser()`) de l'autre.
        //
        // La formuler en fonction de `getRoles()` ferme la question pour de bon :
        // il n'y a plus qu'un endroit qui sache ce qu'une personne est.
        // ⚠️ Et elle passe par `securityRoleFor()`, comme tout le reste, pour que
        // l'orthographe d'un rôle n'ait jamais deux définitions.
        return \in_array(self::securityRoleFor($name), $this->getRoles(), true);
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
    public function getPublicSlug(): ?string { return $this->publicSlug; }
    public function setPublicSlug(?string $slug): self { $this->publicSlug = $slug === null ? null : (trim($slug) ?: null); return $this; }
    public function isPublicProfileEnabled(): bool { return $this->publicProfileEnabled; }
    public function setPublicProfileEnabled(bool $enabled): self { $this->publicProfileEnabled = $enabled; return $this; }
    /** @return list<string> */
    public function getPublicFields(): array { return $this->publicFields ?? []; }
    /** @param list<string> $fields */
    public function setPublicFields(array $fields): self { $this->publicFields = array_values(array_intersect(['name', 'avatar', 'bio', 'badges', 'trainings'], array_unique($fields))); return $this; }
    public function getPublicBio(): ?string { return $this->publicBio; }
    public function setPublicBio(?string $bio): self { $this->publicBio = $bio === null ? null : (mb_substr(trim($bio), 0, 500) ?: null); return $this; }
    public function getPreferredVenue(): ?Venue { return $this->preferredVenue; }
    public function setPreferredVenue(?Venue $venue): self { $this->preferredVenue = $venue; return $this; }
}
