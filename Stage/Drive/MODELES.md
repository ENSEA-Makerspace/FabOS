# FabOS - Spécifications des Modèles de Données (Symfony/Doctrine)
*Version: 1.0 - Migration vers Symfony*
*Date: 15 juin 2026*

---

## 📋 Table des Matières
1. [Vue d'Ensemble](#1-vue-densemble)
2. [Modèles par Domaine](#2-modèles-par-domaine)
   - 2.1 [Core (Configuration)](#21-core-configuration)
   - 2.2 [Authentication (Utilisateurs)](#22-authentication-utilisateurs)
   - 2.3 [Badges](#23-badges)
   - 2.4 [Machines](#24-machines)
   - 2.5 [Reservations](#25-reservations)
   - 2.6 [Evenements](#26-evenements)
   - 2.7 [Formations](#27-formations)
3. [Relations entre Modèles](#3-relations-entre-modèles)
4. [Enums](#4-enums)
5. [Constraints et Validation](#5-constraints-et-validation)
6. [Indexes et Optimisations](#6-indexes-et-optimisations)
7. [Exemples de Code Doctrine](#7-exemples-de-code-doctrine)
8. [Migration depuis Django](#8-migration-depuis-django)

---

## 1. Vue d'Ensemble

Ce document décrit **tous les modèles de données** du projet FabOS, organisés par domaine fonctionnel. Chaque modèle est défini avec :
- **Propriétés** (champs)
- **Relations** (associations avec d'autres modèles)
- **Constraints** (validations)
- **Méthodes** (logique métier)
- **Configuration EasyAdmin**
- **Configuration API Platform**

**Total: 15 modèles** répartis en 7 domaines.

---

## 2. Modèles par Domaine

---

### 2.1 Core (Configuration)

#### ConfigurationFabLab (Singleton)

**Description:** Configuration globale du FabLab. Une seule instance doit exister.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'core_configuration_fablab')]
class ConfigurationFabLab
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(length: 255)]
    private string $nomFablab = 'FabOS Lab';
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $slogan = null;
    
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $descriptionAccueil = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $logo = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $imageHeroAccueil = null;
    
    #[ORM\Column(length: 7)]
    private string $couleurPrimaire = '#A11D56';
    
    #[ORM\Column(length: 7)]
    private string $couleurAlerte = '#FFEB3B';
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Getters et Setters...
}
```

**EasyAdmin:**
```yaml
easy_admin:
    entities:
        ConfigurationFabLab:
            class: App\Entity\Core\ConfigurationFabLab
            label: 'Configuration FabLab'
            icon: 'fa fa-cog'
            disabled_actions: ['new', 'delete']  # Singleton
            form:
                fields:
                    - { property: 'nomFablab' }
                    - { property: 'slogan' }
                    - { property: 'descriptionAccueil', type: 'textarea' }
                    - { property: 'couleurPrimaire', type: 'color' }
                    - { property: 'couleurAlerte', type: 'color' }
```

**API Platform:**
```php
#[ApiResource(
    operations: [
        new Get(collectionOperationName: 'get'),
        new Get(itemOperationName: 'get'),
        new Put(),
    ],
    normalizationContext: ['groups' => ['config:read']],
    denormalizationContext: ['groups' => ['config:write']]
)]
```

---

#### HoraireOuverture

**Description:** Horaires d'ouverture du FabLab par jour de la semaine.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'core_horaire_ouverture')]
class HoraireOuverture
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(type: 'string', length: 10, unique: true, enumType: JourSemaine::class)]
    private JourSemaine $jourSemaine;
    
    #[ORM\Column(type: 'time', nullable: true)]
    private ?\DateTimeInterface $heureOuverture = null;
    
    #[ORM\Column(type: 'time', nullable: true)]
    private ?\DateTimeInterface $heureFermeture = null;
    
    #[ORM\Column(type: 'boolean')]
    private bool $ferme = false;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Getters et Setters...
    
    public function __toString(): string
    {
        if ($this->ferme) {
            return (string) $this->jourSemaine->value . ' - Fermé';
        }
        return sprintf(
            '%s - %s à %s',
            $this->jourSemaine->value,
            $this->heureOuverture?->format('H:i'),
            $this->heureFermeture?->format('H:i')
        );
    }
}
```

**EasyAdmin:**
```yaml
HoraireOuverture:
    class: App\Entity\Core\HoraireOuverture
    label: 'Horaire d\'ouverture'
    icon: 'fa fa-clock'
    form:
        fields:
            - { property: 'jourSemaine', type: 'choice', type_options: { choices: { 'Lundi': 'lundi', 'Mardi': 'mardi', 'Mercredi': 'mercredi', 'Jeudi': 'jeudi', 'Vendredi': 'vendredi', 'Samedi': 'samedi', 'Dimanche': 'dimanche' } } }
            - { property: 'heureOuverture', type: 'time' }
            - { property: 'heureFermeture', type: 'time' }
            - { property: 'ferme', type: 'checkbox' }
```

---

### 2.2 Authentication (Utilisateurs)

#### Utilisateur

**Description:** Utilisateur du système. Hérite de UserInterface et PasswordAuthenticatedUserInterface.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'auth_utilisateur')]
#[ORM\UniqueConstraint(name: 'UNIQ_IDENTIFIANT_RFID', fields: ['identifiantRfid'])]
class Utilisateur implements UserInterface, PasswordAuthenticatedUserInterface
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(length: 180, unique: true)]
    private string $email;
    
    #[ORM\Column(length: 180, unique: true)]
    private string $username;
    
    #[ORM\Column]
    private array $roles = [];
    
    #[ORM\Column]
    private ?string $password = null;
    
    #[ORM\Column(length: 50, nullable: true)]
    private ?string $firstName = null;
    
    #[ORM\Column(length: 50, nullable: true)]
    private ?string $lastName = null;
    
    #[ORM\Column(type: 'string', length: 20, enumType: UtilisateurStatut::class)]
    private UtilisateurStatut $statut = UtilisateurStatut::ETUDIANT;
    
    #[ORM\Column(type: 'integer')]
    private int $tempsPresenceTotal = 0;  // En secondes
    
    #[ORM\Column(length: 50, nullable: true)]
    private ?string $identifiantRfid = null;
    
    #[ORM\Column(type: 'boolean')]
    private bool $isVerified = false;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\ManyToMany(targetEntity: Badge::class, mappedBy: 'utilisateurs')]
    private Collection $badges;
    
    #[ORM\OneToMany(mappedBy: 'utilisateur', targetEntity: Reservation::class)]
    private Collection $reservations;
    
    #[ORM\OneToMany(mappedBy: 'utilisateur', targetEntity: Progression::class)]
    private Collection $progressions;
    
    public function __construct()
    {
        $this->badges = new ArrayCollection();
        $this->reservations = new ArrayCollection();
        $this->progressions = new ArrayCollection();
    }
    
    // UserInterface methods
    public function getUserIdentifier(): string
    {
        return (string) $this->email;
    }
    
    public function getRoles(): array
    {
        $roles = $this->roles;
        $roles[] = 'ROLE_USER';
        return array_unique($roles);
    }
    
    public function setRoles(array $roles): self
    {
        $this->roles = $roles;
        return $this;
    }
    
    public function getPassword(): ?string
    {
        return $this->password;
    }
    
    public function setPassword(string $password): self
    {
        $this->password = $password;
        return $this;
    }
    
    public function eraseCredentials(): void
    {
        // Nothing to do
    }
    
    public function getFullName(): string
    {
        return trim(sprintf('%s %s', $this->firstName, $this->lastName)) ?: $this->username;
    }
    
    // Getters et Setters...
}
```

**EasyAdmin:**
```yaml
Utilisateur:
    class: App\Entity\Authentication\Utilisateur
    label: 'Utilisateur'
    icon: 'fa fa-user'
    list:
        fields:
            - 'email'
            - 'username'
            - 'firstName'
            - 'lastName'
            - 'statut'
            - 'tempsPresenceTotal'
            - 'identifiantRfid'
            - 'isVerified'
            - 'createdAt'
        filters: ['statut', 'isVerified']
        actions: ['show', 'edit', 'delete']
    form:
        fields:
            - { property: 'email', type: 'email' }
            - { property: 'username' }
            - { property: 'plainPassword', type: 'password', label: 'Mot de passe', type_options: { mapped: false } }
            - { property: 'firstName' }
            - { property: 'lastName' }
            - { property: 'statut', type: 'choice', type_options: { choices: { 'Étudiant': 'etudiant', 'Externe': 'externe', 'Partenaire': 'partenaire', 'Personnel': 'personnel' } } }
            - { property: 'identifiantRfid' }
            - { property: 'roles', type: 'choice', type_options: { multiple: true, choices: { 'Admin': 'ROLE_ADMIN', 'Staff': 'ROLE_STAFF' } } }
```

---

### 2.3 Badges

#### Badge

**Description:** Badge ou certification qu'un utilisateur peut obtenir.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'badges_badge')]
class Badge
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(length: 255)]
    private string $nom;
    
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;
    
    #[ORM\Column(length: 100, nullable: true)]
    private ?string $icone = null;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\ManyToMany(targetEntity: Utilisateur::class, inversedBy: 'badges')]
    private Collection $utilisateurs;
    
    #[ORM\ManyToMany(targetEntity: Machine::class, mappedBy: 'preRequisBadges')]
    private Collection $machines;
    
    #[ORM\OneToMany(mappedBy: 'badgeRecompense', targetEntity: Formation::class)]
    private Collection $formations;
    
    public function __construct()
    {
        $this->utilisateurs = new ArrayCollection();
        $this->machines = new ArrayCollection();
        $this->formations = new ArrayCollection();
    }
    
    public function __toString(): string
    {
        return $this->nom;
    }
    
    // Getters et Setters...
}
```

**EasyAdmin:**
```yaml
Badge:
    class: App\Entity\Badges\Badge
    label: 'Badge'
    icon: 'fa fa-award'
    list:
        fields:
            - 'nom'
            - 'description'
            - 'icone'
            - 'createdAt'
    form:
        fields:
            - 'nom'
            - { property: 'description', type: 'textarea' }
            - 'icone'
```

**API Platform:**
```php
#[ApiResource(
    collectionOperations: ['get', 'post'],
    itemOperations: ['get', 'put', 'delete'],
    normalizationContext: ['groups' => ['badge:read']],
    denormalizationContext: ['groups' => ['badge:write']]
)]
```

---

### 2.4 Machines

#### Machine

**Description:** Machine disponible dans le FabLab.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'machines_machine')]
class Machine
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(length: 255)]
    private string $nom;
    
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $descriptionTechnique = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $localisation = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $photo = null;
    
    #[ORM\Column(type: 'string', length: 20, enumType: MachineStatut::class)]
    private MachineStatut $statut = MachineStatut::DISPONIBLE;
    
    #[ORM\Column(type: 'integer')]
    #[Assert\Positive]
    #[Assert\GreaterThanOrEqual(5)]
    private int $granulariteReservation = 30;  // En minutes
    
    #[ORM\Column(type: 'integer', nullable: true)]
    #[Assert\Positive]
    private ?int $limiteReservationsSimultanees = null;
    
    #[ORM\Column(length: 100, unique: true, nullable: true)]
    private ?string $machineToken = null;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\ManyToMany(targetEntity: Badge::class, inversedBy: 'machines')]
    #[ORM\JoinTable(name: 'machines_machine_badge')]
    private Collection $preRequisBadges;
    
    #[ORM\OneToMany(mappedBy: 'machine', targetEntity: Reservation::class)]
    private Collection $reservations;
    
    #[ORM\OneToMany(mappedBy: 'machine', targetEntity: LogUtilisation::class)]
    private Collection $logsUtilisation;
    
    public function __construct()
    {
        $this->preRequisBadges = new ArrayCollection();
        $this->reservations = new ArrayCollection();
        $this->logsUtilisation = new ArrayCollection();
    }
    
    public function __toString(): string
    {
        return $this->nom;
    }
    
    #[ORM\PrePersist]
    public function generateToken(): void
    {
        if (empty($this->machineToken)) {
            $this->machineToken = bin2hex(random_bytes(50));
        }
    }
    
    // Getters et Setters...
}
```

**EasyAdmin:**
```yaml
Machine:
    class: App\Entity\Machines\Machine
    label: 'Machine'
    icon: 'fa fa-cog'
    list:
        fields:
            - 'nom'
            - 'localisation'
            - 'statut'
            - 'granulariteReservation'
            - 'limiteReservationsSimultanees'
            - 'createdAt'
    form:
        fields:
            - 'nom'
            - { property: 'descriptionTechnique', type: 'textarea' }
            - 'localisation'
            - 'photo'
            - { property: 'statut', type: 'choice', type_options: { choices: { 'Disponible': 'disponible', 'Maintenance': 'maintenance', 'Panne': 'panne' } } }
            - 'granulariteReservation'
            - 'limiteReservationsSimultanees'
            - { property: 'preRequisBadges', type: 'entity', type_options: { class: 'App\Entity\Badges\Badge', multiple: true } }
```

---

#### LogUtilisation

**Description:** Historique d'utilisation des machines.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'machines_log_utilisation')]
class LogUtilisation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Machine::class, inversedBy: 'logsUtilisation')]
    #[ORM\JoinColumn(nullable: false)]
    private Machine $machine;
    
    #[ORM\ManyToOne(targetEntity: Utilisateur::class, inversedBy: 'logsUtilisation')]
    #[ORM\JoinColumn(nullable: false)]
    private Utilisateur $utilisateur;
    
    #[ORM\Column(type: 'datetime')]
    private \DateTimeInterface $dateDebut;
    
    #[ORM\Column(type: 'datetime', nullable: true)]
    private ?\DateTimeInterface $dateFin = null;
    
    #[ORM\Column(type: 'integer', nullable: true)]
    private ?int $dureeSecondes = null;  // En secondes
    
    #[ORM\Column(length: 50)]
    private string $source = 'manual';
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }
    
    // Getters et Setters...
}
```

---

### 2.5 Reservations

#### Reservation

**Description:** Réservation d'une machine par un utilisateur.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'reservations_reservation')]
#[ORM\HasLifecycleCallbacks]
class Reservation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Utilisateur::class, inversedBy: 'reservations')]
    #[ORM\JoinColumn(nullable: false)]
    private Utilisateur $utilisateur;
    
    #[ORM\ManyToOne(targetEntity: Machine::class, inversedBy: 'reservations')]
    #[ORM\JoinColumn(nullable: false)]
    private Machine $machine;
    
    #[ORM\Column(type: 'datetime')]
    private \DateTimeInterface $dateDebut;
    
    #[ORM\Column(type: 'datetime')]
    private \DateTimeInterface $dateFin;
    
    #[ORM\Column(type: 'string', length: 20, enumType: ReservationStatut::class)]
    private ReservationStatut $statut = ReservationStatut::VALIDE;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
        $this->updatedAt = new \DateTimeImmutable();
    }
    
    #[ORM\PrePersist]
    #[ORM\PreUpdate]
    public function validate(): void
    {
        $this->validateGranularity();
        $this->validateBadges();
        $this->validateNoOverlap();
        $this->validateSimultaneousLimit();
    }
    
    private function validateGranularity(): void
    {
        $granularite = $this->machine->getGranulariteReservation();
        $minutesFromMidnight = (int) $this->dateDebut->format('H') * 60 + (int) $this->dateDebut->format('i');
        
        if ($minutesFromMidnight % $granularite !== 0) {
            throw new \LogicException(
                sprintf('La date de début doit être alignée sur %d minutes', $granularite)
            );
        }
        
        $dureeMinutes = (int) $this->dateDebut->diff($this->dateFin)->i;
        if ($dureeMinutes % $granularite !== 0) {
            throw new \LogicException(
                sprintf('La durée doit être un multiple de %d minutes', $granularite)
            );
        }
    }
    
    private function validateBadges(): void
    {
        $requiredBadges = $this->machine->getPreRequisBadges();
        if ($requiredBadges->isEmpty()) {
            return;
        }
        
        $userBadges = $this->utilisateur->getBadges();
        $missing = $requiredBadges->filter(
            fn(Badge $b) => !$userBadges->contains($b)
        );
        
        if ($missing->count() > 0) {
            $badgeNames = $missing->map(fn(Badge $b) => $b->getNom())->toArray();
            throw new \LogicException(
                sprintf('Badges manquants: %s', implode(', ', $badgeNames))
            );
        }
    }
    
    private function validateNoOverlap(): void
    {
        $existing = $this->machine->getReservations()->filter(
            fn(Reservation $r) => $r->getId() !== $this->id 
                && in_array($r->getStatut(), [ReservationStatut::VALIDE, ReservationStatut::EN_COURS])
                && $r->getDateDebut() < $this->dateFin
                && $r->getDateFin() > $this->dateDebut
        );
        
        if ($existing->count() > 0) {
            throw new \LogicException('Une réservation chevauche déjà cette machine pour cette période');
        }
    }
    
    private function validateSimultaneousLimit(): void
    {
        $limit = $this->machine->getLimiteReservationsSimultanees();
        if ($limit === null) {
            return;
        }
        
        $activeReservations = $this->utilisateur->getReservations()->filter(
            fn(Reservation $r) => $r->getId() !== $this->id 
                && in_array($r->getStatut(), [ReservationStatut::VALIDE, ReservationStatut::EN_COURS])
                && $r->getMachine() === $this->machine
        );
        
        if ($activeReservations->count() >= $limit) {
            throw new \LogicException(
                sprintf('L\'utilisateur a déjà atteint la limite de %d réservations simultanées', $limit)
            );
        }
    }
    
    // Getters et Setters...
    
    public function __toString(): string
    {
        return sprintf('%s - %s (%s)', $this->utilisateur->getFullName(), $this->machine->getNom(), $this->dateDebut->format('Y-m-d H:i'));
    }
}
```

**EasyAdmin:**
```yaml
Reservation:
    class: App\Entity\Reservations\Reservation
    label: 'Réservation'
    icon: 'fa fa-calendar'
    list:
        fields:
            - 'utilisateur'
            - 'machine'
            - 'dateDebut'
            - 'dateFin'
            - 'statut'
            - 'createdAt'
        filters: ['statut', 'machine', 'utilisateur']
        defaultSort: ['dateDebut' => 'DESC']
    form:
        fields:
            - { property: 'utilisateur', type: 'entity', type_options: { class: 'App\Entity\Authentication\Utilisateur' } }
            - { property: 'machine', type: 'entity', type_options: { class: 'App\Entity\Machines\Machine' } }
            - { property: 'dateDebut', type: 'datetime' }
            - { property: 'dateFin', type: 'datetime' }
            - { property: 'statut', type: 'choice', type_options: { choices: { 'Validée': 'validee', 'Annulée': 'annulee', 'En cours': 'en_cours', 'Terminée': 'terminee' } } }
```

---

### 2.6 Evenements

#### Evenement

**Description:** Événement ou atelier affiché dans le calendrier global.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'evenements_evenement')]
class Evenement
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(type: 'string', length: 255)]
    private string $titre;
    
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;
    
    #[ORM\Column(type: 'datetime')]
    private \DateTimeInterface $dateDebut;
    
    #[ORM\Column(type: 'datetime')]
    private \DateTimeInterface $dateFin;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $lieu = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $image = null;
    
    #[ORM\Column(type: 'string', length: 50, enumType: EvenementType::class)]
    private EvenementType $typeEvenement = EvenementType::AUTRE;
    
    #[ORM\ManyToOne(targetEntity: Formation::class, inversedBy: 'evenements')]
    private ?Formation $formation = null;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
        $this->updatedAt = new \DateTimeImmutable();
    }
    
    // Getters et Setters...
    
    public function __toString(): string
    {
        return $this->titre;
    }
}
```

**EasyAdmin:**
```yaml
Evenement:
    class: App\Entity\Evenements\Evenement
    label: 'Événement'
    icon: 'fa fa-calendar-alt'
    list:
        fields:
            - 'titre'
            - 'dateDebut'
            - 'dateFin'
            - 'lieu'
            - 'typeEvenement'
            - 'formation'
        defaultSort: ['dateDebut' => 'ASC']
    form:
        fields:
            - 'titre'
            - { property: 'description', type: 'textarea' }
            - { property: 'dateDebut', type: 'datetime' }
            - { property: 'dateFin', type: 'datetime' }
            - 'lieu'
            - 'image'
            - { property: 'typeEvenement', type: 'choice', type_options: { choices: { 'Atelier': 'atelier', 'Formation': 'formation', 'Autre': 'autre' } } }
            - { property: 'formation', type: 'entity', type_options: { class: 'App\Entity\Formations\Formation', required: false } }
```

---

### 2.7 Formations

#### Formation

**Description:** Formation proposée par le FabLab (style Moodle).

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_formation')]
class Formation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\Column(length: 255)]
    private string $titre;
    
    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $image = null;
    
    #[ORM\ManyToOne(targetEntity: Badge::class, inversedBy: 'formations')]
    private ?Badge $badgeRecompense = null;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\OneToMany(mappedBy: 'formation', targetEntity: SectionFormation::class, cascade: ['persist'], orphanRemoval: true)]
    private Collection $sections;
    
    #[ORM\OneToOne(mappedBy: 'formation', targetEntity: Quiz::class, cascade: ['persist'])]
    private ?Quiz $quiz = null;
    
    #[ORM\OneToMany(mappedBy: 'formation', targetEntity: Evenement::class)]
    private Collection $evenements;
    
    #[ORM\OneToMany(mappedBy: 'formation', targetEntity: Progression::class)]
    private Collection $progressions;
    
    public function __construct()
    {
        $this->sections = new ArrayCollection();
        $this->evenements = new ArrayCollection();
        $this->progressions = new ArrayCollection();
    }
    
    public function __toString(): string
    {
        return $this->titre;
    }
    
    // Getters et Setters...
}
```

---

#### SectionFormation

**Description:** Section d'une formation.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_section_formation')]
class SectionFormation
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Formation::class, inversedBy: 'sections')]
    #[ORM\JoinColumn(nullable: false)]
    private Formation $formation;
    
    #[ORM\Column(length: 255)]
    private string $titre;
    
    #[ORM\Column(type: 'text')]
    private string $contenu;
    
    #[ORM\Column(length: 255, nullable: true)]
    private ?string $videoUrl = null;
    
    #[ORM\Column(type: 'integer')]
    private int $ordre = 0;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
        $this->updatedAt = new \DateTimeImmutable();
    }
    
    public function __toString(): string
    {
        return sprintf('%s - %s', $this->formation->getTitre(), $this->titre);
    }
    
    // Getters et Setters...
}
```

---

#### Quiz

**Description:** Quiz associé à une formation.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_quiz')]
class Quiz
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\OneToOne(targetEntity: Formation::class, inversedBy: 'quiz')]
    #[ORM\JoinColumn(nullable: false)]
    private Formation $formation;
    
    #[ORM\Column(type: 'integer')]
    #[Assert\Range(min: 0, max: 100)]
    private int $noteMinimaleValidation = 80;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\OneToMany(mappedBy: 'quiz', targetEntity: Question::class, cascade: ['persist'], orphanRemoval: true)]
    private Collection $questions;
    
    public function __construct()
    {
        $this->questions = new ArrayCollection();
        $this->createdAt = new \DateTimeImmutable();
        $this->updatedAt = new \DateTimeImmutable();
    }
    
    public function __toString(): string
    {
        return sprintf('Quiz - %s', $this->formation->getTitre());
    }
    
    // Getters et Setters...
}
```

---

#### Question

**Description:** Question d'un quiz.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_question')]
class Question
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Quiz::class, inversedBy: 'questions')]
    #[ORM\JoinColumn(nullable: false)]
    private Quiz $quiz;
    
    #[ORM\Column(type: 'text')]
    private string $texte;
    
    #[ORM\Column(type: 'string', length: 20, enumType: QuestionType::class)]
    private QuestionType $typeQuestion = QuestionType::QCM;
    
    #[ORM\Column(type: 'integer')]
    private int $ordre = 0;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $updatedAt;
    
    // Relations
    #[ORM\OneToMany(mappedBy: 'question', targetEntity: Choix::class, cascade: ['persist'], orphanRemoval: true)]
    private Collection $choix;
    
    public function __construct()
    {
        $this->choix = new ArrayCollection();
        $this->createdAt = new \DateTimeImmutable();
        $this->updatedAt = new \DateTimeImmutable();
    }
    
    public function __toString(): string
    {
        return substr($this->texte, 0, 50);
    }
    
    // Getters et Setters...
}
```

---

#### Choix

**Description:** Choix de réponse pour une question QCM.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_choix')]
class Choix
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Question::class, inversedBy: 'choix')]
    #[ORM\JoinColumn(nullable: false)]
    private Question $question;
    
    #[ORM\Column(length: 255)]
    private string $texte;
    
    #[ORM\Column(type: 'boolean')]
    private bool $estCorrect = false;
    
    #[ORM\Column(type: 'integer')]
    private int $ordre = 0;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $createdAt;
    
    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }
    
    public function __toString(): string
    {
        return $this->texte;
    }
    
    // Getters et Setters...
}
```

---

#### Progression

**Description:** Progression d'un utilisateur dans une formation.

**Attributs Doctrine:**
```php
#[ORM\Entity]
#[ORM\Table(name: 'formations_progression')]
#[ORM\UniqueConstraint(name: 'UNIQ_UTILISATEUR_FORMATION', fields: ['utilisateur', 'formation'])]
class Progression
{
    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column(type: 'integer')]
    private ?int $id = null;
    
    #[ORM\ManyToOne(targetEntity: Utilisateur::class, inversedBy: 'progressions')]
    #[ORM\JoinColumn(nullable: false)]
    private Utilisateur $utilisateur;
    
    #[ORM\ManyToOne(targetEntity: Formation::class, inversedBy: 'progressions')]
    #[ORM\JoinColumn(nullable: false)]
    private Formation $formation;
    
    #[ORM\Column(type: 'integer')]
    #[Assert\Range(min: 0, max: 100)]
    private int $score = 0;
    
    #[ORM\Column(type: 'boolean')]
    private bool $estCompletee = false;
    
    #[ORM\Column(type: 'datetime_immutable')]
    private \DateTimeImmutable $dateDebut;
    
    #[ORM\Column(type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $dateCompletion = null;
    
    public function __construct()
    {
        $this->dateDebut = new \DateTimeImmutable();
    }
    
    public function __toString(): string
    {
        return sprintf('%s - %s', $this->utilisateur->getFullName(), $this->formation->getTitre());
    }
    
    // Getters et Setters...
}
```

---

## 3. Relations entre Modèles
// granularite == échelle de temps de réservation (tranche de 30min, 10min, 1h ...) 
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         UTILISATEUR (UserInterface)                         │
├─────────────────────────────────────────────────────────────────────────────┤
│ id | email | username | password | firstName | lastName | statut            │
│ identifiantRfid | tempsPresenceTotal | roles | isVerified | createdAt       │
└────────────────┬─────────────────┬──────────────────────┬───────────────────┘
                 │                 │                      │
                 ▼                 ▼                      ▼
┌─────────────────────────┐ ┌────────────────────┐ ┌──────────────────────┐
│      BADGE (M2M)        │ │   RESERVATION      │ │    PROGRESSION       │
├─────────────────────────┤ ├────────────────────┤ ├──────────────────────┤
│ id | nom | description  │ │ id | user | machine│ │ id | user | formation│
│ icone | createdAt       │ │ dateDebut | dateFin│ │ score | completed    │
└─────────────────────────┘ │ statut | created   │ │ dateDebut | dateEnd  │
                            └──────────┬─────────┘ └──────────┬───────────┘
                                       │                      │
                                       ▼                      ▼
                              ┌───────────────────────┐ ┌──────────────────┐
                              │        MACHINE        │ │     FORMATION    │
                              ├───────────────────────┤ ├──────────────────┤
                              │ id | nom | description│ │id | titre | desc │
                              │ localisation | photo  │ │ image | badge    │
                              │ statut | granularite  │ └──────┬───────────┘
                              │ limiteReservations    │        │
                              │ machineToken          │        ▼
                              │ createdAt | updated   │ ┌──────────────────┐
                              └──────────┬────────────┘ │ SECTION          │
                                         │              ├──────────────────┤
                                         │              │ id | formation   │
                                         │              │ titre | contenu  │
                                         ▼              │ videoUrl | ordre │
                              ┌────────────────────┐    │ createdAt        │
                              │   LOG UTILISATION  │    └──────┬───────────┘
                              ├────────────────────┤           │
                              │ id | machine | user│           ▼
                              │ dateDebut | dateFin│       ┌─────────────────┐
                              │ duree | source     │       │       QUIZ      │
                              │ createdAt          │       ├─────────────────┤
                              └────────────────────┘       │ id | formation  │
                                                           │ noteMinimale    │
                                                           └──────┬──────────┘
                                                                  │
                                                                  ▼
                                                            ┌──────────────────┐
                                                            │     QUESTION     │
                                                            ├──────────────────┤
                                                            │ id | quiz | texte│
                                                            │ type | ordre     │
                                                            └──────┬───────────┘
                                                                   │
                                                                   ▼
                                                            ┌──────────────────┐
                                                            │      CHOIX       │
                                                            ├──────────────────┤
                                                            │ id | question    │
                                                            │texte | estCorrect│
                                                            │ ordre            │
                                                            └──────────────────┘
```


``` mermaid 

erDiagram
    %% ========== ENTITÉS PRINCIPALES ==========
    UTILISATEUR ||--o{ RESERVATION : "1:N"
    UTILISATEUR ||--o{ PROGRESSION : "1:N"
    UTILISATEUR ||--o{ LOG_UTILISATION : "1:N"
    UTILISATEUR ||--o{ BADGE : "1:N"

    MACHINE ||--o{ RESERVATION : "1:N"
    MACHINE ||--o{ LOG_UTILISATION : "1:N"

    FORMATION ||--o{ PROGRESSION : "1:N"
    FORMATION ||--o{ SECTION : "1:N"
    FORMATION ||--o{ QUIZ : "1:N"

    SECTION ||--o{ QUIZ : "1:N"
    QUIZ ||--o{ QUESTION : "1:N"
    QUESTION ||--o{ CHOIX : "1:N"

    %% ========== TABLES ==========
    UTILISATEUR {
        int id PK
        string email
        string username
        string password
        string firstName
        string lastName
        string statut
        string identifiantRfid
        int tempsPresenceTotal
        string[] roles
        bool isVerified
        datetime createdAt
        int[] badgeList
    }

    BADGE {
        int id PK
        string nom
        string description
        string icone
        datetime createdAt
        
    }

    RESERVATION {
        int id PK
        int userId FK
        int machineId FK
        datetime dateDebut
        datetime dateFin
        string motif
        datetime created
    }

    PROGRESSION {
        int id PK
        int userId FK
        int formationId FK
        int score
        bool completed
        datetime dateDebut
        datetime dateEnd
    }

    MACHINE {
        int id PK
        string nom
        string description
        string localisation
        string photo
        string statut
        string granularite
        int limiteReservations
        string machineToken 
        datetime createdAt
        datetime updated
    }

    LOG_UTILISATION {
        int id PK
        int machineId FK
        int userId FK
        datetime dateDebut
        datetime dateFin
        int duree
        string source
        datetime createdAt
    }

    FORMATION {
        int id PK
        int badgeId FK
        string titre
        string desc
        string image
        
    }

    SECTION {
        int id PK
        int formationId FK
        string titre
        string contenu
        string videoUrl
        int ordre
        datetime createdAt
    }

    QUIZ {
        int id PK
        int formationId FK
        int noteMinimale
    }

    QUESTION {
        int id PK
        int quizId FK
        string texte
        string type
        int ordre
    }

    CHOIX {
        int id PK
        int questionId FK
        string texte
        bool estCorrect
        int ordre
    }


```


**Schéma simplifié:**
```
Utilisateur ←(M2M)→ Badge
Utilisateur → Reservation → Machine ←(M2M)→ Badge
Utilisateur → Progression → Formation → SectionFormation
Formation → Quiz → Question → Choix
Machine → LogUtilisation → Utilisateur
Evenement → Formation (optionnel)
```

---

## 4. Enums

### 4.1 JourSemaine

```php
<?php
// src/Enum/JourSemaine.php
namespace App\Enum;

enum JourSemaine: string
{
    case LUNDI = 'lundi';
    case MARDI = 'mardi';
    case MERCREDI = 'mercredi';
    case JEUDI = 'jeudi';
    case VENDREDI = 'vendredi';
    case SAMEDI = 'samedi';
    case DIMANCHE = 'dimanche';
    
    public function label(): string
    {
        return match($this) {
            self::LUNDI => 'Lundi',
            self::MARDI => 'Mardi',
            self::MERCREDI => 'Mercredi',
            self::JEUDI => 'Jeudi',
            self::VENDREDI => 'Vendredi',
            self::SAMEDI => 'Samedi',
            self::DIMANCHE => 'Dimanche',
        };
    }
}
```

### 4.2 UtilisateurStatut

```php
<?php
// src/Enum/UtilisateurStatut.php
namespace App\Enum;

enum UtilisateurStatut: string
{
    case ETUDIANT = 'etudiant';
    case EXTERNE = 'externe';
    case PARTENAIRE = 'partenaire';
    case PERSONNEL = 'personnel';
    
    public function label(): string
    {
        return match($this) {
            self::ETUDIANT => 'Étudiant',
            self::EXTERNE => 'Externe',
            self::PARTENAIRE => 'Partenaire',
            self::PERSONNEL => 'Personnel',
        };
    }
}
```

### 4.3 MachineStatut

```php
<?php
// src/Enum/MachineStatut.php
namespace App\Enum;

enum MachineStatut: string
{
    case DISPONIBLE = 'disponible';
    case MAINTENANCE = 'maintenance';
    case PANNE = 'panne';
    
    public function label(): string
    {
        return match($this) {
            self::DISPONIBLE => 'Disponible',
            self::MAINTENANCE => 'En maintenance',
            self::PANNE => 'En panne',
        };
    }
    
    public function cssClass(): string
    {
        return match($this) {
            self::DISPONIBLE => 'bg-green-500',
            self::MAINTENANCE => 'bg-yellow-500',
            self::PANNE => 'bg-red-500',
        };
    }
}
```

### 4.4 ReservationStatut

```php
<?php
// src/Enum/ReservationStatut.php
namespace App\Enum;

enum ReservationStatut: string
{
    case VALIDE = 'validee';
    case ANNULE = 'annulee';
    case EN_COURS = 'en_cours';
    case TERMINEE = 'terminee';
    
    public function label(): string
    {
        return match($this) {
            self::VALIDE => 'Validée',
            self::ANNULE => 'Annulée',
            self::EN_COURS => 'En cours',
            self::TERMINEE => 'Terminée',
        };
    }
}
```

### 4.5 EvenementType

```php
<?php
// src/Enum/EvenementType.php
namespace App\Enum;

enum EvenementType: string
{
    case ATELIER = 'atelier';
    case FORMATION = 'formation';
    case AUTRE = 'autre';
    
    public function label(): string
    {
        return match($this) {
            self::ATELIER => 'Atelier',
            self::FORMATION => 'Formation',
            self::AUTRE => 'Autre',
        };
    }
}
```

### 4.6 QuestionType

```php
<?php
// src/Enum/QuestionType.php
namespace App\Enum;

enum QuestionType: string
{
    case QCM = 'qcm';
    case OUVERT = 'ouvert';
    
    public function label(): string
    {
        return match($this) {
            self::QCM => 'Choix multiples',
            self::OUVERT => 'Question ouverte',
        };
    }
}
```

---

## 5. Constraints et Validation

### 5.1 Constraints Symfony

Toutes les entités doivent avoir des **constraints de validation** :

```php
use Symfony\Component\Validator\Constraints as Assert;

// Exemple dans Machine
#[ORM\Column(type: 'integer')]
#[Assert\Positive]
#[Assert\GreaterThanOrEqual(5)]
private int $granulariteReservation = 30;

// Exemple dans Reservation
#[ORM\Column(type: 'datetime')]
#[Assert\NotNull]
#[Assert\GreaterThanOrEqual('today')]
private \DateTimeInterface $dateDebut;

#[ORM\Column(type: 'datetime')]
#[Assert\NotNull]
#[Assert\Expression(
    "this.getDateDebut() < this.getDateFin()",
    message: "La date de fin doit être après la date de début"
)]
private \DateTimeInterface $dateFin;
```

### 5.2 Validation Custom

Pour les validations complexes (comme la validation des réservations), utiliser :

**Option 1: Méthode validate() avec #[HasLifecycleCallbacks] (recommandé)**
```php
#[ORM\HasLifecycleCallbacks]
class Reservation
{
    #[ORM\PrePersist]
    #[ORM\PreUpdate]
    public function validate(): void
    {
        $this->validateGranularity();
        $this->validateBadges();
        $this->validateNoOverlap();
    }
    
    private function validateGranularity(): void
    {
        // ... logique de validation
    }
}
```

**Option 2: Validator custom**
```php
// src/Validator/Constraints/ValidReservation.php
namespace App\Validator\Constraints;

use Symfony\Component\Validator\Constraint;

/** @Annotation */
class ValidReservation extends Constraint
{
    public $message = 'La réservation est invalide: {{ reason }}';
}

// src/Validator/Constraints/ValidReservationValidator.php
class ValidReservationValidator extends ConstraintValidator
{
    public function validate($reservation, Constraint $constraint): void
    {
        // Logique de validation
    }
}

// Dans l'entité
#[ValidReservation]
class Reservation { ... }
```

---

## 6. Indexes et Optimisations

### 6.1 Indexes Recommandés

```php
// Dans les entités, ajouter des indexes pour les colonnes fréquemment interrogées

// Exemple dans Reservation
#[ORM\Table(name: 'reservations_reservation')]
#[ORM\Index(name: 'IDX_RESA_MACHINE_DATE', columns: ['machine_id', 'date_debut'])]
#[ORM\Index(name: 'IDX_RESA_UTILISATEUR', columns: ['utilisateur_id'])]
#[ORM\Index(name: 'IDX_RESA_STATUT', columns: ['statut'])]
class Reservation { ... }

// Exemple dans Machine
#[ORM\Table(name: 'machines_machine')]
#[ORM\Index(name: 'IDX_MACHINE_STATUT', columns: ['statut'])]
class Machine { ... }

// Exemple dans Utilisateur
#[ORM\Table(name: 'auth_utilisateur')]
#[ORM\Index(name: 'IDX_USER_EMAIL', columns: ['email'])]
#[ORM\Index(name: 'IDX_USER_RFID', columns: ['identifiant_rfid'])]
class Utilisateur { ... }
```

### 6.2 Eager Loading

Pour éviter les problèmes N+1, utiliser le **fetch join** dans les repositories :

```php
// src/Repository/MachineRepository.php
namespace App\Repository;

use App\Entity\Machines\Machine;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;

class MachineRepository extends ServiceEntityRepository
{
    public function findAllWithBadges(): array
    {
        return $this->createQueryBuilder('m')
            ->leftJoin('m.preRequisBadges', 'b')
            ->addSelect('b')
            ->getQuery()
            ->getResult();
    }
    
    public function findWithReservationsAndUser(int $id): ?Machine
    {
        return $this->createQueryBuilder('m')
            ->leftJoin('m.reservations', 'r')
            ->leftJoin('r.utilisateur', 'u')
            ->addSelect('r', 'u')
            ->where('m.id = :id')
            ->setParameter('id', $id)
            ->getQuery()
            ->getOneOrNullResult();
    }
}
```

---

## 7. Exemples de Code Doctrine

### 7.1 Requête Simple

```php
// Dans un contrôleur
$machines = $machineRepository->findAll();

// Avec criteria
$machinesDisponibles = $machineRepository->findBy([
    'statut' => MachineStatut::DISPONIBLE
]);

// Avec ordre
$machines = $machineRepository->findBy([], [
    'nom' => 'ASC'
]);
```

### 7.2 Requête avec Jointure

```php
// Dans un repository
public function findMachinesByUserBadges(Utilisateur $user): array
{
    return $this->createQueryBuilder('m')
        ->leftJoin('m.preRequisBadges', 'b')
        ->leftJoin('b.utilisateurs', 'u')
        ->where('u.id = :user_id')
        ->orWhere('m.preRequisBadges IS EMPTY')
        ->setParameter('user_id', $user->getId())
        ->getQuery()
        ->getResult();
}
```

### 7.3 Requête avec DQL

```php
$dql = "SELECT m FROM App\Entity\Machines\Machine m " .
       "WHERE m.statut = :statut " .
       "AND m.granulariteReservation <= :max_granularite";

$query = $entityManager->createQuery($dql)
    ->setParameter('statut', MachineStatut::DISPONIBLE)
    ->setParameter('max_granularite', 60);

$results = $query->getResult();
```

### 7.4 Requête avec QueryBuilder

```php
$queryBuilder = $entityManager->getRepository(Machine::class)->createQueryBuilder('m');

$queryBuilder
    ->where('m.statut = :statut')
    ->andWhere('m.localisation LIKE :localisation')
    ->setParameter('statut', MachineStatut::DISPONIBLE)
    ->setParameter('localisation', '%Atelier%')
    ->orderBy('m.nom', 'ASC')
    ->setMaxResults(10);

$results = $queryBuilder->getQuery()->getResult();
```

---

## 8. Migration depuis Django

### 8.1 Correspondance Django → Symfony/Doctrine

| Django | Symfony/Doctrine | Exemple |
|--------|-------------------|---------|
| `models.Model` | Entité avec `#[ORM\Entity]` | `#[ORM\Entity] class Machine` |
| `CharField` | `#[ORM\Column(type: 'string')]` | `#[ORM\Column(length: 255)]` |
| `TextField` | `#[ORM\Column(type: 'text')]` | `#[ORM\Column(type: 'text')]` |
| `IntegerField` | `#[ORM\Column(type: 'integer')]` | `#[ORM\Column(type: 'integer')]` |
| `DateTimeField` | `#[ORM\Column(type: 'datetime')]` | `#[ORM\Column(type: 'datetime')]` |
| `BooleanField` | `#[ORM\Column(type: 'boolean')]` | `#[ORM\Column(type: 'boolean')]` |
| `ForeignKey` | `#[ORM\ManyToOne]` | `#[ORM\ManyToOne(targetEntity: Badge::class)]` |
| `ManyToManyField` | `#[ORM\ManyToMany]` | `#[ORM\ManyToMany(targetEntity: Badge::class)]` |
| `choices` | Enum PHP 8.1+ | `enum MachineStatut: string` |
| `auto_now_add` | `#[ORM\Column(type: 'datetime_immutable')]` + constructeur | `private \DateTimeImmutable $createdAt;` |
| `auto_now` | `#[ORM\Column(type: 'datetime_immutable')]` + lifecycle callback | `#[ORM\PreUpdate]` |
| `get_object_or_404()` | `$repository->findOrThrow()` | `$machine = $machineRepo->findOrThrow($id)` |

### 8.2 Étapes de Migration

1. **Créer toutes les entités** Symfony/Doctrine
2. **Générer les migrations**
   ```bash
   php bin/console make:migration
   php bin/console doctrine:migrations:migrate
   ```
3. **Configurer EasyAdmin** pour chaque entité
4. **Créer les repositories** personnalisés
5. **Migrer la logique métier** des méthodes `clean()` Django vers :
   - Constraints Symfony
   - Méthodes `validate()` avec lifecycle callbacks
   - Services de validation
6. **Migrer les vues** Django → Twig
7. **Migrer les contrôleurs** Django → Symfony
8. **Configurer l'authentification** CAS
9. **Configurer API Platform**

---

## 📚 Ressources

- **Doctrine ORM**: https://www.doctrine-project.org/projects/orm.html
- **Doctrine Migrations**: https://symfony.com/doc/current/doctrine.html#migrations-creating-and-executing
- **EasyAdmin**: https://symfony.com/bundles/EasyAdminBundle/current/index.html
- **API Platform**: https://api-platform.com/
- **Symfony Validator**: https://symfony.com/doc/current/validation.html

---

*Document généré par Mistral Vibe - 15 juin 2026*
*Projet: FabOS - Spécifications des modèles de données Symfony/Doctrine*
*Version: 1.0 - À compléter au besoin*
