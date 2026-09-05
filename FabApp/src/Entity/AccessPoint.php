<?php

namespace App\Entity;

use App\Repository\AccessPointRepository;
use Doctrine\ORM\Mapping as ORM;

/**
 * Un point d'accès physique : une porte, un portail, un casier, une zone.
 *
 * 🔴 **Pourquoi ce n'est PAS une `Machine`.** Un lecteur RFID était rattaché
 * obligatoirement à une machine, donc représenter la porte d'entrée imposait
 * d'inventer une machine fictive. Elle serait apparue dans le catalogue des
 * machines, dans les réservations, dans les statistiques d'utilisation et sur
 * les kiosques — et chaque écran aurait eu à connaître l'exception. Une porte
 * n'est pas une machine ; elle n'a ni créneau, ni matériaux, ni maintenance
 * préventive, ni badge de formation.
 *
 * ⚠️ **Le lieu est obligatoire, l'espace ne l'est pas.** Une porte appartient
 * toujours à un site ; elle n'ouvre pas forcément UNE salle (le portail
 * d'entrée n'en ouvre aucune en particulier, un badge de zone en ouvre
 * plusieurs). `place` renseigne « qu'est-ce que ça ouvre » quand la réponse
 * existe, et vaut `NULL` quand elle n'existe pas — pas quand on ne l'a pas
 * saisie.
 *
 * 🅿️ **Ce que cette classe ne porte PAS, volontairement** : aucune règle
 * d'autorisation. Le verdict d'une porte se décide avec les axes lieu / jours /
 * horaires qu'un forfait décrit DÉJÀ (`PackageSpec`). Écrire une seconde
 * mécanique de droits ici serait la deuxième vérité que cette phase existe pour
 * éviter.
 */
#[ORM\Entity(repositoryClass: AccessPointRepository::class)]
#[ORM\Table(name: 'ACCESS_POINT')]
class AccessPoint
{
    /**
     * ⚠️ Les quatre natures sont une LISTE FERMÉE, et le formulaire y puise.
     * Une chaîne libre aurait produit « porte », « Porte », « door » et « entrée »
     * dans la même colonne au bout d'un mois, et plus aucun regroupement possible.
     */
    public const KINDS = ['door', 'gate', 'locker', 'zone'];

    #[ORM\Id]
    #[ORM\GeneratedValue]
    #[ORM\Column]
    private ?int $id = null;

    #[ORM\Column(length: 150)]
    private string $nom = '';

    #[ORM\Column(length: 30, options: ['default' => 'door'])]
    private string $kind = 'door';

    #[ORM\Column(type: 'text', nullable: true)]
    private ?string $description = null;

    #[ORM\Column(length: 150, nullable: true)]
    private ?string $localisation = null;

    #[ORM\ManyToOne(targetEntity: Venue::class)]
    #[ORM\JoinColumn(name: 'venueId', onDelete: 'RESTRICT')]
    private ?Venue $venue = null;

    #[ORM\ManyToOne(targetEntity: Place::class)]
    #[ORM\JoinColumn(name: 'placeId', onDelete: 'SET NULL')]
    private ?Place $place = null;

    #[ORM\Column(name: 'createdAt', type: 'datetime_immutable', options: ['default' => 'CURRENT_TIMESTAMP'])]
    private \DateTimeImmutable $createdAt;

    /**
     * ⚠️ **Archivé, pas supprimé** — la règle de la maison depuis S147/J-2. Les
     * journaux d'accès nomment le lecteur, qui nomme la porte : supprimer la
     * porte laisserait des lignes dont plus personne ne sait de quelle entrée
     * elles parlent.
     */
    #[ORM\Column(name: 'archivedAt', type: 'datetime_immutable', nullable: true)]
    private ?\DateTimeImmutable $archivedAt = null;

    public function __construct()
    {
        $this->createdAt = new \DateTimeImmutable();
    }

    public function getId(): ?int { return $this->id; }
    public function getNom(): string { return $this->nom; }
    public function setNom(string $nom): self { $this->nom = $nom; return $this; }
    public function getKind(): string { return $this->kind; }
    public function setKind(string $kind): self { $this->kind = \in_array($kind, self::KINDS, true) ? $kind : 'door'; return $this; }
    public function getDescription(): ?string { return $this->description; }
    public function setDescription(?string $description): self { $this->description = $description; return $this; }
    public function getLocalisation(): ?string { return $this->localisation; }
    public function setLocalisation(?string $localisation): self { $this->localisation = $localisation; return $this; }
    public function getVenue(): ?Venue { return $this->venue; }
    public function setVenue(?Venue $venue): self { $this->venue = $venue; return $this; }
    public function getPlace(): ?Place { return $this->place; }
    public function setPlace(?Place $place): self { $this->place = $place; return $this; }
    public function getCreatedAt(): \DateTimeImmutable { return $this->createdAt; }

    public function getArchivedAt(): ?\DateTimeImmutable { return $this->archivedAt; }
    public function isArchived(): bool { return $this->archivedAt !== null; }
    public function archive(): self { $this->archivedAt ??= new \DateTimeImmutable(); return $this; }
    public function restore(): self { $this->archivedAt = null; return $this; }

    /** La clé de traduction de la nature — le libellé vit dans les catalogues, pas ici. */
    public function getKindKey(): string { return 'access_points.kind_' . $this->kind; }
}
