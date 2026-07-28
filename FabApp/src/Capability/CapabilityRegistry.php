<?php

namespace App\Capability;

use App\Service\ModuleService;

/**
 * The catalogue an admin actually chooses from.
 *
 * Names describe **what you can do**, never what kind of organisation you are —
 * calling one of these "fablab" would reintroduce the assumption the whole phase
 * exists to remove.
 *
 * Two decisions worth knowing before editing this list:
 *
 * **One switch per module concept.** The plan offered "Materials & stock" both as
 * an add-on of *Book equipment* and as a capability of its own, resolving to the
 * same module either way. Union semantics make that work, but the screen would
 * then show two switches that mirror each other and an operator would reasonably
 * wonder which one is real. It is one capability, and *Book equipment* says in
 * words that it is also what feeds "what this machine accepts".
 *
 * **Add-ons are capabilities with a parent.** Nothing else distinguishes them:
 * they are stored, derived and applied identically. That keeps progressive
 * disclosure a presentation rule rather than a second concept in the model.
 */
final class CapabilityRegistry
{
    public const GROUP_RESOURCE = 'resource';
    public const GROUP_ACTIVITY = 'activity';

    /** @var array<string, Capability>|null */
    private ?array $capabilities = null;

    /** @return array<string, Capability> */
    public function all(): array
    {
        return $this->capabilities ??= $this->build();
    }

    public function get(string $key): ?Capability
    {
        return $this->all()[$key] ?? null;
    }

    /** Top-level capabilities, in catalogue order. @return array<string, Capability> */
    public function roots(): array
    {
        return array_filter($this->all(), static fn (Capability $c): bool => !$c->isAddon());
    }

    /** The add-ons of one capability, in catalogue order. @return array<string, Capability> */
    public function addonsOf(string $key): array
    {
        return array_filter($this->all(), static fn (Capability $c): bool => $c->parent === $key);
    }

    /**
     * Modules no capability claims. They would be unreachable from the capability
     * screen — on forever, or off forever, with no way for the admin to say
     * otherwise. Surfaced in the Advanced panel rather than thrown, because a
     * half-wired new module should not take the admin screen down.
     *
     * @return string[]
     */
    public function unclaimedModules(): array
    {
        $claimed = [];
        foreach ($this->all() as $capability) {
            foreach ($capability->requires as $module) {
                $claimed[$module] = true;
            }
        }

        return array_values(array_filter(
            ModuleService::MODULES,
            static fn (string $module): bool => !isset($claimed[$module]),
        ));
    }

    /** @return array<string, Capability> */
    private function build(): array
    {
        $capabilities = [
            new Capability(
                'book_equipment',
                'Réserver de l’équipement',
                'Machines, imprimantes 3D, découpeuses, microscopes, projecteurs… Les membres réservent des créneaux sur le calendrier partagé. C’est aussi ce qui alimente « ce que cette machine accepte » dans les matériaux.',
                ['machines'],
                self::GROUP_RESOURCE,
            ),
            new Capability(
                'equipment_maintenance',
                'Suivi de maintenance',
                'Un carnet d’entretien par équipement, avec les tâches en retard signalées et rappelées par e-mail.',
                ['maintenance'],
                self::GROUP_RESOURCE,
                parent: 'book_equipment',
                defaultEnabled: false,
            ),
            new Capability(
                'book_spaces',
                'Réserver des espaces',
                'Salles, ateliers, postes de travail. Ils apparaissent comme une couche supplémentaire du même calendrier, à côté de l’équipement.',
                ['places'],
                self::GROUP_RESOURCE,
            ),
            new Capability(
                'book_people',
                'Prendre rendez-vous avec quelqu’un',
                'Les personnes dont le temps est réservable publient leurs disponibilités et acceptent ou refusent les demandes. Indépendant des annuaires : chaque personne reste réservable ou non individuellement.',
                ['person_booking'],
                self::GROUP_RESOURCE,
            ),
            new Capability(
                'run_events',
                'Organiser des événements',
                'Inscriptions (membres et invités), liste d’attente, billets avec QR code, pointage à l’entrée, rappels et affichage kiosque.',
                ['events'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'train_people',
                'Former des personnes',
                'Un catalogue de formations et le suivi de progression de chacun.',
                ['formations'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'credentials',
                'Certifications et badges',
                'Ce qu’une personne a le droit d’utiliser. À activer dès que la réservation d’un équipement doit dépendre d’une formation — y compris sans système de formation : les badges sont *décernés* par les formations, mais *exigés* indépendamment par l’équipement et par le contrôle d’accès physique.',
                ['badges'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'lend_equipment',
                'Prêter du matériel',
                'Un inventaire d’objets empruntables, avec dates de retour et relances.',
                ['loans'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'materials_catalogue',
                'Matériaux et stock',
                'Le catalogue des matériaux et de leur stock, seul ou en complément de l’équipement.',
                ['materials'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'project_gallery',
                'Galerie de projets',
                'Les membres publient ce qu’ils ont fabriqué ; les autres le notent.',
                ['projects'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'leaderboard',
                'Classement',
                'Un palmarès des membres par temps de présence et par nombre d’impressions, sur la semaine, le mois ou depuis toujours.',
                ['leaderboard'],
                self::GROUP_ACTIVITY,
            ),
            new Capability(
                'content_pages',
                'Pages de contenu',
                'Des pages libres rédigées depuis l’administration, avec leur propre entrée de menu.',
                ['lab_pages'],
                self::GROUP_ACTIVITY,
            ),
            // One capability, two modules: "publish who we are" is a single decision
            // for almost everybody. Publishing only one of the two lists is a real
            // but rare wish, and it is exactly what the Advanced panel's override is
            // for — untick `trainers` there and it stays unticked.
            new Capability(
                'team_directories',
                'Annuaires de l’équipe',
                'Des pages publiques présentant l’équipe et les formateurs. Purement de l’affichage : ni les rôles, ni les autorisations, ni le comptoir staff n’en dépendent — ceux-là ne se désactivent pas.',
                ['staff', 'trainers'],
                self::GROUP_ACTIVITY,
            ),
        ];

        $indexed = [];
        foreach ($capabilities as $capability) {
            $indexed[$capability->key] = $capability;
        }

        return $indexed;
    }
}
