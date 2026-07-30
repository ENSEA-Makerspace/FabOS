<?php

namespace App\Feature;

use App\Reservation\ReservableType;

/**
 * The catalogue an operator chooses from — and the single description of what
 * each feature is, which the route gate, the calendar, the booking chokepoint and
 * the admin screen all read.
 *
 * Names describe **what you can do**, never what kind of organisation you are:
 * calling one of these "fablab" would reintroduce the assumption S31 removes.
 *
 * Three groups, because three genuinely different kinds of thing had been hiding
 * behind one word:
 *
 *  - **resource** — a bookable kind. Owns a `ReservableType` and decides whether
 *    new bookings of that kind are accepted at all.
 *  - **activity** — a feature domain, with its own pages and data.
 *  - **directory** — display only: a page and a menu entry, nothing else. The
 *    people listed, their roles and their authorisation are **kernel** and are
 *    not switchable by anything here.
 */
final class SiteFeatureRegistry
{
    public const GROUP_RESOURCE = 'resource';
    public const GROUP_ACTIVITY = 'activity';
    public const GROUP_DIRECTORY = 'directory';

    /** @var array<string, SiteFeature>|null */
    private ?array $features = null;

    /** @return array<string, SiteFeature> */
    public function all(): array
    {
        return $this->features ??= $this->build();
    }

    public function get(string $key): ?SiteFeature
    {
        return $this->all()[$key] ?? null;
    }

    /** @return string[] */
    public function keys(): array
    {
        return array_keys($this->all());
    }

    public function has(string $key): bool
    {
        return isset($this->all()[$key]);
    }

    /** @return array<string, SiteFeature> */
    public function roots(): array
    {
        return array_filter($this->all(), static fn (SiteFeature $f): bool => !$f->isAddon());
    }

    /** @return array<string, SiteFeature> */
    public function addonsOf(string $key): array
    {
        return array_filter($this->all(), static fn (SiteFeature $f): bool => $f->parent === $key);
    }

    /**
     * Features that draw a column on the shared calendar grid.
     *
     * ⚠️ Narrower than the resource group, on purpose. `person_booking` is every
     * bit a bookable kind, but people are booked from their own pages and no
     * column is drawn for them — marking it a calendar layer would bring
     * `/calendrier` back as an empty grid for an appointments-only deployment.
     * Set the flag the day people appear on the grid, not before.
     *
     * @return string[]
     */
    public function calendarLayers(): array
    {
        return array_keys(array_filter($this->all(), static fn (SiteFeature $f): bool => $f->calendarLayer));
    }

    /** Which feature owns a bookable kind, or null if none does. */
    public function featureForReservable(ReservableType $type): ?SiteFeature
    {
        foreach ($this->all() as $feature) {
            if ($feature->reservable === $type) {
                return $feature;
            }
        }

        return null;
    }

    /** @return array<string, SiteFeature> */
    private function build(): array
    {
        $features = [
            new SiteFeature(
                'machines',
                'Réserver de l’équipement',
                'Machines, imprimantes 3D, découpeuses, microscopes, projecteurs… Les membres réservent des créneaux sur le calendrier partagé. C’est aussi ce qui porte « ce que cette machine accepte » dans les matériaux.',
                self::GROUP_RESOURCE,
                calendarLayer: true,
                reservable: ReservableType::Machine,
                recommends: [
                    // ⚠️ Checked against the source, and it is NOT what the plan
                    // assumed. `MachineQualificationService` has no feature check at
                    // all — it reads badge records directly — so switching badges off
                    // does **not** re-open equipment to everyone. What actually
                    // happens is harder to diagnose: the gate keeps refusing while the
                    // pages that would explain the refusal disappear.
                    ['feature' => 'badges', 'cost' => 'La certification continue de bloquer les réservations — elle lit les badges directement — mais les pages qui l’expliquent disparaissent. Un membre à qui l’on refuse un créneau n’a plus aucun moyen de voir ce qui lui manque.'],
                    ['feature' => 'materials', 'cost' => 'La fiche de chaque équipement perd sa section « matériaux acceptés ».'],
                ],
                landingRoute: 'app_machines',
            ),
            new SiteFeature(
                'maintenance',
                'Suivi de maintenance',
                'Un carnet d’entretien par équipement, avec les tâches en retard signalées et rappelées par e-mail.',
                self::GROUP_RESOURCE,
                parent: 'machines',
                landingRoute: 'app_maintenance',
            ),
            new SiteFeature(
                'places',
                'Réserver des espaces',
                'Salles, ateliers, postes de travail. Ils apparaissent comme une couche supplémentaire du même calendrier, à côté de l’équipement.',
                self::GROUP_RESOURCE,
                calendarLayer: true,
                reservable: ReservableType::Place,
                landingRoute: 'app_places',
            ),
            new SiteFeature(
                'person_booking',
                'Prendre rendez-vous avec quelqu’un',
                'Les personnes dont le temps est réservable publient leurs disponibilités et acceptent ou refusent les demandes. Indépendant des annuaires : chaque personne reste réservable ou non individuellement.',
                self::GROUP_RESOURCE,
                reservable: ReservableType::User,
                recommends: [
                    // The "réserver" button lives on the directory pages and nowhere
                    // else — verified by grepping every template for the route. With
                    // both directories off the booking pages still work, but nothing
                    // in the site links to them any more.
                    ['feature' => 'staff', 'cost' => 'Le bouton « réserver » d’une personne n’existe que sur les pages d’annuaire. Sans annuaire, les pages de rendez-vous fonctionnent toujours mais plus rien n’y mène : il faut connaître l’adresse par cœur.'],
                ],
            ),
            new SiteFeature(
                'events',
                'Organiser des événements',
                'Inscriptions (membres et invités), liste d’attente, billets avec QR code, pointage à l’entrée, rappels et affichage kiosque.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_events',
            ),
            new SiteFeature(
                'formations',
                'Former des personnes',
                'Un catalogue de formations et le suivi de progression de chacun.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_formations',
            ),
            new SiteFeature(
                'badges',
                'Certifications et badges',
                'Ce qu’une personne a le droit d’utiliser. À activer dès que la réservation d’un équipement doit dépendre d’une formation — y compris sans système de formation : les badges sont *décernés* par les formations, mais *exigés* indépendamment par l’équipement et par le contrôle d’accès physique.',
                self::GROUP_ACTIVITY,
                recommends: [
                    // The LMS is what *awards* badges; equipment cert-gating and the
                    // RFID door only *consume* them. Without it an admin issues every
                    // credential by hand from /admin/badges — which works, and is a
                    // legitimate way to run a small place.
                    ['feature' => 'formations', 'cost' => 'Plus rien ne décerne les badges automatiquement : chaque certification doit être attribuée à la main depuis l’administration.'],
                ],
                landingRoute: 'app_badges',
            ),
            new SiteFeature(
                'loans',
                'Prêter du matériel',
                'Un inventaire d’objets empruntables, avec dates de retour et relances.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_loans',
            ),
            new SiteFeature(
                'materials',
                'Matériaux et stock',
                'Le catalogue des matériaux et de leur stock, seul ou en complément de l’équipement.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_materials',
            ),
            new SiteFeature(
                'projects',
                'Galerie de projets',
                'Les membres publient ce qu’ils ont fabriqué ; les autres le notent.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_creations',
            ),
            new SiteFeature(
                'leaderboard',
                'Classement',
                'Un palmarès des membres par temps de présence et par nombre d’impressions, sur la semaine, le mois ou depuis toujours.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_leaderboard',
            ),
            new SiteFeature(
                'lab_pages',
                'Pages de contenu',
                'Des pages libres rédigées depuis l’administration, avec leur propre entrée de menu.',
                self::GROUP_ACTIVITY,
                landingRoute: 'app_lab_pages',
            ),
            new SiteFeature(
                'staff',
                'Annuaire de l’équipe',
                'Une page publique présentant l’équipe. Purement de l’affichage : ni les rôles, ni les autorisations, ni le comptoir staff n’en dépendent — ceux-là ne se désactivent pas.',
                self::GROUP_DIRECTORY,
                landingRoute: 'app_staff',
            ),
            new SiteFeature(
                'trainers',
                'Annuaire des formateurs',
                'La même page, pour les formateurs. Là encore, purement de l’affichage.',
                self::GROUP_DIRECTORY,
                landingRoute: 'app_trainers',
            ),
        ];

        $indexed = [];
        foreach ($features as $feature) {
            $indexed[$feature->key] = $feature;
        }

        return $indexed;
    }
}
