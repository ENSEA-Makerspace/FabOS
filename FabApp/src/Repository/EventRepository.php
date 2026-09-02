<?php

namespace App\Repository;

use App\Entity\Event;
use App\Entity\Formation;
use App\Reservation\LabClock;
use Doctrine\Bundle\DoctrineBundle\Repository\ServiceEntityRepository;
use Doctrine\Persistence\ManagerRegistry;

/**
 * @extends ServiceEntityRepository<Event>
 */
class EventRepository extends ServiceEntityRepository
{
    public function __construct(ManagerRegistry $registry, private readonly LabClock $labClock)
    {
        parent::__construct($registry, Event::class);
    }

    /**
     * "Now", in the digits these columns actually hold.
     *
     * ⚠️ `Event` is convention B: `dateDebut`/`dateFin` are human-entered wall-clock
     * values stored as typed, never as UTC instants. `new DateTimeImmutable('now')`
     * is the *server* instant, so comparing the two in SQL was wrong by the lab's
     * UTC offset — two hours for Paris in summer. Around midnight an event moved
     * between "À venir" and "Passé" early or late, on the homepage block, the public
     * events page and the admin tiles alike, because all three came through here.
     */
    private function nowInStoredForm(): \DateTimeImmutable
    {
        return $this->labClock->storedFormOf($this->labClock->now());
    }

    /**
     * The same "now" this repository filters with, for callers that have to
     * classify a row they already hold.
     *
     * ⚠️ Exposed in S139d because the events catalogue was classifying its own
     * cards with `new \DateTimeImmutable()` while its rows had been selected
     * with the wall clock — so a card could sit in the "past" list wearing an
     * "open" chip. A caller that needs the same verdict must be able to ask the
     * same clock; the alternative is every caller inventing its own midnight.
     */
    public function storedNow(): \DateTimeImmutable
    {
        return $this->nowInStoredForm();
    }

    /**
     * Upcoming events (still running or starting in the future), soonest first.
     * An event counts as "upcoming" until its end (or its start, if it has no
     * end) has passed.
     *
     * @return Event[]
     */
    public function findUpcoming(?int $limit = null): array
    {
        $now = $this->nowInStoredForm();
        $qb = $this->createQueryBuilder('e')
            ->andWhere('e.archivedAt IS NULL')
            ->andWhere('COALESCE(e.dateFin, e.dateDebut) >= :now')
            ->setParameter('now', $now)
            ->orderBy('e.dateDebut', 'ASC');

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }

    /**
     * The next sessions of one training (S146d).
     *
     * 🔴 **This is the block S134c2 had to DELETE.** The training page used to show
     * "three next sessions" that FabOS invented, because `Formation` carries
     * `duree` and `formateur` as free text and no date at all — there was nothing
     * real to show. `Event.formation` is the real thing, so the block can come back
     * and be true.
     *
     * ⚠️ Cancelled sessions are kept, not hidden. Somebody who planned around one
     * needs to see that it is off, with the reason the event carries; silently
     * dropping it looks like it never existed. The same rule the catalogue applies.
     *
     * ⚠️ Same definition of "upcoming" as `findUpcoming()` — until its END has
     * passed — so a session in progress does not vanish from the page mid-session.
     *
     * @return Event[]
     */
    public function findUpcomingSessionsFor(Formation $formation, ?int $limit = null): array
    {
        $qb = $this->createQueryBuilder('e')
            ->andWhere('e.archivedAt IS NULL')
            ->andWhere('e.formation = :formation')
            ->andWhere('COALESCE(e.dateFin, e.dateDebut) >= :now')
            ->setParameter('formation', $formation)
            ->setParameter('now', $this->nowInStoredForm())
            ->orderBy('e.dateDebut', 'ASC');

        if ($limit !== null) {
            $qb->setMaxResults($limit);
        }

        return $qb->getQuery()->getResult();
    }

    /** The catalogue's two halves, and the definition of "past" is the same one. */
    public const WHEN_UPCOMING = 'upcoming';
    public const WHEN_PAST = 'past';

    /**
     * The public catalogue: one filter (when) and one search box.
     *
     * ⚠️ Upcoming is soonest-first and past is most-recent-first. Both are
     * "nearest to now" — a list of past events that started with the oldest one
     * would open on whatever happened first in the lab's history.
     *
     * @return Event[]
     */
    public function findForCatalogue(?string $when, string $search = ''): array
    {
        // ⚠️ S147, J-2 — les archivés sortent des surfaces qui PROPOSENT.
        $qb = $this->createQueryBuilder('e')->andWhere('e.archivedAt IS NULL');
        $this->applyWhen($qb, $when);

        $search = trim($search);
        if ($search !== '') {
            $qb->andWhere('e.titre LIKE :q OR e.description LIKE :q OR e.lieu LIKE :q')
                ->setParameter('q', '%' . $search . '%');
        }

        // 🔴 **S159c — les plus RÉCENTS en premier, quel que soit le filtre**
        // (décision de l'opérateur, 2026-09-02). L'ordre était `ASC` sauf pour
        // « passés » ; il est désormais `DESC` partout, donc la liste s'ouvre sur
        // ce qui vient d'être publié ou programmé.
        // ⚠️ **Ce que ça change, et il faut le savoir** : sur « à venir », un
        // événement dans six mois passe AVANT celui de demain. C'est le sens de
        // « plus récent d'abord » et c'est ce qui a été demandé ; le regroupement
        // par mois rend l'ordre lisible, mais il ne le corrige pas.
        return $qb->orderBy('e.dateDebut', 'DESC')
            ->getQuery()
            ->getResult();
    }

    /** Tile counts, over the UNFILTERED set — see the note in _catalogue.html.twig. */
    public function countWhen(?string $when): int
    {
        // ⚠️ Le compte doit exclure les mêmes lignes que la liste, sinon la pastille
        // annonce des événements que la page ne montre pas (S147, J-2).
        $qb = $this->createQueryBuilder('e')->select('COUNT(e.id)')->andWhere('e.archivedAt IS NULL');
        $this->applyWhen($qb, $when);

        return (int) $qb->getQuery()->getSingleScalarResult();
    }

    private function applyWhen(\Doctrine\ORM\QueryBuilder $qb, ?string $when): void
    {
        if ($when !== self::WHEN_UPCOMING && $when !== self::WHEN_PAST) {
            return;
        }

        // Same rule as findUpcoming(): an event is still upcoming until its end
        // (or its start, when it has no end) has passed. Stated once, here, so
        // the tile count and the list it labels cannot disagree.
        $qb->andWhere(sprintf('COALESCE(e.dateFin, e.dateDebut) %s :now', $when === self::WHEN_PAST ? '<' : '>='))
            ->setParameter('now', $this->nowInStoredForm());
    }
}
