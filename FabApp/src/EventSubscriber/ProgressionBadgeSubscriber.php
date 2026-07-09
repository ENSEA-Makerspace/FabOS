<?php

namespace App\EventSubscriber;

use App\Entity\Badge;
use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurBadge;
use App\Service\TrainingQualificationService;
use Doctrine\Bundle\DoctrineBundle\Attribute\AsDoctrineListener;
use Doctrine\ORM\EntityManagerInterface;
use Doctrine\ORM\Event\PostFlushEventArgs;
use Doctrine\ORM\Event\PostPersistEventArgs;
use Doctrine\ORM\Event\PostUpdateEventArgs;
use Doctrine\ORM\Events;

#[AsDoctrineListener(event: Events::postPersist)]
#[AsDoctrineListener(event: Events::postUpdate)]
#[AsDoctrineListener(event: Events::postFlush)]
class ProgressionBadgeSubscriber
{
    /** @var array<string, array{user: Utilisateur, formation: Formation}> */
    private array $pendingChecks = [];
    private bool $flushingAwards = false;

    public function __construct(
        private readonly TrainingQualificationService $qualification,
    ) {
    }

    public function postPersist(PostPersistEventArgs $args): void
    {
        $this->queueCheck($args->getObject());
    }

    public function postUpdate(PostUpdateEventArgs $args): void
    {
        $this->queueCheck($args->getObject());
    }

    public function postFlush(PostFlushEventArgs $args): void
    {
        if ($this->pendingChecks === [] || $this->flushingAwards) {
            return;
        }

        $this->flushingAwards = true;
        $pending = $this->pendingChecks;
        $this->pendingChecks = [];

        $em = $args->getObjectManager();
        $created = false;

        foreach ($pending as $check) {
            $status = $this->qualification->getStatus($check['formation'], $check['user']);
            $badge = $status['badge'];

            if (!$status['eligible'] || !$badge instanceof Badge) {
                continue;
            }

            $exists = $em->getRepository(UtilisateurBadge::class)->findOneBy([
                'utilisateur' => $check['user'],
                'badge' => $badge,
            ]);

            if ($exists !== null) {
                continue;
            }

            $em->persist((new UtilisateurBadge())
                ->setUtilisateur($check['user'])
                ->setBadge($badge));
            $created = true;
        }

        if ($created && $em instanceof EntityManagerInterface) {
            $em->flush();
        }

        $this->flushingAwards = false;
    }

    private function queueCheck(object $entity): void
    {
        if (!$entity instanceof Progression) {
            return;
        }

        $user = $entity->getUtilisateur();
        $formation = $entity->getFormation();
        if (!$user instanceof Utilisateur || !$formation instanceof Formation || $user->getId() === null) {
            return;
        }

        if ($formation->getId() === null) {
            return;
        }

        $this->pendingChecks[$user->getId() . ':' . $formation->getId()] = [
            'user' => $user,
            'formation' => $formation,
        ];
    }
}
