<?php

namespace App\EventSubscriber;

use App\Entity\Badge;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Entity\UtilisateurBadge;
use Doctrine\Bundle\DoctrineBundle\Attribute\AsDoctrineListener;
use Doctrine\ORM\EntityManagerInterface;
use Doctrine\ORM\Event\PostPersistEventArgs;
use Doctrine\ORM\Event\PostUpdateEventArgs;
use Doctrine\ORM\Event\PostFlushEventArgs;
use Doctrine\ORM\Events;

#[AsDoctrineListener(event: Events::postPersist)]
#[AsDoctrineListener(event: Events::postUpdate)]
#[AsDoctrineListener(event: Events::postFlush)]
class ProgressionBadgeSubscriber
{
    /** @var array<string, array{user: Utilisateur, badge: Badge}> */
    private array $pendingAwards = [];
    private bool $flushingAwards = false;


    public function postPersist(PostPersistEventArgs $args): void
    {
        $this->queueAward($args->getObject());
    }

    public function postUpdate(PostUpdateEventArgs $args): void
    {
        $this->queueAward($args->getObject());
    }

    public function postFlush(PostFlushEventArgs $args): void
    {
        if ($this->pendingAwards === [] || $this->flushingAwards) {
            return;
        }

        $this->flushingAwards = true;
        $em = $args->getObjectManager();
        $created = false;

        foreach ($this->pendingAwards as $award) {
            $exists = $em->getRepository(UtilisateurBadge::class)->findOneBy([
                'utilisateur' => $award['user'],
                'badge' => $award['badge'],
            ]);

            if ($exists !== null) {
                continue;
            }

            $em->persist((new UtilisateurBadge())
                ->setUtilisateur($award['user'])
                ->setBadge($award['badge']));
            $created = true;
        }

        $this->pendingAwards = [];
        if ($created && $em instanceof EntityManagerInterface) {
            $em->flush();
        }
        $this->flushingAwards = false;
    }

    private function queueAward(object $entity): void
    {
        if (!$entity instanceof Progression || !$entity->isCompleted()) {
            return;
        }

        $user = $entity->getUtilisateur();
        $badge = $entity->getFormation()?->getBadge();
        if (!$user instanceof Utilisateur || !$badge instanceof Badge || $user->getId() === null || $badge->getId() === null) {
            return;
        }

        $this->pendingAwards[$user->getId() . ':' . $badge->getId()] = ['user' => $user, 'badge' => $badge];
    }
}
