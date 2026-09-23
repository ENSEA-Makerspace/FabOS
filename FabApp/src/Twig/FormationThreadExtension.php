<?php

namespace App\Twig;

use App\Entity\Formation;
use App\Entity\Utilisateur;
use App\Training\FormationThreads;
use Symfony\Bundle\SecurityBundle\Security;
use Twig\Extension\AbstractExtension;
use Twig\TwigFunction;

/**
 * Ce que l'onglet « Messages » d'une formation doit afficher (S183b).
 *
 * ⚠️ **Une fonction Twig plutôt qu'un paramètre de plus dans chaque contrôleur.**
 * L'onglet apparaît sur deux pages — le suivi et le fil lui-même — et l'action de
 * suivi reçoit déjà onze services. Un paramètre qu'une des deux oublierait de
 * passer vaudrait `null` en silence (pas de `strict_variables` en production) et
 * l'onglet disparaîtrait sans que rien ne le signale.
 */
final class FormationThreadExtension extends AbstractExtension
{
    public function __construct(
        private readonly FormationThreads $threads,
        private readonly Security $security,
    ) {
    }

    public function getFunctions(): array
    {
        return [
            new TwigFunction('formation_thread_state', $this->state(...)),
            new TwigFunction('trainer_inbox_unread', $this->inboxUnread(...)),
        ];
    }

    /** @return array{canWrite: bool, unread: int} */
    public function state(Formation $formation): array
    {
        $user = $this->security->getUser();
        if (!$user instanceof Utilisateur || !$this->threads->canWrite($formation, $user)) {
            return ['canWrite' => false, 'unread' => 0];
        }

        $thread = $this->threads->threadFor($formation, $user, false);

        return [
            'canWrite' => true,
            'unread' => $thread === null ? 0 : $this->threads->unread((int) $thread['id'], (int) $user->getId()),
        ];
    }

    /** ⚠️ `null` pour qui n'est pas formateur : le lien ne doit même pas exister. */
    public function inboxUnread(): ?int
    {
        $user = $this->security->getUser();
        if (!$user instanceof Utilisateur || !$this->threads->isTrainer($user) || !$this->threads->isAvailable()) {
            return null;
        }

        return $this->threads->inboxUnread($user);
    }
}
