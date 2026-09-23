<?php

namespace App\Training;

use App\Entity\Formation;
use App\Entity\Progression;
use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Service\TrainingQualificationService;
use Doctrine\ORM\EntityManagerInterface;

/**
 * Écrire à tous les apprenants d'une formation — sans que personne ne voie les
 * autres (S183).
 *
 * 🔴 **L'invariant que la feuille de route demande — « une annonce n'expose
 * aucune adresse » — n'est pas une fonctionnalité à écrire ici : c'est une
 * propriété du `Mailer`.** `queueToUser()` prend UN utilisateur, écrit UNE ligne
 * de journal avec UNE adresse, dans la langue de cette personne. Il n'existe
 * aucun chemin qui accepte plusieurs destinataires — donc pas de `CC`, pas de
 * `BCC`, pas de liste à oublier de masquer. Ce fichier boucle, et c'est tout.
 *
 * ⚠️ **La conséquence pratique** : une annonce à trente personnes est trente
 * envois, pas un. C'est plus lent et c'est le prix de l'invariant. Grouper pour
 * aller vite serait exactement la façon dont ce genre de fuite arrive.
 *
 * ✅ **`NotificationCategory::NEWS` avait été créée SANS émetteur** — son
 * commentaire le dit : « Nothing emits this yet; the switch exists first ». S183
 * est son premier émetteur, donc la case de préférence que les membres voyaient
 * déjà se met enfin à servir. Et l'annonce est **non transactionnelle** : elle
 * respecte l'opt-out, contrairement à une confirmation de réservation.
 *
 * 🅿️ **Ce que ce fichier ne fait PAS, volontairement** : le fil de discussion
 * privé. Il demande une table de messages, donc une migration, donc l'opérateur —
 * et surtout un modèle de conversation qu'on ne pose pas à la va-vite. La
 * feuille de route le garde, avec l'invariant qui compte : aucun message privé
 * ne bascule implicitement vers la cohorte.
 */
final class CohortAnnouncer
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly Mailer $mailer,
        private readonly TrainingQualificationService $qualification,
    ) {
    }

    /**
     * Les apprenants de cette formation.
     *
     * 🔴 **La cohorte se DÉDUIT des progressions, elle ne se gère pas.** Une
     * table d'inscription serait une seconde vérité sur « qui suit ce cours » —
     * et le jour où quelqu'un commence sans y figurer, il ne reçoit rien.
     *
     * ⚠️ **Les formations INTERNES comptent pour leur parent.** Une progression
     * porte souvent sur `[FABOS SECTION] … · 2` ou sur un quiz ; quelqu'un qui
     * n'a fait que des sections est un apprenant du cours, et l'oublier
     * viderait la cohorte de ceux qui travaillent le plus.
     *
     * @return Utilisateur[] triés par nom, sans doublon
     */
    public function recipients(Formation $formation): array
    {
        $target = $this->qualification->resolveParentFormation($formation) ?? $formation;
        $targetId = $target->getId();

        $found = [];
        foreach ($this->em->getRepository(Progression::class)->findBy([]) as $progression) {
            $user = $progression->getUtilisateur();
            $onFormation = $progression->getFormation();
            if (!$user instanceof Utilisateur || !$onFormation instanceof Formation) {
                continue;
            }

            $parent = $this->qualification->resolveParentFormation($onFormation) ?? $onFormation;
            if ($parent->getId() !== $targetId) {
                continue;
            }

            $id = $user->getId();
            if ($id !== null) {
                $found[$id] = $user;
            }
        }

        $rows = array_values($found);
        usort($rows, static fn (Utilisateur $a, Utilisateur $b): int
            => [$a->getLastName(), $a->getFirstName()] <=> [$b->getLastName(), $b->getFirstName()]);

        return $rows;
    }

    /**
     * Cette personne est-elle dans la cohorte de cette formation ? (S183b)
     *
     * 🔴 **La MÊME règle que `recipients()`, pas une seconde.** Le fil privé doit
     * savoir qui a le droit d'écrire à l'équipe ; si cette question avait sa
     * propre définition, un apprenant pourrait recevoir les annonces d'une
     * formation sans pouvoir écrire à son équipe, ou l'inverse. Les formations
     * internes comptent pour leur parent ici aussi.
     * ⚠️ Restreinte aux progressions de la personne : `recipients()` parcourt
     * toutes celles du labo, ce qui est juste pour une annonce et absurde pour
     * une vérification faite à chaque affichage.
     */
    public function isMember(Formation $formation, Utilisateur $user): bool
    {
        $target = $this->qualification->resolveParentFormation($formation) ?? $formation;
        $targetId = $target->getId();

        foreach ($this->em->getRepository(Progression::class)->findBy(['utilisateur' => $user]) as $progression) {
            $onFormation = $progression->getFormation();
            if (!$onFormation instanceof Formation) {
                continue;
            }

            $parent = $this->qualification->resolveParentFormation($onFormation) ?? $onFormation;
            if ($parent->getId() === $targetId) {
                return true;
            }
        }

        return false;
    }

    /**
     * Envoie l'annonce, **un message par personne**.
     *
     * ⚠️ **Le retour distingue les DEUX raisons de ne pas recevoir** : avoir
     * coupé les annonces, ou un envoi refusé par le mailer. Les confondre
     * ferait dire « 12 sur 30 » sans dire si c'est un choix des membres ou une
     * panne — et l'auteur renverrait son annonce pour rien.
     *
     * @return array{sent: int, muted: int, total: int}
     */
    public function announce(Formation $formation, string $subject, string $body, Utilisateur $from): array
    {
        $recipients = $this->recipients($formation);
        $sent = 0;

        foreach ($recipients as $user) {
            // 🔴 `transactional: false` : une annonce N'EST PAS une confirmation.
            // La passer en transactionnelle la ferait ignorer l'opt-out — c'est
            // la façon dont un produit se met à écrire à des gens qui ont dit non.
            $ok = $this->mailer->queueToUser($user, 'formation_announcement', [
                'formation' => $formation->getTitre(),
                'subject' => $subject,
                'body' => $body,
                'author' => trim(($from->getFirstName() ?? '') . ' ' . ($from->getLastName() ?? '')),
            ], NotificationCategory::NEWS, false);

            if ($ok) {
                ++$sent;
            }
        }

        return [
            'sent' => $sent,
            'muted' => \count($recipients) - $sent,
            'total' => \count($recipients),
        ];
    }
}
