<?php

namespace App\Event;

use App\Entity\Event;
use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Mail\NotificationPreferences;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;

/**
 * Annoncer un événement aux membres — **une fois** (S163).
 *
 * 🔴 **La demande d'origine était « notifier à la création ». Ce n'est pas ce
 * qui est construit, et la raison est dans le modèle** : il n'existe AUCUN état
 * « brouillon » sur un événement. Les seuls champs de cycle de vie sont
 * `cancelledAt` et `archivedAt` — un événement enregistré est en ligne. Notifier
 * à l'enregistrement annoncerait à tout le labo les titres provisoires et les
 * erreurs de date, **et un e-mail parti ne se rattrape pas.** L'annonce est donc
 * un GESTE EXPLICITE, pas un effet de bord.
 *
 * 🔴 **Sous `NEWS`, jamais sous `EVENT`.** `EVENT` n'est pas désabonnable — et
 * c'est juste, il porte les confirmations d'inscription qu'on ne peut pas
 * refuser. Annoncer sous `EVENT` rendrait l'annonce non refusable ; rendre
 * `EVENT` refusable ferait perdre à un inscrit la confirmation de sa propre
 * inscription.
 *
 * ⚠️ **Un message par personne, jamais une liste.** `queueToUser()` prend UN
 * utilisateur et écrit UNE ligne de journal avec UNE adresse, dans SA langue :
 * il n'existe aucun chemin qui accepte plusieurs destinataires, donc pas de
 * `CC`, pas de `BCC`, pas de liste à oublier de masquer. Une annonce à trois
 * cents personnes est trois cents envois. C'est le prix de l'invariant, et
 * grouper pour aller vite est exactement la façon dont ce genre de fuite arrive.
 * C'est le même raisonnement que `CohortAnnouncer` (S183).
 */
final class EventAnnouncer
{
    public function __construct(
        private readonly EntityManagerInterface $em,
        private readonly Connection $db,
        private readonly Mailer $mailer,
        private readonly NotificationPreferences $preferences,
    ) {
    }

    /**
     * Un événement peut-il encore être annoncé ?
     *
     * ⚠️ **Quatre refus, et aucun n'est une question de droits.** Annoncer un
     * événement annulé, archivé, déjà commencé, ou déjà annoncé, envoie un
     * courrier que personne ne peut rattraper. L'écran s'en sert pour ne pas
     * MONTRER le bouton : une affordance qui existe et refuse est pire qu'une
     * affordance absente.
     */
    public function canAnnounce(Event $event, ?\DateTimeImmutable $now = null): bool
    {
        $now ??= new \DateTimeImmutable();
        $start = $event->getDateDebut();

        return $event->getAnnouncedAt() === null
            && $event->getCancelledAt() === null
            && !$event->isArchived()
            && ($start === null || $start > $now);
    }

    /**
     * Qui recevrait vraiment l'annonce.
     *
     * 🔴 **Le compte annoncé à l'écran DOIT être celui réellement mis en file** —
     * c'est la mesure de sortie de S164. Compter « tous les membres actifs »
     * puis n'écrire qu'aux abonnés donnerait « 120 personnes » avant et 87
     * lignes après, sans que rien n'explique l'écart : l'auteur croirait à une
     * panne d'envoi.
     * ⚠️ Les mêmes trois filtres que `queueToUser()` en mode non
     * transactionnel, dans le même ordre : compte actif, courrier accepté,
     * catégorie non refusée.
     *
     * 🅿️ **Les comptes anonymisés sont exclus sans clause spéciale** :
     * `AccountAnonymiser` les passe en `inactif`. Une seconde règle « exclure
     * les adresses sentinelles » serait une seconde vérité à tenir d'accord.
     *
     * @return Utilisateur[] triés par nom
     */
    public function recipients(): array
    {
        $rows = $this->em->getRepository(Utilisateur::class)
            ->findBy(['statut' => 'actif'], ['lastName' => 'ASC', 'firstName' => 'ASC']);

        return array_values(array_filter($rows, function (Utilisateur $user): bool {
            $id = $user->getId();

            return $user->isNotificationEmail()
                && $id !== null
                && $this->preferences->accepts($id, NotificationCategory::NEWS);
        }));
    }

    /**
     * Pose la marque, et **n'envoie rien**.
     *
     * 🔴 **Séparée d'`announce()` pour être MESURABLE.** Une sonde ne peut pas
     * appeler `announce()` : elle écrirait pour de vrai à tous les membres du
     * labo. La course, elle, se prouve ici — deux appels, un seul gagnant — sur
     * un événement jetable, sans qu'un courrier parte.
     *
     * ⚠️ **`UPDATE … WHERE announcedAt IS NULL` et pas « lire puis écrire ».**
     * C'est la base qui tranche : deux clics simultanés arrivent tous les deux,
     * un seul voit une ligne affectée.
     */
    public function claim(int $eventId, int $count, ?\DateTimeImmutable $now = null): bool
    {
        $now ??= new \DateTimeImmutable();

        return (int) $this->db->executeStatement(
            'UPDATE EVENEMENT SET announcedAt = :now, announcedCount = :count WHERE id = :id AND announcedAt IS NULL',
            ['now' => $now->format('Y-m-d H:i:s'), 'count' => $count, 'id' => $eventId],
        ) === 1;
    }

    /**
     * Écrit à tout le monde, **une seule fois**.
     *
     * 🔴 **La marque est POSÉE AVANT le premier envoi, par un `UPDATE`
     * conditionnel.** « Lire `announcedAt`, puis écrire » laisse entre les deux
     * une fenêtre où deux clics simultanés — ou un double POST d'un navigateur
     * impatient — passent tous les deux, et le labo reçoit l'annonce en double.
     * Ici c'est la base qui tranche : le second `UPDATE` touche zéro ligne et
     * l'appelant repart sans rien envoyer.
     *
     * ⚠️ **Et la marque reste posée même si les envois échouent ensuite.** C'est
     * délibéré : entre « quelques membres n'ont rien reçu » et « tout le labo a
     * reçu deux fois », le second est le défaut le plus difficile à réparer.
     *
     * @return array{claimed: bool, sent: int, total: int}
     */
    public function announce(Event $event, ?\DateTimeImmutable $now = null): array
    {
        $now ??= new \DateTimeImmutable();
        $recipients = $this->recipients();

        if (!$this->claim((int) $event->getId(), \count($recipients), $now)) {
            return ['claimed' => false, 'sent' => 0, 'total' => 0];
        }

        // ⚠️ L'entité en mémoire porte encore l'ancienne valeur : l'`UPDATE` est
        // passé par DBAL, sous l'ORM. Sans ça l'écran rendu juste après dirait
        // « jamais annoncé » d'un événement qui vient de l'être.
        $event->setAnnouncedAt($now)->setAnnouncedCount(\count($recipients));

        $sent = 0;
        foreach ($recipients as $user) {
            // 🔴 `transactional: false` : une annonce n'est PAS une confirmation.
            // La passer en transactionnelle lui ferait ignorer l'opt-out — c'est
            // la façon dont un produit se met à écrire à des gens qui ont dit non.
            $ok = $this->mailer->queueToUser($user, 'event_announced', [
                // Chaînes ISO : le contexte est stocké en JSON et rendu plus tard
                // par le worker, éventuellement dans une autre langue.
                'event' => $event->getTitre(),
                'start' => $event->getDateDebut()?->format(\DATE_ATOM),
                'end' => $event->getDateFin()?->format(\DATE_ATOM),
                'place' => $event->getLieu(),
            ], NotificationCategory::NEWS, false);

            if ($ok) {
                ++$sent;
            }
        }

        return ['claimed' => true, 'sent' => $sent, 'total' => \count($recipients)];
    }
}
