<?php

declare(strict_types=1);

namespace App\Security;

use App\Entity\Utilisateur;
use App\Mail\Mailer;
use App\Mail\NotificationCategory;
use App\Repository\UtilisateurRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\HttpFoundation\Session\SessionInterface;
use Symfony\Component\Routing\Generator\UrlGeneratorInterface;

/**
 * S189 — l'entrée : un compte ne s'ouvre qu'une fois son adresse prouvée.
 *
 * 🔴 **Avant, n'importe quelle adresse, même inventée, ouvrait un compte actif**
 * (`setIsVerified(true)` à la création). Et `/register` répondait « un compte
 * existe déjà » : un oracle d'appartenance, là où « mot de passe oublié »
 * appliquait déjà l'invariant inverse.
 *
 * ✅ **Les deux se corrigent ENSEMBLE, et seulement ensemble.** Une fois que
 * l'accès passe par la boîte mail, l'inscription peut répondre la même chose
 * que l'adresse soit libre ou prise : dans les deux cas UN courrier part à
 * l'adresse tapée — un lien d'activation, ou « vous avez déjà un compte ». Seule
 * la propriétaire de la boîte voit la différence.
 *
 * ⚠️ **Sans courrier opérationnel, rien de tout ça ne tient** : un lien qui ne
 * part pas enferme dehors tout nouvel inscrit. `isRequired()` le dit, et
 * l'inscription retombe alors sur l'ancien comportement (compte ouvert tout de
 * suite) — une installation sans mail ne doit pas perdre son inscription.
 *
 * La session garde l'adresse tapée (pour l'écran « vérifiez votre boîte ») et,
 * seulement si un compte a VRAIMENT été créé, son id : c'est ce qui permet de
 * renvoyer le lien et de corriger l'adresse sans jamais dire lequel des deux cas
 * on est en train de vivre.
 */
final class AccountActivation
{
    public const SESSION_EMAIL = 'activation.email';
    public const SESSION_USER = 'activation.user';
    public const SESSION_SENT_AT = 'activation.sentAt';

    /** Un renvoi par minute : assez pour une boîte lente, pas un canon à courrier. */
    public const RESEND_COOLDOWN_SECONDS = 60;

    public function __construct(
        private readonly AccountVerificationTokenizer $tokens,
        private readonly Mailer $mailer,
        private readonly UtilisateurRepository $users,
        private readonly EntityManagerInterface $entityManager,
        private readonly UrlGeneratorInterface $urls,
    ) {
    }

    public function isRequired(): bool
    {
        return $this->mailer->isOperational();
    }

    /** Le compte vient d'être créé : lui envoyer son lien. */
    public function sendLink(Utilisateur $user, ?\DateTimeImmutable $now = null): void
    {
        $this->mailer->queueToUser($user, 'account_verify', [
            'activationUrl' => $this->urls->generate('app_register_activate', [
                'token' => $this->tokens->create($user, $now ?? new \DateTimeImmutable()),
            ], UrlGeneratorInterface::ABSOLUTE_URL),
            'validHours' => (int) (AccountVerificationTokenizer::TTL_SECONDS / 3600),
        ], NotificationCategory::GENERAL, true);
    }

    /**
     * Quelqu'un a tapé l'adresse d'un compte existant. On l'écrit À CE COMPTE —
     * c'est la seule personne qui a le droit de l'apprendre.
     */
    public function sendAlreadyRegistered(Utilisateur $owner): void
    {
        $this->mailer->queueToUser($owner, 'account_exists', [
            'loginUrl' => $this->urls->generate('app_login', [], UrlGeneratorInterface::ABSOLUTE_URL),
            'forgotUrl' => $this->urls->generate('app_forgot_password', [], UrlGeneratorInterface::ABSOLUTE_URL),
        ], NotificationCategory::GENERAL, true);
    }

    /** Ce que l'écran « vérifiez votre boîte » doit retenir, identique dans les deux cas. */
    public function remember(SessionInterface $session, string $email, ?Utilisateur $created): void
    {
        $session->set(self::SESSION_EMAIL, $email);
        $session->set(self::SESSION_USER, $created?->getId());
        $session->set(self::SESSION_SENT_AT, time());
    }

    /** Le compte en attente de CETTE session, s'il existe encore et n'est pas activé. */
    public function pendingOf(SessionInterface $session): ?Utilisateur
    {
        $id = $session->get(self::SESSION_USER);
        $user = is_int($id) ? $this->users->find($id) : null;

        return $user instanceof Utilisateur && !$user->isVerified() ? $user : null;
    }

    /** @return int secondes avant qu'un renvoi soit permis (0 = maintenant) */
    public function cooldownLeft(SessionInterface $session, ?int $now = null): int
    {
        $sentAt = (int) $session->get(self::SESSION_SENT_AT, 0);

        return max(0, $sentAt + self::RESEND_COOLDOWN_SECONDS - ($now ?? time()));
    }

    /** Renvoyer le lien. Ne dit jamais s'il y avait un compte à qui l'envoyer. */
    public function resend(SessionInterface $session): void
    {
        $pending = $this->pendingOf($session);
        if ($pending !== null) {
            $this->sendLink($pending);
        }
        $session->set(self::SESSION_SENT_AT, time());
    }

    /**
     * Corriger une adresse mal tapée, sans recréer de compte.
     *
     * ⚠️ Si la nouvelle adresse est déjà prise, le compte en attente GARDE
     * l'ancienne et la propriétaire de la nouvelle reçoit « vous avez déjà un
     * compte » — exactement ce que l'inscription aurait fait. L'écran, lui, dit la
     * même chose dans les deux cas.
     */
    public function correct(SessionInterface $session, string $newEmail): void
    {
        $pending = $this->pendingOf($session);
        $owner = $this->users->findOneBy(['email' => $newEmail]);

        if ($owner instanceof Utilisateur && $owner !== $pending) {
            $this->sendAlreadyRegistered($owner);
        } elseif ($pending !== null) {
            // Changer l'adresse change l'empreinte : le lien parti vers la
            // mauvaise adresse meurt à cet instant.
            $pending->setEmail($newEmail);
            $this->entityManager->flush();
            $this->sendLink($pending);
        }

        $session->set(self::SESSION_EMAIL, $newEmail);
        $session->set(self::SESSION_SENT_AT, time());
    }

    /**
     * @return Utilisateur|null le compte activé (ou déjà actif pour ce lien), null si
     *                          le lien est faux, expiré, ou périmé par une correction
     */
    public function activate(string $token, ?\DateTimeImmutable $now = null): ?Utilisateur
    {
        $id = $this->tokens->userIdIfValid($token, $now ?? new \DateTimeImmutable());
        $user = $id === null ? null : $this->users->find($id);
        if (!$user instanceof Utilisateur || !$this->tokens->matchesAccount($token, $user)) {
            return null;
        }

        $user->setIsVerified(true);
        $this->entityManager->flush();

        return $user;
    }
}
