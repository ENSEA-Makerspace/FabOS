<?php

declare(strict_types=1);

namespace App\Identity;

use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;

/**
 * S196 — LE seul endroit qui décide quoi faire d'une identité externe : lier,
 * créer ou refuser. Aucun module de connexion n'a de règle d'identité à lui.
 *
 * Les invariants (`USAGE_RIGHTS_VISION.md`, « Identité externe ») :
 *   - l'identité, c'est `(émetteur, identifiant immuable)` — **jamais l'e-mail** ;
 *   - 🔴 **aucun rapprochement silencieux par e-mail** : une adresse déjà prise
 *     par un compte local donne un compte DISTINCT (S197 permettra de lier, avec
 *     preuve des deux côtés) ;
 *   - une adresse que le fournisseur ne garantit pas n'est pas reprise ;
 *   - 🔴 **le statut local gagne** : un compte désactivé ici reste refusé, même
 *     si le fournisseur l'accepte ;
 *   - aucun attribut ne donne un rôle ni un forfait (les affiliations sont LUES).
 *
 * `decide()` n'écrit rien : c'est ce que montre « Tester ». `apply()` rejoue la
 * même décision sous verrou et l'exécute.
 */
final class ExternalIdentityService
{
    /** RFC 2606 : `.invalid` ne résout jamais — une adresse de remplacement ne peut atteindre personne. */
    public const PLACEHOLDER_DOMAIN = 'external.invalid';

    public function __construct(
        private readonly Connection $db,
        private readonly EntityManagerInterface $em,
        private readonly UserPasswordHasherInterface $hasher,
    ) {
    }

    public function decide(ExternalProfile $profile, bool $forUpdate = false): IdentityDecision
    {
        if ($profile->disabledAtSource) {
            return IdentityDecision::refuse('identity.refused.disabled_at_source');
        }

        $linked = $this->db->fetchOne(
            'SELECT userId FROM EXTERNAL_IDENTITY WHERE issuer = :issuer AND subject = :subject AND revokedAt IS NULL' . ($forUpdate ? ' FOR UPDATE' : ''),
            ['issuer' => $profile->issuer, 'subject' => $profile->subject],
        );
        if ($linked !== false) {
            $user = $this->db->fetchAssociative('SELECT id, statut, email, firstName, lastName, username FROM UTILISATEUR WHERE id = ?', [(int) $linked]);
            if ($user === false) {
                return IdentityDecision::refuse('identity.refused.link_broken');
            }
            // ⚠️ Même règle que `ActiveAccountChecker` : on compare à `actif`, un
            // statut inconnu refuse.
            if ($user['statut'] !== 'actif') {
                return IdentityDecision::refuse('identity.refused.local_inactive');
            }

            return IdentityDecision::link((int) $user['id'], self::label($user));
        }

        $notes = [];
        $email = null;
        if ($profile->email === null) {
            $notes[] = ['identity.note.no_email', []];
        } elseif (!$profile->emailVerified) {
            $notes[] = ['identity.note.email_unverified', ['%email%' => $profile->email]];
        } elseif ($this->db->fetchOne('SELECT 1 FROM UTILISATEUR WHERE email = ?', [$profile->email])) {
            $notes[] = ['identity.note.email_taken', ['%email%' => $profile->email]];
        } else {
            $email = $profile->email;
            $notes[] = ['identity.note.email_used', ['%email%' => $profile->email]];
        }
        if ($profile->affiliations !== []) {
            $notes[] = ['identity.note.affiliations', ['%list%' => implode(', ', $profile->affiliations)]];
        }

        [$first, $last] = [$profile->firstName, $profile->lastName];
        if ($first === null && $last === null) {
            $first = $profile->displayName;
        }

        return IdentityDecision::create($email, $first, $last, $notes);
    }

    /** @throws IdentityRefusal */
    public function apply(ExternalProfile $profile): Utilisateur
    {
        return $this->db->transactional(function () use ($profile): Utilisateur {
            $decision = $this->decide($profile, forUpdate: true);
            if ($decision->outcome === IdentityDecision::REFUSE) {
                throw new IdentityRefusal((string) $decision->reason);
            }
            $now = (new \DateTimeImmutable())->format('Y-m-d H:i:s');

            if ($decision->outcome === IdentityDecision::LINK) {
                $this->db->executeStatement(
                    'UPDATE EXTERNAL_IDENTITY SET lastClaimsAt = ? WHERE issuer = ? AND subject = ? AND revokedAt IS NULL',
                    [$now, $profile->issuer, $profile->subject],
                );
                $user = $this->em->find(Utilisateur::class, (int) $decision->userId);
                if (!$user instanceof Utilisateur) {
                    throw new IdentityRefusal('identity.refused.link_broken');
                }

                return $user;
            }

            $suffix = substr(hash('sha256', $profile->issuer . "\0" . $profile->subject), 0, 16);
            $user = (new Utilisateur())
                ->setEmail($decision->email ?? sprintf('%s-%s@%s', $profile->providerKey, $suffix, self::PLACEHOLDER_DOMAIN))
                ->setUsername(sprintf('%s-%s', $profile->providerKey, $suffix))
                ->setFirstName($decision->firstName)
                ->setLastName($decision->lastName)
                // L'adresse est soit garantie par le fournisseur, soit une
                // adresse de remplacement qui ne reçoit rien : rien à confirmer.
                ->setIsVerified(true);
            // Aucun mot de passe local utilisable : 256 bits jetés.
            $user->setPassword($this->hasher->hashPassword($user, bin2hex(random_bytes(32))));
            $this->em->persist($user);
            $this->em->flush();
            $this->db->insert('EXTERNAL_IDENTITY', [
                'userId' => $user->getId(), 'issuer' => $profile->issuer, 'subject' => $profile->subject,
                'providerKey' => $profile->providerKey, 'lastClaimsAt' => $now, 'createdAt' => $now,
            ]);

            return $user;
        });
    }

    public function revokeAll(int $userId): void
    {
        $this->db->executeStatement('UPDATE EXTERNAL_IDENTITY SET revokedAt = NOW() WHERE userId = :user AND revokedAt IS NULL', ['user' => $userId]);
    }

    /** @param array<string, mixed> $user */
    private static function label(array $user): string
    {
        $name = trim(((string) ($user['firstName'] ?? '')) . ' ' . ((string) ($user['lastName'] ?? '')));

        return sprintf('#%d %s', (int) $user['id'], $name !== '' ? $name : (string) $user['username']);
    }
}
