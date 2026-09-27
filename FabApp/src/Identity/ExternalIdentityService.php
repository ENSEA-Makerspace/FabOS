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
 *   - une adresse que le fournisseur ne garantit pas n'est pas reprise telle
 *     quelle : elle passe par la confirmation de S189 (plus d'adresse de
 *     remplacement depuis S197 — un compte sans adresse réelle ne pouvait ni
 *     recevoir un courrier, ni récupérer son accès) ;
 *   - 🔴 **le statut local gagne** : un compte désactivé ici reste refusé, même
 *     si le fournisseur l'accepte ;
 *   - aucun attribut ne donne un rôle ni un forfait (les affiliations sont LUES).
 *
 * `decide()` n'écrit rien : c'est ce que montre « Tester ». `apply()` rejoue la
 * même décision sous verrou et l'exécute.
 *
 * S197 — **plus aucun compte n'est ouvert avec un trou.** S'il manque une
 * adresse réelle (absente, non garantie, ou déjà prise ici) ou tout nom, la
 * décision est `COMPLETE` : la personne complète elle-même, et une adresse
 * qu'elle tape passe par la confirmation de S189. « Déjà prise » propose aussi
 * de LIER le compte existant — après s'y être connecté (preuve des deux côtés).
 */
final class ExternalIdentityService
{
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
        $needs = [];
        $taken = false;
        $email = null;
        if ($profile->email === null) {
            $needs[] = IdentityDecision::NEED_EMAIL;
            $notes[] = ['identity.note.no_email', []];
        } elseif (!$profile->emailVerified) {
            $needs[] = IdentityDecision::NEED_EMAIL;
            $notes[] = ['identity.note.email_unverified', ['%email%' => $profile->email]];
        } elseif ($this->db->fetchOne('SELECT 1 FROM UTILISATEUR WHERE email = ?', [$profile->email])) {
            $needs[] = IdentityDecision::NEED_EMAIL;
            $taken = true;
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
        if ($first === null && $last === null) {
            $needs[] = IdentityDecision::NEED_NAME;
            $notes[] = ['identity.note.no_name', []];
        }

        return $needs === []
            ? IdentityDecision::create($email, $first, $last, $notes)
            : IdentityDecision::complete($needs, $first, $last, $taken, $notes);
    }

    /** @throws IdentityRefusal */
    public function apply(ExternalProfile $profile): Utilisateur
    {
        return $this->db->transactional(function () use ($profile): Utilisateur {
            $decision = $this->decide($profile, forUpdate: true);
            if ($decision->outcome === IdentityDecision::REFUSE) {
                throw new IdentityRefusal((string) $decision->reason);
            }
            // ⚠️ Un profil à compléter ne s'ouvre JAMAIS ici : c'est la page
            // « Complétez votre compte » qui appelle `provision()`.
            if ($decision->outcome === IdentityDecision::COMPLETE) {
                throw new IdentityRefusal('identity.refused.incomplete');
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

            return $this->provision($profile, (string) $decision->email, $decision->firstName, $decision->lastName, verified: true);
        });
    }

    /**
     * Ouvrir le compte d'une identité externe et l'y lier (`provisioned` = 1 :
     * son mot de passe local est un aléa que personne ne connaît).
     *
     * @param bool $verified l'adresse est-elle prouvée (garantie par le fournisseur) ?
     *                       Sinon le compte attend le lien de confirmation de S189.
     */
    public function provision(ExternalProfile $profile, string $email, ?string $firstName, ?string $lastName, bool $verified): Utilisateur
    {
        return $this->db->transactional(function () use ($profile, $email, $firstName, $lastName, $verified): Utilisateur {
            if ($this->linkedUserId($profile, forUpdate: true) !== null) {
                throw new IdentityRefusal('identity.refused.already_linked');
            }
            if ($this->db->fetchOne('SELECT 1 FROM UTILISATEUR WHERE email = ?', [$email])) {
                throw new IdentityRefusal('identity.refused.email_taken');
            }
            $suffix = substr(hash('sha256', $profile->issuer . "\0" . $profile->subject), 0, 16);
            $user = (new Utilisateur())
                ->setEmail($email)
                ->setUsername(sprintf('%s-%s', $profile->providerKey, $suffix))
                ->setFirstName($firstName)
                ->setLastName($lastName)
                ->setIsVerified($verified);
            // Aucun mot de passe local utilisable : 256 bits jetés.
            $user->setPassword($this->hasher->hashPassword($user, bin2hex(random_bytes(32))));
            $this->em->persist($user);
            $this->em->flush();
            $this->insertLink($profile, (int) $user->getId(), provisioned: true);

            return $user;
        });
    }

    /**
     * S197 — lier un compte local EXISTANT, une fois que sa propriétaire s'y est
     * connectée (mot de passe, second facteur compris) : preuve des deux côtés.
     */
    public function linkExisting(ExternalProfile $profile, Utilisateur $user): void
    {
        $this->db->transactional(function () use ($profile, $user): void {
            $already = $this->linkedUserId($profile, forUpdate: true);
            if ($already !== null && $already !== $user->getId()) {
                throw new IdentityRefusal('identity.refused.already_linked');
            }
            if ($already === null) {
                $this->insertLink($profile, (int) $user->getId(), provisioned: false);
            }
        });
    }

    /**
     * Le fournisseur qui a CRÉÉ ce compte, s'il y en a un — « mot de passe
     * oublié » renvoie alors vers lui. Null avant la migration S197.
     */
    public function provisioningProvider(Utilisateur $user): ?string
    {
        try {
            $key = $this->db->fetchOne('SELECT providerKey FROM EXTERNAL_IDENTITY WHERE userId = ? AND provisioned = 1 AND revokedAt IS NULL ORDER BY id LIMIT 1', [$user->getId()]);
        } catch (\Throwable) {
            return null;
        }

        return $key === false ? null : (string) $key;
    }

    private function linkedUserId(ExternalProfile $profile, bool $forUpdate = false): ?int
    {
        $id = $this->db->fetchOne(
            'SELECT userId FROM EXTERNAL_IDENTITY WHERE issuer = :issuer AND subject = :subject AND revokedAt IS NULL' . ($forUpdate ? ' FOR UPDATE' : ''),
            ['issuer' => $profile->issuer, 'subject' => $profile->subject],
        );

        return $id === false ? null : (int) $id;
    }

    private function insertLink(ExternalProfile $profile, int $userId, bool $provisioned): void
    {
        $now = (new \DateTimeImmutable())->format('Y-m-d H:i:s');
        $row = [
            'userId' => $userId, 'issuer' => $profile->issuer, 'subject' => $profile->subject,
            'providerKey' => $profile->providerKey, 'lastClaimsAt' => $now, 'createdAt' => $now,
        ];
        try {
            $this->db->insert('EXTERNAL_IDENTITY', $row + ['provisioned' => $provisioned ? 1 : 0]);
        } catch (\Doctrine\DBAL\Exception\InvalidFieldNameException) {
            // Avant la migration S197 : on lie sans la marque.
            $this->db->insert('EXTERNAL_IDENTITY', $row);
        }
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
