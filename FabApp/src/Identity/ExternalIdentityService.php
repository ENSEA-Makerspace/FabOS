<?php
namespace App\Identity;
use App\Entity\Utilisateur;
use Doctrine\DBAL\Connection;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Component\PasswordHasher\Hasher\UserPasswordHasherInterface;

/** Links only issuer+subject. Email is initial contact data, never an identity key or role source. */
final class ExternalIdentityService
{
    public function __construct(private readonly Connection $db, private readonly EntityManagerInterface $em, private readonly UserPasswordHasherInterface $hasher) {}
    public function resolveOrProvision(OidcProvider $provider, string $subject, ?string $email, ?string $displayName = null): Utilisateur
    {
        $subject = trim($subject); if ($subject === '') { throw new \InvalidArgumentException('OIDC subject manquant.'); }
        return $this->db->transactional(function () use ($provider,$subject,$email,$displayName): Utilisateur {
            $id = $this->db->fetchOne('SELECT userId FROM EXTERNAL_IDENTITY WHERE issuer=:issuer AND subject=:subject AND revokedAt IS NULL FOR UPDATE', ['issuer'=>$provider->issuer,'subject'=>$subject]);
            if ($id !== false) { $user=$this->em->find(Utilisateur::class,(int)$id); if (!$user instanceof Utilisateur || $user->getStatut() !== 'actif') { throw new \RuntimeException('Compte local suspendu.'); } return $user; }
            $suffix = substr(hash('sha256',$provider->issuer."\0".$subject),0,16);
            $localEmail = filter_var($email,FILTER_VALIDATE_EMAIL) ? strtolower((string)$email) : "oidc-$suffix@invalid.local";
            if ($this->db->fetchOne('SELECT 1 FROM UTILISATEUR WHERE email=:email', ['email'=>$localEmail])) { $localEmail="oidc-$suffix@invalid.local"; }
            $user=(new Utilisateur())->setEmail($localEmail)->setUsername('oidc-'.$suffix)->setFirstName($displayName ?: null)->setIsVerified(true);
            $user->setPassword($this->hasher->hashPassword($user,bin2hex(random_bytes(32)))); $this->em->persist($user); $this->em->flush();
            $this->db->insert('EXTERNAL_IDENTITY',['userId'=>$user->getId(),'issuer'=>$provider->issuer,'subject'=>$subject,'providerKey'=>$provider->key,'lastClaimsAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s'),'createdAt'=>(new \DateTimeImmutable())->format('Y-m-d H:i:s')]);
            return $user;
        });
    }
    public function revokeAll(int $userId): void { $this->db->executeStatement('UPDATE EXTERNAL_IDENTITY SET revokedAt=NOW() WHERE userId=:user AND revokedAt IS NULL',['user'=>$userId]); }
}
