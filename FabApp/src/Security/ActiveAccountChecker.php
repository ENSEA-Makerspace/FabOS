<?php

namespace App\Security;

use App\Entity\Utilisateur;
use Symfony\Component\Security\Core\Authentication\Token\TokenInterface;
use Symfony\Component\Security\Core\Exception\CustomUserMessageAccountStatusException;
use Symfony\Component\Security\Core\User\UserCheckerInterface;
use Symfony\Component\Security\Core\User\UserInterface;

/**
 * Un compte « inactif » ne se connecte plus.
 *
 * 🔴 **Le défaut, mesuré le 2026-09-05.** `UTILISATEUR.statut` offre exactement
 * `actif` et `inactif`, l'écran d'administration propose de basculer l'un vers
 * l'autre… et **rien ne lisait cette valeur à la connexion**. Pas de
 * `user_checker` dans `security.yaml`, et le fournisseur charge par e-mail sans
 * condition. Un compte désactivé gardait donc son mot de passe et son accès :
 * `inactif` retirait la personne de l'annuaire, des listes d'équipe et du profil
 * public, mais **pas du produit**.
 *
 * ⚠️ **C'est un écart d'ATTENTE avant d'être une faille.** Un opérateur qui
 * désactive un compte croit avoir coupé l'accès — c'est ce que le mot dit. Le
 * jour où il croit à tort, il ne le découvre pas : il n'y a rien à voir.
 *
 * ✅ **Et le risque de fermeture était mesuré avant d'être pris** : un seul
 * compte inactif sur la boîte, `anonymised-8@anonymised.invalid`, c'est-à-dire un
 * compte déjà anonymisé. Personne de réel n'est mis dehors par ce fichier.
 *
 * ⚠️ **`checkPreAuth`, pas `checkPostAuth`** : le refus tombe AVANT la
 * vérification du mot de passe, donc un compte désactivé ne peut pas non plus
 * servir à deviner si une adresse existe.
 * ⚠️ Le message est volontairement le même pour tout compte refusé ici — dire
 * « ce compte est désactivé » renseignerait un attaquant sur l'existence de
 * l'adresse.
 */
final class ActiveAccountChecker implements UserCheckerInterface
{
    public function checkPreAuth(UserInterface $user): void
    {
        if (!$user instanceof Utilisateur) {
            return;
        }

        // ⚠️ On compare à `actif`, jamais à `inactif` : un statut inconnu — une
        // valeur héritée, une faute de frappe dans une migration — doit refuser,
        // pas passer. Le repli va vers le fermé.
        if ($user->getStatut() !== 'actif') {
            throw new CustomUserMessageAccountStatusException('security.account_unavailable');
        }
    }

    /**
     * 🔴 **La signature prend le JETON, et l'oublier a cassé la production.**
     * `UserCheckerInterface::checkPostAuth()` déclare
     * `(UserInterface $user, ?TokenInterface $token = null)` dans cette version de
     * Symfony. Une première version l'a écrite sans le second paramètre : `php -l`
     * l'a validée — la SYNTAXE est correcte — et le site a rendu 500 au premier
     * `cache:clear`, parce qu'une incompatibilité d'interface ne se voit qu'au
     * CHARGEMENT de la classe.
     * ⚠️ C'est la même famille que `tools/ctor_arity.py` : entre `php -l` et
     * l'exécution, il y a un trou, et il se paie en prod.
     */
    public function checkPostAuth(UserInterface $user, ?TokenInterface $token = null): void
    {
    }
}
