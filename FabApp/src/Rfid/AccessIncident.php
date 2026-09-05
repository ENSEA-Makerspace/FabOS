<?php

namespace App\Rfid;

use App\Entity\AccessRfidLog;

/**
 * Un refus d'accès, et **ce qu'il faut faire pour qu'il n'arrive plus** (S176).
 *
 * 🔴 **Le défaut que ce fichier remplace.** `/admin/access-rfid-logs` disait
 * parfaitement CE QUI s'est passé — « Badge requis manquant », « Lecteur
 * inactif » — et jamais QUOI FAIRE. Un opérateur devant un refus devait deviner
 * seul si le problème était le membre, sa formation, son badge ou le boîtier,
 * puis aller le chercher dans un autre écran, en retapant un nom. C'est
 * exactement la mesure de sortie de S176 : « un refus se corrige depuis
 * l'incident, sans chercher dans un journal ».
 *
 * ⚠️ **Une CAUSE, une correction, un endroit.** La correspondance vit ici et pas
 * dans une cascade de `{% if %}` : deux gabarits listent déjà ces lignes
 * (`/admin/access-rfid-logs` et `/admin/utilisateurs/{id}`), et une règle
 * recopiée dans deux gabarits est une règle qui diverge — c'est très exactement
 * ce qui est arrivé au vocabulaire des statuts avant que `_rfid_result` ne
 * l'unifie.
 *
 * ⚠️ **Ce service ne rend PAS d'URL, il rend un nom de route.** Résoudre l'URL
 * ici court-circuiterait `can_reach()`, donc afficherait à un formateur un lien
 * vers un écran que le pare-feu lui refusera. Le gabarit demande la permission ;
 * ce fichier ne connaît que la destination.
 *
 * 🅿️ **Ce qu'il ne fait pas, volontairement** : il ne CORRIGE rien. Un bouton
 * « accorder le badge » directement depuis un journal est une écriture sans
 * contexte — on ne voit ni les autres droits du membre, ni pourquoi le badge
 * manque. Il mène là où la décision se prend, avec tout ce qu'il faut pour la
 * prendre.
 */
final class AccessIncident
{
    /**
     * ⚠️ **Les deux générations de vocabulaire, ensemble.** Cette table contient
     * `missing_badge` ET `REQUIRED_BADGE_MISSING`, `NO_TRAINING` ET
     * `TRAINING_REQUIRED` : un firmware que personne n'a mis à jour continue
     * d'écrire l'ancienne forme. N'en traiter qu'une laisserait 42 des 72 refus
     * de cette installation sans correction proposée — mesuré le 2026-09-06.
     *
     * `fix` est une clé de traduction ; `target` dit vers QUOI mener, et la
     * méthode ci-dessous décide si la cible existe vraiment sur cette ligne.
     */
    private const CAUSES = [
        'missing_badge' => ['fix' => 'incident.fix_missing_badge', 'target' => 'user'],
        'REQUIRED_BADGE_MISSING' => ['fix' => 'incident.fix_missing_badge', 'target' => 'user'],
        'NO_TRAINING' => ['fix' => 'incident.fix_no_training', 'target' => 'user'],
        'TRAINING_REQUIRED' => ['fix' => 'incident.fix_no_training', 'target' => 'user'],
        'reader_inactive' => ['fix' => 'incident.fix_reader_inactive', 'target' => 'reader'],
        'unknown_rfid' => ['fix' => 'incident.fix_unknown_rfid', 'target' => 'users'],
        'unknown_machine' => ['fix' => 'incident.fix_unknown_machine', 'target' => 'reader'],
        'invalid_payload' => ['fix' => 'incident.fix_invalid_payload', 'target' => 'reader'],
        'unauthorized_device' => ['fix' => 'incident.fix_unauthorized_device', 'target' => 'reader'],
        'device_api_not_configured' => ['fix' => 'incident.fix_api_not_configured', 'target' => 'settings'],
        'server_error' => ['fix' => 'incident.fix_server_error', 'target' => null],
    ];

    /**
     * @return array{fix: string, route: ?string, params: array<string, int|string>}|null
     *         `null` quand la ligne n'est pas un incident — une autorisation n'a
     *         rien à corriger, et proposer une action y serait du bruit.
     */
    public function of(AccessRfidLog $log): ?array
    {
        if ($log->isAuthorized()) {
            return null;
        }

        $cause = self::CAUSES[$log->getStatus()] ?? null;
        if ($cause === null) {
            return null;
        }

        [$route, $params] = $this->destination($cause['target'], $log);

        return ['fix' => $cause['fix'], 'route' => $route, 'params' => $params];
    }

    /**
     * ⚠️ **Une cible ABSENTE rend l'incident sans lien, pas un lien mort.** Un
     * refus `unknown_rfid` n'a par construction aucun membre — c'est même sa
     * définition — et 18 des 30 `missing_badge` de cette installation n'ont
     * aucun lecteur, parce qu'ils ont été écrits avant que la colonne existe.
     * Fabriquer un lien vers `null` produirait `/admin/utilisateurs/` : une page
     * de liste présentée comme la fiche de quelqu'un.
     *
     * @return array{0: ?string, 1: array<string, int|string>}
     */
    private function destination(?string $target, AccessRfidLog $log): array
    {
        return match ($target) {
            'user' => $log->getUtilisateur()?->getId() !== null
                ? ['app_admin_user_detail', ['id' => $log->getUtilisateur()->getId()]]
                : [null, []],
            'reader' => $log->getReader()?->getId() !== null
                ? ['app_admin_rfid_reader_edit', ['id' => $log->getReader()->getId()]]
                : ['app_admin_rfid_readers', []],
            'users' => ['app_admin_users', []],
            'settings' => ['app_admin_rfid_readers', []],
            default => [null, []],
        };
    }
}
