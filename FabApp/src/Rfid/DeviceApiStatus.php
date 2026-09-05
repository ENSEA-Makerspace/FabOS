<?php

namespace App\Rfid;

/**
 * L'API des boîtiers est-elle seulement ALLUMÉE ? (S176)
 *
 * 🔴 **Le défaut que ce fichier rend visible.** Depuis S171, un appel de boîtier
 * arrivant sans `FABOS_RFID_API_TOKEN` configuré reçoit `503
 * device_api_not_configured` — la garde échoue FERMÉE, ce qui est juste. Mais
 * **aucun écran ne le disait**. Sur cette installation la variable est absente :
 * tout boîtier qu'on brancherait serait refusé, en silence, pendant que
 * `/admin/rfid-readers` affiche des lecteurs « prêts » et que le formulaire
 * explique posément comment les câbler.
 *
 * ⚠️ **Une garde correcte et invisible est une panne différée.** Le premier
 * boîtier posé au mur aurait donné un refus que personne n'aurait su expliquer,
 * parce que la cause n'est ni dans le boîtier, ni dans le membre, ni dans le
 * lecteur — elle est dans un `.env` que rien n'affiche.
 *
 * ⚠️ **Ce service ne rend JAMAIS le jeton, ni sa longueur, ni son empreinte.**
 * Il répond à une question booléenne posée par un écran d'administration. Un
 * écran qui montre un secret pour prouver qu'il existe est le défaut que S171 a
 * retiré du mode d'emploi des lecteurs.
 */
final class DeviceApiStatus
{
    public const VAR = 'FABOS_RFID_API_TOKEN';

    /**
     * ⚠️ **Les trois sources, dans le même ordre que la garde elle-même.**
     * `RfidMachineController::rejectUnauthorizedDevice()` lit `getenv()` puis
     * `$_ENV` puis `$_SERVER` ; lire autrement ici produirait un écran qui
     * annonce « configurée » à une API qui refuse — la deuxième vérité pour un
     * seul fait, encore.
     */
    public function isConfigured(): bool
    {
        $fromEnv = getenv(self::VAR);
        $value = $fromEnv !== false
            ? $fromEnv
            : ($_ENV[self::VAR] ?? $_SERVER[self::VAR] ?? '');

        return trim((string) $value) !== '';
    }
}
