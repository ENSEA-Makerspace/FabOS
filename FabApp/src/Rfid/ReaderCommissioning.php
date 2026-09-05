<?php

namespace App\Rfid;

use App\Entity\RfidReader;

/**
 * La mise en service d'un boîtier, étape par étape (S176).
 *
 * 🔴 **Ce que le formulaire ne disait pas.** L'écran d'un lecteur explique
 * parfaitement comment le câbler — le bloc `.env` à recopier — et ne dit nulle
 * part **où on en est**. Nommé ? associé ? déjà vu ? et surtout : l'API des
 * boîtiers est-elle seulement allumée sur cette installation ? Le seul retour
 * possible était de poser le boîtier au mur et de regarder s'il refuse.
 *
 * ⚠️ **Une liste ORDONNÉE, et l'ordre est celui du travail réel** : on nomme,
 * on associe, on branche, on regarde s'il parle. Une étape en amont qui manque
 * rend les suivantes sans objet — annoncer « jamais connecté » à un boîtier
 * qu'on n'a pas encore associé enverrait chercher une panne de réseau.
 *
 * ⚠️ **Elle ne rend pas un pourcentage.** « 3 sur 5 » invite à courir après le
 * chiffre ; ce qui aide est de savoir QUELLE étape manque, et pourquoi elle
 * compte. Même raison qui a fait reformuler le critère « taux d'aide » de J-10.
 *
 * ✅ **Aucune migration** : tout se déduit de ce qui existe déjà, comme
 * `ReaderHealth`. Ce qui manquait n'était pas dans la base, c'était à l'écran.
 */
final class ReaderCommissioning
{
    public function __construct(private readonly DeviceApiStatus $api)
    {
    }

    /**
     * @return list<array{key: string, done: bool, blocking: bool}>
     *         `blocking` distingue « pas encore fait » de « rien ne marchera
     *         tant que ce n'est pas fait » : sans jeton d'API configuré, aucun
     *         boîtier de l'installation ne peut être autorisé, quel que soit le
     *         soin apporté aux quatre autres étapes.
     */
    public function steps(RfidReader $reader): array
    {
        return [
            ['key' => 'named', 'done' => trim($reader->getName()) !== '', 'blocking' => false],
            ['key' => 'paired', 'done' => $reader->hasValidTarget(), 'blocking' => false],
            ['key' => 'token', 'done' => trim($reader->getReaderToken()) !== '', 'blocking' => false],
            /*
             * 🔴 **L'étape que personne ne pouvait voir.** Depuis S171 la garde
             * de l'API échoue FERMÉE quand `FABOS_RFID_API_TOKEN` est absent —
             * ce qui est juste — mais aucun écran ne le disait. Sur cette
             * installation la variable n'existe pas : tout boîtier branché
             * serait refusé, en silence, pendant que la liste affiche des
             * lecteurs « prêts ». Une garde correcte et invisible est une panne
             * différée.
             */
            ['key' => 'api', 'done' => $this->api->isConfigured(), 'blocking' => true],
            ['key' => 'seen', 'done' => $reader->getLastSeenAt() !== null, 'blocking' => false],
        ];
    }

    /** La première étape qui manque, ou `null` quand tout est fait. */
    public function nextStep(RfidReader $reader): ?string
    {
        foreach ($this->steps($reader) as $step) {
            if (!$step['done']) {
                return $step['key'];
            }
        }

        return null;
    }
}
