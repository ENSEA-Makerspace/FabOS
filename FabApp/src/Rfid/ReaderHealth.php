<?php

namespace App\Rfid;

use App\Entity\RfidReader;

/**
 * Dans quel état est CE boîtier, vraiment ? (S172)
 *
 * 🔴 **Le défaut que ce fichier remplace.** `/admin/rfid-readers` affichait un
 * BOOLÉEN : « Actif » en vert, « Inactif » en gris. Un lecteur ACTIF mais muet
 * depuis deux mois y était donc **vert et « Actif »** — un voyant qui reste au
 * vert quoi qu'il arrive n'informe pas, il rassure, ce qui est pire.
 *
 * ⚠️ **Et il faut être exact sur ce qui a été MESURÉ.** L'unique lecteur de la
 * boîte est `isActive = 0` : il affichait « Inactif », correctement, et j'ai
 * d'abord écrit le contraire. Ce qui est prouvé à l'écran, en le passant
 * temporairement à actif puis en le remettant : le même lecteur, muet depuis le
 * 2026-07-10, passe de « Actif » **vert** à « Hors ligne » **rouge**. Les cinq
 * autres états sont une chaîne de tests courte et lisible, pas une mesure.
 *
 * ⚠️ **`isActive` n'est pas un état, c'est une INTENTION** : « je veux que ce
 * lecteur serve ». L'état, lui, est ce qui se passe — associé ou non, vu ou non,
 * cohérent ou non. Les mélanger est exactement ce que la revue Équipement
 * reproche à l'écran : « le booléen actif + `lastSeenAt` ne distingue pas prêt,
 * hors ligne, non configuré, erreur ou association invalide ».
 *
 * ✅ **Et tout se DÉDUIT de ce qui existe déjà** : aucune migration, aucune
 * colonne neuve. Les états manquants n'étaient pas absents de la base, ils
 * étaient absents de l'écran.
 *
 * ⚠️ **Le seuil hors-ligne est un CHOIX, pas une mesure.** Aucun boîtier ne
 * tourne aujourd'hui, donc personne ne sait à quelle cadence ils se signalent.
 * Une heure est volontairement large : mieux vaut annoncer « prêt » un peu trop
 * longtemps que crier « hors ligne » à chaque coupure Wi-Fi. À resserrer le jour
 * où un vrai lecteur donne sa cadence.
 *
 * ⚠️ `lastSeenAt` est un horodatage MACHINE — convention A au sens de `LabClock`,
 * donc en UTC. Le comparer à un `now` réel est juste ; c'est l'heure MURALE qui
 * demanderait une conversion.
 */
final class ReaderHealth
{
    /** Une heure sans nouvelle et le boîtier est déclaré hors ligne. */
    public const OFFLINE_AFTER = 3600;

    public const ARCHIVED = 'archived';
    public const DISABLED = 'disabled';
    public const UNPAIRED = 'unpaired';
    public const INVALID_PAIRING = 'invalid_pairing';
    public const NEVER_SEEN = 'never_seen';
    public const OFFLINE = 'offline';
    public const READY = 'ready';

    /**
     * 🔴 **L'ordre des tests EST la définition.** Un boîtier archivé et hors
     * ligne est archivé : on annonce la cause la plus AMONT, celle sur laquelle
     * on peut agir. Annoncer « hors ligne » d'un lecteur qu'on a soi-même
     * désactivé enverrait chercher une panne qui n'existe pas.
     *
     * @return array{state: string, signal: string, label: string}
     */
    public function of(RfidReader $reader, ?\DateTimeImmutable $now = null): array
    {
        $now ??= new \DateTimeImmutable();

        if ($reader->isArchived()) {
            return $this->row(self::ARCHIVED, 'muted');
        }
        if (!$reader->isActive()) {
            return $this->row(self::DISABLED, 'muted');
        }

        /*
         * 🔴 **S175 — « associé » ne veut plus dire « a une machine ».** Un
         * boîtier peut désormais commander une PORTE (`AccessPoint`), et cette
         * chaîne de tests aurait déclaré « non associé » chaque lecteur d'entrée
         * du labo — en jaune, en permanence. La question est posée une seule
         * fois, à l'entité : `targetKind()`.
         */
        if ($reader->targetKind() === null) {
            // ⚠️ `caution` et non `stop` : un lecteur créé mais pas encore
            // associé est un travail EN COURS, pas une panne.
            return $this->row(self::UNPAIRED, 'caution');
        }
        // 🔴 Une machine sans jeton ne peut répondre à aucun appel du boîtier :
        // l'association existe à l'écran et ne vaut rien à l'usage. C'est le cas
        // que le booléen ne pouvait pas dire.
        // ⚠️ **Un point d'accès n'a PAS de jeton, et n'en a pas besoin** : il est
        // adressé par le jeton du LECTEUR, pas par le sien. Lui inventer un
        // `accessPointToken` par symétrie serait un second identifiant sans
        // écrivain. Donc ce test ne concerne que la branche machine, et le dire
        // vaut mieux que de laisser croire qu'on l'a oublié.
        $machine = $reader->getMachine();
        if ($machine !== null && trim((string) $machine->getMachineToken()) === '') {
            return $this->row(self::INVALID_PAIRING, 'stop');
        }

        $seen = $reader->getLastSeenAt();
        if ($seen === null) {
            return $this->row(self::NEVER_SEEN, 'caution');
        }
        if ($now->getTimestamp() - $seen->getTimestamp() > self::OFFLINE_AFTER) {
            return $this->row(self::OFFLINE, 'stop');
        }

        return $this->row(self::READY, 'go');
    }

    /** @return array{state: string, signal: string, label: string} */
    private function row(string $state, string $signal): array
    {
        // ⚠️ La clé de traduction est dérivée de l'état, jamais écrite à côté :
        // un état neuf sans libellé se voit tout de suite.
        return ['state' => $state, 'signal' => $signal, 'label' => 'rfid_readers.state_' . $state];
    }
}
