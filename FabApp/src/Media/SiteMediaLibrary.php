<?php

namespace App\Media;

use App\Image\ImageNormalizer;
use Doctrine\DBAL\Connection;
use Symfony\Component\HttpFoundation\File\UploadedFile;

/**
 * Les images d'identité du site — logo et compagnie — téléversées depuis
 * l'écran, jamais déposées sur le serveur à la main (S165).
 *
 * 🔴 **Ce qui existait avant : une CHAÎNE.** `site_logo_path` nommait un fichier
 * qui devait déjà se trouver dans `public/images/`. Rien ne téléversait, donc
 * poser un logo demandait un accès SSH — un réglage qu'on ne peut pas régler
 * depuis l'écran qui le propose.
 *
 * 🔴 **Le nom du fichier téléversé n'atteint JAMAIS le disque.** Le fichier
 * s'appelle `<mediaId>.<ext>`, où `mediaId` est tiré au sort. Deux personnes qui
 * envoient `logo.png` ne s'écrasent pas, et un nom choisi par l'utilisateur ne
 * traverse pas le système de fichiers. Le nom d'origine est gardé pour
 * l'affichage, et pour ça seulement.
 *
 * 🔴 **SVG REFUSÉ, et ce n'est pas un oubli.** L'ancienne expression régulière
 * l'acceptait. Un SVG est un document XML qui peut porter `<script>`, servi
 * depuis NOTRE origine : c'est du code exécuté dans la session de chaque
 * visiteur. Tant que les fichiers ne sont pas servis depuis un domaine séparé,
 * le format ne peut pas être accepté — et le refus le dit, plutôt que de le
 * laisser passer en silence.
 * ⚠️ La contrepartie est réelle : un logo vectoriel devient un PNG. C'est le prix,
 * et l'écran l'annonce avant qu'on choisisse le fichier.
 *
 * ⚠️ **Le type est décidé par le CONTENU, pas par l'extension.** `getimagesize()`
 * lit les octets ; une extension est une affirmation de l'appelant.
 *
 * ⚠️ **DBAL et fail-safe**, comme les autres dépôts de configuration : sans la
 * table, la médiathèque est vide et le site sert le logo livré. C'est ce qui rend
 * le code déployable avant sa migration.
 */
final class SiteMediaLibrary
{
    /** ⚠️ Sondée une fois par processus, comme `MailOverrides`. */
    private ?bool $storageReady = null;

    /**
     * 🔴 Trois formats matriciels, et pas un de plus. Voir la note sur le SVG
     * en tête de classe : la liste est courte parce que chaque entrée est une
     * surface d'attaque de plus, pas parce que personne n'y a pensé.
     *
     * @var array<string, string> mime => extension
     */
    private const ACCEPTED = [
        'image/png' => 'png',
        'image/jpeg' => 'jpg',
        'image/webp' => 'webp',
    ];

    /**
     * ⚠️ Avant normalisation. `ImageNormalizer` ramène ensuite le grand côté à
     * 2400 px : ce plafond-ci n'existe que pour ne pas décoder en mémoire une
     * image que personne n'a voulu envoyer.
     */
    private const MAX_BYTES = 12_000_000;

    public function __construct(
        private readonly Connection $db,
        private readonly ImageNormalizer $images,
        private readonly string $uploadDir,
    ) {
    }

    /** @return list<array<string, mixed>> les plus récentes d'abord */
    public function all(): array
    {
        if (!$this->isStorageReady()) {
            return [];
        }

        try {
            return $this->db->fetchAllAssociative('SELECT * FROM SITE_MEDIA ORDER BY uploadedAt DESC, id DESC');
        } catch (\Throwable) {
            return [];
        }
    }

    /** @return array<string, mixed>|null */
    public function find(string $mediaId): ?array
    {
        if (!$this->isStorageReady() || !self::isMediaId($mediaId)) {
            return null;
        }

        try {
            $row = $this->db->fetchAssociative('SELECT * FROM SITE_MEDIA WHERE mediaId = ?', [$mediaId]);
        } catch (\Throwable) {
            return null;
        }

        return is_array($row) ? $row : null;
    }

    /** Le chemin public, relatif à `public/`, tel qu'`asset()` l'attend. */
    public function assetPath(string $mediaId): ?string
    {
        $row = $this->find($mediaId);

        return $row === null ? null : 'uploads/identity/' . $row['filename'];
    }

    /**
     * Range un fichier téléversé.
     *
     * ⚠️ **Les dimensions sont mesurées APRÈS normalisation, pas avant.**
     * `getimagesize()` rapporte comment les pixels sont STOCKÉS ; un navigateur
     * les dessine corrigés par l'orientation EXIF. Les deux divergent sur toute
     * orientation de 5 à 8, c'est-à-dire toute photo prise de travers.
     * `ImageNormalizer` remet l'image d'aplomb d'abord ; mesurer ensuite donne
     * les chiffres qu'on verra à l'écran, et évite d'avoir à raisonner sur la
     * différence. (Et c'est lui, pas un second bout de code EXIF, parce que deux
     * implémentations divergent et que la panne est une photo couchée que
     * personne ne remarque pendant un mois.)
     *
     * @return array{ok: bool, mediaId?: string, error?: string}
     */
    public function store(UploadedFile $file): array
    {
        if (!$this->isStorageReady()) {
            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        if ($file->getSize() !== false && $file->getSize() > self::MAX_BYTES) {
            return ['ok' => false, 'error' => 'media.error_too_big'];
        }

        $probe = @getimagesize($file->getPathname());
        $mime = is_array($probe) ? (string) ($probe['mime'] ?? '') : '';
        if (!isset(self::ACCEPTED[$mime])) {
            // ⚠️ Un SVG arrive ici : `getimagesize()` ne lui rend pas de mime
            // matriciel. Le message doit donc nommer le cas, sinon l'opérateur
            // croit à une panne de téléversement.
            return ['ok' => false, 'error' => 'media.error_format'];
        }

        $extension = self::ACCEPTED[$mime];
        $mediaId = bin2hex(random_bytes(16));

        if (!is_dir($this->uploadDir) && !@mkdir($this->uploadDir, 0775, true) && !is_dir($this->uploadDir)) {
            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        try {
            $file->move($this->uploadDir, $mediaId . '.' . $extension);
        } catch (\Throwable) {
            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        $path = $this->uploadDir . '/' . $mediaId . '.' . $extension;

        // ⚠️ `capUploaded()` rend l'extension RÉELLEMENT écrite, qui n'est pas
        // toujours celle passée : un PNG sans canal alpha est une photo dans un
        // format qui ne compresse pas les photos, et il revient en `jpg`. Le nom
        // se construit sur ce retour, sinon il ment sur son contenu.
        $written = $this->images->capUploaded($path, $extension);
        if ($written !== $extension) {
            $newPath = $this->uploadDir . '/' . $mediaId . '.' . $written;
            if (@rename($path, $newPath)) {
                $path = $newPath;
                $extension = $written;
            }
        }

        $final = @getimagesize($path);

        try {
            $this->db->executeStatement(
                'INSERT INTO SITE_MEDIA (mediaId, filename, originalName, mimeType, width, height, bytes, uploadedAt)
                 VALUES (?, ?, ?, ?, ?, ?, ?, NOW())',
                [
                    $mediaId,
                    $mediaId . '.' . $extension,
                    mb_substr($file->getClientOriginalName(), 0, 255),
                    is_array($final) ? (string) ($final['mime'] ?? $mime) : $mime,
                    is_array($final) ? (int) $final[0] : null,
                    is_array($final) ? (int) $final[1] : null,
                    @filesize($path) ?: null,
                ],
            );
        } catch (\Throwable) {
            // 🔴 La ligne n'a pas été écrite : le fichier ne doit pas rester. Un
            // fichier sans ligne est invisible depuis l'écran et indéboulonnable.
            @unlink($path);

            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        return ['ok' => true, 'mediaId' => $mediaId];
    }

    /**
     * Supprime — **sauf si un thème s'en sert**.
     *
     * 🔴 **Le refus porte sur le BROUILLON autant que sur le publié.** Supprimer
     * l'image qu'un brouillon référence laisserait une publication future poser
     * un logo qui n'existe plus : un site à moitié rhabillé, découvert par les
     * visiteurs. Les deux références comptent, et l'appelant les fournit — ce
     * fichier ne connaît pas les thèmes.
     *
     * @param list<string> $referenced les mediaId qu'un thème utilise
     *
     * @return array{ok: bool, error?: string}
     */
    public function delete(string $mediaId, array $referenced): array
    {
        if (!$this->isStorageReady() || !self::isMediaId($mediaId)) {
            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        if (in_array($mediaId, $referenced, true)) {
            return ['ok' => false, 'error' => 'media.error_in_use'];
        }

        $row = $this->find($mediaId);
        if ($row === null) {
            return ['ok' => false, 'error' => 'media.error_unknown'];
        }

        try {
            $this->db->executeStatement('DELETE FROM SITE_MEDIA WHERE mediaId = ?', [$mediaId]);
        } catch (\Throwable) {
            return ['ok' => false, 'error' => 'media.error_storage'];
        }

        // ⚠️ La ligne d'abord, le fichier ensuite. Dans l'autre ordre, un échec
        // laisse une ligne qui pointe sur rien — et l'écran affiche une image
        // cassée que personne ne peut retirer.
        @unlink($this->uploadDir . '/' . basename((string) $row['filename']));

        return ['ok' => true];
    }

    public static function isMediaId(string $value): bool
    {
        return preg_match('/^[0-9a-f]{32}$/', $value) === 1;
    }

    private function isStorageReady(): bool
    {
        if ($this->storageReady !== null) {
            return $this->storageReady;
        }

        try {
            return $this->storageReady = $this->db->createSchemaManager()->tablesExist(['SITE_MEDIA']);
        } catch (\Throwable) {
            return $this->storageReady = false;
        }
    }
}
