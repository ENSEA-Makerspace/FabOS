<?php

namespace App\Event;

use App\Entity\Event;

/**
 * What KIND of artwork an event carries — and therefore how the page must draw it.
 *
 * **The rule, and why it is a measurement rather than a setting.** An organiser
 * uploads one of two genuinely different things:
 *
 *   - **a poster** — already a finished design. Its title, its date and its
 *     artwork are *inside the image*. Cropping it to a 21:8 band cuts the words
 *     off, and writing our own title over it says everything twice.
 *   - **a photo** — a picture of the workshop, the room, last year's edition.
 *     It carries no words, so the page has to supply them, and laying them over
 *     the image is what makes the result look like a poster.
 *
 * A poster is portrait and a photo is landscape. That is not a coincidence — it
 * is what the two formats *are*, everywhere, on every wall. So the orientation of
 * the file is the answer, and asking the operator to also tick a box would be
 * asking them to restate something they already told us by uploading the file.
 * It also means no new column, no migration, and nothing that can be set wrong.
 *
 * ⚠️ The cut is at exactly 1:1. Square counts as landscape and gets the overlay,
 * because a square image has room under an overlaid title and a portrait one does
 * not. Move the threshold and you change which layout existing events get, with
 * no edit and no audit trail — so if it ever needs to move, say why here.
 *
 * ⚠️ `getimagesize()` reads the file header, not the pixels, so this is a stat
 * plus a few hundred bytes. It is still memoised per filename: the catalogue asks
 * about twenty events in one request and the same poster can appear twice.
 */
final class EventArtwork
{
    public const POSTER = 'poster';
    public const BANNER = 'banner';

    /** @var array<string, array{kind: string, width: int, height: int}|null> */
    private array $cache = [];

    public function __construct(private readonly string $projectDir)
    {
    }

    /**
     * @return array{has: bool, kind: string, path: ?string, width: int, height: int, ratio: float}
     *         `kind` is BANNER for an event with no artwork at all: with nothing to
     *         show, the page falls back to the brand gradient and writes over it,
     *         which is the banner layout with the picture removed.
     */
    public function describe(Event $event): array
    {
        $none = ['has' => false, 'kind' => self::BANNER, 'path' => null, 'width' => 0, 'height' => 0, 'ratio' => 0.0];

        $filename = $event->getPosterFilename();
        if ($filename === null || $filename === '') {
            return $none;
        }

        $measured = $this->measure($filename);
        if ($measured === null) {
            // The row names a file that is not on disk, or is not an image. The
            // event still has to render, so treat it as artwork-less rather than
            // emitting an <img> that draws a broken-image glyph.
            return $none;
        }

        return [
            'has' => true,
            'kind' => $measured['kind'],
            'path' => 'uploads/events/' . $filename,
            'width' => $measured['width'],
            'height' => $measured['height'],
            'ratio' => $measured['height'] > 0 ? $measured['width'] / $measured['height'] : 0.0,
        ];
    }

    public function isPoster(Event $event): bool
    {
        return $this->describe($event)['kind'] === self::POSTER && $this->describe($event)['has'];
    }

    /** @return array{kind: string, width: int, height: int}|null */
    private function measure(string $filename): ?array
    {
        if (array_key_exists($filename, $this->cache)) {
            return $this->cache[$filename];
        }

        // ⚠️ basename() before touching the filesystem. The column is written by
        // the admin upload with a generated name, but this reads a path out of the
        // database and a stored `../../` would otherwise walk out of the folder.
        $path = $this->projectDir . '/public/uploads/events/' . basename($filename);

        $size = is_file($path) ? @getimagesize($path) : false;
        if ($size === false || ($size[0] ?? 0) < 1 || ($size[1] ?? 0) < 1) {
            return $this->cache[$filename] = null;
        }

        return $this->cache[$filename] = [
            'kind' => $size[0] < $size[1] ? self::POSTER : self::BANNER,
            'width' => $size[0],
            'height' => $size[1],
        ];
    }
}
