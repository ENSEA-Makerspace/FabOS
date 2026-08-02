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
     * Long edge of the cached copy, in pixels. One size for both the cards and
     * the detail page's hero: 1600 is generous for a 1400 px banner and far more
     * than a 464 px card needs, and a second size would be a second thing to
     * invalidate for a saving nobody would notice.
     */
    private const THUMB_EDGE = 1600;

    /**
     * @return array{has: bool, kind: string, path: ?string, thumb: ?string, width: int, height: int, ratio: float}
     *         `kind` is BANNER for an event with no artwork at all: with nothing to
     *         show, the page falls back to the brand gradient and writes over it,
     *         which is the banner layout with the picture removed.
     *
     *         `thumb` is what a LIST should draw and `path` is what the detail
     *         page draws. ⚠️ They are different for a reason measured on the live
     *         box: the two posters there are 5712×4284 PNGs of **23 MB each**, so
     *         the catalogue was pulling 46 MB to fill two 464 px cards, behind a
     *         single-threaded `php -S`. The cards stayed blank for seconds and
     *         looked broken. `thumb` falls back to `path` if the resize cannot be
     *         made, so a caller never has to check.
     */
    public function describe(Event $event): array
    {
        $none = ['has' => false, 'kind' => self::BANNER, 'path' => null, 'thumb' => null, 'width' => 0, 'height' => 0, 'ratio' => 0.0];

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

        $path = 'uploads/events/' . $filename;

        return [
            'has' => true,
            'kind' => $measured['kind'],
            'path' => $path,
            'thumb' => $this->thumbnail($filename, $measured) ?? $path,
            'width' => $measured['width'],
            'height' => $measured['height'],
            'ratio' => $measured['height'] > 0 ? $measured['width'] / $measured['height'] : 0.0,
        ];
    }

    /**
     * The cached card-sized copy, made on first request and reused after.
     *
     * ⚠️ **Every failure path returns null**, and the caller falls back to the
     * original. A missing thumbnail must cost a slow card, never a 500 — this
     * runs inside a page render, on operator-supplied files, and GD will happily
     * fail on a file that `getimagesize()` was perfectly happy with.
     *
     * ⚠️ Regenerated when the source is newer than the cache, so replacing a
     * poster under the same name (which the admin upload does not do today, but
     * could) cannot leave the old picture on the cards forever.
     *
     * @param array{kind: string, width: int, height: int, orientation: int} $measured
     */
    private function thumbnail(string $filename, array $measured): ?string
    {
        $edge = max($measured['width'], $measured['height']);
        if ($edge <= self::THUMB_EDGE || !function_exists('imagecreatefromstring')) {
            // Already small enough — a second copy would be pure cost.
            return null;
        }

        $source = $this->projectDir . '/public/uploads/events/' . basename($filename);
        $name = pathinfo(basename($filename), PATHINFO_FILENAME) . '-' . self::THUMB_EDGE . '.jpg';
        $cacheDir = $this->projectDir . '/public/uploads/events/cache';
        $target = $cacheDir . '/' . $name;
        $web = 'uploads/events/cache/' . $name;

        if (is_file($target) && filemtime($target) >= filemtime($source)) {
            return $web;
        }

        if (!is_dir($cacheDir) && !@mkdir($cacheDir, 0o775, true) && !is_dir($cacheDir)) {
            return null;
        }

        try {
            $raw = @file_get_contents($source);
            $image = $raw !== false ? @imagecreatefromstring($raw) : false;
            unset($raw);
            if ($image === false) {
                return null;
            }

            // ⚠️ **Bake the orientation in.** GD reads stored pixels and writes
            // stored pixels; the tag does not survive into the JPEG we emit. Skip
            // this and the cached copy is the one image on the page lying on its
            // side, while the original beside it looks fine — which is exactly
            // what shipped before this line existed.
            $image = $this->upright($image, $measured['orientation'] ?? 1);

            $scale = self::THUMB_EDGE / $edge;
            $resized = @imagescale(
                $image,
                max(1, (int) round($measured['width'] * $scale)),
                max(1, (int) round($measured['height'] * $scale)),
            );
            imagedestroy($image);
            if ($resized === false) {
                return null;
            }

            // JPEG rather than PNG: these are photographs and posters, and the
            // 23 MB PNG that started this is 23 MB precisely because PNG does not
            // compress photographs. Transparency is not worth keeping here — the
            // card crops the picture to fill an opaque box either way.
            $ok = @imagejpeg($resized, $target, 82);
            imagedestroy($resized);

            return $ok ? $web : null;
        } catch (\Throwable) {
            return null;
        }
    }

    public function isPoster(Event $event): bool
    {
        $art = $this->describe($event);

        return $art['has'] && $art['kind'] === self::POSTER;
    }

    /**
     * Apply an EXIF orientation to a GD image, returning an upright one.
     *
     * `imagerotate()` turns counter-clockwise, which is the opposite of how the
     * tag is usually described — 6 means "the camera was rotated 90° clockwise",
     * so the fix is −90. Both are here rather than in a comment at one call site
     * because getting the sign wrong is invisible on a square test image.
     *
     * @param \GdImage $image
     */
    private function upright(\GdImage $image, int $orientation): \GdImage
    {
        $rotate = match ($orientation) {
            3, 4 => 180,
            5, 8 => 90,
            6, 7 => -90,
            default => 0,
        };

        if ($rotate !== 0) {
            $rotated = @imagerotate($image, $rotate, 0);
            if ($rotated !== false) {
                imagedestroy($image);
                $image = $rotated;
            }
        }

        // 2, 4, 5 and 7 are the mirrored halves of the set. Rare — no phone
        // produces them — but a two-line case is cheaper than a wrong picture.
        if (in_array($orientation, [2, 4, 5, 7], true)) {
            @imageflip($image, IMG_FLIP_HORIZONTAL);
        }

        return $image;
    }

    /** @return array{kind: string, width: int, height: int, orientation: int}|null */
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

        // ⚠️ **Orientation first, then measure.** `getimagesize()` reports how the
        // pixels are STORED; a browser draws how they are meant to be SEEN. The
        // two disagree on any file with an EXIF orientation of 5–8, which is
        // every photo taken by a phone held sideways — and the first two posters
        // on the live box are exactly that: 5712×4284 on disk, portrait on screen.
        // Measuring the stored dimensions called a portrait poster a landscape
        // banner and cropped it to a letterbox.
        $orientation = $this->orientation($path, (string) ($size['mime'] ?? ''));
        [$width, $height] = in_array($orientation, [5, 6, 7, 8], true)
            ? [$size[1], $size[0]]
            : [$size[0], $size[1]];

        return $this->cache[$filename] = [
            'kind' => $width < $height ? self::POSTER : self::BANNER,
            'width' => $width,
            'height' => $height,
            'orientation' => $orientation,
        ];
    }

    /**
     * The EXIF orientation tag, 1–8, defaulting to 1 (upright).
     *
     * ⚠️ **`exif_read_data()` is not enough**, and that is the whole reason this
     * exists. It reads JPEG APP1 fine, but returns nothing for a PNG's `eXIf`
     * chunk — and the artwork on the live box is PNG. So the chunk is located by
     * hand and its TIFF header parsed for tag 0x0112. Thirty lines, no
     * dependency, and it fails to 1 on anything it does not recognise.
     */
    private function orientation(string $path, string $mime): int
    {
        try {
            if ($mime === 'image/jpeg' && function_exists('exif_read_data')) {
                $exif = @exif_read_data($path);

                return $this->clampOrientation($exif['Orientation'] ?? 1);
            }

            if ($mime !== 'image/png') {
                return 1;
            }

            // The eXIf chunk is near the front, before the pixel data. Reading a
            // fixed prefix keeps a 23 MB file from being pulled into memory just
            // to answer "which way up".
            $head = @file_get_contents($path, false, null, 0, 262_144);
            $at = $head !== false ? strpos($head, 'eXIf') : false;
            if ($at === false) {
                return 1;
            }

            $length = unpack('N', substr($head, $at - 4, 4))[1] ?? 0;
            $tiff = substr($head, $at + 4, min($length, 65_536));

            return $this->orientationFromTiff($tiff);
        } catch (\Throwable) {
            return 1;
        }
    }

    /** Scan IFD0 of a TIFF header for tag 0x0112. */
    private function orientationFromTiff(string $tiff): int
    {
        $byteOrder = substr($tiff, 0, 2);
        if ($byteOrder !== 'II' && $byteOrder !== 'MM') {
            return 1;
        }

        $short = static fn (int $o): int => unpack($byteOrder === 'II' ? 'v' : 'n', substr($tiff, $o, 2))[1] ?? 0;
        $long = static fn (int $o): int => unpack($byteOrder === 'II' ? 'V' : 'N', substr($tiff, $o, 4))[1] ?? 0;

        $ifd = $long(4);
        if ($ifd < 8 || $ifd + 2 > strlen($tiff)) {
            return 1;
        }

        $count = $short($ifd);
        for ($i = 0; $i < $count; ++$i) {
            $entry = $ifd + 2 + ($i * 12);
            if ($entry + 12 > strlen($tiff)) {
                break;
            }
            if ($short($entry) === 0x0112) {
                // A SHORT value sits in the first two bytes of the 4-byte value
                // field, in the file's own byte order — not at an offset.
                return $this->clampOrientation($short($entry + 8));
            }
        }

        return 1;
    }

    private function clampOrientation(mixed $value): int
    {
        $value = is_numeric($value) ? (int) $value : 1;

        return $value >= 1 && $value <= 8 ? $value : 1;
    }
}
