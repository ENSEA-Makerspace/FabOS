<?php

namespace App\Image;

/**
 * Makes an uploaded image fit the page it is going to land on — **once, at the
 * door**, before it is ever stored.
 *
 * **Why this exists (S80).** The two event posters on the live box were
 * 5712×4284 PNGs of **23 MB each**, because that is what the camera produced and
 * nothing between the camera and the disk had an opinion. S79 fixed the symptom
 * on one surface — `EventArtwork` caches a display copy — but every other upload
 * path (lab-page photos, avatars, creation images) still stores whatever it is
 * given, and the original still has to be decoded (~98 MB in memory for that
 * file) whenever a cache is rebuilt.
 *
 * **The two jobs, and why they are in one class.** Capping the size and honouring
 * EXIF orientation are the same pass over the same pixels, and separating them is
 * how you end up with a correctly-sized picture lying on its side. They were
 * written for `EventArtwork` first; this is where they live now, and that class
 * calls in rather than keeping a copy. ⚠️ **Two copies of EXIF logic will drift,
 * and the failure is a sideways photograph nobody notices for a month.**
 *
 * ⚠️ **Orientation before dimensions, always.** `getimagesize()` reports how the
 * pixels are *stored*; a browser draws them EXIF-corrected. The two disagree on
 * any orientation of 5–8, which is every photo taken by a phone held sideways —
 * and both live posters are exactly that. Measuring the stored dimensions is what
 * classified a portrait poster as a landscape banner and cropped it to a
 * letterbox.
 *
 * ⚠️ **`exif_read_data()` is not sufficient.** It reads JPEG APP1 and returns
 * nothing for a PNG's `eXIf` chunk — and the artwork here is PNG. The chunk is
 * located and its TIFF header parsed by hand below.
 */
final class ImageNormalizer
{
    /**
     * Long edge a *stored* upload is capped to.
     *
     * Deliberately larger than any surface draws: `EventArtwork`'s display copy
     * is 1600, cards are ~460 and the widest hero is 1400. 2400 leaves room to
     * re-cut a bigger derivative later without going back to the operator for
     * the file, while still being ~1/25th of what a phone camera hands over.
     */
    public const STORED_EDGE = 2400;

    /**
     * Rewrite an uploaded file in place: upright, capped, re-encoded.
     *
     * ⚠️ Returns the extension that was actually written, which is **not always
     * the one passed in**: a PNG with no alpha channel is a photograph stored in
     * a format that cannot compress photographs, and that is the entire 23 MB.
     * It comes back as `jpg`. The caller must build its filename from the return
     * value, not from what the browser uploaded — otherwise the name lies about
     * the contents.
     *
     * ⚠️ **Every failure path returns the original extension and leaves the file
     * untouched.** An upload that cannot be re-encoded must still be storable —
     * refusing a member's photo because GD did not like it is a worse outcome
     * than keeping a large one.
     */
    public function capUploaded(string $path, string $extension, int $maxEdge = self::STORED_EDGE): string
    {
        if (!function_exists('imagecreatefromstring') || !is_file($path)) {
            return $extension;
        }

        $size = @getimagesize($path);
        if ($size === false || ($size[0] ?? 0) < 1 || ($size[1] ?? 0) < 1) {
            return $extension;
        }

        $orientation = $this->orientation($path, (string) ($size['mime'] ?? ''));
        $turned = in_array($orientation, [5, 6, 7, 8], true);
        [$width, $height] = $turned ? [$size[1], $size[0]] : [$size[0], $size[1]];

        // A PNG that carries no alpha channel is a photograph in the wrong
        // container. Anything with real transparency keeps its format — a logo
        // flattened onto black is a worse bug than a large file.
        $target = $extension;
        if ($extension === 'png' && !$this->pngHasAlpha($path)) {
            $target = 'jpg';
        }

        $edge = max($width, $height);
        if ($edge <= $maxEdge && $orientation === 1 && $target === $extension) {
            // Nothing to do: right way up, small enough, right container.
            return $extension;
        }

        try {
            $raw = @file_get_contents($path);
            $image = $raw !== false ? @imagecreatefromstring($raw) : false;
            unset($raw);
            if ($image === false) {
                return $extension;
            }

            $image = $this->upright($image, $orientation);

            if ($edge > $maxEdge) {
                $scale = $maxEdge / $edge;
                $resized = @imagescale($image, max(1, (int) round($width * $scale)), max(1, (int) round($height * $scale)));
                if ($resized !== false) {
                    imagedestroy($image);
                    $image = $resized;
                }
            }

            $ok = match ($target) {
                'jpg' => @imagejpeg($image, $path, 86),
                'webp' => @imagewebp($image, $path, 86),
                default => $this->writePng($image, $path),
            };
            imagedestroy($image);

            return $ok ? $target : $extension;
        } catch (\Throwable) {
            return $extension;
        }
    }

    /** Alpha preserved — this path only runs for a PNG that actually has some. */
    private function writePng(\GdImage $image, string $path): bool
    {
        imagealphablending($image, false);
        imagesavealpha($image, true);

        return @imagepng($image, $path, 6);
    }

    /**
     * Whether a PNG carries an alpha channel, read straight off IHDR.
     *
     * Colour type is byte 25 of the file; 4 (grey+alpha) and 6 (RGBA) have one.
     * Reading the header beats scanning pixels: it is exact, and it is 26 bytes
     * rather than the 98 MB the file in question decodes to.
     */
    private function pngHasAlpha(string $path): bool
    {
        $head = @file_get_contents($path, false, null, 0, 26);
        if ($head === false || strlen($head) < 26) {
            return true; // Unreadable header: assume alpha and keep the format.
        }

        return in_array(ord($head[25]), [4, 6], true);
    }

    /** The EXIF orientation tag, 1–8, defaulting to 1 (upright). */
    public function orientation(string $path, string $mime): int
    {
        try {
            if ($mime === 'image/jpeg' && function_exists('exif_read_data')) {
                return $this->clamp(@exif_read_data($path)['Orientation'] ?? 1);
            }

            if ($mime !== 'image/png') {
                return 1;
            }

            // The eXIf chunk sits near the front, before the pixel data. Reading
            // a fixed prefix keeps a 23 MB file out of memory just to answer
            // "which way up".
            $head = @file_get_contents($path, false, null, 0, 262_144);
            $at = $head !== false ? strpos($head, 'eXIf') : false;
            if ($at === false || $at < 4) {
                return 1;
            }

            $length = unpack('N', substr($head, $at - 4, 4))[1] ?? 0;

            return $this->orientationFromTiff(substr($head, $at + 4, min($length, 65_536)));
        } catch (\Throwable) {
            return 1;
        }
    }

    /**
     * Apply an EXIF orientation to a GD image, returning an upright one.
     *
     * ⚠️ `imagerotate()` turns **counter-clockwise**, which is the opposite of
     * how the tag is described — 6 means "the camera was rotated 90° clockwise",
     * so the correction is −90. Getting the sign wrong is invisible on a square
     * test image, which is why the mapping is stated here rather than inline.
     */
    public function upright(\GdImage $image, int $orientation): \GdImage
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

        // 2, 4, 5 and 7 are the mirrored half of the set. No phone produces them,
        // but a two-line case is cheaper than a wrong picture.
        if (in_array($orientation, [2, 4, 5, 7], true)) {
            @imageflip($image, IMG_FLIP_HORIZONTAL);
        }

        return $image;
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
                return $this->clamp($short($entry + 8));
            }
        }

        return 1;
    }

    private function clamp(mixed $value): int
    {
        $value = is_numeric($value) ? (int) $value : 1;

        return $value >= 1 && $value <= 8 ? $value : 1;
    }
}
