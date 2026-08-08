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

    /**
     * Long edge of a stored creation image, and of its thumbnail.
     *
     * Smaller than `STORED_EDGE` because a creation image has no re-cut case: it
     * is drawn at ~460 in the leaderboard grid and at most ~1400 on the detail
     * page, and nothing else consumes it.
     */
    public const CREATION_EDGE = 1600;
    public const CREATION_THUMB_EDGE = 640;

    /**
     * Byte target for the main creation image. Not a hard cap — the quality
     * ladder stops descending once a step fits, and takes the last step if none
     * does. A picture that will not compress is still better stored than refused.
     */
    private const CREATION_TARGET_BYTES = 850_000;

    /** WebP where the box supports it, JPEG otherwise. Callers name the file from this. */
    public function outputExtension(): string
    {
        return function_exists('imagewebp') ? 'webp' : 'jpg';
    }

    /** Whether GD can do the work at all. A caller that gets `false` must refuse the upload. */
    public function isAvailable(): bool
    {
        return extension_loaded('gd')
            && function_exists('getimagesize')
            && function_exists('imagecreatetruecolor')
            && function_exists('imagecopyresampled')
            && (function_exists('imagewebp') || function_exists('imagejpeg'));
    }

    /**
     * Write a creation image and its thumbnail from one upload.
     *
     * ⚠️ **This is the merged form of what were briefly two classes.** `main`
     * grew a `CreationImageOptimizer` that did the derivative pair well — the
     * quality ladder, the byte target and the atomic tmp+rename below are its
     * work — while orienting with `exif_read_data()` alone, which returns nothing
     * for a PNG and left those sideways. That half is deleted; this method calls
     * `orientation()`/`upright()` like every other path here, so there is one EXIF
     * implementation in the codebase and it is the one that reads PNG.
     *
     * ⚠️ **Orientation is applied once, to the source, before either resize** —
     * deriving a thumbnail from a sideways main image gives you two wrong files
     * instead of one.
     *
     * Returns false without writing anything if either derivative fails; a
     * half-written pair is worse than none, because the template's `onerror`
     * fallback would silently mask it.
     */
    public function storeCreationImage(string $sourcePath, string $imageDir, string $thumbDir, string $fileName): bool
    {
        if (!$this->isAvailable() || !is_file($sourcePath)) {
            return false;
        }

        $fileName = basename($fileName);
        if ($fileName === '' || preg_match('/^[A-Za-z0-9._-]+$/', $fileName) !== 1) {
            return false;
        }

        $extension = strtolower(pathinfo($fileName, PATHINFO_EXTENSION));
        $extension = $extension === 'jpeg' ? 'jpg' : $extension;
        if (!in_array($extension, ['webp', 'jpg'], true)) {
            return false;
        }

        if (!$this->ensureDirectory($imageDir) || !$this->ensureDirectory($thumbDir)) {
            return false;
        }

        $size = @getimagesize($sourcePath);
        if ($size === false || ($size[0] ?? 0) < 1 || ($size[1] ?? 0) < 1) {
            return false;
        }

        $main = null;
        $thumb = null;

        try {
            $raw = @file_get_contents($sourcePath);
            $source = $raw !== false ? @imagecreatefromstring($raw) : false;
            unset($raw);
            if ($source === false) {
                return false;
            }

            $source = $this->upright($source, $this->orientation($sourcePath, (string) ($size['mime'] ?? '')));

            $main = $this->scaleWithin($source, self::CREATION_EDGE);
            $thumb = $this->scaleWithin($source, self::CREATION_THUMB_EDGE);
            imagedestroy($source);

            if ($main === null || $thumb === null) {
                return false;
            }

            $mainPath = rtrim($imageDir, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR . $fileName;
            $thumbPath = rtrim($thumbDir, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR . $fileName;

            $wroteMain = $this->writeLadder($main, $mainPath, $extension, [82, 78, 74, 70, 66, 62], self::CREATION_TARGET_BYTES);
            $wroteThumb = $this->writeLadder($thumb, $thumbPath, $extension, [78, 74, 70], null);

            if (!$wroteMain || !$wroteThumb) {
                @unlink($mainPath);
                @unlink($thumbPath);

                return false;
            }

            return true;
        } catch (\Throwable) {
            return false;
        } finally {
            if ($main instanceof \GdImage) {
                imagedestroy($main);
            }
            if ($thumb instanceof \GdImage) {
                imagedestroy($thumb);
            }
        }
    }

    /**
     * Copy scaled to fit within $maxEdge, never enlarged.
     *
     * ⚠️ Flattened onto white, because both output containers here are lossy and
     * JPEG has no alpha at all — an unflattened transparent PNG comes out with a
     * black background, which is the bug this line exists to prevent.
     */
    private function scaleWithin(\GdImage $source, int $maxEdge): ?\GdImage
    {
        $width = imagesx($source);
        $height = imagesy($source);
        $ratio = min(1, $maxEdge / max($width, $height));
        $targetWidth = max(1, (int) round($width * $ratio));
        $targetHeight = max(1, (int) round($height * $ratio));

        $target = @imagecreatetruecolor($targetWidth, $targetHeight);
        if ($target === false) {
            return null;
        }

        imagefilledrectangle($target, 0, 0, $targetWidth, $targetHeight, (int) imagecolorallocate($target, 255, 255, 255));

        if (!@imagecopyresampled($target, $source, 0, 0, 0, 0, $targetWidth, $targetHeight, $width, $height)) {
            imagedestroy($target);

            return null;
        }

        return $target;
    }

    /**
     * Encode at descending quality until the result fits $maxBytes, then move it
     * into place. Written to a `.tmp` and renamed so a reader never sees a
     * partial file at the real path.
     *
     * @param list<int> $qualities
     */
    private function writeLadder(\GdImage $image, string $path, string $extension, array $qualities, ?int $maxBytes): bool
    {
        $tmp = $path . '.tmp';
        $wrote = false;

        foreach ($qualities as $quality) {
            @unlink($tmp);
            $wrote = $extension === 'webp'
                ? function_exists('imagewebp') && @imagewebp($image, $tmp, $quality)
                : @imagejpeg($image, $tmp, $quality);

            if (!$wrote || !is_file($tmp)) {
                continue;
            }

            if ($maxBytes === null || @filesize($tmp) <= $maxBytes) {
                break;
            }
        }

        if (!$wrote || !is_file($tmp) || !@rename($tmp, $path)) {
            @unlink($tmp);

            return false;
        }

        return true;
    }

    private function ensureDirectory(string $directory): bool
    {
        return is_dir($directory) || @mkdir($directory, 0775, true) || is_dir($directory);
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
