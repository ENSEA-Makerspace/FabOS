<?php

namespace App\Service;

use Symfony\Component\HttpFoundation\File\UploadedFile;

final class CreationImageOptimizer
{
    private const MAIN_MAX_DIMENSION = 1600;
    private const THUMB_MAX_DIMENSION = 640;
    private const MAIN_TARGET_MAX_BYTES = 850000;

    public function getOutputExtension(): string
    {
        return function_exists('imagewebp') ? 'webp' : 'jpg';
    }

    public function isAvailable(): bool
    {
        return extension_loaded('gd')
            && function_exists('getimagesize')
            && function_exists('imagecreatetruecolor')
            && function_exists('imagecopyresampled')
            && (function_exists('imagewebp') || function_exists('imagejpeg'));
    }

    public function saveOptimizedCreationImage(UploadedFile $uploadedFile, string $imageDirectory, string $thumbDirectory, string $fileName): bool
    {
        if (!$this->isAvailable()) {
            return false;
        }

        if (!$this->ensureDirectory($imageDirectory) || !$this->ensureDirectory($thumbDirectory)) {
            return false;
        }

        $fileName = basename($fileName);
        if ($fileName === '' || !preg_match('/^[A-Za-z0-9._-]+$/', $fileName)) {
            return false;
        }

        $sourcePath = $uploadedFile->getPathname();
        $sourceInfo = @getimagesize($sourcePath);
        if ($sourceInfo === false || empty($sourceInfo[0]) || empty($sourceInfo[1]) || empty($sourceInfo['mime'])) {
            return false;
        }

        $source = $this->createImageResource($sourcePath, (string) $sourceInfo['mime']);
        if (!$source) {
            return false;
        }

        $sourceWidth = imagesx($source);
        $sourceHeight = imagesy($source);
        if ($sourceWidth <= 0 || $sourceHeight <= 0) {
            imagedestroy($source);
            return false;
        }

        $extension = strtolower(pathinfo($fileName, PATHINFO_EXTENSION));
        if (!in_array($extension, ['webp', 'jpg', 'jpeg'], true)) {
            imagedestroy($source);
            return false;
        }
        if ($extension === 'jpeg') {
            $extension = 'jpg';
        }

        $mainImage = $this->resizeToMaxDimension($source, self::MAIN_MAX_DIMENSION);
        $thumbImage = $this->resizeToMaxDimension($source, self::THUMB_MAX_DIMENSION);
        imagedestroy($source);

        if (!$mainImage || !$thumbImage) {
            if ($mainImage) {
                imagedestroy($mainImage);
            }
            if ($thumbImage) {
                imagedestroy($thumbImage);
            }

            return false;
        }

        $mainPath = rtrim($imageDirectory, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR . $fileName;
        $thumbPath = rtrim($thumbDirectory, DIRECTORY_SEPARATOR) . DIRECTORY_SEPARATOR . $fileName;

        $mainSaved = $this->writeOptimizedImage($mainImage, $mainPath, $extension, [82, 78, 74, 70, 66, 62], self::MAIN_TARGET_MAX_BYTES);
        $thumbSaved = $this->writeOptimizedImage($thumbImage, $thumbPath, $extension, [78, 74, 70], null);

        imagedestroy($mainImage);
        imagedestroy($thumbImage);

        if (!$mainSaved || !$thumbSaved) {
            @unlink($mainPath);
            @unlink($thumbPath);
            return false;
        }

        return true;
    }

    private function createImageResource(string $path, string $mime): mixed
    {
        $image = match ($mime) {
            'image/jpeg' => function_exists('imagecreatefromjpeg') ? @imagecreatefromjpeg($path) : false,
            'image/png' => function_exists('imagecreatefrompng') ? @imagecreatefrompng($path) : false,
            'image/webp' => function_exists('imagecreatefromwebp') ? @imagecreatefromwebp($path) : false,
            default => false,
        };

        if (!$image) {
            return false;
        }

        if ($mime === 'image/jpeg') {
            $image = $this->applyJpegOrientation($image, $path);
        }

        return $image;
    }

    private function applyJpegOrientation(mixed $image, string $path): mixed
    {
        if (!function_exists('exif_read_data')) {
            return $image;
        }

        $exif = @exif_read_data($path);
        $orientation = is_array($exif) ? (int) ($exif['Orientation'] ?? 1) : 1;

        $rotated = match ($orientation) {
            3 => @imagerotate($image, 180, 0),
            6 => @imagerotate($image, 270, 0),
            8 => @imagerotate($image, 90, 0),
            default => false,
        };

        if ($rotated) {
            imagedestroy($image);
            return $rotated;
        }

        return $image;
    }

    private function resizeToMaxDimension(mixed $source, int $maxDimension): mixed
    {
        $sourceWidth = imagesx($source);
        $sourceHeight = imagesy($source);
        $ratio = min(1, $maxDimension / max($sourceWidth, $sourceHeight));
        $targetWidth = max(1, (int) round($sourceWidth * $ratio));
        $targetHeight = max(1, (int) round($sourceHeight * $ratio));

        $target = imagecreatetruecolor($targetWidth, $targetHeight);
        if (!$target) {
            return false;
        }

        $white = imagecolorallocate($target, 255, 255, 255);
        imagefilledrectangle($target, 0, 0, $targetWidth, $targetHeight, $white);

        if (!imagecopyresampled($target, $source, 0, 0, 0, 0, $targetWidth, $targetHeight, $sourceWidth, $sourceHeight)) {
            imagedestroy($target);
            return false;
        }

        return $target;
    }

    /** @param list<int> $qualities */
    private function writeOptimizedImage(mixed $image, string $targetPath, string $extension, array $qualities, ?int $targetMaxBytes): bool
    {
        $tmpPath = $targetPath . '.tmp';
        $saved = false;

        foreach ($qualities as $quality) {
            @unlink($tmpPath);
            $saved = $extension === 'webp'
                ? function_exists('imagewebp') && @imagewebp($image, $tmpPath, $quality)
                : @imagejpeg($image, $tmpPath, $quality);

            if (!$saved || !is_file($tmpPath)) {
                continue;
            }

            if ($targetMaxBytes === null || filesize($tmpPath) <= $targetMaxBytes || $quality === end($qualities)) {
                break;
            }
        }

        if (!$saved || !is_file($tmpPath)) {
            @unlink($tmpPath);
            return false;
        }

        if (!@rename($tmpPath, $targetPath)) {
            @unlink($tmpPath);
            return false;
        }

        return true;
    }

    private function ensureDirectory(string $directory): bool
    {
        return is_dir($directory) || mkdir($directory, 0775, true) || is_dir($directory);
    }
}
