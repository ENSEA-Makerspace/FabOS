<?php

namespace App\Service;

use App\Entity\Formation;

final class QuizCatalogService
{
    public const INTERNAL_CATEGORY = 'Quiz interne';

    public function isInternalQuizFormation(?Formation $formation): bool
    {
        return $formation instanceof Formation
            && mb_strtolower(trim((string) $formation->getCategorie())) === mb_strtolower(self::INTERNAL_CATEGORY);
    }

    public function getParentFormationId(?Formation $formation): ?int
    {
        return $this->getIntMarker($formation, 'FABOS_PARENT_FORMATION_ID');
    }

    public function getMachineId(?Formation $formation): ?int
    {
        return $this->getIntMarker($formation, 'FABOS_MACHINE_ID');
    }

    public function getQuizKey(?Formation $formation): ?string
    {
        return $this->getMarker($formation, 'FABOS_QUIZ_KEY');
    }

    public function getQuizContext(?Formation $formation): string
    {
        return strtolower($this->getMarker($formation, 'FABOS_QUIZ_CONTEXT') ?? 'page');
    }

    public function getParentSectionId(?Formation $formation): ?int
    {
        return $this->getIntMarker($formation, 'FABOS_SECTION_ID');
    }

    public function isSectionQuizFormation(?Formation $formation): bool
    {
        return $this->isInternalQuizFormation($formation)
            && $this->getQuizContext($formation) === 'section';
    }

    public function isBonusQuizFormation(?Formation $formation): bool
    {
        if (!$this->isInternalQuizFormation($formation)) {
            return false;
        }

        $value = strtolower($this->getMarker($formation, 'FABOS_BONUS') ?? '0');

        return in_array($value, ['1', 'true', 'yes', 'oui'], true);
    }

    public function isRequiredPageQuizFormation(?Formation $formation): bool
    {
        return $this->isInternalQuizFormation($formation)
            && !$this->isSectionQuizFormation($formation)
            && !$this->isBonusQuizFormation($formation);
    }

    private function getIntMarker(?Formation $formation, string $key): ?int
    {
        $value = $this->getMarker($formation, $key);
        if ($value === null || !ctype_digit($value)) {
            return null;
        }

        $number = (int) $value;

        return $number > 0 ? $number : null;
    }

    private function getMarker(?Formation $formation, string $key): ?string
    {
        if (!$formation instanceof Formation) {
            return null;
        }

        $metadata = trim((string) $formation->getPrerequis());
        if ($metadata === '') {
            return null;
        }

        $pattern = '/(?:^|;)\s*' . preg_quote($key, '/') . '=([^;]+)\s*(?:;|$)/u';
        if (preg_match($pattern, $metadata, $matches) !== 1) {
            return null;
        }

        $value = trim($matches[1]);

        return $value !== '' ? $value : null;
    }
}
