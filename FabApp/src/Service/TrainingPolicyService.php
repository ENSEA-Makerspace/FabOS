<?php

namespace App\Service;

use App\Entity\Formation;
use App\Entity\Machine;

final class TrainingPolicyService
{
    public const MODE_FREE = 'free';
    public const MODE_THEORY = 'theory';
    public const MODE_THEORY_PHYSICAL = 'theory_physical';

    /**
     * Politique d'accès pilotée par le code, sans colonne ni migration supplémentaire.
     * Les machines absentes de cette liste conservent par défaut une formation théorique.
     */
    private const MACHINE_MODES = [
        'printer-01' => self::MODE_FREE,
        'oscilloscope-01' => self::MODE_FREE,
        'ultimaker-s5-01' => self::MODE_THEORY,
        'prusa-mk3s-01' => self::MODE_THEORY,
        'vinyl-cutter-01' => self::MODE_THEORY,
        'laser-co2-01' => self::MODE_THEORY_PHYSICAL,
        'station-soudure-01' => self::MODE_THEORY_PHYSICAL,
        'cnc-fraiseuse-01' => self::MODE_THEORY_PHYSICAL,
        'brodeuse-01' => self::MODE_THEORY_PHYSICAL,
    ];

    /** @var list<string> */
    private const PHYSICAL_FORMATION_KEYWORDS = [
        'laser',
        'soudure',
        'fraiseuse',
        'cnc',
        'brodeuse',
    ];

    /** @var list<string> */
    private const FREE_MACHINE_KEYWORDS = [
        'imprimante 3d test',
        'oscilloscope',
    ];

    /** @var list<string> */
    private const PHYSICAL_MACHINE_KEYWORDS = [
        'laser',
        'soudure',
        'fraiseuse',
        'cnc',
        'brodeuse',
    ];

    public function getMachineMode(Machine $machine): string
    {
        $token = mb_strtolower(trim((string) $machine->getMachineToken()));
        if ($token !== '' && isset(self::MACHINE_MODES[$token])) {
            return self::MACHINE_MODES[$token];
        }

        $name = $this->normalize($machine->getNom());
        foreach (self::FREE_MACHINE_KEYWORDS as $keyword) {
            if (str_contains($name, $keyword)) {
                return self::MODE_FREE;
            }
        }

        foreach (self::PHYSICAL_MACHINE_KEYWORDS as $keyword) {
            if (str_contains($name, $keyword)) {
                return self::MODE_THEORY_PHYSICAL;
            }
        }

        return self::MODE_THEORY;
    }

    public function machineRequiresTraining(Machine $machine): bool
    {
        return $this->getMachineMode($machine) !== self::MODE_FREE;
    }

    public function machineRequiresPhysicalTraining(Machine $machine): bool
    {
        return $this->getMachineMode($machine) === self::MODE_THEORY_PHYSICAL;
    }

    public function formationRequiresPhysicalTraining(Formation $formation): bool
    {
        $haystack = $this->normalize(sprintf(
            '%s %s',
            $formation->getTitre(),
            (string) $formation->getCategorie(),
        ));

        foreach (self::PHYSICAL_FORMATION_KEYWORDS as $keyword) {
            if (str_contains($haystack, $keyword)) {
                return true;
            }
        }

        return false;
    }

    /**
     * @return array{
     *   physicalRequired: bool,
     *   practicalLabel: string,
     *   practicalShortLabel: string,
     *   practicalDescription: string,
     *   practicalTone: string
     * }
     */
    public function getFormationPolicy(Formation $formation): array
    {
        $physicalRequired = $this->formationRequiresPhysicalTraining($formation);

        return [
            'physicalRequired' => $physicalRequired,
            'practicalLabel' => $physicalRequired
                ? 'Formation pratique nécessaire'
                : 'Formation pratique non nécessaire',
            'practicalShortLabel' => $physicalRequired ? 'Pratique requise' : 'Sans pratique obligatoire',
            'practicalDescription' => $physicalRequired
                ? 'Une prise en main en présentiel avec un membre du FabLab complète les quiz en ligne.'
                : 'La validation en ligne suffit : aucune séance pratique obligatoire n’est demandée.',
            'practicalTone' => $physicalRequired ? 'required' : 'optional',
        ];
    }

    private function normalize(?string $value): string
    {
        $value = mb_strtolower(trim((string) $value));
        $value = str_replace([
            'à', 'â', 'ä', 'á', 'ã', 'å',
            'ç',
            'é', 'è', 'ê', 'ë',
            'î', 'ï', 'í',
            'ô', 'ö', 'ó', 'õ',
            'ù', 'û', 'ü', 'ú',
            'ÿ',
        ], [
            'a', 'a', 'a', 'a', 'a', 'a',
            'c',
            'e', 'e', 'e', 'e',
            'i', 'i', 'i',
            'o', 'o', 'o', 'o',
            'u', 'u', 'u', 'u',
            'y',
        ], $value);

        return preg_replace('/\s+/', ' ', $value) ?? $value;
    }
}
