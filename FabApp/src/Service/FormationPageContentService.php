<?php

namespace App\Service;

use App\Entity\Formation;
use App\Entity\Section;
use App\Repository\SectionRepository;
use Doctrine\ORM\EntityManagerInterface;
use Symfony\Contracts\Translation\TranslatorInterface;

final class FormationPageContentService
{
    /**
     * The default formation page.
     *
     * ⚠️ S134c — **the values here are translation keys, and only where the string
     * is FabOS speaking.** The distinction the operator drew, and the one to keep:
     *
     *   UI      — FabOS's own words, identical on every formation: the section
     *             headings, the "how the guided path works" cards, what a practical
     *             sign-off means, the two navigation cards. Keys; translated.
     *   content — anything describing *this* training: its description, objectives,
     *             prerequisites, programme, sessions. **A training exists in one
     *             language and that is it.** Never a key, never translated.
     *
     * `program` and `sessions` below are content and stay literal. They are also
     * plainly wrong as shipped defaults — a fabricated four-part timetable and three
     * fake dates that every install serves until an operator overrides them — but
     * that is a product question, not a translation one. Logged, not renamed.
     *
     * `getContent()` translates this array **before** merging an operator's stored
     * block over it, so their own prose is never passed through a catalogue.
     *
     * @var array<string, mixed>
     */
    private const DEFAULTS = [
        'labels' => [
            'descriptionTitle' => 'fdet.section_description',
            'objectivesTitle' => 'fdet.section_objectives',
            'prerequisitesTitle' => 'fdet.section_prereq',
            'materialTitle' => 'fdet.section_material',
        ],
        'journey' => [
            'kicker' => 'journey.kicker',
            'title' => 'journey.title',
            'intro' => 'journey.intro',
            'cards' => [
                ['title' => 'journey.card1_title', 'text' => 'journey.card1_text'],
                ['title' => 'journey.card2_title', 'text' => 'journey.card2_text'],
                ['title' => 'journey.card3_title', 'text' => 'journey.card3_text'],
            ],
        ],
        /*
         * 🔴 **S134c2 — the invented programme and the three invented sessions are
         * gone.** They were served to a member as if they described the course he
         * was about to take: "00:00 Accueil et sécurité … 02:00 Mise en pratique",
         * and "Mardi prochain 14:00-16:30 · Places disponibles" — a slot attached
         * to no data, that he could not book, on a day nobody scheduled.
         *
         * ⚠️ The TITLES stay keys, because a section heading is interface. Only
         * the VALUES underneath were the lie. A formation with nothing written
         * now renders no programme block and no sessions block at all — see
         * `formation-detail.html.twig`, which tests each list before drawing its
         * card.
         *
         * ⚠️ The examples are not deleted from the product: `EXAMPLES` below is
         * what the ADMIN content editor may offer as a starting point to fill in.
         * The difference that matters is who is being shown them and as what —
         * a suggestion to an operator, never a fact to a member.
         */
        'program' => [
            'title' => 'fdet.section_program',
            'items' => [],
        ],
        'sessions' => [
            'title' => 'fdet.section_sessions',
            'items' => [],
        ],
        'practical' => [
            'title' => 'fdet.practical_card_title',
            'requiredLabel' => 'fdet.practical_required_label',
            'requiredDescription' => 'fdet.practical_required_desc',
            'requiredStatus' => 'fdet.practical_status_required',
            'optionalLabel' => 'fdet.practical_not_required_pill',
            'optionalDescription' => 'fdet.practical_optional_desc',
            'optionalStatus' => 'fdet.practical_status_optional',
        ],
        'related' => [
            'title' => 'fdet.section_related',
            'items' => [
                [
                    'badge' => 'fdet.related_badge',
                    'title' => 'fdet.related_title',
                    'description' => 'fdet.related_description',
                    'button' => 'fdet.related_action',
                ],
                [
                    'badge' => 'fdet.tracking_badge',
                    'title' => 'fdet.tracking_title',
                    'description' => 'fdet.tracking_description',
                    'button' => 'fdet.tracking_action',
                ],
            ],
        ],
    ];

    /**
     * ⚠️ **Starting points for the ADMIN editor, never defaults for the page.**
     * These are the values S134c2 removed from `DEFAULTS`. They are kept because
     * an operator facing an empty programme editor has a fair question — "what
     * goes in here?" — and an example answers it. What they must never do again
     * is reach a member's screen as though someone had written them.
     *
     * Literal French on purpose: this is CONTENT, and the standing rule is that
     * the interface is translated and content is not. A lab writes its programme
     * in its own language.
     */
    public const EXAMPLES = [
        'program' => [
            ['time' => '00:00', 'title' => 'Accueil et sécurité', 'description' => 'Présentation des risques, EPI, consignes atelier et bonnes pratiques.'],
            ['time' => '00:30', 'title' => 'Préparation du projet', 'description' => 'Choix du fichier, réglages principaux et validation avec un encadrant.'],
            ['time' => '01:15', 'title' => 'Démonstration machine', 'description' => 'Lancement, surveillance, arrêt et résolution des incidents fréquents.'],
            ['time' => '02:00', 'title' => 'Mise en pratique', 'description' => 'Production accompagnée et validation des acquis pour l’usage autonome.'],
        ],
    ];

    public function __construct(
        private readonly EntityManagerInterface $entityManager,
        private readonly SectionRepository $sections,
        private readonly TranslatorInterface $translator,
    ) {
    }

    /**
     * Resolve the keys in DEFAULTS through the catalogue.
     *
     * ⚠️ Only values shaped like `namespace.key` are translated. That is what keeps
     * the content values ('00:00', 'available', a course's own programme text)
     * literal without needing a second list to maintain — and it is why a new
     * default must be either an obvious key or obviously not one.
     */
    private function localize(mixed $value): mixed
    {
        if (is_array($value)) {
            return array_map($this->localize(...), $value);
        }

        if (is_string($value) && preg_match('/^[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*$/', $value) === 1) {
            return $this->translator->trans($value);
        }

        return $value;
    }

    /** @return array<string, mixed> */
    public function getContent(Formation $formation): array
    {
        // ⚠️ Translate the defaults FIRST, then merge the operator's stored block on
        // top. The other order would push their own prose through the catalogue.
        $defaults = $this->localize(self::DEFAULTS);
        $content = $defaults;

        foreach ($this->sections->findPageContentBlocks($formation) as $section) {
            $key = substr($section->getTitre(), strlen(SectionRepository::PAGE_BLOCK_PREFIX));
            if (!array_key_exists($key, self::DEFAULTS)) {
                continue;
            }

            $decoded = $this->decode($section->getContenu());
            if ($decoded === null) {
                continue;
            }

            $content[$key] = $this->mergeRecursive($defaults[$key], $decoded);
        }

        $content['lists'] = [
            'objectives' => $this->splitList($formation->getObjectifs(), ['.', "\n"]),
            'prerequisites' => $this->splitList($formation->getPrerequis(), ["\n", '.']),
            'material' => $this->splitList($formation->getMaterielFourni(), [',', "\n"]),
        ];

        return $content;
    }

    /** @param array<string, mixed> $payload */
    public function saveBlock(Formation $formation, string $key, array $payload): void
    {
        if (!array_key_exists($key, self::DEFAULTS)) {
            throw new \InvalidArgumentException('Bloc de contenu inconnu.');
        }

        $section = $this->sections->findPageContentBlock($formation, $key);
        if (!$section instanceof Section) {
            $section = (new Section())
                ->setFormation($formation)
                ->setTitre(SectionRepository::PAGE_BLOCK_PREFIX . $key)
                ->setOrdre(-1000 + array_search($key, array_keys(self::DEFAULTS), true));
            $this->entityManager->persist($section);
        }

        $section->setContenu(json_encode($payload, JSON_UNESCAPED_UNICODE | JSON_UNESCAPED_SLASHES | JSON_THROW_ON_ERROR));
        $this->entityManager->flush();
    }

    /** @return array<string, mixed> */
    public function getDefaults(): array
    {
        // Translated: the content editor shows these to the operator as the starting
        // point they are about to overwrite, not as keys.
        return $this->localize(self::DEFAULTS);
    }

    /** @return list<string> */
    private function splitList(?string $value, array $delimiters): array
    {
        $value = trim((string) $value);
        if ($value === '') {
            return [];
        }

        $patternParts = array_map(static fn (string $delimiter): string => preg_quote($delimiter, '/'), $delimiters);
        $parts = preg_split('/(?:' . implode('|', $patternParts) . ')+/u', $value) ?: [];

        return array_values(array_filter(array_map(
            static fn (string $item): string => trim($item),
            $parts,
        ), static fn (string $item): bool => $item !== ''));
    }

    /** @return array<string, mixed>|null */
    private function decode(?string $value): ?array
    {
        if (!is_string($value) || trim($value) === '') {
            return null;
        }

        try {
            $decoded = json_decode($value, true, 512, JSON_THROW_ON_ERROR);
        } catch (\JsonException) {
            return null;
        }

        return is_array($decoded) ? $decoded : null;
    }

    private function mergeRecursive(mixed $defaults, mixed $stored): mixed
    {
        if (!is_array($defaults) || !is_array($stored)) {
            return $stored;
        }

        if (array_is_list($defaults)) {
            return array_is_list($stored) ? $stored : $defaults;
        }

        $result = $defaults;
        foreach ($stored as $key => $value) {
            if (array_key_exists($key, $defaults)) {
                $result[$key] = $this->mergeRecursive($defaults[$key], $value);
            }
        }

        return $result;
    }
}
