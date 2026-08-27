<?php

namespace App\Form;

use App\Calendar\EventSeries;
use App\Entity\Event;
use App\Entity\EventCategory;
use App\Entity\Formation;
use App\Repository\EventCategoryRepository;
use App\Repository\FormationRepository;
use App\Service\QuizCatalogService;
use App\Service\TrainingQualificationService;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\DateTimeType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class EventAdminType extends AbstractType
{
    /**
     * L'ordre et le découpage de l'écran, déclarés ici — motif `SECTIONS`
     * (S151, R3), déroulé par `site/_form_sections.html.twig`.
     *
     * 🔴 **Cet écran posait ses 15 champs à plat**, et ses deux gabarits (création
     * et édition) tenaient chacun leur liste — donc « un champ ajouté ici ne
     * dessine RIEN tant qu'il n'est pas nommé là-bas », ce que le gabarit
     * documentait déjà comme un défaut vécu : les champs `category` et `formation`
     * de S146f sont sortis invisibles.
     *
     * ⚠️ **Ce qui est replié n'est pas ce qui est rare, c'est ce qui a un défaut
     * SENSÉ.** Un événement sans places déclarées est illimité, un événement est
     * ouvert aux invités, et il n'appartient ni à une catégorie ni à une formation.
     * Les quatre champs correspondants ne posent donc aucune question à qui crée un
     * atelier ordinaire. Le titre, les dates et le lieu, si.
     *
     * ⚠️ **`repeatEvery` / `repeatCount` n'existent qu'à la création** — le gabarit
     * teste la présence de chaque nom, c'est ce qui permet à une seule liste de
     * servir les deux écrans.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_event_form.section_identity',
            'fields' => ['titre', 'description'],
        ],
        [
            'title' => 'admin_event_form.section_when',
            'fields' => ['dateDebut', 'dateFin', 'repeatEvery', 'repeatCount'],
        ],
        [
            'title' => 'admin_event_form.section_where',
            'fields' => ['locationMode', 'venue', 'lieu', 'address'],
        ],
        [
            // Repli : les deux défauts — places illimitées, invités bienvenus —
            // sont le cas courant. Le titre dit ce qu'on trouve dedans, sinon
            // replier revient à cacher.
            'title' => 'admin_event_form.section_registration',
            'fold' => true,
            'fields' => ['capacite', 'guestsAllowed'],
        ],
        [
            // Repli : deux rattachements facultatifs, et un événement n'en a
            // besoin d'aucun.
            'title' => 'admin_event_form.section_link',
            'fold' => true,
            'fields' => ['category', 'formation'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'form.title',
                'row_attr' => ['class' => 'full'],
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 180, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('dateDebut', DateTimeType::class, [
                'label' => 'Début',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'constraints' => [new Assert\NotNull(message: 'La date de début est obligatoire.')],
            ])
            ->add('dateFin', DateTimeType::class, [
                'label' => 'Fin (optionnelle)',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            // ⚠️ **Two different questions, two different fields** (S146f/S146d).
            // The category is a LABEL — how the lab describes this event. The
            // training is a LINK — which training this is a session of. A category
            // called "Séance de formation" cannot answer the second: it does not say
            // WHICH training, so it can neither list a training's real sessions nor
            // make an enrolment mean anything. Both optional; an event needs neither.
            ->add('category', EntityType::class, [
                'label' => 'form.category',
                'class' => EventCategory::class,
                'choice_label' => 'label',
                'required' => false,
                'placeholder' => 'event_categories.field.none',
                // ⚠️ Archived categories are excluded from the PICKER only. An event
                // already carrying one keeps showing it — see EventCategoryRepository.
                'query_builder' => static fn (EventCategoryRepository $repository) => $repository
                    ->createQueryBuilder('c')
                    ->andWhere('c.archivedAt IS NULL')
                    ->orderBy('c.position', 'ASC')
                    ->addOrderBy('c.label', 'ASC'),
                'help' => 'Comment cet événement est décrit : atelier, portes ouvertes… Purement descriptif.',
            ])
            ->add('formation', EntityType::class, [
                'label' => 'Séance de la formation',
                'class' => Formation::class,
                'choice_label' => 'titre',
                'required' => false,
                'placeholder' => 'event_categories.field.no_formation',
                // 🔴 **FabOS's own internal training rows must not be offered.**
                // `[FABOS SECTION] …` and `[FABOS BONUS] …` carry the categories
                // `Quiz interne` and `Validation physique`; they are scaffolding for
                // the guided path, `/formations/{id}` 404s for them, and linking a
                // session to one would point members at a page that does not exist.
                // `TrainingQualificationService::isInternalCategory()` is the one rule
                // — expressed here as the query it implies, not copied as a literal.
                'query_builder' => static fn (FormationRepository $repository) => $repository
                    ->createQueryBuilder('f')
                    ->andWhere('f.categorie IS NULL OR LOWER(TRIM(f.categorie)) NOT IN (:internal)')
                    ->setParameter('internal', [
                        mb_strtolower(QuizCatalogService::INTERNAL_CATEGORY),
                        mb_strtolower(TrainingQualificationService::PHYSICAL_CATEGORY),
                    ])
                    ->orderBy('f.titre', 'ASC'),
                // 🔴 Says what the link does NOT do. Attending never certifies:
                // a trainer validates, and that is a safety rule, not a preference.
                'help' => 'Rattache cet événement à une formation : il apparaîtra dans ses prochaines séances. La présence ne valide aucun badge — un formateur le fait.',
            ])
            ->add('venue', VenueChoiceType::class, [
                'required' => false,
                'placeholder' => 'venues.field.venue_none',
            ])
            ->add('lieu', TextType::class, [
                'label' => 'Nom du lieu',
                'row_attr' => ['class' => 'full'],
                'required' => false,
                'help' => 'Le nom courant de l\'endroit : « Grande salle », « Atelier bois »…',
                'constraints' => [new Assert\Length(max: 180, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('locationMode', ChoiceType::class, [
                'label' => 'Où se déroule l\'événement ?',
                'row_attr' => ['class' => 'full'],
                'choices' => [
                    'Au fablab' => Event::LOCATION_ONSITE,
                    'Ailleurs (adresse spécifique)' => Event::LOCATION_OFFSITE,
                ],
                'expanded' => true,
                'help' => 'Au fablab, l\'adresse est reprise automatiquement des réglages du site.',
            ])
            ->add('address', TextType::class, [
                'label' => 'Adresse (si ailleurs)',
                'row_attr' => ['class' => 'full'],
                'required' => false,
                'help' => 'Adresse postale complète. Un lien d\'itinéraire est généré automatiquement.',
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'L\'adresse ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'form.description',
                'row_attr' => ['class' => 'full'],
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('capacite', IntegerType::class, [
                'label' => 'Nombre de places',
                'required' => false,
                'help' => 'Laissez vide pour un nombre de places illimité. Au-delà, les inscriptions passent en liste d\'attente.',
                'constraints' => [new Assert\PositiveOrZero(message: 'Le nombre de places ne peut pas être négatif.')],
            ])
            ->add('guestsAllowed', CheckboxType::class, [
                'label' => 'Ouvert aux personnes sans compte',
                'required' => false,
                'help' => 'Décochez pour réserver cet événement aux membres connectés. Les invités déjà inscrits gardent leur place.',
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);

        // ⚠️ **Only when creating.** Editing an existing event must never silently
        // generate more of them, and the two fields would be a question with no
        // honest answer on a row that already exists.
        if ($options['allow_repeat']) {
            $builder
                ->add('repeatEvery', ChoiceType::class, [
                    'label' => 'Répéter',
                    'mapped' => false,
                    'required' => false,
                    // 🔴 **S151 — le câblage Stimulus descend du gabarit vers ici.**
                    // La boucle partagée appelle `form_row(form[name])` sans options :
                    // un `data-action` écrit dans le gabarit disparaîtrait à la
                    // conversion, et avec lui le masquage de `repeatCount`. Ce n'est
                    // pas une perte : quel champ pilote quel autre est un fait sur le
                    // FORMULAIRE, pas sur la page qui le dessine — c'est la même règle
                    // que pour `row_attr`, écrite dans le thème admin.
                    'attr' => [
                        'data-conditional-field-target' => 'source',
                        'data-action' => 'change->conditional-field#apply',
                    ],
                    // ⚠️ No blank option: `NONE` already IS "once only", and a blank
                    // above it would be a second way to say the same thing — and the
                    // one the browser preselects.
                    'placeholder' => false,
                    'data' => EventSeries::NONE,
                    'choices' => [
                        'Une seule fois' => EventSeries::NONE,
                        'Toutes les semaines' => EventSeries::EVERY_WEEK,
                        'Une semaine sur deux' => EventSeries::EVERY_TWO_WEEKS,
                    ],
                    // 🔴 Says what this does NOT create. The events are independent
                    // rows from the moment they exist: each one moves, fills up or is
                    // called off on its own, and editing this one will not touch them.
                    'help' => 'Crée plusieurs événements d\'un coup. Ils sont ensuite indépendants : déplacer ou annuler l\'un ne touche pas les autres.',
                ])
                ->add('repeatCount', IntegerType::class, [
                    'label' => 'Nombre de séances',
                    'mapped' => false,
                    'required' => false,
                    'data' => 1,
                    // ⚠️ Sur la RANGÉE, pas sur le widget : c'est l'étiquette et
                    // l'aide qui doivent disparaître avec le champ. Le thème recopie
                    // les clés de `row_attr` autres que `class` sur l'enveloppe.
                    'row_attr' => [
                        'data-conditional-field-target' => 'dependent',
                        'data-conditional-field-show-when' => 'week two_weeks',
                    ],
                    // ⚠️ The old help said "no effect if the event does not repeat" —
                    // a sentence explaining why a control is inert. The control is now
                    // simply not drawn until it can do something.
                    'help' => 'Jusqu\'à 12 séances, générées d\'un coup.',
                    'constraints' => [
                        new Assert\Range(
                            min: 1,
                            max: EventSeries::MAX_OCCURRENCES,
                            notInRangeMessage: 'Entre {{ min }} et {{ max }} séances.',
                        ),
                    ],
                ]);
        }
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Event::class,
            'allow_repeat' => false,
            'constraints' => [
                new Assert\Callback([$this, 'validateDates']),
            ],
        ]);
        $resolver->setAllowedTypes('allow_repeat', 'bool');
    }

    public function validateDates(?Event $event, \Symfony\Component\Validator\Context\ExecutionContextInterface $context): void
    {
        if ($event === null) {
            return;
        }
        $start = $event->getDateDebut();
        $end = $event->getDateFin();
        if ($event->getLocationMode() === Event::LOCATION_OFFSITE && ($event->getAddress() ?? '') === '') {
            $context->buildViolation('Indiquez l\'adresse, ou choisissez « Au fablab ».')
                ->atPath('address')
                ->addViolation();
        }

        if ($start !== null && $end !== null && $end < $start) {
            $context->buildViolation('La date de fin doit être après la date de début.')
                ->atPath('dateFin')
                ->addViolation();
        }
    }
}
