<?php

namespace App\Form;

use App\Entity\Machine;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class MachineAdminType extends AbstractType
{
    private const STATUSES = ['idle', 'active', 'maintenance', 'unavailable', 'disponible'];

    public function __construct(private readonly Suggest $suggest)
    {
    }

    /**
     * **Le découpage de l'écran, décidé ici et pas dans les gabarits (S150).**
     *
     * Dix-neuf champs posés à plat, c'est un mur : rien ne dit lesquels décident
     * de la réservation et lesquels ne décorent que la fiche publique. Chaque
     * entrée est un groupe — un titre, puis ses champs dans l'ordre — et
     * `fold: true` met le groupe derrière un `<details>` (règle 1 de
     * `docs/FORM-DESIGN.md` : l'écran s'ouvre sur son cas courant).
     *
     * ⚠️ **C'est ici et pas dans les gabarits parce qu'ils sont DEUX** —
     * `admin-machine-new` et `admin-machine-edit` rendent le même type. Chacun
     * écrivait sa propre liste de champs, et les deux listes avaient déjà
     * divergé du `FormType` : 🔴 `manufacturer` et `model` n'étaient dans
     * aucune des deux, donc `form_end()` les rendait par `form_rest()`, sans
     * thème, APRÈS le bouton « Enregistrer ». Deux champs qui alimentent tout
     * l'écran « Modèles et marques » traînaient sous les actions du formulaire.
     *
     * ⚠️ **Un champ absent de SECTIONS n'est pas caché, il retombe dans
     * `form_rest()`** — au même endroit disgracieux. Ajouter un `->add()` sans
     * l'ajouter ici, c'est reproduire exactement le défaut ci-dessus.
     *
     * ⚠️ **`machineToken` n'existe que sur l'écran de création.** Le gabarit
     * teste la présence de chaque nom avant de le rendre ; c'est ce qui permet
     * à une seule liste de servir les deux écrans.
     */
    public const SECTIONS = [
        [
            // ⚠️ 0.7.0 — la première section tient en deux questions. Les badges
            // (« qui peut la réserver ? ») viennent juste après, dans le gabarit :
            // ils ne sont pas un champ du `FormType`.
            'title' => 'admin_machine_form.section_identity',
            'fields' => ['nom', 'categorie'],
        ],
        [
            // ⚠️ `venue` et `localisation` sont deux questions différentes et
            // voisines exprès : le lieu est le SITE (ce sur quoi les listes
            // filtrent), la localisation est l'endroit DEDANS — la salle,
            // l'établi. Elles vont ensemble, donc elles sont côte à côte
            // (règle 6) ; aucune ne remplace l'autre.
            'title' => 'admin_machine_form.section_place',
            'fields' => ['venue', 'localisation'],
        ],
        [
            // ⚠️ N'existe qu'à l'ÉDITION : une machine se crée « disponible », sans
            // limite. `_form_sections` ne rend pas une section dont aucun champ
            // n'existe.
            'title' => 'admin_machine_form.section_booking',
            'fields' => ['statut', 'limiteReservations'],
        ],
        [
            'title' => 'admin_machine_form.section_more',
            'fold' => true,
            'fields' => ['niveau', 'granularite', 'machineToken', 'manufacturer', 'model', 'description'],
        ],
        [
            // Repli : rien ici ne décide qui peut réserver, tout y décrit la
            // fiche publique. C'est une deuxième passe, pas la création.
            'title' => 'admin_machine_form.section_public',
            'fold' => true,
            'fields' => ['materiaux', 'caracteristiques', 'prerequis', 'photo', 'icone', 'popularite'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        // 0.7.0 — ce que le lab sait déjà (`MachineCreationHints::all()`), posé sur
        // les champs par `Suggest` pour le contrôleur Stimulus du même nom : des tuiles à cliquer au
        // lieu d'une case vide. Sans JavaScript, les champs restent ce qu'ils étaient.
        $hints = $options['hints'];
        $creation = $options['include_machine_token'];

        // ⚠️ **L'ordre des `->add()` suit SECTIONS**, pour que l'ordre de
        // tabulation, l'ordre de `form_rest()` et l'ordre à l'écran ne soient
        // pas trois ordres différents.
        //
        // ⚠️ **`row_attr: {class: 'full'}` est la SEULE façon de décider de la
        // largeur.** Le thème le recopie sur l'enveloppe ; `.form-field.full`
        // prend les deux colonnes de la grille. Règle 6 : deux champs côte à
        // côte seulement quand ils vont ensemble. Les paires voulues ici sont
        // catégorie/niveau, lieu/localisation, granularité/limite,
        // marque/modèle et photo/icône — tout le reste est pleine largeur.
        $builder
            ->add('nom', TextType::class, [
                'label' => 'form.name',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            // ⚠️ Still free text, deliberately (S133). The category catalogue is
            // now real, but `MACHINE.categoryLabel` remains the stored value and a
            // `ChoiceType` here would make an existing machine unsavable the day
            // its category is archived. The `list` attribute offers the catalogue
            // without refusing anything outside it; the categories screen shows
            // whatever gets typed as "not adopted" and lets it be adopted.
            ->add('categorie', TextType::class, [
                'label' => 'form.category',
                'mapped' => false,
                'required' => false,
                'data' => $options['category_label'],
                'help' => 'admin_machine_form.help_categorie',
                'row_attr' => ['class' => 'full'],
                'attr' => ['list' => 'machine-category-options'] + $this->suggest->one($hints['categories'] ?? [], true, [
                    'data-suggest-checks-value' => json_encode($hints['badgesByCategory'] ?? new \stdClass()),
                    'data-suggest-checks-name-value' => 'requiredBadges[]',
                ]),
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', ChoiceType::class, [
                'label' => 'form.level',
                'mapped' => false,
                'required' => false,
                'placeholder' => '—',
                'choices' => ['1' => 1, '2' => 2, '3' => 3],
                'choice_translation_domain' => false,
                'data' => $options['level_value'],
                'help' => 'admin_machine_form.help_niveau',
                'invalid_message' => 'Le niveau doit être compris entre 1 et 3.',
            ])
            ->add('description', TextareaType::class, [
                'label' => 'form.description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_description',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            // `VenueChoiceType` porte déjà son libellé ET son aide (`venues.help.venue`).
            ->add('venue', VenueChoiceType::class)
            ->add('localisation', TextType::class, [
                'label' => 'form.location',
                'required' => false,
                'help' => 'admin_machine_form.help_localisation',
                'attr' => $this->suggest->one($hints['locations'] ?? []),
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La localisation ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('granularite', TextType::class, [
                'label' => 'admin_machine_form.granularite',
                'required' => false,
                'help' => 'admin_machine_form.help_granularite',
                'attr' => ['list' => 'machine-slot-options'],
                'constraints' => [
                    new Assert\Regex(pattern: '/^\\d*$/', message: 'La granularité doit être un entier positif.'),
                    new Assert\Length(max: 50, maxMessage: 'La granularité ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ]);

        if (!$creation) {
            $builder
                ->add('statut', ChoiceType::class, [
                    'label' => 'form.status',
                    // 0.7.1 — des mots, plus les valeurs brutes (« idle »).
                    'choices' => array_combine(array_map(static fn (string $s): string => 'admin_machine_form.statut_' . $s, self::STATUSES), self::STATUSES),
                    'row_attr' => ['class' => 'full'],
                    'help' => 'admin_machine_form.help_statut',
                    'invalid_message' => 'Statut invalide.',
                    'constraints' => [new Assert\Choice(choices: self::STATUSES, message: 'Statut invalide.')],
                ])
                ->add('limiteReservations', IntegerType::class, [
                    'label' => 'admin_machine_form.limite_reservations',
                    'help' => 'admin_machine_form.help_limite',
                    'constraints' => [
                        new Assert\PositiveOrZero(message: 'La limite de réservations doit être un entier positif.'),
                        new Assert\LessThanOrEqual(value: 1000, message: 'La limite de réservations est trop élevée.'),
                    ],
                ]);
        }

        if ($options['include_machine_token']) {
            $builder->add('machineToken', TextType::class, [
                'label' => 'admin_machine_form.machine_token',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_token',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le token machine est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le token machine ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ]);
        }

        $builder
            // 🔴 Ces deux champs n'étaient rendus par AUCUN des deux gabarits.
            // Ils ne sont pas décoratifs : `AdminController::machineModels()`
            // trie et regroupe tout l'écran « Modèles et marques » dessus.
            ->add('manufacturer', TextType::class, [
                'label' => 'admin_machine_form.manufacturer',
                'required' => false,
                'help' => 'admin_machine_form.help_manufacturer',
                'attr' => ['list' => 'machine-manufacturer-options'],
                'constraints' => [new Assert\Length(max: 150)],
            ])
            ->add('model', TextType::class, ['label' => 'admin_machine_form.model', 'required' => false, 'attr' => ['list' => 'machine-model-options'], 'constraints' => [new Assert\Length(max: 150)]])
            ->add('materiaux', TextareaType::class, [
                'label' => 'admin_machine_form.materiaux',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['materials']),
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_materiaux',
                'attr' => $this->suggest->many($hints['materials'] ?? []),
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La liste des matériaux ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('caracteristiques', TextareaType::class, [
                'label' => 'admin_machine_form.caracteristiques',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['features']),
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_caracteristiques',
                'attr' => $this->suggest->many($hints['features'] ?? []),
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La liste des caractéristiques ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'label' => 'form.prerequisites',
                'mapped' => false,
                'required' => false,
                'data' => $options['requirement_description'],
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_prerequis',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('photo', TextType::class, [
                'label' => 'admin_machine_form.photo',
                'required' => false,
                'help' => 'admin_machine_form.help_photo',
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La photo ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('icone', TextType::class, [
                'label' => 'form.icon',
                'mapped' => false,
                'required' => false,
                'data' => $options['icon_slug'],
                'help' => 'admin_machine_form.help_icone',
                'constraints' => [new Assert\Length(max: 50, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ]);

        if (!$creation) {
            $builder
                ->add('popularite', IntegerType::class, [
                    'label' => 'admin_machine_form.popularite',
                    'mapped' => false,
                    'required' => false,
                    'data' => $options['popularity'],
                    'row_attr' => ['class' => 'full'],
                    'help' => 'admin_machine_form.help_popularite',
                    'constraints' => [new Assert\Range(notInRangeMessage: 'La popularité doit être comprise entre {{ min }} et {{ max }}.', min: 0, max: 5)],
                ]);
        } else {
            // « Créer, puis une autre identique » : le lab possède souvent la même machine en plusieurs exemplaires.
            $builder->add('saveAgain', SubmitType::class, ['label' => 'admin_machine_new.submit_again']);
        }

        $builder
            // ⚠️ S151 — `common.save` et non « Enregistrer » : ce bouton est le seul
            // de l'écran, et il était le dernier libellé français en dur d'un
            // formulaire qui déroule maintenant ses sections traduites.
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Machine::class,
            'category_label' => null,
            'level_value' => null,
            'icon_slug' => null,
            'materials' => [],
            'features' => [],
            'requirement_description' => null,
            'popularity' => null,
            'include_machine_token' => false,
            'hints' => [],
        ]);
    }
}
