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
            'title' => 'admin_machine_form.section_identity',
            'fields' => ['nom', 'categorie', 'niveau', 'description'],
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
            'title' => 'admin_machine_form.section_booking',
            'fields' => ['statut', 'granularite', 'limiteReservations', 'machineToken'],
        ],
        [
            // Repli : rien ici ne décide qui peut réserver, tout y décrit la
            // fiche publique. C'est une deuxième passe, pas la création.
            'title' => 'admin_machine_form.section_public',
            'fold' => true,
            'fields' => ['manufacturer', 'model', 'materiaux', 'caracteristiques', 'prerequis'],
        ],
        [
            'title' => 'admin_machine_form.section_media',
            'fold' => true,
            'fields' => ['photo', 'icone', 'popularite'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
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
                'label' => 'Nom',
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
                'label' => 'Catégorie',
                'mapped' => false,
                'required' => false,
                'data' => $options['category_label'],
                'help' => 'admin_machine_form.help_categorie',
                'attr' => ['list' => 'machine-category-options'],
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'Niveau',
                'mapped' => false,
                'required' => false,
                'data' => $options['level_value'],
                'help' => 'admin_machine_form.help_niveau',
                'constraints' => [new Assert\Range(notInRangeMessage: 'Le niveau doit être compris entre {{ min }} et {{ max }}.', min: 1, max: 3)],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_description',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            // `VenueChoiceType` porte déjà son libellé ET son aide (`venues.help.venue`).
            ->add('venue', VenueChoiceType::class)
            ->add('localisation', TextType::class, [
                'label' => 'Localisation',
                'required' => false,
                'help' => 'admin_machine_form.help_localisation',
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La localisation ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('statut', ChoiceType::class, [
                'label' => 'Statut',
                'choices' => array_combine(self::STATUSES, self::STATUSES),
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_statut',
                'invalid_message' => 'Statut invalide.',
                'constraints' => [new Assert\Choice(choices: self::STATUSES, message: 'Statut invalide.')],
            ])
            ->add('granularite', TextType::class, [
                'label' => 'Granularité',
                'required' => false,
                'help' => 'admin_machine_form.help_granularite',
                'constraints' => [
                    new Assert\Regex(pattern: '/^\\d*$/', message: 'La granularité doit être un entier positif.'),
                    new Assert\Length(max: 50, maxMessage: 'La granularité ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('limiteReservations', IntegerType::class, [
                'label' => 'Limite réservations',
                'help' => 'admin_machine_form.help_limite',
                'constraints' => [
                    new Assert\PositiveOrZero(message: 'La limite de réservations doit être un entier positif.'),
                    new Assert\LessThanOrEqual(value: 1000, message: 'La limite de réservations est trop élevée.'),
                ],
            ]);

        if ($options['include_machine_token']) {
            $builder->add('machineToken', TextType::class, [
                'label' => 'Token machine',
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
                'label' => 'Marque',
                'required' => false,
                'help' => 'admin_machine_form.help_manufacturer',
                'constraints' => [new Assert\Length(max: 150)],
            ])
            ->add('model', TextType::class, ['label' => 'Modèle', 'required' => false, 'constraints' => [new Assert\Length(max: 150)]])
            ->add('materiaux', TextareaType::class, [
                'label' => 'Matériaux',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['materials']),
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_materiaux',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La liste des matériaux ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('caracteristiques', TextareaType::class, [
                'label' => 'Caractéristiques',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['features']),
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_caracteristiques',
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La liste des caractéristiques ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'label' => 'Prérequis',
                'mapped' => false,
                'required' => false,
                'data' => $options['requirement_description'],
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_prerequis',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('photo', TextType::class, [
                'label' => 'Photo',
                'required' => false,
                'help' => 'admin_machine_form.help_photo',
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La photo ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('icone', TextType::class, [
                'label' => 'Icône',
                'mapped' => false,
                'required' => false,
                'data' => $options['icon_slug'],
                'help' => 'admin_machine_form.help_icone',
                'constraints' => [new Assert\Length(max: 50, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('popularite', IntegerType::class, [
                'label' => 'Popularité',
                'mapped' => false,
                'required' => false,
                'data' => $options['popularity'],
                'row_attr' => ['class' => 'full'],
                'help' => 'admin_machine_form.help_popularite',
                'constraints' => [new Assert\Range(notInRangeMessage: 'La popularité doit être comprise entre {{ min }} et {{ max }}.', min: 0, max: 5)],
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
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
        ]);
    }
}
