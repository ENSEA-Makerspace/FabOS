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

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('nom', TextType::class, [
                'label' => 'Nom',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('venue', VenueChoiceType::class)
            ->add('localisation', TextType::class, [
                'label' => 'Localisation',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La localisation ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('manufacturer', TextType::class, ['label' => 'Marque', 'required' => false, 'constraints' => [new Assert\Length(max: 150)]])
            ->add('model', TextType::class, ['label' => 'Modèle', 'required' => false, 'constraints' => [new Assert\Length(max: 150)]])
            ->add('statut', ChoiceType::class, [
                'label' => 'Statut',
                'choices' => array_combine(self::STATUSES, self::STATUSES),
                'invalid_message' => 'Statut invalide.',
                'constraints' => [new Assert\Choice(choices: self::STATUSES, message: 'Statut invalide.')],
            ])
            ->add('granularite', TextType::class, [
                'label' => 'Granularité',
                'required' => false,
                'constraints' => [
                    new Assert\Regex(pattern: '/^\\d*$/', message: 'La granularité doit être un entier positif.'),
                    new Assert\Length(max: 50, maxMessage: 'La granularité ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('limiteReservations', IntegerType::class, [
                'label' => 'Limite réservations',
                'constraints' => [
                    new Assert\PositiveOrZero(message: 'La limite de réservations doit être un entier positif.'),
                    new Assert\LessThanOrEqual(value: 1000, message: 'La limite de réservations est trop élevée.'),
                ],
            ]);

        if ($options['include_machine_token']) {
            $builder->add('machineToken', TextType::class, [
                'label' => 'Token machine',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le token machine est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le token machine ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ]);
        }

        $builder
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
                'attr' => ['list' => 'machine-category-options'],
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'Niveau',
                'mapped' => false,
                'required' => false,
                'data' => $options['level_value'],
                'constraints' => [new Assert\Range(notInRangeMessage: 'Le niveau doit être compris entre {{ min }} et {{ max }}.', min: 1, max: 3)],
            ])
            ->add('icone', TextType::class, [
                'label' => 'Icône',
                'mapped' => false,
                'required' => false,
                'data' => $options['icon_slug'],
                'constraints' => [new Assert\Length(max: 50, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('materiaux', TextareaType::class, [
                'label' => 'Matériaux',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['materials']),
                'help' => 'Un matériau par ligne.',
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La liste des matériaux ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('caracteristiques', TextareaType::class, [
                'label' => 'Caractéristiques',
                'mapped' => false,
                'required' => false,
                'data' => implode("\n", $options['features']),
                'help' => 'Une caractéristique par ligne.',
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La liste des caractéristiques ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'label' => 'Prérequis',
                'mapped' => false,
                'required' => false,
                'data' => $options['requirement_description'],
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('popularite', IntegerType::class, [
                'label' => 'Popularité',
                'mapped' => false,
                'required' => false,
                'data' => $options['popularity'],
                'constraints' => [new Assert\Range(notInRangeMessage: 'La popularité doit être comprise entre {{ min }} et {{ max }}.', min: 0, max: 5)],
            ])
            ->add('photo', TextType::class, [
                'label' => 'Photo',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'La photo ne doit pas dépasser {{ limit }} caractères.')],
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
