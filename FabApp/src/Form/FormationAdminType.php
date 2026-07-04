<?php

namespace App\Form;

use App\Entity\Badge;
use App\Entity\Formation;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class FormationAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'Titre',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('image', TextType::class, [
                'label' => 'Image',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: "L'image ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('categorie', TextType::class, [
                'label' => 'Catégorie',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'Niveau',
                'required' => false,
                'constraints' => [new Assert\Range(notInRangeMessage: 'Le niveau doit être compris entre {{ min }} et {{ max }}.', min: 1, max: 3)],
            ])
            ->add('duree', TextType::class, [
                'label' => 'Durée',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La durée ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('formateur', TextType::class, [
                'label' => 'Formateur',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Le formateur ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('placesTotales', IntegerType::class, [
                'label' => 'Places totales',
                'required' => false,
                'constraints' => [
                    new Assert\PositiveOrZero(message: 'Le nombre de places doit être un entier positif.'),
                    new Assert\LessThanOrEqual(value: 1000, message: 'Le nombre de places est trop élevé.'),
                ],
            ])
            ->add('objectifs', TextareaType::class, [
                'label' => 'Objectifs',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les objectifs ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'label' => 'Prérequis',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('materielFourni', TextareaType::class, [
                'label' => 'Matériel fourni',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Le matériel fourni ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('badge', EntityType::class, [
                'label' => 'Badge associé',
                'class' => Badge::class,
                'choice_label' => static fn (Badge $badge): string => sprintf('%s (#%d)', $badge->getNom(), $badge->getId()),
                'placeholder' => 'Aucun badge',
                'required' => false,
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Formation::class,
        ]);
    }
}
