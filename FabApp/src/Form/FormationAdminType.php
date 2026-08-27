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
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`. Une seule liste, dans le formulaire,
     * au lieu d'une par gabarit qui rendait le même type.
     *
     * 🔴 **Ce gabarit dessinait ses rangées à la main** (`form_label` +
     * `form_widget`), donc il sautait le thème : le champ obligatoire n'était
     * marqué nulle part — 1 champ requis, **0 mention « requis »** à l'écran,
     * mesuré avant conversion. La règle 5 de S149 existe depuis, dans le thème.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_formation_form.section_identity',
            'fields' => ['titre', 'description', 'categorie', 'niveau'],
        ],
        [
            'title' => 'admin_formation_form.section_session',
            'fields' => ['duree', 'formateur', 'placesTotales'],
        ],
        [
            'title' => 'admin_formation_form.section_content',
            'fold' => true,
            'fields' => ['objectifs', 'prerequis', 'materielFourni'],
        ],
        [
            'title' => 'admin_formation_form.section_media',
            'fold' => true,
            'fields' => ['image', 'badge'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'form.title',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'form.description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('image', TextType::class, [
                'label' => 'Image',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: "L'image ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('categorie', TextType::class, [
                'label' => 'form.category',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'form.level',
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
                'row_attr' => ['class' => 'full'],
                'label' => 'Objectifs',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les objectifs ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequis', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'form.prerequisites',
                'required' => false,
                'constraints' => [new Assert\Length(max: 3000, maxMessage: 'Les prérequis ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('materielFourni', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
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
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Formation::class,
        ]);
    }
}
