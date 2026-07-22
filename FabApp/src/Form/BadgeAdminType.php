<?php

namespace App\Form;

use App\Entity\Badge;
use App\Repository\BadgeRepository;
use Doctrine\ORM\QueryBuilder;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class BadgeAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $badge = $options['data'] ?? null;

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
            ->add('icone', TextType::class, [
                'label' => 'Icône',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('whereRecognized', TextType::class, [
                'label' => 'Reconnu à/par',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('prerequisiteBadge', EntityType::class, [
                'label' => 'Badge prérequis (ex : lvl1 pour un badge lvl2)',
                'class' => Badge::class,
                'choice_label' => 'nom',
                'required' => false,
                'placeholder' => 'Aucun',
                'query_builder' => fn (BadgeRepository $repository): QueryBuilder => $repository->createQueryBuilder('b')
                    ->andWhere('b.id != :self')
                    ->setParameter('self', $badge instanceof Badge ? ($badge->getId() ?? 0) : 0)
                    ->orderBy('b.nom', 'ASC'),
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Badge::class,
        ]);
    }
}
