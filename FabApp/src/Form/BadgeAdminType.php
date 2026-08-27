<?php

namespace App\Form;

use App\Entity\Badge;
use App\Entity\Institution;
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
                'label' => 'form.name',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'form.description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('icone', TextType::class, [
                'label' => 'form.icon',
                'required' => false,
                'constraints' => [new Assert\Length(max: 255, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ])
            ->add('institutions', EntityType::class, [
                'label' => 'admin_badge_form.institutions',
                'class' => Institution::class,
                'choice_label' => 'nom',
                'multiple' => true,
                'expanded' => true,
                'required' => false,
            ])
            ->add('prerequisiteBadge', EntityType::class, [
                'label' => 'admin_badge_form.prerequisite',
                'class' => Badge::class,
                'choice_label' => 'nom',
                'required' => false,
                'placeholder' => 'common.none',
                'query_builder' => fn (BadgeRepository $repository): QueryBuilder => $repository->createQueryBuilder('b')
                    ->andWhere('b.id != :self')
                    ->setParameter('self', $badge instanceof Badge ? ($badge->getId() ?? 0) : 0)
                    ->orderBy('b.nom', 'ASC'),
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Badge::class,
        ]);
    }
}
