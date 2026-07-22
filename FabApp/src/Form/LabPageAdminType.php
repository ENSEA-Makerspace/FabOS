<?php

namespace App\Form;

use App\Entity\LabPage;
use App\Repository\LabPageRepository;
use Doctrine\ORM\QueryBuilder;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class LabPageAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $page = $options['data'] ?? null;

        $builder
            ->add('titre', TextType::class, [
                'label' => 'Titre',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('contenu', TextareaType::class, [
                'label' => 'Contenu',
                'required' => false,
            ])
            ->add('parentPage', EntityType::class, [
                'label' => 'Page parente (2 niveaux maximum)',
                'class' => LabPage::class,
                'choice_label' => 'titre',
                'required' => false,
                'placeholder' => 'Aucune (page de premier niveau)',
                // Only top-level pages can be a parent, and a page can't be its own parent:
                // this keeps the hierarchy fixed at exactly 2 levels.
                'query_builder' => fn (LabPageRepository $repository): QueryBuilder => $repository->createQueryBuilder('p')
                    ->andWhere('p.parentPage IS NULL')
                    ->andWhere('p.id != :self')
                    ->setParameter('self', $page instanceof LabPage ? ($page->getId() ?? 0) : 0)
                    ->orderBy('p.titre', 'ASC'),
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => LabPage::class,
        ]);
    }
}
