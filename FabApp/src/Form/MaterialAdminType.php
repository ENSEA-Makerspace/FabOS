<?php

namespace App\Form;

use App\Entity\Machine;
use App\Entity\Material;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class MaterialAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'Nom',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('category', TextType::class, [
                'label' => 'Catégorie (ex : filament, plaque, résine)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 80)],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('imageUrl', UrlType::class, [
                'label' => 'URL de l’image (optionnel)',
                'required' => false,
                'default_protocol' => 'https',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('icon', TextType::class, [
                'label' => 'Icône emoji (si pas d’image)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 16)],
            ])
            ->add('specs', TextareaType::class, [
                'label' => 'Spécifications (une par ligne, « clé : valeur »)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            ->add('storageLocation', TextType::class, [
                'label' => 'Emplacement de stockage',
                'required' => false,
                'constraints' => [new Assert\Length(max: 180)],
            ])
            ->add('purchaseUrl', UrlType::class, [
                'label' => 'Lien d’achat (où acheter)',
                'required' => false,
                'default_protocol' => 'https',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('color', TextType::class, [
                'label' => 'Couleur (optionnel)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 60)],
            ])
            ->add('machines', EntityType::class, [
                'label' => 'Machines qui acceptent ce matériau',
                'class' => Machine::class,
                'choice_label' => 'nom',
                'multiple' => true,
                'expanded' => true,
                'required' => false,
                'by_reference' => false,
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('machine')->orderBy('machine.nom', 'ASC'),
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Material::class,
        ]);
    }
}
