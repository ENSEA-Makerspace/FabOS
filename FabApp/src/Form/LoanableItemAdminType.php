<?php

namespace App\Form;

use App\Entity\LoanableItem;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class LoanableItemAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'Nom',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 150),
                ],
            ])
            ->add('category', TextType::class, [
                'label' => 'Catégorie (ex : outil, kit, équipement)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 80)],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            ->add('quantity', IntegerType::class, [
                'label' => 'Quantité (unités possédées)',
                'empty_data' => '1',
                'constraints' => [new Assert\PositiveOrZero(message: 'La quantité doit être positive.')],
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
            ->add('storageLocation', TextType::class, [
                'label' => 'Emplacement de stockage',
                'required' => false,
                'constraints' => [new Assert\Length(max: 180)],
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => LoanableItem::class]);
    }
}
