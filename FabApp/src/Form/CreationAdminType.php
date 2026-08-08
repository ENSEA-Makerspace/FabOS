<?php

namespace App\Form;

use App\Entity\Creation;
use App\Entity\Utilisateur;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\FileType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class CreationAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('title', TextType::class, [
                'label' => 'Titre',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description courte',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('author', EntityType::class, [
                'label' => 'Auteur utilisateur',
                'class' => Utilisateur::class,
                'choice_label' => static fn (Utilisateur $user): string => $user->getDisplayName() . ' (' . $user->getEmail() . ')',
                'placeholder' => 'Aucun compte lié',
                'required' => false,
            ])
            ->add('authorName', TextType::class, [
                'label' => 'Nom auteur libre',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Le nom auteur ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('imageUpload', FileType::class, [
                'label' => 'Image de la création',
                'mapped' => false,
                'required' => false,
                'help' => 'Formats acceptés : PNG, JPG, JPEG, WEBP. Taille max : 5 Mo. L’image sera optimisée automatiquement.',
                'attr' => ['accept' => 'image/png,image/jpeg,image/webp'],
                'constraints' => [
                    new Assert\File(
                        maxSize: '5M',
                        maxSizeMessage: 'L’image est trop lourde. Taille maximum autorisée : 5 Mo.',
                        uploadIniSizeErrorMessage: 'L’image dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                        mimeTypes: ['image/png', 'image/jpeg', 'image/webp'],
                        mimeTypesMessage: 'Choisissez une image PNG, JPG, JPEG ou WEBP.',
                    ),
                ],
            ])
            ->add('fileUpload', FileType::class, [
                'label' => 'Fichier projet',
                'mapped' => false,
                'required' => false,
                'help' => 'Formats acceptés : STL, 3MF, OBJ, STEP, PDF, ZIP, AFDESIGN. Taille max : 20 Mo.',
                'attr' => ['accept' => '.stl,.3mf,.obj,.step,.pdf,.zip,.afdesign'],
                'constraints' => [new Assert\File(
                    maxSize: '20M',
                    maxSizeMessage: 'Le fichier projet est trop lourd. Taille maximum autorisée : 20 Mo.',
                    uploadIniSizeErrorMessage: 'Le fichier projet dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                )],
            ])
            ->add('externalUrl', UrlType::class, [
                'label' => 'Lien externe',
                'required' => false,
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'Le lien ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('printDurationMinutes', IntegerType::class, [
                'label' => 'Temps d’impression (minutes)',
                'required' => false,
                'constraints' => [new Assert\PositiveOrZero(message: 'Le temps d’impression doit être positif.')],
            ])
            ->add('isPublished', CheckboxType::class, [
                'label' => 'Publié',
                'required' => false,
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Creation::class,
        ]);
    }
}
