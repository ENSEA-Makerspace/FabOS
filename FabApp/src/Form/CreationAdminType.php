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
    /**
     * L'ordre et le découpage de l'écran — motif `SECTIONS` (S151, R3), déroulé
     * par `site/_form_sections.html.twig`. Une seule liste, dans le formulaire,
     * au lieu d'une par gabarit qui rendait le même type.
     *
     * ⚠️ Le bloc « image actuelle » du gabarit n'est PAS un champ : il montre ce
     * qui est déjà enregistré, et il reste dans le partiel, au-dessus des sections.
     */
    public const SECTIONS = [
        [
            'title' => 'admin_creation_form.section_project',
            'fields' => ['title', 'description', 'isPublished'],
        ],
        [
            'title' => 'admin_creation_form.section_author',
            'fields' => ['author', 'authorName'],
        ],
        [
            'title' => 'admin_creation_form.section_files',
            'fold' => true,
            'fields' => ['imageUpload', 'fileUpload', 'externalUrl', 'printDurationMinutes'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('title', TextType::class, [
                'label' => 'form.title',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'row_attr' => ['class' => 'full'],
                'label' => 'form.short_description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('author', EntityType::class, [
                'label' => 'admin_creation_form.author',
                'class' => Utilisateur::class,
                'choice_label' => static fn (Utilisateur $user): string => $user->getDisplayName() . ' (' . $user->getEmail() . ')',
                'placeholder' => 'admin_creation_form.ph_author',
                'required' => false,
            ])
            ->add('authorName', TextType::class, [
                'label' => 'admin_creation_form.author_name',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Le nom auteur ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('imageUpload', FileType::class, [
                'label' => 'admin_creation_form.image_upload',
                'mapped' => false,
                'required' => false,
                'help' => 'admin_creation_form.help_image_upload',
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
                'label' => 'admin_creation_form.file_upload',
                'mapped' => false,
                'required' => false,
                'help' => 'admin_creation_form.help_file_upload',
                'attr' => ['accept' => '.stl,.3mf,.obj,.step,.pdf,.zip,.afdesign'],
                'constraints' => [new Assert\File(
                    maxSize: '20M',
                    maxSizeMessage: 'Le fichier projet est trop lourd. Taille maximum autorisée : 20 Mo.',
                    uploadIniSizeErrorMessage: 'Le fichier projet dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                )],
            ])
            ->add('externalUrl', UrlType::class, [
                'label' => 'admin_creation_form.external_url',
                'required' => false,
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'Le lien ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('printDurationMinutes', IntegerType::class, [
                'label' => 'admin_creation_form.print_duration',
                'required' => false,
                'constraints' => [new Assert\PositiveOrZero(message: 'Le temps d’impression doit être positif.')],
            ])
            ->add('isPublished', CheckboxType::class, [
                'row_attr' => ['class' => 'checkbox'],
                'label' => 'admin_creation_form.is_published',
                'required' => false,
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Creation::class,
        ]);
    }
}
