<?php

namespace App\Form;

use App\Entity\Creation;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\FileType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class CreationUserType extends AbstractType
{
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
                'label' => 'form.short_description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('imageUpload', FileType::class, [
                'label' => 'creation_user_form.image_upload',
                'mapped' => false,
                'required' => true,
                'help' => 'creation_user_form.help_image_upload',
                'attr' => ['accept' => 'image/png,image/jpeg,image/webp'],
                'constraints' => [
                    new Assert\NotBlank(message: 'Ajoute une image de ta création.'),
                    new Assert\File(
                        maxSize: '5M',
                        maxSizeMessage: 'L’image est trop lourde. Taille maximum autorisée : 5 Mo.',
                        uploadIniSizeErrorMessage: 'L’image dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                        mimeTypes: ['image/png', 'image/jpeg', 'image/webp'],
                        mimeTypesMessage: 'Choisis une image PNG, JPG, JPEG ou WEBP.',
                    ),
                ],
            ])
            ->add('printDurationFormatted', TextType::class, [
                'label' => 'creation_user_form.print_duration',
                'mapped' => false,
                'required' => false,
                'help' => 'creation_user_form.help_print_duration',
                'attr' => [
                    'placeholder' => '01:30',
                    'inputmode' => 'numeric',
                    'pattern' => '\d{1,3}:[0-5]\d',
                ],
                'constraints' => [
                    new Assert\Regex(
                        pattern: '/^\d{1,3}:[0-5]\d$/',
                        message: 'Utilise le format HH:MM, par exemple 01:30.',
                        match: true,
                        normalizer: 'trim',
                    ),
                ],
            ])
            ->add('fileUpload', FileType::class, [
                'label' => 'creation_user_form.file_upload',
                'mapped' => false,
                'required' => false,
                'help' => 'creation_user_form.help_file_upload',
                'attr' => ['accept' => '.stl,.3mf,.obj,.step,.pdf,.zip,.afdesign'],
                'constraints' => [new Assert\File(
                    maxSize: '20M',
                    maxSizeMessage: 'Le fichier projet est trop lourd. Taille maximum autorisée : 20 Mo.',
                    uploadIniSizeErrorMessage: 'Le fichier projet dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                )],
            ])
            ->add('externalUrl', UrlType::class, [
                'label' => 'creation_user_form.external_url',
                'required' => false,
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'Le lien ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('tagsInput', TextType::class, [
                'label' => 'creation_user_form.tags',
                'mapped' => false,
                'required' => false,
                'help' => 'creation_user_form.help_tags',
                'attr' => ['placeholder' => 'impression 3d, déco, cadeau'],
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Les tags ne doivent pas dépasser {{ limit }} caractères.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'creation_user_form.submit']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Creation::class,
        ]);
    }
}
