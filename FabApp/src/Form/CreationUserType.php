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
            ->add('imageUpload', FileType::class, [
                'label' => 'Image obligatoire',
                'mapped' => false,
                'required' => true,
                'help' => 'PNG, JPG, JPEG ou WEBP. 5 Mo maximum. L’image sera optimisée automatiquement.',
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
                'label' => 'Temps d’impression',
                'mapped' => false,
                'required' => false,
                'help' => 'Format HH:MM. Exemples : 00:45 pour 45 minutes, 01:30 pour 1h30, 02:05 pour 2h05.',
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
                'label' => 'Fichier projet optionnel',
                'mapped' => false,
                'required' => false,
                'help' => 'STL, 3MF, OBJ, STEP, PDF, ZIP ou AFDESIGN. 20 Mo maximum.',
                'attr' => ['accept' => '.stl,.3mf,.obj,.step,.pdf,.zip,.afdesign'],
                'constraints' => [new Assert\File(
                    maxSize: '20M',
                    maxSizeMessage: 'Le fichier projet est trop lourd. Taille maximum autorisée : 20 Mo.',
                    uploadIniSizeErrorMessage: 'Le fichier projet dépasse la limite PHP du serveur. Mets upload_max_filesize à 20M et post_max_size à 25M.',
                )],
            ])
            ->add('externalUrl', UrlType::class, [
                'label' => 'Lien externe optionnel',
                'required' => false,
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'Le lien ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'Publier ma création']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Creation::class,
        ]);
    }
}
