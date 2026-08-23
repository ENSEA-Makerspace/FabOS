<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/network`, « initialiser cette instance » (S148, J-22).
 *
 * ⚠️ **Les trois formulaires de l'écran sont convertis, pas seulement celui que
 * le décompte de la revue avait retenu.** En laisser deux écrits à la main aurait
 * donné une page à deux dessins — exactement le défaut que J-22 range.
 */
final class NetworkIdentityType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'network.name',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: 'Le nom est obligatoire.')],
            ])
            ->add('origin', UrlType::class, [
                'label' => 'network.origin',
                'empty_data' => '',
                'attr' => ['placeholder' => 'https://fab.example.org'],
                'constraints' => [
                    new Assert\NotBlank(message: "L'origine est obligatoire."),
                    new Assert\Url(message: "Cette adresse n'est pas une URL valide."),
                ],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null, 'csrf_token_id' => 'network_settings']);
    }
}
