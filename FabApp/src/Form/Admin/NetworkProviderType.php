<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/network`, un fournisseur OIDC (S148, J-22).
 *
 * ⚠️ **Les cinq champs étaient produits par une boucle Twig sur une liste de
 * noms**, ce qui les rendait tous `required` et tous en texte libre — y compris
 * l'émetteur, qui est une URL. Les écrire un par un ici les rend au type qu'ils
 * ont réellement.
 *
 * 🔴 `secretEnv` est le NOM d'une variable d'environnement, jamais le secret
 * lui-même. La borne de longueur et le libellé le disent ; rien ici ne doit
 * inviter à coller une valeur.
 */
final class NetworkProviderType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('key', TextType::class, [
                'label' => 'network.key',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Cette clé est obligatoire.'),
                    new Assert\Length(max: 64, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('label', TextType::class, [
                'label' => 'network.label',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('issuer', UrlType::class, [
                'label' => 'network.issuer',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: "L'émetteur est obligatoire."),
                    new Assert\Url(message: "Cette adresse n'est pas une URL valide."),
                ],
            ])
            ->add('clientId', TextType::class, [
                'label' => 'network.clientId',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: "L'identifiant client est obligatoire.")],
            ])
            ->add('secretEnv', TextType::class, [
                'label' => 'network.secretEnv',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom de la variable est obligatoire.'),
                    new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('enabled', CheckboxType::class, [
                'label' => 'network.enabled',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null, 'csrf_token_id' => 'network_settings']);
    }
}
