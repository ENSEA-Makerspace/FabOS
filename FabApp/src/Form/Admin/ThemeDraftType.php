<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/themes`, le brouillon de thème (S148, J-22).
 *
 * 🔴 **Les trois refus de `ThemeManager::saveDraft()` étaient des exceptions dont
 * le message partait en flash, suivies d'une redirection : les quatre champs
 * étaient perdus.** Une couleur tapée « 9E1B56 » sans le dièse effaçait donc
 * aussi le nom de l'organisation, le nom du lieu et le fichier de logo. Les mêmes
 * règles sont ici, en contraintes, sur leur champ.
 *
 * ⚠️ **`ThemeManager` garde ses `throw`, délibérément.** C'est lui le point de
 * passage : un import, une commande ou un autre écran doivent buter sur la même
 * règle. Ce que le type ajoute, c'est de la dire AVANT, là où l'opérateur tape.
 *
 * ⚠️ `orgName` et `venueLabel` étaient TRONQUÉS à 80 par `mb_substr()` ; la
 * contrainte les refuse. La borne reste 80 des deux côtés.
 */
final class ThemeDraftType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('orgName', TextType::class, [
                'label' => 'admin_themes.org_name',
                'empty_data' => '',
                'attr' => ['maxlength' => 80],
                'constraints' => [
                    new Assert\NotBlank(message: 'Les deux noms publics sont obligatoires.'),
                    new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('venueLabel', TextType::class, [
                'label' => 'admin_themes.venue_label',
                'empty_data' => '',
                'attr' => ['maxlength' => 80],
                'constraints' => [
                    new Assert\NotBlank(message: 'Les deux noms publics sont obligatoires.'),
                    new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('primaryColor', TextType::class, [
                'label' => 'admin_themes.primary_color',
                'required' => false,
                'attr' => ['maxlength' => 7, 'placeholder' => '#9E1B56'],
                'constraints' => [new Assert\Regex(
                    pattern: '/^#(?:[0-9a-f]{3}|[0-9a-f]{6})$/i',
                    message: 'La couleur doit être un code hexadécimal, par exemple #9E1B56.',
                )],
            ])
            ->add('logoPath', TextType::class, [
                'label' => 'admin_themes.logo_path',
                'required' => false,
                'attr' => ['maxlength' => 255, 'placeholder' => 'logo.svg'],
                'constraints' => [new Assert\Regex(
                    pattern: '/^[A-Za-z0-9._-]+\.(png|jpe?g|webp|svg)$/i',
                    message: 'Le logo doit être un nom de fichier image dans public/images/.',
                )],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null, 'csrf_token_id' => 'admin_themes']);
    }
}
