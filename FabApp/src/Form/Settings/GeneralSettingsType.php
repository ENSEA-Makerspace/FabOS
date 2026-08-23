<?php

namespace App\Form\Settings;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * `/admin/settings`, carte « Général » (S147, J-22).
 *
 * ⚠️ **Les libellés et les aides sont des CLÉS, pas du français en dur.** Les
 * `FormType` existants de cette base écrivent leurs libellés directement en
 * français, ce qui les rend intraduisibles ; le gabarit que ce type remplace
 * utilisait déjà les clés `admin_settings.*`, présentes dans les cinq langues.
 * Les reprendre telles quelles était la seule façon de convertir sans régresser.
 *
 * 🔴 **Un `placeholder` n'est PAS traduit par Symfony** — c'est un attribut HTML,
 * pas une option de formulaire. Le laisser sous forme de clé aurait affiché
 * « admin_settings.org_placeholder » dans le champ. D'où le traducteur injecté.
 *
 * ⚠️ **`csrf_token_id` explicite, et ce n'est pas de la plomberie.** Le défaut de
 * l'application est `submit`, qui figure dans `stateless_token_ids` : son jeton
 * vit dans un cookie posé sur la réponse, qu'une requête construite depuis la
 * console ne reçoit jamais — un tel formulaire est donc **intestable par sonde**.
 * Un identifiant propre à la carte retombe sur le jeton de SESSION, qui se teste,
 * et il est de surcroît plus étroit qu'un jeton partagé par tout le site.
 */
final class GeneralSettingsType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $length = static fn (int $max): array => [new Assert\Length(max: $max, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')];

        $builder
            ->add('org_name', TextType::class, [
                'label' => 'admin_settings.org_name',
                'help' => 'admin_settings.org_name_help',
                'required' => false,
                'empty_data' => '',
                'attr' => ['maxlength' => 80, 'placeholder' => $this->translator->trans('admin_settings.org_placeholder')],
                'constraints' => $length(80),
            ])
            ->add('venue_label', TextType::class, [
                'label' => 'admin_settings.venue_name',
                'help' => 'admin_settings.venue_name_help',
                'required' => false,
                'empty_data' => '',
                'attr' => ['maxlength' => 80, 'placeholder' => $this->translator->trans('admin_settings.venue_placeholder')],
                'constraints' => $length(80),
            ])
            ->add('lab_pages_nav_label', TextType::class, [
                'label' => 'admin_settings.lab_menu_label',
                'help' => 'admin_settings.lab_menu_help',
                'required' => false,
                'empty_data' => '',
                'attr' => ['maxlength' => 60, 'placeholder' => $this->translator->trans('admin_settings.lab_menu_placeholder')],
                'constraints' => $length(60),
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'settings_general',
        ]);
    }
}
