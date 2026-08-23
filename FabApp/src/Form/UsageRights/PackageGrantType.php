<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\TimeType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Un grant v2, avec sa première plage horaire (S147, J-22).
 *
 * ⚠️ **Neuf champs qui posent UNE phrase** : « ce package laisse *utiliser* les
 * *imprimantes 3D* du *lieu X*, le *jeudi* de *14 h* à *18 h* ». Le formulaire
 * les garde ensemble parce que c'est ainsi qu'un opérateur y pense — la carte
 * dit la phrase, pas les colonnes d'une table.
 *
 * 🔴 **La plage est FACULTATIVE et son absence n'est pas neutre** : un grant sans
 * plage ouvre toute la semaine. C'est pour ça que les trois champs de temps sont
 * `required => false` et que rien ici ne les impose — mais aussi pourquoi le
 * contrôleur, s'il en reçoit une incomplète, garde le grant et signale la plage
 * plutôt que de tout refuser : le grant est la partie qui compte.
 *
 * ⚠️ Les heures rendent une CHAÎNE `H:i`, comme aujourd'hui : `GrantWindow::
 * fromClock()` les lit ainsi, et c'est lui qui sait que minuit en fin de plage
 * vaut 1440 et non 0.
 */
final class PackageGrantType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('feature', ChoiceType::class, [
                'label' => 'usage_rights.grant_feature',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['feature_choices'],
                'constraints' => [new Assert\NotBlank(message: 'Choisissez une fonctionnalité.')],
            ])
            ->add('grant_action', ChoiceType::class, [
                'label' => 'usage_rights.grant_action',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['action_choices'],
            ])
            ->add('venue_id', ChoiceType::class, [
                'label' => 'venue_context.menu',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['venue_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_venue',
            ])
            ->add('section', TextType::class, [
                'label' => 'usage_rights.grant_section',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('reservable', ChoiceType::class, [
                'label' => 'usage_rights.grant_resource',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['resource_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_resource',
            ])
            ->add('category_label', ChoiceType::class, [
                'label' => 'usage_rights.grant_category',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['category_choices'],
                'required' => false,
                'placeholder' => 'usage_rights.grant_any_category',
            ])
            ->add('day_of_week', ChoiceType::class, [
                'label' => 'usage_rights.grant_window',
                'row_attr' => ['class' => 'afp-select'],
                'label_attr' => ['class' => 'afp-k'],
                'choice_translation_domain' => false,
                'choices' => $options['day_choices'],
                // ⚠️ `placeholder => false` et non l'oubli du défaut : sans lui,
                // un `ChoiceType` non requis s'ajoute une option VIDE en tête, et
                // c'est elle qui serait présélectionnée à la place de « à toute
                // heure ». La liste porte déjà son propre « tout ».
                'placeholder' => false,
                'required' => false,
            ])
            ->add('start_time', TimeType::class, $this->clock() + ['label' => 'usage_rights.window_from'])
            ->add('end_time', TimeType::class, $this->clock() + ['label' => 'usage_rights.window_to']);
    }

    /** @return array<string, mixed> */
    private function clock(): array
    {
        return [
            'widget' => 'single_text', 'html5' => true, 'input' => 'string', 'input_format' => 'H:i', 'required' => false,
            'row_attr' => ['class' => 'afp-select'], 'label_attr' => ['class' => 'afp-k'],
        ];
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['feature_choices', 'action_choices', 'venue_choices', 'resource_choices', 'category_choices', 'day_choices', 'package_key'])
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_grants_' . $o['package_key']);
    }
}
