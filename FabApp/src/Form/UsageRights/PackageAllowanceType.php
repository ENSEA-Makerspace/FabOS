<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\NumberType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Une allocation : « 6 heures de machine par semaine » (S147, J-22).
 *
 * ⚠️ **Deux champs de quantité pour une seule question, et c'est l'unité qui
 * décide lequel compte.** En heures, le contrôleur lit `allowance_hours` et le
 * convertit en minutes ; en séances, il lit `allowance_count`. Les deux restent
 * facultatifs ici parce que le champ inutile est masqué à l'écran — c'est le
 * contrôleur qui sait lequel il vient de demander, et lui seul.
 * 🔴 Ne pas « simplifier » en un champ unique sans traiter la conversion : une
 * allocation stockée en minutes et lue en séances est un quota faux et muet.
 */
final class PackageAllowanceType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('allowance_feature', ChoiceType::class, [
                'label' => 'usage_rights.allowance_feature',
                'choices' => $options['feature_choices'],
            ])
            ->add('allowance_reservable', ChoiceType::class, [
                'label' => 'usage_rights.allowance_reservable',
                'choices' => $options['reservable_choices'],
                'required' => false,
            ])
            ->add('allowance_unit', ChoiceType::class, [
                'label' => 'usage_rights.allowance_unit',
                'choices' => $options['unit_choices'],
            ])
            ->add('allowance_hours', NumberType::class, [
                'label' => 'usage_rights.allowance_hours',
                'required' => false,
                'scale' => 2,
                'attr' => ['min' => 0, 'step' => '0.25'],
                'constraints' => [new Assert\PositiveOrZero(message: 'La quantité doit être positive.')],
            ])
            ->add('allowance_count', NumberType::class, [
                'label' => 'usage_rights.allowance_count',
                'required' => false,
                'attr' => ['min' => 0, 'step' => '1'],
                'constraints' => [new Assert\PositiveOrZero(message: 'La quantité doit être positive.')],
            ])
            ->add('allowance_period', ChoiceType::class, [
                'label' => 'usage_rights.allowance_period',
                'choices' => $options['period_choices'],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['feature_choices', 'reservable_choices', 'unit_choices', 'period_choices', 'package_key'])
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_allowances_' . $o['package_key']);
    }
}
