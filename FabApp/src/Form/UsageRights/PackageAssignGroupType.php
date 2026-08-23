<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\DateTimeType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Attribuer un package à un GROUPE (S147, J-22).
 *
 * 🔴 **Les deux dates rendent une CHAÎNE, pas un objet, et c'est délibéré.** Le
 * contrôleur les passe à un helper qui construit la date **dans le fuseau du
 * labo** — `new \DateTimeImmutable($value, $zone)`. Laisser Symfony hydrater un
 * `DateTimeImmutable` l'aurait construit dans le fuseau PHP par défaut (UTC sur
 * cette boîte), et une validité « à partir du 1er mars 00:00 » serait devenue
 * 01:00 ou 23:00 selon la saison, sans rien à l'écran pour le dire. `input` reste
 * `string`, `model_timezone` et `view_timezone` sont posés à la MÊME valeur pour
 * qu'aucun décalage ne soit seulement possible, et le helper garde son travail.
 */
final class PackageAssignGroupType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('group_key', ChoiceType::class, [
                'label' => 'usage_rights.assign_group',
                'choices' => $options['group_choices'],
                'placeholder' => 'usage_rights.assign_pick_group',
                'constraints' => [new Assert\NotBlank(message: 'Choisissez le groupe à qui accorder ce package.')],
            ])
            ->add('valid_from', DateTimeType::class, $this->moment($options) + ['label' => 'usage_rights.valid_from'])
            ->add('valid_until', DateTimeType::class, $this->moment($options) + ['label' => 'usage_rights.valid_until']);
    }

    /** @return array<string, mixed> */
    private function moment(array $options): array
    {
        return [
            'widget' => 'single_text',
            'html5' => true,
            'input' => 'string',
            'input_format' => 'Y-m-d\TH:i',
            'model_timezone' => $options['lab_timezone'],
            'view_timezone' => $options['lab_timezone'],
            'required' => false,
        ];
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['group_choices', 'lab_timezone', 'package_key'])
            ->setAllowedTypes('group_choices', 'array')
            ->setAllowedTypes('lab_timezone', 'string')
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_assign_group_' . $o['package_key']);
    }
}
