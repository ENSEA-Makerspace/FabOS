<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\DateTimeType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/staff/acces-exceptionnels`, accorder une dérogation (S148, J-22).
 *
 * 🔴 **C'est le formulaire le plus cher à retaper de l'écran, et il redirigeait.**
 * Sept champs — une personne dans une liste de tout le monde, un type, un
 * identifiant de ressource, deux dates, un plafond, un motif — et « choisissez la
 * personne » repartait en 302 : tout était à refaire. Il se re-rend maintenant.
 *
 * ⚠️ **Les deux dates rendent une CHAÎNE**, comme sur l'éditeur de packages et
 * pour la même raison : le contrôleur les construit dans le fuseau du LABO, et
 * `AccessPass` les relit dans ce fuseau. Laisser Symfony hydrater un objet le
 * ferait en UTC et « valable à partir de 9 h » deviendrait 10 h ou 8 h selon la
 * saison, sans rien à l'écran pour le dire.
 */
final class AccessPassType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('user_id', ChoiceType::class, [
                'label' => 'access_passes.person',
                'choice_translation_domain' => false,
                'choices' => $options['people_choices'],
                'placeholder' => 'access_passes.choose',
                'constraints' => [new Assert\NotBlank(message: 'Choisissez la personne à qui accorder cet accès.')],
            ])
            ->add('reservable_type', ChoiceType::class, [
                'label' => 'access_passes.resource_type',
                'help' => 'access_passes.all_types_help',
                'choices' => $options['type_choices'],
                'placeholder' => 'access_passes.all_types',
                'required' => false,
            ])
            ->add('reservable_id', IntegerType::class, [
                'label' => 'access_passes.resource',
                'help' => 'access_passes.resource_help',
                'required' => false,
                'attr' => ['min' => 1, 'placeholder' => $options['resource_placeholder']],
                'constraints' => [new Assert\Positive(message: 'Cet identifiant doit être un entier positif.')],
            ])
            ->add('valid_from', DateTimeType::class, $this->moment($options) + ['label' => 'access_passes.valid_from'])
            ->add('valid_until', DateTimeType::class, $this->moment($options) + ['label' => 'access_passes.valid_until'])
            ->add('max_uses', IntegerType::class, [
                'label' => 'access_passes.max_uses',
                'required' => false,
                'attr' => ['min' => 1, 'placeholder' => $options['unlimited_placeholder']],
                'constraints' => [new Assert\Positive(message: 'Le nombre d’utilisations doit être un entier positif.')],
            ])
            ->add('reason', TextType::class, [
                'label' => 'access_passes.reason',
                'required' => false,
                'attr' => ['maxlength' => 255, 'placeholder' => $options['reason_placeholder']],
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ]);
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
            ->setDefaults([
                'data_class' => null,
                'csrf_token_id' => 'staff_access_passes',
                'resource_placeholder' => '',
                'unlimited_placeholder' => '',
                'reason_placeholder' => '',
            ])
            ->setRequired(['people_choices', 'type_choices', 'lab_timezone'])
            ->setAllowedTypes('people_choices', 'array')
            ->setAllowedTypes('type_choices', 'array')
            ->setAllowedTypes('lab_timezone', 'string');
    }
}
