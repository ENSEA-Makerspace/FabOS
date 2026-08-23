<?php

namespace App\Form\Settings;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;

/**
 * `/admin/settings`, carte « Avancé » (S147, J-22).
 *
 * 🔴 **La règle métier ne descend PAS ici, et c'est délibéré.** Activer
 * l'application des droits d'usage n'est refusé que si l'installation n'est pas
 * prête *ou* si la case de confirmation n'est pas cochée — deux conditions qui
 * dépendent de l'état actuel du réglage et d'un décompte calculé dans le
 * contrôleur. Une contrainte de formulaire qui aurait besoin des deux serait une
 * copie de la décision, et deux copies divergent. Le type déclare les champs ;
 * le contrôleur garde le verdict.
 *
 * ⚠️ `usage_rights_confirm_enable` n'existe à l'écran que lorsque le réglage est
 * ENCORE désactivé — confirmer ce qui est déjà en place n'a pas de sens. Le champ
 * est toujours construit (un formulaire dont la forme change selon l'état est un
 * formulaire dont la validation change avec) et c'est le gabarit qui le montre ou
 * non, exactement comme avant.
 */
final class AdvancedSettingsType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('development_mode', CheckboxType::class, [
                'label' => 'admin_development.enabled_label',
                'help' => 'admin_development.enabled_hint',
                'required' => false,
            ])
            ->add('usage_rights_enforced', CheckboxType::class, [
                'label' => 'usage_rights.settings_enabled',
                'help' => 'usage_rights.settings_hint',
                'required' => false,
            ])
            ->add('usage_rights_confirm_enable', CheckboxType::class, [
                'label' => 'usage_rights.settings_confirm_enable',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'settings_advanced',
        ]);
    }
}
