<?php

namespace App\Form\Settings;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * `/admin/settings`, carte « Exploitation et sécurité » (S147, J-22).
 *
 * ⚠️ **C'est la carte la moins « formulaire » des cinq**, et il faut le dire :
 * elle mêle deux champs de texte, une matrice de rôles, une case qui déclenche
 * une ACTION plutôt que d'enregistrer un réglage, et un jeton en lecture seule
 * qui n'est pas un champ du tout. Le jeton reste du balisage dans le gabarit ;
 * seuls les quatre vrais champs passent par le type.
 *
 * 🔴 **Un changement visible, assumé** : les rôles s'affichaient « Nom
 * `ROLE_X` » avec l'identifiant technique en `<code>`. Un `ChoiceType` échappe
 * ses libellés, donc ils lisent maintenant « Nom — ROLE_X », en texte. C'est la
 * même information, sans la police à chasse fixe.
 *
 * ⚠️ `regenerate_ical_token` n'est pas un réglage : c'est un ordre ponctuel, et
 * le contrôleur le lit comme tel. Il vit ici parce que c'est là que l'opérateur
 * le cherche, pas parce que c'est un état.
 */
final class OperationsSettingsType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $roles = [];
        foreach ($options['assignable_roles'] as $role) {
            $roles[$role['label'] . ' — ' . $role['securityRole']] = $role['securityRole'];
        }

        $builder
            ->add('lab_rules_html', TextareaType::class, [
                'label' => 'admin_settings.rules_text',
                'help' => 'admin_settings.rules_text_help',
                'required' => false,
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => [
                    'class' => 'settings-textarea-tall',
                    'placeholder' => $this->translator->trans('admin_settings.rules_placeholder'),
                ],
            ])
            ->add('lab_rules_pdf_url', UrlType::class, [
                'label' => 'admin_settings.rules_pdf',
                'help' => 'admin_settings.rules_pdf_help',
                'required' => false,
                'empty_data' => '',
                'attr' => ['placeholder' => 'https://...'],
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('booking_identity_roles', ChoiceType::class, [
                'label' => 'admin_settings.privacy_roles',
                // ⚠️ Deux phrases d'aide, dont une avec du balisage voulu.
                'help' => 'admin_settings.privacy_hint',
                'help_html' => true,
                'choices' => $roles,
                'multiple' => true,
                'expanded' => true,
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('regenerate_ical_token', CheckboxType::class, [
                'label' => 'admin_settings.regen_token',
                'help' => 'admin_settings.ical_token_help',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults([
                'data_class' => null,
                'csrf_token_id' => 'settings_operations',
            ])
            ->setRequired('assignable_roles')
            ->setAllowedTypes('assignable_roles', 'array');
    }
}
