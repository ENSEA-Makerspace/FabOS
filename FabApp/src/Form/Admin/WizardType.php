<?php

namespace App\Form\Admin;

use App\Service\SiteSettingService;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/wizard` — les six réponses du premier démarrage (S147, J-22).
 *
 * ⚠️ **Les mêmes réglages que `/admin/settings`, posés une seule fois.** Le nom,
 * le lieu, l'URL publique, le fuseau et la langue vivent dans `SiteSettingService`
 * et s'éditent ensuite carte par carte ; cet écran ne fait que les demander tous
 * d'un coup, à quelqu'un qui n'a encore rien vu. Les contraintes sont donc les
 * mêmes — un fuseau inconnu doit être refusé ici exactement comme là-bas.
 *
 * 🔴 Et il y a une différence qui compte : l'assistant a un bouton « passer ».
 * Il ne soumet pas ce formulaire, il poste `action=skip` — sinon un champ
 * obligatoire empêcherait de sauter une étape qu'on a le droit de sauter.
 */
final class WizardType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('org_name', TextType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_settings.org_name',
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('venue_label', TextType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_settings.venue_name',
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 80, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('public_base_url', UrlType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_wizard.public_url',
                'required' => false,
                'empty_data' => '',
                'attr' => ['placeholder' => 'https://fablab.exemple.fr'],
            ])
            ->add('lab_address', TextType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_wizard.address',
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('timezone', ChoiceType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_settings.timezone_label',
                'choices' => array_combine(\DateTimeZone::listIdentifiers(), \DateTimeZone::listIdentifiers()),
                'constraints' => [new Assert\Callback(static function (?string $value, $context): void {
                    if ($value !== null && $value !== '' && !SiteSettingService::isValidTimezone($value)) {
                        $context->buildViolation('Fuseau horaire inconnu.')->addViolation();
                    }
                })],
            ])
            ->add('default_locale', ChoiceType::class, [
                'row_attr' => ['class' => 'wz-field'],
                'label' => 'admin_settings.default_language',
                'choices' => array_flip($options['available_locales']),
                'constraints' => [new Assert\Choice(
                    choices: array_keys($options['available_locales']),
                    message: 'Langue invalide.',
                )],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults([
                'data_class' => null,
                'csrf_token_id' => 'admin_wizard',
            ])
            ->setRequired('available_locales')
            ->setAllowedTypes('available_locales', 'array');
    }
}
