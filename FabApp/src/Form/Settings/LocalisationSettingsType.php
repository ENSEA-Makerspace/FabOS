<?php

namespace App\Form\Settings;

use App\Service\SiteSettingService;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/settings`, carte « Localisation » (S147, J-22).
 *
 * 🔴 **C'est la carte qui refusait quelque chose, et c'est là que la conversion
 * paie.** Avant : un fuseau inconnu déclenchait un message d'erreur en haut de
 * page et le champ revenait silencieusement à sa valeur enregistrée. Maintenant
 * l'erreur est **sur le champ**, et ce que l'opérateur a choisi reste affiché.
 *
 * ⚠️ Les deux listes restent des `ChoiceType` fermés : le libellé de langue vient
 * du `LocaleCatalog` et les fuseaux de `DateTimeZone::listIdentifiers()`, donc
 * une valeur hors liste est déjà impossible depuis l'écran. La contrainte est là
 * pour la requête forgée, pas pour l'opérateur — et c'est elle qui remplace le
 * `array_key_exists()` que le contrôleur faisait à la main.
 */
final class LocalisationSettingsType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('default_locale', ChoiceType::class, [
                'label' => 'admin_settings.default_language',
                'help' => 'admin_settings.default_language_help',
                // `choices` attend libellé => valeur ; le catalogue rend code => libellé.
                'choices' => array_flip($options['available_locales']),
                'constraints' => [new Assert\Choice(
                    choices: array_keys($options['available_locales']),
                    message: 'Langue invalide.',
                )],
            ])
            ->add('timezone', ChoiceType::class, [
                'label' => 'admin_settings.timezone_label',
                'help' => 'admin_settings.timezone_now',
                'choices' => array_combine(\DateTimeZone::listIdentifiers(), \DateTimeZone::listIdentifiers()),
                'constraints' => [new Assert\Callback(static function (?string $value, $context): void {
                    // ⚠️ Vide est accepté — le contrôleur l'ignorait déjà, et un
                    // fuseau qu'on ne touche pas n'est pas un fuseau qu'on efface.
                    if ($value !== null && $value !== '' && !SiteSettingService::isValidTimezone($value)) {
                        $context->buildViolation('Fuseau horaire inconnu.')->addViolation();
                    }
                })],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults([
                'data_class' => null,
                'csrf_token_id' => 'settings_localisation',
            ])
            ->setRequired('available_locales')
            ->setAllowedTypes('available_locales', 'array');
    }
}
