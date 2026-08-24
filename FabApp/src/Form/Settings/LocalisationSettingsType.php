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
 *
 * ⚠️ **S150 — le fuseau s'annonce avec son décalage** (règle 4). La liste rendait
 * `Europe/Paris` tout seul, ce qui n'apprend rien à qui ne connaît pas déjà la
 * base tz ; Fabman écrit `(UTC+02:00) Paris`. Les libellés deviennent donc
 * `(UTC+02:00) Europe/Paris`, l'ordre alphabétique par identifiant est conservé
 * — c'est celui qu'on parcourt à la recherche de sa ville — et **la valeur
 * enregistrée ne bouge pas** : `choices` est libellé => valeur, seul le libellé
 * change.
 *
 * 🔴 **Le décalage est celui de MAINTENANT, pas une constante.** `Europe/Paris`
 * est +01:00 en janvier et +02:00 en juillet. La liste est donc juste au moment
 * où la page est rendue et pas au-delà — ce qui est exactement ce qu'on veut dire,
 * puisque la ligne de conséquence sous le champ affiche l'heure locale du même
 * instant. Ne jamais mettre ce tableau en cache applicatif.
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
                'choices' => self::timezoneChoices(),
                'constraints' => [new Assert\Callback(static function (?string $value, $context): void {
                    // ⚠️ Vide est accepté — le contrôleur l'ignorait déjà, et un
                    // fuseau qu'on ne touche pas n'est pas un fuseau qu'on efface.
                    if ($value !== null && $value !== '' && !SiteSettingService::isValidTimezone($value)) {
                        $context->buildViolation('Fuseau horaire inconnu.')->addViolation();
                    }
                })],
            ]);
    }

    /**
     * `(UTC+02:00) Europe/Paris` => `Europe/Paris`, pour les ~420 identifiants.
     *
     * ⚠️ Le décalage n'est pas traduit et n'a pas à l'être : `UTC+02:00` est une
     * notation, pas une phrase — même règle que le nom d'un fuseau, qui reste
     * `Europe/Paris` dans les cinq langues parce que c'est un identifiant.
     *
     * ⚠️ Un identifiant que PHP refuse (la base tz du serveur peut être plus
     * ancienne que la liste) ne fait pas tomber l'écran : il garde son libellé nu.
     *
     * @return array<string, string>
     */
    private static function timezoneChoices(): array
    {
        $now = new \DateTimeImmutable('now');
        $choices = [];
        foreach (\DateTimeZone::listIdentifiers() as $identifier) {
            try {
                $offset = (new \DateTimeZone($identifier))->getOffset($now);
            } catch (\Throwable) {
                $choices[$identifier] = $identifier;
                continue;
            }
            $label = sprintf(
                '(UTC%s%02d:%02d) %s',
                $offset < 0 ? '-' : '+',
                intdiv(abs($offset), 3600),
                intdiv(abs($offset) % 3600, 60),
                $identifier,
            );
            $choices[$label] = $identifier;
        }

        return $choices;
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
