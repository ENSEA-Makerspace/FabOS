<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\NumberType;
use Symfony\Component\Form\Extension\Core\Type\TimeType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;

/**
 * LA SAISIE D'UN PACKAGE — quatre lignes de restriction, une d'extension (S153).
 *
 * ⚠️ **Chaque axe est une case « aucune restriction » PLUS sa liste.** Les cases
 * « tout / tout le temps / partout / sans limite » arrivent COCHÉES, parce que
 * c'est l'état mesuré des packages existants : 21 grants, et les 21 sans lieu,
 * sans ressource, sans catégorie, sans fenêtre. Décocher révèle le détail, en
 * CSS pur (`:has()`) — donc le formulaire est utilisable sans qu'un contrôleur
 * ait tourné.
 *
 * 🔴 **Les listes cachées POSTENT quand même.** Le repli est du CSS : il masque,
 * il ne désactive rien. C'est exactement le piège que S149 a payé sur les champs
 * de plage, et la réponse est la même — c'est la CASE qui décide, jamais le
 * contenu de la liste. `PackageSpec::isUnrestricted()` et le compilateur ne lisent
 * une liste que si sa case est décochée.
 *
 * ⚠️ **Les heures sont des `TimeType` en `input: 'string'`.** Le compilateur
 * appelle `GrantWindow::fromClock()`, qui parle « HH:MM » : la valeur doit rester
 * une CHAÎNE, sans quoi un aller-retour par `DateTime` glisse un fuseau dans une
 * heure de mur. Même réglage que `PackageGrantType::clock()`.
 */
final class PackageSpecType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('features_all', CheckboxType::class, ['label' => 'usage_rights.spec_features_all', 'required' => false])
            ->add('features', ChoiceType::class, [
                'label' => false,
                'choice_translation_domain' => false,
                'choices' => $options['feature_choices'],
                'multiple' => true,
                'expanded' => true,
                'required' => false,
            ])
            ->add('days_all', CheckboxType::class, ['label' => 'usage_rights.spec_days_all', 'required' => false])
            ->add('days', ChoiceType::class, [
                'label' => false,
                'choice_translation_domain' => false,
                'choices' => $options['day_choices'],
                'multiple' => true,
                'expanded' => true,
                'required' => false,
            ])
            // 🔴 **`TimeType` en `input: 'string'`, et les DEUX moitiés comptent.**
            // Première version en `TextType` avec `attr: {type: time}` : le thème
            // pose `type` lui-même, l'attribut était ignoré, et le champ rendait
            // `type="text"` — mesuré au navigateur, invisible au lint.
            // `widget: single_text` donne le sélecteur du navigateur ;
            // `input: 'string'` + `input_format: 'H:i'` gardent la valeur en
            // « HH:MM », qui est ce que `GrantWindow::fromClock()` attend. Un
            // aller-retour par `DateTime` serait une conversion de plus, donc un
            // fuseau de plus. Même réglage que `PackageGrantType::clock()`.
            ->add('start_time', TimeType::class, [
                'label' => 'usage_rights.window_from',
                'widget' => 'single_text', 'html5' => true,
                'input' => 'string', 'input_format' => 'H:i',
                'required' => false,
            ])
            ->add('end_time', TimeType::class, [
                'label' => 'usage_rights.window_to',
                'widget' => 'single_text', 'html5' => true,
                'input' => 'string', 'input_format' => 'H:i',
                'required' => false,
            ])
            ->add('venues_all', CheckboxType::class, ['label' => 'usage_rights.spec_venues_all', 'required' => false])
            ->add('venues', ChoiceType::class, [
                'label' => false,
                'choice_translation_domain' => false,
                'choices' => $options['venue_choices'],
                'multiple' => true,
                'expanded' => true,
                'required' => false,
            ])
            ->add('categories_all', CheckboxType::class, ['label' => 'usage_rights.spec_categories_all', 'required' => false])
            ->add('categories', ChoiceType::class, [
                'label' => false,
                'choice_translation_domain' => false,
                'choices' => $options['category_choices'],
                'multiple' => true,
                'expanded' => true,
                'required' => false,
            ])
            ->add('quota_unlimited', CheckboxType::class, ['label' => 'usage_rights.spec_quota_all', 'required' => false])
            ->add('quota_hours', NumberType::class, [
                'label' => 'usage_rights.spec_quota_hours',
                'required' => false,
                'html5' => true,
                'scale' => 2,
                'attr' => ['min' => 0, 'step' => '0.25'],
            ])
            ->add('quota_period', ChoiceType::class, [
                'label' => 'usage_rights.allowance_period',
                'choice_translation_domain' => false,
                'choices' => $options['period_choices'],
                'required' => false,
            ])
            ->add('hours_exempt', CheckboxType::class, [
                'label' => 'usage_rights.spec_hours_exempt',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['feature_choices', 'day_choices', 'venue_choices', 'category_choices', 'period_choices', 'package_key'])
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_spec_' . $o['package_key']);
    }
}
