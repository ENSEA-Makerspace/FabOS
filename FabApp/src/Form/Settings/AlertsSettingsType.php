<?php

namespace App\Form\Settings;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;
use Symfony\Contracts\Translation\TranslatorInterface;

/**
 * `/admin/settings`, carte « Alertes » (S147, J-22).
 *
 * ⚠️ Le texte n'a **pas** de libellé propre : la case au-dessus porte la phrase,
 * et le champ la complète. `label => false` reproduit ça exactement plutôt que
 * d'inventer un intitulé que personne n'a écrit.
 *
 * 🔴 **S150 — `label => false` + repli = champ sans nom accessible.** Le texte
 * passe derrière un `<details>` (règle 2 : décochée, la case ne montre pas son
 * sous-champ), et un `<summary>` n'étiquette rien : ce n'est pas un `<label>`.
 * Tant que le libellé visible était la case juste au-dessus, la proximité
 * suffisait à l'œil ; dans un repli il n'y a plus rien du tout. D'où
 * l'`aria-label`, qui reprend la clé que le `<summary>` affiche — même mot, deux
 * porteurs, aucune clé inventée.
 *
 * ⚠️ `aria-label` est un attribut HTML, donc **Symfony ne le traduit pas** :
 * même piège que `placeholder`, même traducteur injecté.
 */
final class AlertsSettingsType extends AbstractType
{
    public function __construct(private readonly TranslatorInterface $translator)
    {
    }

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('alert_banner_enabled', CheckboxType::class, [
                'label' => 'admin_settings.banner_enabled',
                'required' => false,
            ])
            ->add('alert_banner_text', TextareaType::class, [
                'label' => false,
                'help' => 'admin_settings.banner_help',
                'required' => false,
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => [
                    'placeholder' => $this->translator->trans('admin_settings.banner_placeholder'),
                    'aria-label' => $this->translator->trans('admin_settings.banner_heading'),
                ],
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'settings_alerts',
        ]);
    }
}
