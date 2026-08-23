<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;

/**
 * `/admin/utilisateurs/{id}`, carte « Type de personne » (S148, J-22).
 *
 * ⚠️ **Aucune contrainte, et ce n'est pas un oubli.** Trois cases ne peuvent pas
 * être invalides : toute combinaison est un état que l'opérateur a le droit de
 * demander. Ce que la conversion apporte ici n'est donc pas le refus — c'est le
 * balisage : trois `.form-field-check` du thème au lieu de trois `<label
 * class="admin-choice-row">` écrits à la main, et l'aide de la troisième case
 * rendue par `form_help()` au lieu d'un `<p>` posé à côté.
 *
 * 🔴 La conséquence de décocher « réservable » — les rendez-vous à venir sont
 * ANNULÉS et leurs titulaires prévenus — reste dans le contrôleur : elle dépend
 * de l'état d'AVANT, qu'un type de formulaire ne voit pas.
 */
final class PersonTypeType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('is_staff', CheckboxType::class, [
                'label' => 'admin_user_detail.is_staff',
                'required' => false,
            ])
            ->add('is_trainer', CheckboxType::class, [
                'label' => 'admin_user_detail.is_trainer',
                'required' => false,
            ])
            ->add('is_bookable', CheckboxType::class, [
                'label' => 'admin_user_detail.is_bookable',
                'help' => 'admin_user_detail.is_bookable_help',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired('user_id')
            ->setAllowedTypes('user_id', 'int')
            // ⚠️ Le jeton reste PAR UTILISATEUR, comme avant la conversion : un
            // jeton partagé rendrait une soumission valable sur n'importe quelle
            // fiche.
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'person_type_' . $o['user_id']);
    }
}
