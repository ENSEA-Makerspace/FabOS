<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;

/**
 * `/admin/utilisateurs/{id}`, carte des rendez-vous (S148, J-22).
 *
 * 🔴 **REVUE R5 (2026-09-03) — « Équipe » et « Formateur » sont RETIRÉES d'ici.**
 * Depuis S159e, ces deux cases écrivaient exactement les mêmes lignes que le
 * panneau « Groupes » situé quelques centimètres plus haut sur la même page :
 * deux contrôles pour un seul fait, c'est-à-dire la règle de toute la phase
 * S158/S159 prise en défaut sur son propre écran.
 *
 * ⚠️ **Ce qui se perd, et il faut le dire** : les cases étaient plus RAPIDES que
 * le menu du panneau Groupes — un clic contre deux. C'est un vrai coût, accepté
 * parce qu'un opérateur qui voit deux endroits pour la même chose finit par
 * croire qu'ils font des choses différentes.
 *
 * ✅ **Ce qui reste ici est un fait d'une autre nature** : « réservable » est
 * porté par une COLONNE de l'utilisateur, pas par une appartenance, et il
 * n'existe nulle part ailleurs.
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
