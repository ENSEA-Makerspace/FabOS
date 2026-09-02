<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Attribuer un forfait à un GROUPE (S147, J-22).
 *
 * 🔴 **Les deux DATES ont été retirées (revue R3, 2026-09-03), et c'est le
 * doublon que toute la phase S158/S159 a servi à supprimer.** « Jusqu'à quand »
 * s'écrivait à deux endroits — ici, et sur l'appartenance au groupe. Les deux
 * s'appliquaient, l'accès s'arrêtant à la première échéance atteinte ; mais rien
 * à l'écran ne disait lequel des deux champs répondait à « Sofia jusqu'au
 * 30 juin ». Un opérateur avait deux endroits plausibles, dont un seul était
 * juste : c'est exactement la forme de défaut que cette phase a rangée partout
 * ailleurs.
 *
 * ✅ **Et le choix est tranché par la mesure, pas par le goût** : les trois
 * attributions de groupe vivantes de la boîte (#73, #74, #75) n'ont ni
 * `validFrom` ni `validUntil`. La surface d'écriture retirée ne servait à
 * personne.
 *
 * ⚠️ **Ce qui reste, et pourquoi** : la date va sur **l'appartenance**, où elle
 * concerne une personne — c'est déjà ce que fait la conversion de S159 étape 3,
 * qui porte volontairement la date d'Alvaro sur son appartenance et laisse
 * l'attribution de groupe sans bornes. Le MODÈLE, lui, ne bouge pas :
 * `UsagePackageRepository::assignGroup()` accepte toujours deux dates, et les
 * colonnes restent — c'est le chemin de ce qu'une MACHINE écrira (module
 * commerce). Ce qui disparaît est la surface humaine, pas la capacité.
 *
 * 🅿️ Le jour où attribuer un forfait à TOUT un groupe pour une durée limitée
 * aura un usage réel, les champs reviendront — avec une phrase qui dit en quoi
 * ils diffèrent de la date d'appartenance.
 */
final class PackageAssignGroupType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('group_key', ChoiceType::class, [
                'label' => 'usage_rights.assignment_group',
                'choice_translation_domain' => false,
                'choices' => $options['group_choices'],
                'placeholder' => 'usage_rights.select_group',
                'constraints' => [new Assert\NotBlank(message: 'Choisissez le groupe à qui accorder ce forfait.')],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['group_choices', 'package_key'])
            ->setAllowedTypes('group_choices', 'array')
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_assign_group_' . $o['package_key']);
    }
}
