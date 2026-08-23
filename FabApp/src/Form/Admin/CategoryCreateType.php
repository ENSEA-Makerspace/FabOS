<?php

namespace App\Form\Admin;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Créer une catégorie — d'événement ou de machine (S148, J-22).
 *
 * ⚠️ **Un seul type pour les deux écrans, parce que c'est le MÊME formulaire.**
 * Deux champs, les mêmes bornes (100 et 50, celles des colonnes), la même règle.
 * Ce qui diffère entre `/admin/evenements/categories` et
 * `/admin/machines/categories` — les libellés, les exemples, le jeton — sont des
 * options, pas du comportement. Les *contrôleurs*, eux, restent séparés : les
 * événements portent une clé étrangère et les machines se joignent par LIBELLÉ,
 * donc renommer et archiver n'y veulent pas dire la même chose. C'est la création
 * qui est commune, et elle seule.
 *
 * 🔴 **L'unicité du slug reste dans le contrôleur.** Elle demande le dépôt et la
 * dérivation du slug ; la recopier ici en ferait une deuxième décision, et deux
 * décisions divergent. Le contrôleur pose l'erreur SUR le champ `label`.
 */
final class CategoryCreateType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('label', TextType::class, [
                'label' => $options['name_label'],
                'attr' => ['placeholder' => $options['name_placeholder'], 'maxlength' => 100],
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom de la catégorie est obligatoire.'),
                    new Assert\Length(max: 100, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('icon_slug', TextType::class, [
                'label' => $options['icon_label'],
                'required' => false,
                'attr' => ['placeholder' => $options['icon_placeholder'], 'maxlength' => 50],
                'constraints' => [new Assert\Length(max: 50, maxMessage: "L'icône ne doit pas dépasser {{ limit }} caractères.")],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            ->setRequired(['name_label', 'name_placeholder', 'icon_label', 'icon_placeholder', 'csrf_token_id'])
            ->setAllowedTypes('name_label', 'string')
            ->setAllowedTypes('name_placeholder', 'string')
            ->setAllowedTypes('icon_label', 'string')
            ->setAllowedTypes('icon_placeholder', 'string');
    }
}
