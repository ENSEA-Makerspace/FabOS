<?php

namespace App\Form\UsageRights;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\Options;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/usage-rights/{id}/edit`, la carte « le package lui-même » (S147, J-22).
 *
 * 🔴 **S153 — ce formulaire ne décrit plus QUE l'identité du package.** La
 * matrice de fonctionnalités et la case « accès complet » en sont parties : ce
 * que le package autorise se saisit maintenant en un seul endroit, la carte
 * « Ce que ce package autorise », et `PackageSpecCompiler` l'écrit dans les cinq
 * tables. Les laisser ici aurait été garder deux écritures pour un même fait —
 * exactement les « deux vérités » que la mesure a trouvées en base, un package
 * portant `fullAccess = 1` ET quatorze grants.
 *
 * ⚠️ Sauvegarder l'identité ne touche donc plus ni `fullAccess` ni les lignes de
 * feature : le contrôleur repasse à `save()` ce que le package portait déjà.
 *
 * ⚠️ `row_attr` garde `.usage-form-field`, la classe de cet écran : le thème
 * impose l'ordre libellé / contrôle / aide / erreurs, pas une apparence.
 */
final class PackageDetailsType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'adm.col_name',
                'row_attr' => ['class' => 'usage-form-field'],
                'attr' => ['maxlength' => 120],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 120, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'usage_rights.description_label',
                'row_attr' => ['class' => 'usage-form-field'],
                'required' => false,
                'empty_data' => '',
                'attr' => ['maxlength' => 1000],
                'constraints' => [new Assert\Length(max: 1000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('active', CheckboxType::class, [
                'label' => 'usage_rights.active',
                // `.usage-check` est la ligne case-puis-libellé de cet écran ;
                // le thème rend déjà les deux dans cet ordre.
                'row_attr' => ['class' => 'usage-check'],
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver
            ->setDefaults(['data_class' => null])
            // Le jeton reste porté par le package, comme avant : deux packages
            // ouverts dans deux onglets ne doivent pas partager un jeton.
            ->setRequired('package_key')
            ->setDefault('csrf_token_id', static fn (Options $o): string => 'usage_package_form_' . $o['package_key']);
    }
}
