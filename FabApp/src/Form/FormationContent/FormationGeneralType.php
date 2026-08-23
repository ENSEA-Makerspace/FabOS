<?php

namespace App\Form\FormationContent;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/formations/{id}/content`, carte « Informations générales » (S147, J-22).
 *
 * ⚠️ **Ce n'est pas `FormationAdminType` et ça ne peut pas l'être.** Les deux
 * écrans touchent la même entité, mais pas les mêmes valeurs : ici `objectifs`,
 * `prerequis` et `materielFourni` sont des **listes une-par-ligne** que le
 * contrôleur recolle avec trois séparateurs différents (`. `, `\n`, `, `), et
 * `niveau` accepte 0 alors que l'écran d'administration le borne à 1–3. Fusionner
 * les deux types demanderait un jeu d'options qui redéciderait tout ça à chaque
 * appel — c'est-à-dire deux formulaires déguisés en un.
 *
 * 🔴 **Trois longueurs qui TRONQUAIENT deviennent trois refus.** Le contrôleur
 * passait `categorie`, `duree` et `formateur` par un `mb_substr()` : une catégorie
 * de 120 caractères était enregistrée coupée à 100, sans un mot à l'opérateur. La
 * contrainte refuse et la valeur reste à l'écran.
 */
final class FormationGeneralType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'formation_content.field_title',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 255, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'formation_content.field_description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('categorie', TextType::class, [
                'label' => 'formation_content.field_category',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La catégorie ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('niveau', IntegerType::class, [
                'label' => 'formation_content.field_level',
                'required' => false,
                'constraints' => [new Assert\PositiveOrZero(message: 'Le niveau ne peut pas être négatif.')],
            ])
            ->add('duree', TextType::class, [
                'label' => 'formation_content.field_duration',
                'required' => false,
                'constraints' => [new Assert\Length(max: 100, maxMessage: 'La durée ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('formateur', TextType::class, [
                'label' => 'formation_content.field_trainer',
                'required' => false,
                'constraints' => [new Assert\Length(max: 150, maxMessage: 'Le formateur ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('placesTotales', IntegerType::class, [
                'label' => 'formation_content.field_places',
                'required' => false,
                'constraints' => [new Assert\PositiveOrZero(message: 'Le nombre de places ne peut pas être négatif.')],
            ])
            // ⚠️ Les trois listes gardent leur phrase d'aide : le thème rend
            // `form_help()` depuis S147, donc elle survit à la conversion. Sans
            // `help`, l'opérateur perdrait « Un objectif par ligne. » en silence.
            ->add('objectifs', TextareaType::class, [
                'label' => 'formation_content.field_objectives',
                'help' => 'formation_content.one_per_line_objective',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('prerequis', TextareaType::class, [
                'label' => 'formation_content.field_prerequisites',
                'help' => 'formation_content.one_per_line_prerequisite',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('materielFourni', TextareaType::class, [
                'label' => 'formation_content.field_material',
                'help' => 'formation_content.one_per_line_item',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            // ⚠️ Explicite, sinon l'écran bascule sur le jeton *stateless* et
            // devient invérifiable par sonde (règle 2 de la conversion).
            'csrf_token_id' => 'formation_content_general',
        ]);
    }
}
