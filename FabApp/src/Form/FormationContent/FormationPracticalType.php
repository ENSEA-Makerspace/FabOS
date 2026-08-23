<?php

namespace App\Form\FormationContent;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/formations/{id}/content`, carte « Formation pratique » (S147, J-22).
 *
 * ⚠️ **Cette carte ne décide de rien.** Le fait qu'une validation pratique soit
 * nécessaire est calculé ailleurs ; ici on écrit seulement les deux textes que
 * l'un ou l'autre cas affichera. Comme pour les titres, un champ vidé retombe sur
 * son défaut plutôt que d'être refusé.
 */
final class FormationPracticalType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $short = static fn (string $label, string $rowClass = ''): array => [
            'label' => $label,
            'required' => false,
            'row_attr' => $rowClass !== '' ? ['class' => $rowClass] : [],
            'constraints' => [new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
        ];

        $builder
            ->add('title', TextType::class, $short('formation_content.box_title', 'full'))
            ->add('requiredLabel', TextType::class, $short('formation_content.required_label'))
            ->add('requiredStatus', TextType::class, $short('formation_content.required_status'))
            ->add('requiredDescription', TextareaType::class, [
                'label' => 'formation_content.required_description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ])
            ->add('optionalLabel', TextType::class, $short('formation_content.optional_label'))
            ->add('optionalStatus', TextType::class, $short('formation_content.optional_status'))
            ->add('optionalDescription', TextareaType::class, [
                'label' => 'formation_content.optional_description',
                'required' => false,
                'row_attr' => ['class' => 'full'],
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'formation_content_practical',
        ]);
    }
}
