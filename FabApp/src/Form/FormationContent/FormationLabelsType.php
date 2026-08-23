<?php

namespace App\Form\FormationContent;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/formations/{id}/content`, carte « Titres des boîtes » (S147, J-22).
 *
 * ⚠️ **Aucun champ n'est obligatoire, et c'est voulu.** Vider un titre ne le
 * refuse pas : le contrôleur remet le libellé par défaut, exactement comme avant
 * la conversion. La seule chose que ce type ajoute, c'est une borne — un titre de
 * boîte est une ligne, pas un paragraphe, et rien ne la posait.
 */
final class FormationLabelsType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        foreach ([
            'descriptionTitle' => 'formation_content.label_description',
            'objectivesTitle' => 'formation_content.label_objectives',
            'prerequisitesTitle' => 'formation_content.label_prerequisites',
            'materialTitle' => 'formation_content.label_material',
        ] as $field => $label) {
            $builder->add($field, TextType::class, [
                'label' => $label,
                'required' => false,
                'constraints' => [new Assert\Length(max: 120, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ]);
        }
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'formation_content_labels',
        ]);
    }
}
