<?php

namespace App\Form;

use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Le texte qu'un exploitant réécrit pour un e-mail, dans UNE langue (S161).
 *
 * ⚠️ **Aucune contrainte `NotBlank` : le vide est une RÉPONSE.** Vider les deux
 * champs veut dire « reviens au texte livré », et c'est le « revenir au texte
 * livré » que S162 demande — obtenu sans bouton supplémentaire.
 *
 * ⚠️ **Le refus d'un champ inconnu n'est PAS ici.** Il dépend du gabarit qu'on
 * édite, donc du contexte de la requête ; il vit dans le contrôleur, qui
 * connaît la clé. Une contrainte qui aurait besoin qu'on lui passe la liste
 * serait une liste de plus à tenir d'accord.
 */
final class MailOverrideType extends AbstractType
{
    public const SECTIONS = [
        [
            'title' => 'mail_editor.section_text',
            'fields' => ['subject', 'body'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('subject', TextType::class, [
                'label' => 'mail_editor.field_subject',
                'help' => 'mail_editor.help_subject',
                'required' => false,
                'empty_data' => '',
                'constraints' => [new Assert\Length(max: 255, maxMessage: 'L’objet ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('body', TextareaType::class, [
                'label' => 'mail_editor.field_body',
                'help' => 'mail_editor.help_body',
                'required' => false,
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => ['rows' => 14],
                'constraints' => [new Assert\Length(max: 8000, maxMessage: 'Le message ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null]);
    }
}
