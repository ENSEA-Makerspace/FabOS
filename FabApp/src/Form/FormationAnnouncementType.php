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
 * L'annonce d'un formateur à sa cohorte (S183).
 *
 * ⚠️ **Deux champs, et pas un de plus.** Pas de destinataires à cocher : la
 * cohorte se déduit des progressions, et offrir une liste à cocher inviterait à
 * composer une audience — donc à la stocker, donc à avoir deux vérités sur « qui
 * suit ce cours ».
 */
final class FormationAnnouncementType extends AbstractType
{
    public const SECTIONS = [
        [
            'title' => 'announce.section_message',
            'fields' => ['subject', 'body'],
        ],
    ];

    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('subject', TextType::class, [
                'label' => 'announce.field_subject',
                'help' => 'announce.help_subject',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Un objet est obligatoire.'),
                    new Assert\Length(max: 150, maxMessage: 'L’objet ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('body', TextareaType::class, [
                'label' => 'announce.field_body',
                'help' => 'announce.help_body',
                'empty_data' => '',
                'row_attr' => ['class' => 'full'],
                'attr' => ['rows' => 10],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le message est obligatoire.'),
                    new Assert\Length(max: 4000, maxMessage: 'Le message ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('save', SubmitType::class, ['label' => 'announce.send']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null]);
    }
}
