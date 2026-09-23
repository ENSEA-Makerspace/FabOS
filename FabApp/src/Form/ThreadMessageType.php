<?php

namespace App\Form;

use App\Training\FormationThreads;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Un message dans un fil de formation (S183b).
 *
 * ⚠️ **Un seul champ, et pas de destinataire.** Il n'y a rien à choisir : un fil
 * relie UN apprenant à l'équipe. Un champ « à qui » serait précisément le chemin
 * par lequel un message privé finirait chez la cohorte.
 */
final class ThreadMessageType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('body', TextareaType::class, [
                'label' => 'thread.field_body',
                'help' => 'thread.help_body',
                'empty_data' => '',
                'attr' => ['rows' => 5, 'maxlength' => FormationThreads::MAX_LENGTH],
                'constraints' => [
                    new Assert\NotBlank(message: 'Le message est vide.'),
                    new Assert\Length(max: FormationThreads::MAX_LENGTH, maxMessage: 'Le message ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('send', SubmitType::class, ['label' => 'thread.send']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null]);
    }
}
