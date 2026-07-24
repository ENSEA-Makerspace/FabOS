<?php

namespace App\Form;

use App\Entity\Event;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\DateTimeType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class EventAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('titre', TextType::class, [
                'label' => 'Titre',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le titre est obligatoire.'),
                    new Assert\Length(max: 180, maxMessage: 'Le titre ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('dateDebut', DateTimeType::class, [
                'label' => 'Début',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'constraints' => [new Assert\NotNull(message: 'La date de début est obligatoire.')],
            ])
            ->add('dateFin', DateTimeType::class, [
                'label' => 'Fin (optionnelle)',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            ->add('lieu', TextType::class, [
                'label' => 'Nom du lieu',
                'required' => false,
                'help' => 'Le nom courant de l\'endroit : « Grande salle », « Atelier bois »…',
                'constraints' => [new Assert\Length(max: 180, maxMessage: 'Ce champ ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('locationMode', ChoiceType::class, [
                'label' => 'Où se déroule l\'événement ?',
                'choices' => [
                    'Au fablab' => Event::LOCATION_ONSITE,
                    'Ailleurs (adresse spécifique)' => Event::LOCATION_OFFSITE,
                ],
                'expanded' => true,
                'help' => 'Au fablab, l\'adresse est reprise automatiquement des réglages du site.',
            ])
            ->add('address', TextType::class, [
                'label' => 'Adresse (si ailleurs)',
                'required' => false,
                'help' => 'Adresse postale complète. Un lien d\'itinéraire est généré automatiquement.',
                'constraints' => [new Assert\Length(max: 500, maxMessage: 'L\'adresse ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('description', TextareaType::class, [
                'label' => 'Description',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000, maxMessage: 'La description ne doit pas dépasser {{ limit }} caractères.')],
            ])
            ->add('capacite', IntegerType::class, [
                'label' => 'Nombre de places',
                'required' => false,
                'help' => 'Laissez vide pour un nombre de places illimité. Au-delà, les inscriptions passent en liste d\'attente.',
                'constraints' => [new Assert\PositiveOrZero(message: 'Le nombre de places ne peut pas être négatif.')],
            ])
            ->add('guestsAllowed', CheckboxType::class, [
                'label' => 'Ouvert aux personnes sans compte',
                'required' => false,
                'help' => 'Décochez pour réserver cet événement aux membres connectés. Les invités déjà inscrits gardent leur place.',
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => Event::class,
            'constraints' => [
                new Assert\Callback([$this, 'validateDates']),
            ],
        ]);
    }

    public function validateDates(?Event $event, \Symfony\Component\Validator\Context\ExecutionContextInterface $context): void
    {
        if ($event === null) {
            return;
        }
        $start = $event->getDateDebut();
        $end = $event->getDateFin();
        if ($event->getLocationMode() === Event::LOCATION_OFFSITE && ($event->getAddress() ?? '') === '') {
            $context->buildViolation('Indiquez l\'adresse, ou choisissez « Au fablab ».')
                ->atPath('address')
                ->addViolation();
        }

        if ($start !== null && $end !== null && $end < $start) {
            $context->buildViolation('La date de fin doit être après la date de début.')
                ->atPath('dateFin')
                ->addViolation();
        }
    }
}
