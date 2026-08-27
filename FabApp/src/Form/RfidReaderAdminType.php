<?php

namespace App\Form;

use App\Entity\Machine;
use App\Entity\RfidReader;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class RfidReaderAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('name', TextType::class, [
                'label' => 'admin_rfid_form.name',
                'empty_data' => '',
                'constraints' => [
                    new Assert\NotBlank(message: 'Le nom est obligatoire.'),
                    new Assert\Length(max: 120, maxMessage: 'Le nom ne doit pas dépasser {{ limit }} caractères.'),
                ],
            ])
            ->add('readerToken', TextType::class, [
                'label' => 'admin_rfid_form.reader_token',
                'required' => false,
                'empty_data' => '',
                'help' => 'admin_rfid_form.help_reader_token',
                'constraints' => [
                    new Assert\Length(max: 120, maxMessage: 'Le readerToken ne doit pas dépasser {{ limit }} caractères.'),
                    new Assert\Regex(pattern: '/^[a-zA-Z0-9_-]*$/', message: 'Utilisez uniquement lettres, chiffres, tirets et underscores.'),
                ],
            ])
            ->add('machine', EntityType::class, [
                'label' => 'admin_rfid_form.machine',
                'class' => Machine::class,
                'choice_label' => static fn (Machine $machine): string => sprintf('%s (%s)', $machine->getNom(), $machine->getMachineToken()),
                'choice_attr' => static fn (Machine $machine): array => ['data-machine-token' => $machine->getMachineToken()],
                'placeholder' => 'admin_rfid_form.ph_machine',
                'constraints' => [new Assert\NotNull(message: 'La machine associée est obligatoire.')],
            ])
            ->add('isActive', CheckboxType::class, [
                'label' => 'admin_rfid_form.is_active',
                'help' => 'admin_rfid_form.help_is_active',
                'required' => false,
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => RfidReader::class,
        ]);
    }
}
