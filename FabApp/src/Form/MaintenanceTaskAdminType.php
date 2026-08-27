<?php

namespace App\Form;

use App\Entity\Machine;
use App\Entity\MaintenanceTask;
use Symfony\Bridge\Doctrine\Form\Type\EntityType;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\ChoiceType;
use Symfony\Component\Form\Extension\Core\Type\DateType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\Extension\Core\Type\SubmitType;
use Symfony\Component\Form\Extension\Core\Type\TextareaType;
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

final class MaintenanceTaskAdminType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('machine', EntityType::class, [
                'label' => 'admin_maintenance_form.machine',
                'class' => Machine::class,
                'choice_label' => 'nom',
                'placeholder' => 'admin_maintenance_form.ph_machine',
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('m')->orderBy('m.nom', 'ASC'),
                'constraints' => [new Assert\NotNull(message: 'Choisissez une machine.')],
            ])
            ->add('title', TextType::class, [
                'label' => 'admin_maintenance_form.title',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: 'L’intitulé est obligatoire.'), new Assert\Length(max: 180)],
            ])
            ->add('type', ChoiceType::class, [
                'label' => 'form.type',
                'choices' => ['admin_maintenance_form.choice_preventive' => MaintenanceTask::TYPE_PREVENTIVE, 'admin_maintenance_form.choice_corrective' => MaintenanceTask::TYPE_CORRECTIVE],
            ])
            ->add('dueDate', DateType::class, [
                'label' => 'admin_maintenance_form.due_date',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            ->add('recurrenceDays', IntegerType::class, [
                'label' => 'admin_maintenance_form.recurrence_days',
                'required' => false,
                'constraints' => [new Assert\Positive(message: 'La récurrence doit être un nombre de jours positif.')],
            ])
            ->add('link', UrlType::class, [
                'label' => 'admin_maintenance_form.link',
                'required' => false,
                'default_protocol' => 'https',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('notes', TextareaType::class, [
                'label' => 'admin_maintenance_form.notes',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            ->add('save', SubmitType::class, ['label' => 'common.save']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => MaintenanceTask::class]);
    }
}
