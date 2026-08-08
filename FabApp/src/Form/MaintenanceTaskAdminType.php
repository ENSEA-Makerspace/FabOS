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
                'label' => 'Machine',
                'class' => Machine::class,
                'choice_label' => 'nom',
                'placeholder' => '— Choisir une machine —',
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('m')->orderBy('m.nom', 'ASC'),
                'constraints' => [new Assert\NotNull(message: 'Choisissez une machine.')],
            ])
            ->add('title', TextType::class, [
                'label' => 'Intitulé de la tâche',
                'empty_data' => '',
                'constraints' => [new Assert\NotBlank(message: 'L’intitulé est obligatoire.'), new Assert\Length(max: 180)],
            ])
            ->add('type', ChoiceType::class, [
                'label' => 'Type',
                'choices' => ['Préventive' => MaintenanceTask::TYPE_PREVENTIVE, 'Corrective' => MaintenanceTask::TYPE_CORRECTIVE],
            ])
            ->add('dueDate', DateType::class, [
                'label' => 'Échéance (optionnelle)',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            ->add('recurrenceDays', IntegerType::class, [
                'label' => 'Récurrence : tous les N jours (optionnel)',
                'required' => false,
                'constraints' => [new Assert\Positive(message: 'La récurrence doit être un nombre de jours positif.')],
            ])
            ->add('link', UrlType::class, [
                'label' => 'Lien (guide / wiki, optionnel)',
                'required' => false,
                'default_protocol' => 'https',
                'constraints' => [new Assert\Length(max: 500), new Assert\Url(message: 'URL invalide.')],
            ])
            ->add('notes', TextareaType::class, [
                'label' => 'Notes (optionnel)',
                'required' => false,
                'constraints' => [new Assert\Length(max: 2000)],
            ])
            ->add('save', SubmitType::class, ['label' => 'Enregistrer']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => MaintenanceTask::class]);
    }
}
