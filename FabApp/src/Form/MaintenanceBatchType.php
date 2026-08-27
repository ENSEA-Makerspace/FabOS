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
use Symfony\Component\Form\Extension\Core\Type\TextType;
use Symfony\Component\Form\Extension\Core\Type\UrlType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * Apply one maintenance task to many machines at once (a first-class need for
 * fleets like "all 3D printers"). Not bound to an entity — the controller
 * fans it out into one MaintenanceTask per selected machine.
 */
final class MaintenanceBatchType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('machines', EntityType::class, [
                'label' => 'Machines concernées',
                'class' => Machine::class,
                'choice_label' => 'nom',
                'multiple' => true,
                'expanded' => true,
                // ⚠️ S151 — `.choice-grid` sur le widget, comme sur le matériau :
                // le gabarit dessinait l'enveloppe à la main pour poser la classe,
                // ce qui laissait le groupe sans nom accessible.
                'attr' => ['class' => 'choice-grid'],
                'row_attr' => ['class' => 'full'],
                'query_builder' => static fn ($repo) => $repo->createQueryBuilder('m')->orderBy('m.nom', 'ASC'),
                'constraints' => [new Assert\Count(min: 1, minMessage: 'Sélectionnez au moins une machine.')],
            ])
            ->add('title', TextType::class, [
                'label' => 'Intitulé de la tâche',
                'constraints' => [new Assert\NotBlank(message: 'L’intitulé est obligatoire.'), new Assert\Length(max: 180)],
            ])
            ->add('type', ChoiceType::class, [
                'label' => 'form.type',
                'choices' => ['admin_maintenance_form.choice_preventive' => MaintenanceTask::TYPE_PREVENTIVE, 'admin_maintenance_form.choice_corrective' => MaintenanceTask::TYPE_CORRECTIVE],
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
            ->add('save', SubmitType::class, ['label' => 'Créer pour toutes les machines']);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults(['data_class' => null]);
    }
}
