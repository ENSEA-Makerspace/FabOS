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

/**
 * Une tâche de maintenance (J-10, resté ouvert à la clôture de la Phase O).
 *
 * 🔴 **Sept champs visibles, ZÉRO aide — et rien dans le gabarit non plus.** Le
 * critère de S149 est « aucun écran à zéro aide pour 8 champs ou plus » ; celui-ci
 * y figurait depuis, et la Phase O s'est fermée sans le traiter. Vérifié avant
 * d'écrire, parce que `PackageSpecType` avait été accusé à tort : lui expliquait
 * dans son GABARIT. Ici le gabarit ne porte pas une phrase.
 *
 * ⚠️ **Quatre aides, pas sept.** Le taux n'est pas l'objectif — le compléter à
 * l'aveugle produit du bruit, et c'est la reformulation même du critère. « Machine »,
 * « Intitulé » et « Notes » ne demandent rien : les trois autres cachent une
 * CONSÉQUENCE qu'on ne peut pas deviner.
 *
 * 🔴 **Chaque aide dit ce que le champ DÉCLENCHE, pas ce qu'il est.** « Échéance :
 * la date d'échéance » n'apprend rien. « Passée cette date, toute l'équipe reçoit
 * un rappel — et sans date, personne n'en reçoit » est ce qu'on ignore. Les trois
 * faits ont été LUS dans le code, pas supposés :
 *   — `MaintenanceOverdueReminderScanner` écrit à chaque membre de l'équipe pour
 *     toute tâche dont l'échéance est passée, et ne voit pas celles sans date ;
 *   — la récurrence compte à partir du jour où la tâche est FAITE
 *     (`AdminController::completeMaintenance`, `today->modify('+N days')`), pas de
 *     l'échéance précédente : une tâche faite en retard décale la suivante ;
 *   — le lien part DANS le mail de rappel, il n'est pas qu'une note interne.
 *
 * ⚠️ **Et l'aide du lien a été RACCOURCIE après l'avoir vue à l'écran** : elle
 * commençait par « Procédure, manuel, fiche fournisseur », que l'étiquette dit
 * déjà — « Lien (guide / wiki, facultatif) ». Une aide qui répète son étiquette
 * apprend à ne pas lire les aides.
 */
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
            /*
             * 🅿️ **Une étiquette, et rien d'autre — dit franchement.** Grep sur
             * `TYPE_PREVENTIVE`/`TYPE_CORRECTIVE` : les deux constantes ne sont
             * lues que par les deux formulaires et l'entité. Aucune garde, aucun
             * filtre, aucun envoi n'en dépend. Sans cette phrase, « corrective »
             * laisse croire que la machine se bloque.
             */
            ->add('type', ChoiceType::class, [
                'label' => 'form.type',
                'help' => 'admin_maintenance_form.help_type',
                'choices' => ['admin_maintenance_form.choice_preventive' => MaintenanceTask::TYPE_PREVENTIVE, 'admin_maintenance_form.choice_corrective' => MaintenanceTask::TYPE_CORRECTIVE],
            ])
            ->add('dueDate', DateType::class, [
                'label' => 'admin_maintenance_form.due_date',
                'help' => 'admin_maintenance_form.help_due_date',
                'widget' => 'single_text',
                'input' => 'datetime_immutable',
                'required' => false,
            ])
            ->add('recurrenceDays', IntegerType::class, [
                'label' => 'admin_maintenance_form.recurrence_days',
                'help' => 'admin_maintenance_form.help_recurrence_days',
                'required' => false,
                'constraints' => [new Assert\Positive(message: 'La récurrence doit être un nombre de jours positif.')],
            ])
            ->add('link', UrlType::class, [
                'label' => 'admin_maintenance_form.link',
                'help' => 'admin_maintenance_form.help_link',
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
