<?php

namespace App\Form\Emails;

use App\Mail\ReminderSettings;
use Symfony\Component\Form\AbstractType;
use Symfony\Component\Form\Extension\Core\Type\CheckboxType;
use Symfony\Component\Form\Extension\Core\Type\IntegerType;
use Symfony\Component\Form\FormBuilderInterface;
use Symfony\Component\OptionsResolver\OptionsResolver;
use Symfony\Component\Validator\Constraints as Assert;

/**
 * `/admin/emails`, carte « Rappels programmés » (S147, J-22).
 *
 * ⚠️ **Les bornes étaient dans le gabarit et nulle part ailleurs.** `min`/`max`
 * sur un `<input type="number">` n'engagent que le navigateur ; le contrôleur
 * lisait `getInt()`, qui rend 0 pour tout ce qu'il ne comprend pas. Un délai
 * négatif ou absurde entrait donc sans un mot, et se manifestait bien plus tard
 * en rappels qui ne partaient jamais. Les mêmes bornes sont maintenant des
 * contraintes, du côté qui décide.
 */
final class MailRemindersType extends AbstractType
{
    public function buildForm(FormBuilderInterface $builder, array $options): void
    {
        $builder
            ->add('reminder_booking', CheckboxType::class, [
                'label' => 'admin_emails.reminder_booking',
                'help' => 'admin_emails.reminder_booking_help',
                'required' => false,
            ])
            ->add('reminder_booking_lead_hours', IntegerType::class, [
                'label' => 'admin_emails.lead_hours',
                'help' => 'admin_emails.lead_hours_help',
                'empty_data' => (string) ReminderSettings::DEFAULT_BOOKING_LEAD_HOURS,
                'attr' => ['min' => 1, 'max' => 168, 'class' => 'settings-input-narrow'],
                'constraints' => [new Assert\Range(min: 1, max: 168, notInRangeMessage: 'Ce délai doit être compris entre {{ min }} et {{ max }} heures.')],
            ])
            ->add('reminder_event', CheckboxType::class, [
                'label' => 'admin_emails.reminder_event',
                'help' => 'admin_emails.reminder_event_help',
                'required' => false,
            ])
            ->add('reminder_event_lead_hours', IntegerType::class, [
                'label' => 'admin_emails.lead_hours',
                'help' => 'admin_emails.lead_hours_help',
                'empty_data' => (string) ReminderSettings::DEFAULT_EVENT_LEAD_HOURS,
                'attr' => ['min' => 1, 'max' => 168, 'class' => 'settings-input-narrow'],
                'constraints' => [new Assert\Range(min: 1, max: 168, notInRangeMessage: 'Ce délai doit être compris entre {{ min }} et {{ max }} heures.')],
            ])
            ->add('reminder_loan_due', CheckboxType::class, [
                'label' => 'admin_emails.reminder_loan_due',
                'required' => false,
            ])
            ->add('reminder_loan_lead_days', IntegerType::class, [
                'label' => 'admin_emails.lead_days',
                'help' => 'admin_emails.lead_days_help',
                'empty_data' => (string) ReminderSettings::DEFAULT_LOAN_LEAD_DAYS,
                'attr' => ['min' => 0, 'max' => 30, 'class' => 'settings-input-narrow'],
                'constraints' => [new Assert\Range(min: 0, max: 30, notInRangeMessage: 'Ce délai doit être compris entre {{ min }} et {{ max }} jours.')],
            ])
            ->add('reminder_loan_overdue', CheckboxType::class, [
                'label' => 'admin_emails.reminder_loan_overdue',
                'help' => 'admin_emails.reminder_loan_overdue_help',
                'required' => false,
            ])
            ->add('reminder_maintenance_overdue', CheckboxType::class, [
                'label' => 'admin_emails.reminder_maintenance',
                'help' => 'admin_emails.reminder_maintenance_help',
                'required' => false,
            ]);
    }

    public function configureOptions(OptionsResolver $resolver): void
    {
        $resolver->setDefaults([
            'data_class' => null,
            'csrf_token_id' => 'emails_reminders',
        ]);
    }
}
